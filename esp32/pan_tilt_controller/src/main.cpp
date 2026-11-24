/*
 * vision_guided_control - ESP32 pan/tilt servo controller
 *
 * Listens on USB serial for "X,Y\n" pixel coordinates produced by the ROS 2
 * serial_bridge_node and drives two servos so the pan/tilt mount follows the
 * target.
 *
 * Build and upload with PlatformIO (VS Code or CLI):
 *   pio run
 *   pio run -t upload
 *
 * The ESP32Servo dependency is declared in platformio.ini, so no manual
 * library installation is required.
 *
 * The control law implemented in moveServos() mirrors the reference Python in
 * ros2_ws/src/vision_guided_control/vision_guided_control/servo_math.py so the
 * simulated behaviour matches the hardware.
 *
 * HARDWARE RULE (see docs/HARDWARE_WIRING.md):
 *   NEVER power the servos from an ESP32 GPIO pin or from the ESP32's own 3.3V
 *   regulator. Use a dedicated 5V supply and tie its ground to the ESP32 GND.
 */

#include <Arduino.h>
#include <ctype.h>
#include <ESP32Servo.h>

/* ------------------------------------------------------------------ pins */

// GPIO 25 and 26 are safe, general-purpose output pins on typical ESP32
// DevKit boards: they are not strapping pins, not used by the USB-UART bridge
// and not connected to the SPI flash. Change here and update
// docs/HARDWARE_WIRING.md if your board differs.
#define PAN_SERVO_PIN   25
#define TILT_SERVO_PIN  26

/* ------------------------------------------------------------- servo limits */

// Safe mechanical range in degrees. Start conservative to protect the
// bracket; measure physical limits before widening toward 0-180.
#define PAN_MIN   20
#define PAN_MAX   160
#define TILT_MIN  20
#define TILT_MAX  160

// Starting position and position assumed when the target is lost.
#define PAN_HOME  90
#define TILT_HOME 90

// Servo pulse widths in microseconds (standard 180-degree hobby servo).
#define SERVO_MIN_PULSE_US   500
#define SERVO_MAX_PULSE_US   2400

/* ------------------------------------------------------------ control tuning */

// Camera resolution the tracker publishes. Must match the tracker_node
// parameters (frame_width / frame_height), otherwise the mapping drifts.
#define FRAME_WIDTH   640
#define FRAME_HEIGHT  480

// Control mode:
//   false = Open-loop pointer (fixed camera watching the scene; absolute mapping)
//   true  = Closed-loop visual servoing (camera mounted on the pan/tilt head;
//           incremental steps toward optical center)
#define CONTROL_MODE_CLOSED_LOOP false
#define CLOSED_LOOP_STEP_DEG     4.0f

// Proportional gain: fraction of the servo half-range applied for a target at
// the opposite frame edge (1.0 = full travel), or step multiplier in closed-loop.
#define GAIN_X        1.0f
#define GAIN_Y        1.0f

// Flip if a servo moves the wrong way for a given target direction.
#define INVERT_PAN    false
#define INVERT_TILT   false

// Ignore target motion smaller than this many pixels to suppress servo jitter.
#define DEADZONE_PX   15

// Exponential moving average factor: 0 = frozen, 1 = no smoothing.
#define SMOOTHING     0.35f

// If no valid coordinate arrives for this long, return to the home position.
#define TARGET_LOST_TIMEOUT_MS   1500UL

// Never write the servos faster than this (50 Hz is plenty and stays smooth).
#define SERVO_UPDATE_INTERVAL_MS 20UL

// Echo the resulting angles back over serial for debugging/testing.
#define SEND_ACK      false

#ifndef SERIAL_BAUD
#define SERIAL_BAUD   115200
#endif

/* ------------------------------------------------------------------- state */

Servo panServo;
Servo tiltServo;

float panAngle  = PAN_HOME;
float tiltAngle = TILT_HOME;
bool targetVisible = false;
bool returningHome = false;

unsigned long lastCommandMs = 0;
unsigned long lastServoWriteMs = 0;

// Non-blocking line buffer for the incoming serial stream.
static const size_t LINE_BUF_SIZE = 32;
char lineBuffer[LINE_BUF_SIZE];
size_t lineLength = 0;
bool discardingLine = false;

/* ------------------------------------------------------- forward declares */

void readSerial();
void processLine(char *buffer);
void moveServos(float x, float y);
void writeServos();

/* ------------------------------------------------------------------ setup */

void setup() {
  Serial.begin(SERIAL_BAUD);

  // ESP32Servo must be told the allowed pulse range before attaching.
  panServo.setPeriodHertz(50);
  tiltServo.setPeriodHertz(50);
  panServo.attach(PAN_SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);
  tiltServo.attach(TILT_SERVO_PIN, SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US);

  // Move to the safe home position before accepting any command.
  panAngle = PAN_HOME;
  tiltAngle = TILT_HOME;
  writeServos();

  if (SEND_ACK) {
    Serial.println("pan_tilt_controller ready");
  }
}

/* ------------------------------------------------------------------ loop */

void loop() {
  readSerial();

  unsigned long now = millis();

  // Return home smoothly when the target has been silent for a while.
  if (targetVisible && (now - lastCommandMs) > TARGET_LOST_TIMEOUT_MS) {
    targetVisible = false;
    returningHome = true;
  }

  // Throttle servo writes: fast enough for smooth motion, gentle on the servo.
  if ((now - lastServoWriteMs) >= SERVO_UPDATE_INTERVAL_MS) {
    lastServoWriteMs = now;
    if (returningHome) {
      panAngle += SMOOTHING * (PAN_HOME - panAngle);
      tiltAngle += SMOOTHING * (TILT_HOME - tiltAngle);
      if (fabs(panAngle - PAN_HOME) < 0.5f && fabs(tiltAngle - TILT_HOME) < 0.5f) {
        panAngle = PAN_HOME;
        tiltAngle = TILT_HOME;
        returningHome = false;
      }
    }
    writeServos();
  }
}

/* ---------------------------------------------------------------- I/O */

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (discardingLine) {
        discardingLine = false;
      } else if (lineLength > 0) {
        lineBuffer[lineLength] = '\0';
        processLine(lineBuffer);
      }
      lineLength = 0;
    } else if (discardingLine) {
      // Discard remaining bytes of an oversized line until the next terminator.
    } else if (lineLength < LINE_BUF_SIZE - 1) {
      lineBuffer[lineLength++] = c;
    } else {
      // Line too long: it cannot be a valid packet, drop until newline and resync.
      lineLength = 0;
      discardingLine = true;
    }
  }
}

void processLine(char *buffer) {
  long x = 0;
  long y = 0;

  char *comma = strchr(buffer, ',');
  if (comma == NULL) {
    return;  // Malformed: no separator.
  }
  *comma = '\0';

  char *yField = comma + 1;
  // Require an ASCII digit first so leading whitespace and +/- signs are
  // rejected consistently with serial_protocol.py.
  if (!isdigit((unsigned char)buffer[0]) || !isdigit((unsigned char)yField[0])) {
    return;
  }

  char *endX = NULL;
  char *endY = NULL;
  x = strtol(buffer, &endX, 10);
  y = strtol(yField, &endY, 10);

  // Reject "320," / ",240" / "12,34,56" / "abc,def" outright.
  if (endX == buffer || *endX != '\0' || endY == yField || *endY != '\0') {
    return;
  }
  if (x < 0 || x > 65535 || y < 0 || y > 65535) {
    return;  // Out of range: never forward impossible coordinates to the servos.
  }

  moveServos((float)x, (float)y);
}

/* -------------------------------------------------------------- control */

void moveServos(float x, float y) {
  float halfWidth = FRAME_WIDTH / 2.0f;
  float halfHeight = FRAME_HEIGHT / 2.0f;

  float errorX = x - halfWidth;
  float errorY = y - halfHeight;

  float targetPan = CONTROL_MODE_CLOSED_LOOP ? panAngle : PAN_HOME;
  float targetTilt = CONTROL_MODE_CLOSED_LOOP ? tiltAngle : TILT_HOME;

  if (fabs(errorX) > DEADZONE_PX) {
    if (CONTROL_MODE_CLOSED_LOOP) {
      float deltaPan = GAIN_X * (errorX / halfWidth) * CLOSED_LOOP_STEP_DEG;
      if (INVERT_PAN) {
        deltaPan = -deltaPan;
      }
      targetPan = panAngle + deltaPan;
    } else {
      targetPan = PAN_HOME + GAIN_X * (errorX / halfWidth)
                  * ((PAN_MAX - PAN_MIN) / 2.0f);
      if (INVERT_PAN) {
        targetPan = PAN_HOME - (targetPan - PAN_HOME);
      }
    }
  }

  // Image y grows downward; tilting "up" for a target above the centre means
  // inverting the y term by default.
  if (fabs(errorY) > DEADZONE_PX) {
    if (CONTROL_MODE_CLOSED_LOOP) {
      float deltaTilt = -GAIN_Y * (errorY / halfHeight) * CLOSED_LOOP_STEP_DEG;
      if (INVERT_TILT) {
        deltaTilt = -deltaTilt;
      }
      targetTilt = tiltAngle + deltaTilt;
    } else {
      targetTilt = TILT_HOME - GAIN_Y * (errorY / halfHeight)
                   * ((TILT_MAX - TILT_MIN) / 2.0f);
      if (INVERT_TILT) {
        targetTilt = TILT_HOME - (targetTilt - TILT_HOME);
      }
    }
  }

  // Exponential moving average keeps motion smooth and non-aggressive.
  panAngle += SMOOTHING * (constrain(targetPan, PAN_MIN, PAN_MAX) - panAngle);
  tiltAngle += SMOOTHING * (constrain(targetTilt, TILT_MIN, TILT_MAX) - tiltAngle);

  // Final safety clamp before anything reaches a physical actuator.
  panAngle = constrain(panAngle, PAN_MIN, PAN_MAX);
  tiltAngle = constrain(tiltAngle, TILT_MIN, TILT_MAX);

  targetVisible = true;
  returningHome = false;
  lastCommandMs = millis();

  if (SEND_ACK) {
    Serial.print("pan=");
    Serial.print((int)panAngle);
    Serial.print(" tilt=");
    Serial.println((int)tiltAngle);
  }
}

void writeServos() {
  panServo.write((int)round(panAngle));
  tiltServo.write((int)round(tiltAngle));
}
