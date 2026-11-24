from setuptools import setup

package_name = 'vision_guided_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tracker_and_bridge.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alwaleed khaled',
    maintainer_email='eng.alwaleedtaleb@gmail.com',
    description=(
        'OpenCV colour tracking publishing /object_position and forwarding it '
        'to an ESP32 pan/tilt servo controller over serial.'
    ),
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_node = vision_guided_control.tracker_node:main',
            'serial_bridge_node = vision_guided_control.serial_bridge_node:main',
        ],
    },
)
