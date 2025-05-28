from setuptools import setup

package_name = 'servo_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Servo controller node using PCA9685 and ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_node = servo_controller.servo_node:main',
            'servo_teleop = servo_controller.servo_teleop_node:main',
            'servo_profiler = servo_controller.servo_profiler_node:main',
        ],
    },
)
