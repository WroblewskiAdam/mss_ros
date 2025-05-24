from setuptools import setup

package_name = 'imu_reader'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Wróblewski',
    maintainer_email='twoj@email.com',
    description='Publikowanie danych z BNO085 jako IMU w ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_node = imu_reader.imu_node:main',
        ],
    },
)
