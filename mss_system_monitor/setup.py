from setuptools import setup

package_name = 'mss_system_monitor'

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
    maintainer='pi',
    maintainer_email='adam01wroblewski@gmail.com',
    description='System monitor dla RPi - temperatura, CPU, RAM, dysk, GPIO, sieć',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_monitor_node = mss_system_monitor.system_monitor_node:main',
        ],
    },
)
