from setuptools import find_packages, setup

package_name = 'mss_health_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='adam01wroblewski@gmail.com',
    description='System monitorowania zdrowia węzłów MSS z health reporting',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'health_monitor_node = mss_health_monitor.health_monitor_node:main',
        ],
    },
)
