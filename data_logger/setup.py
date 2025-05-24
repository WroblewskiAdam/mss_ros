from setuptools import find_packages, setup

package_name = 'data_logger'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Twoje Imie',
    maintainer_email='twoj@email.com',
    description='Logowanie danych z IMU i GPS do CSV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logger_node = data_logger.logger_node:main',
        ],
    },
)