from setuptools import setup

package_name = 'speed_controller'

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
    maintainer='Twoje Imie', # Zmień na swoje
    maintainer_email='twoj@email.com', # Zmień na swoje
    description='Węzeł regulatora prędkości PI dla ciągnika',
    license='TODO: License declaration', # Uzupełnij
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speed_controller_node = speed_controller.speed_controller_node:main',
            'speed_teleop_node = speed_controller.speed_teleop_node:main',
        ],
    },
)