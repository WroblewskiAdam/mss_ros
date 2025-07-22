# relative_position_computer/setup.py
from setuptools import find_packages, setup

package_name = 'relative_position_computer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Twoje Imie',
    maintainer_email='twoj@email.com',
    description='Obliczanie względnej pozycji między pojazdami.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'relative_computer_node = relative_position_computer.relative_computer_node:main',
        ],
    },
)