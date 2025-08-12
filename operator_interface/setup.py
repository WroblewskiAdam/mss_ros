from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'operator_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Kluczowa linia: instaluje wszystkie pliki z folderu 'web'
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adam Wroblewski',
    maintainer_email='adam01wroblewski@gmail.com',
    description='Interfejs webowy dla operatora.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Na razie nie tworzymy węzła backendowego,
            # ale zostawiamy miejsce na przyszłość.
        ],
    },
)
