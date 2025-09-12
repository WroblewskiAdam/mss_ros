from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'operator_interface'

def get_data_files():
    """Funkcja do rekurencyjnego kopiowania plików z katalogu web"""
    data_files = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ]
    
    # Rekurencyjnie znajdź wszystkie pliki w katalogu web
    web_files = []
    for root, dirs, files in os.walk('web'):
        for file in files:
            file_path = os.path.join(root, file)
            web_files.append(file_path)
    
    if web_files:
        data_files.append((os.path.join('share', package_name, 'web'), web_files))
    
    return data_files

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(),
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
