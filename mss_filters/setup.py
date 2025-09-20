from setuptools import find_packages, setup

package_name = 'mss_filters'

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
    description='Filtry sygnałów dla systemu MSS - prędkość, pozycja, kurs',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tractor_filter_node = mss_filters.tractor_filter_node:main',
            'chopper_filter_node = mss_filters.chopper_filter_node:main',
        ],
    },
)
