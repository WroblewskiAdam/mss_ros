from setuptools import find_packages, setup

package_name = 'gps_mockup'

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
    description='GPS Mockup package for simulating GPS data from tractor and chopper',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mockup_node = gps_mockup.mockup_node:main'
        ],
    },
)
