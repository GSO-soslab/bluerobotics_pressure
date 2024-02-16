from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bluerobotics_pressure'
submodules = "bluerobotics_pressure/ms5837"

setup(
    name=package_name,
    version='0.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # install launch file
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lin Zhao',
    maintainer_email='linzhao@uri.edu',
    description='The bluerobotics_pressure package',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bluerobotics_pressure_node = bluerobotics_pressure.driver_node:main',
        ],
    },
)
