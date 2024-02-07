from setuptools import find_packages, setup

package_name = 'bluerobotics_pressure'

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
    maintainer='Lin Zhao',
    maintainer_email='linzhao@uri.edu',
    description='The bluerobotics_pressure package',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tester = bluerobotics_pressure.test_node:main',
        ],
    },
)
