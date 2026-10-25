
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'aruco_detector'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marsorin',
    maintainer_email='user@example.com',
    description='Custom ROS 2 ArUco marker detector using the ZED 2i camera',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = aruco_detector.aruco_node:main',
            'aruco = aruco_detector.aruco:main'
        ],
    },
)
