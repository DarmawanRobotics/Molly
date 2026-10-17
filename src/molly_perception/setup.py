from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'molly_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='LiDAR obstacle sectors, AprilTag detection, person detection and visitors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_sector_node = molly_perception.lidar_sector_node:main',
            'image_throttle_node = molly_perception.image_throttle_node:main',
            'person_detector_node = molly_perception.person_detector_node:main',
            'visitor_tracker_node = molly_perception.visitor_tracker_node:main',
        ],
    },
)
