from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'molly_navigation'

setup(
    name=package_name,
    version='0.5.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='Path finder, route and path following, obstacle avoidance',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'path_follower_node = molly_navigation.path_follower_node:main',
            'obstacle_avoidance_node = molly_navigation.obstacle_avoidance_node:main',
            'path_loader_node = molly_navigation.path_loader_node:main',
            'planner_node = molly_navigation.planner_node:main',
        ],
    },
)
