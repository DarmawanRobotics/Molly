from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'molly_odometry'

setup(
    name=package_name,
    version='0.2.0',
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
    description='LIO odometry (FAST-LIO2 or Leg-KILO), stillness gate, monitor, route recorder',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wait_still = molly_odometry.wait_still:main',
            'odometry_monitor_node = molly_odometry.odometry_monitor_node:main',
            'route_recorder_node = molly_odometry.route_recorder_node:main',
        ],
    },
)
