from __future__ import annotations

from glob import glob

from setuptools import find_packages, setup

package_name = 'molly_bridge'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md', 'PROTOCOL.md']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='The handheld UDP gateway, its WebRTC video and the dashboard gateways',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_udp_node = molly_bridge.teleop_udp_node:main',
            'video_stream_node = molly_bridge.video_stream_node:main',
        ],
    },
)
