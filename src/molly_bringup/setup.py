from __future__ import annotations

from glob import glob

from setuptools import find_packages, setup

package_name = 'molly_bringup'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
        (f'share/{package_name}/config/demo_site', glob('config/demo_site/*.json')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='Molly on the Dharma stack: the Lite3 profile, overrides, robot and sim launch.',
    license='Apache-2.0',
)
