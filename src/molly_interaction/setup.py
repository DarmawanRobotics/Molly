from __future__ import annotations

from glob import glob

from setuptools import find_packages, setup

package_name = 'molly_interaction'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test', 'test.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml', 'README.md']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
        (f'share/{package_name}/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Agus Darmawan',
    maintainer_email='darmawandeveloper@gmail.com',
    description='Speech, gestures, the greeter, questions and the visitors: Molly with people',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'speech_node = molly_interaction.speech_node:main',
            'interaction_node = molly_interaction.interaction_node:main',
            'conversation_node = molly_interaction.conversation_node:main',
            'visitor_tracker_node = molly_interaction.visitor_tracker_node:main',
        ],
    },
)
