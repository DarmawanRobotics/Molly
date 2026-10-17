from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'molly_interaction'

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
    description='Speech, gestures, greeting and questions: Molly with visitors',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_node = molly_interaction.speech_node:main',
            'interaction_node = molly_interaction.interaction_node:main',
            'conversation_node = molly_interaction.conversation_node:main',
        ],
    },
)
