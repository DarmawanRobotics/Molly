#!/usr/bin/env python3
"""
The whole Molly stack on the simulated quadruped of dharma_sim, no robot needed.

    ros2 launch molly sim.launch.py                    # localizes itself, site mall_loop
    ros2 launch molly sim.launch.py autostart:=true    # walks the taught route right away
    ros2 launch molly sim.launch.py rviz:=true         # with the RViz panels

The site (default mall_loop under /tmp/molly_sim/sites) is drawn from the world when it has no
map yet, with two POIs and a tour, so a tour runs right away:
    ros2 service call /tour/control molly_interfaces/srv/TourControl \
        "{command: {kind: 0, tour_id: demo}}"

Drop an obstacle on the route:
    ros2 topic pub --once /sim/obstacles geometry_msgs/msg/PoseArray \
        "{poses: [{position: {x: 2.0, y: -3.0}}]}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_setup(context):
    """Start the simulated robot (and its site) and the robot stack in simulation mode."""
    arg = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    share = get_package_share_directory('molly')
    sim = get_package_share_directory('dharma_sim')
    sites_dir = os.path.expanduser(arg('sites_dir'))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'sim.launch.py')),
            launch_arguments={
                'world': arg('world'),
                'robot_ns': arg('robot_ns'),
                'autostart': arg('autostart'),
                'rviz': arg('rviz'),
                'site': arg('site'),
                'sites_dir': sites_dir,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'robot.launch.py')),
            launch_arguments={
                'robot_ns': arg('robot_ns'),
                'mode': 'guide',
                'site': arg('site'),
                'sites_dir': sites_dir,
                'odometry': 'external',
                'route': arg('route'),
                'site_config': os.path.join(share, 'config', 'sim_site.yaml'),
                'robot_bringup': '',
                'visitors': 'true',
                'speech': 'true',          # engine none in sim_site.yaml: timed, silent
                'greeter': 'true',
                'conversation': 'false',
                'web': arg('web'),
                'health': 'true',
                'webrtc': 'false',  # no simulated camera to stream
            }.items(),
        ),
    ]


def generate_launch_description():
    """Declare the simulation arguments."""
    sim = get_package_share_directory('dharma_sim')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world', default_value=os.path.join(sim, 'config', 'worlds', 'mall_loop.yaml')
            ),
            DeclareLaunchArgument(
                'route', default_value=os.path.join(sim, 'config', 'routes', 'mall_loop.csv')
            ),
            DeclareLaunchArgument('robot_ns', default_value='robot'),
            DeclareLaunchArgument(
                'autostart', default_value='false', description='start the route when localized'
            ),
            DeclareLaunchArgument('rviz', default_value='false'),
            DeclareLaunchArgument(
                'site', default_value='mall_loop', description='site, drawn from the world if new'
            ),
            DeclareLaunchArgument('sites_dir', default_value='/tmp/molly_sim/sites'),
            DeclareLaunchArgument(
                'web', default_value='false', description='rosbridge + video for the dashboard'
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
