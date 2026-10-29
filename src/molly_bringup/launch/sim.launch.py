"""
The same stack on the simulated robot of dharma_sim (odometry external), no hardware.

    ros2 launch molly_bringup sim.launch.py rviz:=true
    ros2 launch molly_bringup sim.launch.py scenario:=person_crossing
    ros2 topic pub --once /sim/obstacles geometry_msgs/msg/PoseArray \
        "{poses: [{position: {x: 2.0, y: -3.0}}]}"

The site (default mall_loop under /tmp/molly_sim/sites) is drawn from the world when it has no
map yet, with the demo POIs and tour of config/demo_site, so a tour runs right away:
    ros2 service call /tour/control molly_interfaces/srv/TourControl \
        "{command: {kind: 0, tour_id: demo}}"
config/sim.yaml adapts the stack to the simulator (PointCloud2 input, no camera, silent speech).
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction

from molly_bringup import bringup, stack
from molly_bringup import launch_utils as lu

SIM_ARGUMENTS = [n for n in lu.ARGUMENTS if n not in ('robot_driver', 'odometry')]
SIM_DEFAULTS = {
    'site': 'mall_loop',
    'sites_dir': '/tmp/molly_sim/sites',
    'fiducials': 'false',
    'webrtc': 'false',  # no simulated camera to stream
    'web': 'false',
    'conversation': 'false',
}


def launch_setup(context):
    """Start the simulated robot, then the stack against it."""
    arg = lu.performer(context)
    options, notes = lu.resolve(arg, sim=True)
    demo = lu.share(lu.PACKAGE, 'config', 'demo_site')
    copied = bringup.seed_site(options.sites_dir, options.site, demo)
    if copied:
        notes.append(LogInfo(msg=f'demo POIs and tours copied into {options.site}: {copied}'))
    sim = stack.sim_include(options, arg('world'), arg('scenario'), bringup.truthy(arg('rviz')))
    return [*notes, *(lu.include_action(i) for i in [sim, *stack.compose(options)])]


def generate_launch_description():
    """Declare the arguments of the robot launch (no driver, odometry) and of the simulator."""
    return LaunchDescription(
        [
            *lu.declare(SIM_ARGUMENTS, SIM_DEFAULTS, env=False),
            DeclareLaunchArgument(
                'world', default_value='', description='world file or name; empty: mall_loop'
            ),
            DeclareLaunchArgument(
                'scenario', default_value='', description='scenario file or name; empty: none'
            ),
            DeclareLaunchArgument('rviz', default_value='false', description='RViz (dharma_rviz)'),
            OpaqueFunction(function=launch_setup),
        ]
    )
