"""
The whole robot in one launch: the Dharma stack with this product's robot profile.

    ros2 launch molly_bringup robot.launch.py
    ros2 launch molly_bringup robot.launch.py robot_driver:=my_robot_driver site:=lobby
    ros2 launch molly_bringup robot.launch.py site:=lobby start_mapping:=true

Every package launch gets robot_params:=config/robot.yaml and one parameter file merged from
config/stack.yaml and params_file (later wins), with the started nodes added to the health
monitor's nodes.expected. The composition itself is in molly_bringup/stack.py.
"""

from __future__ import annotations

from launch import LaunchDescription
from launch.actions import OpaqueFunction

from molly_bringup import launch_utils as lu
from molly_bringup import stack


def launch_setup(context):
    """Validate the arguments and build the robot driver and the stack."""
    arg = lu.performer(context)
    options, notes = lu.resolve(arg)
    driver = lu.driver_action(arg('robot_driver'), options)
    actions = [*notes] + ([driver] if driver is not None else [])
    return actions + [lu.include_action(i) for i in stack.compose(options)]


def generate_launch_description():
    """Declare the arguments (robot_ns, robot_driver, odometry, site and sites_dir: env too)."""
    return LaunchDescription([*lu.declare(lu.ARGUMENTS), OpaqueFunction(function=launch_setup)])
