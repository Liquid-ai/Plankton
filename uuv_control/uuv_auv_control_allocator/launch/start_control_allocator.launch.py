import os
import sys

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch_ros.actions import Node, PushRosNamespace
from launch.actions.opaque_function import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LC
from launch.actions import GroupAction

import launch
import launch.actions

from ament_index_python.packages import get_package_share_directory

bool_aliases = [1, '1', "True", "true"]

def launch_setup(context, *args, **kwargs):
    tam_reset_arg = {"output_dir": LC("output_dir").perform(context)} if (LC("reset_tam").perform(context) in bool_aliases) else LC("tam_config").perform(context)
    fin_manager_node = Node(
        name="auv_control_allocator",
        package="uuv_auv_control_allocator",
        executable="control_allocator",
        output="screen",
        parameters=[LC("config_file").perform(context)],
        remappings=[("tf", "/tf")])

    group = GroupAction([
        PushRosNamespace(LC("namespace").perform(context)), # replace with vehicle namespace
        fin_manager_node
        ])

    return [group]

def generate_launch_description():
    
    ld = launch.LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="ROBOT_NAME"), # Add your robot name here
        DeclareLaunchArgument("reset_tam", default_value="True"), # TODO: Fix TAM Matrix Storage and Loading, need TAM?
        DeclareLaunchArgument("config_file", default_value=os.path.join(get_package_share_directory("uuv_auv_control_allocator"), "config", "actuator_manager.yaml")),
        #DeclareLaunchArgument("tam_config", default_value=os.path.join(get_package_share_directory("uuv_auv_control_allocator"), "config", "TAM.yaml")),
        DeclareLaunchArgument("output_dir", default_value=os.path.join(get_package_share_directory("uuv_auv_control_allocator"), "config")),
        OpaqueFunction(function=launch_setup)
    ])

    return ld


if __name__ == '__main__':
    generate_launch_description()
