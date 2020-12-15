from launch import LaunchDescription
from launch_ros.actions import Node

import os
import pathlib
import xacro

def generate_launch_description():
    file_path = pathlib.Path(__file__)
    # Here, parent first removes the file name
    parent_file_path = pathlib.Path(__file__).parent 
    
    # Xacro
    xacro_file = os.path.join(
        str(parent_file_path),
        'model.xacro'
    )

    doc = xacro.process(xacro_file)
    
    args = ('-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0 -spawn_service_timeout 30 -entity vehicle -topic robot_description').split()

    # There are currently no ways to pass the robot_description as a parameter 
    # to the urdf_spawner, see:
    # https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1039 
    # We should use the robot state publisher to publish the robot description
    # (or pass a file from the disk to the urdf spawner)
    robot_state_description = Node(
        namespace = 'vehicle',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time':False}, {'robot_description': doc}], # Use subst here
    )

    # Urdf spawner
    urdf_spawner = Node(
        name = 'urdf_spawner',
        namespace = 'vehicle',
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=args,
    )

    return (
        LaunchDescription([
            robot_state_description,
            urdf_spawner,
        ])
    )
