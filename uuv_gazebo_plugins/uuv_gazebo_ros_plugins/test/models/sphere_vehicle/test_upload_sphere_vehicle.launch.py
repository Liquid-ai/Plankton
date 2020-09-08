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

    output = os.path.join(
        str(parent_file_path),
        'robot_description.urdf'
    )

    doc = xacro.process(xacro_file)
    with open(output, 'w') as file_out:
        file_out.write(doc)
    
    args = ('-x 0 -y 0 -z 0 -R 0 -P 0 -Y 0 -entity vehicle -file ' + output).split()
    #args='-gazebo_namespace /gazebo -x 0 -y 0 -z 0 -R 0 -P 0 -Y 0 -entity $(var namespace) -file ' + output

    # Urdf spawner
    urdf_spawner = Node(
        node_name = 'urdf_spawner_sphere',
        node_namespace = 'vehicle',
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        # To replace in foxy with parameters=[{'robot_description', Command('ros2 run xacro...')}]
        arguments=args
    )

    return (
        LaunchDescription([
            urdf_spawner,
        ])
    )
    