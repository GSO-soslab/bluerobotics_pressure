import os
import yaml
import pathlib
from launch import LaunchDescription
import launch.actions
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    ld = LaunchDescription()

    param_config = os.path.join(
        get_package_share_directory('bluerobotics_pressure'),
        'config',
        'bar30.yaml'
    )

    node = Node(
        package='bluerobotics_pressure',
        executable='bluerobotics_pressure_node',
        name='bluerobotics_pressure_node',
        namespace="alpha",
        output='screen',
        parameters=[param_config]        
    )

    ld.add_action(node)

    return ld