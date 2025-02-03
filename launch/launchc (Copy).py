#Original Design

import launch
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
import launch_ros
import os
import xacro
from launch.actions import ExecuteProcess


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)

    turtle_pkgg_share = launch_ros.substitutions.FindPackageShare(package='turtle_on_land_demo').find('turtle_on_land_demo')
    model_path = os.path.join(turtle_pkgg_share, 'urdf/URDF1.urdf')

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
    )
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
    )
    
    gazebo= ExecuteProcess (
        cmd = ['gz', 'sim', '/home/turtle_on_land/tutorial_ws/src/turtle_on_land_demo/urdf/URDF1.urdf'],
        output='screen'
    )
    
    return launch.LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH','/home/turtle_on_land/tutorial_ws/src'),
        gazebo
    ])