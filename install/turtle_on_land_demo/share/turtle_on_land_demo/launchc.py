#Original Designs

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

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=['/home/turtle_on_land/tutorial_ws/src']
    )

    #Create empty World
    world=ExecuteProcess(
            cmd=[
                'gz', 'sim', 'empty.sdf'
            ],
            output='screen'
        )

    #Lunch Gazebo with URDF file
    gazebo=ExecuteProcess(
            cmd=[
                'gz', 'service', '-s', '/world/empty/create',
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', 'sdf_filename: "/home/turtle_on_land/tutorial_ws/src/turtle_on_land_demo/urdf/URDF1.urdf", name: "URDF1"'
            ],
            output='screen'
        )
    

    return launch.LaunchDescription([
        gazebo_resource_path,world,gazebo
    ])