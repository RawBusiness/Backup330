#Essential Libraries
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_file = 'URDF1.sdf'
    world_sdf_file = 'world.sdf'
    rviz_config = 'turtle.rviz'

    # Specify the full path to the URDF file
    path_to_urdf = os.path.join(
        get_package_share_directory('turtle_on_land_demo'),
        'urdf',
        urdf_file)
        
    path_to_world_sdf = os.path.join(
        get_package_share_directory('turtle_on_land_demo'),
        'urdf',
        world_sdf_file)
        
    path_to_rviz = os.path.join(
        get_package_share_directory('turtle_on_land_demo'),
        'rviz',
        rviz_config)
        
    gz_args = f"-r -v 4 {path_to_world_sdf}"

    # Set environment variable for GZ_SIM_RESOURCE_PATH
    resource_path = get_package_share_directory('turtle_on_land_demo')
    set_gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(Command(['xacro ', str(path_to_urdf)]), value_type=str)
        }]
    )

    # Create a joint_state_publisher_gui node to visualize and control joint states

    # Launch Gazebo simulator with an empty world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py",
            )
        ),
        launch_arguments={"gz_args": gz_args}.items(),
    )

    # Spawn the robot in Gazebo using the EntityFactory service equivalent
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",  # 'create' executable acts like the service to spawn a model in Gazebo
        arguments=[
            "-name", "robot1",  # Name of the robot
            "-file", path_to_urdf,  # Use the absolute path to your URDF file
            "-x", "0", "-y", "0", "-z", "1.4"  # Initial position of the robot in the world
        ],
        output="screen",
    )

    # Add the ros_gz_bridge node to bridge the joint states and commands
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        arguments=[
            # Bridge joint states from ROS 2 to Gazebo transport
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            
            # Bridge velocity commands if controlling with velocity
            '/cmd_vel@geometry_msgs/msg/Twist[gz.msgs.Twist',
            
            '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',

            '/boxes_image@sensor_msgs/msg/Image[gz.msgs.Image',
            
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        output='screen'
    )
    

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', path_to_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    

    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'),

        # Set the environment variable for GZ_SIM_RESOURCE_PATH
        set_gz_sim_resource_path,

        # Start joint state publisher GUI, robot state publisher, Gazebo simulator, and spawn the robot
        node_robot_state_publisher,
        gz_sim,
        spawn_entity,
        bridge,  # Add the ros_gz_bridge node
        rviz_node
    ])

