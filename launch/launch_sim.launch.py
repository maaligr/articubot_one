import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the world argument
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('articubot_one'), 'worlds', 'my_world.world'),
        description='Path to the world file to load'
    )

    # Include the robot_state_publisher launch file
    package_name = 'articubot_one'
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Run the spawner node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'articubot_one'],
        output='screen'
    )

    return LaunchDescription([
        declare_world_arg,
        rsp,
        gazebo,
        spawn_entity,
    ])
