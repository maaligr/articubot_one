import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the world argument
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(get_package_share_directory('articubot_one'), 'worlds', 'outdoor_world.world','model.world'),
        description='Path to the world file to load'
    )

    # Include the robot_state_publisher launch file
    package_name = 'articubot_one'
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]),
        launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )



    # Gazebo parameters file
    gazebo_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'gazebo_params.yaml')

    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file}.items()
    )

    # Run the spawner node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'articubot_one'],
        output='screen'
    )

    # Joint state publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],  # Corrected parameter definition

        name='joint_state_publisher',

        output='screen'
    )

    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py'
        )]),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(get_package_share_directory('articubot_one'), 'config', 'mapper_params_online_async.yaml')
        }.items()
    )
    
   
    


    

    return LaunchDescription([
        declare_world_arg,
        rsp,
        gazebo,
        spawn_entity,
        joint_state_publisher,
        slam_toolbox,
    ])
