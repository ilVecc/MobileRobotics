import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    this_dir = os.path.dirname(os.path.abspath(__file__))

    # enable communication with Unity
    ros_tcp_endpoint = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_tcp_endpoint'), 'launch', 'endpoint.py')
        ),
    )

    # remappings for nav2 and slam_toolkit
    remap_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0', '0','0','0','1', 'summit_xl_base_footprint','base_footprint']
    )
    remap_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0', '0','0','0','1', 'summit_xl_base_link','base_link']
    )
    remap_front_laser_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0','0','0', '0','0','0','1', 'summit_xl_front_laser_link','base_scan']
    )

    model_arg = DeclareLaunchArgument(name='model', default_value='../../ICE-Lab-rbkairos/Assets/URDF/rbkairos.urdf',
                                      description='Path to robot urdf file')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    # slam toolbox
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': os.path.join(this_dir, 'kairos_params_slam.yaml'),
            'use_sim_time': 'true'
        }.items()
    )

    # navigation stack
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': os.path.join(this_dir, 'kairos_params_nav2.yaml'),
            'use_sim_time': 'true'
        }.items()
    )

    # visualize and command Nav2 via RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(this_dir, 'kairos_setup.rviz')],
        parameters=[{'use_sim_time':True}]
    )

    return LaunchDescription([
        ros_tcp_endpoint,
        remap_footprint,
        remap_base_link,
        remap_front_laser_link,
        model_arg,
        robot_state_publisher_node,
        slam,
        nav2,
        rviz_node
    ])
