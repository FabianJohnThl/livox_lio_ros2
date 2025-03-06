from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        # Static transformers configuration:
        # lidar_init: initial lidar pose (from LIO)
        # livox_frame: current lidar pose (moves relative to lidar_init)
        # base_link, odom, map: used by pointcloud_to_laserscan
        # tree:
        # livox_frame -> odom
        # lidar_init -> base_link        
        # lidar_init -> map
        # map -> cloud
        Node( # Generate static transformer: map -> cloud
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'map', '--child-frame-id', 'cloud'
            ]
        ),
        Node( # Generate static transformer: livox_frame -> odom
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'livox_frame', '--child-frame-id', 'odom'
            ]
        ),
        Node( # Generate static transformer: lidar_init -> base_link
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'lidar_init', '--child-frame-id', 'base_link'
            ]
        ),
        Node( # Generate static transformer: lidar_init -> map
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'lidar_init', '--child-frame-id', 'map'
            ]
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('/cloud_in', '/lio_livox/full_cloud_mapped')], # remap topic of lasers' lio generated map to the /cloud_in topic (used as input from pointcloud_to_laserscan)
            parameters=[{
                'target_frame': 'cloud',
                'transform_tolerance': 0.01,
                'min_height': 0.20,
                'max_height': 0.60,
                'angle_min': -3.1415926535897, #-1.5708,  # -M_PI/2
                'angle_max': 3.1415926535897, #1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.2, # CHECK: this value needs to match the pub_frequency of your MID360 (configured in ../ros_driver_conf/launch/MID360_LIO_launch.py)
                'range_min': 0.1,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
  
