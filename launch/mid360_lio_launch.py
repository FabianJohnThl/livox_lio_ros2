import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
import launch

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_rviz_path = cur_path + '../rviz_cfg'
rviz_config_path = os.path.join(cur_rviz_path, 'mid360_lio_ros2.rviz')

def generate_launch_description():
    ScanRegistration = Node(
        package="lio_livox",
        executable="ScanRegistration",
        parameters=[[FindPackageShare("lio_livox"), "/config", "/mid360_lio.yaml"]],
        output="screen",
    )
    PoseEstimation = Node(
        package="lio_livox",
        executable="PoseEstimation",
        parameters=[[FindPackageShare("lio_livox"), "/config", "/mid360_lio.yaml"]],
        output="screen",
    )
    livox_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config', rviz_config_path]
    )

    launch = LaunchDescription()
    launch.add_action(ScanRegistration)
    launch.add_action(PoseEstimation)
    launch.add_action(livox_rviz)

    return launch
