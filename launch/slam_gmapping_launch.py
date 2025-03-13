from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions


def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    return LaunchDescription([
        launch_ros.actions.Node(
            package='slam_gmapping', executable='slam_gmapping', output='screen', parameters=[{'use_sim_time':use_sim_time, 
                                                                                                'angularUpdate': 0.1, 
                                                                                                'linearUpdate': 0.1, 
                                                                                                'lskip': 0, 
                                                                                                'xmax': 200, 
                                                                                                'xmin': -200, 
                                                                                                'ymax': 200, 
                                                                                                'ymin': -200,
                                                                                                'maxRange': 10,
                                                                                                'maxUrange': 5,
                                                                                                'throttle_scans': 1,
                                                                                                'kernelSize': 1,
                                                                                                'iterations': 1}]),
    ])
