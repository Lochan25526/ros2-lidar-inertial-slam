from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    # Define paths
    rplidar_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_a2m7_launch.py'
    )

    description_launch = os.path.join(
        get_package_share_directory('my_robot_description'),
        'launch',
        'display.launch.py'
    )

    return LaunchDescription([

        # Include Robot State Publisher/Description Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch)
        ),

        # Include RPLIDAR Launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch)
        ),

        # IMU Publisher
        Node(
            package='icm20948_ros2',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen'
        ),

        # Madgwick Filter
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,
                'publish_tf': False,
                'world_frame': 'enu'
            }],
            remappings=[
                ('imu/data_raw', '/imu')
            ]
        ),
    ])