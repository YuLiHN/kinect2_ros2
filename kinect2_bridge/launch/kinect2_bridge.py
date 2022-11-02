import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


parameters=[    {'base_name': 'kinect2'},
                {'sensor': ''},
                {'publish_tf': False},
                {'fps_limit': -1.0},
                {'use_png': False},
                {'jpeg_quality': 90},
                {'png_level': 1},
                {'depth_method': 'default'},
                {'depth_device': -1},
                {'reg_method': 'default'},
                {'reg_device': -1},
                {'max_depth': 12.0},
                {'min_depth': 0.1},
                {'queue_size': 5},
                {'bilateral_filter': True},
                {'edge_aware_filter': True},
                {'worker_threads': 4}]

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kinect2_bridge',
            executable='kinect2_bridge',
            output="screen",
            emulate_tty=True,
            name='my_kinect2_bridge',
            parameters=parameters,
        )
    ])


