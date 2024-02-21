# THIS FILE WILL NEED TO BE CHANGED If you change the name of the packages

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Wether or not to use Gazebo time'),
        # Change the template name on the line below if you change the package name
        Node(package='template_recognition', executable='yolov8_ros2_pt.py', output='screen'),
    ])