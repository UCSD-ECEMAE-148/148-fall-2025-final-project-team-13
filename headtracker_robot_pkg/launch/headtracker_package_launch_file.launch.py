from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import PushRosNamespace
import os

def generate_launch_description():
    package_name = 'headtracker_robot_pkg'
    calibration_file = 'ros_racer_calibration.yaml'
    config = os.path.join(get_package_share_directory(package_name),'config','ros_racer_calibration.yaml')
    original_topic_name = '/cmd_vel'
    new_topic_name = LaunchConfiguration('topic_name',default=original_topic_name)
    ld = LaunchDescription()
    opentrack_node = Node(
            package=package_name,
            executable='opentrack_bridge',
            name='opentrack_bridge_node',
            output='screen')
    headtracker_node = Node(
            package=package_name,
            executable='headtracker_mapper',
            name='headtracker_mapper_node',
            output='screen')
    vesc_node = Node(
            package='ucsd_robocar_actuator2_pkg',
            executable='vesc_twist_node',
            name='vesc_twist_node',
            output='screen',
            remappings=[(original_topic_name,new_topic_name)],
            parameters=[config]) 
    ld.add_action(opentrack_node)
    ld.add_action(headtracker_node)
    ld.add_action(vesc_node)
    return ld
