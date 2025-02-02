import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = '/home/aderajew/ros2_ws/src/my_robot/urdf/coke_can.urdf'
    urdf = '/home/aderajew/ros2_ws/src/my_robot/urdf/organic.urdf'

    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo", "-s", "libgazebo_ros_factory.so"],
                output="screen",
            ),

            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", "coke_can", "-b", "-file", urdf_file],
            ),
            
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=["-entity", "organic", "-b", "-file", urdf],
            ),
            
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[urdf],
           )
        ]
    )

