from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction


def generate_launch_description():
    ld = LaunchDescription()

    camera_trajectory = Node(
        package="my_robot",
        executable="inverse_kinematics",
        arguments=['1.3', '0.0', '0.5', 'o']
    )

    take_photo = Node(
        package="my_robot",
        executable="image_save"
    )

    delay_timer = TimerAction(
        period=5.0,
        actions=[take_photo]
    )

    ld.add_action(camera_trajectory)
    ld.add_action(delay_timer)

    return ld

