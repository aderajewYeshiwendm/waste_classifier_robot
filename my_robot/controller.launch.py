from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():
    urdf = '/home/aderajew/ros2_ws/src/my_robot/urdf/my_robot.urdf'
    urdf_file = '/home/aderajew/ros2_ws/src/my_robot/urdf/coke_can.urdf'
    urdf_bin = '/home/aderajew/ros2_ws/src/my_robot/urdf/bin.urdf'
    urdf_org = '/home/aderajew/ros2_ws/src/my_robot/urdf/organic.urdf'

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
        
    with open(urdf_file, 'r') as infp:
        coke_tin = infp.read()
        
    with open(urdf_bin, 'r') as infp:
        bin_rec = infp.read()
        
    with open(urdf_org, 'r') as infp:
        bin_organic = infp.read()

    return LaunchDescription([
        # Start the robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]
        ),

        # Start Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=["-topic", "/robot_description", "-entity", "gripper_robot"]
        ),

        # Spawn objects in Gazebo
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "coke_can_model", "-b", "-file", urdf_file]
        ),
        
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "bin", "-b", "-file", urdf_bin]
        ),
        
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=["-entity", "organic", "-b", "-file", urdf_org]
        ),

        # Start controller manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[{'robot_description': robot_desc}]
        ),

        # Delay controller loading to ensure controller_manager is running
        TimerAction(
            period=5.0,  # Wait 5 seconds before loading controllers
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
                    output='screen'
                ),

                ExecuteProcess(
                    cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_trajectory_controller'],
                    output='screen'
                )
            ]
        ),
    ])