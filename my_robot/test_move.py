#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import ikpy.chain
import numpy as np
import time
from my_robot.waste_client import WasteClient


class TrajectoryPublisher(Node):

    def __init__(self):
        super().__init__('trajectory_publisher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publisher = self.create_publisher(JointTrajectory, publish_topic, 10)
        self.joints = ['joint1', 'joint2', 'joint3', 'joint5', 'joint6']

        urdf_file = '/home/aderajew/ros2_ws/src/my_robot/urdf/my_robot.urdf'

        # Toolbox interface
        self.robot_initialize(urdf_file)
        self.coordinates = [
            (1.7, 0.0, 0.2, 'o'),
            (1.7, 0.0, 0.2, 'c'),
            
            (1.7, 1.0, 0.5, 'c'),
            (1.0, 1.0, 1.0, 'o'),
            
            (1.7, 1.5, 1.5, 'c'),
            (1.7, 1.5, 1.5, 'o'),
        ]
        self.goal_positions = []

    def robot_initialize(self, urdf_file):
        self.kuka_robot = ikpy.chain.Chain.from_urdf_file(urdf_file)

    def inverse_kinematics_solution(self, x, y, z, claw):
        angles = self.kuka_robot.inverse_kinematics([x, y, z])
        angles = np.delete(angles, [0, 4, 5])

        if claw == "o":
            print("\nClaw Open\n")
            self.goal_positions = list(np.append(angles, [0.00, 0.00]))
        elif claw == "c":
            print("\nClaw Closed\n")
            self.goal_positions = list(np.append(angles, [-0.05, 0.05]))
        else:
            print("\nInvalid claw state. Expected 'o' (open) or 'c' (closed).\n")
            self.goal_positions = []

        print("\nInverse Kinematics Solution:\n", self.goal_positions)
    
    def execute_coordinates(self, coordinates):
        for x, y, z, claw in coordinates:
            self.inverse_kinematics_solution(x, y, z, claw)

            my_robot_trajectory_msg = JointTrajectory()
            my_robot_trajectory_msg.joint_names = self.joints
            point = JointTrajectoryPoint()
            point.positions = self.goal_positions
            point.time_from_start = Duration(sec=5)
            my_robot_trajectory_msg.points.append(point)
            self.trajectory_publisher.publish(my_robot_trajectory_msg)
            self.get_logger().info("Trajectory sent for coordinates: x={}, y={}, z={}, claw={}".format(x, y, z, claw))

            # Sleep for 5 seconds before executing the next coordinate
            time.sleep(5)

    def start_execution(self, response_message):
        # Execute the first three coordinates every time
        self.execute_coordinates(self.coordinates[:2])

        # Determine whether to execute the first two or the remaining two coordinates
        if (response_message.message == 'Organic'):
            self.execute_coordinates(self.coordinates[2:4])
        else:
            self.execute_coordinates(self.coordinates[4:])


def main(args=None):
    rclpy.init(args=args)
    
    client = WasteClient()
    future = client.send_image('/home/aderajew/ros2_ws/src/my_robot/image/camera_image.png')
    rclpy.spin_until_future_complete(client, future)  
    response_message = future.result()
    print('Classification result:', response_message.message)
    
    joint_trajectory_object = TrajectoryPublisher()
    joint_trajectory_object.get_logger().info("Trajectory publisher node started")

    joint_trajectory_object.start_execution(response_message)
    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

