#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint
import ikpy.chain
import sys 
import numpy as np

import os 


class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10);timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint1','joint2','joint3','joint5','joint6']
        
        urdf_file= '/home/aderajew/ros2_ws/src/my_robot/urdf/my_robot.urdf'
        
        ## Toolbox interface
        self.robot_initialize(urdf_file)
        argv = sys.argv[1:] 
        self.inverse_kinematics_solution(float(argv[0]),float(argv[1]),float(argv[2]),argv[3])
    
    def timer_callback(self):
        my_robot_trajectory_msg = JointTrajectory()
        my_robot_trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        my_robot_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(my_robot_trajectory_msg)
        print("\nTrajectory Sent !\n")

    def robot_initialize(self,urdf_file):
        self.kuka_robot = ikpy.chain.Chain.from_urdf_file(urdf_file)
    
    def get_fk_solution(self):
        T=self.kuka_robot.forward_kinematics([0] * 9)
        print("\nTransformation Matrix :\n",T)
    
    def inverse_kinematics_solution(self,x,y,z,claw):
        angles=self.kuka_robot.inverse_kinematics([x,y,z])
        angles=np.delete(angles, [0,4,5])
        

        if (claw=="o"):
            print("\nClaw Open\n")
            self.goal_positions = list(np.append(angles ,[0.00,0.00]) )
        if (claw=="c"):
            print("\nClaw Closed\n")
            self.goal_positions = list(np.append(angles ,[-0.06, 0.06]) ) 
        print("\nInverse Kinematics Solution :\n" ,self.goal_positions)



def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()
    rclpy.spin(joint_trajectory_object)
    joint_trajectory_object.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
