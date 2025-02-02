import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint

class Trajectory_publisher(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint1','joint2','joint3','joint5','joint6']
        self.goal_positions = [0.00,-0.00,2.42,0.00,0.00]        

    def timer_callback(self):
        my_robot_trajectory_msg = JointTrajectory()
        my_robot_trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = self.goal_positions
        point.time_from_start = Duration(sec=2)
        my_robot_trajectory_msg.points.append(point)
        self.trajectory_publihser.publish(my_robot_trajectory_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_trajectory_object = Trajectory_publisher()

    rclpy.spin(joint_trajectory_object)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
