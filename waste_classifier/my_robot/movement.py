import rclpy
from my_robot.waste_client import WasteClient

def main(args=None):
    rclpy.init(args=args)
    client = WasteClient()
    future = client.send_image('/home/aderajew/ros2_ws/src/my_robot/image/camera_image.png')
    rclpy.spin_until_future_complete(client, future)  
    response_message = future.result()
    print('Classification result:', response_message.message)  
    rclpy.shutdown()

if __name__ == '__main__':
    main()

