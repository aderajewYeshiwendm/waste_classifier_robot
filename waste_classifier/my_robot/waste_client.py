import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from my_interfaces.srv import Classification
import cv2
import numpy as np

class WasteClient(Node):

    def __init__(self):
        super().__init__('waste_client')

        self.client_ = self.create_client(Classification, 'waste_classification')

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for the waste_classification service...')

    def send_image(self, image_path):
        img = cv2.imread(image_path)

        image_msg = Image()
        image_msg.height = img.shape[0]
        image_msg.width = img.shape[1]
        image_msg.encoding = "bgr8"
        image_msg.is_bigendian = False
        image_msg.step = 3 * img.shape[1]  

        image_msg.data = np.array(img).tobytes()

        request = Classification.Request()
        request.image_input = image_msg

        future = self.client_.call_async(request)
        future.add_done_callback(self.callback_call_add_two_ints)
        return future

    def callback_call_add_two_ints(self, future):
        try:
            response = future.result()
            message = response.message  
            print('Classification result:', message)
        except Exception as e:
            self.get_logger().error('Service call failed: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    client = WasteClient()
    future = client.send_image('/home/aderajew/ros2_ws/src/b/image/camera_image.png')
    rclpy.spin_until_future_complete(client, future)  
    response_message = future.result()
    print('Classification result:', response_message.message)  
    rclpy.shutdown()

if __name__ == '__main__':
    main()

