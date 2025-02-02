import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from my_interfaces.srv import Classification
import cv2
import numpy as np
import tensorflow as tf

class WasteServer(Node):

    def __init__(self):
        super().__init__('waste_server')

        self.service_ = self.create_service(Classification, 'waste_classification', self.handle_classification_request)
        self.get_logger().info("Waste Classification Server has started")
        self.model = tf.keras.models.load_model('/home/aderajew/waste_vgg16.h5')


    def handle_classification_request(self, request, response):
    
        image_data = np.frombuffer(request.image_input.data, dtype=np.uint8)

        height = request.image_input.height
        width = request.image_input.width
        channels = 3  
        image_data = image_data.reshape((height, width, channels))

        size = (224, 224)
        resized_image = cv2.resize(image_data, size)
        
        expanded_image = np.expand_dims(resized_image, axis=0)

        pred = self.model.predict(expanded_image)

        if float(pred[0][0]) < 0.5:
            response.message = "Organic Waste"
        else:
            response.message = "Recyclable Waste"

        self.get_logger().info('Classification request processed')
        return response
        

def main(args=None):
    rclpy.init(args=args)
    server = WasteServer()
    rclpy.spin(server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

