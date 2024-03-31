import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import cv2

class ThermalCameraSubscriber(Node):
    def __init__(self):
        super().__init__('thermal_camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/thermal_camera/image',
            self.image_callback,
            10
        )
        self.subscription

    def image_callback(self, msg):
        height = msg.height
        width = msg.width
        # Convert ROS Image message to numpy array
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, -1))
        # Convert BGR8 to RGB for display with OpenCV
        cv_image = cv2.cvtColor(data, cv2.COLOR_BGR2RGB)
        cv2.imshow('Thermal Camera', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    subscriber = ThermalCameraSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
