#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class vision_reciever_node(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.images = self.create_subscription(Image, "/cam", self.image_callback, 10)
        self.image_msg = Image()
        self.get_logger().info("vision node started")
        self.timer = self.create_timer(0.1, self.display_image)  # Adjust the period as needed
        self.bridge = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info("Image received")
        self.image_msg = msg

    def display_image(self):
        if self.image_msg is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
                # Your image processing or display code here
            except CvBridgeError as e:
                self.get_logger().error(f"Error converting Image message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = vision_reciever_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
