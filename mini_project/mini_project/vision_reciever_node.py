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
        

    def image_callback(self, msg):
        self.get_logger().info("Image received")
        if msg is not None:
            try:
                # Convert the Image message to a NumPy array
                cv_image = CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
                # Display the image using OpenCV
                cv2.imshow("preview", cv_image)
                cv2.waitKey(1)  # Wait for a short time to update the display (1 millisecond)
            except CvBridgeError as e:
                self.get_logger().error(f"Error converting Image message: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = vision_reciever_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
