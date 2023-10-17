#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class vision_receiver_node(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.images = self.create_subscription(Image, "/cam", self.image_callback, 10)
        self.image_msg = Image()
        self.get_logger().info("vision node started")
        self.bridge = CvBridge()

        # Create a window for image display
        cv2.namedWindow("Image", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        self.get_logger().info("Image received")
        self.image_msg = msg

    def display_image(self):
        if self.image_msg is not None:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
                cv2.imshow("Image", cv_image)  # Display the image
                cv2.waitKey(1)  # Update the display (1 millisecond)
            except CvBridgeError as e:
                self.get_logger().error(f"Error converting Image message: {e}")

    def cleanup(self):
        # Close the OpenCV window when the node shuts down
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = vision_receiver_node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()  # Ensure the window is closed on node shutdown
        rclpy.shutdown()

if __name__ == '__main__':
    main()