import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image 
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np

class ObjectReceiverNode(Node):
    def __init__(self):
        super().__init__("object_receiver_node")
        self.bridge = CvBridge()

        self.image_subscription = self.create_subscription(
            Image, "/camera_image", self.image_callback, 10)
        self.object_subscription = self.create_subscription(
            PoseStamped, "/object_detection_results", self.object_callback, 10)

        self.get_logger().info("Object Receiver Node started")
        self.current_image = None
        self.objects = []

    def image_callback(self, msg):
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def object_callback(self, msg):
        # Store object positions for later overlay
        self.objects.append((int(msg.pose.position.x), int(msg.pose.position.y)))

    def display_dashboard(self):
        if self.current_image is not None:
            # Overlay squares and labels on the image
            size = 20  # Size of the square (adjust as needed)
            color = (0, 0, 255)  # Red color
            thickness = 2  # Line thickness
            
            for x, y in self.objects:
                cv2.rectangle(self.current_image, (x - size, y - size), (x + size, y + size), color, thickness)
                label = f"Object at ({x}, {y})"
                cv2.putText(self.current_image, label, (x + size + 5, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, thickness)

            # Display the image with overlays
            cv2.imshow("Camera Image with Objects", self.current_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
