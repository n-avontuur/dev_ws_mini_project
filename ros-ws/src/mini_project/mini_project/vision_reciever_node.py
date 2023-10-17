import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class vision_receiver_node(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.images = self.create_subscription(Image, "/cam", self.image_callback, 10)
        self.get_logger().info("Vision_subscriber_node started")
        self.bridge = CvBridge()
        #self.image_msg = Image()
        

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.display_image(frame)
        except Exception as e:
            self.get_logger().error(f"Error processing Image message: {e}")

        #self.get_logger().info("Image received")
        #self.image_msg = msg
        #self.display_image()

    def display_image(self,frame):
        cv2.imshow("Image", frame)
        cv2.waitKey(1)

        #if self.image_msg is not None:
        #    try:
        #        cv_image = self.bridge.imgmsg_to_cv2(self.image_msg, desired_encoding="bgr8")
        #        cv2.imshow("Image", cv_image)  # Display the image
        #        cv2.waitKey(1)  # Update the display (1 millisecond)
        #    except CvBridgeError as e:
        #        self.get_logger().error(f"Error converting Image message: {e}")

    #def cleanup(self):
        # Close the OpenCV window when the node shuts down
        

def main(args=None):
    rclpy.init(args=args)
    cv2.startWindowThread() 
    node = vision_receiver_node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

