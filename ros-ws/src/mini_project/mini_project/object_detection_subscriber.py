import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Pose2D
from cv_bridge import CvBridge
import cv2

class DisplayObjectsSubscriber(Node):
    def __init__(self):
        super().__init__('display_objects_subscriber')
        self.bridge = CvBridge()
        self.camera_image = None
        self.bbox_msg = None
        self.pose_msg = None

        self.image_subscription = self.create_subscription(
            Image, 'camera_image', self.image_callback, 10)
        self.bbox_subscription = self.create_subscription(
            BoundingBox2D, 'object_detection_bbox', self.bbox_callback, 10)
        self.pose_subscription = self.create_subscription(
            Pose2D, 'object_detection_pose', self.pose_callback, 10)

    def image_callback(self, msg):
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.display_objects()

    def bbox_callback(self, msg):
        self.bbox_msg = msg
        self.display_objects()

    def pose_callback(self, msg):
        self.pose_msg = msg
        self.display_objects()

    def display_objects(self):
        if self.camera_image is not None:
            display_image = self.camera_image.copy()

            if self.bbox_msg is not None:
                label = "Object"  # You can customize the label as needed
                x = self.bbox_msg.center.position.x
                y = self.bbox_msg.center.position.y
                width = self.bbox_msg.size_x
                height = self.bbox_msg.size_y

                cv2.putText(display_image, label, (int(x - width / 2), int(y - height / 2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.rectangle(display_image, (int(x - width / 2), int(y - height / 2), int(x + width / 2), int(y + height / 2)), (255, 0, 0), 2)

            if self.pose_msg is not None:
                x = self.pose_msg.position.x
                y = self.pose_msg.position.y
                theta = self.pose_msg.theta

                cv2.putText(display_image, f"X: {x:.2f}, Y: {y:.2f}, Theta: {theta:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow("Camera Image with Objects", display_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DisplayObjectsSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
