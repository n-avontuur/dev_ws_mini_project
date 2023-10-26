import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Pose2D
from my_robot_interfaces.msg import CustomObjectInfo
from cv_bridge import CvBridge
import cv2

LABEL_MAP = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
    "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

class DisplayObjectsSubscriber(Node):
    def __init__(self):
        super().__init__('display_objects_subscriber')
        self.bridge = CvBridge()
        self.camera_image = None
        self.tracked_object = None

        self.image_subscription = self.create_subscription(
            Image, 'camera_image', self.image_callback, 10)
        self.disparity_subscription = self.create_subscription(
            Image, 'disparity', self.disparity_callback, 10)
        self.tracking_subscription = self.create_subscription(
            CustomObjectInfo, 'tracked_objects_info', self.tracking_callback, 10)

    def image_callback(self, msg):
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.display_objects()

    def disparity_callback(self, msg):
        self.disparity_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.display_objects()

    def tracking_callback(self, msg):
        self.tracked_object = msg
        self.display_objects()

    def display_objects(self):
        if self.camera_image is not None:
            display_image = self.camera_image.copy()
            disparity_image = self.disparity_image.copy()

            if self.tracked_object is not None:
                if self.tracked_object.status != "REMOVED":
                    t = self.tracked_object
                    if t.label:
                        label = str(t.label)
                    if t.id :
                        id = str(t.id)
                        cv2.putText(display_image, f"ID: {id}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    if map(int, [t.x1, t.y1, t.x2, t.y2]):
                        x1, y1, x2, y2 = map(int, [t.x1, t.y1, t.x2, t.y2])
                        cv2.putText(display_image, label, (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.rectangle(display_image, (x1, y1), (x2, y2), (255, 0, 0), 2) 
                    if t.status :
                        status = str(t.status)
                        cv2.putText(display_image, status, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                    if t.depth_value :
                        depth_value = str(round(t.depth_value,2))
                        cv2.putText(display_image, depth_value, (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)

            cv2.imshow("Camera Image with Objects", display_image)
            #cv2.imshow("disparity_image", disparity_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DisplayObjectsSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()