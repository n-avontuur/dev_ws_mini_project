import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
import cv2

class TrackedObjectSubscriber(Node):
    def __init__(self):
        super().__init__('tracked_object_subscriber')
        self.tracked_objects_subscription = self.create_subscription(
            Float32MultiArray,
            'tracked_objects_info',
            self.tracked_objects_callback,
            10
        )
        self.image_subscription = self.create_subscription(
            Image,
            'camera_image',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.image = None
        self.tracked_objects = []

    def tracked_objects_callback(self, msg):
        self.tracked_objects = msg.data

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def display_tracked_objects(self):
        while rclpy.ok():
            if self.image is not None and self.tracked_objects:
                frame = self.image.copy()

                for i in range(0, len(self.tracked_objects), 7):
                    label = str(int(self.tracked_objects[i]))
                    id = int(self.tracked_objects[i + 1])
                    status = self.tracked_objects[i + 2]
                    x1 = int(self.tracked_objects[i + 3])
                    y1 = int(self.tracked_objects[i + 4])
                    x2 = int(self.tracked_objects[i + 5])
                    y2 = int(self.tracked_objects[i + 6])

                    cv2.putText(frame, label, (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255))
                    cv2.putText(frame, f"ID: {id}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255))
                    cv2.putText(frame, status, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255))
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), cv2.FONT_HERSHEY_SIMPLEX)

                cv2.imshow('Tracked Objects', frame)
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = TrackedObjectSubscriber()
    node.display_tracked_objects()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
