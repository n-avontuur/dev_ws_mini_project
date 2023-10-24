import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from my_robot_interfaces.msg import CustomObjectInfo
from cv_bridge import CvBridge
import cv2
import numpy as np 

LABEL_MAP = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
    "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

class TrackedObjectSubscriber(Node):
    def __init__(self):
        super().__init__('tracked_object_subscriber')
        self.image_subscription = self.create_subscription(
            Image, 'camera_image', self.image_callback, 10)
        self.tracked_objects_subscription = self.create_subscription(
            CustomObjectInfo,
            'tracked_objects_info',
            self.tracked_objects_callback,
            10
        )
        
        self.bridge = CvBridge()
        self.image = Image()
        self.oldImage = Image()
        self.tracked_objects = []

    def tracked_objects_callback(self, msg):
        print("object recieved")
        self.tracked_objects = msg
        

    def image_callback(self, msg):
        print("image recieved")
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.image is not None and not np.array_equal(self.oldImage, self.image):
            self.display_tracked_objects()
        else :
            print("same image  recieved")
        

    def display_tracked_objects(self):
        while rclpy.ok():
            if self.image is not None:
                frame = self.image.copy()
                if self.tracked_objects:
                    for t in self.tracked_objects:
                        x1, y1, x2, y2 = map(int, [t.x1, t.y1, t.x2, t.y2])

                        label = LABEL_MAP[t.label] if 0 <= t.label < len(LABEL_MAP) else str(t.label)

                        cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, f"ID: {t.id}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.putText(frame, t.depth_value, (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)
                print('imshow')
                cv2.imshow("tracked_objects", frame)
                key = cv2.waitKey(1)

                if key == ord('q'):  # Exit on 'q' key press
                    break

        # After the loop, release the OpenCV window
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = TrackedObjectSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
