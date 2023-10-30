import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Pose2D
from my_robot_interfaces.msg import CustomObjectInfo
from cv_bridge import CvBridge
import cv2


class display(Node):
    
    def __init__(self):
        super().__init__('display')
        self.bridge = CvBridge()
        self.camera_image = None
        self.tracked_object = None
        self.detected_objects = {}

        self.image_subscription = self.create_subscription( Image, 'camera_image', self.image_callback, 10)
        self.disparity_subscription = self.create_subscription( Image, 'disparity', self.disparity_callback, 10)
        self.tracking_subscription = self.create_subscription( CustomObjectInfo, 'tracked_objects_info', self.tracking_callback, 10)

    def image_callback(self, msg):
        self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.display_objects()

    def disparity_callback(self, msg):
        self.disparity_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")
        self.display_objects()

    def tracking_callback(self, msg):
        self.tracked_object = msg
        self.collect_objects(msg)
        self.display_objects()

    def display_objects(self):
        if self.camera_image is not None:
            display_image = self.camera_image.copy()
            
            for obj_id, obj_data in self.detected_objects.items():
                label = obj_data.get("label", "")
                x1, y1, x2, y2 = obj_data.get("x1", 0), obj_data.get("y1", 0), obj_data.get("x2", 0), obj_data.get("y2", 0)
                
                if all([label, x1, y1, x2, y2]):
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    cv2.putText(display_image, label, (int(x1) + 10, int(y1) + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 255, 255))
                    cv2.rectangle(display_image, (int(x1), y1), (x2, y2), (255, 0, 0), 2)

                id = str(obj_data.get("id", ""))
                if id:
                    cv2.putText(display_image, f"ID: {id}", (int(x1) + 10, int(y1) + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 255, 255))

                status = obj_data.get("status", "")
                if status:
                    cv2.putText(display_image, status, (int(x1) + 10, int(y1) + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 255, 255))

                depth_value = obj_data.get("depth_value", 0.0)
                if depth_value:
                    depth_value = round(depth_value, 2)
                    cv2.putText(display_image, str(depth_value), (int(x1) + 10, int(y1) + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 255, 255))

            cv2.imshow("Camera Image with Objects", display_image)
            cv2.waitKey(1)


    def collect_objects(self,msg):
        if msg is not None:
            obj_id = msg.id
            label = msg.label
            status = msg.status

            if status == ("REMOVED" or "LOST"):
                # Remove the object with the specified ID from the dictionary if it exists.
                if obj_id in self.detected_objects:
                    print(f"Object {obj_id} removed")
                    del self.detected_objects[obj_id]
            else:
                # Store the object information in the dictionary.
                self.detected_objects[obj_id] = {
                    "label": label,
                    "status": status,
                    "id": obj_id,
                    "x1": msg.x1,
                    "y1": msg.y1,
                    "x2": msg.x2,
                    "y2": msg.y2,
                    "depth_value": msg.depth_value,
                    "start_remove_time": 0
                }


def main(args=None):
    rclpy.init(args=args)
    node = display()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()