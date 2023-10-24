import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from my_robot_interfaces.msg import CustomObjectInfo
from cv_bridge import CvBridge
import cv2

class ObjectNavigator(Node):
    def __init__(self):
        super().__init__('object_navigator')
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image, 'camera_image', self.image_callback, 10)
        self.object_info_subscription = self.create_subscription(
            CustomObjectInfo, 'tracked_objects_info', self.object_info_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.objects_detected = False
        self.object_positions = []

    def object_info_callback(self, msg):
        self.object_positions = msg.objects
        if len(self.object_positions) >= 2:
            self.objects_detected = True

    def image_callback(self, msg):
        try:
            if self.objects_detected:
                # Calculate the goal pose using the positions of the detected objects.
                # Implement logic to determine the goal pose based on object positions.

                goal_x = (self.object_positions[0].x + self.object_positions[1].x) / 2
                goal_y = (self.object_positions[0].y + self.object_positions[1].y) / 2

                cmd_msg = Twist()
                # Calculate linear and angular velocities to navigate to the goal pose.
                cmd_msg.linear.x = 0.1  # Adjust linear velocity as needed
                cmd_msg.angular.z = 0.1 * (goal_x - 0.5)  # Adjust angular velocity as needed

                self.cmd_pub.publish(cmd_msg)
            else:
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.0  # Stop the robot when no objects are detected.
                cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ObjectNavigator()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
