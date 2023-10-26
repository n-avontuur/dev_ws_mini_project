import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from my_robot_interfaces.msg import CustomObjectInfo
from cv_bridge import CvBridge

class movement_node(Node):
    def __init__(self):
        super().__init__('movement_node')
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image, 'camera_image', self.image_callback, 10)
        self.object_info_subscription = self.create_subscription(
            CustomObjectInfo, 'tracked_objects_info', self.object_info_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.detected_objects = {}

    def object_info_callback(self, msg):
        if msg is not None:
            obj_id = msg.id
            label = msg.label
            status = msg.status

            if status == "removed":
                # If an object with this ID exists, remove it from the list.
                if obj_id in self.detected_objects:
                    self.detected_objects[obj_id].clear()
            else:
                # Create a list for the object's information if it doesn't exist.
                if obj_id not in self.detected_objects:
                    self.detected_objects[obj_id] = []

                # Append the object information to the list.
                self.detected_objects[obj_id].append({
                    "label": label,
                    "status": status,
                    "id": obj_id,
                    "x1": msg.x1,
                    "y1": msg.y1,
                    "x2": msg.x2,
                    "y2": msg.y2,
                    "depth_value": msg.depth_value
                })





    def image_callback(self, msg):
        try:
            if self.detected_objects:
                # Calculate the desired goal position using depth and x information.
                # In this example, we'll navigate between the two closest objects.

                closest_obj1 = None
                closest_obj2 = None
                min_distance = float('inf')

                for obj_id, obj_data in self.detected_objects.items():
                    x = (obj_data['x1'] + obj_data['x2']) / 2
                    depth = obj_data['depth_value']

                    if abs(x - 0.5) < min_distance:
                        min_distance = abs(x - 0.5)
                        closest_obj2 = closest_obj1
                        closest_obj1 = obj_data

                if closest_obj1 is not None and closest_obj2 is not None:
                    x1 = (closest_obj1['x1'] + closest_obj1['x2']) / 2
                    x2 = (closest_obj2['x1'] + closest_obj2['x2']) / 2
                    depth1 = closest_obj1['depth_value']
                    depth2 = closest_obj2['depth_value']

                    # Calculate the desired goal position as the midpoint between the two objects.
                    goal_x = (x1 + x2) / 2
                    goal_depth = (depth1 + depth2) / 2

                    # Calculate linear and angular velocities to navigate between the rows.
                    cmd_msg = Twist()
                    cmd_msg.linear.x = 0.1 * (goal_depth - 1.0)  # Adjust linear velocity based on depth
                    cmd_msg.angular.z = 0.1 * (goal_x - 0.5)  # Adjust angular velocity based on x position

                    self.cmd_pub.publish(cmd_msg)
                else:
                    # No suitable objects found to navigate between, stop the robot.
                    cmd_msg = Twist()
                    cmd_msg.linear.x = 0.0
                    cmd_msg.angular.z = 0.0
                    self.cmd_pub.publish(cmd_msg)
            else:
                # No objects detected, stop the robot.
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = movement_node()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
