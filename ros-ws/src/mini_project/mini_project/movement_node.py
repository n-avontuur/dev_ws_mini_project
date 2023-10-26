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
        # self.image_subscription = self.create_subscription(
        #    Image, 'camera_image', self.image_callback, 10)
        self.object_info_subscription = self.create_subscription(
            CustomObjectInfo, 'tracked_objects_info', self.object_info_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.detected_objects = {}

    def object_info_callback(self, msg):
        if msg is not None:
            print("object info re")
            obj_id = int(msg.id)
            label = msg.label
            status = msg.status

            if status == "removed":
                # If an object with this ID exists, remove it from the dictionary.
                if obj_id in self.detected_objects:
                    del self.detected_objects[obj_id]
            else:
                # Store the object information in the dictionary using the ID as the key.
                if obj_id not in self.detected_objects:
                    self.detected_objects[obj_id] = {
                        "label": label,
                        "status": status,
                        "id": int(obj_id),
                        "x1": float(msg.x1),
                        "y1": float(msg.y1),
                        "x2": float(msg.x2),
                        "y2": float(msg.y2),
                        "depth_value": float(msg.depth_value)
                    }
        self.move_turtlebot()

    def image_callback(self, msg):
        self.move_turtlebot()

    def move_turtlebot(self):
        try:
            if self.detected_objects:
                # Initialize variables to keep track of the closest object and its depth.
                closest_obj = None
                min_depth = float('inf')

                for obj_id, obj_data in self.detected_objects.items():
                    depth = obj_data['depth_value']

                    # Check if the current object is closer than the previously found closest object.
                    if depth < min_depth:
                        min_depth = depth
                        closest_obj = obj_data

                if closest_obj is not None:
                    x1 = closest_obj['x1']
                    x2 = closest_obj['x2']

                    # Calculate the desired goal position based on the closest object's depth.
                    goal_depth = min_depth

                    # Check if the closest object is on the left or right side based on x1 and x2.
                    if (x1 + x2) / 2 < 0.5:
                        # Object is on the left side
                        goal_x = x1  # Use x1 as the goal_x
                    else:
                        # Object is on the right side
                        goal_x = x2  # Use x2 as the goal_x

                    # Calculate linear and angular velocities to navigate towards the closest object.
                    cmd_msg = Twist()
                    cmd_msg.linear.x = 0.1 * (goal_depth - 1.0)  # Adjust linear velocity based on depth
                    cmd_msg.angular.z = 0.1 * (goal_x - 0.5)  # Adjust angular velocity based on x position

                    self.cmd_pub.publish(cmd_msg)
                else:
                    # No suitable objects found, stop the robot.
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
