import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import CustomObjectInfo
from cv_bridge import CvBridge

class plan_move(Node):
    cmd_msg = Twist()

    def __init__(self):
        super().__init__('plan_move')
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.object_info_subscription = self.create_subscription(CustomObjectInfo, 'tracked_objects_info', self.object_info_callback, 10)
        self.detected_objects = {}

    def object_info_callback(self, msg):
        self.get_logger().info("Object received")
        if msg is not None:
            obj_id = msg.id
            label = msg.label
            status = msg.status

            if status in ["REMOVED", "LOST"]:
                # Remove the object with the specified ID from the dictionary if it exists.
                if obj_id in self.detected_objects:
                    self.get_logger().info(f"Object {obj_id} removed")
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

        self.calculate_movement()

    def calculate_movement(self):
        if self.detected_objects:
            closest_obj = self.get_closest_object()

            if closest_obj is None:
                self.send_move_cmd(0, 0)
                self.get_logger().info("Stop robot, no objects")
                return

            goal_x = self.calculate_goal_x(self.detected_objects.values())

            min_input, max_input, min_output, max_output = 0, 300, -0.5, 0.5
            rotation_speed = ((goal_x - min_input) / (max_input - min_input)) * (max_output - min_output) + min_output

            if goal_x < 140:
                self.send_move_cmd(0.05, rotation_speed)
                self.get_logger().info("Rotate +")
            elif goal_x > 160:
                self.send_move_cmd(0.05, rotation_speed)
                self.get_logger().info("Rotate -")
            else:
                self.send_move_cmd(0.05, 0.0)
                self.get_logger().info("Straight")

    def get_closest_object(self):
        closest_obj = None
        min_distance = float('inf')

        for obj_id, obj_data in self.detected_objects.items():
            depth = obj_data['depth_value']
            if depth < min_distance:
                min_distance = depth
                closest_obj = obj_data

        return closest_obj

    def calculate_goal_x(self, objects):
        closest_objects = sorted(objects, key=lambda obj: obj.get("depth_value", float('inf')))[:2]
        total_x = sum((obj['x1'] + obj['x2']) / 2 for obj in closest_objects)
        return total_x / 2

    def send_move_cmd(self, x, z):
        self.get_logger().info("Sending move cmd")
        self.cmd_msg.linear.x = float(x)
        self.cmd_msg.angular.z = float(z)
        self.cmd_pub.publish(self.cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    node = plan_move()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
