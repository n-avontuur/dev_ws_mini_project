import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from my_robot_interfaces.msg import CustomObjectInfo
from cv_bridge import CvBridge

class movement_node(Node):

    # Define the map_value function as a lambda function
    map_value = lambda value, in_min, in_max, out_min, out_max: (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def __init__(self):
        super().__init__('movement_node')
        self.bridge = CvBridge()
        self.image_subscription = self.create_subscription(
            Image, '/camera_image', self.image_callback, 10)
        self.object_info_subscription = self.create_subscription(
            CustomObjectInfo, '/tracked_objects_info', self.object_info_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.detected_objects = {}

    def object_info_callback(self, msg):
        print("object recieved")
        if msg is not None:
            obj_id = msg.id
            label = msg.label
            status = msg.status

            if status == "removed":
                # Remove the object with the specified ID from the dictionary if it exists.
                if obj_id in self.detected_objects:
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
                    "depth_value": msg.depth_value
                }
        self.move_robot_cmd()

    def image_callback(self, msg):
        print("image recieved")
        self.move_robot_cmd()

    def move_robot_cmd(self):
        print("move robot")
        try:
            if self.detected_objects:
                # Initialize variables to keep track of the closest object and its depth.
                closest_obj = None
                min_distance = float('inf')

                for obj_id, obj_data in self.detected_objects.items():
                    depth = obj_data['depth_value']

                    # Check if the current object is closer than the previously found closest object.
                    if depth < min_distance:
                        min_distance = depth
                        closest_obj = obj_data

                if closest_obj is not None:
                    x1 = closest_obj['x1']
                    x2 = closest_obj['x2']
                    depth = closest_obj['depth_value']

                    # Calculate the desired goal position based on the closest object's depth and position.
                    goal_x = (x1 + x2) / 2
                    goal_depth = depth

                    # Calculate linear and angular velocities to navigate towards the closest object.
                    cmd_msg = Twist()
                    cmd_msg.linear.x = self.map_value(goal_depth - 1.0, 0.0, 1.0, 0.0, 0.22)
                    cmd_msg.angular.z = self.map_value(goal_x - 0.5, -0.5, 0.5, 0.0, 2.84)
                    print("closes object")
                    self.cmd_pub.publish(cmd_msg)
                else:
                    # No suitable objects found, stop the robot.
                    cmd_msg = Twist()
                    cmd_msg.linear.x = 0.0
                    cmd_msg.angular.z = 0.0
                    print("stop robot no objects")
                    self.cmd_pub.publish(cmd_msg)
            else:
                # No objects detected, stop the robot.
                cmd_msg = Twist()
                cmd_msg.linear.x = 0.0
                cmd_msg.angular.z = 0.0
                self.cmd_pub.publish(cmd_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

        def remapValues(self,linear_velocity,angular_velocity):
            # Define the map_value function as a lambda function
            map_value = lambda value, in_min, in_max, out_min, out_max: (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


            # Define the input and output value ranges
            linear_velocity_min, linear_velocity_max = 0.0, 0.22
            angular_velocity_min, angular_velocity_max = 0.0, 2.84

            # Remap the linear and angular velocities
            remapped_linear_velocity = map_value(linear_velocity, 0.0, 1.0, linear_velocity_min, linear_velocity_max)
            remapped_angular_velocity = map_value(angular_velocity, 0.0, 1.0, angular_velocity_min, angular_velocity_max)
            return remapped_linear_velocity, remapped_angular_velocity


def main(args=None):
    rclpy.init(args=args)
    node = movement_node()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
