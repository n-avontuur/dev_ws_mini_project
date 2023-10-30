import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from my_robot_interfaces.msg import CustomObjectInfo
from cv_bridge import CvBridge
import time

class movement_node(Node):
    cmd_msg = Twist()
    def __init__(self):
        super().__init__('movement_node')
        self.bridge = CvBridge()
        #self.image_subscription = self.create_subscription(Image, 'camera_image', self.image_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.object_info_subscription = self.create_subscription( CustomObjectInfo, 'tracked_objects_info', self.object_info_callback, 10)
        self.detected_objects = {}

    def object_info_callback(self, msg):
        print("Object received")
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

        self.move_robot_cmd()

    def image_callback(self, msg):
        print("image received")
        self.move_robot_cmd()

    def move_robot_cmd(self):
        print("move robot")
        try:
            if self.detected_objects:
                # No objects detected, stop the robot.
                self.move_cmd(0,0)
                closest_obj = None
                min_distance = float('inf')
                for obj_id, obj_data in self.detected_objects.items():
                    depth = obj_data['depth_value']
                    print("depth: ", str(depth))
                    #print("obj_data", obj_data)
                    #print("detected_objects",self.detected_objects)
                    if len(obj_data) < 2:
                        # Check if the current object is closer than the previously found closest object.
                        if depth < min_distance:
                            min_distance = depth
                            closest_obj = obj_data
                            print('closed: ', closest_obj)

                        if closest_obj is None:
                            self.move_cmd(0,0)
                            print("stop robot no objects")
                            return

                        
                        sorted_objects = sorted(self.detected_objects.values(), key=lambda obj: obj.get("depth_value", float('inf')))
                        # Get the two closest objects
                        goal_x = self.goal_x(sorted_objects,depth)

                        # Define the input and output range
                        min_input = 0
                        max_input = 300
                        min_output = -0.5
                        max_output = 0.5

                        # Scale goal_x
                        rotation_speed = ((goal_x - min_input) / (max_input - min_input)) * (max_output - min_output) + min_output

                        if goal_x < 140 :
                            self.move_cmd(0.05,rotation_speed)
                            print("rotate +")
                        elif goal_x > 160:
                            self.move_cmd(0.05,rotation_speed)
                            print("rotate -")
                        else :
                            self.move_cmd(0.05,0.0)
                            print("straight")
                        return
                    if len(obj_data) >= 2:
                        self.move_cmd(0.05,0.0)
                        print("straight")
                else:
                    # No objects detected, stop the robot.
                    self.move_cmd(0,0)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def move_cmd(self, x, z):
        print("sending move cmd")
        self.cmd_msg.linear.x = float(x)
        self.cmd_msg.angular.z = float(z)
        self.cmd_pub.publish(self.cmd_msg)
   
    def goal_x(self,sorted_objects):
        closet_objects = sorted_objects[:2]
        total_x = 0
        for obj in closet_objects:
            x = (obj['x1'] + obj['x2']) / 2
            total_x += x

        average_x = total_x / 2           
        print(f'average_x: {average_x}')
        return average_x


def main(args=None):
    rclpy.init(args=args)
    node = movement_node()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
