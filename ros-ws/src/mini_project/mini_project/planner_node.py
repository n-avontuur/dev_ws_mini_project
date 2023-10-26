import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import CustomObjectInfo
from nav_msgs.msg import OccupancyGrid
import numpy as np
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped

class planner_node(Node):

    def __init__(self):
        super().__init__('planner_node')
    #     self.create_subscription(CustomObjectInfo, 'tracked_objects_info', self.object_callback, 10)
    #     self.map_pub = self.create_publisher(OccupancyGrid, 'object_map', 10)
    #     self.tf_buffer = tf2_ros.Buffer(self)
    #     self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    #     # Define map parameters
    #     self.map_resolution = 0.05  # Grid resolution (adjust as needed)
    #     self.map_width = 100  # Width of the map (adjust as needed)
    #     self.map_height = 100  # Height of the map (adjust as needed)
    #     self.map = np.zeros((self.map_width, self.map_height), dtype=np.int8)  # Initialize an empty map

    # def object_callback(self, msg):
    #     # Process the object information and update the map here
    #     # Calculate the position of the bottle and its grid cell in the map
    #     grid_x = int(msg.x1 / self.map_resolution)
    #     grid_y = int(msg.y1 / self.map_resolution)

    #     if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
    #         # Update the map cell to indicate the presence of a bottle
    #         self.map[grid_x, grid_y] = 100  # You can use different values to represent different states

    #     # Get the camera's origin in the robot's base frame
    #     try:
    #         transform = self.tf_buffer.lookup_transform('base_frame', 'camera_frame', msg.header.stamp)
    #         camera_origin_x = transform.transform.translation.x
    #         camera_origin_y = transform.transform.translation.y
    #         # You can also consider camera orientation for visualization

    #         # Convert the camera's position to map coordinates
    #         camera_map_x = int(camera_origin_x / self.map_resolution)
    #         camera_map_y = int(camera_origin_y / self.map_resolution)

    #         # Mark the camera's origin on the map
    #         if 0 <= camera_map_x < self.map_width and 0 <= camera_map_y < self.map_height:
    #             self.map[camera_map_x, camera_map_y] = 50  # Different value to represent the camera's origin
    #     except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    #         self.get_logger().warning("Failed to lookup camera transform")

    #     # Publish the updated map
    #     map_msg = OccupancyGrid()
    #     map_msg.header.stamp = self.get_clock().now().to_msg()
    #     map_msg.header.frame_id = 'map'
    #     map_msg.info.resolution = self.map_resolution
    #     map_msg.info.width = self.map_width
    #     map_msg.info.height = self.map_height
    #     map_msg.info.origin.position.x = 0.0
    #     map_msg.info.origin.position.y = 0.0
    #     map_msg.data = self.map.flatten().tolist()
    #     self.map_pub.publish(map_msg)

def main(args=None):
    rclpy.init(args=args)
    node = planner_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
