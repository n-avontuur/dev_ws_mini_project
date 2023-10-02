
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class movement_node(Node):
    def __init__(self):
        
        # Specifieceer de naam van de node.
        super().__init__("movement_node") 


        # Inform the operator that script is ready for use.
        self.get_logger().info("movement node for the bot started")
        



def main(args=None):
    rclpy.init(args=args)
    node = movement_node()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()