import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.move_forward()

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.2  # Adjust the linear velocity as needed
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    control_node = ControlNode()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

