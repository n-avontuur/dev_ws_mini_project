import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import depthai

class vision_publisher_node(Node):
    def __init__(self):
        super().__init__("vision_sender_node")
        self.bridge = CvBridge()
        self.cam = self.create_publisher(Image, "/cam", 10)
        self.get_logger().info("Vision_sender_node started")
        self.device = self.init_depthai_device()  # Initialize DepthAI device

    def init_depthai_device(self):
        pipeline = depthai.Pipeline()

        cam_rgb = pipeline.create(depthai.node.ColorCamera)
        cam_rgb.setBoardSocket(depthai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setPreviewSize(300, 300)
        cam_rgb.setFps(15)

        xout_rgb = pipeline.create(depthai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        device = depthai.Device(pipeline)
        return device

    def capture_and_publish_image(self):
        q_rgb = self.device.getOutputQueue("rgb", 8, False)

        while rclpy.ok():
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
                self.publish_image(frame)
            rclpy.spin_once()

    def publish_image(self, frame):
        msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.cam.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = vision_publisher_node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()