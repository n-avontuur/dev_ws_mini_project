import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai
import cv2

class Vision_Sender_Node(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.bridge = CvBridge()
        self.cam = self.create_publisher(Image, "/cam", 10)
        self.image_msg = Image()
        self.get_logger().info("Vision node started")
        self.device = self.init_depthai_device(rgb_fps=15, resolution=(1920, 1080))  # Set the desired FPS for the RGB camera

    def init_depthai_device(self , rgb_fps, resolution):
        pipeline = depthai.Pipeline()

        # Configure the RGB camera with the desired FPS
        cam_rgb = pipeline.create(depthai.node.ColorCamera)
        cam_rgb.setPreviewSize(300,300)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(rgb_fps)
        cam_rgb.setBoardSocket(depthai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        
        xout_rgb = pipeline.create(depthai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        device = depthai.Device(pipeline, usb2Mode=True)
        return device

    def camera_output(self):
        q_rgb = self.device.getOutputQueue("rgb")


        while rclpy.ok():
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
                self.image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_msg.header.stamp = self.get_clock().now().to_msg()
                self.cam.publish(self.image_msg)
                self.get_logger().info("Image sent")


def main(args=None):
    rclpy.init(args=args)
    node = VisionSenderNode()
    try:
        node.camera_output()
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.device.close()  # Close the DepthAI device
        rclpy.shutdown()

if __name__ == '__main__':
    main()
