#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import depthai

class vision_sender_node(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.bridge = CvBridge()
        self.cam = self.create_publisher(Image, "/cam", 10)
        self.image_msg = Image()
        self.get_logger().info("vision node started")

    def cameraOutput(self):
        # Creating empty "pipeline" object
        pipeline = depthai.Pipeline()

        # Add camera to pipeline with definition of previewSize and InterLeaved
        cam_rgb = pipeline.create(depthai.node.ColorCamera)
        cam_rgb.setPreviewSize(300, 300)
        cam_rgb.setInterleaved(False)

        # Receive camera frame by XLink communication
        xout_rgb = pipeline.create(depthai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        # Create USB2 connection, not the default USB3 (Lower bandwidth, fewer issues)
        device = depthai.Device(pipeline, usb2Mode=True)

        # Define host side output queues to access the product results
        q_rgb = device.getOutputQueue("rgb")

        while True:  # Continuous image capture and publishing
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
                self.image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                # Set the timestamp of the Image message
                self.image_msg.header.stamp = self.get_clock().now().to_msg()
                self.cam.publish(self.image_msg)
                self.get_logger().info("Image sent") 


def main(args=None):
    rclpy.init(args=args)
    node = vision_sender_node()
    node.cameraOutput()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
