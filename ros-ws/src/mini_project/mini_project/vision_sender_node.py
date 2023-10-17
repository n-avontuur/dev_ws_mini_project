import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import depthai
import cv2
from rclpy.time import Time

class Vision_Sender_Node(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.bridge = CvBridge()
        self.cam = self.create_publisher(Image, "/cam", 10)
        self.device = self.init_depthai_device(rgb_fps=15, resolution=(1920, 1080))  # Set the desired FPS for the RGB camera
        self.image_msg = Image()
        # self.viewer_window_name = "Image Viewer"
        # cv2.namedWindow(self.viewer_window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info("Vision_publisher_node started")

    def init_depthai_device(self, rgb_fps, resolution):
        pipeline = depthai.Pipeline()

        # Configure the RGB camera with the desired FPS
        cam_rgb = pipeline.create(depthai.node.ColorCamera)
        cam_rgb.setPreviewSize(300, 300)
        cam_rgb.setInterleaved(False)
        cam_rgb.setFps(rgb_fps)
        cam_rgb.setBoardSocket(depthai.CameraBoardSocket.RGB)
        cam_rgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)

        xout_rgb = pipeline.create(depthai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        device = depthai.Device(pipeline, usb2Mode=True)
        return device

    def camera_output(self):
        previous_time = self.get_clock().now()
        rate = rclpy.Rate(15)  # Set the desired publishing rate (e.g., 15 Hz)

        q_rgb = self.device.getOutputQueue("rgb")

        while rclpy.ok():
            in_rgb = q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()
                self.image_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_msg.header.stamp = self.get_clock().now().to_msg()
                self.cam.publish(self.image_msg)  # Publish the image

                # Display the image in the viewer window
                #cv2.imshow(self.viewer_window_name, frame)
                #cv2.waitKey(1)  # Update the display (1 millisecond)

            current_time = self.get_clock().now()
            time_interval = current_time - previous_time
            desired_time_interval = Time.from_sec(1.0 / 15)  # Desired rate of 15 Hz

            if time_interval < desired_time_interval:
                remaining_time = desired_time_interval - time_interval
                rate.sleep(remaining_time)

            previous_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = Vision_Sender_Node()
    try:
        node.camera_output()
    except Exception as e:
        node.get_logger().error(f"An error occurred: {e}")
    finally:
        node.device.close()  # Close the DepthAI device
        rclpy.shutdown()

if __name__ == '__main__':
    main()
