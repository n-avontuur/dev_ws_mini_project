import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Pose2D
from cv_bridge import CvBridge
import cv2
import depthai as dai
import numpy as np
import time

tresholdConfidence = 0.0
nnPath = "/home/n/dev_ws_mini_project/mobilenet-ssd_openvino_2021.2_6shave.blob"
labelMap = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
    "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

FRAME_SIZE = (640, 400)
RESIZE_MAPPING = (300, 300)

class ObjectDepthDetectionNode(Node):


    def __init__(self):
        super().__init__('object_depth_detection_node')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'camera_image', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth_image', 10)
        self.object_publisher = self.create_publisher(BoundingBox2D, 'object_detection', 10)

        self.device = self.init_depthai_device()

        self.fps = 0
        self.prev_frame_time = 0
        self.frame_count = 0
        self.new_frame_time = 0
        self.status_color = (255, 0, 0)

    def init_depthai_device(self):
        pipeline = dai.Pipeline()
        cam = pipeline.createColorCamera()
        cam.setPreviewSize(FRAME_SIZE[0], FRAME_SIZE[1])
        cam.setInterleaved(False)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

        mono_left = pipeline.createMonoCamera()
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)
        mono_right = pipeline.createMonoCamera()
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        stereo = pipeline.createStereoDepth()
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        cam.setBoardSocket(dai.CameraBoardSocket.RGB)

        face_spac_det_nn = pipeline.createMobileNetSpatialDetectionNetwork()
        face_spac_det_nn.setConfidenceThreshold(tresholdConfidence)
        face_spac_det_nn.setBlobPath(nnPath)
        face_spac_det_nn.setDepthLowerThreshold(100)
        face_spac_det_nn.setDepthUpperThreshold(5000)

        face_det_manip = pipeline.createImageManip()
        face_det_manip.initialConfig.setResize(RESIZE_MAPPING[0], RESIZE_MAPPING[1])
        face_det_manip.initialConfig.setKeepAspectRatio(False)

        cam.preview.link(face_det_manip.inputImage)
        face_det_manip.out.link(face_spac_det_nn.input)
        stereo.depth.link(face_spac_det_nn.inputDepth)

        x_preview_out = pipeline.createXLinkOut()
        x_preview_out.setStreamName("preview")
        cam.preview.link(x_preview_out.input)

        det_out = pipeline.createXLinkOut()
        det_out.setStreamName('det_out')
        face_spac_det_nn.out.link(det_out.input)

        disparity_out = pipeline.createXLinkOut()
        disparity_out.setStreamName("disparity")
        stereo.disparity.link(disparity_out.input)

        device = dai.Device(pipeline)
        return device

    def detect_objects_and_depth(self):
        bbox = BoundingBox2D()
        bbox.center = Pose2D()
        while rclpy.ok():
            q_rgb = self.device.getOutputQueue('preview', maxSize=1, blocking=False)
            q_depth = self.device.getOutputQueue('disparity', maxSize=1, blocking=False)
            q_object_detection = self.device.getOutputQueue('det_out', maxSize=1, blocking=False)

            rgb_frame = q_rgb.get().getCvFrame()
            depth_frame = q_depth.get().getCvFrame()
            object_detection = q_object_detection.tryGet()

            if object_detection:
                detections = object_detection.detections
                for detection in detections:
                    depth = self.calculate_depth(detection, depth_frame)

                    bbox.center.position.x = (detection.xmin + detection.xmax) / 2
                    bbox.center.position.y = (detection.ymin + detection.ymax) / 2
                    bbox.size_x = detection.xmax - detection.xmin
                    bbox.size_y = detection.ymax - detection.ymin

                    x1 = int(detection.xmin * FRAME_SIZE[0])
                    y1 = int(detection.ymin * FRAME_SIZE[1])
                    x2 = int(detection.xmax * FRAME_SIZE[0])
                    y2 = int(detection.ymax * FRAME_SIZE[1])

                    # Draw the bounding box
                    cv2.rectangle(rgb_frame, (x1, y1), (x2, y2), self.status_color, 2)

                    # Get the label for the detected object
                    class_index = detection.label
                    label = labelMap[class_index]

                    # Overlay the label near the bounding box
                    label_position = (x1, y1 - 10)  # Adjust the position as needed
                    cv2.putText(rgb_frame, label, label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, self.status_color)

                    self.display_info(rgb_frame, depth_frame, bbox, depth, 'Object Detected', self.status_color)

            self.frame_count += 1
            if self.frame_count >= 10:
                self.new_frame_time = time.time()
                fps = 1 / (self.new_frame_time - self.prev_frame_time)
                self.prev_frame_time = self.new_frame_time
                self.fps = fps

            self.object_publisher.publish(bbox)

            rgb_image_msg = self.bridge.cv2_to_imgmsg(rgb_frame, 'bgr8')
            self.image_publisher.publish(rgb_image_msg)

            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_frame, 'mono8')
            self.depth_publisher.publish(depth_image_msg)

            cv2.imshow("Face Cam", rgb_frame)
            cv2.imshow("Disparity Map", depth_frame)

            key = cv2.waitKey(1)
            if key == 27:
                break

    def calculate_depth(self, detection, depth_frame):
        xmin = max(0, detection.xmin)
        ymin = max(0, detection.ymin)
        xmax = min(detection.xmax, 1)
        ymax = min(detection.ymax, 1)

        x = int(xmin * FRAME_SIZE[0])
        y = int(ymin * FRAME_SIZE[1])
        w = int(xmax * FRAME_SIZE[0] - xmin * FRAME_SIZE[0])
        h = int(ymax * FRAME_SIZE[1] - ymin * FRAME_SIZE[1])

        bbox = (x, y, w, h)

        coord_x = detection.spatialCoordinates.x
        coord_y = detection.spatialCoordinates.y
        coord_z = detection.spatialCoordinates.z

        coordinates = (coord_x, coord_y, coord_z)
        return coordinates

    def display_info(self, frame, disp_frame, bbox, coordinates, label, status_color):
        x_center = int(bbox.center.position.x)
        y_center = int(bbox.center.position.y)
        size_x = bbox.size_x
        size_y = bbox.size_y

        x1 = int(x_center - size_x / 2)
        y1 = int(y_center - size_y / 2)
        x2 = int(x_center + size_x / 2)
        y2 = int(y_center + size_y / 2)

        cv2.rectangle(frame, (x1, y1), (x2, y2), status_color, 2)
        cv2.rectangle(frame, (5, 5, 175, 100), (50, 0, 0), -1)
        cv2.putText(frame, label, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, status_color)
        cv2.putText(frame, f'FPS: {self.fps:.2f}', (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255))

        if coordinates is not None:
            x, y, w, h = x1, y1, x2 - x1, y2 - y1
            cv2.rectangle(disp_frame, (x, y), (x + w, y + h), status_color, 2)
            cv2.rectangle(disp_frame, (5, 5, 185, 50), (50, 0, 0), -1)
            _, _, coord_z = coordinates
            cv2.putText(disp_frame, f'Depth: {coord_z}mm', (15, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255))

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDepthDetectionNode()

    try:
        node.detect_objects_and_depth()
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
