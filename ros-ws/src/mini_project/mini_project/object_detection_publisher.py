import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import BoundingBox2D, Pose2D
from cv_bridge import CvBridge
import cv2
import depthai as dai
import numpy as np

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
        self.bbox_publisher = self.create_publisher(BoundingBox2D, 'object_detection_bbox', 10)
        self.pose_publisher = self.create_publisher(Pose2D, 'object_detection_pose', 10)

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
        countObjects = 0
        while rclpy.ok():
            q_rgb = self.device.getOutputQueue('preview', maxSize=1, blocking=False)
            q_depth = self.device.getOutputQueue('disparity', maxSize=1, blocking=False)
            q_object_detection = self.device.getOutputQueue('det_out', maxSize=1, blocking=False)

            rgb_frame = q_rgb.get().getCvFrame()
            depth_frame = q_depth.get().getCvFrame()
            object_detection = q_object_detection.tryGet()

            if object_detection:
                print("object detected")
                detections = object_detection.detections
                for detection in detections:
                    countObjects += 1
                    label = labelMap[detection.label]
                    depth = self.calculate_depth(detection, depth_frame)

                    bbox = BoundingBox2D()
                    bbox.center = Pose2D()
                    bbox.center.position.x = (detection.xmin + detection.xmax) / 2
                    bbox.center.position.y = (detection.ymin + detection.ymax) / 2
                    bbox.size_x = detection.xmax - detection.xmin
                    bbox.size_y = detection.ymax - detection.ymin

                    object_detection = Pose2D()
                    object_detection.position.x = bbox.center.position.x
                    object_detection.position.y = bbox.center.position.y
                    object_detection.theta = depth[2]

                    print("countObjects:"+str(countObjects))
                    self.bbox_publisher.publish(bbox)
                    self.pose_publisher.publish(object_detection)

            rgb_image_msg = self.bridge.cv2_to_imgmsg(rgb_frame, 'bgr8')
            self.image_publisher.publish(rgb_image_msg)

            depth_image_msg = self.bridge.cv2_to_imgmsg(depth_frame, 'mono8')
            self.depth_publisher.publish(depth_image_msg)

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
