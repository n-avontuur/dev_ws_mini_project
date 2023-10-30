import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import depthai as dai
import numpy as np

from my_robot_interfaces.msg import CustomObjectInfo


# Define your constants
THRESHOLD_CONFIDENCE = 0.0
NN_PATH = "/home/niels/dev_ws_mini_project/mobilenet-ssd_openvino_2021.2_6shave.blob"
LABEL_MAP = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
    "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

class object_detection(Node):

    def __init__(self):
        super().__init__('object_detection')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'camera_image', 10)
        self.tracked_objects_publisher = self.create_publisher(CustomObjectInfo, 'tracked_objects_info', 10)
        self.depth_publisher = self.create_publisher(Image, 'disparity', 10)
        self.device = self.init_depthai_device()

    def init_depthai_device(self):
        pipeline = dai.Pipeline()
        camRgb = pipeline.create(dai.node.ColorCamera)
        detectionNetwork = pipeline.create(dai.node.MobileNetDetectionNetwork)
        objectTracker = pipeline.create(dai.node.ObjectTracker)
        xlinkOut = pipeline.create(dai.node.XLinkOut)
        trackerOut = pipeline.create(dai.node.XLinkOut)
        disparity_out = pipeline.create(dai.node.XLinkOut)

        # Set camera properties
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(10)

        # Configure the detection network
        detectionNetwork.setBlobPath(NN_PATH)
        detectionNetwork.setConfidenceThreshold(THRESHOLD_CONFIDENCE)
        detectionNetwork.input.setBlocking(False)

        # Configure the object tracker
        objectTracker.setDetectionLabelsToTrack([5])  # Track only bottles
        objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

        # Left Mono Camera
        mono_left = pipeline.createMonoCamera()
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.LEFT)

        # Right Mono Camera
        mono_right = pipeline.createMonoCamera()
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.RIGHT)

        # Stereo Depth
        stereo = pipeline.createStereoDepth()
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        stereo.disparity.link(disparity_out.input)


        camRgb.preview.link(detectionNetwork.input)
        objectTracker.passthroughTrackerFrame.link(xlinkOut.input)

        detectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)
        detectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
        detectionNetwork.out.link(objectTracker.inputDetections)
        objectTracker.out.link(trackerOut.input)

        xlinkOut.setStreamName("preview")
        trackerOut.setStreamName("tracklets")
        disparity_out.setStreamName("disparity")

        device = dai.Device(pipeline)
        return device

    def detect_objects_and_depth(self):
        preview = self.device.getOutputQueue("preview", 4, False)
        tracklets = self.device.getOutputQueue("tracklets", 4, False)
        disparity_queue = self.device.getOutputQueue("disparity", 4, False)

        frame = None

        while rclpy.ok():
            imgFrame = preview.get()
            track = tracklets.get()
            disparity = disparity_queue.get()

            frame = imgFrame.getCvFrame()
            depth_frame = disparity.getCvFrame()

            trackletsData = track.tracklets
            tracked_objects_data = []

            for t in trackletsData:
                roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                x1 = int(roi.topLeft().x)
                y1 = int(roi.topLeft().y)
                x2 = int(roi.bottomRight().x)
                y2 = int(roi.bottomRight().y)
                label = LABEL_MAP[t.label]
                status = t.status.name

                # Calculate depth using the formula
                depth_value = self.get_object_depth(depth_frame, t)

                # Create an instance of CustomObjectInfo and populate its fields
                custom_info = CustomObjectInfo()
                custom_info.label = str(label)
                custom_info.id = t.id
                custom_info.x1 = float(x1)
                custom_info.y1 = float(y1)
                custom_info.x2 = float(x2)
                custom_info.y2 = float(y2)
                custom_info.status = status
                custom_info.depth_value = depth_value

                tracked_objects_data.append(custom_info)

                print("object tracked")

            # Publish the list of tracked objects
            for custom_info in tracked_objects_data:
                self.tracked_objects_publisher.publish(custom_info)
                
            self.depth_publisher.publish( self.bridge.cv2_to_imgmsg(depth_frame, 'mono8'))
            self.image_publisher.publish( self.bridge.cv2_to_imgmsg(frame, 'bgr8'))

            
    def get_object_depth(self, disparity_frame, t):
        roi = t.roi.denormalize(disparity_frame.shape[1], disparity_frame.shape[0])
        x1 = int(roi.topLeft().x)
        y1 = int(roi.topLeft().y)
        x2 = int(roi.bottomRight().x)
        y2 = int(roi.bottomRight().y)

        
        # Calculate the average disparity value within the ROI
        disparity_roi = disparity_frame[y1:y2, x1:x2]
        mean_disparity = np.mean(disparity_roi)
        mean_disparity = 1/mean_disparity

        # You may need to adjust these parameters based on your calibration
        baseline = 7.5  # Example baseline in millimeters
        focal_length = 477  # Example focal length in pixels
        depth = focal_length * baseline / (mean_disparity + 1e-6) /1000
        depth = round(depth,2)

        return depth

def main(args=None):
    rclpy.init(args=args)
    node = object_detection()
    rate = node.create_rate(2)
    node.detect_objects_and_depth()
    rate.sleep()  
    rclpy.shutdown()

if __name__ == '__main__':
    main()
