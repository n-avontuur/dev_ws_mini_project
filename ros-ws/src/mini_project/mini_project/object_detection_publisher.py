import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import depthai as dai
from my_robot_interfaces.msg import CustomObjectInfo
import time

# Define your constants
THRESHOLD_CONFIDENCE = 0.0
NN_PATH = "/home/n/dev_ws_mini_project/mobilenet-ssd_openvino_2021.2_6shave.blob"
LABEL_MAP = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
    "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]
10
class ObjectDepthDetectionNode(Node):

    def __init__(self):
        super().__init__('object_depth_detection_node')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'camera_image', 10)
        self.tracked_objects_publisher = self.create_publisher(CustomObjectInfo, 'tracked_objects_info', 10)
        self.device = self.init_depthai_device()

    def init_depthai_device(self):
        pipeline = dai.Pipeline()
        camRgb = pipeline.create(dai.node.ColorCamera)
        detectionNetwork = pipeline.create(dai.node.MobileNetDetectionNetwork)
        objectTracker = pipeline.create(dai.node.ObjectTracker)
        xlinkOut = pipeline.create(dai.node.XLinkOut)
        trackerOut = pipeline.create(dai.node.XLinkOut)

        xlinkOut.setStreamName("preview")
        trackerOut.setStreamName("tracklets")

        # Set camera properties
        camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(15)

        # Configure the detection network
        detectionNetwork.setBlobPath(NN_PATH)
        detectionNetwork.setConfidenceThreshold(THRESHOLD_CONFIDENCE)
        detectionNetwork.input.setBlocking(False)

        # Configure the object tracker
        objectTracker.setDetectionLabelsToTrack([15])  # Track only person
        objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

        camRgb.preview.link(detectionNetwork.input)
        objectTracker.passthroughTrackerFrame.link(xlinkOut.input)

        detectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)
        detectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
        detectionNetwork.out.link(objectTracker.inputDetections)
        objectTracker.out.link(trackerOut.input)

        device = dai.Device(pipeline)
        return device

    def detect_objects_and_depth(self):
        preview = self.device.getOutputQueue("preview", 4, False)
        tracklets = self.device.getOutputQueue("tracklets", 4, False)
        startTime = time.monotonic()
        counter = 0
        fps = 0
        frame = None

        while rclpy.ok():
            imgFrame = preview.get()
            track = tracklets.get()
            counter += 1
            current_time = time.monotonic()

            if (current_time - startTime) > 1:
                fps = counter / (current_time - startTime)
                counter = 0
                startTime = current_time

            color = (255, 0, 0)
            frame = imgFrame.getCvFrame()
            trackletsData = track.tracklets
            tracked_objects_data = []

            for t in trackletsData:
                roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
                x1 = int(roi.topLeft().x)
                y1 = int(roi.topLeft().y)
                x2 = int(roi.bottomRight().x)
                y2 = int(roi.bottomRight().y)

                try:
                    label = LABEL_MAP[t.label]
                except:
                    label = t.label

                cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, 255)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, cv2.FONT_HERSHEY_SIMPLEX)

                # Calculate depth using spatialCoordinates
                spatial_coord_x = t.spatialCoordinates.x
                spatial_coord_y = t.spatialCoordinates.y
                spatial_coord_z = t.spatialCoordinates.z

                # You may need to adjust these parameters based on your calibration
                baseline = 100  # Example baseline in millimeters
                focal_length = 477  # Example focal length in pixels

                # Calculate depth using the formula
                depth_value = (baseline * focal_length) / (spatial_coord_z + 1e-6)  # Adding a small value to prevent division by zero

                # Create an instance of CustomObjectInfo and populate its fields
                custom_info = CustomObjectInfo()
                custom_info.label = str(label)
                custom_info.id = t.id
                custom_info.x1 = float(x1)
                custom_info.y1 = float(y1)
                custom_info.x2 = float(x2)
                custom_info.y2 = float(y2)
                custom_info.depth_value = depth_value

                tracked_objects_data.append(custom_info)

                print("object tracked")

            # Publish the list of tracked objects
            # for custom_info in tracked_objects_data:
            self.tracked_objects_publisher.publish(tracked_objects_data)

            rgb_image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.image_publisher.publish(rgb_image_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDepthDetectionNode()
    node.detect_objects_and_depth()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
