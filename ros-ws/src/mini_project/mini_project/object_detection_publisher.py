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
fullFrameTracking = False

class ObjectDepthDetectionNode(Node):
    def __init__(self):
        super().__init__('object_depth_detection_node')
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'camera_image', 10)
        self.depth_publisher = self.create_publisher(Image, 'depth_image', 10)
        self.object_publisher = self.create_publisher(BoundingBox2D, 'object_detection', 10)

        self.device = self.init_depthai_device()

    def init_depthai_device(self):
        pipeline = dai.Pipeline()
        # Define sources and outputs
        camRgb = pipeline.create(dai.node.ColorCamera)
        detectionNetwork = pipeline.create(dai.node.MobileNetDetectionNetwork)
        objectTracker = pipeline.create(dai.node.ObjectTracker)
        manip = pipeline.create(dai.node.ImageManip)

        xlinkOut = pipeline.create(dai.node.XLinkOut)
        trackerOut = pipeline.create(dai.node.XLinkOut)
        disparity_out = pipeline.createXLinkOut() 

        # Properties
        camRgb.setPreviewSize(RESIZE_MAPPING[0], RESIZE_MAPPING[1])  # Set the desired image size
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(40)

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
        stereo.setDepthAlign(dai.CameraBoardSocket.RGB)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        stereo.disparity.link(disparity_out.input)

        # MobileNet Detection Network settings
        detectionNetwork.setBlobPath(nnPath)
        detectionNetwork.setConfidenceThreshold(0.5)
        detectionNetwork.input.setBlocking(False)

        # ImageManip to resize frames to 300x300
        manip.initialConfig.setResize(RESIZE_MAPPING[0], RESIZE_MAPPING[1])

        # Configure the ObjectTracker
        objectTracker.setDetectionLabelsToTrack([5, 15])  # Track only a person
        objectTracker.setTrackerType(dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM)
        objectTracker.setTrackerIdAssignmentPolicy(dai.TrackerIdAssignmentPolicy.SMALLEST_ID)

        # Linking
        camRgb.preview.link(manip.inputImage)  # Link the ImageManip for resizing
        manip.out.link(detectionNetwork.input)  # Link resized image to the detection network
        objectTracker.passthroughTrackerFrame.link(xlinkOut.input)
        

        xlinkOut.setStreamName("preview")
        trackerOut.setStreamName("tracklets")
        disparity_out.setStreamName("disparity")

        if fullFrameTracking:
            camRgb.video.link(objectTracker.inputTrackerFrame)
        else:
            detectionNetwork.passthrough.link(objectTracker.inputTrackerFrame)

        detectionNetwork.passthrough.link(objectTracker.inputDetectionFrame)
        detectionNetwork.out.link(objectTracker.inputDetections)
        objectTracker.out.link(trackerOut.input)

        device = dai.Device(pipeline)
        return device

    def detect_objects_and_depth(self):
        fps = 0
        counter = 0
        startTime = time.monotonic()
        preview = self.device.getOutputQueue("preview", 4, False)
        tracklets = self.device.getOutputQueue("tracklets", 4, False)
        disparity_queue = self.device.getOutputQueue("disparity", 4, False)
        while rclpy.ok():
            imgFrame = preview.get()
            track = tracklets.get()
            disparity = disparity_queue.get()
            

            counter += 1
            current_time = time.monotonic()
            if (current_time - startTime) > 1:
                fps = counter / (current_time - startTime)
                counter = 0
                startTime = current_time

            frame = imgFrame.getCvFrame()
            depth_frame = disparity.getCvFrame()
            trackletsData = track.tracklets
            for t in trackletsData:
                try:
                    label = labelMap[t.label]
                except:
                    label = t.label

                
                bbox = self.get_bbox_from_tracklet(t)
                depth = self.get_object_depth(depth_frame, t)
                self.object_publisher.publish(bbox)

                self.visualize_results(frame, t, label, depth, fps)

            cv2.imshow("tracker", frame)
            cv2.imshow("Depth", depth_frame)

            if cv2.waitKey(1) == ord('q'):
                break

    def get_object_depth(self, disparity_frame, t):
        roi = t.roi.denormalize(disparity_frame.shape[1], disparity_frame.shape[0])
        x1 = int(roi.topLeft().x)
        y1 = int(roi.topLeft().y)
        x2 = int(roi.bottomRight().x)
        y2 = int(roi.bottomRight().y)

        # Calculate the average disparity value within the ROI
        disparity_roi = disparity_frame[y1:y2, x1:x2]
        mean_disparity = np.mean(disparity_roi)

        # Use the disparity-to-depth conversion formula
        baseline = 1  # The baseline of your stereo camera in meters
        focal_length = 13  # The focal length of your stereo camera
        depth = mean_disparity / (baseline * focal_length)

        return depth


    def get_bbox_from_tracklet(self, tracklet):
        bbox = BoundingBox2D()
        x_center = (tracklet.roi.x + tracklet.roi.width) / 2
        y_center = (tracklet.roi.y + tracklet.roi.height) / 2
        bbox.center.position.x = x_center
        bbox.center.position.y = y_center
        bbox.size_x = tracklet.roi.width
        bbox.size_y = tracklet.roi.height
        return bbox

    def visualize_results(self, frame, t, label, depth, fps):
        roi = t.roi.denormalize(frame.shape[1], frame.shape[0])
        x1 = int(roi.topLeft().x)
        y1 = int(roi.topLeft().y)
        x2 = int(roi.bottomRight().x)
        y2 = int(roi.bottomRight().y)

        try:
            label = labelMap[t.label]
        except:
            label = t.label

        # Display the label, ID, status, and depth
        cv2.putText(frame, str(label), (x1 + 10, y1 + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0))
        cv2.putText(frame, f"ID: {[t.id]}", (x1 + 10, y1 + 35), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0))
        cv2.putText(frame, t.status.name, (x1 + 10, y1 + 50), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0))
        cv2.putText(frame, f"Depth: {depth:.2f} mm", (x1 + 10, y1 + 65), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 255, 0))

        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 255), cv2.FONT_HERSHEY_SIMPLEX)

        cv2.putText(frame, "NN fps: {:.2f}".format(fps), (2, frame.shape[0] - 4), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (0, 255, 0))


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
