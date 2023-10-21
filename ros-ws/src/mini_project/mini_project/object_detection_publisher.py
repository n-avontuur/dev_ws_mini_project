import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import depthai as dai
import numpy as np

tresholdConfidence = 0.8
nnPath = "/home/n/dev_ws_mini_project/mobilenet-ssd_openvino_2021.2_6shave.blob"
labelMap = ["background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat", "chair", "cow",
            "diningtable", "dog", "horse", "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"]

resizemanip = 300, 300

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__("object_detection_publisher_node")
        self.bridge = CvBridge()
        self.image_msg = Image()
        self.device = self.init_depthai_device()  # Initialize DepthAI device
        self.cam = self.create_publisher(Image, "/camera_image", 10)
        self.get_logger().info("Object Detection Publisher Node started")

    def init_depthai_device(self):
        pipeline = dai.Pipeline()

        monoRight = pipeline.create(dai.node.MonoCamera)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        manip = pipeline.create(dai.node.ImageManip)
        self.stereo = pipeline.create(dai.node.StereoDepth)
        nn = pipeline.create(dai.node.MobileNetDetectionNetwork)

        nnOut = pipeline.create(dai.node.XLinkOut)
        disparityOut = pipeline.create(dai.node.XLinkOut)
        xoutRight = pipeline.create(dai.node.XLinkOut)

        disparityOut.setStreamName("disparity")
        xoutRight.setStreamName("rectifiedRight")
        nnOut.setStreamName("nn")

        monoRight.setCamera("right")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        self.stereo.setRectifyEdgeFillColor(0)
        manip.initialConfig.setResize(resizemanip)
        manip.initialConfig.setFrameType(dai.ImgFrame.Type.BGR888p)

        nn.setConfidenceThreshold(tresholdConfidence)
        nn.setBlobPath(nnPath)
        nn.setNumInferenceThreads(2)
        nn.input.setBlocking(False)

        monoRight.out.link(self.stereo.right)
        monoLeft.out.link(self.stereo.left)
        self.stereo.rectifiedRight.link(manip.inputImage)
        self.stereo.disparity.link(disparityOut.input)
        manip.out.link(nn.input)
        manip.out.link(xoutRight.input)
        nn.out.link(nnOut.input)

        device = dai.Device(pipeline)
        return device

    def detect_objects_and_publish(self):
        q_rgb = self.device.getOutputQueue("rectifiedRight", maxSize=4, blocking=False)
        q_disparity = self.device.getOutputQueue("disparity", maxSize=4, blocking=False)
        q_nn = self.device.getOutputQueue("nn", maxSize=4, blocking=False)

        rightFrame = None
        disparityFrame = None
        detections = []

        def frameNorm(frame, bbox):
            normVals = np.full(len(bbox), frame.shape[0])
            normVals[::2] = frame.shape[1]
            return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

        def calculate_depth(detection, disparity_frame):
            x_min, y_min, x_max, y_max = int(detection.xmin), int(detection.ymin), int(detection.xmax), int(detection.ymax)
            
            # Ensure valid bounds
            if x_max <= x_min:
                self.get_logger().info("flipped x")
                x_min, x_max = x_max, x_min
            if y_max <= y_min:
                self.get_logger().info("flipped y")
                y_min, y_max = y_max, y_min
            
            # Calculate the average disparity within the bounding box
            disparity_crop = disparity_frame[x_min:x_max,y_min:y_max]  # Corrected the order of slicing
            
            
            # Check for an empty ROI
            if disparity_crop.size == 0:
                self.get_logger().info("disparity crop 0")
                return float('nan')
            
            average_disparity = np.nanmean(disparity_crop)  # Use np.nanmean to handle NaN values
            
            # Check if average_disparity is NaN
            if np.isnan(average_disparity):
                self.get_logger().info("average_disparity")
                return float('nan')
            
            # Convert disparity to distance
            distance = 1.0 / average_disparity

            self.get_logger().info("correct values")
            return round(distance, 2)  # Round to 2 decimal places


        def show(name, frame):
            color = (255, 0, 0)
            for detection in detections:
                bbox = frameNorm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                cv2.putText(frame, labelMap[detection.label], (bbox[0] + 10, bbox[1] + 20), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                cv2.putText(frame, f"{int(detection.confidence * 100)}%", (bbox[0] + 10, bbox[1] + 40), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                
                # Display min and max values side by side
                text = f"Min: ({bbox[0]}, {bbox[1]}) Max: ({bbox[2]}, {bbox[3]})"
                cv2.putText(frame, text, (bbox[0] + 10, bbox[1] + 60), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
                
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), color, 2)
                
                distance = calculate_depth(detection, disparityFrame)
                cv2.putText(frame, f"Distance: {distance} m", (bbox[0] + 10, bbox[1] + 80), cv2.FONT_HERSHEY_TRIPLEX, 0.5, color)
            cv2.imshow(name, frame)


        disparityMultiplier = 255 / self.stereo.getMaxDisparity()

        while rclpy.ok():
            if q_nn.has():
                detections = q_nn.get().detections

            if q_rgb.has():
                rightFrame = q_rgb.get().getCvFrame()

            if q_disparity.has():
                disparityFrame = q_disparity.get().getFrame()
                disparityFrame = (disparityFrame * disparityMultiplier).astype(np.uint8)
                disparityFrame = cv2.applyColorMap(disparityFrame, cv2.COLORMAP_JET)
                show("disparity", disparityFrame)

            if rightFrame is not None:
                show("rectified right", rightFrame)

            if cv2.waitKey(1) == ord('q'):
                break

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node.detect_objects_and_publish())
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
