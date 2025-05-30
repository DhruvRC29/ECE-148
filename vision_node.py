import rclpy
from rclpy.node import Node
import depthai as dai
import cv2
from parallel_parking_bot_msgs.msg import ParkingSpot  # assume a custom msg

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        # Declare parameters for color threshold
        self.declare_parameter('lower_blue_hsv', [100, 100, 100])
        self.declare_parameter('upper_blue_hsv', [130, 255, 255])
        lb = self.get_parameter('lower_blue_hsv').get_parameter_value().integer_array_value
        ub = self.get_parameter('upper_blue_hsv').get_parameter_value().integer_array_value
        self.lower_blue = tuple(int(v) for v in lb)
        self.upper_blue = tuple(int(v) for v in ub)
        # Setup DepthAI pipeline
        pipeline = dai.Pipeline()
        cam = pipeline.createColorCamera()
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setIspScale(2, 3)  # downscale to speed up, for example
        cam.setInterleaved(False)
        xout = pipeline.createXLinkOut()
        xout.setStreamName("cam_out")
        cam.preview.link(xout.input)
        # Start pipeline
        self.device = dai.Device(pipeline)
        self.q = self.device.getOutputQueue(name="cam_out", maxSize=4, blocking=False)
        # Publisher for parking spot detection
        self.pub = self.create_publisher(ParkingSpot, 'parking_spot', 10)
        # Create a timer to periodically grab frames
        self.timer = self.create_timer(0.1, self.process_frame)  # 10 Hz (adjust as needed)

    def process_frame(self):
        in_frame = self.q.tryGet()
        if in_frame is None:
            return
        frame = in_frame.getCvFrame()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        spot_detected = False
        spot_msg = ParkingSpot()
        # Find largest contour that looks like a rectangle
        max_area = 0
        best_cnt = None
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 5000:  # ignore very small patches; threshold tuned experimentally
                continue
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            if len(approx) == 4 and area > max_area:
                # Found a rectangle candidate
                max_area = area
                best_cnt = cnt
        if best_cnt is not None:
            # Compute offset of spot from image center
            x, y, w, h = cv2.boundingRect(best_cnt)
            cx = x + w/2
            cy = y + h/2
            img_width = frame.shape[1]
            offset_x = (cx - img_width/2) / (img_width/2)  # normalized offset (-1 to 1)
            spot_msg.found = True
            spot_msg.x_offset = float(offset_x)
            # (distance could be estimated via known size or stereo depth if needed)
            spot_detected = True
        else:
            spot_msg.found = False
        self.pub.publish(spot_msg)
        # Optionally, for debugging: show or record images (omitted for headless operation)
