import rclpy
from rclpy.node import Node
import depthai as dai
import cv2
import numpy as np
from parallel_parking_bot.msg import ParkingSpot

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.publisher = self.create_publisher(ParkingSpot, 'parking_spot', 10)
        self.lower_blue = np.array([100, 100, 100])
        self.upper_blue = np.array([130, 255, 255])
        self.setup_camera()
        self.timer = self.create_timer(0.1, self.detect_spot)

    def setup_camera(self):
        pipeline = dai.Pipeline()
        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setPreviewSize(640, 480)
        cam_rgb.setInterleaved(False)
        xout = pipeline.createXLinkOut()
        xout.setStreamName("video")
        cam_rgb.preview.link(xout.input)
        self.device = dai.Device(pipeline)
        self.q = self.device.getOutputQueue(name="video", maxSize=4, blocking=False)

    def detect_spot(self):
        in_frame = self.q.tryGet()
        if in_frame is None:
            return
        frame = in_frame.getCvFrame()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        found = False
        x_offset = 0.0
        for cnt in contours:
            if cv2.contourArea(cnt) > 5000:
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                if len(approx) == 4:
                    x, _, w, _ = cv2.boundingRect(cnt)
                    cx = x + w / 2
                    x_offset = (cx - 320) / 320.0
                    found = True
                    break
        msg = ParkingSpot()
        msg.found = found
        msg.x_offset = float(x_offset)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
