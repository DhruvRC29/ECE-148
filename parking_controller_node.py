import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from parallel_parking_bot_msgs.msg import ParkingSpot
from ackermann_msgs.msg import AckermannDriveStamped
import time

class ParkingController(Node):
    def __init__(self):
        super().__init__('parking_controller')
        # Subscribe to vision detection and joystick topics
        self.create_subscription(ParkingSpot, 'parking_spot', self.vision_callback, 10)
        self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        # Publisher to send drive commands (for VESC)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive_cmd', 10)
        # Controller state
        self.state = 'SEARCHING'
        self.spot_offset = 0.0
        self.forward_mode = False  # whether to do forward park (could be set via another input)
        # Define some movement speeds/angles (could be loaded from config)
        self.cruise_speed = 1.0  # m/s (just an example for approach)
        self.steer_right = 0.4   # ~0.4 rad ~ 23 degrees right
        self.steer_left  = -0.4  # ~ -0.4 rad ~ -23 degrees left
        # Start moving forward slowly to search
        self.send_drive(steering=0.0, speed=self.cruise_speed)

    def vision_callback(self, msg: ParkingSpot):
        if self.state == 'SEARCHING' and msg.found:
            # Spot detected: stop the car and transition to WAIT_FOR_INPUT
            self.send_drive(steering=0.0, speed=0.0)  # stop
            self.spot_offset = msg.x_offset
            self.state = 'WAIT_FOR_INPUT'
            self.get_logger().info("Parking spot found. Waiting for user to press Y to park.")
            # (Optional alignment logic: if spot_offset indicates we are not parallel, 
            # adjust position or steering slightly here before parking.)

    def joy_callback(self, joy_msg: Joy):
        # Assuming Logitech F710 (XInput mode), Y button is index 3:
        if self.state == 'WAIT_FOR_INPUT' and joy_msg.buttons[3] == 1:
            # Trigger parking maneuver
            self.state = 'PARKING'
            if self.forward_mode:
                self.get_logger().info("Executing forward parking maneuver...")
                self.execute_forward_park()
            else:
                self.get_logger().info("Executing reverse parking maneuver...")
                self.execute_reverse_park()
            self.state = 'FINISHED'
            self.get_logger().info("Parking maneuver completed.")

    def execute_reverse_park(self):
        # Step 1: Turn right and reverse
        self.send_drive(self.steer_right, speed=-0.5)
        time.sleep(2.0)  # reverse for 2 seconds (adjust time for proper distance)
        # Step 2: Turn left while continuing to reverse
        self.send_drive(self.steer_left, speed=-0.5)
        time.sleep(2.0)  # another 2 seconds
        # Step 3: Straighten wheels and stop
        self.send_drive(steering=0.0, speed=0.0)
        # (Now car should be in the spot. Optionally, one could do a small forward straighten if needed.)

    def execute_forward_park(self):
        # Example forward park sequence (if needed):
        # Step 1: Turn left and forward
        self.send_drive(self.steer_left, speed=0.5)
        time.sleep(2.0)
        # Step 2: Turn right to straighten while moving forward
        self.send_drive(self.steer_right, speed=0.5)
        time.sleep(2.0)
        # Step 3: Straighten and stop
        self.send_drive(steering=0.0, speed=0.0)

    def send_drive(self, steering: float, speed: float):
        # Helper to publish a drive command
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steering
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

