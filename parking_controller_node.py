import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import time

class AutoParkingControllerNode(Node):
    def __init__(self):
        super().__init__('auto_parking_controller_node')
        self.drive_pub = self.create_publisher(AckermannDriveStamped, 'drive_cmd', 10)
        self.timer = self.create_timer(2.0, self.start_parking)  # Wait 2s after launch

    def start_parking(self):
        self.get_logger().info("Starting automatic parking maneuver...")
        self.timer.cancel()
        self.execute_reverse_park()

    def execute_reverse_park(self):
        self.send_drive(0.4, -0.3)  # Steer right, reverse
        time.sleep(2.0)
        self.send_drive(-0.4, -0.3)  # Steer left, reverse
        time.sleep(2.0)
        self.send_drive(0.0, 0.0)  # Stop
        self.get_logger().info("Finished parking.")

    def send_drive(self, steering, speed):
        msg = AckermannDriveStamped()
        msg.drive.steering_angle = steering
        msg.drive.speed = speed
        self.drive_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AutoParkingControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
