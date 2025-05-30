import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from pyvesc import VESC

class VescDriverNode(Node):
    def __init__(self):
        super().__init__('vesc_driver_node')
        self.declare_parameter('vesc_port', '/dev/ttyACM0')
        port = self.get_parameter('vesc_port').get_parameter_value().string_value
        try:
            self.vesc = VESC(serial_port=port)
            self.get_logger().info(f"Connected to VESC at {port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to VESC: {e}")
            raise
        self.sub = self.create_subscription(AckermannDriveStamped, 'drive_cmd', self.cmd_cb, 10)
        self.steering_center = 0.53
        self.steering_scale = 0.20

    def cmd_cb(self, msg):
        angle = msg.drive.steering_angle
        speed = msg.drive.speed
        servo_pos = self.steering_center + (angle / 0.4) * self.steering_scale
        servo_pos = max(0.0, min(1.0, servo_pos))
        duty = max(-1.0, min(1.0, speed / 3.0))
        try:
            self.vesc.set_servo(servo_pos)
            self.vesc.set_duty_cycle(duty)
        except Exception as e:
            self.get_logger().error(f"VESC command failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VescDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
