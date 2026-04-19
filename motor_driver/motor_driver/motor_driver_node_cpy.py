import rclpy
from rclpy.node import Node
from motor_interfaces.msg import PWM
import lgpio

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        self.subscription = self.create_subscription(
            PWM,
            '/motor_pwm',
            self.callback,
            10
        )

        # Open GPIO chip
        self.h = lgpio.gpiochip_open()

        # PWM pins (CHANGE THESE for your wiring)
        self.left_pin = 18
        self.right_pin = 19

        # Set PWM frequency
        self.freq = 1000

        lgpio.gpio_claim_output(self.h, self.left_pin)
        lgpio.gpio_claim_output(self.h, self.right_pin)

        self.get_logger().info("Motor driver initialized with lgpio")

    def clamp(self, x):
        return max(-1.0, min(1.0, x))

    def set_pwm(self, pin, value):
        # Convert -1..1 → 0..100 duty cycle
        duty = int((self.clamp(value) + 1.0) * 50.0)

        lgpio.tx_pwm(self.h, pin, self.freq, duty)

    def callback(self, msg):
        self.set_pwm(self.left_pin, msg.left_pwm)
        self.set_pwm(self.right_pin, msg.right_pwm)

        self.get_logger().info(
            f"PWM L={msg.left_pwm:.2f} R={msg.right_pwm:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
