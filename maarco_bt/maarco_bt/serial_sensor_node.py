#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading

# node for reading data from sensors.
class SerialSensorNode(Node):
    def __init__(self):
        super().__init__('serial_sensor_node')

        # ROS2 params for port and baud rate.
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.pub_raw = self.create_publisher(Float32MultiArray, 'serial/raw', 10)

        try:
            self.ser = serial.Serial(port, baud_rate, timeout=1.0)
            self.get_logger().info(f'Opened serial port {port} at {baud_rate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        self.running = True
        self.thread = threading.Thread(target=self.read_loop, daemon=True)
        self.thread.start()

    # Override to also close serial port.
    def destroy_node(self):
        self.running = False
        if self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)
    node = SerialSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()