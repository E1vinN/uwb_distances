#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class UWBReceiver(Node):
    def __init__(self):
        super().__init__('uwb_receiver')
        self.publisher_1 = self.create_publisher(Float32, 'uwb_distance_1', 10)
        self.publisher_2 = self.create_publisher(Float32, 'uwb_distance_2', 10)
        self.serial_port_1 = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
        self.serial_port_2 = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=1)
        self.timer = self.create_timer(0.1, self.read_distance)  # Adjust the timer interval if needed

    def read_distance(self):
        self.read_from_serial(self.serial_port_1, self.publisher_1, 'Anchor 1')
        self.read_from_serial(self.serial_port_2, self.publisher_2, 'Anchor 2')

    def read_from_serial(self, serial_port, publisher, anchor_name):
        while serial_port.in_waiting > 0:  # Read all available data
            distance_str = serial_port.readline().decode('utf-8').strip()
            try:
                distance = float(distance_str)
                msg = Float32()
                msg.data = distance
                publisher.publish(msg)
                self.get_logger().info(f'Publishing from {anchor_name}: {msg.data}')
            except ValueError:
                self.get_logger().warn(f'{anchor_name}: {distance_str}')

def main(args=None):
    rclpy.init(args=args)
    node = UWBReceiver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.serial_port_1.close()
        node.serial_port_2.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
