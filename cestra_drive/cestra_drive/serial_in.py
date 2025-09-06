
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial
from cestra_drive import arduino_interface as ar

class SerialIn(Node):

    def __init__(self, arduino = ar.ArduinoInterface('/dev/ttyACM0', 9600)):
        super().__init__('SerialIn')
        self.arduino = arduino # Create arduino_interface in SerialIn node.
        self.publisher_ = self.create_publisher(String, 'arduino_in', 10)
        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):
        self.read_arduino()
        self.write_arduino("Hello from ROS2")

    def write_arduino(self, line):
        if not line: return
        self.arduino.write_line(line)
        self.get_logger().info(f'Sending: "{line}"')

    def read_arduino(self):
        raw_data = self.arduino.read_line()
        if raw_data == "NONE" or raw_data is None: return

        arduino_in = String()
        arduino_in.data = raw_data

        self.publisher_.publish(arduino_in)
        self.get_logger().info(f'Receiving: "{arduino_in.data}"')

def main(args=None):
    rclpy.init(args=args)

    serial_in = SerialIn()
    rclpy.spin(serial_in)

    serial_in.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
