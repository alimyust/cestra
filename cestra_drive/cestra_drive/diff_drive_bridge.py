
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Float64

import serial
from cestra_drive import arduino_interface as ar

class DiffDriveBridge(Node):
    def __init__(self, arduino = ar.ArduinoInterface('/dev/ttyACM0', 9600)):
        super().__init__('DiffDriveBridge')

        self.arduino = arduino # Create arduino_interface in DiffDriveBridge node.

        self.publisher_ = self.create_publisher(String, 'wheel_odom', 10)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        
        # self.create_subscription(Float64, 'left_wheel_velocity', self.lw_callback, 10)
        # self.create_subscription(Float64, 'right_wheel_velocity', self.rw_callback, 10)

        self.timer = self.create_timer(1, self.timer_callback)

    def cmd_vel_callback(self, msg:Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        command = f"{linear_x},{angular_z}"
        self.write_arduino(command)

    # def lw_callback(self, msg:Float64):
    #     left_wheel_velocity = msg.data
    #     command = f"L{left_wheel_velocity}"
    #     self.write_arduino(command)

    # def rw_callback(self, msg:Float64):
    #     right_wheel_velocity = msg.data
    #     command = f"R{right_wheel_velocity}"
    #     self.write_arduino(command)

    def timer_callback(self):
        self.read_arduino()

    def write_arduino(self, line):
        if not line: return
        self.arduino.write_line(line)
        self.get_logger().info(f'Sending: "{line}"')

    def read_arduino(self):
        raw_data = self.arduino.read_line()
        if not raw_data: return

        arduino_in = String()
        arduino_in.data = raw_data

        self.publisher_.publish(arduino_in)
        self.get_logger().info(f'Receiving: "{arduino_in.data}"')

def main(args=None):
    rclpy.init(args=args)

    diff_drive_bridge = DiffDriveBridge()
    rclpy.spin(diff_drive_bridge)

    diff_drive_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()