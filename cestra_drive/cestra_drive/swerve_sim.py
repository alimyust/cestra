

from math import atan2
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import String

class SwerveSim(Node):
    def __init__(self):
        super().__init__('SwerveSim')

        self.cmd_vel_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.wheel_pub = {
            'front_left': self.create_publisher(Float64, '/front_left/wheel_joint/cmd_vel', 10),
            'front_right': self.create_publisher(Float64, '/front_right/wheel_joint/cmd_vel', 10),
            'rear_left': self.create_publisher(Float64, '/rear_left/wheel_joint/cmd_vel', 10),
            'rear_right': self.create_publisher(Float64, '/rear_right/wheel_joint/cmd_vel', 10)
        }

        # Publishers for steering position commands
        self.steer_pub = {
            'front_left': self.create_publisher(Float64, '/front_left/steer_joint/cmd_vel', 10),
            'front_right': self.create_publisher(Float64, '/front_right/steer_joint/cmd_vel', 10),
            'rear_left': self.create_publisher(Float64, '/rear_left/steer_joint/cmd_vel', 10),
            'rear_right': self.create_publisher(Float64, '/rear_right/steer_joint/cmd_vel', 10)
        }
        # Wheel positions relative to robot center (of 0.3m)
        self.wheel_pos = [
            (0.3, 0.3),   # Front left
            (0.3, -0.3),  # Front right
            (-0.3, 0.3),  # Rear left
            (-0.3, -0.3)  # Rear right
        ]

    def cmd_vel_callback(self, msg:Twist):

        linear_x = msg.linear.x
        lieanr_y = msg.linear.y
        w = msg.angular.z # omega (yaw)
        # Big V (desired movement for whole body)

        for i, (wheel_x, wheel_y) in enumerate(self.wheel_pos):
            # Calculate the wheel speed and angle based on robot velocity and wheel position
            wheel_speed = ((linear_x - w * wheel_y) ** 2 + (lieanr_y + w * wheel_x) ** 2) ** 0.5
            steer_angle = atan2(lieanr_y + w * wheel_x, linear_x - w * wheel_y)

            wheel_name = list(self.wheel_pub.keys())[i]
            self.get_logger().info(f"Wheel: {wheel_name}, wheel_speed={wheel_speed}, steer_angle={steer_angle}")

            speed_msg = Float64()
            speed_msg.data = wheel_speed
            self.wheel_pub[wheel_name].publish(speed_msg)

            angle_msg = Float64()
            angle_msg.data = steer_angle
            self.steer_pub[wheel_name].publish(angle_msg)


def main(args=None):
    rclpy.init(args=args)

    swerve_sim = SwerveSim()
    rclpy.spin(swerve_sim)

    swerve_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()