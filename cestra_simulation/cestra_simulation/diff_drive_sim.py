import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

"""
Left Cmd-Vel + wheel radius => Wheel position

"""


class DiffDriveSim(Node):
    def __init__(self):
        super().__init__('diff_drive_sim')
        
        self.get_logger().info('DiffDriveSim node has been started.')
        self.wheel_radius = 0.05  # meters
        self.wheel_base = 0.3   # meters

        self.subscriber = self.create_subscription(Twist,'cmd_vel',self.cmd_vel_callback,10)
        self.left_wheel_publisher_ = self.create_publisher(Float64, 'left_wheel_velocity', 10)
        self.right_wheel_publisher_ = self.create_publisher(Float64, 'right_wheel_velocity', 10)
            
    def cmd_vel_callback(self, msg:Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        v_left = (linear_x - angular_z * (self.wheel_base / 2.0)) / self.wheel_radius
        v_right = (linear_x + angular_z * (self.wheel_base / 2.0)) / self.wheel_radius

        left_wheel_msg = Float64()
        right_wheel_msg = Float64()
        left_wheel_msg.data = v_left
        right_wheel_msg.data = v_right

        self.left_wheel_publisher_.publish(left_wheel_msg)
        self.right_wheel_publisher_.publish(right_wheel_msg)
        self.get_logger().info(f'Published wheel commands - Left: {v_left}, Right: {v_right}')
    
def main(args=None):
    rclpy.init(args=args)

    diff_drive_sim = DiffDriveSim()
    rclpy.spin(diff_drive_sim)

    diff_drive_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()