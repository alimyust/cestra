import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import gz.transport13 as gz_transport
try:
    # Prefer the plain Twist message when available in gz.msgs
    from gz.msgs.twist_pb2 import Twist as GzTwist
    from gz.msgs.vector3d_pb2 import Vector3d as GzVector3d
    _GZ_TWIST_PB_OK = True
except Exception:
    # Fallback: some installations expose a TwistCmd message instead
    try:
        from gz.msgs.twist_cmd_pb2 import TwistCmd as GzTwist
        from gz.msgs.vector3d_pb2 import Vector3d as GzVector3d
        _GZ_TWIST_PB_OK = True
    except Exception:
        GzTwist = None
        GzVector3d = None
        _GZ_TWIST_PB_OK = False


# gz topic -t "/model/vehicle/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: -0.1}"
# ros2 topic pub /model/vehicle/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 5.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.1}}

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__("cmd_vel_bridge")
        # Parameters: ROS topic to subscribe and GZ topic to publish
        self.declare_parameter('ros_topic', 'orion/cmd_vel')
        self.declare_parameter('gz_topic', '/model/cestra/cmd_vel')

        ros_topic = self.get_parameter('ros_topic').get_parameter_value().string_value
        gz_topic = self.get_parameter('gz_topic').get_parameter_value().string_value

        if not _GZ_TWIST_PB_OK:
            self.get_logger().error('Could not import Gazebo protobuf Twist message types (gz.msgs). '
                                    'Make sure gz Python packages are available.')

        # Create a Gazebo transport node and advertise the twist topic
        try:
            self.gz_node = gz_transport.Node()
            # advertise with the type name that most gz plugins expect
            self.gz_pub = self.gz_node.Advertise(gz_topic, 'gz.msgs.Twist')
            self.get_logger().info(f'Advertising to GZ topic: {gz_topic}')
        except Exception as e:
            self.get_logger().error(f'Failed to create Gazebo transport publisher: {e}')
            self.gz_node = None
            self.gz_pub = None

        # ROS subscription
        self.sub = self.create_subscription(
            Twist,
            ros_topic,
            self._twist_cb,
            10
        )
        self.get_logger().info(f'Subscribed to ROS topic: {ros_topic}')

    def _twist_cb(self, msg: Twist):
        """Callback when a ROS Twist is received; repack into a gz message and publish."""
        if not self.gz_pub or not GzTwist or not GzVector3d:
            # Nothing to do
            self.get_logger().warning('Gazebo publisher or message types not available; dropping cmd_vel')
            return

        try:
            gz_msg = GzTwist()
            # set linear
            gz_msg.linear.x = float(msg.linear.x)
            gz_msg.linear.y = float(msg.linear.y)
            gz_msg.linear.z = float(msg.linear.z)
            # set angular
            # some gz Twist message variants nest angular as a Vector3d named "angular"
            # while others may name differently; assume common fields exist
            gz_msg.angular.x = float(msg.angular.x)
            gz_msg.angular.y = float(msg.angular.y)
            gz_msg.angular.z = float(msg.angular.z)

            # Publish to Gazebo
            self.gz_pub.Publish(gz_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish to Gazebo topic: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()

        