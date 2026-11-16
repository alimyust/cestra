
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped

from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
import cv2


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # Convert ROS Image messages into OpenCV images
        self.bridge = CvBridge()

        # Subscribe to the robot's camera
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        # Publisher for stamped velocity commands
        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Proportional steering gain
        self.Kp = 0.005

        # Forward driving speed (m/s)
        self.linear_speed = 0.1

        self.get_logger().info('Line follower node started')

    def image_callback(self, msg):

        # --------------------------------------------------
        # 1. Convert ROS image message to OpenCV image
        # --------------------------------------------------

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # Get image dimensions
        height, width, _ = frame.shape

        # --------------------------------------------------
        # 2. Image preprocessing
        # --------------------------------------------------

        # Convert the image from BGR to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Threshold the image to isolate dark pixels.
        # Black pixels become white (255) in the mask.
        _, mask = cv2.threshold(
            gray,
            60,
            255,
            cv2.THRESH_BINARY_INV
        )

        # Only examine the bottom portion of the image
        roi = mask[int(height * 0.6):height, :]

        # --------------------------------------------------
        # 3. Find the line contour
        # --------------------------------------------------

        contours, _ = cv2.findContours(
            roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        # Create a stamped velocity message
        cmd = TwistStamped()

        # Set timestamp and reference frame
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        if contours:

            # Assume the largest contour is the line
            largest_contour = max(
                contours,
                key=cv2.contourArea
            )

            # Calculate the contour's center of mass
            M = cv2.moments(largest_contour)

            if M['m00'] > 0:

                cx = int(M['m10'] / M['m00'])

                # --------------------------------------------------
                # 4. Calculate horizontal line error
                # --------------------------------------------------

                # Image center represents desired line position
                image_center = width // 2

                # Positive error means line is to the right
                error = cx - image_center

                # --------------------------------------------------
                # 5. Proportional controller
                # --------------------------------------------------

                # Drive forward and steer toward the line
                cmd.twist.linear.x = self.linear_speed
                cmd.twist.angular.z = -self.Kp * error

                # Draw line center and image center
                cv2.circle(
                    frame,
                    (cx, int(height * 0.8)),
                    8,
                    (0, 0, 255),
                    -1
                )

                cv2.line(
                    frame,
                    (image_center, 0),
                    (image_center, height),
                    (255, 0, 0),
                    2
                )

            else:
                # Invalid contour: stop the robot
                cmd.twist.linear.x = 0.0
                cmd.twist.angular.z = 0.0

        else:
            # No line detected: stop the robot
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

        # --------------------------------------------------
        # 6. Publish velocity commands
        # --------------------------------------------------

        self.cmd_pub.publish(cmd)

        # --------------------------------------------------
        # 7. Display camera and threshold images
        # --------------------------------------------------

        cv2.imshow('Camera', frame)
        cv2.imshow('Line Detection', mask)

        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = LineFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # Publish a final zero-velocity command
    stop = TwistStamped()
    stop.header.stamp = node.get_clock().now().to_msg()
    stop.header.frame_id = 'base_link'

    node.cmd_pub.publish(stop)

    node.destroy_node()
    cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
