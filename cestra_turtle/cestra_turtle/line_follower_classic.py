
# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from sensor_msgs.msg import Image
# from geometry_msgs.msg import TwistStamped

# from rclpy.qos import qos_profile_sensor_data

# from cv_bridge import CvBridge
# import cv2


# class LineFollower(Node):

#     def __init__(self):
#         super().__init__('line_follower')

#         # Convert ROS Image messages into OpenCV images
#         self.bridge = CvBridge()

#         # Subscribe to the robot's camera
#         self.image_sub = self.create_subscription(
#             Image,
#             '/camera/image_raw',
#             self.image_callback,
#             qos_profile_sensor_data
#         )

#         # Publisher for stamped velocity commands
#         self.cmd_pub = self.create_publisher(
#             TwistStamped,
#             '/cmd_vel',
#             10
#         )

#         # Proportional steering gain
#         self.Kp = 0.005

#         # Forward driving speed (m/s)
#         self.linear_speed = 0.1

#         self.get_logger().info('Line follower node started')

#     def image_callback(self, msg):

#         # --------------------------------------------------
#         # 1. Convert ROS image message to OpenCV image
#         # --------------------------------------------------

#         frame = self.bridge.imgmsg_to_cv2(
#             msg,
#             desired_encoding='bgr8'
#         )

#         # Get image dimensions
#         height, width, _ = frame.shape

#         # --------------------------------------------------
#         # 2. Image preprocessing
#         # --------------------------------------------------

#         # Convert the image from BGR to grayscale
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#         # Threshold the image to isolate dark pixels.
#         # Black pixels become white (255) in the mask.
#         _, mask = cv2.threshold(
#             gray,
#             60,
#             255,
#             cv2.THRESH_BINARY_INV
#         )

#         # Only examine the bottom portion of the image
#         roi = mask[int(height * 0.6):height, :]

#         # --------------------------------------------------
#         # 3. Find the line contour
#         # --------------------------------------------------

#         contours, _ = cv2.findContours(
#             roi,
#             cv2.RETR_EXTERNAL,
#             cv2.CHAIN_APPROX_SIMPLE
#         )

#         # Create a stamped velocity message
#         cmd = TwistStamped()

#         # Set timestamp and reference frame
#         cmd.header.stamp = self.get_clock().now().to_msg()
#         cmd.header.frame_id = 'base_link'

#         if contours:

#             # Assume the largest contour is the line
#             largest_contour = max(
#                 contours,
#                 key=cv2.contourArea
#             )

#             # Calculate the contour's center of mass
#             M = cv2.moments(largest_contour)

#             if M['m00'] > 0:

#                 cx = int(M['m10'] / M['m00'])

#                 # --------------------------------------------------
#                 # 4. Calculate horizontal line error
#                 # --------------------------------------------------

#                 # Image center represents desired line position
#                 image_center = width // 2

#                 # Positive error means line is to the right
#                 error = cx - image_center

#                 # --------------------------------------------------
#                 # 5. Proportional controller
#                 # --------------------------------------------------

#                 # Drive forward and steer toward the line
#                 cmd.twist.linear.x = self.linear_speed
#                 cmd.twist.angular.z = -self.Kp * error

#                 # Draw line center and image center
#                 cv2.circle(
#                     frame,
#                     (cx, int(height * 0.8)),
#                     8,
#                     (0, 0, 255),
#                     -1
#                 )

#                 cv2.line(
#                     frame,
#                     (image_center, 0),
#                     (image_center, height),
#                     (255, 0, 0),
#                     2
#                 )

#             else:
#                 # Invalid contour: stop the robot
#                 cmd.twist.linear.x = 0.0
#                 cmd.twist.angular.z = 0.0

#         else:
#             # No line detected: stop the robot
#             cmd.twist.linear.x = 0.0
#             cmd.twist.angular.z = 0.0

#         # --------------------------------------------------
#         # 6. Publish velocity commands
#         # --------------------------------------------------

#         self.cmd_pub.publish(cmd)

#         # --------------------------------------------------
#         # 7. Display camera and threshold images
#         # --------------------------------------------------

#         cv2.imshow('Camera', frame)
#         cv2.imshow('Line Detection', mask)

#         cv2.waitKey(1)


# def main(args=None):

#     rclpy.init(args=args)

#     node = LineFollower()

#     try:
#         rclpy.spin(node)

#     except KeyboardInterrupt:
#         pass

#     # Publish a final zero-velocity command
#     stop = TwistStamped()
#     stop.header.stamp = node.get_clock().now().to_msg()
#     stop.header.frame_id = 'base_link'

#     node.cmd_pub.publish(stop)

#     node.destroy_node()
#     cv2.destroyAllWindows()

#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()



#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        self.bridge = CvBridge()

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------

        self.use_webcam = self.declare_parameter(
            'use_webcam', True
        ).value

        self.camera_id = self.declare_parameter(
            'camera_id', 0
        ).value

        self.camera_width = self.declare_parameter(
            'camera_width', 640
        ).value

        self.camera_height = self.declare_parameter(
            'camera_height', 480
        ).value

        self.camera_fps = self.declare_parameter(
            'camera_fps', 30.0
        ).value

        self.camera_topic = self.declare_parameter(
            'camera_topic', '/camera/image_raw'
        ).value

        # Crop: retain image rows from this fraction to bottom.
        self.crop_start = self.declare_parameter(
            'crop_start', 0.6
        ).value

        self.blur_kernel = self.declare_parameter(
            'blur_kernel', 5
        ).value

        # Red HSV ranges. OpenCV hue is 0-179.
        self.low_h1 = self.declare_parameter(
            'low_h1', 0
        ).value

        self.high_h1 = self.declare_parameter(
            'high_h1', 10
        ).value

        self.low_h2 = self.declare_parameter(
            'low_h2', 170
        ).value

        self.high_h2 = self.declare_parameter(
            'high_h2', 179
        ).value

        self.saturation_min = self.declare_parameter(
            'saturation_min', 100
        ).value

        self.value_min = self.declare_parameter(
            'value_min', 50
        ).value

        self.morph_kernel = self.declare_parameter(
            'morph_kernel', 5
        ).value

        self.open_iterations = self.declare_parameter(
            'open_iterations', 1
        ).value

        self.close_iterations = self.declare_parameter(
            'close_iterations', 2
        ).value

        self.min_pixels = self.declare_parameter(
            'min_pixels', 100
        ).value

        # Controller
        self.Kp = self.declare_parameter(
            'kp', 0.005
        ).value

        self.linear_speed = self.declare_parameter(
            'linear_speed', 0.1
        ).value

        self.max_angular = self.declare_parameter(
            'max_angular', 1.5
        ).value

        self.frame_timeout = self.declare_parameter(
            'frame_timeout', 0.5
        ).value

        # Visualization switches
        self.show_camera = self.declare_parameter(
            'show_camera', False
        ).value

        self.show_cropped = self.declare_parameter(
            'show_cropped', False
        ).value

        self.show_blurred = self.declare_parameter(
            'show_blurred', False
        ).value

        self.show_red_mask = self.declare_parameter(
            'show_red_mask', False
        ).value

        self.show_clean_mask = self.declare_parameter(
            'show_clean_mask', False
        ).value

        self.show_debug = self.declare_parameter(
            'show_debug', False
        ).value

        # --------------------------------------------------
        # Publishers
        # --------------------------------------------------

        self.camera_pub = self.create_publisher(
            Image, '/camera/image_raw', 10
        )

        self.cropped_pub = self.create_publisher(
            Image, '/vision/cropped', 10
        )

        self.blurred_pub = self.create_publisher(
            Image, '/vision/blurred', 10
        )

        self.red_mask_pub = self.create_publisher(
            Image, '/vision/red_mask', 10
        )

        self.clean_mask_pub = self.create_publisher(
            Image, '/vision/clean_mask', 10
        )

        self.debug_pub = self.create_publisher(
            Image, '/vision/lane_debug', 10
        )

        self.offset_pub = self.create_publisher(
            Float32, '/vision/lane_offset', 10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped, '/cmd_vel', 10
        )

        # --------------------------------------------------
        # Camera input
        # --------------------------------------------------

        self.cap = None

        if self.use_webcam:
            self.cap = cv2.VideoCapture(self.camera_id)

            self.cap.set(
                cv2.CAP_PROP_FRAME_WIDTH,
                self.camera_width
            )

            self.cap.set(
                cv2.CAP_PROP_FRAME_HEIGHT,
                self.camera_height
            )

            self.cap.set(
                cv2.CAP_PROP_FPS,
                self.camera_fps
            )

            if not self.cap.isOpened():
                raise RuntimeError(
                    f'Could not open camera {self.camera_id}'
                )

            self.camera_timer = self.create_timer(
                1.0 / self.camera_fps,
                self.capture_frame
            )

            self.get_logger().info(
                f'Using webcam /dev/video{self.camera_id}'
            )

        else:
            self.image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.image_callback,
                qos_profile_sensor_data
            )

            self.get_logger().info(
                f'Subscribing to {self.camera_topic}'
            )

        # Stop if camera frames stop arriving.
        self.last_frame_time = self.get_clock().now()

        self.watchdog_timer = self.create_timer(
            0.1,
            self.check_camera_timeout
        )

        self.get_logger().info('Red lane follower started')

    # ======================================================
    # Camera capture
    # ======================================================

    def capture_frame(self):
        if self.cap is None:
            return

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning(
                'Failed to read webcam frame'
            )
            self.stop_robot()
            return

        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        self.camera_pub.publish(msg)

        # Process the captured frame directly.
        self.process_frame(frame, msg.header.stamp)

    # ======================================================
    # ROS image subscription (simulation / external camera)
    # ======================================================

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            self.process_frame(frame, msg.header.stamp)

        except Exception as error:
            self.get_logger().error(
                f'Image conversion failed: {error}'
            )
            self.stop_robot()

    # ======================================================
    # Image publishing helper
    # ======================================================

    def publish_image(self, image, publisher, stamp):
        encoding = 'bgr8' if image.ndim == 3 else 'mono8'

        msg = self.bridge.cv2_to_imgmsg(
            image,
            encoding=encoding
        )

        msg.header.stamp = stamp
        msg.header.frame_id = 'camera'

        publisher.publish(msg)

    # ======================================================
    # Main image processing pipeline
    # ======================================================

    def process_frame(self, frame, stamp):

        self.last_frame_time = self.get_clock().now()

        height, width, _ = frame.shape

        # --------------------------------------------------
        # 1. Original camera image
        # --------------------------------------------------

        if self.show_camera:
            cv2.imshow('1 - Camera', frame)

        # --------------------------------------------------
        # 2. Crop ROI
        # --------------------------------------------------

        crop_y = int(height * self.crop_start)

        cropped = frame[crop_y:height, :]

        self.publish_image(
            cropped,
            self.cropped_pub,
            stamp
        )

        if self.show_cropped:
            cv2.imshow('2 - Cropped ROI', cropped)

        # --------------------------------------------------
        # 3. Gaussian blur
        # --------------------------------------------------

        kernel = max(1, int(self.blur_kernel))

        if kernel % 2 == 0:
            kernel += 1

        blurred = cv2.GaussianBlur(
            cropped,
            (kernel, kernel),
            0
        )

        self.publish_image(
            blurred,
            self.blurred_pub,
            stamp
        )

        if self.show_blurred:
            cv2.imshow('3 - Blurred', blurred)

        # --------------------------------------------------
        # 4. Red color segmentation (BGR -> HSV)
        # --------------------------------------------------

        hsv = cv2.cvtColor(
            blurred,
            cv2.COLOR_BGR2HSV
        )

        lower_red_1 = np.array([
            self.low_h1,
            self.saturation_min,
            self.value_min
        ])

        upper_red_1 = np.array([
            self.high_h1,
            255,
            255
        ])

        lower_red_2 = np.array([
            self.low_h2,
            self.saturation_min,
            self.value_min
        ])

        upper_red_2 = np.array([
            self.high_h2,
            255,
            255
        ])

        mask_1 = cv2.inRange(
            hsv,
            lower_red_1,
            upper_red_1
        )

        mask_2 = cv2.inRange(
            hsv,
            lower_red_2,
            upper_red_2
        )

        red_mask = cv2.bitwise_or(
            mask_1,
            mask_2
        )

        self.publish_image(
            red_mask,
            self.red_mask_pub,
            stamp
        )

        if self.show_red_mask:
            cv2.imshow('4 - Red Threshold', red_mask)

        # --------------------------------------------------
        # 5. Morphological cleanup
        # --------------------------------------------------

        morph_size = max(1, int(self.morph_kernel))

        morph = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (morph_size, morph_size)
        )

        clean_mask = cv2.morphologyEx(
            red_mask,
            cv2.MORPH_OPEN,
            morph,
            iterations=self.open_iterations
        )

        clean_mask = cv2.morphologyEx(
            clean_mask,
            cv2.MORPH_CLOSE,
            morph,
            iterations=self.close_iterations
        )

        self.publish_image(
            clean_mask,
            self.clean_mask_pub,
            stamp
        )

        if self.show_clean_mask:
            cv2.imshow('5 - Clean Mask', clean_mask)

        # --------------------------------------------------
        # 6. Find lane center using image moments
        # --------------------------------------------------

        moments = cv2.moments(clean_mask)

        pixel_count = cv2.countNonZero(clean_mask)

        debug = cv2.cvtColor(
            clean_mask,
            cv2.COLOR_GRAY2BGR
        )

        image_center = width // 2

        offset_msg = Float32()

        cmd = TwistStamped()
        cmd.header.stamp = stamp
        cmd.header.frame_id = 'base_link'

        if pixel_count >= self.min_pixels:

            cx = int(
                moments['m10'] / moments['m00']
            )

            # Positive = lane is right of image center.
            error = cx - image_center

            offset_msg.data = float(error)

            angular = -self.Kp * error

            angular = max(
                -self.max_angular,
                min(self.max_angular, angular)
            )

            cmd.twist.linear.x = float(self.linear_speed)
            cmd.twist.angular.z = float(angular)

            # Draw lane center.
            cv2.circle(
                debug,
                (cx, debug.shape[0] // 2),
                8,
                (0, 255, 0),
                -1
            )

            cv2.putText(
                debug,
                f'Offset: {error}px',
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

        else:
            # No reliable red lane detected.
            offset_msg.data = float('nan')

            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

            cv2.putText(
                debug,
                'NO LANE DETECTED',
                (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2
            )

        # Draw image center.
        cv2.line(
            debug,
            (image_center, 0),
            (image_center, debug.shape[0]),
            (255, 0, 0),
            2
        )

        self.offset_pub.publish(offset_msg)

        self.publish_image(
            debug,
            self.debug_pub,
            stamp
        )

        # --------------------------------------------------
        # 7. Publish velocity commands
        # --------------------------------------------------

        self.cmd_pub.publish(cmd)

        if self.show_debug:
            cv2.imshow('6 - Lane Debug', debug)

        if any([
            self.show_camera,
            self.show_cropped,
            self.show_blurred,
            self.show_red_mask,
            self.show_clean_mask,
            self.show_debug
        ]):
            cv2.waitKey(1)

    # ======================================================
    # Watchdog
    # ======================================================

    def check_camera_timeout(self):
        elapsed = (
            self.get_clock().now() - self.last_frame_time
        ).nanoseconds / 1e9

        if elapsed > self.frame_timeout:
            self.stop_robot()

    def stop_robot(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ======================================================
    # Cleanup
    # ======================================================

    def destroy_node(self):
        self.stop_robot()

        if self.cap is not None:
            self.cap.release()

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = LineFollower()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(f'Line follower error: {error}')

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
