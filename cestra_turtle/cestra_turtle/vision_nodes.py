import rclpy
import cv2
import numpy as np
import math
import time

from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import TwistStamped

from cv_bridge import CvBridge


# ============================================================
# Base image node
# ============================================================
class WebcamNode(Node):

    def __init__(self):
        super().__init__('webcam_node')

        self.bridge = CvBridge()

        self.camera_id = self.declare_parameter(
            'camera_id', 0
        ).value

        self.width = self.declare_parameter(
            'width', 640
        ).value

        self.height = self.declare_parameter(
            'height', 480
        ).value

        self.fps = self.declare_parameter(
            'fps', 30.0
        ).value

        self.show_image = self.declare_parameter(
            'show_image', True
        ).value

        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        self.cap = cv2.VideoCapture(self.camera_id)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            raise RuntimeError('Could not open webcam')

        self.timer = self.create_timer(
            1.0 / self.fps,
            self.capture_frame
        )

        self.get_logger().info('OpenCV webcam started')

    def capture_frame(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning('Failed to capture frame')
            return

        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'

        self.image_pub.publish(msg)

        if self.show_image:
            cv2.imshow('Webcam', frame)
            cv2.waitKey(1)

    def destroy_node(self):

        self.cap.release()
        cv2.destroyAllWindows()

        super().destroy_node()

class ImageNode(Node):

    def __init__(self, name, input_topic, output_topic):
        super().__init__(name)

        self.bridge = CvBridge()

        self.show_image = self.declare_parameter(
            'show_image', False
        ).value

        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.image_pub = self.create_publisher(
            Image,
            output_topic,
            10
        )

    def publish_image(self, image, encoding='bgr8'):
        msg = self.bridge.cv2_to_imgmsg(
            image,
            encoding=encoding
        )
        self.image_pub.publish(msg)

    def display(self, title, image):
        if self.show_image:
            cv2.imshow(title, image)
            cv2.waitKey(1)

    def image_callback(self, msg):
        raise NotImplementedError


# ============================================================
# 1. CROP NODE
# ============================================================

class CropNode(ImageNode):

    def __init__(self):
        super().__init__(
            'crop_node',
            '/camera/image_raw',
            '/vision/cropped'
        )

        self.crop_start = self.declare_parameter(
            'crop_start', 0.5
        ).value

        self.crop_end = self.declare_parameter(
            'crop_end', 1.0
        ).value

        self.get_logger().info('Crop node started')

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        height, width, _ = frame.shape

        y1 = int(height * self.crop_start)
        y2 = int(height * self.crop_end)

        cropped = frame[y1:y2, :]

        self.publish_image(cropped)

        self.display('1 - Cropped Image', cropped)


# ============================================================
# 2. BLUR NODE
# ============================================================

class BlurNode(ImageNode):

    def __init__(self):
        super().__init__(
            'blur_node',
            '/vision/cropped',
            '/vision/blurred'
        )

        self.kernel_size = self.declare_parameter(
            'kernel_size', 5
        ).value

        self.get_logger().info('Blur node started')

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        k = max(1, int(self.kernel_size))

        # Gaussian kernel must have odd dimensions
        if k % 2 == 0:
            k += 1

        blurred = cv2.GaussianBlur(
            frame,
            (k, k),
            0
        )

        self.publish_image(blurred)

        self.display('2 - Blurred Image', blurred)


# ============================================================
# 3. RED THRESHOLD NODE
# ============================================================

class ThresholdNode(ImageNode):

    def __init__(self):
        super().__init__(
            'threshold_node',
            '/vision/blurred',
            '/vision/red_mask'
        )

        # Red has two hue ranges in OpenCV HSV.
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
            'high_h2', 180
        ).value

        self.saturation_min = self.declare_parameter(
            'saturation_min', 100
        ).value

        self.value_min = self.declare_parameter(
            'value_min', 50
        ).value

        self.get_logger().info('Threshold node started')

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        # Convert BGR to HSV
        hsv = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2HSV
        )

        # First red hue range
        lower_red1 = np.array([
            self.low_h1,
            self.saturation_min,
            self.value_min
        ])

        upper_red1 = np.array([
            self.high_h1,
            255,
            255
        ])

        # Second red hue range
        lower_red2 = np.array([
            self.low_h2,
            self.saturation_min,
            self.value_min
        ])

        upper_red2 = np.array([
            self.high_h2,
            255,
            255
        ])

        mask1 = cv2.inRange(
            hsv,
            lower_red1,
            upper_red1
        )

        mask2 = cv2.inRange(
            hsv,
            lower_red2,
            upper_red2
        )

        # Combine both red ranges
        mask = cv2.bitwise_or(
            mask1,
            mask2
        )

        self.publish_image(mask, 'mono8')

        self.display('3 - Red Threshold', mask)


# ============================================================
# 4. CLEANUP NODE
# ============================================================

class CleanupNode(ImageNode):

    def __init__(self):
        super().__init__(
            'cleanup_node',
            '/vision/red_mask',
            '/vision/clean_mask'
        )

        self.kernel_size = self.declare_parameter(
            'kernel_size', 5
        ).value

        self.open_iterations = self.declare_parameter(
            'open_iterations', 1
        ).value

        self.close_iterations = self.declare_parameter(
            'close_iterations', 2
        ).value

        self.get_logger().info('Cleanup node started')

    def image_callback(self, msg):

        mask = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='mono8'
        )

        k = max(1, int(self.kernel_size))

        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (k, k)
        )

        # Remove small isolated noise
        cleaned = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
            iterations=self.open_iterations
        )

        # Fill small holes and connect nearby regions
        cleaned = cv2.morphologyEx(
            cleaned,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=self.close_iterations
        )

        self.publish_image(cleaned, 'mono8')

        self.display('4 - Cleaned Mask', cleaned)


# ============================================================
# 5. LANE OFFSET NODE
# ============================================================

class LaneOffsetNode(Node):

    def __init__(self):
        super().__init__('lane_offset_node')

        self.bridge = CvBridge()

        self.show_image = self.declare_parameter(
            'show_image', False
        ).value

        self.min_pixels = self.declare_parameter(
            'min_pixels', 100
        ).value

        self.mask_sub = self.create_subscription(
            Image,
            '/vision/clean_mask',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.offset_pub = self.create_publisher(
            Float32,
            '/vision/lane_offset',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/vision/lane_debug',
            10
        )

        self.get_logger().info('Lane offset node started')

    def image_callback(self, msg):

        mask = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='mono8'
        )

        height, width = mask.shape

        # Calculate image moments from the red mask.
        # No contour detection is used.
        M = cv2.moments(mask, binaryImage=True)

        offset_msg = Float32()

        debug = cv2.cvtColor(
            mask,
            cv2.COLOR_GRAY2BGR
        )

        image_center = width // 2

        cv2.line(
            debug,
            (image_center, 0),
            (image_center, height),
            (255, 0, 0),
            2
        )

        # Check whether enough red pixels were detected
        if M['m00'] >= self.min_pixels:

            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Positive = lane is right of image center
            offset = float(cx - image_center)

            offset_msg.data = offset

            cv2.circle(
                debug,
                (cx, cy),
                8,
                (0, 255, 0),
                -1
            )

            cv2.line(
                debug,
                (image_center, cy),
                (cx, cy),
                (0, 0, 255),
                2
            )

        else:
            # NaN indicates that no valid lane was detected
            offset_msg.data = float('nan')

        self.offset_pub.publish(offset_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(
            debug,
            encoding='bgr8'
        )

        self.debug_pub.publish(debug_msg)

        if self.show_image:
            cv2.imshow('5 - Lane Offset', debug)
            cv2.waitKey(1)


# ============================================================
# 6. STEERING CONTROLLER NODE
# ============================================================

class SteeringNode(Node):

    def __init__(self):
        super().__init__('steering_node')

        self.Kp = self.declare_parameter(
            'kp', 0.005
        ).value

        self.linear_speed = self.declare_parameter(
            'linear_speed', 0.1
        ).value

        self.max_angular = self.declare_parameter(
            'max_angular', 1.5
        ).value

        self.timeout = self.declare_parameter(
            'timeout', 0.5
        ).value

        self.last_offset = None
        self.last_update = None

        self.offset_sub = self.create_subscription(
            Float32,
            '/vision/lane_offset',
            self.offset_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # Publish commands at 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.control_callback
        )

        self.get_logger().info('Steering node started')

    def offset_callback(self, msg):

        if not math.isfinite(msg.data):
            self.last_offset = None
            return

        self.last_offset = msg.data
        self.last_update = time.monotonic()

    def control_callback(self):

        cmd = TwistStamped()

        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'

        # Stop if no valid measurement or measurements are stale
        if (
            self.last_offset is None
            or self.last_update is None
            or time.monotonic() - self.last_update > self.timeout
        ):
            cmd.twist.linear.x = 0.0
            cmd.twist.angular.z = 0.0

        else:

            error = self.last_offset

            angular = -self.Kp * error

            # Limit steering output
            angular = max(
                -self.max_angular,
                min(self.max_angular, angular)
            )

            cmd.twist.linear.x = self.linear_speed
            cmd.twist.angular.z = angular

        self.cmd_pub.publish(cmd)


# ============================================================
# ROS 2 EXECUTABLE ENTRY POINTS
# ============================================================

def run_node(node_class):

    rclpy.init()

    node = node_class()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        # Ensure the robot receives a stop command
        if isinstance(node, SteeringNode):
            stop = TwistStamped()
            stop.header.stamp = node.get_clock().now().to_msg()
            stop.header.frame_id = 'base_link'
            node.cmd_pub.publish(stop)

        node.destroy_node()

        cv2.destroyAllWindows()

        rclpy.shutdown()


def crop_main():
    run_node(CropNode)


def blur_main():
    run_node(BlurNode)


def threshold_main():
    run_node(ThresholdNode)


def cleanup_main():
    run_node(CleanupNode)


def offset_main():
    run_node(LaneOffsetNode)


def steering_main():
    run_node(SteeringNode)

def webcam_main():
    run_node(WebcamNode)