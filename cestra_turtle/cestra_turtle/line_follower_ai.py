
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped

from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge

import cv2
import numpy as np
import torch
import torch.nn.functional as F 

from PIL import Image as PILImage

from transformers import (
    CLIPSegProcessor,
    CLIPSegForImageSegmentation
)


class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower')

        # --------------------------------------------------
        # 1. Initialize AI model
        # --------------------------------------------------

        self.device = (
            "cuda" if torch.cuda.is_available() else "cpu"
        )

        self.get_logger().info(
            f"Loading CLIPSeg model on {self.device}..."
        )

        model_name = "CIDAS/clipseg-rd64-refined"

        self.processor = CLIPSegProcessor.from_pretrained(
            model_name
        )

        self.model = CLIPSegForImageSegmentation.from_pretrained(
            model_name
        )

        self.model.to(self.device)
        self.model.eval()

        self.prompt = "red tape on the floor"

        self.get_logger().info("CLIPSeg model loaded")

        # --------------------------------------------------
        # 2. ROS initialization
        # --------------------------------------------------

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            10
        )

        # --------------------------------------------------
        # 3. Controller parameters
        # --------------------------------------------------

        self.Kp = 0.005

        self.linear_speed = 0.08

        self.max_angular_speed = 1.0

        # AI segmentation threshold
        self.segmentation_threshold = 0.5

        # Ignore very small detections
        self.min_contour_area = 50

        self.get_logger().info(
            "AI line follower started"
        )

    # ------------------------------------------------------
    # AI segmentation
    # ------------------------------------------------------

    def segment_line(self, frame):

        height, width, _ = frame.shape

        # OpenCV BGR -> PIL RGB
        rgb = cv2.cvtColor(
            frame,
            cv2.COLOR_BGR2RGB
        )

        pil_image = PILImage.fromarray(rgb)

        # Prepare image and text prompt
        inputs = self.processor(
            text=[self.prompt],
            images=[pil_image],
            padding=True,
            return_tensors="pt"
        )

        inputs = {
            key: value.to(self.device)
            for key, value in inputs.items()
        }

        # Run neural network inference
        with torch.inference_mode():

            outputs = self.model(**inputs)

            # Convert model logits to probabilities
            probabilities = torch.sigmoid(
                outputs.logits
            )

            # Resize segmentation output to camera resolution
            probabilities = F.interpolate(
                probabilities.unsqueeze(1),
                size=(height, width),
                mode="bilinear",
                align_corners=False
            )

        # Convert probability map into binary mask
        mask = (
            probabilities[0, 0].cpu().numpy()
            > self.segmentation_threshold
        ).astype(np.uint8) * 255

        return mask

    # ------------------------------------------------------
    # Camera callback
    # ------------------------------------------------------

    def image_callback(self, msg):

        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        height, width, _ = frame.shape

        # --------------------------------------------------
        # 4. AI perception
        # --------------------------------------------------

        mask = self.segment_line(frame)

        # Only inspect bottom 40% of image
        roi_start = int(height * 0.6)

        roi = mask[roi_start:height, :]

        # --------------------------------------------------
        # 5. Find detected line
        # --------------------------------------------------

        contours, _ = cv2.findContours(
            roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        cmd = TwistStamped()

        cmd.header.stamp = (
            self.get_clock().now().to_msg()
        )

        cmd.header.frame_id = 'base_link'

        if contours:

            largest_contour = max(
                contours,
                key=cv2.contourArea
            )

            area = cv2.contourArea(
                largest_contour
            )

            if area > self.min_contour_area:

                M = cv2.moments(
                    largest_contour
                )

                if M['m00'] > 0:

                    # Center of detected tape
                    cx = int(
                        M['m10'] / M['m00']
                    )

                    cy = int(
                        M['m01'] / M['m00']
                    ) + roi_start

                    # --------------------------------------
                    # 6. Proportional steering
                    # --------------------------------------

                    image_center = width // 2

                    error = cx - image_center

                    angular_z = -self.Kp * error

                    angular_z = np.clip(
                        angular_z,
                        -self.max_angular_speed,
                        self.max_angular_speed
                    )

                    cmd.twist.linear.x = (
                        self.linear_speed
                    )

                    cmd.twist.angular.z = float(
                        angular_z
                    )

                    # Visualization
                    cv2.circle(
                        frame,
                        (cx, cy),
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
                    self.stop_robot(cmd)

            else:
                self.stop_robot(cmd)

        else:
            self.stop_robot(cmd)

        # --------------------------------------------------
        # 7. Publish velocity
        # --------------------------------------------------

        self.cmd_pub.publish(cmd)

        # --------------------------------------------------
        # 8. Visualization
        # --------------------------------------------------

        cv2.imshow("AI Line Follower", frame)

        cv2.imshow("CLIPSeg Mask", mask)

        cv2.waitKey(1)

    # ------------------------------------------------------
    # Stop robot
    # ------------------------------------------------------

    def stop_robot(self, cmd):

        cmd.twist.linear.x = 0.0
        cmd.twist.angular.z = 0.0


def main(args=None):

    rclpy.init(args=args)

    node = LineFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    # Final stop command
    stop = TwistStamped()

    stop.header.stamp = (
        node.get_clock().now().to_msg()
    )

    stop.header.frame_id = 'base_link'

    node.cmd_pub.publish(stop)

    node.destroy_node()

    cv2.destroyAllWindows()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
