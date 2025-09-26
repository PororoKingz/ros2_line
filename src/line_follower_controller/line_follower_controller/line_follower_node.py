#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from line_follower_controller.image_processing import (
    get_roi_portion,
    process_image,
    scale_contour_from_roi_to_frame,
    calculate_error,
    draw_contour_and_corners,
    draw_expected_path_dots,
    scale_error,
    adjust_linear_velocity
)

# Konfigurasi
MAX_SPEED = 1.0
MIN_SPEED = 0.1
SEARCH_TURN_SPEED = 0.4  # seberapa cepat memutar saat garis hilang
SEARCH_LINEAR_SPEED = 0.0
LOST_TURN_ANGLE = 0.5  # kecepatan rotasi saat hilang

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.get_logger().info('line_follower node created')

        self.bridge = CvBridge()
        self.prev_angle = 0.0
        self.lost_counter = 0  # untuk mendeteksi berapa lama garis hilang

        self.image_frame_subscriber = self.create_subscription(
            Image, '/camera_sensor/image_raw',
            self.image_frame_callback, 10)
        self.processed_image_publisher = self.create_publisher(
            Image, '/processed_frame', 10)
        self.cmd_vel_publisher = self.create_publisher(
            Twist, '/cmd_vel', 10)

    def image_frame_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        roi, roi_x, roi_y = get_roi_portion(frame)
        binary_image, contour = process_image(roi)

        if contour is not None:
            contour = scale_contour_from_roi_to_frame(contour, roi_x, roi_y)
            roi_width = roi.shape[1]
            angle, frame = calculate_error(
                frame, contour, True,
                scale_fn=scale_error(roi_width),
                error_text="Angle")
            frame = draw_contour_and_corners(frame, contour)
            frame = draw_expected_path_dots(frame, True)

            # Kecepatan linear dinamis tergantung error
            speed = adjust_linear_velocity(angle, MIN_SPEED, MAX_SPEED)

            # Simpan sudut terakhir
            self.prev_angle = angle
            self.lost_counter = 0  # reset

            self.control_robot(speed, angle)
        else:
            # Ketika kehilangan garis: putar berdasarkan arah sebelumnya
            self.lost_counter += 1
            direction = -LOST_TURN_ANGLE if self.prev_angle < 0 else LOST_TURN_ANGLE

            self.control_robot(SEARCH_LINEAR_SPEED, direction)
            frame = draw_expected_path_dots(frame, False)

        # Publish hasil proses ke topik
        self.processed_image_publisher.publish(
            self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

    def control_robot(self, linear, angular):
        twist = Twist()
        twist.linear.x = -linear  # arah ke depan negatif
        twist.angular.z = angular
        self.cmd_vel_publisher.publish(twist)
        self.get_logger().info(f"Linear: {linear:.2f}, Angular: {angular:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
