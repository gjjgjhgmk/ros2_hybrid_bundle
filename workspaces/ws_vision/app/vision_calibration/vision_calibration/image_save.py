#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import threading


class ImageSaver(Node):
    def __init__(self):
        super().__init__("image_saver")

        # 参数
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("save_path", "/workspace/calibration_images")
        self.declare_parameter("save_key", "s")

        self.rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        self.save_path = self.get_parameter("save_path").get_parameter_value().string_value
        self.save_key = self.get_parameter("save_key").get_parameter_value().string_value

        # 创建保存目录
        self.save_path = os.path.abspath(self.save_path)
        os.makedirs(self.save_path, exist_ok=True)
        self.get_logger().info(f"Will save images to: {self.save_path}")

        self.bridge = CvBridge()

        self.latest_rgb = None
        self.rgb_lock = threading.Lock()

        # 订阅RGB图像话题
        self.rgb_sub = self.create_subscription(Image, self.rgb_topic, self.rgb_callback, 10)

        # 用 timer 替代 keyboard 监听线程
        self.timer = self.create_timer(0.1, self.display_and_listen)

        self.get_logger().info("Image Saver initialized")
        self.get_logger().info(f'Press "{self.save_key}" in the image window to save RGB image')

    def rgb_callback(self, msg):
        try:
            with self.rgb_lock:
                self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting RGB image: {str(e)}")

    def save_image(self):
        if self.latest_rgb is None:
            self.get_logger().warn("No RGB image available to save")
            return

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        try:
            with self.rgb_lock:
                image_path = os.path.join(self.save_path, f"rgb_{timestamp}.png")
                cv2.imwrite(image_path, self.latest_rgb)

            self.get_logger().info(f"Image saved: {image_path}")
        except Exception as e:
            self.get_logger().error(f"Error saving image: {str(e)}")

    def display_and_listen(self):
        if self.latest_rgb is not None:
            with self.rgb_lock:
                cv2.imshow("RGB Image (press 's' to save)", self.latest_rgb)
            key = cv2.waitKey(1) & 0xFF
            if key == ord(self.save_key):
                self.save_image()
            elif key == ord("q"):  # 添加退出功能
                self.get_logger().info("Quit requested")
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageSaver()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
