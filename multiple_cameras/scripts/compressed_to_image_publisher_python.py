#!/usr/bin/env python3
                                             
from sensor_msgs.msg import CompressedImage, Image

import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge



class CompressedImageSubscriber(Node):
    def __init__(self):
        super().__init__('multiple_camera_compressed_image_subscriber')

        self.bridge = CvBridge()

        # Subscribers
        self.camera0_subscriber = self.create_subscription(CompressedImage, 'camera0/image/compressed', self.camera0_compressed_image_callback, 10)
        self.camera1_subscriber = self.create_subscription(CompressedImage, 'camera1/image/compressed', self.camera1_compressed_image_callback, 10)
        self.camera2_subscriber = self.create_subscription(CompressedImage, 'camera2/image/compressed', self.camera2_compressed_image_callback, 10)
        self.camera3_subscriber = self.create_subscription(CompressedImage, 'camera3/image/compressed', self.camera3_compressed_image_callback, 10)

        # Publishers
        self.camera0_publisher = self.create_publisher(Image, 'camera0/image/decompressed', 10)
        self.camera1_publisher = self.create_publisher(Image, 'camera1/image/decompressed', 10)
        self.camera2_publisher = self.create_publisher(Image, 'camera2/image/decompressed', 10)
        self.camera3_publisher = self.create_publisher(Image, 'camera3/image/decompressed', 10)

    def camera0_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera0_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error('Error decoding/publishing camera 0')
 
    def camera1_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera1_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error('Error decoding/publishing camera 1')
    
    def camera2_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera2_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error('Error decoding/publishing camera 2')
    
    def camera3_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera3_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error('Error decoding/publishing camera 3')                        


def main(args=None):
    rclpy.init(args=args)

    compressed_image_subscriber = CompressedImageSubscriber()
    
    rclpy.spin(compressed_image_subscriber)

    compressed_image_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()