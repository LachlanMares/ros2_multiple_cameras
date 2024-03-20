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

        self.camera0_name = self.string_parameter("camera0/name", "camera0")

        if self.camera0_name.lower() != "none":
            if self.camera0_name != "camera0":
                self.camera0_subscriber = self.create_subscription(CompressedImage, f'{self.camera0_name}/image/compressed', self.camera0_compressed_image_callback, 10)
                self.camera0_publisher = self.create_publisher(Image, f'{self.camera0_name}/image/decompressed', 10)
            else:
                self.camera0_subscriber = self.create_subscription(CompressedImage, 'camera0/image/compressed', self.camera0_compressed_image_callback, 10)
                self.camera0_publisher = self.create_publisher(Image, 'camera0/image/decompressed', 10)

        self.camera1_name = self.string_parameter("camera1/name", "camera1")

        if self.camera1_name.lower() != "none":
            if self.camera1_name != "camera1":
                self.camera1_subscriber = self.create_subscription(CompressedImage, f'{self.camera1_name}/image/compressed', self.camera1_compressed_image_callback, 10)
                self.camera1_publisher = self.create_publisher(Image, f'{self.camera1_name}/image/decompressed', 10)
            else:
                self.camera1_subscriber = self.create_subscription(CompressedImage, 'camera1/image/compressed', self.camera1_compressed_image_callback, 10)
                self.camera1_publisher = self.create_publisher(Image, 'camera1/image/decompressed', 10)

        self.camera2_name = self.string_parameter("camera2/name", "camera2")

        if self.camera2_name.lower() != "none":
            if self.camera2_name != "camera2":
                self.camera2_subscriber = self.create_subscription(CompressedImage, f'{self.camera2_name}/image/compressed', self.camera2_compressed_image_callback, 10)
                self.camera2_publisher = self.create_publisher(Image, f'{self.camera2_name}/image/decompressed', 10)
            else:
                self.camera2_subscriber = self.create_subscription(CompressedImage, 'camera2/image/compressed', self.camera2_compressed_image_callback, 10)
                self.camera2_publisher = self.create_publisher(Image, 'camera2/image/decompressed', 10)

        self.camera3_name = self.string_parameter("camera3/name", "camera3")

        if self.camera3_name.lower() != "none":
            if self.camera3_name != "camera3":
                self.camera3_subscriber = self.create_subscription(CompressedImage, f'{self.camera3_name}/image/compressed', self.camera3_compressed_image_callback, 10)
                self.camera3_publisher = self.create_publisher(Image, f'{self.camera3_name}/image/decompressed', 10)
            else:
                self.camera3_subscriber = self.create_subscription(CompressedImage, 'camera3/image/compressed', self.camera3_compressed_image_callback, 10)
                self.camera3_publisher = self.create_publisher(Image, 'camera3/image/decompressed', 10)
            
        self.camera4_name = self.string_parameter("camera4/name", "camera4")

        if self.camera4_name.lower() != "none":
            if self.camera4_name != "camera4":
                self.camera4_subscriber = self.create_subscription(CompressedImage, f'{self.camera4_name}/image/compressed', self.camera4_compressed_image_callback, 10)
                self.camera4_publisher = self.create_publisher(Image, f'{self.camera4_name}/image/decompressed', 10)
            else:
                self.camera4_subscriber = self.create_subscription(CompressedImage, 'camera4/image/compressed', self.camera4_compressed_image_callback, 10)
                self.camera4_publisher = self.create_publisher(Image, 'camera4/image/decompressed', 10)

        self.camera5_name = self.string_parameter("camera5/name", "camera5")

        if self.camera5_name.lower() != "none":
            if self.camera5_name != "camera5":
                self.camera5_subscriber = self.create_subscription(CompressedImage, f'{self.camera5_name}/image/compressed', self.camera5_compressed_image_callback, 10)
                self.camera5_publisher = self.create_publisher(Image, f'{self.camera5_name}/image/decompressed', 10)
            else:
                self.camera5_subscriber = self.create_subscription(CompressedImage, 'camera5/image/compressed', self.camera5_compressed_image_callback, 10)
                self.camera5_publisher = self.create_publisher(Image, 'camera5/image/decompressed', 10)

        self.camera6_name = self.string_parameter("camera6/name", "camera6")

        if self.camera6_name.lower() != "none":
            if self.camera6_name != "camera6":
                self.camera6_subscriber = self.create_subscription(CompressedImage, f'{self.camera6_name}/image/compressed', self.camera6_compressed_image_callback, 10)
                self.camera6_publisher = self.create_publisher(Image, f'{self.camera6_name}/image/decompressed', 10)
            else:
                self.camera6_subscriber = self.create_subscription(CompressedImage, 'camera6/image/compressed', self.camera6_compressed_image_callback, 10)
                self.camera6_publisher = self.create_publisher(Image, 'camera6/image/decompressed', 10)

        self.camera7_name = self.string_parameter("camera7/name", "camera7")

        if self.camera7_name.lower() != "none":
            if self.camera7_name != "camera7":
                self.camera7_subscriber = self.create_subscription(CompressedImage, f'{self.camera7_name}/image/compressed', self.camera7_compressed_image_callback, 10)
                self.camera7_publisher = self.create_publisher(Image, f'{self.camera7_name}/image/decompressed', 10)
            else:
                self.camera7_subscriber = self.create_subscription(CompressedImage, 'camera7/image/compressed', self.camera7_compressed_image_callback, 10)
                self.camera7_publisher = self.create_publisher(Image, 'camera7/image/decompressed', 10)

    def string_parameter(self, param_name: str, default_value: str):
        self.declare_parameter(param_name, default_value)
        
        try:
            updated_param = self.get_parameter(param_name).get_parameter_value().string_value
        
        except Exception as e:
            self.get_logger().error(f"Unable to read parameter: {param_name}")
            updated_param = default_value
        
        return updated_param
        
    def camera0_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera0_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error decoding/publishing camera {self.camera0_name}')
 
    def camera1_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera1_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error decoding/publishing camera {self.camera1_name}')
    
    def camera2_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera2_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error decoding/publishing camera {self.camera2_name}')
    
    def camera3_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera3_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error decoding/publishing camera {self.camera3_name}')                        

    def camera4_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera4_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error decoding/publishing camera {self.camera4_name}')
    
    def camera5_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera5_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error decoding/publishing camera {self.camera5_name}')  

    def camera6_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera6_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error decoding/publishing camera {self.camera6_name}')  

    def camera7_compressed_image_callback(self, msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
            self.camera7_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

        except Exception as e:
            self.get_logger().error(f'Error decoding/publishing camera {self.camera7_name}')  


def main(args=None):
    rclpy.init(args=args)
    compressed_image_subscriber = CompressedImageSubscriber()
    rclpy.spin(compressed_image_subscriber)
    compressed_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()