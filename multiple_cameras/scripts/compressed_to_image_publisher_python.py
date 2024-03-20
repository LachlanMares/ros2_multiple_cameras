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

        self.camera0_name = self.string_parameter("camera0_name", " ")

        if self.camera0_name != " ":
            self.camera0_subscriber = self.create_subscription(CompressedImage, f'{self.camera0_name}/image/compressed', self.camera0_compressed_image_callback, 10)
            self.camera0_publisher = self.create_publisher(Image, f'{self.camera0_name}/image/decompressed', 10)
        else:
            self.camera0_name = "0"
            self.camera0_subscriber = self.create_subscription(CompressedImage, 'camera0/image/compressed', self.camera0_compressed_image_callback, 10)
            self.camera0_publisher = self.create_publisher(Image, 'camera0/image/decompressed', 10)

        self.camera1_name = self.string_parameter("camera1_name", " ")

        if self.camera1_name != " ":
            self.camera1_subscriber = self.create_subscription(CompressedImage, f'{self.camera1_name}/image/compressed', self.camera1_compressed_image_callback, 10)
            self.camera1_publisher = self.create_publisher(Image, f'{self.camera1_name}/image/decompressed', 10)
        else:
            self.camera1_name = "1"
            self.camera1_subscriber = self.create_subscription(CompressedImage, 'camera1/image/compressed', self.camera1_compressed_image_callback, 10)
            self.camera1_publisher = self.create_publisher(Image, 'camera1/image/decompressed', 10)

        self.camera2_name = self.string_parameter("camera2_name", " ")

        if self.camera2_name != " ":
            self.camera2_subscriber = self.create_subscription(CompressedImage, f'{self.camera2_name}/image/compressed', self.camera2_compressed_image_callback, 10)
            self.camera2_publisher = self.create_publisher(Image, f'{self.camera2_name}/image/decompressed', 10)
        else:
            self.camera2_name = "2"
            self.camera2_subscriber = self.create_subscription(CompressedImage, 'camera2/image/compressed', self.camera2_compressed_image_callback, 10)
            self.camera2_publisher = self.create_publisher(Image, 'camera2/image/decompressed', 10)

        self.camera3_name = self.string_parameter("camera3_name", " ")

        if self.camera3_name != " ":
            self.camera3_subscriber = self.create_subscription(CompressedImage, f'{self.camera3_name}/image/compressed', self.camera3_compressed_image_callback, 10)
            self.camera3_publisher = self.create_publisher(Image, f'{self.camera3_name}/image/decompressed', 10)
        else:
            self.camera3_name = "3"
            self.camera3_subscriber = self.create_subscription(CompressedImage, 'camera3/image/compressed', self.camera3_compressed_image_callback, 10)
            self.camera3_publisher = self.create_publisher(Image, 'camera3/image/decompressed', 10)
        
        self.camera4_name = self.string_parameter("camera4_name", " ")

        if self.camera4_name != " ":
            self.camera4_subscriber = self.create_subscription(CompressedImage, f'{self.camera4_name}/image/compressed', self.camera4_compressed_image_callback, 10)
            self.camera4_publisher = self.create_publisher(Image, f'{self.camera4_name}/image/decompressed', 10)
        else:
            self.camera4_name = "4"
            self.camera4_subscriber = self.create_subscription(CompressedImage, 'camera4/image/compressed', self.camera4_compressed_image_callback, 10)
            self.camera4_publisher = self.create_publisher(Image, 'camera4/image/decompressed', 10)

        self.camera5_name = self.string_parameter("camera5_name", " ")

        if self.camera5_name != " ":
            self.camera5_subscriber = self.create_subscription(CompressedImage, f'{self.camera5_name}/image/compressed', self.camera5_compressed_image_callback, 10)
            self.camera5_publisher = self.create_publisher(Image, f'{self.camera5_name}/image/decompressed', 10)
        else:
            self.camera5_name = "5"
            self.camera5_subscriber = self.create_subscription(CompressedImage, 'camera5/image/compressed', self.camera5_compressed_image_callback, 10)
            self.camera5_publisher = self.create_publisher(Image, 'camera5/image/decompressed', 10)

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

def main(args=None):
    rclpy.init(args=args)

    compressed_image_subscriber = CompressedImageSubscriber()
    
    rclpy.spin(compressed_image_subscriber)

    compressed_image_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()