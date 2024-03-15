#!/usr/bin/env python3

from multiple_cameras.srv import ModifyCameraSettings    
from multiple_cameras.msg import MultipleCameraStatus                                                        
from sensor_msgs.msg import CompressedImage

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
import threading 
import time

class USBCamera:
    def __init__(self, dev_id: int):
        self.camera_settings = {
            "dev_id": dev_id,
            "height": 720,
            "width": 1280,
            "raw_height": 720,
            "raw_width": 1280,
            "hardware_fps": 30,
            "supported_fps": [],
            "hardware_supports_fps_change": False,
            "running": False,
            "resize": False,
            "enable_publishing": False,
        }

        self.cap = cv2.VideoCapture(self.camera_settings["dev_id"]*2)  # USB cameras hold two dev/video slots
        self.latest_image = None

        self.camera_read_thread = threading.Thread(target=self.camera_read_loop, daemon=True)
        self.read_lock = threading.Lock()

    def initialise_camera(self, config: dict):
        if self.cap.isOpened():
            fps_set = False
            ret, self.latest_image = self.cap.read()
            
            if ret:
                im_shape = self.latest_image.shape
                self.camera_settings["height"] = config["height"]
                self.camera_settings["width"] = config["width"]
                self.camera_settings["raw_height"] = im_shape[0]
                self.camera_settings["raw_width"] = im_shape[1]
                self.camera_settings["hardware_fps"] = self.cap.get(cv2.CAP_PROP_FPS)
                self.camera_settings["enable_publishing"] = config["enable_publishing"]

                if self.camera_settings["height"] != im_shape[0] or self.camera_settings["width"] != im_shape[1]:
                    self.camera_settings["resize"] = True
                else:
                    self.camera_settings["resize"] = False

                if config["hardware_supports_fps_change"]:  # Not all USB cameras do
                    self.camera_settings["supported_fps"] = config["supported_fps"]

                    if self.camera_settings["hardware_fps"] != config["fps"]:
                         for supported_fps in self.camera_settings["supported_fps"]:
                            if supported_fps >= config["fps"] and not fps_set:
                                self.cap.set(cv2.CAP_PROP_FPS, float(supported_fps))
                                self.camera_settings["hardware_fps"] = supported_fps
                                fps_set = True 
                    
                    if not fps_set:
                        self.cap.set(cv2.CAP_PROP_FPS, float(config["supported_fps"][-1]))
                        self.camera_settings["hardware_fps"] = config["supported_fps"][-1]
            
                self.camera_settings["running"] = True
                
        return self.camera_settings["running"] 

    def get_some(self):
        self.camera_read_thread.start()

    def camera_read_loop(self):
        while self.camera_settings["running"]:
            if self.camera_settings["enable_publishing"]:
                ret, frame = self.cap.read()

                if ret:
                    with self.read_lock:
                        if self.camera_settings["resize"]:
                            self.latest_image = cv2.resize(frame, (self.camera_settings["width"], self.camera_settings["height"]), interpolation=cv2.INTER_CUBIC)
                        else:
                            self.latest_image = frame.copy()
            else:
                time.sleep(0.05)
    
    def get_latest_image(self):
        with self.read_lock:
            if self.camera_settings["enable_publishing"]:
                valid = False if self.latest_image is None else True
                latest_image = self.latest_image.copy() if valid else None
            else:
                valid, latest_image = False, None

        return valid, latest_image

    def configure_camera(self):
        if self.camera_settings["height"] != self.camera_settings["raw_height"] or self.camera_settings["width"] != self.camera_settings["raw_width"]:
            self.camera_settings["resize"] = True
        else:
            self.camera_settings["resize"] = False

        return True


class MultipleCameraPublisher(Node):

    def __init__(self):
        super().__init__('multiple_camera_node')
        self.modify_camera_service = self.create_service(ModifyCameraSettings, 'multiple_camera/modify_camera_settings', self.modify_camera_settings_callback)       
        
        self.bridge = CvBridge()

        self.camera0 = USBCamera(0)
        self.camera0_fps = 10
        self.camera0_image_shape = [480, 640]
        self.camera0_exists = False
        self.get_logger().info(f"Initialising camera 0")

        if self.camera0.initialise_camera(config={
            "height": self.camera0_image_shape[0],
            "width": self.camera0_image_shape[1],
            "fps": self.camera0_fps,
            "supported_fps": [],
            "hardware_supports_fps_change": False,
            "enable_publishing": True,
            }):
           
            self.camera0_publisher = self.create_publisher(CompressedImage, 'camera0/image/compressed', 10)
            self.camera0_timer = self.create_timer(1/self.camera0_fps, self.camera0_timer_callback)
            self.get_logger().info("Camera 0 is ready....")
   
        else:
            self.get_logger().info("Camera 0 is dead....")

        self.camera1 = USBCamera(1)
        self.camera1_fps = 10
        self.camera1_image_shape = [480, 640]
        self.camera1_exists = False
        self.get_logger().info(f"Initialising camera 1")
        
        if self.camera1.initialise_camera(config={
            "height": self.camera1_image_shape[0],
            "width": self.camera1_image_shape[1],
            "fps": self.camera1_fps,
            "supported_fps": [],
            "hardware_supports_fps_change": False,
            "enable_publishing": True,
            }):
            
            self.camera1_publisher = self.create_publisher(CompressedImage, 'camera1/image/compressed', 10)
            self.camera1_timer = self.create_timer(1/self.camera1_fps, self.camera1_timer_callback)
            self.get_logger().info("Camera 1 is ready....")
        
        else:
            self.get_logger().info("Camera 1 is dead....")

        self.camera2 = USBCamera(2)
        self.camera2_fps = 10
        self.camera2_image_shape = [480, 640]
        self.camera2_exists = False
        self.get_logger().info(f"Initialising camera 2")

        if self.camera2.initialise_camera(config={
            "height": self.camera2_image_shape[0],
            "width": self.camera2_image_shape[1],
            "fps": self.camera2_fps,
            "supported_fps": [],
            "hardware_supports_fps_change": False,
            "enable_publishing": True,
            }):
                
            self.camera2_publisher = self.create_publisher(CompressedImage, 'camera2/image/compressed', 10)
            self.camera2_timer = self.create_timer(1/self.camera2_fps, self.camera2_timer_callback)
            self.get_logger().info("Camera 2 is ready....")
            
        else:
            self.get_logger().info("Camera 2 is dead....")

        self.camera3 = USBCamera(3)
        self.camera3_fps = 10
        self.camera3_image_shape = [480, 640]
        self.camera3_exists = False
        self.get_logger().info(f"Initialising camera 3")

        if self.camera3.initialise_camera(config={
            "height": self.camera3_image_shape[0],
            "width": self.camera3_image_shape[1],
            "fps": self.camera2_fps,
            "supported_fps": [],
            "hardware_supports_fps_change": False,
            "enable_publishing": True,
            }):
                
            self.camera3_publisher = self.create_publisher(CompressedImage, 'camera3/image/compressed', 10)
            self.camera3_timer = self.create_timer(1/self.camera3_fps, self.camera3_timer_callback)
            self.get_logger().info("Camera 3 is ready....")
            
        else:
            self.get_logger().info("Camera 3 is dead....")

        if self.camera0.camera_settings["running"]:
            self.camera0.get_some()
            self.camera0_exists = True
        else:
            del self.camera0

        if self.camera1.camera_settings["running"]:
            self.camera1.get_some()
            self.camera1_exists = True
        else:
            del self.camera1

        if self.camera2.camera_settings["running"]:
            self.camera2.get_some()
            self.camera2_exists = True
        else:
            del self.camera2
        
        if self.camera3.camera_settings["running"]:
            self.camera3.get_some()
            self.camera3_exists = True
        else:
            del self.camera3

        self.status_publisher = self.create_publisher(MultipleCameraStatus, 'multiple_camera/status', 10)
        self.status_timer = self.create_timer(1.0, self.status_timer_callback)

    def modify_camera_settings_callback(self, request, response): 
        """
        int64 camera_id
        bool enable_publishing
        int64 height
        int64 width
        int64 fps
        ---
        bool success
        """
        if request.camera_id == 0:
            if self.camera0_exists:
                self.camera0.camera_settings["enable_publishing"] = request.enable_publishing
                self.camera0.camera_settings["height"] = request.height
                self.camera0.camera_settings["width"] = request.width

                self.get_logger().info(f"Camera 0 (h {request.height}, w {request.width}), publish {request.enable_publishing}")

                if self.camera0_fps != request.fps and 0 < request.fps <= 30:
                    self.camera0_fps = request.fps
                    self.camera0_timer.timer_period_ns = 1/self.camera0_fps * 1e9
                    self.get_logger().info(f"Camera 0 set to {self.camera0_fps} FPS")

                response.success = self.camera0.configure_camera()
            else:
                response.success = False

        elif request.camera_id == 1:
            if self.camera1_exists:
                self.camera1.camera_settings["enable_publishing"] = request.enable_publishing
                self.camera1.camera_settings["height"] = request.height
                self.camera1.camera_settings["width"] = request.width
                
                self.get_logger().info(f"Camera 1 (h {request.height}, w {request.width}), publish {request.enable_publishing}")
                
                if self.camera1_fps != request.fps and 0 < request.fps <= 30:
                    self.camera1_fps = request.fps
                    self.camera1_timer.timer_period_ns = 1/self.camera1_fps * 1e9
                    self.get_logger().info(f"Camera 1 set to {self.camera1_fps} FPS")

                response.success = self.camera1.configure_camera()
            else:
                response.success = False

        elif request.camera_id == 2:
            if self.camera2_exists:
                self.camera2.camera_settings["enable_publishing"] = request.enable_publishing
                self.camera2.camera_settings["height"] = request.height
                self.camera2.camera_settings["width"] = request.width

                self.get_logger().info(f"Camera 2 (h {request.height}, w {request.width}), publish {request.enable_publishing}")

                if self.camera2_fps != request.fps and 0 < request.fps <= 30:
                    self.camera2_fps = request.fps
                    self.camera2_timer.timer_period_ns = 1/self.camera2_fps * 1e9
                    self.get_logger().info(f"Camera 2 set to {self.camera2_fps} FPS")

                response.success = self.camera2.configure_camera()
            else:
                response.success = False

        elif request.camera_id == 3:
            if self.camera3_exists:
                self.camera3.camera_settings["enable_publishing"] = request.enable_publishing
                self.camera3.camera_settings["height"] = request.height
                self.camera3.camera_settings["width"] = request.width

                self.get_logger().info(f"Camera 3 (h {request.height}, w {request.width}), publish {request.enable_publishing}")

                if self.camera3_fps != request.fps and 0 < request.fps <= 30:
                    self.camera3_fps = request.fps
                    self.camera3_timer.timer_period_ns = 1/self.camera3_fps * 1e9
                    self.get_logger().info(f"Camera 3 set to {self.camera3_fps} FPS")
                
                response.success = self.camera3.configure_camera()
            else:
                response.success = False
        else:
            response.success = False
        
        return response
    
    def status_timer_callback(self):
        sts_msg = MultipleCameraStatus()

        if self.camera0_exists:
            sts_msg.camera_0.opened = self.camera0.camera_settings["running"]
            sts_msg.camera_0.enable_publishing = self.camera0.camera_settings["enable_publishing"]
            sts_msg.camera_0.height = self.camera0.camera_settings["height"]
            sts_msg.camera_0.width = self.camera0.camera_settings["width"]
            sts_msg.camera_0.fps = self.camera0_fps

        if self.camera1_exists:
            sts_msg.camera_1.opened = self.camera1.camera_settings["running"]
            sts_msg.camera_1.enable_publishing = self.camera1.camera_settings["enable_publishing"]
            sts_msg.camera_1.height = self.camera1.camera_settings["height"]
            sts_msg.camera_1.width = self.camera1.camera_settings["width"]
            sts_msg.camera_1.fps = self.camera1_fps
        
        if self.camera2_exists:
            sts_msg.camera_2.opened = self.camera2.camera_settings["running"]
            sts_msg.camera_2.enable_publishing = self.camera2.camera_settings["enable_publishing"]
            sts_msg.camera_2.height = self.camera2.camera_settings["height"]
            sts_msg.camera_2.width = self.camera2.camera_settings["width"]
            sts_msg.camera_2.fps = self.camera2_fps

        if self.camera3_exists:
            sts_msg.camera_3.opened = self.camera3.camera_settings["running"]
            sts_msg.camera_3.enable_publishing = self.camera3.camera_settings["enable_publishing"]
            sts_msg.camera_3.height = self.camera3.camera_settings["height"]
            sts_msg.camera_3.width = self.camera3.camera_settings["width"]
            sts_msg.camera_3.fps = self.camera3_fps

        self.status_publisher.publish(sts_msg)

    def camera0_timer_callback(self):
        valid, frame = self.camera0.get_latest_image()
        
        if valid:
            img_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.camera0_publisher.publish(img_msg)

    def camera1_timer_callback(self):
        valid, frame = self.camera1.get_latest_image()
            
        if valid:              
            img_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.camera1_publisher.publish(img_msg)

    def camera2_timer_callback(self):
        valid, frame = self.camera2.get_latest_image()
            
        if valid:              
            img_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.camera2_publisher.publish(img_msg)

    def camera3_timer_callback(self):
        valid, frame = self.camera3.get_latest_image()
            
        if valid:              
            img_msg = self.bridge.cv2_to_compressed_imgmsg(frame)
            self.camera3_publisher.publish(img_msg)            


def main(args=None):
    rclpy.init(args=args)

    multiple_camera_publisher = MultipleCameraPublisher()
    
    rclpy.spin(multiple_camera_publisher)

    rclpy.shutdown()

if __name__ == '__main__':
    main()