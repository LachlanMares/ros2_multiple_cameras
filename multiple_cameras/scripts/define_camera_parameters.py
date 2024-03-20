#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from multiple_cameras.srv import CameraId  

class CameraParameterNode(Node):
    def __init__(self):
        super().__init__('camera_parameter_node')
        
        # Define and Set ROS parameters
        self.params_dict = {self.string_parameter("camera0/name", "camera0"): self.string_parameter('camera0/dev_id', '/dev/video0'),
                            self.string_parameter("camera1/name", "camera1"): self.string_parameter('camera1/dev_id', '/dev/video2'),
                            self.string_parameter("camera2/name", "camera2"): self.string_parameter('camera2/dev_id', '/dev/video4'),
                            self.string_parameter("camera3/name", "camera3"): self.string_parameter('camera3/dev_id', '/dev/video6'),
                            self.string_parameter("camera4/name", "camera4"): self.string_parameter('camera4/dev_id', '/dev/video8'),
                            self.string_parameter("camera5/name", "camera5"): self.string_parameter('camera5/dev_id', '/dev/video10'),
                            self.string_parameter("camera6/name", "camera6"): self.string_parameter('camera6/dev_id', '/dev/video12'),
                            self.string_parameter("camera7/name", "camera7"): self.string_parameter('camera7/dev_id', '/dev/video14'),
                            }

        self.get_camera_id_service = self.create_service(CameraId, 
                                                         self.string_parameter("camera_id_service", "multiple_camera/get_camera_id"), 
                                                         self.get_camera_id_callback)    
    
    def int_parameter(self, param_name: str, default_value: int):
        self.declare_parameter(param_name, default_value)
        
        try:
            updated_param = self.get_parameter(param_name).get_parameter_value().integer_value
     
        except Exception as e:
            self.get_logger().error(f"Unable to read parameter: {param_name}")
            updated_param = default_value
        
        return updated_param    
    
    def string_parameter(self, param_name: str, default_value: str):
        self.declare_parameter(param_name, default_value)
        
        try:
            updated_param = self.get_parameter(param_name).get_parameter_value().string_value
        
        except Exception as e:
            self.get_logger().error(f"Unable to read parameter: {param_name}")
            updated_param = default_value
        
        return updated_param
    
    def get_camera_id_callback(self, request, response):
        """
        string camera_name
        ---
        string dev_id
        """       
        if request.camera_name in self.params_dict.keys():
            response.dev_id = self.params_dict[request.camera_name]
        else:
            response.dev_id = -1

        return response


def main(args=None):
    rclpy.init(args=args)
    camera_parameter_node = CameraParameterNode()
    rclpy.spin(camera_parameter_node)
    camera_parameter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()