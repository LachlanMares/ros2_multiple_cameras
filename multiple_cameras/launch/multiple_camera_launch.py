from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="multiple_cameras",
            executable="multiple_camera_publisher",
            name="multiple_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"modify_camera_settings_service": "multiple_camera/modify_camera_settings",
                 "camera0_compressed_topic": "camera0/image/compressed", 
                 "camera0_dev_id": 0, 
                 "camera0_supported_fps": [5, 10, 20, 30], 
                 "camera1_compressed_topic": "camera1/image/compressed",
                 "camera1_dev_id": 2, 
                 "camera1_supported_fps": [5, 10, 20, 30], 
                 "camera2_compressed_topic": "camera2/image/compressed",
                 "camera2_dev_id": 4, 
                 "camera2_supported_fps": [5, 10, 20, 30], 
                 "camera3_compressed_topic": "camera3/image/compressed",
                 "camera3_dev_id": 6, 
                 "camera3_supported_fps": [5, 10, 20, 30], 
                 "multiple_camera_status_topic": "multiple_camera/status",    
                 }
            ]
        )
    ])