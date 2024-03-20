from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="multiple_cameras",
            executable="multiple_camera_publisher_python.py",
            name="multiple_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"modify_camera_settings_service": "multiple_camera/modify_camera_settings",
                 
                 "camera0/compressed_topic": "camera0/image/compressed", 
                 "camera0/dev_id": 0, 
                 "camera0/fps": 10, 
                 "camera0/height": 720, 
                 "camera0/width": 1080, 
                 
                 "camera1/compressed_topic": "camera1/image/compressed", 
                 "camera1/dev_id": 1, 
                 "camera1/fps": 10, 
                 "camera1/height": 720, 
                 "camera1/width": 1080, 
                 
                 "camera2/compressed_topic": "camera2/image/compressed", 
                 "camera2/dev_id": 2, 
                 "camera2/fps": 10, 
                 "camera2/height": 720, 
                 "camera2/width": 1080, 
                
                 "camera3/compressed_topic": "camera3/image/compressed", 
                 "camera3/dev_id": 3, 
                 "camera3/fps": 10, 
                 "camera3/height": 720, 
                 "camera3/width": 1080, 

                 "camera4/compressed_topic": "camera4/image/compressed", 
                 "camera4/dev_id": 4, 
                 "camera4/fps": 10, 
                 "camera4/height": 720, 
                 "camera4/width": 1080, 

                 "camera5/compressed_topic": "camera5/image/compressed", 
                 "camera5/dev_id": 5, 
                 "camera5/fps": 10, 
                 "camera5/height": 720, 
                 "camera5/width": 1080, 

                 "multiple_camera_status_topic": "multiple_camera/status",    
                 "multiple_camera_status_message_hz": 1.0, 
                 }
            ]
        )
    ])