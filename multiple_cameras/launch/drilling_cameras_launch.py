from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([       
        Node(
            package="multiple_cameras",
            executable="multiple_camera_publisher_python.py",
            name="drilling_camera_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"modify_camera_settings_service": "multiple_camera/modify_camera_settings_string",
                 "camera_id_service": "multiple_camera/get_camera_id",

                 "camera0/name": "saitama", 
                 "camera0/fps": 10, 
                 "camera0/height": 720, 
                 "camera0/width": 1080,

                 "camera1/name": "demon_cyborg", 
                 "camera1/fps": 10, 
                 "camera1/height": 720, 
                 "camera1/width": 1080,                   

                 "camera2/name": "watchdog_man", 
                 "camera2/fps": 10, 
                 "camera2/height": 720, 
                 "camera2/width": 1080, 

                 "camera3/name": "metal_bat", 
                 "camera3/fps": 10, 
                 "camera3/height": 720, 
                 "camera3/width": 1080, 

                 "camera4/name": "none", 
                 "camera4/fps": 10, 
                 "camera4/height": 720, 
                 "camera4/width": 1080, 
                 
                 "camera5/name": "none", 
                 "camera5/fps": 10, 
                 "camera5/height": 720, 
                 "camera5/width": 1080, 

                 "camera6/name": "none", 
                 "camera6/fps": 10, 
                 "camera6/height": 720, 
                 "camera6/width": 1080, 

                 "camera7/name": "none", 
                 "camera7/fps": 10, 
                 "camera7/height": 720, 
                 "camera7/width": 1080, 

                 "multiple_camera_status_topic": "multiple_camera/status",    
                 "multiple_camera_status_message_hz": 1.0, 
                 }
            ]
        )
    ])