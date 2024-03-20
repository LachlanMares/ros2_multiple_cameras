from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="multiple_cameras",
            executable="compressed_to_image_publisher_python.py",
            name="drilling_camera_compressed_image_subscriber",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"camera0/name": "saitama",
                 "camera1/name": "demon_cyborg",
                 "camera2/name": "watchdog_man",
                 "camera3/name": "metal_bat",
                 "camera4/name": "none",
                 "camera5/name": "none",
                 "camera6/name": "none",
                 "camera7/name": "none",
                 }
            ]
        )
    ])