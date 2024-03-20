from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="multiple_cameras",
            executable="compressed_to_image_publisher_python.py",
            name="multiple_camera_compressed_image_subscriber",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"camera0/name": "saitama",
                 "camera1/name": "demon_cyborg",
                 "camera2/name": "puri_puri_prisoner",
                 "camera3/name": "tanktop_master",
                 "camera4/name": "metal_bat",
                 "camera5/name": "superalloy_darkshine",
                 "camera6/name": "flashy_flash",
                 "camera7/name": "watchdog_man",
                 }
            ]
        )
    ])