from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="multiple_cameras",
            executable="define_camera_parameters.py",
            name="define_camera_parameters",
            output="screen",
            emulate_tty=True,
            parameters=[
                {'camera_id_service': 'multiple_camera/get_camera_id',
                 'camera0/name': 'saitama', 
                 'camera0/dev_id': 1,
                 'camera1/name': 'demon_cyborg', 
                 'camera1/dev_id': 0,
                 'camera2/name': 'puri_puri_prisoner', 
                 'camera2/dev_id': 2,
                 'camera3/name': 'tanktop_master', 
                 'camera3/dev_id': 3,
                 'camera4/name': 'metal_bat', 
                 'camera4/dev_id': 4,
                 'camera5/name': 'superalloy_darkshine', 
                 'camera5/dev_id': 5,
                 'camera6/name': 'flashy_flash', 
                 'camera6/dev_id': 6,
                 'camera7/name': 'watchdog_man', 
                 'camera7/dev_id': 7,                         
                }
            ],
        ),
    ])