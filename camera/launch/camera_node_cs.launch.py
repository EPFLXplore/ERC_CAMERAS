from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python import get_package_share_directory


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():

    # CAMERA LEFT
    camera_cs_0 = Node(
        package='camera',
        executable='camera',
        name='camera_cs_0',
        namespace='/ROVER',
        parameters=[
            {'camera_type': "monocular"},
            {'topic_service': "/ROVER/req_camera_cs_0"},
            {'topic_pub': "/ROVER/feed_camera_cs_0"},
            {'bw_pub': "/ROVER/bw_camera_cs_0"}, 
            {'devrule': "/dev/v4l/by-id/usb-046d_Brio_100_2414LZ53EPF8-video-index0"},
            {'screenshot': '/ROVER/screenshot_camera_cs_0'},
            {'fps': 15},
            {'x': 640},
            {'y': 480}
        ],
    )

    # CAMERA UP LEFT
    camera_cs_1 = Node(
        package='camera',
        executable='camera',
        name='camera_cs_1',
        namespace='/ROVER',
        parameters=[
            {'camera_type': "monocular"},
            {'topic_service': "/ROVER/req_camera_cs_1"},
            {'topic_pub': "/ROVER/feed_camera_cs_1"},
            {'bw_pub': "/ROVER/bw_camera_cs_1"},
            {'devrule': "/dev/v4l/by-id/usb-046d_Brio_100_2416LZ54BFC8-video-index0"},
            {'screenshot': '/ROVER/screenshot_camera_cs_1'},
            {'fps': 15},
            {'x': 640},
            {'y': 480}
        ],
    )

    # CAMERA RIGHT
    camera_cs_2 = Node(
        package='camera',
        executable='camera',
        name='camera_cs_2',
        namespace='/ROVER',
        parameters=[
            {'camera_type': "monocular"},
            {'topic_service': "/ROVER/req_camera_cs_2"},
            {'topic_pub': "/ROVER/feed_camera_cs_2"}, 
            {'bw_pub': "/ROVER/bw_camera_cs_2"},
            {'devrule': "/dev/v4l/by-id/usb-046d_Brio_100_2417LZ5087Q8-video-index0"},
            {'screenshot': '/ROVER/screenshot_camera_cs_2'},
            {'fps': 15},
            {'x': 640},
            {'y': 480}
        ],
    )
    
    # CAMERA UP RIGHT
    camera_cs_3 = Node(
        package='camera',
        executable='camera',
        name='camera_cs_3',
        namespace='/ROVER',
        parameters=[
            {'camera_type': "monocular"},
            {'topic_service': "/ROVER/req_camera_cs_3"},
            {'topic_pub': "/ROVER/feed_camera_cs_3"}, 
            {'bw_pub': "/ROVER/bw_camera_cs_3"},
            {'devrule': "/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0"},
            {'screenshot': '/ROVER/screenshot_camera_cs_3'},
            {'fps': 15},
            {'x': 640},
            {'y': 480}
        ],
    )

    
    return LaunchDescription(
        [
            camera_cs_0,
            camera_cs_1,
            camera_cs_2,
            camera_cs_3
        ]
    )
