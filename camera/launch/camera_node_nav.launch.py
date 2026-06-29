#Author: Arno Laurie
#Date: 22/11/2024

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
import os
from ament_index_python import get_package_share_directory
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit


def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path


def generate_launch_description():

    # nav_front_camera = Node(
    #     package='camera',
    #     executable='camera',
    #     name='camera_nav_front',
    #     namespace='/NAV',
    #     parameters=[
    #         {'camera_type': "oakd_stereo"},
    #         {'topic_service': "/NAV/req_camera_nav_0"},
    #         {'topic_pub': "/NAV/feed_camera_nav_0"},
    #         {'depth': "/NAV/depth_camera_nav_0"},
    #         {'bw_pub': "/NAV/bw_camera_nav_0"},
    #         {'depth_req': "/NAV/depth_req_camera_nav_0"},
    #         {'devrule': ""},
    #         {'info': "/NAV/camera_info_"}, # we concatenate the devrule
    #         {'state': "/NAV/state_camera_nav_0"},
    #         {'screenshot': '/NAV/screenshot_camera_nav_0'},
    #         {'fps': 5},
    #         {'x': 1280},
    #         {'y': 720},
    #         {'flip_camera':False}
    #     ],
    # )

    # nav_realsense_aruco_camera_left = Node(
    #     package='camera',
    #     executable='camera',
    #     name='camera_aruco_left',
    #     namespace='/NAV',
    #     parameters=[
    #         {'camera_type': "realsense_stereo"},
    #         {'topic_service': "/NAV/req_camera_nav_1"},
    #         {'topic_pub': "/NAV/feed_camera_nav_1"},
    #         {'bw_pub': "/NAV/bw_camera_nav_1"}, 
    #         {'devrule': "102122061110"},
    #         {'info': "/NAV/camera_info_"}, # we concatenate the devrule
    #         {'depth_req': "/NAV/depth_req_camera_nav_1"},
    #         {'state': "/NAV/state_camera_nav_1"},
    #         {'fps': 6},
    #         {'x': 1280},
    #         {'y': 720}
    #     ],
    # )

    # nav_realsense_aruco_camera_right = Node(
    #     package='camera',
    #     executable='camera',
    #     name='camera_aruco_right',
    #     namespace='/NAV',
    #     parameters=[
    #         {'camera_type': "realsense_stereo"},
    #         {'topic_service': "/NAV/req_camera_nav_2"},
    #         {'topic_pub': "/NAV/feed_camera_nav_2"},
    #         {'bw_pub': "/NAV/bw_camera_nav_2"}, 
    #         {'devrule': "135322062945"},
    #         {'info': "/NAV/camera_info_"}, # we concatenate the devrule
    #         {'depth_req': "/NAV/depth_req_camera_nav_2"},
    #         {'state': "/NAV/state_camera_nav_2"},
    #         {'fps': 6},
    #         {'x': 1280},
    #         {'y': 720}
    #     ],
    # )

    # nav_test_camera = Node(
    #     package='camera',
    #     executable='camera',
    #     name='camera_nav_test',
    #     namespace='/NAV',
    #     parameters=[
    #         {'camera_type': "oak1w_stereo"},
    #         {'topic_service': "/NAV/req_camera_nav_3"},
    #         {'topic_pub': "/NAV/feed_camera_nav_3"},
    #         {'depth': "/NAV/depth_camera_nav_3"},
    #         {'bw_pub': "/NAV/bw_camera_nav_3"},
    #         {'depth_req': "/NAV/depth_req_camera_nav_3"},
    #         {'devrule': "19443010714B177E00"}, # To get with print("Found Oak1W Stereo Camera : ", dai.Device.getAllAvailableDevices()) under the name : "deviceId"
    #         {'info': "/NAV/camera_info_"}, # we concatenate the devrule
    #         {'state': "/NAV/state_camera_nav_3"},
    #         {'screenshot': '/NAV/screenshot_camera_nav_3'},
    #         {'fps': 30},
    #         {'x': 1280},
    #         {'y': 720},
    #         {'flip_camera':False}
    #     ],
    # )

    nav_0_oak1w_21W_T2544_0008 = LifecycleNode(
        package='camera',
        executable='camera',
        name='nav_0_oak1w_21W_T2544_0008',
        namespace='/NAV',
        parameters=[
            {'camera_type': "oak1w_stereo"},
            {'topic_service': "/NAV/req_camera_nav_0"},
            {'topic_pub': "/NAV/feed_camera_nav_0"},
            {'depth': "/NAV/depth_camera_nav_0"},
            {'bw_pub': "/NAV/bw_camera_nav_0"},
            {'depth_req': "/NAV/depth_req_camera_nav_0"},
            {'devrule': ""},
            {'info': "/NAV/camera_info_0"}, # we concatenate the devrule
            {'state': "/NAV/state_camera_nav_0"},
            {'screenshot': '/NAV/screenshot_camera_nav_0'},
            {'fps': 5},
            {'x': 1280},
            {'y': 720},
            {'flip_camera':False},
            {'cam_id':"19443010714B177E00"},
            {'health_check_period_sec': 1.0},
            {'health_timeout_sec': 3.0},
        ],
        output='screen'
    )

    nav_1_oak1w_21W_T2544_0069 = LifecycleNode(
        package='camera',
        executable='camera',
        name='nav_1_oak1w_21W_T2544_0069',
        namespace='/NAV',
        parameters=[
            {'camera_type': "oak1w_stereo"},
            {'topic_service': "/NAV/req_camera_nav_1"},
            {'topic_pub': "/NAV/feed_camera_nav_1"},
            {'depth': "/NAV/depth_camera_nav_1"},
            {'bw_pub': "/NAV/bw_camera_nav_1"},
            {'depth_req': "/NAV/depth_req_camera_nav_1"},
            {'devrule': ""},
            {'info': "/NAV/camera_info_1"}, # we concatenate the devrule
            {'state': "/NAV/state_camera_nav_1"},
            {'screenshot': '/NAV/screenshot_camera_nav_1'},
            {'fps': 5},
            {'x': 1280},
            {'y': 720},
            {'flip_camera':False},
            {'cam_id':"19443010A19E157E00"},
            {'health_check_period_sec': 1.0},
            {'health_timeout_sec': 3.0},
        ],
        output='screen',
    )

    nav_2_oak1w_21W_T2544_0035 = LifecycleNode(
        package='camera',
        executable='camera',
        name='nav_2_oak1w_21W_T2544_0035',
        namespace='/NAV',
        parameters=[
            {'camera_type': "oak1w_stereo"},
            {'topic_service': "/NAV/req_camera_nav_2"},
            {'topic_pub': "/NAV/feed_camera_nav_2"},
            {'depth': "/NAV/depth_camera_nav_2"},
            {'bw_pub': "/NAV/bw_camera_nav_2"},
            {'depth_req': "/NAV/depth_req_camera_nav_2"},
            {'devrule': ""},
            {'info': "/NAV/camera_info_2"}, # we concatenate the devrule
            {'state': "/NAV/state_camera_nav_2"},
            {'screenshot': '/NAV/screenshot_camera_nav_2'},
            {'fps': 5},
            {'x': 1280},
            {'y': 720},
            {'flip_camera':False},
            {'cam_id':"19443010816C177E00"},
            {'health_check_period_sec': 1.0},
            {'health_timeout_sec': 3.0},
        ],
        output='screen',
    )

    nav_3_oakd = LifecycleNode(
        package='camera',
        executable='camera',
        name='nav_3_oakd',
        namespace='/NAV',
        parameters=[
            {'camera_type': "oak1w_stereo"},
            {'topic_service': "/NAV/req_camera_nav_3"},
            {'topic_pub': "/NAV/feed_camera_nav_3"},
            {'depth': "/NAV/depth_camera_nav_3"},
            {'bw_pub': "/NAV/bw_camera_nav_3"},
            {'depth_req': "/NAV/depth_req_camera_nav_3"},
            {'devrule': ""},
            {'info': "/NAV/camera_info_3"}, # we concatenate the devrule
            {'state': "/NAV/state_camera_nav_3"},
            {'screenshot': '/NAV/screenshot_camera_nav_3'},
            {'fps': 5},
            {'x': 1280},
            {'y': 720},
            {'flip_camera':False},
            # {'cam_id':"19443010714B177E00"},
            {'cam_id':"19443010F1C5E01200"},
            {'health_check_period_sec': 1.0},
            {'health_timeout_sec': 3.0},
        ],
        output='screen',
    )

    configure_camera_0 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/NAV/nav_0_oak1w_21W_T2544_0008',
             'configure'],
        output='screen'
    )

    configure_camera_1 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/NAV/nav_1_oak1w_21W_T2544_0069',
             'configure'],
        output='screen'
    )

    configure_camera_2 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/NAV/nav_2_oak1w_21W_T2544_0035',
             'configure'],
        output='screen'
    )

    configure_camera_3 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/NAV/nav_3_oakd',
             'configure'],
        output='screen'
    ) 

    activate_camera_0 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/NAV/nav_0_oak1w_21W_T2544_0008',
             'activate'],
        output='screen'
    )

    activate_camera_1 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/NAV/nav_1_oak1w_21W_T2544_0069',
             'activate'],
        output='screen'
    )

    activate_camera_2 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/NAV/nav_2_oak1w_21W_T2544_0035',
             'activate'],
        output='screen'
    )

    activate_camera_3 = ExecuteProcess(
        cmd=['ros2', 'lifecycle', 'set',
             '/NAV/nav_3_oakd',
             'activate'],
        output='screen'
    )

    call_camera_0 = ExecuteProcess(
        cmd=['ros2', 'service', 'call',
             '/NAV/req_camera_nav_0',
             'std_srvs/srv/SetBool',
             '{data: true}'],
        output='screen'
    )

    call_camera_1 = ExecuteProcess(
        cmd=['ros2', 'service', 'call',
             '/NAV/req_camera_nav_1',
             'std_srvs/srv/SetBool',
             '{data: true}'],
        output='screen'
    )

    call_camera_2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call',
             '/NAV/req_camera_nav_2',
             'std_srvs/srv/SetBool',
             '{data: true}'],
        output='screen'
    )

    call_camera_3 = ExecuteProcess(
        cmd=['ros2', 'service', 'call',
             '/NAV/req_camera_nav_3',
             'std_srvs/srv/SetBool',
             '{data: true}'],
        output='screen'
    )

    # Stagger all three: lifecycle configure creates services/camera objects;
    # SetBool activation still owns hardware open/stream start.
    nav_0_delayed = TimerAction(period=0.0, actions=[nav_0_oak1w_21W_T2544_0008])
    nav_1_delayed = TimerAction(period=15.0, actions=[nav_1_oak1w_21W_T2544_0069])
    nav_2_delayed = TimerAction(period=30.0, actions=[nav_2_oak1w_21W_T2544_0035])
    nav_3_delayed = TimerAction(period=45.0, actions=[nav_3_oakd])

    configure_camera_0_delayed = TimerAction(period=1.0, actions=[configure_camera_0])
    configure_camera_1_delayed = TimerAction(period=16.0, actions=[configure_camera_1])
    configure_camera_2_delayed = TimerAction(period=31.0, actions=[configure_camera_2])
    configure_camera_3_delayed = TimerAction(period=46.0, actions=[configure_camera_3])

    activate_camera_0_delayed = TimerAction(period=5.0, actions=[activate_camera_0])
    activate_camera_1_delayed = TimerAction(period=20.0, actions=[activate_camera_1])
    activate_camera_2_delayed = TimerAction(period=35.0, actions=[activate_camera_2])
    activate_camera_3_delayed = TimerAction(period=50.0, actions=[activate_camera_3])

    # Start streaming only after `ros2 lifecycle set ... activate` exits, so
    # `lifecycle_active` is True before SetBool (fixed 1s timers can race on Jetson).
    call_after_activate_0 = RegisterEventHandler(
        OnProcessExit(target_action=activate_camera_0, on_exit=[call_camera_0])
    )
    call_after_activate_1 = RegisterEventHandler(
        OnProcessExit(target_action=activate_camera_1, on_exit=[call_camera_1])
    )
    call_after_activate_2 = RegisterEventHandler(
        OnProcessExit(target_action=activate_camera_2, on_exit=[call_camera_2])
    )
    call_after_activate_3 = RegisterEventHandler(
        OnProcessExit(target_action=activate_camera_3, on_exit=[call_camera_3])
    )

    return LaunchDescription([
        # nav_front_camera,
        # TimerAction(period=2.0, actions=[nav_realsense_aruco_camera_left]),
        # TimerAction(period=4.0, actions=[nav_realsense_aruco_camera_right]),
        nav_0_delayed,
        nav_1_delayed,
        nav_2_delayed,
        nav_3_delayed,
        configure_camera_0_delayed,
        configure_camera_1_delayed,
        configure_camera_2_delayed,
        configure_camera_3_delayed,
        activate_camera_0_delayed,
        activate_camera_1_delayed,
        activate_camera_2_delayed,
        activate_camera_3_delayed,
        call_after_activate_0,
        call_after_activate_1,
        call_after_activate_2,
        call_after_activate_3,
    ])