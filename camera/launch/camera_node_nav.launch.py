# Author: Paul Bourgois
# Date: 02/07/2026

from launch import LaunchDescription
from launch.actions import TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import LifecycleNode


def generate_launch_description():

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
            {'info': "/NAV/camera_info_0"},
            {'state': "/NAV/state_camera_nav_0"},
            {'screenshot': '/NAV/screenshot_camera_nav_0'},
            {'fps': 5},
            {'x': 1280},
            {'y': 720},
            {'flip_camera': False},
            {'cam_id': "19443010714B177E00"},
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
            {'info': "/NAV/camera_info_1"},
            {'state': "/NAV/state_camera_nav_1"},
            {'screenshot': '/NAV/screenshot_camera_nav_1'},
            {'fps': 5},
            {'x': 1280},
            {'y': 720},
            {'flip_camera': False},
            {'cam_id': "19443010A19E157E00"},
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
            {'info': "/NAV/camera_info_2"},
            {'state': "/NAV/state_camera_nav_2"},
            {'screenshot': '/NAV/screenshot_camera_nav_2'},
            {'fps': 5},
            {'x': 1280},
            {'y': 720},
            {'flip_camera': False},
            {'cam_id': "19443010816C177E00"},
            {'health_check_period_sec': 1.0},
            {'health_timeout_sec': 3.0},
        ],
        output='screen',
    )

    configure_camera_0 = ExecuteProcess(
        cmd=[
            'ros2', 'lifecycle', 'set',
            '/NAV/nav_0_oak1w_21W_T2544_0008',
            'configure'
        ],
        output='screen'
    )

    configure_camera_1 = ExecuteProcess(
        cmd=[
            'ros2', 'lifecycle', 'set',
            '/NAV/nav_1_oak1w_21W_T2544_0069',
            'configure'
        ],
        output='screen'
    )

    configure_camera_2 = ExecuteProcess(
        cmd=[
            'ros2', 'lifecycle', 'set',
            '/NAV/nav_2_oak1w_21W_T2544_0035',
            'configure'
        ],
        output='screen'
    )

    activate_camera_0 = ExecuteProcess(
        cmd=[
            'ros2', 'lifecycle', 'set',
            '/NAV/nav_0_oak1w_21W_T2544_0008',
            'activate'
        ],
        output='screen'
    )

    activate_camera_1 = ExecuteProcess(
        cmd=[
            'ros2', 'lifecycle', 'set',
            '/NAV/nav_1_oak1w_21W_T2544_0069',
            'activate'
        ],
        output='screen'
    )

    activate_camera_2 = ExecuteProcess(
        cmd=[
            'ros2', 'lifecycle', 'set',
            '/NAV/nav_2_oak1w_21W_T2544_0035',
            'activate'
        ],
        output='screen'
    )

    call_camera_0 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/NAV/req_camera_nav_0',
            'std_srvs/srv/SetBool',
            '{data: true}'
        ],
        output='screen'
    )

    call_camera_1 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/NAV/req_camera_nav_1',
            'std_srvs/srv/SetBool',
            '{data: true}'
        ],
        output='screen'
    )

    call_camera_2 = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/NAV/req_camera_nav_2',
            'std_srvs/srv/SetBool',
            '{data: true}'
        ],
        output='screen'
    )

    return LaunchDescription([
        nav_0_oak1w_21W_T2544_0008,

        TimerAction(
            period=1.0,
            actions=[configure_camera_0]
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=configure_camera_0,
                on_exit=[activate_camera_0]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=activate_camera_0,
                on_exit=[
                    call_camera_0,
                    nav_1_oak1w_21W_T2544_0069,
                    TimerAction(period=1.0, actions=[configure_camera_1]),
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=configure_camera_1,
                on_exit=[activate_camera_1]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=activate_camera_1,
                on_exit=[
                    call_camera_1,
                    nav_2_oak1w_21W_T2544_0035,
                    TimerAction(period=1.0, actions=[configure_camera_2]),
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=configure_camera_2,
                on_exit=[activate_camera_2]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=activate_camera_2,
                on_exit=[call_camera_2]
            )
        ),
    ])