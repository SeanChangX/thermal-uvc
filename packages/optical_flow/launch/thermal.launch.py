import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    uvc = Node(
        package='libuvc_infra',
        executable='uvc_cam_node',
        name='uvc_cam_node',
        output='screen'
    )

    optical = Node(
        package='optical_flow',
        executable='optical_flow_node',
        name='optical_flow_node',
        output='screen'
    )

    ld = LaunchDescription()

    ld.add_action(uvc)
    # ld.add_action(optical)
    ld.add_action(
        TimerAction(
            period=1.0,
            actions=[optical]
        )
    )

    return ld
