#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[
                {'fcu_url': "udp://:14540@127.0.0.1:14557"},
                {'gcs_url': ''},
                {'system_id': 1},
                {'component_id': 1},
                {'target_system_id': 1},
                {'target_component_id': 1}
            ]
        )
    #     Node(
    #         package='sim',
    #         executable='arm_vehicle.py',
    #         name='arm_vehicle',
    #         output='screen'
    #         ),
    #    Node(
    #        package='sim',
    #        executable='set_velocity.py',
    #        name='set_velocity',
    #        output='screen',
    #    )
    ])