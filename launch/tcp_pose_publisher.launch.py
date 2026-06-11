"""
tcp_pose_publisher.launch.py

Brings up:
  1. aubo_client_node  (JSON-RPC ↔ TCP bridge, from aubo_ros2_driver)
  2. tcp_pose_publisher (polls getTcpPose, publishes PoseStamped)

Usage:
  ros2 launch aubo_ros2_projects tcp_pose_publisher.launch.py

With a real robot:
  ros2 launch aubo_ros2_projects tcp_pose_publisher.launch.py \
      robot_ip:=192.168.1.100 robot:=rob1 publish_rate:=20.0

Then inspect the output:
  ros2 topic echo /tcp_pose_publisher/tcp_pose
  ros2 topic hz  /tcp_pose_publisher/tcp_pose
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch arguments ─────────────────────────────────────────────────────
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.12.108',
        description='IP address of the AUBO robot controller',
    )
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='30004',
        description='JSON-RPC TCP port on the controller',
    )
    robot_arg = DeclareLaunchArgument(
        'robot',
        default_value='rob1',
        description='Robot prefix used in JSON-RPC method names',
    )
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='How often to poll getTcpPose and publish PoseStamped (Hz)',
    )
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='ROS2 log level (debug / info / warn / error)',
    )

    # ── Bring up the JSON-RPC ↔ TCP bridge ───────────────────────────────────
    aubo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('aubo_ros2_driver'),
                'launch',
                'aubo_client.launch.py',
            ])
        ),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'port':     LaunchConfiguration('port'),
            'robot':    LaunchConfiguration('robot'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    # ── TCP Pose Publisher node ───────────────────────────────────────────────
    # frame_id is built as "<robot>/base_link" (e.g. "rob1/base_link").
    # The PoseStamped header.frame_id must match the TF tree published by
    # robot_state_publisher if you want RViz to display the pose correctly.
    tcp_pose_publisher_node = Node(
        package='aubo_ros2_projects',
        executable='tcp_pose_publisher.py',
        name='tcp_pose_publisher',
        output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'frame_id': [LaunchConfiguration('robot'), '/base_link'],
            'topic': 'tcp_pose',
        }],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    return LaunchDescription([
        robot_ip_arg,
        port_arg,
        robot_arg,
        publish_rate_arg,
        log_level_arg,
        aubo_client_launch,
        tcp_pose_publisher_node,
    ])
