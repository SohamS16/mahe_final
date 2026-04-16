"""
nav_lidar_only.launch.py
========================
Minimal launch for PURE LiDAR obstacle avoidance testing.
Launches ONLY:
  1. EKF node          — fuses odom + IMU → /odom_fused  (needed for stuck detection yaw)
  2. LiDAR analyzer    — raw scan → /lidar/analysis
  3. Nav controller    — pure LiDAR state machine → /cmd_vel

Camera nodes (aruco_detector, sign_detector, status_logger) are intentionally
NOT started. The nav_controller also has camera callbacks disabled internally.

Usage:
    ros2 launch mahe_nav nav_lidar_only.launch.py

With debug logging:
    ros2 launch mahe_nav nav_lidar_only.launch.py log_level:=debug

With a different forward index (if robot drives backward):
    ros2 launch mahe_nav nav_lidar_only.launch.py lidar_forward_index:=0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg  = get_package_share_directory('mahe_nav')
    ekf_cfg  = os.path.join(nav_pkg, 'config', 'ekf.yaml')

    log_level = LaunchConfiguration('log_level')
    fwd_idx   = LaunchConfiguration('lidar_forward_index')

    # ── 1. EKF (robot_localization) ─────────────────────────────────────────────
    # Required for pose_yaw used in U-turn tracking and stuck detection.
    ekf_node = Node(
        package    = 'robot_localization',
        executable = 'ekf_node',
        name       = 'ekf_filter_node',
        output     = 'screen',
        parameters = [ekf_cfg, {'use_sim_time': True}],
        remappings = [
            ('odometry/filtered', '/odom_fused'),
        ],
        arguments  = ['--ros-args', '--log-level', log_level],
    )

    # ── 2. LiDAR analyzer ────────────────────────────────────────────────────────
    # Converts raw LaserScan → structured LidarAnalysis message.
    lidar_node = Node(
        package    = 'mahe_nav',
        executable = 'lidar_analyzer',
        name       = 'lidar_analyzer',
        output     = 'screen',
        parameters = [
            {'use_sim_time': True},
            {'lidar_forward_index': fwd_idx},
        ],
        arguments  = ['--ros-args', '--log-level', log_level],
    )

    # ── 3. Nav controller ────────────────────────────────────────────────────────
    # Pure LiDAR state machine. Camera callbacks are disabled inside the node.
    nav_node = Node(
        package    = 'mahe_nav',
        executable = 'nav_controller',
        name       = 'nav_controller',
        output     = 'screen',
        parameters = [{'use_sim_time': True}],
        arguments  = ['--ros-args', '--log-level', log_level],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'log_level',
            default_value = 'info',
            description   = 'Logging level: debug, info, warn, error'
        ),
        DeclareLaunchArgument(
            'lidar_forward_index',
            default_value = '180',
            description   = (
                'LiDAR index pointing FORWARD. '
                'Default 180 = physics convention (angle_min=-π, index 180 = 0 rad = fwd). '
                'Set to 0 if robot drives backward.'
            )
        ),
        ekf_node,
        lidar_node,
        nav_node,
    ])
