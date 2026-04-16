"""
nav.launch.py
=============
Launches the full MAHE UGV navigation stack on top of the already-running
simulation (sim.launch.py must be running first).

Starts:
  1.  EKF node          — robot_localization, fuses odom + IMU → /odom_fused
  2.  LiDAR analyzer    — raw scan → structured /lidar/analysis
  3.  ArUco detector    — camera → /aruco/detections
  4.  Sign detector     — camera → /sign_detection
  5.  Status logger     — aggregates all events, publishes /mission_status
  6.  Nav controller    — state machine, publishes /cmd_vel

Usage:
    ros2 launch mahe_nav nav.launch.py

To run with debug logging:
    ros2 launch mahe_nav nav.launch.py log_level:=debug
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_pkg   = get_package_share_directory('mahe_nav')
    ekf_cfg   = os.path.join(nav_pkg, 'config', 'ekf.yaml')

    log_level = LaunchConfiguration('log_level')
    fwd_idx   = LaunchConfiguration('lidar_forward_index')

    # ── Robot state publisher is already running from sim.launch.py ─────────────
    # DO NOT start it again here.

    # ── 1. EKF (robot_localization) ─────────────────────────────────────────────
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

    # ── 3. ArUco detector ────────────────────────────────────────────────────────
    aruco_node = Node(
        package    = 'mahe_nav',
        executable = 'aruco_detector',
        name       = 'aruco_detector',
        output     = 'screen',
        parameters = [
            {'use_sim_time': True},
            {'marker_size_m': 0.150},   # confirmed from OBJ geometry
        ],
        arguments  = ['--ros-args', '--log-level', log_level],
    )

    # ── 4. Sign detector ─────────────────────────────────────────────────────────
    sign_node = Node(
        package    = 'mahe_nav',
        executable = 'sign_detector',
        name       = 'sign_detector',
        output     = 'screen',
        parameters = [
            {'use_sim_time': True},
            {'match_threshold': 0.45},
        ],
        arguments  = ['--ros-args', '--log-level', log_level],
    )

    # ── 5. Status logger ─────────────────────────────────────────────────────────
    logger_node = Node(
        package    = 'mahe_nav',
        executable = 'status_logger',
        name       = 'status_logger',
        output     = 'screen',
        parameters = [{'use_sim_time': True}],
        arguments  = ['--ros-args', '--log-level', log_level],
    )

    # ── 6. Navigation controller ─────────────────────────────────────────────────
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
            default_value='info',
            description='Logging level: debug, info, warn, error'
        ),
        DeclareLaunchArgument(
            'lidar_forward_index',
            default_value='180',
            description=(
                'LiDAR array index pointing FORWARD.  '
                'Physics correct = 180 (angle_min=-π → index 180 = angle 0 = fwd).  '
                'Set to 0 if robot moves backward (document convention).'
            )
        ),
        ekf_node,
        lidar_node,
        aruco_node,
        sign_node,
        logger_node,
        nav_node,
    ])
