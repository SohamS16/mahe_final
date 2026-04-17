"""
status_logger_node.py
=====================
Mission dashboard for the current MAHE UGV arena.

Subscribes to:
  /odom_fused          nav_msgs/Odometry
  /aruco/detections    mahe_nav_interfaces/ArucoDetection
  /sign_detection      mahe_nav_interfaces/SignDetection
  /lidar/analysis      mahe_nav_interfaces/LidarAnalysis

Publishes:
  /mission_status      std_msgs/String  (JSON, 2 Hz)

Logs to:
  /tmp/mahe_ugv_mission_<timestamp>.log
"""

import math
import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from mahe_nav_interfaces.msg import ArucoDetection, SignDetection, LidarAnalysis


# ArUco ID → human-readable command (matches nav_controller_node.py)
ARUCO_CMD = {
    0: 'TURN RIGHT',
    1: 'TURN LEFT',
    2: 'FOLLOW GREEN',
    3: 'U-TURN',
    4: 'FOLLOW ORANGE',
}

LOG_FILE = f'/tmp/mahe_ugv_mission_{int(time.time())}.log'


class StatusLoggerNode(Node):

    def __init__(self):
        super().__init__('status_logger')

        # State
        self.pose_x   = 0.0
        self.pose_y   = 0.0
        self.pose_yaw = 0.0
        self.start_time = time.time()

        self.aruco_log     = {}   # mid → {time_s, cmd, dist}
        self.sign_log      = []   # [{type, time_s, confidence}]
        self.last_sign     = 'NONE'
        self.fsm_state     = 'UNKNOWN'
        self.junction_type = 'UNKNOWN'
        self.forward_dist  = 0.0

        # Log file
        try:
            self.log_file = open(LOG_FILE, 'w')
            self._write(f'=== MAHE UGV Mission Log — {time.ctime()} ===\n')
        except IOError as e:
            self.get_logger().error(f'Cannot open log file: {e}')
            self.log_file = None

        # Publishers
        self.pub_status = self.create_publisher(String, '/mission_status', 10)

        # Subscribers
        best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable    = QoSProfile(depth=10)

        self.create_subscription(Odometry,      '/odom_fused',       self._odom_cb,  reliable)
        self.create_subscription(ArucoDetection,'/aruco/detections', self._aruco_cb, best_effort)
        self.create_subscription(SignDetection, '/sign_detection',   self._sign_cb,  best_effort)
        self.create_subscription(LidarAnalysis, '/lidar/analysis',   self._lidar_cb, best_effort)

        # Dashboard timer — 2 Hz
        self.create_timer(0.5, self._dashboard)
        self.get_logger().info(f'Status logger active. Logging to: {LOG_FILE}')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _lidar_cb(self, msg: LidarAnalysis):
        self.forward_dist  = msg.forward_dist
        self.junction_type = msg.junction_type if msg.junction_type else 'UNKNOWN'

    def _aruco_cb(self, msg: ArucoDetection):
        mid     = msg.marker_id
        elapsed = time.time() - self.start_time
        cmd     = ARUCO_CMD.get(mid, f'UNKNOWN_ID_{mid}')

        # Log first detection only
        if msg.first_detection:
            self.aruco_log[mid] = {
                'time_s': round(elapsed, 2),
                'cmd':    cmd,
                'dist':   round(float(msg.distance), 3),
            }
            line = (
                f'[{elapsed:7.2f}s] *** FIRST ARUCO *** '
                f'ID={mid} | CMD={cmd} | '
                f'dist={msg.distance:.2f}m | '
                f'bearing={math.degrees(msg.bearing_angle_rad):.1f}° | '
                f'pos=({self.pose_x:.2f}, {self.pose_y:.2f})'
            )
            self.get_logger().info(line)
            self._write(line)
        else:
            # Repeat detection — only log if within action threshold
            if float(msg.distance) <= 0.8:
                line = (
                    f'[{elapsed:7.2f}s] ARUCO repeat '
                    f'ID={mid} | dist={msg.distance:.2f}m | '
                    f'bearing={math.degrees(msg.bearing_angle_rad):.1f}°'
                )
                self.get_logger().debug(line)

    def _sign_cb(self, msg: SignDetection):
        if not msg.sign_type or msg.sign_type == 'NONE':
            return
        if msg.confidence < 0.45:
            return

        elapsed = time.time() - self.start_time

        # Debounce — suppress same sign within 3s
        recent = [e for e in self.sign_log if e['type'] == msg.sign_type]
        if recent and (elapsed - recent[-1]['time_s']) < 3.0:
            return

        self.last_sign = msg.sign_type
        self.sign_log.append({
            'type':       msg.sign_type,
            'time_s':     round(elapsed, 2),
            'confidence': round(float(msg.confidence), 3),
        })

        line = (
            f'[{elapsed:7.2f}s] SIGN: {msg.sign_type} '
            f'(conf={msg.confidence:.2f}) | '
            f'pos=({self.pose_x:.2f}, {self.pose_y:.2f})'
        )
        self.get_logger().info(line)
        self._write(line)

    # ── Dashboard ─────────────────────────────────────────────────────────────

    def _dashboard(self):
        elapsed  = time.time() - self.start_time
        seen_ids = sorted(self.aruco_log.keys())

        lines = [
            '─' * 52,
            f'  T={elapsed:6.1f}s  |  pos=({self.pose_x:.2f}, {self.pose_y:.2f})  |  yaw={math.degrees(self.pose_yaw):.1f}°',
            f'  fwd={self.forward_dist:.2f}m  |  junction={self.junction_type}',
            f'  ARUCO seen: {seen_ids}',
            f'  Last sign : {self.last_sign}',
            '─' * 52,
        ]
        for l in lines:
            self.get_logger().info(l)

        self._publish_status(elapsed, seen_ids)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _write(self, text: str):
        if self.log_file:
            self.log_file.write(text + '\n')
            self.log_file.flush()

    def _publish_status(self, elapsed: float, seen_ids: list):
        msg = String()
        msg.data = json.dumps({
            'elapsed_s':    round(elapsed, 1),
            'pos':          [round(self.pose_x, 2), round(self.pose_y, 2)],
            'yaw_deg':      round(math.degrees(self.pose_yaw), 1),
            'junction':     self.junction_type,
            'forward_dist': round(self.forward_dist, 2),
            'aruco_seen':   seen_ids,
            'last_sign':    self.last_sign,
        })
        self.pub_status.publish(msg)

    def destroy_node(self):
        if self.log_file:
            self._write(f'\n=== Mission ended {time.ctime()} ===')
            self.log_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StatusLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()