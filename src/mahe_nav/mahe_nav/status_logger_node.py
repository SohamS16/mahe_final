import math
import os
import time
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from std_msgs.msg import String
from nav_msgs.msg import Odometry

from mahe_nav_interfaces.msg import ArucoDetection, SignDetection

# -- Known ArUco world positions (for drift estimation only) --
ARUCO_WORLD_POS = {
    0: (4.526, 2.893, 0.274),
    1: (10.541, 3.593, 0.274),
    2: (8.828, 0.542, 0.299),
    3: (7.328, 1.368, 0.274),
}

ARUCO_MILESTONE = {
    0: 'MAZE ENTRY CONFIRMED',
    1: 'MAZE EXIT CONFIRMED',
    2: 'T-JUNCTION APPROACH',
    3: 'GOAL REACHED',
}

LOG_FILE = f'/tmp/mahe_ugv_mission_{int(time.time())}.log'


class StatusLoggerNode(Node):

    def __init__(self):
        super().__init__('status_logger')

        self.aruco_log = {}   
        self.sign_log = []
        self.last_sign_name = "NONE"
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.start_time = time.time()

        try:
            self.log_file = open(LOG_FILE, 'w')
            self._write_log(f'=== MAHE UGV Mission Log started {time.ctime()} ===')
        except IOError as e:
            self.get_logger().error(f'Cannot open log file: {e}')
            self.log_file = None

        self.pub_status = self.create_publisher(String, '/mission_status', 10)

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=10)

        self.create_subscription(Odometry, '/odom_fused', self._odom_cb, reliable)
        self.create_subscription(ArucoDetection, '/aruco/detections', self._aruco_cb, sensor_qos)
        self.create_subscription(SignDetection, '/sign_detection', self._sign_cb, sensor_qos)

        # Dashboard timer - set to 2.0s to act as a periodic summary
        self.create_timer(2.0, self._dashboard)
        self.get_logger().info(f'Status logger active. Logging to: {LOG_FILE}')

    def _odom_cb(self, msg: Odometry):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    def _aruco_cb(self, msg: ArucoDetection):
        mid     = msg.marker_id
        elapsed = time.time() - self.start_time

        # ── Compute corrected pose the same way the nav controller does ────────
        corrected_x = corrected_y = None
        drift_str = 'N/A'

        if mid in ARUCO_WORLD_POS and msg.distance > 0.05:
            wx, wy, _ = ARUCO_WORLD_POS[mid]
            d = float(msg.distance)
            β = float(msg.bearing_angle_rad)
            θ = self.pose_yaw
            world_angle = θ + β
            corrected_x = wx - d * math.cos(world_angle)
            corrected_y = wy - d * math.sin(world_angle)
            drift = math.hypot(corrected_x - self.pose_x, corrected_y - self.pose_y)
            drift_str = f'{drift:.3f} m'

        milestone = ARUCO_MILESTONE.get(mid, f'ARUCO_{mid}_DETECTED')

        if msg.first_detection:
            self.aruco_log[mid] = {
                'time_s':    round(elapsed, 2),
                'milestone': milestone,
                'drift':     drift_str,
            }

        # ── Always log every detection with full detail ────────────────────────
        tag = '*** FIRST ***' if msg.first_detection else '(repeat)'
        ekf_str  = f'EKF=({self.pose_x:.3f}, {self.pose_y:.3f}, {math.degrees(self.pose_yaw):.1f}°)'
        corr_str = (
            f'Corrected=({corrected_x:.3f}, {corrected_y:.3f})'
            if corrected_x is not None else 'Corrected=N/A'
        )
        log_line = (
            f'[{elapsed:7.2f}s] {tag} {milestone} | '
            f'ID={mid} dist={msg.distance:.2f}m '
            f'bearing={math.degrees(msg.bearing_angle_rad):.1f}° | '
            f'{ekf_str} | {corr_str} | Drift={drift_str}'
        )
        self.get_logger().info(log_line)
        self._write_log(log_line)

    def _sign_cb(self, msg: SignDetection):
        if msg.sign_type == 'NONE' or msg.confidence < 0.45: return

        elapsed = time.time() - self.start_time
        recent = [e for e in self.sign_log if e['type'] == msg.sign_type]
        if recent and (elapsed - recent[-1]['time_s']) < 3.0: return

        self.last_sign_name = msg.sign_type
        self.sign_log.append({'type': msg.sign_type, 'time_s': round(elapsed, 2)})
        
        # IMMEDIATE TERMINAL LOG
        log_line = f'[{elapsed:7.2f}s] SIGN DETECTED: {msg.sign_type} (Confidence: {msg.confidence:.2f})'
        self.get_logger().info(log_line)
        self._write_log(log_line)

    def _dashboard(self):
        elapsed = time.time() - self.start_time
        seen = sorted(self.aruco_log.keys())
        
        # Periodic Summary Box
        lines = [
            '---------------------------------------------------',
            f'| STATUS @ {elapsed:6.1s}s | POS: ({self.pose_x:.2f}, {self.pose_y:.2f}) | SIGN: {self.last_sign_name}',
            f'| ARUCO SEEN: {seen}',
            '---------------------------------------------------'
        ]
        for l in lines:
            self.get_logger().info(l)
        self._publish_status()

    def _write_log(self, text: str):
        if self.log_file:
            self.log_file.write(text + '\n')
            self.log_file.flush()

    def _publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'aruco_seen': sorted(list(self.aruco_log.keys())),
            'last_sign': self.last_sign_name,
            'pose': [round(self.pose_x, 2), round(self.pose_y, 2), round(math.degrees(self.pose_yaw), 1)],
        })
        self.pub_status.publish(msg)

    def destroy_node(self):
        if self.log_file: self.log_file.close()
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
