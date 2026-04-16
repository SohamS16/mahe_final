"""
lidar_analyzer_node.py
======================
Converts the raw LaserScan into structured analysis that the nav_controller
can use directly — no raw scan indices in the controller.

LIDAR SPECS (from URDF)
-----------------------
    Sensor    : gpu_lidar, 360 samples, 10 Hz
    min_angle : -π rad     max_angle : +π rad
    min_range : 0.12 m     max_range : 30.0 m

INDEX ↔ DIRECTION MAPPING
--------------------------
    In the ROS LaserScan message the angle at index i is:
        angle_i = angle_min + i * angle_increment
              = -π + i * (2π / 360)

    Standard ROS convention: angle 0 = FORWARD, +ve = LEFT (CCW from above).
    Therefore:
        index 0   → angle = -π       → BACKWARD
        index 90  → angle = -π/2     → RIGHT
        index 180 → angle = 0        → FORWARD
        index 270 → angle = +π/2     → LEFT

    NOTE: The robot description document lists index 0 = FORWARD.
    If the robot behaves backwards in testing, swap by setting
    parameter  lidar_forward_index = 0  (document convention)
    vs the default lidar_forward_index = 180  (physics convention).

OPENING DETECTION ALGORITHM
-----------------------------
1.  Replace inf/NaN readings with max_range.
2.  Smooth the scan with a 5-sample median filter to kill noise spikes.
3.  Scan all 360°.  An "open gap" starts where range > OPEN_THRESHOLD and
    ends where range drops back below OPEN_THRESHOLD.
4.  For each gap: compute angular width and estimate physical width at the
    mean distance:
        physical_width ≈ 2 * mean_dist * tan(angular_width / 2)
5.  Mark the gap as PASSABLE if:
        physical_width ≥ PASSABILITY_FACTOR × ROBOT_WIDTH
    PASSABILITY_FACTOR = 1.50  (leaves margin)
    ROBOT_WIDTH        = 0.350 m  (conservative — wider than wheel separation)
    → passable threshold ≈ 0.525 m  (allows the 580 mm tight gap)

U-SHAPE DETECTION (closed row / dead end)
------------------------------------------
    A closed row presents walls on three sides.  Detected when:
        forward_dist < CLOSED_ROW_DEPTH  AND
        left_dist    < CLOSED_ROW_DEPTH  AND
        right_dist   < CLOSED_ROW_DEPTH
    where CLOSED_ROW_DEPTH = 2.5 m  (tuned to row dimensions ≈ 700-1000 mm).
    The back direction must be open (robot came from there) so we also check:
        back_dist > forward_dist  (just a sanity check, not strictly required)

TOPICS
------
  SUB  /r1_mini/lidar    sensor_msgs/LaserScan
  PUB  /lidar/analysis   mahe_nav_interfaces/LidarAnalysis
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan
from mahe_nav_interfaces.msg import LidarAnalysis


# ── Robot geometry ──────────────────────────────────────────────────────────────
ROBOT_WIDTH_M       = 0.350   # metres — conservative body width
PASSABILITY_FACTOR  = 1.20    # Lowered from 1.5 for testing — easier gap detection
PASSABLE_THRESHOLD  = PASSABILITY_FACTOR * ROBOT_WIDTH_M   # 0.42 m

# ── Thresholds ──────────────────────────────────────────────────────────────────
OPEN_THRESHOLD      = 0.80    # metres — range > this considered "open"
CLOSED_ROW_DEPTH    = 2.5     # metres — rows shorter than this are "closed"

# ── Smoothing kernel width (samples) ────────────────────────────────────────────
MEDIAN_KERNEL       = 5


class LidarAnalyzerNode(Node):

    def __init__(self):
        super().__init__('lidar_analyzer')

        # ── Parameters ─────────────────────────────────────────────────────────
        self.declare_parameter('lidar_forward_index', 180)  # physics-correct default
        self.declare_parameter('open_threshold',      OPEN_THRESHOLD)
        self.declare_parameter('closed_row_depth',    CLOSED_ROW_DEPTH)
        self.declare_parameter('robot_width_m',       ROBOT_WIDTH_M)
        self.declare_parameter('passability_factor',  PASSABILITY_FACTOR)

        fwd = self.get_parameter('lidar_forward_index').value
        self.FORWARD  = fwd % 360
        self.LEFT     = (fwd + 90)  % 360
        self.BACKWARD = (fwd + 180) % 360
        self.RIGHT    = (fwd + 270) % 360

        self.open_thresh  = self.get_parameter('open_threshold').value
        self.closed_depth = self.get_parameter('closed_row_depth').value
        robot_w           = self.get_parameter('robot_width_m').value
        p_factor          = self.get_parameter('passability_factor').value
        self.passable_thr = p_factor * robot_w

        self.get_logger().info(
            f'LiDAR analyzer: FORWARD={self.FORWARD} LEFT={self.LEFT} '
            f'RIGHT={self.RIGHT} BACK={self.BACKWARD}  '
            f'passable_thr={self.passable_thr:.3f} m')

        # ── Publisher / Subscriber ──────────────────────────────────────────────
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.pub = self.create_publisher(LidarAnalysis, '/lidar/analysis', 10)

        self.sub = self.create_subscription(
            LaserScan, '/r1_mini/lidar', self._scan_cb, sensor_qos)

    # ── Scan callback ───────────────────────────────────────────────────────────
    def _scan_cb(self, msg: LaserScan):
        n   = len(msg.ranges)
        raw = np.array(msg.ranges, dtype=np.float32)

        # Replace inf / NaN / below-min with max_range so gaps read as "open"
        bad = np.logical_or(np.isinf(raw), np.isnan(raw))
        bad = np.logical_or(bad, raw < msg.range_min)
        raw[bad] = msg.range_max

        # Median smoothing (circular — wraps at 0/359)
        k      = MEDIAN_KERNEL
        padded = np.concatenate([raw[-(k//2):], raw, raw[:k//2]])
        smooth = np.array([
            float(np.median(padded[i:i+k]))
            for i in range(n)
        ], dtype=np.float32)

        # ── Sector distances (Continuous Zones) ─────────────────────────────────
        # Find absolute minimum distance in angular sectors to cover all blind spots
        def sector_min(start_idx: int, end_idx: int) -> float:
            if start_idx <= end_idx:
                return float(np.min(smooth[start_idx:end_idx+1]))
            else: # Wraps around
                return float(np.min(np.concatenate([smooth[start_idx:], smooth[:end_idx+1]])))

        # Convert degrees to index offsets based on actual resolution
        # Avoid zero division if increment is bizarre
        ang_inc = msg.angle_increment if msg.angle_increment > 0 else 0.01745
        def deg2idx(deg_val):
            return int(abs(math.radians(deg_val)) / ang_inc)

        o_20  = deg2idx(20)   # ±20° for Front
        o_110 = deg2idx(110)  # Sweep left/right past 90° for full flanks
        
        idx_fwd_right = (self.FORWARD - o_20) % n
        idx_fwd_left  = (self.FORWARD + o_20) % n
        
        idx_left_front = (self.FORWARD + o_20 + 1) % n
        idx_left_back  = (self.FORWARD + o_110) % n
        
        idx_rgt_front = (self.FORWARD - o_110) % n
        idx_rgt_back  = (self.FORWARD - (o_20 + 1)) % n

        idx_back_left = (self.FORWARD + o_110 + 1) % n
        idx_back_rgt  = (self.FORWARD - (o_110 + 1)) % n

        fwd_d  = sector_min(idx_fwd_right, idx_fwd_left)
        left_d = sector_min(idx_left_front, idx_left_back)
        rgt_d  = sector_min(idx_rgt_front, idx_rgt_back)
        bck_d  = sector_min(idx_back_left, idx_back_rgt)

        # ── Threat angles: WHERE inside each sector is the closest object ───────
        def sector_argmin_angle(start_idx: int, end_idx: int) -> float:
            """Returns relative angle (rad, 0=fwd, +ve=left) of closest point."""
            if start_idx <= end_idx:
                local_idx = int(np.argmin(smooth[start_idx:end_idx+1]))
                abs_idx   = start_idx + local_idx
            else:
                combined   = np.concatenate([smooth[start_idx:], smooth[:end_idx+1]])
                local_idx  = int(np.argmin(combined))
                abs_idx    = (start_idx + local_idx) % n
            raw_ang   = msg.angle_min + abs_idx * msg.angle_increment
            fwd_ang   = msg.angle_min + self.FORWARD * msg.angle_increment
            rel       = raw_ang - fwd_ang
            while rel >  math.pi: rel -= 2*math.pi
            while rel < -math.pi: rel += 2*math.pi
            return rel

        fwd_threat_ang  = sector_argmin_angle(idx_fwd_right,  idx_fwd_left)
        left_threat_ang = sector_argmin_angle(idx_left_front, idx_left_back)
        rgt_threat_ang  = sector_argmin_angle(idx_rgt_front,  idx_rgt_back)
        bck_threat_ang  = sector_argmin_angle(idx_back_left,  idx_back_rgt)

        # ── U-shape / closed-row detection ─────────────────────────────────────
        is_u = (fwd_d  < self.closed_depth and
                left_d < self.closed_depth and
                rgt_d  < self.closed_depth)

        # ── Lateral corridor width estimate ────────────────────────────────────
        lateral_width = left_d + rgt_d + ROBOT_WIDTH_M

        # ── Opening detection ───────────────────────────────────────────────────
        opening_angles  = []
        opening_widths  = []
        opening_pass    = []
        opening_shapes  = []

        in_gap      = False
        gap_start   = 0
        gap_dists   = []

        for i in range(n):
            r = float(smooth[i])
            if r > self.open_thresh:
                if not in_gap:
                    in_gap    = True
                    gap_start = i
                    gap_dists = []
                gap_dists.append(r)
            else:
                if in_gap:
                    in_gap = False
                    self._record_gap(
                        gap_start, i - 1, gap_dists, n, msg,
                        opening_angles, opening_widths, opening_pass, opening_shapes, smooth)

        # Handle gap that wraps around index 359 → 0
        if in_gap:
            self._record_gap(
                gap_start, n - 1, gap_dists, n, msg,
                opening_angles, opening_widths, opening_pass, opening_shapes, smooth)

        # ── Best opening (widest passable gap) ─────────────────────────────────
        best_idx = -1
        best_w   = 0.0
        for j, (w, p) in enumerate(zip(opening_widths, opening_pass)):
            if p and w > best_w:
                best_w   = w
                best_idx = j

        # ── Build and publish message ───────────────────────────────────────────
        out                         = LidarAnalysis()
        out.header                  = msg.header
        out.forward_dist            = fwd_d
        out.left_dist               = left_d
        out.right_dist              = rgt_d
        out.back_dist               = bck_d
        out.forward_threat_angle    = fwd_threat_ang
        out.left_threat_angle       = left_threat_ang
        out.right_threat_angle      = rgt_threat_ang
        out.back_threat_angle       = bck_threat_ang
        out.is_u_shape              = is_u
        out.corridor_width_lateral  = lateral_width
        out.opening_angles_rad      = [float(a) for a in opening_angles]
        out.opening_widths_m        = [float(w) for w in opening_widths]
        out.opening_passable        = list(opening_pass)
        out.opening_shapes          = list(opening_shapes)
        out.best_opening_idx        = best_idx
        out.no_forward_passage      = (fwd_d < ROBOT_WIDTH_M * 1.2)

        self.pub.publish(out)

        self.get_logger().debug(
            f'fwd={fwd_d:.2f}  l={left_d:.2f}  r={rgt_d:.2f}  b={bck_d:.2f}  '
            f'U={is_u}  openings={len(opening_angles)}  best={best_idx}')

    # ── Record a detected gap ───────────────────────────────────────────────────
    def _record_gap(self, start_idx, end_idx, gap_dists, n_samples, msg,
                    angles_out, widths_out, pass_out, shapes_out, smooth):
        # ── Boundary points: the actual solid edges of the gap ──────────────────
        idx_right_bound = (start_idx - 1) % n_samples
        idx_left_bound  = (end_idx + 1)   % n_samples

        R1 = float(smooth[idx_right_bound])   # right-side wall distance
        R2 = float(smooth[idx_left_bound])    # left-side wall distance

        n_gap     = end_idx - start_idx + 1   # raw count, no modulo
        ang_width = (n_gap + 2) * msg.angle_increment

        # ── Law of Cosines: true Euclidean gap width ────────────────────────────
        phys_width = math.sqrt(R1**2 + R2**2 - 2.0 * R1 * R2 * math.cos(ang_width))
        passable   = phys_width >= self.passable_thr

        # ── Gap Shape Classification (inspired by research paper Dmax method) ───
        # Build (X, Y) Cartesian coordinates for interior gap points
        gap_indices = [(start_idx + k) % n_samples for k in range(n_gap)]
        if len(gap_indices) >= 3:
            # Straight-line baseline between the two boundary wall points
            X1 = R1 * math.cos(msg.angle_min + idx_right_bound * msg.angle_increment)
            Y1 = R1 * math.sin(msg.angle_min + idx_right_bound * msg.angle_increment)
            X2 = R2 * math.cos(msg.angle_min + idx_left_bound  * msg.angle_increment)
            Y2 = R2 * math.sin(msg.angle_min + idx_left_bound  * msg.angle_increment)
            baseline_len = math.hypot(X2 - X1, Y2 - Y1) + 1e-6

            # Dmax: how far do the interior gap ray endpoints deviate from the baseline?
            dmax = 0.0
            for gi in gap_indices:
                ang_i = msg.angle_min + gi * msg.angle_increment
                Xp = smooth[gi] * math.cos(ang_i)
                Yp = smooth[gi] * math.sin(ang_i)
                # Perpendicular distance from point to the baseline
                d_perp = abs((Y2 - Y1)*Xp - (X2 - X1)*Yp + X2*Y1 - Y2*X1) / baseline_len
                dmax = max(dmax, d_perp)

            S = baseline_len
            if dmax >= 0.2 * S:
                shape = 'curved'      # Interior arcs inward — curved/round gap
            elif abs(R1 - R2) > 0.4:  # Boundary walls very uneven — L-shaped entry
                shape = 'asymmetric'
            else:
                shape = 'flat'        # Straight uniform opening
        else:
            shape = 'flat'            # Too few points to classify

        # ── Centre angle of gap ─────────────────────────────────────────────────
        centre_i   = (start_idx + end_idx) // 2
        centre_ang = msg.angle_min + centre_i * msg.angle_increment
        fwd_ang    = msg.angle_min + self.FORWARD * msg.angle_increment
        rel_angle  = centre_ang - fwd_ang
        while rel_angle >  math.pi: rel_angle -= 2*math.pi
        while rel_angle < -math.pi: rel_angle += 2*math.pi

        angles_out.append(rel_angle)
        widths_out.append(phys_width)
        pass_out.append(passable)
        shapes_out.append(shape)

        self.get_logger().debug(
            f'  gap: angle={math.degrees(rel_angle):.1f}°  '
            f'width={phys_width:.3f}m  shape={shape}  passable={passable}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarAnalyzerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
