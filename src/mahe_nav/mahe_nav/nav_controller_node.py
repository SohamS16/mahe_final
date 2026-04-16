import math
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from mahe_nav_interfaces.msg import ArucoDetection, SignDetection, LidarAnalysis

class State(Enum):
    INIT = auto()
    EXPLORE_FORWARD = auto()
    U_TURN_RECOVERY = auto()
    MAZE_NAVIGATE = auto()
    T_JUNCTION_SPIN = auto()
    BACKTRACK = auto()

# --- Configuration Constants ---
# Exploration speeds
V_MAX        = 0.30   # Reduced from 0.40 — safer for tight unknown arenas
V_MIN        = 0.10   # Creep speed for corners and narrow gaps
V_MAZE       = 0.07   # Extra-cautious for 580mm passages

# Distance thresholds
SLOWDOWN_DIST   = 1.5   # Start slowing down 1.5m before front wall
STOP_DIST       = 0.35  # Hard stop if front closer than this (was 0.28)
MAZE_STOP_DIST  = 0.25  # Tighter stop inside narrow corridors (was 0.20)

# Angular rates
W_TURN       = 0.50   # Gentle spin rate — avoids flinging into opposite wall
W_SCAN       = 0.35   # Very slow rotation when scanning for a gap

# ── ArUco Marker World Positions ────────────────────────────────────────────
# Map each marker ID to its known (x, y) world position in the odom/world frame.
# These MUST be measured from your arena layout / sim world file.
# Units: metres. Update these values to match your actual arena.
MARKER_WORLD_POS = {
    0: (4.526, 2.893),   # Maze entry
    1: (10.541, 3.593),  # Maze exit
    2: (8.828, 0.542),   # T-junction
    3: (7.328, 1.368),   # Goal
}

class NavControllerNode(Node):
    def __init__(self):
        super().__init__('nav_controller')
        
        self.state = State.INIT
        self.pose_x = self.pose_y = self.pose_yaw = 0.0
        self.spawn_yaw = None
        self.lidar = None
        self.sign = None
        self.aruco_seen = set()
        
        # Physical Thresholds
        self.PASSABLE_THR = 0.525
        
        # Stuck Prevention
        self.last_progress_time = time.time()
        self.last_pos = (0.0, 0.0)
        self.u_turn_entry_yaw = 0.0
        self.stuck_recovery_until = 0.0   # timestamp: hold recovery until this time

        # ROS 2 Interfaces
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        
        best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, '/odom_fused', self._odom_cb, 10)
        self.create_subscription(LidarAnalysis, '/lidar/analysis', self._lidar_cb, best_effort)
        self.create_subscription(SignDetection, '/sign_detection', self._sign_cb, best_effort)
        self.create_subscription(ArucoDetection, '/aruco/detections', self._aruco_cb, best_effort)

        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('Reactive Nav Controller: Unified Version Active')

    def _odom_cb(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0-2.0*(q.y*q.y + q.z*q.z))

    def _lidar_cb(self, msg): 
        self.lidar = msg

    def _sign_cb(self, msg):
        self.sign = msg

    def _aruco_cb(self, msg: 'ArucoDetection'):
        mid = msg.marker_id
        self.aruco_seen.add(mid)

        # ══════════════════════════════════════════════════════════════════════
        #  ARUCO POSE BEHAVIOUR — flip between two modes by commenting the flag
        # ══════════════════════════════════════════════════════════════════════
        #
        #  CASE 1 — OBSERVE ONLY (safe default)
        #    • ArUco pose is computed and printed in the log.
        #    • pose_x / pose_y are NOT changed.
        #    • Robot continues relying on EKF (odom + IMU) as normal.
        #    • Use this first — verify the corrected values look sensible.
        #
        #    To activate: COMMENT OUT the line below  →  # ARUCO_POSE_OVERRIDE = True
        #
        #  CASE 2 — TRUST ARUCO (override EKF position)
        #    • When a known marker is seen, pose_x / pose_y are overwritten with
        #      the ArUco-derived world position. EKF yaw is still kept.
        #    • Only safe if MARKER_WORLD_POS coords are verified against the SDF.
        #
        #    To activate: UNCOMMENT the line below  →  ARUCO_POSE_OVERRIDE = True
        #
        # ──────────────────────────────────────────────────────────────────────
        ARUCO_POSE_OVERRIDE = True        # ← Comment this line for CASE 1
        #                                 # ← Uncomment this line for CASE 2
        # ══════════════════════════════════════════════════════════════════════

        if mid in MARKER_WORLD_POS and msg.distance > 0.05:
            mx, my = MARKER_WORLD_POS[mid]
            d  = float(msg.distance)
            β  = float(msg.bearing_angle_rad)
            θ  = self.pose_yaw               # current EKF yaw — always trusted

            # Direction robot→marker in world frame
            world_angle = θ + β

            # Robot world position back-computed from known marker position
            corrected_x = mx - d * math.cos(world_angle)
            corrected_y = my - d * math.sin(world_angle)

            drift = math.hypot(corrected_x - self.pose_x, corrected_y - self.pose_y)

            if 'ARUCO_POSE_OVERRIDE' in dir() and ARUCO_POSE_OVERRIDE:
                # ── CASE 2: Override poses ─────────────────────────────────
                self.pose_x = corrected_x
                self.pose_y = corrected_y
                mode_tag = 'OVERRIDE APPLIED'
            else:
                # ── CASE 1: Observe only ───────────────────────────────────
                mode_tag = 'OBSERVE ONLY (no override)'

            self.get_logger().info(
                f'[ARUCO {mid}] {mode_tag} | '
                f'EKF=({self.pose_x:.3f}, {self.pose_y:.3f}) | '
                f'ArUco corrected=({corrected_x:.3f}, {corrected_y:.3f}) | '
                f'Drift={drift:.3f}m | '
                f'dist={d:.2f}m bearing={math.degrees(β):.1f}°'
            )

        # ── State machine transition on first detection ────────────────────────
        if msg.first_detection:
            if mid == 0:
                self._transition(State.MAZE_NAVIGATE)



    def _control_loop(self):
        if not self.lidar: return
        
        if self.state == State.INIT:
            self.spawn_yaw = self.pose_yaw
            return self._transition(State.EXPLORE_FORWARD)

        # ── Stuck recovery hold window ──────────────────────────────────────────
        now = time.time()
        if now < self.stuck_recovery_until:
            # ── Prefer forward gaps over reversing ─────────────────────────────
            # Check for any passable gap in the front hemisphere (±90°)
            front_gaps = [
                (angle, width)
                for angle, width, passable in zip(
                    self.lidar.opening_angles_rad,
                    self.lidar.opening_widths_m,
                    self.lidar.opening_passable)
                if passable and abs(angle) <= math.radians(90)
            ]

            if front_gaps:
                # A driveable path exists in the front hemisphere — take it
                best = max(front_gaps, key=lambda x: x[1])
                self.get_logger().info(
                    f"RECOVERY: Front gap found at {math.degrees(best[0]):.1f}° "
                    f"width={best[1]:.2f}m — driving through")
                self.stuck_recovery_until = 0.0  # cancel hold, go explore
                self._move(V_MIN, best[0])
            elif self.lidar.back_dist > 0.4:
                # Front blocked, rear is clear — reverse with slight turn
                self._move(-0.20, 0.4)
            else:
                # Completely enclosed — spin to expose a new direction
                self._move(0.0, W_TURN)
            return  # always skip explore during recovery window


        self._check_stuck()

        if self.state == State.EXPLORE_FORWARD:
            self._handle_explore()
        elif self.state == State.U_TURN_RECOVERY:
            self._handle_u_turn()
        elif self.state == State.MAZE_NAVIGATE:
            self._handle_maze()

    def _handle_explore(self):
        # ── Dynamic speed: ramp down as front wall approaches ──────────────────
        speed_factor = min(1.0, self.lidar.forward_dist / SLOWDOWN_DIST)
        v_cmd = max(V_MIN, V_MAX * speed_factor)

        # ── Dead-End Detection: DISABLED for testing ───────────────────────────
        # U-turn recovery temporarily removed to isolate exploration + collision
        # avoidance behaviour. Re-enable once basic movement is validated.
        # if self.lidar.is_u_shape and self.lidar.forward_dist < 0.60:
        #     self.u_turn_entry_yaw = self.pose_yaw
        #     return self._transition(State.U_TURN_RECOVERY)

        # ── Soft Cornering: anticipate L-shaped turns early ────────────────────
        # If front is narrowing AND one flank is wide open, drop speed NOW
        if self.lidar.forward_dist < 2.0:
            is_l_left  = self.lidar.left_dist > 1.5 and self.lidar.right_dist < 1.2
            is_l_right = self.lidar.right_dist > 1.5 and self.lidar.left_dist < 1.2
            if is_l_left or is_l_right:
                v_cmd = V_MIN

        # ── Gap Steering: two-tier forward-first selection ─────────────────────
        target_angle, _ = self._select_best_gap()

        if target_angle is not None:
            self._move(v_cmd, target_angle)
        else:
            # No passable gap in any direction — creep + scan
            # Stuck detector triggers proper recovery if this persists 9s
            self._move(V_MIN, W_SCAN)

    def _select_best_gap(self):
        """
        Two-tier gap selection for robust exploration:

        Tier 1 — Forward cone (±35°): If any passable gap exists near straight
                  ahead, pick the WIDEST of those. Keeps the robot centred in
                  corridors and prevents it swerving into irrelevant side gaps.

        Tier 2 — Side gaps: Only if NO forward gap exists at all, pick the
                  widest passable gap from any direction. Handles T-junctions
                  and corners where forward is genuinely blocked.

        Returns: (target_angle, gap_width) or (None, None) if no passable gap.
        """
        FORWARD_CONE = math.radians(35)  # ±35° considered "forward"

        forward_gaps = []
        side_gaps    = []

        for angle, width, passable in zip(
            self.lidar.opening_angles_rad,
            self.lidar.opening_widths_m,
            self.lidar.opening_passable
        ):
            if not passable:
                continue
            if abs(angle) <= FORWARD_CONE:
                forward_gaps.append((angle, width))
            else:
                side_gaps.append((angle, width))

        # Tier 1: widest gap roughly ahead — stay centred in corridor
        if forward_gaps:
            best = max(forward_gaps, key=lambda x: x[1])
            return best[0], best[1]

        # Tier 2: forward blocked — turn toward widest reachable side gap
        if side_gaps:
            best = max(side_gaps, key=lambda x: x[1])
            return best[0], best[1]

        return None, None


    def _handle_u_turn(self):
        self._move(0.0, W_TURN)
        
        # Oscillation Fix: Must turn at least 150 degrees (~2.6 rad) from entry
        yaw_diff = abs(self.pose_yaw - self.u_turn_entry_yaw)
        if yaw_diff > math.pi:
            yaw_diff = 2.0 * math.pi - yaw_diff
            
        if self.lidar.forward_dist > 1.5 and yaw_diff > 2.6: 
            self._transition(State.EXPLORE_FORWARD)

    def _handle_maze(self):
        # Logic for 580mm Narrow Gap
        valid_paths = []
        for i in range(len(self.lidar.opening_angles_rad)):
            if self.lidar.opening_widths_m[i] > self.PASSABLE_THR:
                valid_paths.append({
                    'angle': self.lidar.opening_angles_rad[i],
                    'width': self.lidar.opening_widths_m[i]
                })

        if not valid_paths:
            return self._move(0.0, W_TURN)

        # Centering Strategy: Pick the path closest to current heading
        best_path = min(valid_paths, key=lambda x: abs(x['angle']))
        
        # Steering gain for precision in tight 580mm path
        steering_gain = 1.3 if best_path['width'] < 0.65 else 1.0
        self._move(V_MAZE, best_path['angle'] * steering_gain)

    def _move(self, v, w):
        if not self.lidar: return

        # --- Soft Corridor Centering ---
        # Only apply when actually moving forward to avoid contaminating spin commands
        if v > 0.05 and self.lidar.left_dist < 2.0 and self.lidar.right_dist < 2.0:
            diff = self.lidar.right_dist - self.lidar.left_dist
            w += diff * 0.4  # Gentle gain — won't fight the gap-steering

        # --- Wall Repulsion (Emergency) ---
        # Fires only when physically dangerously close to a wall
        # High gain override to prevent scraping
        REPULSION_DIST = 0.30  # Slightly wider than before for simulator
        if self.lidar.left_dist < REPULSION_DIST:
            w -= 0.55  # Push right
        if self.lidar.right_dist < REPULSION_DIST:
            w += 0.55  # Push left

        # Clamp: prevent combined corrections from swerving into opposite wall
        w = max(min(w, W_TURN * 1.5), -W_TURN * 1.5)

        # --- Emergency Brake & Corner Clearance ---
        current_limit = MAZE_STOP_DIST if self.state == State.MAZE_NAVIGATE else STOP_DIST
        if self.lidar.forward_dist < current_limit:
            if self.lidar.back_dist > 0.3:
                v = -0.06 # Soft reverse clearance
            else:
                v = 0.0
        
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def _transition(self, new_state):
        self.get_logger().info(f"Transition: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.last_progress_time = time.time()

    def _check_stuck(self):
        now = time.time()
        dist = math.hypot(self.pose_x - self.last_pos[0], self.pose_y - self.last_pos[1])

        if dist > 0.10:
            # Made meaningful progress — reset tracker
            self.last_pos = (self.pose_x, self.pose_y)
            self.last_progress_time = now
        elif (now - self.last_progress_time) > 9.0:
            # Stuck for 9 seconds — trigger timed recovery
            self.get_logger().warn("STUCK: Triggering 2.5s recovery hold")
            self.stuck_recovery_until = now + 2.5
            self.last_progress_time = now

def main(args=None):
    rclpy.init(args=args)
    node = NavControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
