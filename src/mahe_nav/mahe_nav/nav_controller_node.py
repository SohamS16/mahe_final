import math
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from mahe_nav_interfaces.msg import ArucoDetection, SignDetection, LidarAnalysis

# --- FIXED: [BUG 2] ---
ACTION_DISTANCE_THRESHOLD = 1.25


class State(Enum):
    INIT            = auto()
    EXPLORE_FORWARD = auto()
    SIGN_EXECUTE    = auto()   # Executing a sign manoeuvre
    GOAL_REACHED    = auto()   # Terminal: mission complete
    U_TURN_RECOVERY = auto()
    MAZE_NAVIGATE   = auto()
    T_JUNCTION_SPIN = auto()
    BACKTRACK       = auto()
    FOLLOW_GREEN    = auto()
    FOLLOW_ORANGE   = auto()


# ── Exploration speeds ───────────────────────────────────────────────────────
V_MAX  = 0.30   # m/s — normal forward speed
V_MIN  = 0.10   # m/s — creep
V_MAZE = 0.07   # m/s — tight corridor

# ── Distance thresholds ──────────────────────────────────────────────────────
SLOWDOWN_DIST  = 1.5    # m — begin ramping down
STOP_DIST      = 0.35   # m — hard-stop ahead
MAZE_STOP_DIST = 0.25   # m — tighter in maze

# ── Exploration angular rates ────────────────────────────────────────────────
W_TURN = 0.50   # rad/s — gentle spin for gap-finding
W_SCAN = 0.35   # rad/s — very slow scan

# ── Sign Turn Parameters ─────────────────────────────────────────────────────
#
#  ALL turns are PURE PIVOTS (linear.x = 0) — tightest possible turning radius.
#
#  Differential-drive cmd_vel semantics (ROS REP-103):
#    linear.x   = forward body speed  [m/s]
#    angular.z  = yaw rate            [rad/s],  +ve = CCW = robot-body LEFT
#
#  How the diff-drive controller maps these to wheel speeds:
#    v_right = linear.x + (angular.z * track_width / 2)
#    v_left  = linear.x - (angular.z * track_width / 2)
#
#  Pure pivot  (linear.x = 0):
#    → right wheel and left wheel spin at equal speed in OPPOSITE directions
#    → zero net translation — robot rotates on the spot
#
#  LEFT  sign  (90° CCW):  angular.z = +W_SIGN_TURN  → left wheel back,  right wheel fwd
#  RIGHT sign  (90° CW):   angular.z = -W_SIGN_TURN  → right wheel back, left wheel fwd
#  INPLACE     (180° CCW): angular.z = +W_SIGN_SPIN  → same pure pivot, larger target
#
W_SIGN_TURN = 0.70   # rad/s  — pivot rate for LEFT / RIGHT  (90°  target)
W_SIGN_SPIN = 0.60   # rad/s  — spin  rate for INPLACE_ROTATION (180° target)

# Debounce / confirmation
SIGN_CONFIRM_N      = 2      # consecutive detections required before acting
SIGN_DEBOUNCE_SEC   = 2.0    # second window for consecutive detection
SIGN_COOLDOWN_SEC   = 4.0    # suppress re-trigger after execution
SIGN_MIN_CONFIDENCE = 0.45

# ── LIDAR Safety During Sign Execution ──────────────────────────────────────
# Layer 1 — pre-turn  : checked before starting pivot
TURN_AHEAD_SIDE_THRESH  = 0.40   # m — side clearance required before starting
# Layer 2 — mid-turn  : checked every control tick during pivot
TURN_DURING_FWD_ABORT   = 0.20   # m — hard-abort pivot if forward drops below this
TURN_DURING_FWD_SLOW    = 0.30   # m — halve angular rate if forward drops below this
# Post-turn
CLEAR_PATH_THRESH       = 0.45   # m — forward must exceed this to resume exploration

# ── ArUco Marker World Positions ─────────────────────────────────────────────
MARKER_WORLD_POS = {
    0: (1.796,  0.975),
    1: (0.450, -0.441),
    2: (1.320, -1.341),
    3: (1.796, -1.875),
    4: (-0.450, 1.341),
}


class NavControllerNode(Node):
    def __init__(self):
        super().__init__('nav_controller')

        self.state     = State.INIT
        self.pose_x    = self.pose_y = self.pose_yaw = 0.0
        self.spawn_yaw = None
        self.lidar     = None
        self.sign      = None
        self.aruco_seen = set()

        # --- FIXED: STEP 1 - REMOVE SEQUENCE TRACKING ENTIRELY ---
        
        self.latest_scan = None # For BUG 4

        self.PASSABLE_THR = 0.525

        # Stuck prevention
        self.last_progress_time   = time.time()
        self.last_pos             = (0.0, 0.0)
        self.u_turn_entry_yaw     = 0.0
        self.stuck_recovery_until = 0.0

        # ── Sign execution state ─────────────────────────────────────────────
        self.sign_turn_type       = "NONE"
        self.sign_turn_w          = 0.0      # angular.z to command during pivot
        self.sign_turn_target_rad = 0.0      # target yaw change (magnitude, radians)
        self.sign_turn_start_yaw  = 0.0
        self.sign_turn_started    = False    # True after first 5° of rotation

        # Debounce / confirmation
        self.last_sign_type     = "NONE"
        self.sign_confirm_count = 0
        self.last_sign_time     = 0.0
        self.sign_cooldown_until = 0.0

        # ROS 2 interfaces
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        best_effort  = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry,       '/odom_fused',       self._odom_cb,  10)
        self.create_subscription(LidarAnalysis,  '/lidar/analysis',   self._lidar_cb, best_effort)
        self.create_subscription(SignDetection,  '/sign_detection',   self._sign_cb,  best_effort)
        self.create_subscription(ArucoDetection, '/aruco/detections', self._aruco_cb, best_effort)
        # --- FIXED: [BUG 4] ---
        self.create_subscription(LaserScan,      '/scan',             self._scan_cb,  best_effort)

        self.create_timer(0.05, self._control_loop)
        self.get_logger().info('NavController: Sign-Aware Differential-Drive Version Active')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _scan_cb(self, msg):
        # --- FIXED: [BUG 4] ---
        self.latest_scan = msg

    def _lidar_cb(self, msg):
        self.lidar = msg

    def _sign_cb(self, msg):
        """Debounce and confirmation logic before queuing a sign action."""
        self.sign = msg

        # Ignore new signs while already executing one, or at goal
        if self.state in (State.SIGN_EXECUTE, State.GOAL_REACHED):
            return

        now = time.time()
        if now < self.sign_cooldown_until:
            return

        sign_type = msg.sign_type.upper() if msg.sign_type else "NONE"
        if sign_type == "NONE" or msg.confidence < SIGN_MIN_CONFIDENCE:
            return

        # Consecutive detection counter (reset if sign changed or gap too long)
        if sign_type == self.last_sign_type and (now - self.last_sign_time) < SIGN_DEBOUNCE_SEC:
            self.sign_confirm_count += 1
        else:
            self.sign_confirm_count = 1
            self.last_sign_type = sign_type

        self.last_sign_time = now

        if self.sign_confirm_count >= SIGN_CONFIRM_N:
            self._queue_sign_action(sign_type)
            self.sign_confirm_count = 0

    def _aruco_cb(self, msg: 'ArucoDetection'):
        mid = msg.marker_id
        
        # --- FIXED: STEP 1 - REMAP ARUCO IDs (IDs ARE NOW 0 TO 4) ---
        if mid > 4:
            return

        self.aruco_seen.add(mid)
        d  = float(msg.distance)
        β  = float(msg.bearing_angle_rad)

        ARUCO_POSE_OVERRIDE = True  # ← comment for observe-only mode

        if mid in MARKER_WORLD_POS and d > 0.05:
            if d <= ACTION_DISTANCE_THRESHOLD:
                mx, my = MARKER_WORLD_POS[mid]
                θ  = self.pose_yaw
                world_angle = θ + β
                corrected_x = mx - d * math.cos(world_angle)
                corrected_y = my - d * math.sin(world_angle)
                
                if 'ARUCO_POSE_OVERRIDE' in dir() and ARUCO_POSE_OVERRIDE:
                    self.pose_x = corrected_x
                    self.pose_y = corrected_y

        cmd_map = {
            0: "Take left",
            1: "Take right",
            2: "Start following green",
            3: "Take U-turn",
            4: "Start following orange"
        }
        cmd_name = cmd_map.get(mid, "Unknown")
        status_str = "Unknown"

        # --- FIXED: STEP 2 - SIMPLE EXECUTION LOGIC ---
        if d > ACTION_DISTANCE_THRESHOLD:
            status_str = "Approaching"
            self.get_logger().info(f"[ARUCO {mid}] Detected at {d:.2f}m - Too far, approaching...")
        else:
            status_str = "Executing"
            self.get_logger().info(f"[ARUCO {mid}] In range at {d:.2f}m - Executing: {cmd_name}")

        # --- FIXED: STEP 4 - VERBOSE PRINT ON DETECTION ---
        verbose_msg = (
            f"\n----------------------------------\n"
            f"ARUCO DETECTED\n"
            f"  ID       : {mid}\n"
            f"  Command  : {cmd_name}\n"
            f"  Distance : {d:.2f}m\n"
            f"  Bearing  : {math.degrees(β):.1f}°\n"
            f"  Status   : {status_str}\n"
            f"----------------------------------"
        )
        self.get_logger().info(verbose_msg)

        if status_str == "Executing":
            self.execute_marker_command(mid)

    def check_lidar_clearance(self, direction, scan_msg):
        # --- FIXED: [BUG 4] ---
        if scan_msg is None:
            return False
        ranges = scan_msg.ranges
        n = len(ranges)
        if direction == 'LEFT':
            cone = ranges[int(n*0.1):int(n*0.4)]
        elif direction == 'RIGHT':
            cone = ranges[int(n*0.6):int(n*0.9)]
        elif direction == 'U_TURN':
            cone = ranges[int(n*0.4):int(n*0.6)]
        else:
            return False
        return min(cone) > 0.45

    def execute_marker_command(self, marker_id: int):
        """Routes sequence-validated ArUco marker ID."""
        action_executed = False
        match marker_id:
            case 0:
                # --- FIXED: STEP 5 - LIDAR SAFETY CHECK ---
                if self.check_lidar_clearance('LEFT', self.latest_scan):
                    action_executed = self._queue_sign_action("LEFT")
                else:
                    self.get_logger().warn(f"[ARUCO {marker_id}] LIDAR BLOCKED on LEFT. Holding position.")
            case 1:
                # --- FIXED: STEP 5 - LIDAR SAFETY CHECK ---
                if self.check_lidar_clearance('RIGHT', self.latest_scan):
                    action_executed = self._queue_sign_action("RIGHT")
                else:
                    self.get_logger().warn(f"[ARUCO {marker_id}] LIDAR BLOCKED on RIGHT. Holding position.")
            case 2:
                self._transition(State.FOLLOW_GREEN)
                action_executed = True
            case 3:
                # --- FIXED: STEP 5 - LIDAR SAFETY CHECK ---
                if self.check_lidar_clearance('U_TURN', self.latest_scan):
                    action_executed = self._queue_sign_action("INPLACE_ROTATION")
                else:
                    self.get_logger().warn(f"[ARUCO {marker_id}] LIDAR BLOCKED on U_TURN. Holding position.")
            case 4:
                self._transition(State.FOLLOW_ORANGE)
                action_executed = True

        # --- FIXED: STEP 1 - REMOVE SEQUENCE TRACKING ENTIRELY ---
        # Sequence tracking removed as requested.

    # ── Sign Action ──────────────────────────────────────────────────────────

    def _queue_sign_action(self, sign_type: str) -> bool:
        """
        Set up a sign manoeuvre.
        
        ALL turns are pure pivots: linear.x = 0, angular.z = ±W.
        Differential-drive wheel mapping (for physical understanding/tuning):
            v_right =  angular.z * track/2
            v_left  = -angular.z * track/2

        LEFT  (90° CCW): angular.z = +W_SIGN_TURN → right fwd, left backward
        RIGHT (90° CW) : angular.z = −W_SIGN_TURN → left  fwd, right backward
        INPLACE (180°) : angular.z = +W_SIGN_SPIN → CCW spin-on-spot
        """
        self.get_logger().info(f'[SIGN] Confirmed: {sign_type}')

        if sign_type == "LEFT":
            if self.lidar and self.lidar.left_dist < TURN_AHEAD_SIDE_THRESH:
                self.get_logger().warn(
                    f'[SIGN] LEFT blocked — left_dist={self.lidar.left_dist:.2f}m < '
                    f'{TURN_AHEAD_SIDE_THRESH}m — aborting')
                return False
            self._start_pivot(w=+W_SIGN_TURN, target_deg=90.0, sign_type=sign_type)

        elif sign_type == "RIGHT":
            if self.lidar and self.lidar.right_dist < TURN_AHEAD_SIDE_THRESH:
                self.get_logger().warn(
                    f'[SIGN] RIGHT blocked — right_dist={self.lidar.right_dist:.2f}m < '
                    f'{TURN_AHEAD_SIDE_THRESH}m — aborting')
                return False
            self._start_pivot(w=-W_SIGN_TURN, target_deg=90.0, sign_type=sign_type)

        elif sign_type == "INPLACE_ROTATION":
            self._start_pivot(w=+W_SIGN_SPIN, target_deg=180.0, sign_type=sign_type)

        elif sign_type == "STOP":
            self._move_sign(0.0, 0.0)
            self.stuck_recovery_until = time.time() + 3.0
            self.sign_cooldown_until  = time.time() + SIGN_COOLDOWN_SEC
            self.get_logger().info('[SIGN] STOP — holding 3 s')

        elif sign_type == "GOAL":
            self._move_sign(0.0, 0.0)
            self.sign_cooldown_until = time.time() + 99999.0
            self._transition(State.GOAL_REACHED)
            self.get_logger().info('━━━ [STATUS] MISSION COMPLETE — GOAL REACHED ━━━')

        elif sign_type == "FORWARD":
            self.sign_cooldown_until = time.time() + SIGN_COOLDOWN_SEC
            self.get_logger().info('[SIGN] FORWARD — continuing')

        self.last_sign_time = time.time()
        return True

    def _start_pivot(self, w: float, target_deg: float, sign_type: str):
        """Arm SIGN_EXECUTE for a pivot turn. Logs LIDAR context for tuning."""
        self.sign_turn_type       = sign_type
        self.sign_turn_w          = w
        self.sign_turn_target_rad = math.radians(target_deg)
        self.sign_turn_start_yaw  = self.pose_yaw
        self.sign_turn_started    = False

        if self.lidar:
            self.get_logger().info(
                f'[SIGN] Pivot start: {sign_type} | w={w:+.2f} rad/s | '
                f'target={target_deg:.0f}° | '
                f'fwd={self.lidar.forward_dist:.2f} '
                f'left={self.lidar.left_dist:.2f} '
                f'right={self.lidar.right_dist:.2f} m')

        self._transition(State.SIGN_EXECUTE)

    # ── Control Loop ─────────────────────────────────────────────────────────

    def _control_loop(self):
        if not self.lidar:
            return

        if self.state == State.INIT:
            self.spawn_yaw = self.pose_yaw
            return self._transition(State.EXPLORE_FORWARD)

        # Terminal state — hold still forever
        if self.state == State.GOAL_REACHED:
            self._move_sign(0.0, 0.0)
            return

        # Stuck recovery hold window (shared with STOP sign hold)
        now = time.time()
        if now < self.stuck_recovery_until:
            front_gaps = [
                (angle, width)
                for angle, width, passable in zip(
                    self.lidar.opening_angles_rad,
                    self.lidar.opening_widths_m,
                    self.lidar.opening_passable)
                if passable and abs(angle) <= math.radians(90)
            ]
            if front_gaps:
                best = max(front_gaps, key=lambda x: x[1])
                self.stuck_recovery_until = 0.0
                self._move(V_MIN, best[0])
            elif self.lidar.back_dist > 0.4:
                self._move(-0.20, 0.4)
            else:
                self._move(0.0, W_TURN)
            return

        self._check_stuck()

        if self.state == State.SIGN_EXECUTE:
            self._handle_sign_execute()
        elif self.state == State.EXPLORE_FORWARD:
            self._handle_explore()
        elif self.state == State.U_TURN_RECOVERY:
            self._handle_u_turn()
        elif self.state == State.MAZE_NAVIGATE:
            self._handle_maze()

    # ── Sign Execute Handler ──────────────────────────────────────────────────

    def _handle_sign_execute(self):
        """
        Odom yaw-tracked pivot with 2-layer LIDAR safety:
          Layer 1 — pre-turn  (checked in _queue_sign_action before entry)
          Layer 2 — mid-turn  (checked here every 50 ms tick)

        Turn is complete when |yaw_turned| >= target − 3° tolerance.
        The robot then stops and waits for a clear forward path before resuming.
        """
        yaw_diff = self._angle_diff(self.pose_yaw, self.sign_turn_start_yaw)

        # Mark as properly started after first 5° so we don't false-complete
        if abs(yaw_diff) > math.radians(5.0):
            self.sign_turn_started = True

        # ── Completion check ─────────────────────────────────────────────────
        if self.sign_turn_started and \
                abs(yaw_diff) >= self.sign_turn_target_rad - math.radians(3.0):
            self._move_sign(0.0, 0.0)
            self.get_logger().info(
                f'[SIGN] {self.sign_turn_type} complete | '
                f'turned={math.degrees(yaw_diff):.1f}° '
                f'target={math.degrees(self.sign_turn_target_rad):.0f}°')
            self.sign_turn_type    = "NONE"
            self.sign_turn_started = False
            self.sign_cooldown_until = time.time() + SIGN_COOLDOWN_SEC

            if self.lidar and self.lidar.forward_dist > CLEAR_PATH_THRESH:
                self._transition(State.EXPLORE_FORWARD)
            else:
                # Path still blocked — trigger short recovery then explore
                fwd = self.lidar.forward_dist if self.lidar else -1.0
                self.get_logger().warn(
                    f'[SIGN] Path blocked after turn fwd={fwd:.2f}m — recovery hold')
                self.stuck_recovery_until = time.time() + 1.5
                self._transition(State.EXPLORE_FORWARD)
            return

        # ── Mid-turn LIDAR safety (Layer 2) ──────────────────────────────────
        if not self.lidar:
            self._move_sign(0.0, self.sign_turn_w)
            return

        fwd  = self.lidar.forward_dist
        back = self.lidar.back_dist

        # Hard abort — completely boxed, dangerous to continue
        if fwd < TURN_DURING_FWD_ABORT and back < 0.30:
            self.get_logger().warn(
                f'[SIGN] MID-TURN ABORT: fwd={fwd:.2f}m back={back:.2f}m')
            self._move_sign(0.0, 0.0)
            self.sign_turn_type    = "NONE"
            self.sign_turn_started = False
            self.stuck_recovery_until = time.time() + 1.5
            self.sign_cooldown_until  = time.time() + SIGN_COOLDOWN_SEC
            self._transition(State.EXPLORE_FORWARD)
            return

        # Soft slow — forward narrowing, halve angular rate
        w = self.sign_turn_w
        if fwd < TURN_DURING_FWD_SLOW:
            w *= 0.5
            self.get_logger().warn(
                f'[SIGN] MID-TURN SLOW: fwd={fwd:.2f}m → w={w:+.2f} rad/s')

        self._move_sign(0.0, w)

    # ── Exploration ───────────────────────────────────────────────────────────

    def _handle_explore(self):
        speed_factor = min(1.0, self.lidar.forward_dist / SLOWDOWN_DIST)
        v_cmd = max(V_MIN, V_MAX * speed_factor)

        if self.lidar.forward_dist < 2.0:
            is_l_left  = self.lidar.left_dist > 1.5 and self.lidar.right_dist < 1.2
            is_l_right = self.lidar.right_dist > 1.5 and self.lidar.left_dist < 1.2
            if is_l_left or is_l_right:
                v_cmd = V_MIN

        target_angle, _ = self._select_best_gap()
        if target_angle is not None:
            self._move(v_cmd, target_angle)
        else:
            self._move(V_MIN, W_SCAN)

    def _select_best_gap(self):
        FORWARD_CONE = math.radians(35)
        forward_gaps, side_gaps = [], []

        for angle, width, passable in zip(
                self.lidar.opening_angles_rad,
                self.lidar.opening_widths_m,
                self.lidar.opening_passable):
            if not passable:
                continue
            (forward_gaps if abs(angle) <= FORWARD_CONE else side_gaps).append((angle, width))

        if forward_gaps:
            return max(forward_gaps, key=lambda x: x[1])
        if side_gaps:
            return max(side_gaps, key=lambda x: x[1])
        return None, None

    def _handle_u_turn(self):
        self._move(0.0, W_TURN)
        yaw_diff = abs(self.pose_yaw - self.u_turn_entry_yaw)
        if yaw_diff > math.pi:
            yaw_diff = 2.0 * math.pi - yaw_diff
        if self.lidar.forward_dist > 1.5 and yaw_diff > 2.6:
            self._transition(State.EXPLORE_FORWARD)

    def _handle_maze(self):
        valid_paths = [
            {'angle': self.lidar.opening_angles_rad[i],
             'width': self.lidar.opening_widths_m[i]}
            for i in range(len(self.lidar.opening_angles_rad))
            if self.lidar.opening_widths_m[i] > self.PASSABLE_THR
        ]
        if not valid_paths:
            return self._move(0.0, W_TURN)
        best_path = min(valid_paths, key=lambda x: abs(x['angle']))
        steering_gain = 1.3 if best_path['width'] < 0.65 else 1.0
        self._move(V_MAZE, best_path['angle'] * steering_gain)

    # ── Movement Primitives ───────────────────────────────────────────────────

    def _move_sign(self, v: float, w: float):
        """
        cmd_vel for sign manoeuvres.
        Intentionally SKIPS corridor centering and wall repulsion — these
        corrections would corrupt a deliberate pivot angular.z command.
        Only the emergency forward brake remains active.
        """
        if not self.lidar:
            return
        # Emergency brake only
        current_limit = MAZE_STOP_DIST if self.state == State.MAZE_NAVIGATE else STOP_DIST
        if v > 0 and self.lidar.forward_dist < current_limit:
            v = 0.0
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def _move(self, v: float, w: float):
        """
        cmd_vel for exploration / recovery.
        Applies corridor centering, wall repulsion, and emergency brake.
        NOT used during SIGN_EXECUTE.
        """
        if not self.lidar:
            return

        # Soft corridor centering (forward motion only)
        if v > 0.05 and self.lidar.left_dist < 2.0 and self.lidar.right_dist < 2.0:
            diff = self.lidar.right_dist - self.lidar.left_dist
            w += diff * 0.4

        # Wall repulsion (emergency)
        REPULSION_DIST = 0.30
        if self.lidar.left_dist  < REPULSION_DIST:
            w -= 0.55
        if self.lidar.right_dist < REPULSION_DIST:
            w += 0.55

        w = max(min(w, W_TURN * 1.5), -W_TURN * 1.5)

        # Emergency brake
        current_limit = MAZE_STOP_DIST if self.state == State.MAZE_NAVIGATE else STOP_DIST
        if self.lidar.forward_dist < current_limit:
            v = -0.06 if self.lidar.back_dist > 0.3 else 0.0

        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _angle_diff(yaw_a: float, yaw_b: float) -> float:
        """Signed angular difference a − b, wrapped to (−π, +π]."""
        diff = yaw_a - yaw_b
        while diff >  math.pi: diff -= 2.0 * math.pi
        while diff <= -math.pi: diff += 2.0 * math.pi
        return diff

    def _transition(self, new_state: State):
        self.get_logger().info(f'Transition: {self.state.name} → {new_state.name}')
        self.state = new_state
        self.last_progress_time = time.time()

    def _check_stuck(self):
        now  = time.time()
        dist = math.hypot(self.pose_x - self.last_pos[0],
                          self.pose_y - self.last_pos[1])
        if dist > 0.10:
            self.last_pos = (self.pose_x, self.pose_y)
            self.last_progress_time = now
        elif (now - self.last_progress_time) > 9.0:
            self.get_logger().warn('STUCK: Triggering 2.5s recovery hold')
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
