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
ACTION_DISTANCE_THRESHOLD = 0.8
APPROACH_TRIGGER_DIST     = 1.2   # [FIX 6] Trigger APPROACH_TAG earlier to account for momentum


# === PHASE 4: FSM ===
class State(Enum):
    EXPLORE       = auto()
    APPROACH_TAG  = auto()
    TAG_ACTION    = auto()
    FOLLOW_GREEN  = auto()
    FOLLOW_BLUE   = auto()
    FOLLOW_ORANGE = auto()
    UTURN         = auto()
    RECOVERY      = auto()
    HALT          = auto()


# ── Exploration speeds ───────────────────────────────────────────────────────
V_MAX  = 0.30   # m/s — normal forward speed
V_MIN  = 0.10   # m/s — creep

# ── Distance thresholds ──────────────────────────────────────────────────────
SLOWDOWN_DIST  = 1.5    # m — begin ramping down
STOP_DIST      = 0.35   # m — hard-stop ahead

# ── Exploration angular rates ────────────────────────────────────────────────
W_TURN = 0.50   # rad/s — gentle spin for gap-finding
W_SCAN = 0.35   # rad/s — very slow scan

# Turn angular rates
W_SIGN_TURN = 0.70   # rad/s
W_SIGN_SPIN = 0.40   # rad/s  [FIX 4] Reduced from 0.60 — pi/0.40 = ~7.85s for true 180deg

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

        # === PHASE 4: FSM ===
        self.state = State.EXPLORE
        self.state_start_time = time.time()
        self.logged_tags = set()

        self.pose_x = self.pose_y = self.pose_yaw = 0.0
        self.lidar = None
        self.latest_sign = None
        self.latest_aruco = None

        self.PASSABLE_THR = 0.525

        # Turn execution / state trackers
        self.active_turn_cmd  = "NONE"   # NONE, LEFT, RIGHT
        self.uturn_post_state = State.EXPLORE
        self.indicator_blue_arrow = False
        self.last_pos = (0.0, 0.0)
        self.last_progress_time = time.time()

        # [FIX 2] Proper halt flag in __init__ instead of hasattr()
        self.has_halted = False

        # ROS 2 interfaces
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        best_effort  = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry,       '/odom_fused',       self._odom_cb,  10)
        self.create_subscription(LidarAnalysis,  '/lidar/analysis',   self._lidar_cb, best_effort)
        self.create_subscription(SignDetection,  '/sign_detection',   self._sign_cb,  best_effort)
        self.create_subscription(ArucoDetection, '/aruco/detections', self._aruco_cb, best_effort)

        self.create_timer(0.1, self._fsm_tick)
        self.get_logger().info('NavController: Phase 4 FSM Active')


    # ── Callbacks (Pure State Modifiers) ─────────────────────────────────────

    def _odom_cb(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.pose_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _lidar_cb(self, msg):
        self.lidar = msg

    def _sign_cb(self, msg):
        self.latest_sign = msg
        sign_type = msg.sign_type.upper() if msg.sign_type else "NONE"

        # Phase 4 Rule 1: Massive Red tile check via pixel width (>70px ~ 5000px sq)
        if sign_type == "GOAL" and (msg.pixel_width * msg.pixel_width > 5000):
            if self.state != State.HALT:
                self.get_logger().info("━━━ [STATUS] MASSIVE RED TILE DETECTED ━━━")
                self._transition(State.HALT)

    def _aruco_cb(self, msg: ArucoDetection):
        self.latest_aruco = msg


    # ── FSM Dispatcher & Helpers ─────────────────────────────────────────────

    def _transition(self, new_state: State):
        self.get_logger().info(f'Transition: {self.state.name} → {new_state.name}')
        self.state = new_state
        self.state_start_time = time.time()
        self.last_progress_time = time.time()
        self.active_turn_cmd = "NONE"

    def _publish_vel(self, v: float, w: float):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.pub_cmd.publish(msg)

    def _fsm_tick(self):
        # [FIX 3] Expanded exclusion list — states that must run even without LiDAR data
        LIDAR_FREE_STATES = (State.HALT, State.APPROACH_TAG, State.TAG_ACTION, State.UTURN)
        if not self.lidar and self.state not in LIDAR_FREE_STATES:
            return

        match self.state:
            case State.EXPLORE:       self._handle_explore()
            case State.APPROACH_TAG:  self._handle_approach_tag()
            case State.TAG_ACTION:    self._handle_tag_action()
            case State.FOLLOW_GREEN:  self._handle_follow_green()
            case State.FOLLOW_BLUE:   self._handle_follow_blue()
            case State.FOLLOW_ORANGE: self._handle_follow_orange()
            case State.UTURN:         self._handle_uturn()
            case State.RECOVERY:      self._handle_recovery()
            case State.HALT:          self._handle_halt()


    # ── State Handlers ───────────────────────────────────────────────────────

    def _handle_explore(self):
        # Watchdog for stuck prevention
        now  = time.time()
        dist = math.hypot(self.pose_x - self.last_pos[0], self.pose_y - self.last_pos[1])
        if dist > 0.10:
            self.last_pos = (self.pose_x, self.pose_y)
            self.last_progress_time = now
        elif (now - self.last_progress_time) > 9.0:
            self.get_logger().warn('STUCK: Timeout in EXPLORE')
            self._transition(State.RECOVERY)
            return

        # General Exploration behavior
        speed_factor = min(1.0, self.lidar.forward_dist / SLOWDOWN_DIST)
        v_cmd = max(V_MIN, V_MAX * speed_factor)

        # [FIX 6 Correction] Check ArUco for smooth deceleration
        if self.latest_aruco:
            d   = float(self.latest_aruco.distance)
            mid = self.latest_aruco.marker_id
            if mid <= 4:
                if d <= ACTION_DISTANCE_THRESHOLD:
                    # Phase 4 Rule 3: Publish zero immediately on entering APPROACH_TAG
                    self._publish_vel(0.0, 0.0)
                    self._transition(State.APPROACH_TAG)
                    return
                elif d <= APPROACH_TRIGGER_DIST:
                    v_cmd = V_MIN

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

    def _move(self, v: float, w: float):
        if v > 0.05 and self.lidar.left_dist < 2.0 and self.lidar.right_dist < 2.0:
            diff  = self.lidar.right_dist - self.lidar.left_dist
            w    += diff * 0.4
        REPULSION_DIST = 0.30
        if self.lidar.left_dist  < REPULSION_DIST: w -= 0.55
        if self.lidar.right_dist < REPULSION_DIST: w += 0.55
        w = max(min(w, W_TURN * 1.5), -W_TURN * 1.5)
        if self.lidar.forward_dist < STOP_DIST:
            v = -0.06 if self.lidar.back_dist > 0.3 else 0.0
        self._publish_vel(v, w)

    def _handle_approach_tag(self):
        elapsed = time.time() - self.state_start_time

        # Phase 4 Rule 3: Keep robot stopped during entire stabilization window
        self._publish_vel(0.0, 0.0)

        if elapsed < 1.5:
            return  # waiting for camera reading to stabilize

        if not self.latest_aruco:
            self._transition(State.EXPLORE)
            return

        mid = self.latest_aruco.marker_id
        d   = float(self.latest_aruco.distance)

        # [FIX 5] Clear latest_aruco after consuming so stale data is never re-processed
        self.latest_aruco = None

        # Only act if marker is within execution distance
        if d > ACTION_DISTANCE_THRESHOLD:
            self.get_logger().info(f'[ARUCO] ID {mid} detected at {d:.2f}m — too far. Resuming approach.')
            self._transition(State.EXPLORE)
            return

        # Phase 4 Rule 4: Re-detection guard ONLY if distance <= threshold
        if mid in self.logged_tags:
            if d <= ACTION_DISTANCE_THRESHOLD:
                self.get_logger().info(f'[ARUCO] ID {mid} already logged! Forcing 180 U-Turn!')
                self.uturn_post_state = State.EXPLORE
                self._transition(State.UTURN)
            else:
                self.get_logger().info(f'[ARUCO] ID {mid} already logged but far ({d:.2f}m). Ignoring.')
                self._transition(State.EXPLORE)
            return

        # New tag — log and dispatch
        self.logged_tags.add(mid)
        self.get_logger().info(f'[ARUCO] Logged new ID {mid} at {d:.2f}m — executing action!')

        # ID Dispatch Rules
        if mid == 0:
            self.active_turn_cmd = "RIGHT"
            self._transition(State.TAG_ACTION)
        elif mid == 1:
            self.active_turn_cmd = "LEFT"
            self._transition(State.TAG_ACTION)
        elif mid == 2:
            self._transition(State.FOLLOW_GREEN)
        elif mid == 3:
            self.uturn_post_state = State.FOLLOW_BLUE
            self._transition(State.UTURN)
        elif mid == 4:
            self._transition(State.FOLLOW_ORANGE)
        else:
            self._transition(State.EXPLORE)

    def _handle_tag_action(self):
        elapsed = time.time() - self.state_start_time

        # Watchdog timeout: TAG_ACTION -> RECOVERY
        if elapsed > 10.0:
            self.get_logger().warn("TAG_ACTION Timeout! Recovering.")
            self._transition(State.RECOVERY)
            return

        # Phase 4 Rule 2: Non-blocking turn execution (4.0 seconds)
        if elapsed < 4.0:
            if self.active_turn_cmd == "LEFT":
                self._publish_vel(0.0, +W_SIGN_TURN)
            elif self.active_turn_cmd == "RIGHT":
                self._publish_vel(0.0, -W_SIGN_TURN)
            else:
                self._transition(State.EXPLORE)
        else:
            self._publish_vel(0.0, 0.0)
            self._transition(State.EXPLORE)

    def _handle_follow_green(self):
        elapsed = time.time() - self.state_start_time

        # Check for next ArUco tag
        if (self.latest_aruco and
                float(self.latest_aruco.distance) <= ACTION_DISTANCE_THRESHOLD and
                self.latest_aruco.marker_id not in self.logged_tags):
            self._publish_vel(0.0, 0.0)
            self._transition(State.APPROACH_TAG)
            return

        # [FIX 1] FOLLOW states use gap navigation directly, NOT _handle_explore()
        # This avoids firing the EXPLORE stuck watchdog while in a FOLLOW state
        if elapsed > 120.0:
            self.get_logger().warn("FOLLOW_GREEN timeout! Falling back to EXPLORE.")
            self._transition(State.EXPLORE)
            return

        if not self.lidar:
            return
        target_angle, _ = self._select_best_gap()
        if target_angle is not None:
            self._move(V_MIN, target_angle)
        else:
            self._publish_vel(V_MIN, W_SCAN)

    def _handle_follow_blue(self):
        elapsed = time.time() - self.state_start_time

        # Check for next ArUco tag
        if (self.latest_aruco and
                float(self.latest_aruco.distance) <= ACTION_DISTANCE_THRESHOLD and
                self.latest_aruco.marker_id not in self.logged_tags):
            self._publish_vel(0.0, 0.0)
            self._transition(State.APPROACH_TAG)
            return

        # [FIX 1] FOLLOW states use gap navigation directly, NOT _handle_explore()
        if elapsed > 120.0:
            self.get_logger().warn("FOLLOW_BLUE timeout! Falling back to EXPLORE.")
            self._transition(State.EXPLORE)
            return

        if not self.lidar:
            return
        target_angle, _ = self._select_best_gap()
        if target_angle is not None:
            self._move(V_MIN, target_angle)
        else:
            self._publish_vel(V_MIN, W_SCAN)

    def _handle_follow_orange(self):
        elapsed = time.time() - self.state_start_time

        # Check for next ArUco tag
        if (self.latest_aruco and
                float(self.latest_aruco.distance) <= ACTION_DISTANCE_THRESHOLD and
                self.latest_aruco.marker_id not in self.logged_tags):
            self._publish_vel(0.0, 0.0)
            self._transition(State.APPROACH_TAG)
            return

        # [FIX 1] FOLLOW states use gap navigation directly, NOT _handle_explore()
        if elapsed > 120.0:
            self.get_logger().warn("FOLLOW_ORANGE timeout! Falling back to EXPLORE.")
            self._transition(State.EXPLORE)
            return

        if not self.lidar:
            return
        target_angle, _ = self._select_best_gap()
        if target_angle is not None:
            self._move(V_MIN, target_angle)
        else:
            self._publish_vel(V_MIN, W_SCAN)

    def _handle_uturn(self):
        elapsed = time.time() - self.state_start_time
        self.indicator_blue_arrow = True

        # Watchdog timeout: UTURN -> RECOVERY
        if elapsed > 15.0:
            self.get_logger().warn("UTURN Timeout! Recovering.")
            self._transition(State.RECOVERY)
            return

        # [FIX 4] W_SIGN_SPIN=0.40 rad/s × ~7.85s = pi rad = true 180deg
        if elapsed < 7.9:
            self._publish_vel(0.0, +W_SIGN_SPIN)
        else:
            self._publish_vel(0.0, 0.0)
            self.indicator_blue_arrow = False
            self._transition(self.uturn_post_state)

    def _handle_recovery(self):
        elapsed = time.time() - self.state_start_time

        if elapsed < 1.0:
            self._publish_vel(-0.20, 0.0)    # 1s reverse
        elif elapsed < 3.0:
            self._publish_vel(0.0, 0.4)      # 2s rotate / scan
        else:
            self._publish_vel(0.0, 0.0)
            self.get_logger().info("Recovery complete, returning to EXPLORE")
            self._transition(State.EXPLORE)

    def _handle_halt(self):
        # Stop the robot completely every tick
        self._publish_vel(0.0, 0.0)

        # [FIX 2] Use self.has_halted flag (set in __init__) instead of hasattr()
        # Prevents SystemExit from being raised repeatedly at 10Hz
        if not self.has_halted:
            self.has_halted = True

            with open('/tmp/mahe_nav_mission_log.txt', 'w') as f:
                f.write("MISSION COMPLETE\n")
                f.write(f"Logged Tags: {self.logged_tags}\n")
                f.write(f"Final Position: ({self.pose_x:.2f}, {self.pose_y:.2f})\n")

            self.get_logger().info("━━━ MISSION COMPLETE: Logs written. Node Halt. ━━━")
            self.destroy_node()
            rclpy.try_shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = NavControllerNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('SystemExit raised. Graceful Stop.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()