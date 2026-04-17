#!/usr/bin/env python3
"""
cv_follower_node.py
ArtPark Floor Marker Tracking — Phase 2 CV Node
MIT Hackathon Round 3

Camera  : /r1_mini/camera/image_raw (640x480, forward-facing, slight downward pitch)
Odom    : /odom_fused (nav_msgs/Odometry) — for robot_yaw + angular_vel gate
ArUco   : /aruco/detections (mahe_nav_interfaces/ArucoDetection) — colour mode switching ONLY
Output  : /floor_marker/detection (mahe_nav_interfaces/FloorMarkerDetection)

DESIGN DECISION — always running:
  CV runs from node start. ArUco events switch the active colour (GREEN → BLUE → ORANGE).
  There is NO dormant gate. The blue pixel gate (>3000px) acts as the natural
  "logo in frame" guard — CV will not emit detections unless the logo is physically
  in view. This makes the node independently testable without ArUco.

All parameters are empirically validated from capture_log.csv (73 frames).
DO NOT change HSV thresholds or ROI values without re-running calibration.

KNOWN SIGN CONVENTION — READ BEFORE EDITING:
  Direction angle is computed as:  raw_angle = atan2(-dy, dx)
    where dx = petal_cx - logo_cx
          dy = petal_cy - logo_cy  (image Y increases DOWNWARD)
  Negating dy converts image coords to standard math convention (Y-up, CCW positive).
  This differs from the shorthand in the calibration report which wrote atan2(dy, dx).
  The negated form is physically correct. Verify on a tile with known orientation
  before deployment. One tile check is mandatory.
"""

import math
import collections
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge

from mahe_nav_interfaces.msg import ArucoDetection, FloorMarkerDetection


# ─────────────────────────────────────────────────────────────
#  VALIDATED CONSTANTS  (do not edit without re-running calibration)
# ─────────────────────────────────────────────────────────────

# ROI — percentage-based, floor-anchored (Test 2 validated)
# top=40% keeps wall clutter out; bottom=100% recovers the Y=441-479 band
# that the old fixed ROI (top=192, bottom=441) cut at x <= 1.2 m.
ROI_TOP_FRACTION    = 0.40
ROI_BOTTOM_FRACTION = 1.00

# Blue logo-presence gates (Test 3 validated against 73-frame CSV)
BLUE_GATE_LOGO      = 3000   # px — logo is in frame; gate all detection on this
BLUE_GATE_APPROACH  = 5000   # px — optimal detection range has begun (informational)

# HSV thresholds — all pixel-verified against actual Gazebo frames
# Green  : H≈59, S≈148, V≈199 measured in F10/F11
HSV_GREEN_LO  = np.array([45,  80,  80],  dtype=np.uint8)
HSV_GREEN_HI  = np.array([85,  255, 255], dtype=np.uint8)
# Orange : H≈14, S≈193, V≈246 measured in F09 (Test 1 — 10,180 px detected, H[10-15] confirmed)
HSV_ORANGE_LO = np.array([5,   120, 120], dtype=np.uint8)
HSV_ORANGE_HI = np.array([20,  255, 255], dtype=np.uint8)
# Blue   : logo centre — presence gate + centroid anchor + BLUE arrow mode
HSV_BLUE_LO   = np.array([100, 80,  80],  dtype=np.uint8)
HSV_BLUE_HI   = np.array([140, 255, 255], dtype=np.uint8)

# Morphology — removes single-pixel HSV noise after masking
MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

# Contour / geometry sanity guards
MIN_PETAL_AREA   = 200    # px² — below this is noise
MIN_PETAL_VECTOR = 15.0   # px  — too close to logo centre (probably the disc)
MAX_PETAL_VECTOR = 120.0  # px  — outside tile boundary (noise)

# Frame gate — reject frames captured during active rotation
MAX_ANGULAR_VEL  = 0.05   # rad/s — validated: 8/73 blind frames occur above this

# Majority vote parameters (architecture design)
VOTE_WINDOW      = 7      # rolling buffer length
VOTE_THRESHOLD   = 5      # min agreements to emit a direction
VOTE_TIMEOUT_SEC = 3.0    # seconds → emit DIRECTION_TIMEOUT, nav must retry

# ArUco tag IDs that switch colour mode
# These match the SDF-confirmed tag layout for this arena.
TAG_ID_GREEN  = 1    # Tag 1 → switch to GREEN following
TAG_ID_BLUE   = 3    # Tag 3 → switch to BLUE following (post U-turn)
TAG_ID_ORANGE = 2    # Tag 2 → switch to ORANGE following
TAG_DIST_GATE = 1.25 # metres — tag must be within this distance to trigger switch

# Direction bins — world angle is CCW from +X (standard ROS/math convention)
# Arena start: robot faces +Y (north). At yaw=0 robot faces east (+X).
DIRECTION_BINS = [
    ((-22.5,   22.5),  "RIGHT"),
    (( 22.5,   67.5),  "FORWARD_RIGHT"),
    (( 67.5,  112.5),  "FORWARD"),
    ((112.5,  157.5),  "FORWARD_LEFT"),
    ((157.5,  180.1),  "LEFT"),        # 180.1 closes the edge-case at exactly ±180°
    ((-180.1, -157.5), "LEFT"),
    ((-157.5, -112.5), "BACKWARD_LEFT"),
    ((-112.5,  -67.5), "BACKWARD"),
    (( -67.5,  -22.5), "BACKWARD_RIGHT"),
]

COLLAPSE_TO_4 = {
    "FORWARD":        "FORWARD",
    "FORWARD_RIGHT":  "FORWARD",
    "FORWARD_LEFT":   "FORWARD",
    "RIGHT":          "RIGHT",
    "BACKWARD_RIGHT": "BACKWARD",
    "BACKWARD":       "BACKWARD",
    "BACKWARD_LEFT":  "BACKWARD",
    "LEFT":           "LEFT",
}


# ─────────────────────────────────────────────────────────────
#  UTILITY FUNCTIONS
# ─────────────────────────────────────────────────────────────

def normalise_angle(a: float) -> float:
    """Normalise angle to (-180, 180]. Handles ±180 edge correctly."""
    a = a % 360.0          # 0 to 360
    if a > 180.0:
        a -= 360.0         # -180 to 180
    return a


def angle_to_direction_8(angle_deg: float) -> str:
    """Map a world angle (degrees, CCW from +X) to an 8-bin direction string."""
    a = normalise_angle(angle_deg)
    for (lo, hi), label in DIRECTION_BINS:
        if lo <= a < hi:
            return label
    return "UNKNOWN"


def angle_to_direction_4(angle_deg: float) -> str:
    return COLLAPSE_TO_4.get(angle_to_direction_8(angle_deg), "UNKNOWN")


def quat_to_yaw(q) -> float:
    """Extract yaw (radians) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ─────────────────────────────────────────────────────────────
#  NODE
# ─────────────────────────────────────────────────────────────

class CvFollowerNode(Node):

    def __init__(self):
        super().__init__("cv_follower_node")
        self.get_logger().info("CV Follower Node starting — always running, colour=GREEN default")

        self.bridge = CvBridge()

        # ── State ──────────────────────────────────────────────────────────
        # cv_mode starts GREEN — no dormant gate.
        # ArUco callback upgrades: GREEN → BLUE → ORANGE.
        self.cv_mode         = "GREEN"
        self.robot_yaw_rad   = 0.0
        self.angular_vel     = 0.0
        self.tile_count      = 0
        self.vote_buffer     = collections.deque(maxlen=VOTE_WINDOW)
        self.vote_start_time = None
        self.last_direction  = "NONE"

        # ── QoS ───────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # ── Subscriptions ─────────────────────────────────────────────────
        self.sub_image = self.create_subscription(
            Image,
            "/r1_mini/camera/image_raw",
            self._image_cb,
            qos_profile=sensor_qos
        )
        self.sub_odom = self.create_subscription(
            Odometry,
            "/odom_fused",
            self._odom_cb,
            qos_profile=sensor_qos
        )
        self.sub_aruco = self.create_subscription(
            ArucoDetection,
            "/aruco/detections",
            self._aruco_cb,
            10
        )

        # ── Publishers ────────────────────────────────────────────────────
        self.pub_detection = self.create_publisher(
            FloorMarkerDetection,
            "/floor_marker/detection",
            10
        )

        self.get_logger().info(
            "CV Follower Node ready. "
            "Subscribed: /r1_mini/camera/image_raw, /odom_fused, /aruco/detections"
        )

    # ──────────────────────────────────────────────────────────────────────
    #  CALLBACKS
    # ──────────────────────────────────────────────────────────────────────

    def _odom_cb(self, msg: Odometry):
        """Cache robot yaw and angular velocity from fused odometry."""
        self.robot_yaw_rad = quat_to_yaw(msg.pose.pose.orientation)
        self.angular_vel   = abs(msg.twist.twist.angular.z)

    def _aruco_cb(self, msg: ArucoDetection):
        """
        Switch colour mode based on detected tag ID and proximity.
        Does NOT activate/deactivate CV — only changes which colour to chase.
        Mode transitions are one-way: GREEN → BLUE → ORANGE.
        """
        if msg.distance > TAG_DIST_GATE:
            return

        tag_id = msg.tag_id

        if tag_id == TAG_ID_BLUE and self.cv_mode == "GREEN":
            self.cv_mode = "BLUE"
            self._reset_vote_buffer()
            self.get_logger().info(
                f"CV colour mode → BLUE (Tag {TAG_ID_BLUE} at {msg.distance:.2f}m)"
            )

        elif tag_id == TAG_ID_ORANGE and self.cv_mode == "BLUE":
            self.cv_mode = "ORANGE"
            self._reset_vote_buffer()
            self.get_logger().info(
                f"CV colour mode → ORANGE (Tag {TAG_ID_ORANGE} at {msg.distance:.2f}m)"
            )

        # TAG_ID_GREEN (Tag 1) — no action needed, already GREEN by default.
        # Log it for debugging only.
        elif tag_id == TAG_ID_GREEN:
            self.get_logger().info(
                f"Tag {TAG_ID_GREEN} seen at {msg.distance:.2f}m — already GREEN mode"
            )

    def _image_cb(self, msg: Image):
        """
        Main CV pipeline. Called on every camera frame.

        Stage 1 : Frame gate   — reject turns, enforce angular vel limit
        Stage 2 : ROI crop     — percentage-based, floor-anchored
        Stage 3 : Blue gate    — confirm logo is in frame before any colour work
        Stage 4 : Colour mask  — green / orange / blue depending on mode
        Stage 5 : Direction    — centroid vector → world angle
        Stage 6 : Vote + emit  — majority vote with timeout
        """

        # ── STAGE 1: Frame Gate ───────────────────────────────────────────
        # Reject frames during rotation — logo sweeps out of frame,
        # producing the 8/73 all-zero blind frames seen in CSV.
        if self.angular_vel > MAX_ANGULAR_VEL:
            return

        # ── Convert ───────────────────────────────────────────────────────
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return

        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # ── STAGE 2: ROI Crop ─────────────────────────────────────────────
        # top=40% excludes ceiling/wall clutter.
        # bottom=100% recovers Y=441-479 band cut by old fixed ROI (Test 2).
        roi_top = int(h * ROI_TOP_FRACTION)
        roi_hsv = hsv[roi_top:h, 0:w]

        # ── STAGE 3: Blue Logo-Presence Gate ─────────────────────────────
        # Blue is the most stable logo indicator (dominant pixel mass).
        # 3000px threshold separates logo-present from logo-absent across all 73 frames.
        blue_mask = cv2.inRange(roi_hsv, HSV_BLUE_LO, HSV_BLUE_HI)
        blue_px   = int(cv2.countNonZero(blue_mask))

        if blue_px < BLUE_GATE_LOGO:
            return  # Logo not in frame — skip, do NOT feed vote buffer

        # Log approach window opening (informational, does not gate anything)
        if blue_px >= BLUE_GATE_APPROACH:
            pass  # optimal detection range: x ≈ 1.5-2.5 m per CSV data

        # Find blue blob centroid — used as tile centre anchor for direction
        blue_contours, _ = cv2.findContours(
            blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not blue_contours:
            return

        largest_blue = max(blue_contours, key=cv2.contourArea)
        bm = cv2.moments(largest_blue)
        if bm["m00"] == 0:
            return

        logo_cx = bm["m10"] / bm["m00"]
        logo_cy = bm["m01"] / bm["m00"]

        # ── STAGE 4: Active Colour Extraction ─────────────────────────────
        active_colour = self.cv_mode  # GREEN | BLUE | ORANGE

        if active_colour == "GREEN":
            colour_lo, colour_hi = HSV_GREEN_LO,  HSV_GREEN_HI
        elif active_colour == "ORANGE":
            colour_lo, colour_hi = HSV_ORANGE_LO, HSV_ORANGE_HI
        elif active_colour == "BLUE":
            colour_lo, colour_hi = HSV_BLUE_LO,   HSV_BLUE_HI
        else:
            return

        colour_mask = cv2.inRange(roi_hsv, colour_lo, colour_hi)

        # BLUE mode only: mask out the inner disc so only outer petals remain.
        # The large central blue region would otherwise dominate the contour.
        if active_colour == "BLUE":
            colour_mask = self._mask_inner_logo(colour_mask, largest_blue, inner_fraction=0.45)

        # Morphological opening — erode then dilate to remove single-pixel HSV noise
        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_OPEN, MORPH_KERNEL)

        petal_px = int(cv2.countNonZero(colour_mask))
        if petal_px < MIN_PETAL_AREA:
            # Petal not visible — PETAL_NOT_VISIBLE, do not feed vote buffer
            return

        petal_contours, _ = cv2.findContours(
            colour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not petal_contours:
            return

        # BLUE mode only: prefer elongated contours (petals AR > 1.5 vs disc AR ≈ 1.0)
        if active_colour == "BLUE":
            petal_contours = self._filter_elongated(petal_contours, min_ar=1.5)
            if not petal_contours:
                return

        largest_petal = max(petal_contours, key=cv2.contourArea)
        pm = cv2.moments(largest_petal)
        if pm["m00"] == 0:
            return

        petal_cx = pm["m10"] / pm["m00"]
        petal_cy = pm["m01"] / pm["m00"]

        # ── STAGE 5: Direction Computation ───────────────────────────────
        # Both logo and petal centroids are in ROI-space coordinates.
        # dx/dy are relative — coordinate offset doesn't matter.
        dx = petal_cx - logo_cx
        dy = petal_cy - logo_cy   # positive = petal is BELOW logo in image = south

        vector_mag = math.sqrt(dx * dx + dy * dy)

        # Geometric sanity: reject implausible vectors
        # < 15px → petal centroid inside the central disc (noise)
        # > 120px → petal is outside the tile boundary (noise)
        if not (MIN_PETAL_VECTOR <= vector_mag <= MAX_PETAL_VECTOR):
            return

        # Negate dy to convert image Y-down → standard math Y-up convention.
        # atan2(-dy, dx) then gives CCW-from-+X angle in standard math coords.
        # IMPORTANT: validated formula in calibration report wrote atan2(dy,dx)
        # without negation — verify final direction on one known-orientation tile.
        raw_angle_rad = math.atan2(-dy, dx)
        raw_angle_deg = math.degrees(raw_angle_rad)

        # Add robot yaw to convert camera-relative angle → world angle.
        # robot_yaw_rad is CCW-positive from +X (standard ROS convention from /odom_fused).
        robot_yaw_deg   = math.degrees(self.robot_yaw_rad)
        world_angle_deg = normalise_angle(raw_angle_deg + robot_yaw_deg)

        direction_4 = angle_to_direction_4(world_angle_deg)
        if direction_4 == "UNKNOWN":
            return

        # ── STAGE 6: Majority Vote + Emit ─────────────────────────────────
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.vote_start_time is None:
            self.vote_start_time = now

        self.vote_buffer.append(direction_4)

        vote_counts = collections.Counter(self.vote_buffer)
        top_dir, top_count = vote_counts.most_common(1)[0]

        if top_count >= VOTE_THRESHOLD:
            # Strong consensus — emit direction
            self.tile_count += 1    # 1-indexed: first emit → tile 1
            confidence = top_count / VOTE_WINDOW
            self._emit_detection(
                direction   = top_dir,
                colour      = active_colour,
                confidence  = confidence,
                petal_cx    = petal_cx,
                petal_cy    = petal_cy,
                logo_cx     = logo_cx,
                logo_cy     = logo_cy,
                raw_angle   = raw_angle_deg,
                world_angle = world_angle_deg,
                blue_px     = blue_px,
                petal_px    = petal_px,
                header      = msg.header
            )
            self.get_logger().info(
                f"[TILE {self.tile_count}] {active_colour} → {top_dir} "
                f"(conf={confidence:.2f}, world_angle={world_angle_deg:.1f}°, "
                f"blue={blue_px}px, petal={petal_px}px)"
            )
            self._reset_vote_buffer()
            return

        # Timeout — buffer stalled, no consensus reached
        elapsed = now - self.vote_start_time
        if elapsed >= VOTE_TIMEOUT_SEC:
            self.get_logger().warn(
                f"DIRECTION_TIMEOUT after {elapsed:.1f}s — "
                f"best: {top_dir} ({top_count}/{VOTE_WINDOW}). Nav must retry."
            )
            self._emit_detection(
                direction   = "TIMEOUT",
                colour      = active_colour,
                confidence  = top_count / VOTE_WINDOW,
                petal_cx    = petal_cx,
                petal_cy    = petal_cy,
                logo_cx     = logo_cx,
                logo_cy     = logo_cy,
                raw_angle   = raw_angle_deg,
                world_angle = world_angle_deg,
                blue_px     = blue_px,
                petal_px    = petal_px,
                header      = msg.header
            )
            self._reset_vote_buffer()

    # ──────────────────────────────────────────────────────────────────────
    #  HELPERS
    # ──────────────────────────────────────────────────────────────────────

    def _reset_vote_buffer(self):
        self.vote_buffer.clear()
        self.vote_start_time = None

    def _emit_detection(self, direction, colour, confidence,
                        petal_cx, petal_cy, logo_cx, logo_cy,
                        raw_angle, world_angle, blue_px, petal_px,
                        header):
        msg = FloorMarkerDetection()
        msg.header          = header
        msg.colour          = colour
        msg.direction       = direction
        msg.confidence      = float(confidence)
        msg.tile_count      = self.tile_count   # 1-indexed after increment above
        msg.petal_cx        = float(petal_cx)
        msg.petal_cy        = float(petal_cy)
        msg.logo_cx         = float(logo_cx)
        msg.logo_cy         = float(logo_cy)
        msg.raw_angle_deg   = float(raw_angle)
        msg.world_angle_deg = float(world_angle)
        msg.blue_px         = blue_px
        msg.petal_px        = petal_px
        self.pub_detection.publish(msg)

    def _mask_inner_logo(self, mask, blue_contour, inner_fraction: float = 0.45):
        """
        Zero out the inner circular region of the logo bounding box.
        Prevents the large central blue disc from being detected as a petal
        during BLUE arrow mode.
        inner_fraction=0.45 → radius = 45% of half the bounding box side.
        """
        out = mask.copy()
        x, y, bw, bh = cv2.boundingRect(blue_contour)
        cx = x + bw // 2
        cy = y + bh // 2
        radius = int(min(bw, bh) * inner_fraction / 2)
        if radius > 5:
            cv2.circle(out, (cx, cy), radius, 0, thickness=-1)
        return out

    def _filter_elongated(self, contours, min_ar: float = 1.5):
        """
        Keep only contours with bounding-rect aspect ratio >= min_ar.
        Petals: AR > 1.5. Central disc: AR ≈ 1.0.
        Used exclusively in BLUE mode.
        """
        result = []
        for c in contours:
            if cv2.contourArea(c) < MIN_PETAL_AREA:
                continue
            _, _, cw, ch = cv2.boundingRect(c)
            if cw == 0 or ch == 0:
                continue
            ar = max(cw, ch) / min(cw, ch)
            if ar >= min_ar:
                result.append(c)
        return result


# ──────────────────────────────────────────────────────────────────────────
#  ENTRY POINT
# ──────────────────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = CvFollowerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
