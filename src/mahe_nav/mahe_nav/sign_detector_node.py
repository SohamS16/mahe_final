"""
MAHE UGV Navigation: Sign Detector Node (Phase 2 CV Floor Marker Tracking)
==========================================================================
Replaces the old sign detector logic with the validated computer vision pipeline
from cv_follower_node_corrected.py. Completely rewritten to process FloorMarkerDetection.
"""

import math
import time
from collections import deque, Counter
import numpy as np
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from mahe_nav_interfaces.msg import ArucoDetection, FloorMarkerDetection

# === PHASE 2 CV: CONSTANTS ===
ROI_TOP_FRACTION    = 0.40
ROI_BOTTOM_FRACTION = 1.00

BLUE_GATE_LOGO      = 3000
BLUE_GATE_APPROACH  = 5000

HSV_GREEN_LO  = np.array([45,  80,  80],  dtype=np.uint8)
HSV_GREEN_HI  = np.array([85,  255, 255], dtype=np.uint8)
HSV_ORANGE_LO = np.array([5,   120, 120], dtype=np.uint8)
HSV_ORANGE_HI = np.array([20,  255, 255], dtype=np.uint8)
HSV_BLUE_LO   = np.array([100, 80,  80],  dtype=np.uint8)
HSV_BLUE_HI   = np.array([140, 255, 255], dtype=np.uint8)

MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

MIN_PETAL_AREA   = 200
MIN_PETAL_VECTOR = 15.0
MAX_PETAL_VECTOR = 120.0

MAX_ANGULAR_VEL  = 0.05

# === PHASE 2 CV: Red Tile HALT Detection ===
RED_HSV_LOWER = np.array([0,   120, 120])
RED_HSV_UPPER = np.array([10,  255, 255])
RED_HSV_LOWER2 = np.array([170, 120, 120])
RED_HSV_UPPER2 = np.array([180, 255, 255])

VOTE_WINDOW      = 7
VOTE_THRESHOLD   = 5
VOTE_TIMEOUT_SEC = 3.0

TAG_ID_GREEN  = 1
TAG_ID_BLUE   = 3
TAG_ID_ORANGE = 2
TAG_DIST_GATE = 1.25

DIRECTION_BINS = [
    ((-22.5,   22.5),  "RIGHT"),
    (( 22.5,   67.5),  "FORWARD_RIGHT"),
    (( 67.5,  112.5),  "FORWARD"),
    ((112.5,  157.5),  "FORWARD_LEFT"),
    ((157.5,  180.1),  "LEFT"),
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

# === PHASE 2 CV: UTILITY FUNCTIONS ===
def normalise_angle(a: float) -> float:
    a = a % 360.0
    if a > 180.0:
        a -= 360.0
    return a

def angle_to_direction_8(angle_deg: float) -> str:
    a = normalise_angle(angle_deg)
    for (lo, hi), label in DIRECTION_BINS:
        if lo <= a < hi:
            return label
    return "UNKNOWN"

def angle_to_direction_4(angle_deg: float) -> str:
    return COLLAPSE_TO_4.get(angle_to_direction_8(angle_deg), "UNKNOWN")

# === PHASE 2 CV: CLASS DEFINITION ===
class SignDetectorNode(Node):

    def __init__(self):
        super().__init__('sign_detector')

        self.cv_mode = "GREEN"
        self.pose_yaw = 0.0
        self.angular_vel = 0.0
        self.tile_count = 0
        self.vote_buffer = deque(maxlen=VOTE_WINDOW)
        self.vote_start_time = None
        self.bridge = CvBridge()

        self.pub_floor = self.create_publisher(
            FloorMarkerDetection, '/floor_marker/detection', 10)

        sensor_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        self.create_subscription(
            Image, '/r1_mini/camera/image_raw', self._image_cb, sensor_qos)
        self.create_subscription(
            Odometry, '/odom_fused', self._odom_cb, sensor_qos)
        self.create_subscription(
            ArucoDetection, '/aruco/detections', self._aruco_cb, sensor_qos)

        self.get_logger().info("Sign Detector (CV Floor Marker Mode) Active")

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.pose_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.angular_vel = abs(msg.twist.twist.angular.z)

    def _aruco_cb(self, msg: ArucoDetection):
        if msg.distance <= TAG_DIST_GATE:
            # Check for tag 3 and distance <= ARUCO_SWITCH_DIST and cv_mode == "GREEN" 
            # Note: the user wrote TAG_ID_BLUE (which is 3) rather than raw number sometimes, we will use id
            if msg.marker_id == 3 and self.cv_mode == "GREEN":
                self.cv_mode = "BLUE"
                self.vote_buffer.clear()
                self.vote_start_time = None
                self.get_logger().info("CV mode switched to BLUE")
            elif msg.marker_id == 2 and self.cv_mode == "BLUE":
                self.cv_mode = "ORANGE"
                self.vote_buffer.clear()
                self.vote_start_time = None
                self.get_logger().info("CV mode switched to ORANGE")

    def _image_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self._pipeline(bgr, msg.header)
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")

    # === PHASE 2 CV: PIPELINE ===
    def _pipeline(self, bgr, header):
        if self.angular_vel > MAX_ANGULAR_VEL:
            return

        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        roi_top = int(h * ROI_TOP_FRACTION)
        roi_hsv = hsv[roi_top:h, 0:w]

        # === PHASE 2 CV: Red Tile HALT Detection ===
        # Run independently of cv_mode — checks for massive red floor region
        red_mask  = cv2.inRange(roi_hsv, RED_HSV_LOWER,  RED_HSV_UPPER)
        red_mask2 = cv2.inRange(roi_hsv, RED_HSV_LOWER2, RED_HSV_UPPER2)
        red_mask  = cv2.bitwise_or(red_mask, red_mask2)
        red_px    = int(np.count_nonzero(red_mask))

        if red_px > 8000:
            halt_msg = FloorMarkerDetection()
            halt_msg.header.stamp = self.get_clock().now().to_msg()
            halt_msg.colour    = "RED"
            halt_msg.direction = "HALT"
            halt_msg.confidence = 1.0
            halt_msg.blue_px   = 0
            halt_msg.petal_px  = red_px
            self.pub_floor.publish(halt_msg)
            self.get_logger().info(f'[CV] RED TILE DETECTED — red_px={red_px} — publishing HALT')
            return   # skip rest of pipeline this frame

        blue_mask = cv2.inRange(roi_hsv, HSV_BLUE_LO, HSV_BLUE_HI)
        blue_px   = int(cv2.countNonZero(blue_mask))

        if blue_px < BLUE_GATE_LOGO:
            return

        blue_contours, _ = cv2.findContours(
            blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if not blue_contours:
            return

        largest_blue = max(blue_contours, key=cv2.contourArea)
        bm = cv2.moments(largest_blue)
        if bm["m00"] == 0:
            return

        logo_cx = bm["m10"] / bm["m00"]
        logo_cy = bm["m01"] / bm["m00"]

        active_colour = self.cv_mode

        if active_colour == "GREEN":
            colour_lo, colour_hi = HSV_GREEN_LO,  HSV_GREEN_HI
        elif active_colour == "ORANGE":
            colour_lo, colour_hi = HSV_ORANGE_LO, HSV_ORANGE_HI
        elif active_colour == "BLUE":
            colour_lo, colour_hi = HSV_BLUE_LO,   HSV_BLUE_HI
        else:
            return

        colour_mask = cv2.inRange(roi_hsv, colour_lo, colour_hi)

        if active_colour == "BLUE":
            colour_mask = self._mask_inner_logo(colour_mask, largest_blue, inner_fraction=0.45)

        colour_mask = cv2.morphologyEx(colour_mask, cv2.MORPH_OPEN, MORPH_KERNEL)

        petal_px = int(cv2.countNonZero(colour_mask))
        if petal_px < MIN_PETAL_AREA:
            return

        petal_contours, _ = cv2.findContours(
            colour_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
        if not petal_contours:
            return

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

        dx = petal_cx - logo_cx
        dy = petal_cy - logo_cy

        vector_mag = math.sqrt(dx * dx + dy * dy)
        if not (MIN_PETAL_VECTOR <= vector_mag <= MAX_PETAL_VECTOR):
            return

        raw_angle_rad = math.atan2(-dy, dx)
        raw_angle_deg = math.degrees(raw_angle_rad)

        robot_yaw_deg   = math.degrees(self.pose_yaw)
        world_angle_deg = normalise_angle(raw_angle_deg + robot_yaw_deg)

        direction_4 = angle_to_direction_4(world_angle_deg)
        if direction_4 == "UNKNOWN":
            return

        now = time.time()

        if self.vote_start_time is None:
            self.vote_start_time = now

        self.vote_buffer.append(direction_4)

        vote_counts = Counter(self.vote_buffer)
        top_dir, top_count = vote_counts.most_common(1)[0]

        if top_count >= VOTE_THRESHOLD:
            self.tile_count += 1
            confidence = top_count / VOTE_WINDOW
            self._emit(
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
                header      = header
            )
            return

        elapsed = now - self.vote_start_time
        if elapsed >= VOTE_TIMEOUT_SEC:
            self._emit(
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
                header      = header
            )

    # === PHASE 2 CV: HELPERS ===
    def _emit(self, direction, colour, confidence, petal_cx, petal_cy, logo_cx, logo_cy, raw_angle, world_angle, blue_px, petal_px, header):
        msg = FloorMarkerDetection()
        msg.header          = header
        msg.colour          = colour
        msg.direction       = direction
        msg.confidence      = float(confidence)
        msg.tile_count      = int(self.tile_count)
        msg.petal_cx        = float(petal_cx)
        msg.petal_cy        = float(petal_cy)
        msg.logo_cx         = float(logo_cx)
        msg.logo_cy         = float(logo_cy)
        msg.raw_angle_deg   = float(raw_angle)
        msg.world_angle_deg = float(world_angle)
        msg.blue_px         = int(blue_px)
        msg.petal_px        = int(petal_px)
        self.pub_floor.publish(msg)
        
        self.get_logger().info(f"[CV] EMIT | colour={colour} dir={direction} conf={confidence:.2f} tile#{self.tile_count} world_angle={world_angle:.1f}°")
        
        self.vote_buffer.clear()
        self.vote_start_time = None

    def _mask_inner_logo(self, mask, blue_contour, inner_fraction: float = 0.45):
        out = mask.copy()
        x, y, bw, bh = cv2.boundingRect(blue_contour)
        cx = x + bw // 2
        cy = y + bh // 2
        radius = int(min(bw, bh) * inner_fraction / 2)
        if radius > 5:
            cv2.circle(out, (cx, cy), radius, 0, thickness=-1)
        return out

    def _filter_elongated(self, contours, min_ar: float = 1.5):
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

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorNode()
    try:
        rclpy.spin(node)
    except SystemExit:
        node.get_logger().info('SystemExit raised. Graceful Stop.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
