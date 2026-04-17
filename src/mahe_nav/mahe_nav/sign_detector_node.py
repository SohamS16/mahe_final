import os
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mahe_nav_interfaces.msg import SignDetection

MATCH_THRESHOLD       = 0.75
SIGN_PHYSICAL_WIDTH_M = 0.250
FOCAL_LENGTH_PX       = 534.7

# OpenCV colours (BGR)
CLR_CONTOUR = (0, 255,   0)   # green  — every 4-corner contour
CLR_BEST    = (0,   0, 255)   # red    — best matched sign bounding box
CLR_LABEL   = (0, 255, 255)   # yellow — overlay text


class SignDetectorNode(Node):
    def __init__(self):
        super().__init__('sign_detector')

        self.declare_parameter(
            'templates_dir',
            '/home/yusuf/ros2_mahe_ugv/src/gazebo_gefier_r1-main/'
            'mini_r1_v1_description/meshes')
        self.declare_parameter('enable_viz', True)

        templates_dir   = self.get_parameter('templates_dir').value
        self.enable_viz = self.get_parameter('enable_viz').value

        self.bridge    = CvBridge()
        self.templates = {}
        self._load_templates(templates_dir)

        self.detection_history = []
        self.CONSENSUS_FRAMES  = 5

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(SignDetection, '/sign_detection', 10)
        self.sub = self.create_subscription(
            Image, '/r1_mini/camera/image_raw', self._image_cb, sensor_qos)

        # Attempt to open visualisation windows
        if self.enable_viz:
            try:
                cv2.namedWindow('Camera + Detection', cv2.WINDOW_NORMAL)
                cv2.namedWindow('Mask / Threshold',   cv2.WINDOW_NORMAL)
                cv2.resizeWindow('Camera + Detection', 640, 480)
                cv2.resizeWindow('Mask / Threshold',   640, 480)
                self.get_logger().info('Sign Detector: Visualisation windows open')
            except Exception as exc:
                self.get_logger().warn(
                    f'Sign Detector: Cannot open viz windows ({exc}) — disabling')
                self.enable_viz = False

        self.get_logger().info(
            f'Sign Detector Active — Templates: {len(self.templates)} | '
            f'viz={self.enable_viz}')

    # ── Template Loading ──────────────────────────────────────────────────────

    def _load_templates(self, directory: str):
        files = {
            'FORWARD':          'forckward.png',
            'LEFT':             'left.png',
            'RIGHT':            'right.png',
            'STOP':             'stop.png',
            'INPLACE_ROTATION': 'rotate.png',
            'GOAL':             'goal.png',
        }
        for name, fname in files.items():
            path = os.path.join(directory, fname)
            if os.path.exists(path):
                img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                img = cv2.resize(img, (64, 64))
                self.templates[name] = cv2.createCLAHE(clipLimit=2.0).apply(img)
            else:
                self.get_logger().warn(f'Template missing: {path}')

    # ── Image Callback ────────────────────────────────────────────────────────

    def _image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        # Binary threshold — isolates bright sign panels
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

        contours, _ = cv2.findContours(
            thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Visualisation frame (copy only when needed)
        viz_frame = cv_img.copy() if self.enable_viz else None

        best_sign  = 'NONE'
        best_score = 0.0
        best_w     = 0.0
        best_cx    = best_cy = 0.0
        best_box   = None          # (x, y, w, h) for the top match

        for cnt in contours:
            if cv2.contourArea(cnt) < 400:
                continue

            peri   = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)

            # Keep only rectangular (sign-like) contours
            if len(approx) != 4:
                continue

            x, y, w, h = cv2.boundingRect(approx)

            # Draw every valid contour in green on the viz frame
            if viz_frame is not None:
                cv2.drawContours(viz_frame, [approx], -1, CLR_CONTOUR, 2)

            roi = gray[y:y + h, x:x + w]
            if roi.size == 0:
                continue

            roi = cv2.resize(roi, (64, 64))
            roi = cv2.createCLAHE(clipLimit=2.0).apply(roi)

            for name, tpl in self.templates.items():
                res   = cv2.matchTemplate(roi, tpl, cv2.TM_CCOEFF_NORMED)
                _, score, _, _ = cv2.minMaxLoc(res)
                if score > best_score:
                    best_score = score
                    best_sign  = name
                    best_w     = float(w)
                    best_cx    = float(x + w / 2)
                    best_cy    = float(y + h / 2)
                    best_box   = (x, y, w, h)

        # Consensus filter
        current_det = best_sign if best_score > MATCH_THRESHOLD else 'NONE'
        self.detection_history.append(current_det)
        if len(self.detection_history) > self.CONSENSUS_FRAMES:
            self.detection_history.pop(0)
        final_sign = (
            self.detection_history[0]
            if len(set(self.detection_history)) == 1
            else 'NONE')

        # ── Visualisation ─────────────────────────────────────────────────────
        if self.enable_viz and viz_frame is not None:

            # Red bounding box + label on the best matched sign
            if best_box is not None and best_score > MATCH_THRESHOLD:
                bx, by, bw, bh = best_box
                cv2.rectangle(viz_frame,
                              (bx, by), (bx + bw, by + bh),
                              CLR_BEST, 2)
                label = f'{best_sign}  {best_score:.2f}'
                cv2.putText(viz_frame, label,
                            (bx, max(by - 8, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.60, CLR_LABEL, 2)

            # Consensus status bar at top of frame
            status_col = (0, 220, 0) if final_sign != 'NONE' else (120, 120, 120)
            cv2.putText(viz_frame, f'SIGN: {final_sign}',
                        (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.85, status_col, 2)

            try:
                cv2.imshow('Camera + Detection', viz_frame)
                cv2.imshow('Mask / Threshold',   thresh)
                cv2.waitKey(1)   # non-blocking — does not stall ROS spin
            except Exception as exc:
                self.get_logger().warn(f'imshow error: {exc} — disabling viz')
                self.enable_viz = False

        # ── Publish ───────────────────────────────────────────────────────────
        out = SignDetection()
        out.header            = msg.header
        out.sign_type         = final_sign
        out.confidence        = float(best_score)
        out.distance_estimate = (
            (FOCAL_LENGTH_PX * SIGN_PHYSICAL_WIDTH_M) / best_w
            if best_w > 0 else 0.0)
        out.image_x = best_cx
        out.image_y = best_cy
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
