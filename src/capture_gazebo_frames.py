#!/usr/bin/env python3
"""
capture_gazebo_frames.py

Run inside Docker container:
  python3 /ros2_ws/src/capture_gazebo_frames.py

What it does:
- Subscribes to /r1_mini/camera/image_raw
- Subscribes to /r1_mini/odom to detect when robot has moved ~1 tile
- Saves a frame:
    1. Every time robot moves ~0.5m (approx one tile width)
    2. Every 3 seconds regardless (safety net for slow movement)
    3. On keypress via ROS parameter trigger (manual override)
- Saves to /ros2_ws/src/gazebo_captures/ with timestamp + odom position in filename
- Also saves the raw HSV image alongside each frame for immediate debugging

Requirements (all standard in ROS2 Humble):
  rclpy, sensor_msgs, nav_msgs, cv_bridge, opencv-python
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import math
import time
from datetime import datetime
import csv

# ── CONFIG ─────────────────────────────────────────────────────────────────────
SAVE_DIR        = "/home/soham/ros2_mahe_ugv/src/gazebo_captures"
TILE_MOVE_DIST  = 0.45      # metres — save frame when robot moves this far
TIME_INTERVAL   = 3.0       # seconds — also save every N seconds regardless
CAMERA_TOPIC    = "/r1_mini/camera/image_raw"
ODOM_TOPIC      = "/r1_mini/odom"

# ── HSV ranges from natural PNG analysis ───────────────────────────────────────
BLUE_LOW   = np.array([110, 80,  80])
BLUE_HIGH  = np.array([135, 255, 255])
GREEN_LOW  = np.array([40,  60,  60])
GREEN_HIGH = np.array([90,  255, 255])
ORANGE_LOW  = np.array([5,   100, 100])
ORANGE_HIGH = np.array([22,  255, 255])
RED_LOW1   = np.array([0,   120, 70])
RED_HIGH1  = np.array([10,  255, 255])
RED_LOW2   = np.array([168, 120, 70])
RED_HIGH2  = np.array([180, 255, 255])

os.makedirs(SAVE_DIR, exist_ok=True)


class GazeboFrameCapture(Node):

    def __init__(self):
        super().__init__('gazebo_frame_capture')

        self.bridge       = CvBridge()
        self.latest_frame = None
        self.frame_count  = 0
        self.saved_count  = 0

        # Odometry tracking
        self.last_saved_x   = None
        self.last_saved_y   = None
        self.current_x      = 0.0
        self.current_y      = 0.0
        self.current_yaw    = 0.0

        # Time tracking
        self.last_save_time = time.time()

        # Subscribers
        self.img_sub  = self.create_subscription(
            Image, CAMERA_TOPIC, self.image_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self.odom_callback, 10)

        # Timer — checks every 0.5s whether to save
        self.timer = self.create_timer(0.5, self.check_and_save)

        self.get_logger().info(f"Capture node started. Saving to: {SAVE_DIR}")
        self.get_logger().info(f"Trigger: every {TILE_MOVE_DIST}m OR every {TIME_INTERVAL}s")

    # ── Callbacks ──────────────────────────────────────────────────────────────

    def image_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.frame_count += 1
        except Exception as e:
            self.get_logger().error(f"Image convert error: {e}")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.degrees(math.atan2(siny, cosy))

    # ── Save logic ─────────────────────────────────────────────────────────────

    def check_and_save(self):
        if self.latest_frame is None:
            return

        now         = time.time()
        time_trigger = (now - self.last_save_time) >= TIME_INTERVAL

        dist_trigger = False
        if self.last_saved_x is None:
            dist_trigger = True  # first save always
        else:
            dx   = self.current_x - self.last_saved_x
            dy   = self.current_y - self.last_saved_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist >= TILE_MOVE_DIST:
                dist_trigger = True

        if dist_trigger or time_trigger:
            self.save_frame(trigger="dist" if dist_trigger else "time")
            self.last_saved_x   = self.current_x
            self.last_saved_y   = self.current_y
            self.last_save_time = now

    def save_frame(self, trigger="manual"):
        frame = self.latest_frame.copy()
        h, w  = frame.shape[:2]

        ts    = datetime.now().strftime("%H%M%S_%f")[:-3]
        xpos  = round(self.current_x, 3)
        ypos  = round(self.current_y, 3)
        yaw   = round(self.current_yaw, 1)

        base_name = f"frame_{self.saved_count:04d}_{trigger}_x{xpos}_y{ypos}_yaw{yaw}_{ts}"

        # ── 1. Raw frame ────────────────────────────────────────────────────────
        raw_path = os.path.join(SAVE_DIR, base_name + "_RAW.jpg")
        cv2.imwrite(raw_path, frame)

        # ── 2. Annotated frame with ROI box + HSV masks overlaid ───────────────
        annotated = frame.copy()

        # ROI = bottom 55% of frame, excluding bottom 8% (robot body)
        roi_top    = int(h * 0.40)
        roi_bottom = int(h * 0.92)
        roi_left   = int(w * 0.10)
        roi_right  = int(w * 0.90)

        # Draw ROI boundary
        cv2.rectangle(annotated, (roi_left, roi_top), (roi_right, roi_bottom),
                      (255, 255, 0), 2)
        cv2.putText(annotated, "ROI", (roi_left + 5, roi_top + 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        # Extract ROI
        roi = frame[roi_top:roi_bottom, roi_left:roi_right]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Build color masks on ROI
        blue_mask   = cv2.inRange(hsv_roi, BLUE_LOW,   BLUE_HIGH)
        green_mask  = cv2.inRange(hsv_roi, GREEN_LOW,  GREEN_HIGH)
        orange_mask = cv2.inRange(hsv_roi, ORANGE_LOW, ORANGE_HIGH)
        red_mask    = cv2.bitwise_or(
                          cv2.inRange(hsv_roi, RED_LOW1, RED_HIGH1),
                          cv2.inRange(hsv_roi, RED_LOW2, RED_HIGH2))

        # Overlay masks as colored tints on annotated frame
        def tint_roi(ann_img, mask, color_bgr, offset_top, offset_left):
            tint = np.zeros_like(roi)
            tint[mask > 0] = color_bgr
            ann_img[offset_top:offset_top+roi.shape[0],
                    offset_left:offset_left+roi.shape[1]] = cv2.addWeighted(
                ann_img[offset_top:offset_top+roi.shape[0],
                        offset_left:offset_left+roi.shape[1]], 0.6, tint, 0.4, 0)

        tint_roi(annotated, blue_mask,   (255, 0,   0),   roi_top, roi_left)
        tint_roi(annotated, green_mask,  (0,   255, 0),   roi_top, roi_left)
        tint_roi(annotated, orange_mask, (0,   165, 255), roi_top, roi_left)
        tint_roi(annotated, red_mask,    (0,   0,   255), roi_top, roi_left)

        # Pixel count stats
        stats = (f"B:{blue_mask.sum()//255} "
                 f"G:{green_mask.sum()//255} "
                 f"O:{orange_mask.sum()//255} "
                 f"R:{red_mask.sum()//255}")
        cv2.putText(annotated, stats, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 2)
        cv2.putText(annotated, stats, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        pos_txt = f"x:{xpos} y:{ypos} yaw:{yaw} | frame#{self.saved_count} | {trigger}"
        cv2.putText(annotated, pos_txt, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
        cv2.putText(annotated, pos_txt, (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        ann_path = os.path.join(SAVE_DIR, base_name + "_ANNOT.jpg")
        cv2.imwrite(ann_path, annotated)

        # ── 3. Side-by-side mask strips (full-width debug sheet) ───────────────
        k      = np.ones((3, 3), np.uint8)
        masks  = {
            "BLUE":   cv2.morphologyEx(blue_mask,   cv2.MORPH_CLOSE, k),
            "GREEN":  cv2.morphologyEx(green_mask,  cv2.MORPH_CLOSE, k),
            "ORANGE": cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, k),
            "RED":    cv2.morphologyEx(red_mask,    cv2.MORPH_CLOSE, k),
        }
        panels = []
        for label, m in masks.items():
            panel = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
            cv2.putText(panel, label, (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            panels.append(panel)

        mask_sheet = cv2.hconcat(panels)
        mask_path  = os.path.join(SAVE_DIR, base_name + "_MASKS.jpg")
        cv2.imwrite(mask_path, mask_sheet)

        # ── 4. Append row to CSV log ───────────────────────────────────────────────
        csv_path = os.path.join(SAVE_DIR, "capture_log.csv")
        file_exists = os.path.isfile(csv_path)

        row = {
            "frame_index":    self.saved_count,
            "trigger":        trigger,
            "timestamp":      ts,
            "odom_x":         xpos,
            "odom_y":         ypos,
            "yaw_deg":        yaw,
            "blue_pixels":    int(blue_mask.sum() // 255),
            "green_pixels":   int(green_mask.sum() // 255),
            "orange_pixels":  int(orange_mask.sum() // 255),
            "red_pixels":     int(red_mask.sum() // 255),
            "roi_top":        roi_top,
            "roi_bottom":     roi_bottom,
            "roi_left":       roi_left,
            "roi_right":      roi_right,
            "frame_width":    w,
            "frame_height":   h,
            "raw_file":       base_name + "_RAW.jpg",
            "annot_file":     base_name + "_ANNOT.jpg",
            "masks_file":     base_name + "_MASKS.jpg",
        }

        with open(csv_path, "a", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=list(row.keys()))
            if not file_exists:
                writer.writeheader()
            writer.writerow(row)

        self.saved_count += 1
        self.get_logger().info(
            f"[{self.saved_count}] Saved: {base_name} | {stats}")


def main():
    rclpy.init()
    node = GazeboFrameCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            f"Stopped. Total frames saved: {node.saved_count} → {SAVE_DIR}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()