import math
import numpy as np
import cv2
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose2D
from nav_msgs.msg import Odometry
from mahe_nav_interfaces.msg import ArucoDetection

# === PORTED: ARUCO POSE CORRECTION ===
MARKER_WORLD_POS = {
    0: (1.796,  0.975),
    1: (0.450, -0.441),
    2: (1.320, -1.341),
    3: (1.796, -1.875),
    4: (-0.450, 1.341),
}

# Dictionary and Marker Specs
ARUCO_DICT_ID = cv2.aruco.DICT_APRILTAG_36h11
MARKER_SIZE_M = 0.150  

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        # Parameters
        self.declare_parameter('marker_size_m', MARKER_SIZE_M)
        self.declare_parameter('camera_topic', '/r1_mini/camera/image_raw')
        self.declare_parameter('info_topic', '/r1_mini/camera/camera_info')
        self.declare_parameter('confirmation_frames', 3) 
        self.declare_parameter('camera_frame', 'CAM')

        self.marker_size = self.get_parameter('marker_size_m').value
        self.conf_frames = self.get_parameter('confirmation_frames').value
        self.camera_frame = self.get_parameter('camera_frame').value

        # ArUco Setup
        try:
            aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
            aruco_params = cv2.aruco.DetectorParameters()
            aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self.detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        except AttributeError:
            self.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT_ID)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.detector = None

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.seen_ids = set()
        self.sighting_counts = defaultdict(int) 
        self.pose_yaw = 0.0

        # ROS 2 Interfaces
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub_detection = self.create_publisher(ArucoDetection, '/aruco/detections', 10)
        
        # === PORTED: ARUCO POSE CORRECTION ===
        self.pub_pose_correction = self.create_publisher(Pose2D, '/aruco/pose_correction', 10)
        self.create_subscription(Odometry, '/odom_fused', self._odom_cb, 10)
        
        self.create_subscription(CameraInfo, self.get_parameter('info_topic').value, 
                                 self._info_cb, sensor_qos)
        self.create_subscription(Image, self.get_parameter('camera_topic').value, 
                                 self._image_cb, sensor_qos)

        self.get_logger().info('ArUco Detector Initialized')

    # === PORTED: ARUCO POSE CORRECTION ===
    def _odom_cb(self, msg):
        q = msg.pose.pose.orientation
        self.pose_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def _image_cb(self, msg):
        if self.camera_matrix is None:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        
        if self.detector:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is None:
            return

        for i, marker_id in enumerate(ids.flatten()):
            mid = int(marker_id)
            self.sighting_counts[mid] += 1
            if self.sighting_counts[mid] < self.conf_frames:
                continue

            half = self.marker_size / 2.0
            obj_pts = np.array([[-half, half, 0], [half, half, 0], 
                                [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
            
            success, rvec, tvec = cv2.solvePnP(obj_pts, corners[i][0].astype(np.float64), 
                                               self.camera_matrix, self.dist_coeffs, 
                                               flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not success: continue

            tvec = tvec.flatten()
            bearing_rad = float(math.atan2(-tvec[0], tvec[2]))
            distance = float(np.linalg.norm(tvec))

            det_msg = ArucoDetection()
            det_msg.header = msg.header
            det_msg.header.frame_id = self.camera_frame
            det_msg.marker_id = mid
            det_msg.distance = distance
            det_msg.bearing_angle_rad = bearing_rad
            det_msg.position_camera = Point(x=float(tvec[0]), y=float(tvec[1]), z=float(tvec[2]))

            # Populate raw 6-DOF fields so downstream nodes can compute accurate pose
            det_msg.tvec = [float(tvec[0]), float(tvec[1]), float(tvec[2])]
            det_msg.rvec = [float(rvec.flatten()[0]), float(rvec.flatten()[1]), float(rvec.flatten()[2])]

            # === PORTED: ARUCO POSE CORRECTION ===
            if mid in MARKER_WORLD_POS and distance <= 0.8:
                mx, my = MARKER_WORLD_POS[mid]
                world_angle = self.pose_yaw + bearing_rad
                corrected_x = mx - distance * math.cos(world_angle)
                corrected_y = my - distance * math.sin(world_angle)
                
                corr_msg = Pose2D()
                corr_msg.x = corrected_x
                corr_msg.y = corrected_y
                corr_msg.theta = self.pose_yaw
                self.pub_pose_correction.publish(corr_msg)

            det_msg.first_detection = mid not in self.seen_ids
            if det_msg.first_detection:
                self.seen_ids.add(mid)
                self.get_logger().info(
                    f'ArUco {mid} FIRST DETECTION | dist={distance:.2f}m '
                    f'bearing={math.degrees(bearing_rad):.1f}° '
                    f'tvec=[{tvec[0]:.3f}, {tvec[1]:.3f}, {tvec[2]:.3f}]'
                )

            self.pub_detection.publish(det_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
