import os, math, numpy as np, cv2, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from mahe_nav_interfaces.msg import SignDetection

MATCH_THRESHOLD = 0.75  
SIGN_PHYSICAL_WIDTH_M = 0.250
FOCAL_LENGTH_PX = 534.7

class SignDetectorNode(Node):
    def __init__(self):
        super().__init__('sign_detector')
        self.declare_parameter('templates_dir', 
            '/home/yusuf/ros2_mahe_ugv/src/gazebo_gefier_r1-main/mini_r1_v1_description/meshes')
        templates_dir = self.get_parameter('templates_dir').value
        
        self.bridge = CvBridge()
        self.templates = {}
        self._load_templates(templates_dir)
        self.detection_history = []
        self.CONSENSUS_FRAMES = 5
        
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(SignDetection, '/sign_detection', 10)
        self.sub = self.create_subscription(Image, '/r1_mini/camera/image_raw', self._image_cb, sensor_qos)
        self.get_logger().info(f'Sign Detector Active — Templates: {len(self.templates)}')

    def _load_templates(self, directory):
        files = {'FORWARD': 'forckward.png', 'LEFT': 'left.png', 'RIGHT': 'right.png', 
                 'STOP': 'stop.png', 'INPLACE_ROTATION': 'rotate.png', 'GOAL': 'goal.png'}
        for name, fname in files.items():
            path = os.path.join(directory, fname)
            if os.path.exists(path):
                img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                img = cv2.resize(img, (64, 64)) 
                self.templates[name] = cv2.createCLAHE(clipLimit=2.0).apply(img)

    def _image_cb(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except:
            return
        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        best_sign, best_score, best_w = 'NONE', 0.0, 0.0
        best_cx = best_cy = 0.0

        for cnt in contours:
            if cv2.contourArea(cnt) < 400: continue
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.04 * peri, True)
            if len(approx) == 4:
                x, y, w, h = cv2.boundingRect(approx)
                roi = gray[y:y+h, x:x+w]
                if roi.size == 0: continue
                roi = cv2.resize(roi, (64, 64))
                roi = cv2.createCLAHE(clipLimit=2.0).apply(roi)
                for name, tpl in self.templates.items():
                    res = cv2.matchTemplate(roi, tpl, cv2.TM_CCOEFF_NORMED)
                    _, score, _, _ = cv2.minMaxLoc(res)
                    if score > best_score:
                        best_score, best_sign, best_w = score, name, float(w)
                        best_cx, best_cy = float(x + w/2), float(y + h/2)

        current_det = best_sign if best_score > MATCH_THRESHOLD else 'NONE'
        self.detection_history.append(current_det)
        if len(self.detection_history) > self.CONSENSUS_FRAMES: self.detection_history.pop(0)
        final_sign = self.detection_history[0] if len(set(self.detection_history)) == 1 else 'NONE'

        out = SignDetection()
        out.header = msg.header
        out.sign_type = final_sign
        out.confidence = float(best_score)
        out.distance_estimate = (FOCAL_LENGTH_PX * SIGN_PHYSICAL_WIDTH_M) / best_w if best_w > 0 else 0.0
        out.image_x, out.image_y = best_cx, best_cy
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = SignDetectorNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
