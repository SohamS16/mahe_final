# Detected ArUco marker with pose in camera frame and robot frame
std_msgs/Header header

int32 marker_id                     # 0, 1, 2, or 3

# Pose of marker centre in camera optical frame (z=depth, x=right, y=down)
geometry_msgs/Point position_camera  # metres
float32 distance                     # metres — Euclidean distance camera→marker

# Bearing in robot body frame (positive = left of forward)
float32 bearing_angle_rad            # radians, robot horizontal plane

# Raw OpenCV rvec/tvec for full 6-DOF pose if needed downstream
float32[3] tvec                      # translation vector [x,y,z] in camera frame
float32[3] rvec                      # rotation vector (Rodrigues)

bool first_detection                 # true only on first-ever detection of this ID
