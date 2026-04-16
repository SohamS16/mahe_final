# Detected directional / action sign from camera image
std_msgs/Header header

# Sign type — one of: FORWARD, LEFT, RIGHT, STOP, INPLACE_ROTATION, GOAL, NONE
string sign_type

float32 confidence                   # 0.0–1.0 template match score
float32 distance_estimate            # metres, estimated from apparent pixel width
float32 pixel_width                  # detected sign width in pixels (for debug)

# Pixel centre of detection in the camera image
float32 image_x
float32 image_y
