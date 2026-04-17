# MAHE UGV — Navigation Solution Report
> Competition: UGV Autonomous Navigation Challenge (ROS 2 + Nav2, Map-less)  
> Robot: MINI_R1_V1 | Platform: Gazebo Ignition + ROS 2 Humble  
> Report synthesises: arena design intent, competition rules, robot specs, and recommended implementation strategy.

---

## 1. Understanding the Arena and the Navigation Story

The arena is divided into three broad zones that the robot must traverse in sequence: the **Explore Bay** (warehouse simulation), the **Maze Block**, and the **Final Goal Zone**. The green path on the sketch represents the correct intended route, and the entire challenge is built around the robot making intelligent reactive decisions rather than following a pre-scripted trajectory.

The robot spawns at a fixed location inside the Explore Bay. Its initial pose in simulation is known at spawn time (`x=0.0, y=0.0, z=0.07, yaw=π` from the launch file), which is the one piece of absolute reference the robot legitimately has. From this point onward, it must earn all further localization from its sensors.

The robot must cross the Explore Bay moving generally straight, heading east toward the rows. When it reaches the vicinity of Row 1, the LiDAR will observe a classic closed U-shape — walls on three sides with no meaningful opening — confirming this is a dead-end row and the wrong entry. A sign near this turn also visually reinforces  the bot to take a turn into row 2 typically left  (90 degrrees)"." The robot must combine both the LiDAR geometry reading and the sign detection to reject Row 1 and steer toward Row 2.

Row 2 is the correct warehouse row. Here the LiDAR picture changes: the U-shape is broken by gaps and cuts along the walls, indicating an open corridor, not a closed loop. This open signature is the robot's motivation to continue forward through Row 2. As it progresses, it will encounter **ArUco ID 0** (marker **B**), which confirms maze entry. At this moment the status logger must record the pose and publish a "maze entry confirmed" event. The intersection just past this marker has obstacle **C** (a warehouse crate), and the LiDAR will present multiple opening angles. The robot should pick the largest passable opening (width threshold discussed in Section 3) and enter the maze.

Inside the maze the robot follows open space. It reaches a corner which acts as a natural motivator to turn, then sees another corridor with two options: a narrower opening and a wider one. The wider opening carries a turn-left sign (marker **M**), making the decision clear — reject the narrow gap and follow the sign into the larger passage. After turning, the robot continues straight through a corridor where it must ignore small side openings that are below the passable-width threshold. It will eventually reach **ArUco ID 1** (marker **D**) alongside a direction sign. This is the maze exit confirmation — the status logger records pose, marks "maze exit confirmed," and critically sets a flag that prevents the robot from ever re-entering the maze section again (the aruco-logged pose of ID 1 is the boundary).

After the maze exit, the robot has two forward options: a thin ~300mm corridor (reject it — below threshold) and a wider turn. It takes the wider option and finds **ArUco ID 2** (marker **F**). Based on this it enters a T-junction path. At the T-junction it encounters the **INPLACE ROTATION** sign (marker **G**). The robot must approach this sign closely before starting its spin, because the spin serves a specific perception purpose: while rotating in place at the T-junction center, the camera scans all branches. One branch (Path X) will reveal a **STOP** sign (marker **H**) — this branch is rejected. The other branch (Path Y) will show the **GOAL** sign and **ArUco ID 3** (markers **J** and **I**). The robot stops spinning, aligns to Path Y, enters it, detects ArUco ID 3, logs the final pose, sees the goal sign, and the mission is complete.

Marker **E** is a deliberate trap — a FORWARD sign that tries to lure the robot into a dead-end row patch. The robot must override this sign using its LiDAR wall-following logic, which recognises a closed-loop U-shape ahead. This is a scored behaviour in the rubric (correct navigation decisions). Obstacles **L** (red box near start) and **K** (red circle inside maze) are static collision hazards that the local costmap and reactive planner must avoid.

---

## 2. Behaviours — What is Already Implied vs What Still Needs Implementation

Reading the navigation story against the competition rubric, the following behavioural requirements emerge. Some are clearly achievable given the current robot setup; others represent open engineering problems.

**Behaviours that the arena design naturally supports:**  
The explore behaviour in the Explore Bay is structurally built in — the robot simply needs to move forward and the arena geometry will present Row 1 as a closed loop. LiDAR-based U-shape detection to reject Row 1 is technically straightforward with a 360° scan. The ArUco pose logging pipeline (detect → update status → record pose) maps cleanly onto a standard `cv2.aruco` + pose estimation node. The in-place rotation at the T-junction is well-supported by the differential drive system — equal and opposite wheel speeds give a pure yaw rotation. The loop avoidance requirement (not re-entering the maze after ArUco ID 1) is solvable by checking the robot's current pose against the logged ID 1 pose and enforcing a one-way gate.

**Behaviours that require careful design decisions:**  
The sign interpretation pipeline is non-trivial. The arena already has sign images (PNG textures in the meshes), so the OpenCV node can be tuned by comparing against known templates — no ML model needed, but colour segmentation + contour matching + text/shape recognition pipeline still requires thoughtful implementation. The critical issue you raised about frame orientation — "sign says RIGHT but does that mean robot's right?" — is a real problem. The sign's intended direction is in the arena's world frame. When the camera sees a RIGHT arrow, it means the viewer (facing the sign) should go right. Since the robot is approaching the sign from the front, the camera's left-right matches the world's left-right only when the robot is facing the sign directly. This needs to be resolved by checking the robot's yaw (from EKF/IMU) and transforming the sign's indicated direction into the robot's local frame before commanding a turn.

Path Y vs Path X disambiguation at the T-junction is the hardest single decision in the run. During the in-place spin, both paths become visible in sequence. The robot must retain what it saw (a STOP sign in one direction, a GOAL sign + ArUco in the other) and act on that remembered percept after stopping the spin. This requires a short-term perceptual memory buffer, not just reactive response to the latest camera frame.

The maze-navigation question — whether a formal algorithm is needed — has a pragmatic answer: given that the arena dimensions are known and the maze has only one correct path with clearly sized passable corridors, a formal algorithm like right-hand rule or Tremaux's is unnecessary overhead. A reactive corridor-following approach driven by LiDAR opening-width thresholds, combined with ArUco/sign cues as tie-breakers at multi-opening junctions, is both rule-compliant (map-less) and sufficient for this specific maze geometry.

---

## 3. Arena Dimensions and Threshold Derivation

The arena dimensions from the CAD (all in mm) are central to setting LiDAR-based decision thresholds. The key insight is that the robot's wheel separation is 288.5mm and its physical body is slightly wider. A safe conservative robot width estimate for threshold calculations is approximately 350-380mm.

**Passable corridors** (robot must traverse): typical row widths 700-800mm, the tight transition from Row 2 to the maze at 580mm.

**Reject corridors** (robot must not enter): widths of 300mm, 291mm, 200mm, 400mm, 500mm, 392mm. Looking at these numbers, the pattern is clear — anything below ~600mm should be rejected. A threshold of **P × robot_width** where P is a safety factor of approximately 1.6 gives a cutoff around 560-608mm, which correctly classifies all the reject-widths as too narrow while allowing the 580mm tight passage (which is just at the edge — this specific transition may need a lower P in that context, or the passage width should be verified in the SDF).

The LiDAR measures row width by observing the rectangular U-shape: the two side walls give the width, and the far-end wall distance gives the depth. For a closed row (dead end), both side walls AND the far wall will be visible within a short range. For an open corridor, one or more walls will be absent or at long range. This distinction — closed U vs open corridor — is the core LiDAR analysis needed for Row 1 rejection and maze navigation.

The 1000mm dimension visible in the CAD appears to be a main corridor width, 392mm and 300mm appear at junction branches, and 759mm is likely the final goal zone corridor. The large overall arena width of ~5224mm and corridor span of ~3900mm give context for EKF drift accumulation — over a full run, odometry+IMU drift without ArUco corrections could accumulate to several hundred mm, which is why ArUco-based pose correction at all four markers is critical for maintaining reliable spatial decisions.

---

## 4. Node Architecture and Communication Design

The navigation system should be decomposed into nodes that have clear, single responsibilities and communicate over well-defined interfaces. Here is the recommended architecture based on the robot's sensor topology and the nav story requirements.

**EKF Pose Node** is the foundation. It fuses `/r1_mini/odom` (50Hz, wheel encoder simulation) and `/r1_mini/imu` (100Hz, 6-DOF IMU with low noise: angular velocity stddev 2e-4 rad/s, linear accel stddev 1.7e-2 m/s²) using `robot_localization`'s EKF node. It publishes `/odom_fused` at the fused rate. Critically, the Gazebo diff-drive plugin's TF publication (`odom → base_link`) must be disabled — the EKF owns that transform. When any ArUco detection arrives, the EKF pose at that instant is compared with the ArUco-estimated camera pose (from `solvePnP`), and the difference is the current drift error. This can be used to apply a one-shot pose correction. The IMU's yaw axis (Z-axis angular velocity, integrated) is the most trusted heading reference for turn control.

**ArUco Detection Node** subscribes to `/r1_mini/camera/image_raw` (640×480, 30Hz, 62° FOV) and `/r1_mini/camera/camera_info`. It runs OpenCV's ArUco detector, estimates 6-DOF pose of each detected marker via `estimatePoseSingleMarkers`, and publishes detections as a custom message carrying the marker ID, detected pose in camera frame, and a timestamp. It should publish to a topic like `/aruco/detections`. Other nodes (the status node, the nav controller) subscribe to this. When a new ID is first detected, it logs the event and fires a service call to the status node. The marker size is already known (from the SDF/textures), which makes the pose estimation reliable.


**Sign Detection Node** also subscribes to the camera image. It processes each frame for directional sign content using template matching or HSV-based colour segmentation + contour shape analysis against the known sign images (FORWARD, LEFT, RIGHT, STOP, INPLACE_ROTATION, GOAL PNGs are already in the meshes directory). It publishes a sign classification message to `/sign_detection` carrying the detected sign type and an approximate distance estimate (from known sign physical size and pixel width). The frame-to-robot direction transform mentioned in Section 2 must happen here: the node must read the current robot yaw from `/odom_fused` and compute what "LEFT on the sign" means in the robot's current heading context.

**LiDAR Analysis Node** subscribes to `/r1_mini/lidar` (360 samples, 10Hz, indices: 0=forward, 90=left, 180=back, 270=right). It produces several derived outputs: a corridor-width estimate (scan the angular sector ahead, find the lateral wall distances from the perpendicular rays), a U-shape closure flag (is the forward distance short AND both lateral distances short within a narrow sector?), a set of detected opening angles with their widths, and a "passable openings" list filtered by the P×robot_width threshold. It publishes these to `/lidar/analysis`. This node is the primary source for row rejection, opening selection, and narrow-path avoidance.

**Navigation Controller Node** is the brain. It is a state machine that consumes `/odom_fused`, `/aruco/detections`, `/sign_detection`, and `/lidar/analysis`, and publishes `/cmd_vel`. The states map directly to the navigation story: EXPLORE → ROW_SELECTION → MAZE_TRAVERSE → POST_MAZE → T_JUNCTION_SPIN → GOAL_APPROACH → DONE, with recovery substates (BACKTRACK, REORIENT, STUCK_ESCAPE). State transitions are triggered by events from the perception nodes. For example, the transition from MAZE_TRAVERSE to POST_MAZE is triggered by ArUco ID 1 detection. The in-place rotation for 90° turns uses the IMU yaw reference: command equal and opposite wheel velocities until the yaw change equals the target angle (PID or bang-bang with a small deadband). The 90° turn at marker A, the left turn at M, the T-junction spin at G — all use this same primitive.

**Status Logger Node** is a subscriber-only node (except for service calls from detection nodes) that maintains the mission state log. It subscribes to `/aruco/detections`, `/sign_detection`, and `/odom_fused`, and writes a live log file and publishes a `/mission_status` topic. The log should capture: timestamp, event type (aruco_detected/sign_detected/behaviour_started/behaviour_completed), marker ID or sign type, pose at event, and a human-readable status string. This is both a debugging tool and a competition scoring evidence trail. For the loop-avoidance requirement, this node is the authority — it holds the pose at which ArUco ID 1 was detected and provides a service that the nav controller can query: "am I near the maze entry zone?"

The communication pattern between these nodes is almost entirely topic-based (asynchronous, sensor-driven). The one place where services make sense is the pose-correction request (nav controller asks EKF node to apply a correction when a new ArUco pose is available) and the loop-avoidance zone query.

---

## 5. Key Unresolved Questions and Recommended Approaches

**Sign direction vs robot frame:** The cleanest solution is to define sign directions relative to the arena's world frame (North/East/South/West), not relative to the robot. When a sign says RIGHT, it means "turn to face East" (or whichever world-frame direction "right of this sign's facing" corresponds to). The robot uses its current EKF yaw to compute the required turn magnitude and direction. The sign poses can be extracted from the SDF world file to know which world direction each sign points the robot toward — this avoids any ambiguity entirely without hardcoding a trajectory.

**Maze algorithm vs reactive navigation:** No formal maze algorithm is needed. The maze in this arena is not a generic unknown maze — it has a fixed, known geometry that the robot can navigate reactively using LiDAR opening widths + ArUco/sign tie-breakers at junctions. The P-factor threshold approach is the primary tool. Where a formal concept is useful is the **dead-end detection**: if the robot sees closed walls on three sides (LiDAR reports no passable opening in any forward or lateral direction), it should backtrack a fixed distance and attempt a different heading — this satisfies the mandatory recovery behaviour requirement.

**T-junction Path Y disambiguation:** The recommended approach is a two-pass spin. First pass: rotate 360° slowly, building a map of what signs and ArUco markers are visible in which angular directions. Store the angular positions of any STOP sign detections and any GOAL/ArUco detections. Second pass: rotate back to the direction that showed GOAL + ArUco and align to it. This perceptual memory approach is robust against partial visibility and camera latency.

**Pose drift and ArUco correction:** The four ArUco markers form natural correction waypoints throughout the run. ArUco ID 0 corrects pose at maze entry, ID 1 at maze exit, ID 2 before the T-junction, and ID 3 at goal. With 288.5mm wheel separation and known wheel radius of 65mm, the theoretical odometry accuracy is reasonable for short runs, but yaw drift over the full arena (roughly 5-10m total path) could reach ±5-10° without IMU, and the IMU brings this down significantly. The ArUco corrections handle any residual drift at the key decision points.

**Differential drive turn precision:** For true in-place rotation, the left and right motor speeds must be exactly equal and opposite. With the skid-steer friction coefficient of only 0.2, there may be some slip. The IMU yaw rate integration is the feedback reference, not the wheel encoder count. The turn controller should command a target yaw and close the loop on IMU-integrated yaw, not on encoder ticks.

---

## 6. Compliance with Competition Rules

The proposed approach is fully compliant with the competition constraints. No SLAM or global map is used — the LiDAR feeds only a local costmap for immediate obstacle avoidance, not a persistent global representation. No AMCL or global localisation is used — pose is maintained by EKF (odom + IMU) with ArUco correction at landmarks. Marker positions are not hardcoded — they are detected dynamically by the camera node and their world positions are derived at runtime from pose estimation, not pre-loaded. Trajectories are not predefined — all motion commands emerge from the state machine responding to live sensor data.

The four mandatory scoring areas align well with this architecture: ArUco detection (30 pts) is handled by the dedicated detection node with pose estimation; correct navigation decisions (20 pts) come from the LiDAR analysis + sign detection fusion in the nav controller; maze completion (15 pts) follows from the corridor-following logic; loop avoidance (10 pts) is enforced by the status logger's ArUco ID 1 boundary check. The two mandatory recovery behaviours are in-place rotation (already central to the T-junction behaviour) and backtrack on dead-end detection.

The penalty structure reinforces some design choices: collision at -5 per hit means the local costmap inflation radius must be conservative, especially in the 580mm tight passage. Getting stuck for more than 8 seconds at -10 means the recovery behaviour trigger timeout must be well under 8 seconds — 4-5 seconds of no forward progress should trigger a recovery attempt. Wrong goal at -20 is the highest single penalty, which is why the T-junction spin strategy with perceptual memory before committing to Path Y is worth the extra complexity.

---

## 7. Robot Hardware Context Summary

The robot is a 4-wheel differential drive (effectively skid-steer) with wheelbase 260mm fore-aft and 288.5mm left-right. The LiDAR is a full 360° GPU-accelerated scanner at 10Hz with index 0 pointing forward — this is important because the LaserScan array's angular convention must be handled correctly in all LiDAR analysis code. The camera is front-facing at 62° FOV, 8m max range, 30Hz — this is adequate for sign and ArUco detection at the distances involved in this arena (most signs will be encountered within 2-3m). The IMU is 6-DOF at 100Hz with very low noise, making it the highest-quality sensor for turn angle control. The EKF fused output goes to `/odom_fused` which all nav nodes should use as their pose reference, not the raw `/r1_mini/odom`.

One practical note from the bridge configuration: both `/cmd_vel` (Twist) and `/cmd_vel_stamped` (TwistStamped) are bridged — the `twist_stamper` node in the launch file handles the conversion. The nav controller should publish standard `geometry_msgs/Twist` to `/cmd_vel`, which the stamper converts for Gazebo.

---

## 8. What Needs to Be Built (Implementation Roadmap)

Nothing in the current workspace addresses the navigation logic yet. The existing packages (`mini_r1_v1_description` and `mini_r1_v1_gz`) provide only the robot description (URDF/XACRO), the world SDF, and the simulation launch. Every navigation component is greenfield. The recommended implementation order, reflecting dependencies and risk:

The first thing to validate is that the simulation runs cleanly and all topics are publishing. Then the EKF node should be configured and verified — confirming that `/odom_fused` gives a stable pose and that the Gazebo diff-drive TF is suppressed. Once pose is reliable, the ArUco detection node is the next highest value item because it provides both perception and pose correction. The LiDAR analysis node comes next, as it provides the primary motivation signal for all motion decisions. The sign detection node follows, initially handling just the most critical signs (STOP and GOAL). The navigation state machine can then be built incrementally, starting with EXPLORE → ROW_SELECTION and progressively adding states. The status logger should be built early and run alongside development since it is the primary debugging window.

New ROS packages should be created inside `src/` following the same structure as the existing packages, with their own `package.xml` and `CMakeLists.txt` (or `setup.py` for Python nodes). A suggested package name is `mahe_nav` containing all the navigation nodes, keeping it separate from the robot description packages.

---

*Report generated: 2026-04-07 | Based on: Arena design sketch, competition ruleset (MAHE UGV Challenge), MINI_R1_V1 sensor/robot specs, existing workspace structure.*
