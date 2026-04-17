# ARTPARK Arena — Final Consolidated Navigation Ideation

> **Landmark-anchored, SLAM-free, Nav2-free autonomous traversal** for the ARTPARK 5×4 grid arena.
> Consolidated from Yusuf's point-to-point route analysis and the systematic phase-based ideation, with improvements from the existing codebase.
> **No code in this document — pure strategy and specification.**

---

## Table of Contents

- [Part A — Arena Ground Truth](#part-a--arena-ground-truth)
- [Part B — The Route: Point-by-Point Trajectory](#part-b--the-route-point-by-point-trajectory)
- [Part C — Core Motion: LiDAR Centering & Obstacle Handling](#part-c--core-motion-lidar-centering--obstacle-handling)
- [Part D — Perception: AprilTags, Floor Colour & Arrow CV](#part-d--perception-apriltags-floor-colour--arrow-cv)
- [Part E — Memory: Anti-Backtracking & Counters](#part-e--memory-anti-backtracking--counters)
- [Part F — The Brain: State Machine](#part-f--the-brain-state-machine)
- [Part G — Hard Constraints & Must-Not-Happen Rules](#part-g--hard-constraints--must-not-happen-rules)
- [Part H — Logging & Diagnostics](#part-h--logging--diagnostics)
- [Part I — Edge Cases & Recovery](#part-i--edge-cases--recovery)
- [Part J — Phased Execution Plan](#part-j--phased-execution-plan)
- [Part K — Timing Budget](#part-k--timing-budget)

---

# Part A — Arena Ground Truth

> Everything extracted and verified from the SDF world file `mahe_arena.sdf`.

## A.1 Physical Dimensions

| Parameter | Value | Source |
|---|---|---|
| Tile pitch (centre-to-centre) | **0.9 m** | SDF tile positions |
| Navigable grid | **5 rows × 4 columns** (r0–r4, c0–c3) | 20 tiles total |
| Outer boundary (wall-to-wall) | **≈ 3.61 m wide × 4.51 m tall** | Outer wall positions ±1.805, ±2.255 |
| Wall thickness | **0.01 m** | SDF geometry |
| Wall height | **0.30 m** | SDF geometry |
| Usable corridor width | **≈ 0.89 m** | Tile pitch minus wall thickness |
| Cylindrical obstacle (CP) | Centre **(0.9, 0.45)**, radius **0.225 m** | SDF `obstacle` model |
| World coordinate origin | **(0, 0)** at arena centre | SDF convention |

## A.2 Grid-to-World Coordinate Map

Row 0 is the TOP of the arena. Y increases upward (north), X increases rightward (east).

| Grid Cell | World (x, y) | Content |
|---|---|---|
| **(r0, c0)** | (−1.35, +1.80) | 🟢 **START** — green tile |
| (r0, c1) | (−0.45, +1.80) | ARTPARK logo |
| (r0, c2) | (+0.45, +1.80) | ARTPARK logo |
| (r0, c3) | (+1.35, +1.80) | ARTPARK logo |
| (r1, c0) | (−1.35, +0.90) | ARTPARK logo |
| (r1, c1) | (−0.45, +0.90) | ARTPARK logo |
| (r1, c2) | (+0.45, +0.90) | ARTPARK logo |
| (r1, c3) | (+1.35, +0.90) | ARTPARK logo |
| (r2, c0) | (−1.35, +0.00) | ARTPARK logo |
| (r2, c1) | (−0.45, +0.00) | ARTPARK logo |
| (r2, c2) | (+0.45, +0.00) | ARTPARK logo |
| (r2, c3) | (+1.35, +0.00) | ARTPARK logo |
| (r3, c0) | (−1.35, −0.90) | ARTPARK logo |
| (r3, c1) | (−0.45, −0.90) | ARTPARK logo |
| (r3, c2) | (+0.45, −0.90) | ARTPARK logo |
| (r3, c3) | (+1.35, −0.90) | ARTPARK logo |
| (r4, c0) | (−1.35, −1.80) | Solid red tile |
| (r4, c1) | (−0.45, −1.80) | ARTPARK logo (arrows) |
| (r4, c2) | (+0.45, −1.80) | ARTPARK logo (arrows) |
| (r4, c3) | (+1.35, −1.80) | 🔴 **STOP** — red tile area |

## A.3 Internal Walls — Exact Spans (from SDF)

| Wall | SDF Centre (x, y) | SDF Size (w × h) | Actual Span |
|---|---|---|---|
| **wall_A** (vertical) | (−0.90, −0.90) | 0.01 × 2.70 | x = −0.90, from y = −2.25 to y = +0.45 |
| **wall_C** (horizontal) | (−0.45, +1.35) | 2.70 × 0.01 | y = +1.35, from x = −1.80 to x = +0.90 |
| **wall_D** (vertical) | (0.00, +0.45) | 0.01 × 1.80 | x = 0.00, from y = −0.45 to y = +1.35 |
| **wall_E** (horizontal) | (+0.45, −0.45) | 0.90 × 0.01 | y = −0.45, from x = 0.00 to x = +0.90 |
| **wall_F** (horizontal) | (+0.90, −1.35) | 1.80 × 0.01 | y = −1.35, from x = 0.00 to x = +1.80 |

## A.4 AprilTag Positions (from SDF — IDs 0–4)

| SDF Model | World Pose (x, y, z) | Yaw | Mounted On | Faces | Robot Must Approach From |
|---|---|---|---|---|---|
| **tag0** (ID 0) | (1.796, 0.975, 0.175) | π (180°) | Right outer wall | **West** | Heading east toward right wall |
| **tag1** (ID 1) | (0.45, −0.441, 0.175) | 0 | wall_E south face | **North** | Heading south, sees tag on wall_E |
| **tag2** (ID 2) | (1.32, −1.341, 0.175) | 0 | wall_F south face | **North** | Heading south, sees tag on wall_F |
| **tag3** (ID 3) | (1.796, −1.875, 0.175) | π (180°) | Right outer wall | **West** | Heading east toward right wall |
| **tag4** (ID 4) | (−0.45, 1.341, 0.175) | 0 | wall_C south face | **North** | Heading south, sees tag on wall_C |

> **IMPORTANT:** The SDF model name (`tag0`, `tag1`…) corresponds to the decoded AprilTag dictionary ID (0, 1, 2, 3, 4). This MUST be verified by decoding the actual `tag_X_padded.png` images before writing dispatch logic.

## A.5 Key Passable Gaps

The internal walls create a highly constrained maze. Critical passage points:

| Gap | Between | Why It Matters |
|---|---|---|
| **Top corridor** (Row 0) | r0,c0 ↔ r0,c1 ↔ r0,c2 ↔ r0,c3 | Fully open east-west corridor above wall_C |
| **Col 0 ↔ Col 1** at row 1 | r1,c0 ↔ r1,c1 | Gap between wall_A top (y=+0.45) and wall_C (y=+1.35) |
| **Row 0 ↔ Row 1** at col 3 | r0,c3 ↔ r1,c3 | Gap between wall_C right end (x=+0.90) and right wall (x=+1.805) |
| **Right column** (Col 3) | r1,c3 ↔ r2,c3 ↔ r3,c3 | Open north-south corridor right of wall_D/E |
| **Col 1 ↔ Col 2** at rows 2–3 | r2,c1 ↔ r2,c2 | Gap below wall_D bottom (y=−0.45) |
| **Row 2 ↔ Row 3** at cols 0–1 | r2,c0 ↔ r3,c0 | Gap left of wall_F start (x < 0.0) |

## A.6 ARTPARK Logo & Arrow Semantics

### Logo Structure

Each tile bears an ARTPARK insignia: a blue network circle surrounded by three coloured petals — green, orange, and blue connection dots. The logo can appear at different rotations, and the rotation encodes the arrow direction for each colour.

### Colour Mission Phases

| Colour | Active Phase | Trigger Tag | End Condition |
|---|---|---|---|
| **Green** | After Tag 1 | Tag 1 ("start following green") | Reaching Tag 3 |
| **Blue** | After Tag 3 U-turn | Tag 3 ("U-turn") | Reaching Tag 2 |
| **Orange** | After Tag 2 | Tag 2 ("start following orange") | Reaching STOP (red tile) |

### How Arrow Direction is Encoded

The direction of the target-colour petal relative to the centre of the logo indicates the direction to drive. The robot's down-camera captures the tile, CV determines the petal position relative to logo centre, then converts from image-relative to world-relative direction using the robot's current heading.

### Critical Risk: Texture Similarity

Most tiles share the same default logo orientation. Only a few tiles (`r1,c0`, `r3,c0`, `r4,c1/c2/c3`) have visibly different rotations. This means:
1. The competition may provide different logo rotations than the current SDF textures show
2. A template-matching fallback is needed alongside HSV centroid extraction
3. The HSV-petal-centroid method is primary; template matching is the safety net

---

# Part B — The Route: Point-by-Point Trajectory

> This is the **core navigation story** — adapted from Yusuf's point-by-point analysis (#A → #L). The robot discovers this path reactively through LiDAR gap detection and ArUco tag responses, but the logic at each waypoint is precisely specified.

## B.1 Waypoint Map

```
#A (START, green tile r0,c0)
 ↓ Path B — drive south in col 0 corridor
#C (right turn at row 1 level — gap between wall_A top and wall_C)
 → drive east through row 1
#D (ArUco ID 4 on wall_C south face — facing north)
 ↓ turn right (south) after detecting ID 4
#E (junction near r2,c1 — CP visible nearby)
 → drive east, navigate around Circular Pillar
#F (ArUco ID 0 on right wall — facing west)
 ↓ turn left (south) after detecting ID 0
#G (junction near r2,c2 — must NOT go back to #D or #F)
 → turn right toward clean opening
#H (ArUco ID 1 on wall_E south face — CV task starts: GREEN arrows)
 ↓ follow green arrows through tiles T1, T2
#I (junction — T-junction decision, must take LEFT toward shorter corridor)
 → drive into corridor toward #J
#J (ArUco ID 3 on right wall — U-TURN, dead end)
 ↑ 180° turn, switch to BLUE arrows
 ← return through #I, take right gap toward corridor
#K (ArUco ID 2 on wall_F south face — switch to ORANGE arrows)
 ↓ follow orange arrows, take left turn into final gap
#L (STOP — red tile detected, mission complete)
```

## B.2 Detailed Waypoint Decision Logic

### #A → Path B (START)
- **Location**: Green tile at r0,c0 (−1.35, +1.80)
- **Action**: Confirm green tile under camera. Log START. Begin driving south (−Y direction)
- **LiDAR**: Walls left (outer wall) and right (wall_A is further south), open corridor ahead (south)
- **Strategy**: Centre between left outer wall and the corridor walls using bilateral LiDAR PD

### Path B → #C (First Turn)
- **Detection**: LiDAR detects gap on the RIGHT side — wall_A ends at y = +0.45, creating the gap between wall_A top and wall_C
- **Decision**: Take the right turn (east) into the gap
- **Key signal**: The right-side LiDAR reading suddenly increases when wall_A ends — detect this range jump as a gap opening

### #C → #D (Approach Tag 4)
- **Direction**: Driving east through the gap at row 1 level
- **Detection**: Front camera sees ArUco ID 4 on wall_C south face (tag faces north, robot approaching from south)
- **CRITICAL**: Do NOT log/execute the tag action until the robot is within `ACTION_DISTANCE_THRESHOLD` (≤ 1.25 m). The camera can see tags from far away — premature execution causes wrong positioning

### #D → #E (Post-Tag-4 Navigation)
- **Action on Tag 4**: Store the detection. Turn RIGHT (south, −Y direction)
- **Location after turn**: Near r1,c1 area, heading south
- **LiDAR view at #E**: Corner with gap behind (north to #D) and gap to the east. Wall_A on the left side
- **Strategy**: Must go east toward ArUco ID 0. Must NOT backtrack north to #D

### #E → #F (Navigating Past the Circular Pillar)
- **Obstacle**: The Circular Pillar (CP) at (0.9, 0.45) — radius 0.225 m — sits in this path zone
- **LiDAR signature**: CP appears as a curved arc (not a flat wall). Use the existing Dmax shape classification from `lidar_analyzer_node.py` — if `dmax >= 0.2 * baseline_length`, classify as "curved"
- **Strategy**: When detecting a curved obstacle, arc toward the side with more clearance, pass it, then re-centre. The gap between the pillar and the walls is tight — choose the wider side
- **Yusuf's note**: Use "Dmax logic" from previous projects to detect the specific shape of features (like the CP arc) and differentiate from gaps

### #F (ArUco ID 0 — Take Left)
- **Location**: Right outer wall at (1.796, 0.975)
- **Detection**: Robot heading east toward right wall sees ID 0 (tag faces west)
- **Action**: Turn LEFT (now heading south, −Y). Verify LiDAR clearance on left before executing turn
- **Pattern**: Use existing `check_lidar_clearance('LEFT', ...)` safety check

### #F → #G (Junction Decision — MUST NOT BACKTRACK)
- **LiDAR at #G**: Gap left (back to #D area), gap behind (#F area), and a clean opening ahead/right
- **HARD RULE**: MUST NOT go back to #D or #F. The visited-cell tracker must mark those corridors as traversed
- **Decision**: Go toward the clean opening (southward corridor)
- **How enforced**: Bloom filter marks cells near #D and #F as visited → junction scorer deprioritises them

### #G → #H (ArUco ID 1 — Start Green CV Task)
- **Location**: Wall_E south face at (0.45, −0.441), tag faces north
- **Detection**: Robot heading south sees ID 1 on wall_E
- **Action**: Log TAG_1. Activate **GREEN arrow-following mode**. The CV task begins from this point
- **State transition**: EXPLORE → TAG_ACTION → FOLLOW_GREEN

### #H → T1, T2 → #I (Green Arrow Following)
- **Mode**: ARROW_FOLLOW with active colour = GREEN
- **Per-tile protocol**: Drive ~0.85 m → stop → read green petal direction from logo → rotate to indicated direction → increment green counter → resume
- **T-junction at T2 area near #I**: LiDAR shows a large corridor on the right and a smaller corridor on the left
- **Decision**: Must take the **LEFT** smaller corridor toward #I — the GREEN arrow on the tile directs this path. Do NOT go into the right large corridor
- **Fallback**: If arrow is unclear, compare corridor depths via LiDAR and pick the shorter one (which leads to #I / #J)

### #I → #J (Dead End with ArUco ID 3 — U-Turn)
- **LiDAR at #I**: Wall ahead, wall right, gap left, gap behind
- **Decision**: Must NOT go backward. Take the left gap toward #J
- **Detection at #J**: ArUco ID 3 on right outer wall at (1.796, −1.875), tag faces west
- **Action**: Perform **180° in-place U-turn** (NOT a 360° spin). Switch to **BLUE arrow mode**
- **Turn execution**: Pure pivot at +0.60 rad/s (CCW), monitored by IMU/EKF yaw until Δyaw ≈ 180° (tolerance ±3°)

### Return from #J through #I (Blue Arrow Following)
- **After U-turn**: Return heading back through corridor, reading BLUE petal directions tile-by-tile
- **At #I exit**: Wall ahead and left, gap on the right. Select right gap, traverse straight
- **CRITICAL at #K region**: See a gap on the right (back to #H area) and a longer corridor straight ahead. MUST NOT go to #H — continue straight toward #K
- **Enforcement**: Visited-cell tracker flags cells near #H as previously visited → junction scorer skips them

### #K (ArUco ID 2 — Start Orange)
- **Location**: Wall_F south face at (1.32, −1.341), tag faces north
- **Detection**: Robot heading south sees ID 2
- **Action**: Log TAG_2. Turn LEFT 90°. Switch to **ORANGE arrow mode**
- **State transition**: FOLLOW_BLUE → TAG_ACTION → FOLLOW_ORANGE

### #K → #L (STOP — Mission Complete)
- **Mode**: ARROW_FOLLOW with active colour = ORANGE
- **Follow orange arrows through remaining tiles toward red STOP area**
- **End detection**: Down-camera detects **RED tile** — solid red, >40% frame fill, **3 consecutive frames required** (prevents false triggers from red logo components)
- **Action**: Publish zero cmd_vel. Log STOP with timestamp and position. Write all counters to CSV. Shutdown nodes

---

# Part C — Core Motion: LiDAR Centering & Obstacle Handling

> Reuse and extend the existing `lidar_analyzer_node.py` architecture.

## C.1 Corridor Centering (Bilateral PD)

| Parameter | Value |
|---|---|
| Error signal | `left_dist − right_dist` |
| Kp | 0.4 (from existing `_move()`) |
| Kd | 0.0 initially (add if oscillation occurs) |
| Dead-band | ±0.02 m |
| Active condition | `v > 0.05` and both sides < 2.0 m |

## C.2 Wall Repulsion (Emergency)

Already implemented in existing code: if either `left_dist` or `right_dist` < 0.30 m, apply angular correction (±0.55 rad/s). Prevents wall clipping during turns and narrow passages.

## C.3 Speed Control

| Front Distance | Linear Speed | Behaviour |
|---|---|---|
| > 1.5 m | 0.30 m/s (V_MAX) | Cruising |
| 0.35–1.5 m | Linear ramp V_MIN to V_MAX | Approaching junction/wall |
| < 0.35 m | 0.0 (or −0.06 reverse if back clear) | Emergency stop |

## C.4 Cylindrical Pillar (CP) Handling

The existing `lidar_analyzer_node.py` already classifies gap shapes using the **Dmax method**:
- If `dmax >= 0.2 * baseline_length` → shape = "curved" (this is the pillar)
- If shape is "curved", the nav controller must:
  1. Identify which side of the curve has more open space
  2. Arc toward the larger gap at reduced speed (V_MIN)
  3. Re-centre using bilateral distances after passing

## C.5 LiDAR Wall-Aligned Heading Correction (NEW — High Impact)

All arena walls are axis-aligned (0°/90°/180°/270°). By fitting lines to LiDAR wall detections (RANSAC or Hough transform), the robot gets a **drift-free heading reference** at ~10 Hz.

This heading measurement injects into the EKF as a yaw-only observation, eliminating yaw drift — the #1 failure mode in narrow corridor navigation that causes corner-clipping and misaligned turns.

## C.6 Turn Execution

All turns are **pure in-place pivots** (linear.x = 0):

| Turn | angular.z | Target Angle | Tolerance |
|---|---|---|---|
| LEFT 90° | +0.70 rad/s (CCW) | 90° | ±3° |
| RIGHT 90° | −0.70 rad/s (CW) | 90° | ±3° |
| U-TURN 180° | +0.60 rad/s (CCW) | 180° | ±3° |

**Three-layer safety** (from existing codebase):
1. **Pre-turn**: Check side clearance > 0.40 m before starting
2. **Mid-turn**: Monitor forward distance every 50 ms — abort if < 0.20 m, halve angular rate if < 0.30 m
3. **Post-turn**: Require forward > 0.45 m before resuming exploration

## C.7 EKF Sensor Fusion

| Source | Data | Rate | Role |
|---|---|---|---|
| Wheel odometry (`/odom`) | x, y, yaw | 50 Hz | Primary dead-reckoning |
| IMU (`/imu/data`) | angular velocity, accel | 100 Hz | Heading rate + short-term acc |
| LiDAR heading correction | yaw (axis-aligned) | 10 Hz | **Drift-free heading anchor** |
| AprilTag pose correction | x, y, yaw (absolute) | Intermittent | **Drift-free position anchor** |

---

# Part D — Perception: AprilTags, Floor Colour & Arrow CV

## D.1 AprilTag Detection

Reuse existing `aruco_detector_node.py` with `DICT_APRILTAG_36h11`, marker size 0.15 m, confirmation = 3 frames.

### Proximity-Gated Execution (Yusuf's Key Insight)

> **WARNING: The camera detects tags from far away.** Navigation decisions must ONLY execute when the robot is at the correct position — within `ACTION_DISTANCE_THRESHOLD` (1.25 m). Premature execution at long range causes the robot to turn in the wrong location. This is already implemented in the existing `nav_controller_node.py` and must be preserved.

### Tag Action Dispatch Table

| Tag ID | SDF Model | Location | Action | Arrow Mode |
|---|---|---|---|---|
| **4** | tag4 | wall_C south face | Turn RIGHT 90° | — |
| **0** | tag0 | Right outer wall | Turn LEFT 90° | — |
| **1** | tag1 | wall_E south face | Activate CV task | → **GREEN** |
| **3** | tag3 | Right outer wall | U-TURN 180° | → **BLUE** |
| **2** | tag2 | wall_F south face | Turn LEFT 90° + activate | → **ORANGE** |

### Tag-Anchored Drift Reset

Each tag has a known world position from the SDF. When detected within range:
1. PnP (`solvePnP`) computes camera-to-tag pose (already implemented)
2. Using robot heading, compute corrected world position
3. Override EKF estimate — snaps position back to truth
4. All subsequent junction decisions use corrected position

### Active Tag Scanning

Tags face specific directions and can be missed if the robot doesn't look their way. Perform brief ±45° to ±90° scanning sweeps at:
- Every junction (where ≥ 2 directions are open)
- Every corridor dead-end (wall ahead)
- After completing any turn

## D.2 Floor Colour Detection (START & STOP)

| Tile | HSV Range | Fill Threshold | Confirmation |
|---|---|---|---|
| **Green (START)** | H: 35–85, S: 100–255, V: 100–255 | > 30% frame | 1 frame (on green tile at boot) |
| **Red (STOP)** | H: 0–10 ∪ 170–180, S: 150–255, V: 100–255 | > 40% frame | **3 consecutive frames** |

**Auto-HSV Calibration at Boot**: During INIT (robot on green tile), capture 10 frames from down-camera and sample actual HSV distributions. Dynamically adjust thresholds to match lighting conditions.

## D.3 Arrow Direction Detection (CV Task)

### Stop-Read-Go Protocol (per tile during ARROW_FOLLOW)

1. Drive forward ~0.85 m (slightly under one tile pitch)
2. Stop completely (zero cmd_vel)
3. Wait 200 ms for camera stabilisation
4. Capture 5 frames from down-camera
5. For each frame: HSV filter for active colour → find largest contour → compute centroid → vector from logo centre → quantize to {N, S, E, W}
6. Majority vote across 5 readings for robustness
7. Rotate in-place to face the voted direction
8. Increment active-colour arrow counter
9. Resume driving

### Blue Circle Disambiguation

The large blue central circle in the logo can confuse the blue-arrow detector:
- Mask out the inner 40–50% of the logo bounding box before searching for blue petals
- Filter by contour shape: petals are elongated (aspect ratio > 1.5), the circle is round (≈ 1.0)
- Position filter: only accept colour blobs in the outer 30% of logo area

### Template-Matching Fallback

If HSV centroid fails (petal too small, partial occlusion):
- Pre-capture logo at 0°, 90°, 180°, 270° orientations
- Match current frame against all 4 templates
- Require match confidence > 0.6 to accept
- Best-matching template directly indicates arrow direction

### Tile Transition Confirmation

Don't rely on odometry alone (0.9 m). Confirm with multiple signals:
- Odometry indicates ≥ 0.80 m travel since last tile
- Down-camera detects a fresh, fully-centred logo
- Optionally: detect thin dark edge line between tiles

---

# Part E — Memory: Anti-Backtracking & Counters

> From Yusuf: "Do not re-traverse certain gaps. Use a lightweight cost grid to mark traversed areas."

## E.1 Bloom Filter for Visited Cells

Spatial hashing with O(1) lookup:

| Parameter | Value |
|---|---|
| Cell size | 0.225 m (quarter tile — fine enough for narrow passages) |
| Total cells in arena | ~320 |
| Bloom filter capacity | 500 (headroom for drift duplicates) |
| Bit array size | ~3,067 bits (384 bytes) |
| Hash functions (k) | 7 |
| False positive rate | ~1% |
| False negatives | **Impossible** — a visited cell is always correctly identified |

**Marking**: Every 0.10 m of travel (from odometry), discretise current EKF position into grid cell and insert.

**Querying at junctions**: For each open direction from LiDAR, project the next cell 0.9 m ahead and query the Bloom filter:
- "Not present" → **definitely unvisited** → HIGH priority
- "Present" → **probably visited** → LOW priority

### Junction Scoring Priority

| Priority | Rule |
|---|---|
| **1st** | Any unvisited direction |
| **2nd** | Among unvisited, prefer forward (no turn overhead) |
| **3rd** | If ALL report "visited" (possible false positive or backtrack), pick forward or right-hand rule |

### Override Rule

If all directions report "visited" at a junction, override the Bloom filter and pick a direction anyway. The maze has so few branches that forward-preference resolves this.

## E.2 Arrow Counter (Per-Colour)

| Colour | Incremented When |
|---|---|
| GREEN | Each tile successfully read during green arrow phase |
| BLUE | Each tile during blue arrow phase |
| ORANGE | Each tile during orange arrow phase |

## E.3 Tile Counter

- **Unique tiles**: Python `set()` tracking cell IDs entered (never exceeds ~25)
- **Total traversals**: Simple integer counter (includes revisits)

---

# Part F — The Brain: State Machine

## F.1 Nine-State FSM

| State | Purpose |
|---|---|
| **INIT** | Boot, calibrate HSV, confirm green START tile, sensor health checks |
| **EXPLORE** | LiDAR-driven corridor traversal, junction decisions, tag scanning |
| **TAG_ACTION** | Central dispatch — execute the correct manoeuvre for each tag ID |
| **FOLLOW_GREEN** | Arrow follow with active colour = GREEN (stop-read-go per tile) |
| **FOLLOW_BLUE** | Arrow follow with active colour = BLUE |
| **FOLLOW_ORANGE** | Arrow follow with active colour = ORANGE |
| **UTURN** | Clean 180° in-place rotation after Tag 3 |
| **RECOVERY** | Stuck detection → backtrack → re-scan |
| **HALT** | Mission complete — stop all movement, write all logs |

## F.2 State Transitions

```
INIT ──(green tile confirmed)──► EXPLORE

EXPLORE ──(Tag detected in range)──► TAG_ACTION
EXPLORE ──(Red tile, 3 frames)──► HALT
EXPLORE ──(Stuck timeout)──► RECOVERY

TAG_ACTION ──(Tag 4: turn RIGHT)──► EXPLORE
TAG_ACTION ──(Tag 0: turn LEFT)──► EXPLORE
TAG_ACTION ──(Tag 1: green mode)──► FOLLOW_GREEN
TAG_ACTION ──(Tag 3: U-turn)──► UTURN
TAG_ACTION ──(Tag 2: orange mode)──► FOLLOW_ORANGE

UTURN ──(180° complete)──► FOLLOW_BLUE

FOLLOW_GREEN ──(Tag detected)──► TAG_ACTION
FOLLOW_BLUE ──(Tag detected)──► TAG_ACTION
FOLLOW_ORANGE ──(Red tile)──► HALT

FOLLOW_* ──(Arrow lost 3 tiles)──► RECOVERY
RECOVERY ──(unstuck)──► EXPLORE
RECOVERY ──(5 min total)──► HALT
```

## F.3 Re-Detection Handling

If a tag that was already logged is re-detected:
1. Execute 180° turnaround (the robot is going in circles)
2. Mark current corridor as visited in Bloom filter
3. Transition to EXPLORE (escape explored territory)
4. Do NOT double-log the tag

## F.4 Watchdog Timers

| Timer | Duration | Trigger |
|---|---|---|
| State transition watchdog | 30s without any state change | → RECOVERY |
| Forward motion watchdog | 8s with < 5 cm movement | → RECOVERY (existing in codebase) |
| Tag proximity linger | 3s within 0.3 m of tag without action | → Force TAG_ACTION |
| Mission total timeout | 300s (5 min) | → Emergency HALT |

---

# Part G — Hard Constraints & Must-Not-Happen Rules

> **CAUTION:** These are absolute rules derived from Yusuf's analysis and arena constraints. Violating any one causes mission failure.

### G.1 — NO PREMATURE TAG EXECUTION
Tags must NOT trigger navigation commands until the robot is within 1.25 m. The camera sees tags from 3+ metres — acting too early causes turns at wrong positions. The `ACTION_DISTANCE_THRESHOLD` gate in the existing code must be preserved.

### G.2 — NO BACKTRACKING TO LOGGED TAGS
After visiting #D, the robot must never return to #D. After visiting #F, must not return to #F. The visited-cell Bloom filter enforces this at every junction by deprioritising visited corridors.

### G.3 — NO COLLISION WITH CYLINDRICAL PILLAR
The CP at (0.9, 0.45) sits in a critical path. Must detect its curved LiDAR profile and arc around it — never drive straight through.

### G.4 — SERIAL LOGGING ONLY
ArUco IDs must be logged in the order they are first encountered, each logged exactly once. No duplicates, no out-of-order logging.

### G.5 — NO FALSE RED STOPS
Red tile detection requires 3 consecutive frames with >40% fill. The red petals/components in ARTPARK logos must NOT trigger HALT.

### G.6 — TAG 3 = 180° U-TURN, NOT 360°
The U-turn is exactly 180° — the robot reverses direction. It is NOT a full 360° spin-in-place. After the U-turn the robot must be facing the opposite direction and start BLUE arrow following.

### G.7 — ARROWS OVERRIDE JUNCTION LOGIC
During ARROW_FOLLOW mode, the tile's petal direction dictates the next heading — NOT the LiDAR junction scorer or Bloom filter priority. The arrow IS the path.

### G.8 — TAG DETECTION ALWAYS OVERRIDES ARROWS
If an AprilTag is seen during ARROW_FOLLOW mode, immediately interrupt arrow following and enter TAG_ACTION. Tags always take priority.

---

# Part H — Logging & Diagnostics

## H.1 Mission Log (`mission_log.csv`)

| Column | Type | Example |
|---|---|---|
| event | string | START, TAG_4, TAG_0, TAG_1, TAG_3, TAG_2, STOP |
| timestamp | float | Unix epoch |
| x | float | EKF x position at event |
| y | float | EKF y position at event |
| heading_deg | float | EKF heading at event |

Each event logged **at most once**. Append-only file. Written immediately when event occurs (not buffered).

## H.2 Arrow Count Log (`arrow_count_log.csv`)

| colour | count |
|---|---|
| GREEN | N |
| BLUE | N |
| ORANGE | N |

Written once at HALT.

## H.3 Real-Time Debug Topic (`/mission/debug`)

Publish at 2 Hz (extend existing `status_logger_node.py`):
```
STATE=EXPLORE | POS=(0.42, 0.88) | HDG=87° | TAGS=[4,0] | ARROW=GREEN:3 | FRONT=0.83m
```

## H.4 Odometry Trail (`odom_trail.csv`)

Log `(timestamp, x, y, yaw)` at 5 Hz throughout mission. Enables post-mortem path visualisation and debugging.

## H.5 Event Frame Capture

Auto-save camera frame whenever:
- A tag is detected (with bounding box overlay)
- An arrow direction is read (with colour blob highlighted)
- A tile colour is classified (green/red confirmation)
- A RECOVERY event triggers

Store in timestamped directory for post-run debugging.

---

# Part I — Edge Cases & Recovery

| # | Edge Case | Detection | Response |
|---|---|---|---|
| 1 | Tag re-detected after logging | `tag_id in logged_set` | 180° turnaround + mark corridor + EXPLORE |
| 2 | Tag partially visible | decision_margin < 30 | Ignore, drive closer until confidence rises |
| 3 | Two tags visible at once | Multiple detections in frame | Process nearest (smallest distance) first |
| 4 | Arrow colour missing 3 tiles | Empty contour for active colour | Creep 0.3 m, retry. Rotate ±15°. If still missing → EXPLORE fallback |
| 5 | Wrong colour blob detected | E.g., orange during green phase | Ignore all non-active colours unconditionally |
| 6 | Corner clipping | LiDAR wall < 0.08 m on one side | Stop, reverse 0.1 m, steer 5° away, resume |
| 7 | Cylinder blocks path | Front < 0.25 m, shape = "curved" | Arc circumnavigation using wider-side heuristic |
| 8 | Robot bumped/kidnapped | Large sudden odometry jump | RECOVERY → 360° scan for any tag → relocalize |
| 9 | Green tile seen again | Bloom filter or `start_logged` flag | Treat as normal tile, do NOT re-log START |
| 10 | Red tile false alarm | Red blob < 40% fill or < 3 frames | Ignore — require sustained high-fill detection |
| 11 | All Bloom directions "visited" | Junction scorer returns all visited | Override Bloom, pick forward or right-hand rule |
| 12 | EKF drift > 1 cell | No tag sighting for > 3 m of travel | Reduce speed, add ±5° yaw oscillation to widen scan |
| 13 | Camera glare on tag | Intermittent detection / low margins | Increase Gaussian blur sigma to 0.8 |
| 14 | Tag on perpendicular wall | Tag facing ≠ robot heading | Active ±90° scanning sweeps at junctions catch these |
| 15 | No open frontiers for 10s | LiDAR shows no passable gaps | Reverse 0.5 m → rotate 90° CW → re-scan |

---

# Part J — Phased Execution Plan

## Phase 1 — Foundation: LiDAR Motion & State Estimation

**Goal**: Robot drives straight in corridors, centres itself, detects junctions, executes turns, and fuses sensors into EKF.

**Tasks**:
- Adapt `lidar_analyzer_node.py` topic names to new arena (verify `/r1_mini/lidar` vs `/scan`)
- Verify LiDAR forward index mapping (index 180 = forward in physics convention)
- Implement bilateral PD corridor centering (extend existing `_move()`)
- Implement junction detection using existing opening algorithm
- Implement 90° and 180° pure-pivot turns with yaw tracking (extend existing `_start_pivot`)
- Configure `robot_localization` EKF with wheel odometry + IMU
- Add LiDAR wall-alignment heading correction into EKF (new capability)
- Implement cylindrical obstacle detection and circumnavigation

**Validation Checklist**:
- Robot drives full length of a single corridor without touching walls
- Robot stays centred to within ±3 cm
- Robot detects junctions reliably and stops
- Robot executes 90° turns and exits aligned with new corridor
- Robot circumnavigates the cylinder and re-centres
- EKF heading stays within ±2° of true heading over 3 m (verify with ground truth)

---

## Phase 2 — Perception: Tags & Floor Vision

**Goal**: Robot detects all 5 AprilTags, reads floor colours, and extracts arrow petal directions.

**Tasks**:
- Verify AprilTag dictionary ID mapping by decoding `tag_X_padded.png` images
- Adapt `aruco_detector_node.py` for new arena (topics, marker positions, frame IDs)
- Implement proximity-gated tag execution (distance < 1.25 m — existing but verify)
- Implement tag-anchored drift reset (PnP → pose override — existing but verify)
- Implement green/red floor colour detection with auto-HSV calibration
- Implement HSV petal centroid extraction for arrow direction (new node)
- Implement blue-circle masking for blue arrow phase
- Build template-matching fallback (4 reference orientations)
- Verify or add down-camera node (check URDF for camera config)

**Validation Checklist**:
- All 5 tags decode with correct IDs from their respective viewing directions
- Tag detection works during brief scanning sweeps, not just stationary
- EKF position snaps correctly when tag seen (compare to SDF ground truth)
- Green START tile detected reliably on boot
- Red STOP tile detected reliably (no false positives from red logo)
- Arrow direction correctly extracted on at least 5 different tile textures
- Template-matching fallback gives correct orientation when HSV fails

---

## Phase 3 — Memory: Visited Tracking

**Goal**: Robot tracks where it has been and avoids backtracking.

**Tasks**:
- Implement Bloom filter with spatial hashing (cell size 0.225 m)
- Implement marking every 0.10 m of travel from odometry
- Implement junction scoring (unvisited > visited, forward > turn)
- Implement tile counter (unique set + total count)
- Implement arrow counter (per-colour dictionary)

**Validation Checklist**:
- Bloom filter correctly marks cells along a manual drive path
- Previously visited cells detected as "present" even after drift correction
- Junction scorer correctly prefers unvisited over visited directions
- Tile counter increments exactly once per tile transition
- Arrow counter correctly tracks across colour-mode switches

---

## Phase 4 — Brain: State Machine

**Goal**: Complete mission FSM that ties all subsystems together.

**Tasks**:
- Implement the 9-state FSM with all transitions (refactor existing `NavControllerNode`)
- Implement TAG_ACTION dispatch for all 5 tag IDs
- Implement FOLLOW_GREEN, FOLLOW_BLUE, FOLLOW_ORANGE with stop-read-go protocol
- Implement UTURN state with blue arrow activation
- Implement re-detection handling (180° turnaround, no double-logging)
- Implement all watchdog timers
- Implement RECOVERY strategies (reverse, rotate, re-scan, fallback to EXPLORE)
- Implement HALT with graceful shutdown and complete log writing

**Validation Checklist**:
- FSM transitions correctly through all 9 states with synthetic inputs
- Re-detecting a logged tag triggers turnaround (not double-logging)
- Arrow mode correctly set/cleared on tag transitions
- RECOVERY successfully unsticks the robot in a dead-end
- HALT fires on red tile, not on red-coloured logo elements
- All watchdog timers fire at correct durations

---

## Phase 5 — Integration, Testing & Diagnostics

**Goal**: End-to-end mission success in Gazebo simulation.

**Tasks**:
- Create unified launch file for all nodes (extend existing `nav.launch.py`)
- Wire up all topics and message interfaces
- Implement CSV logging (mission log, arrow counts, odom trail)
- Extend `/mission/debug` real-time status topic
- Run full mission in Gazebo — verify tag encounter order, arrow counts, total time
- Tune PID gains, speeds, and thresholds based on simulation results
- Add event frame capture for debugging
- Stress-test recovery behaviours (manually block paths, add noise)
- Verify total mission completes within 5-minute budget

**Validation Checklist**:
- All nodes launch successfully and communicate
- Diagnostic topic reports correct state in real-time
- CSV files written with correct format and data
- Complete autonomous mission from green START to red STOP
- All tags logged in correct encounter order: TAG_4 → TAG_0 → TAG_1 → TAG_3 → TAG_2
- Arrow counts match expected tile traversals
- Total mission time < 300 seconds

---

# Part K — Timing Budget

| Segment | Est. Duration | Notes |
|---|---|---|
| INIT + calibration | 3–5 s | HSV sampling, sensor checks, initial scan |
| EXPLORE → Tag 4 (#A→#D) | 8–12 s | ~3 tiles cruising + junction |
| Tag 4 action (RIGHT turn) | 2 s | 90° pivot |
| EXPLORE → Tag 0 (#D→#F) | 5–8 s | ~2 tiles + CP navigation |
| Tag 0 action (LEFT turn) | 2 s | 90° pivot |
| EXPLORE → Tag 1 (#F→#H) | 8–12 s | ~3 tiles |
| Tag 1 action (green activate) | 1 s | Mode switch |
| GREEN arrow follow (5–8 tiles) | 20–35 s | Stop-read-go ≈ 3–4 s/tile |
| Tag 3 + UTURN | 3–5 s | 180° rotation |
| BLUE arrow follow (5–8 tiles) | 20–35 s | Stop-read-go ≈ 3–4 s/tile |
| Tag 2 action (LEFT turn + orange) | 2 s | Pivot + mode switch |
| ORANGE arrow follow (3–5 tiles) | 12–20 s | Stop-read-go ≈ 3–4 s/tile |
| HALT | 1 s | Zero velocity + log write |
| **Total** | **~90–140 s** | **Well within 5-min limit** |

### Speed Optimisations (if margin is tight)
1. Pipeline tag detection on a separate thread — never blocks driving
2. Reduce stop-read-go dwell from 200 ms to 100 ms if camera exposes fast
3. Single-frame arrow read when confidence > 95% — skip 5-frame vote
4. Don't scan at every junction — only scan near known tag-bearing walls
5. Pre-rotate during deceleration — start the turn before fully stopping

---

## ROS 2 Node Architecture Summary

| Node | Input Topics | Output Topics | Rate |
|---|---|---|---|
| **LiDAR Analyzer** | `/scan` or `/r1_mini/lidar` | `/lidar/analysis` (LidarAnalysis) | 10–20 Hz |
| **ArUco Detector** | `/camera/image_raw`, `/camera/camera_info` | `/aruco/detections` (ArucoDetection) | 15 Hz |
| **Floor Colour Detector** | `/camera_down/image_raw` | `/floor/colour` (custom msg) | 10 Hz |
| **Arrow Petal Detector** | `/camera_down/image_raw` | `/arrow/direction` (custom msg) | On-demand |
| **EKF** (`robot_localization`) | `/odom`, `/imu/data`, LiDAR heading, Tag pose | `/odom_fused` (Odometry) | 50 Hz |
| **Spatial Memory** | `/odom_fused` | Internal (queried by FSM) | Passive |
| **Mission FSM** | All perception + memory topics | `/cmd_vel` (Twist), CSV logs | 20 Hz |
| **Status Logger** | All detection topics, `/odom_fused` | `/mission_status`, `/mission/debug` | 2 Hz |

> **Implementation priority order:**
> 1. Phase 1 — LiDAR centering + turns + EKF (foundation — nothing works without this)
> 2. Phase 2a — AprilTag detection + tag ID verification (need this to validate route)
> 3. Phase 2b — Floor colour: green START + red STOP (must be bulletproof)
> 4. Phase 2c — Arrow petal direction (hardest CV task — test extensively in isolation)
> 5. Phase 4 — State machine (ties everything together)
> 6. Phase 3 — Bloom filter + counters (lowest priority — maze is highly constrained)
> 7. Phase 5 — Diagnostics + logging + end-to-end integration test
