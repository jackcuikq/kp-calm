# Team Kp Calm — Perceptive UGV racing (COSC 281)

A camera-only yellow-road follower for the course UGV. It follows the yellow road, avoids blue
"water", takes the wider branch at forks, and turns in place to search when it loses the road.

## Submitted version

- Branch `main`. The navigation code and race configuration were last changed in commit
  `0701c4f` ("race", 2026-11-17).
- **Race-day setting:** we raced on the **High** profile with `requested_forward_speed_m_s`
  set to **0.65** on the car. The committed `navigation.yaml` still says `1.40`. To reproduce
  the race run, change that line to `0.65` before building (see [Known limitations](#known-limitations)).

## What is ours

Everything outside the two items below is the supplied course baseline and is unchanged
(`a2r_robot`, `a2r_robot_interfaces`, `course_robot_interfaces`, `runtime/`, `viewer/`).

| Path | Purpose |
| --- | --- |
| `robot/src/team_navigation/team_navigation/navigation.py` | The single ROS 2 node `navigation`. A 30 Hz timer runs perception (HSV yellow/blue/orange masks), estimation (stale-frame rejection, EMA on the offset, scan state), planning (follow / scan / stop, speed from the offset) and PD steering, then publishes one `MotionRequest` per tick. |
| `robot/src/team_navigation/config/navigation.yaml` | All tuning parameters: speeds, PD gains, HSV thresholds, blue corridor, scan and fork settings. They are checked when the node starts. |
| `robot/src/team_navigation/team_navigation/record_camera.py` | `record_camera` tool: saves a short AVI clip from the camera topic for offline threshold tuning. |
| `robot/scripts/start_robot_motion.sh` | Starts the base with motion enabled and our measured track width (0.22 m). |
| `tests/test_project_robot.py` | Offline tests for the navigation logic (added alongside each change). |

## Dependencies

There are no dependencies beyond the supplied Jetson image: ROS 2 Jazzy, `~/ros-venv`,
OpenCV, NumPy and `cv_bridge`. The offline tests need only `pytest` (`requirements-test.txt`).

## Build (on the Jetson)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros-venv/bin/activate
cd ~/a2r-student/robot
python -m colcon build --symlink-install
```

Four packages should finish.

## Launch (on the Jetson)

Terminal A starts the base with motion enabled:

```bash
cd ~/a2r-student/robot
./scripts/start_robot_motion.sh
```

Terminal B starts our navigation node:

```bash
source /opt/ros/jazzy/setup.bash
source ~/a2r-student/robot/install/setup.bash
source ~/ros-venv/bin/activate
source /etc/a2r-robot/robot.sh
ros2 launch team_navigation team.launch.py namespace:="$ROBOT_NAMESPACE"
```

On the gamepad:
1. Release the sticks and press **Y** to select the speed profile. We used **High**.
2. **Hold RB** to let the car drive autonomously.
3. Press **A** to stop.

For a first check without motion, use the disabled launch in [robot/README.md](robot/README.md#run).

## Reproducing a representative test

1. **Offline (no robot, no ROS):** from the repository root, run:
   ```bash
   python3 -m venv .venv && source .venv/bin/activate
   python -m pip install -r requirements-test.txt
   python -m pytest tests -q
   ```
   All 93 tests should pass. The navigation tests use synthetic frames to check blue rejection,
   scan entry and exit, the stop after one empty revolution, and the PD steering sign.
2. **On the car:** lay a short yellow paper road with one tight curve and blue paper along both
   edges. Build, launch as above, set `requested_forward_speed_m_s: 0.65` and hold RB on the
   High profile. The car should follow the curve. If its nose crosses into blue, it should
   turn in place and resume once it sees yellow. If it is driven off the end of the road, it
   should scan one full turn and then stop.

## Known limitations

- **High speed:** at the committed 1.40 m/s the car reacts late in curves. The PD gains we tried
  did not fix this, so we raced at 0.65 m/s. Image delay is a suspected cause, but we never
  measured it.
- **Fork false trigger:** approaching the fork, the car can see blue before the yellow branches.
  It then starts a scan instead of following the visible road.
- **Fixed scan direction:** the car always scans in one direction, set per track by
  `scan_anticlockwise`. It does not pick a direction from what it last saw.
- **Lighting:** the HSV thresholds were calibrated under the lab lighting and may need retuning
  elsewhere.
- **Not implemented:** the IMU is subscribed but unused, the car does not use odometry, and the
  planned arc-sampling lookahead was never built.

## Not included

`build/`, `install/`, `log/`, camera clips and videos, caches and credentials are git-ignored.
Lab passwords are on Canvas.

## Attribution and AI disclosure

The baseline is from the A2R Lab course repository ([MIT license](LICENSE),
[course attribution](robot/NOTICE.md)). The course's own setup guide is kept in
[docs/SETUP.md](docs/SETUP.md) and [robot/README.md](robot/README.md).

We used LLM tools (Claude, GitHub Copilot and Codex) for initial code generation, debugging and
drafting. The team reviewed and tested all suggestions on the robot before use.
