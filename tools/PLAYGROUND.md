# Playground — run `navigation.py` without the car

`tools/playground.py` runs the **real** `navigation.py` + `navigation.yaml`
(any teammate's version) against a simulated map and camera, or against real
camera frames. No ROS needed; works on a Mac.

```bash
pip install opencv-python numpy matplotlib
cd kp-calm
python tools/playground.py --help
```

Every run calls the same four hooks `tick()` calls on the robot:
`process_image → update_estimate → choose_action → compute_motion`, then
applies the base's tread conversion and speed-profile clamp, moves a car model,
and renders the next camera frame. Both observation contracts are understood
(`offset_px` in 160-px units, or `offset` as a fraction of half the width).

## 1. Closed loop on a built-in track

```bash
python tools/playground.py                                   # oval, live window
python tools/playground.py --track fork --tiles --blue        # paper seams, blue borders, a Y-fork
python tools/playground.py --track wiggle --noise 6 --jpeg 70 --exposure 0.1   # camera realism
python tools/playground.py --set requested_forward_speed_m_s=0.15               # Low-profile speed
python tools/playground.py --limit 0.6 --set requested_forward_speed_m_s=0.3    # Medium profile
python tools/playground.py --latency 6                        # 300 ms image delay: does it oscillate?
```

Tracks: `oval`, `figure8`, `wiggle`, `fork` (branch peels off on the right-hand
end and rejoins), `straight`.

Window keys: `space` pause · `n` step · `r` reset · `←→` nudge the car sideways
5 cm · `↑↓` turn it 10° · `m` toggle the node's `navigation/debug` image ·
`+ -` requested speed · `[ ]` the parameter named by `--tune KEY` · `s`
screenshot · `q` quit.

Right panel: top-down map with the driven path; below it, whatever the node
publishes with `self.publish_debug(bgr)` (perception overlay).

## 2. Headless runs, plots, numbers

```bash
python tools/playground.py --headless --seconds 90 --track fork --tiles --blue \
    --set requested_forward_speed_m_s=0.15 --video fork.mp4 --plot fork.png --csv fork.csv --json fork.json
```

Printed summary:

| field | meaning |
| --- | --- |
| `cte_mean_cm`, `cte_max_cm` | cross-track error to the followed centreline |
| `margin_min_cm` | smallest distance from the car to the road edge (0 = left the road) |
| `off_road_s` | seconds spent off the yellow |
| `stop_s` | seconds the node asked for zero motion |
| `fork_flag_s` | seconds the node believed it was at a fork |
| `branch_switches` | lane changes between main road and branch (should be 0 or 1 per pass) |
| `pipeline_ms_*` | time of the four hooks per tick on **this** laptop (robot budget: 50 ms) |

`--plot` writes the offset-over-time / command / cross-track figure the handout asks for.

## 3. Parameter sweep

```bash
python tools/playground.py --sweep requested_forward_speed_m_s=0.05,0.15,0.3 --limit 0.6 --track wiggle
python tools/playground.py --sweep kp_rad_s_per_px=0.01,0.02,0.04 --latency 4
python tools/playground.py --sweep steering_kp=0.4,0.8,1.6 --nav ~/suhavi_wip/navigation.py
```

One headless run per value, one table row each.

## 4. Replay real camera frames (open loop)

Record a run on the car (base + team launch running, prepared terminal):

```bash
bash ~/Projects/kp-calm/tools/record.sh lap1        # Ctrl+C to stop -> ~/bags/<timestamp>_lap1
```

On the laptop:

```bash
scp -r a2r-lab-jetson@ROBOT_IP:bags/<timestamp>_lap1 .
pip install rosbags
python tools/bag2video.py <timestamp>_lap1 --frames lap1_frames
```

That writes `<name>.mp4` (camera with offset / request / tread / delay
overlaid, debug image beside it), `_telemetry.csv`, `_request.csv`,
`_motors.csv`, `_imu.csv`, a `.png` with offset-over-time, request vs tread
command, image age + pipeline time, gyro yaw rate (the handout's plots, from
real data), and `lap1_frames/` for replay:

```bash
python tools/playground.py --replay ~/frames                 # window, 20 fps
python tools/playground.py --replay ~/frames --step          # one frame per key press
python tools/playground.py --replay ~/frames --headless --plot replay.png --csv replay.csv
python tools/playground.py --replay lap1.mp4 --perception node --nav ~/suhavi_wip/navigation.py
```

Prints how often the road was found, pipeline time, stop and fork counts. The
pictures do not react to the commands here — this is for perception
thresholds and for checking what the node *would* have requested.

## 5. Your own map

Paint a top-down map in any editor: yellow road (`H≈26`, strong S/V), blue
borders, grey floor; or start from the generated examples:

```bash
python tools/playground.py --make-maps tools/maps          # writes oval/figure8/wiggle/fork/straight .png + .json
python tools/playground.py --map tools/maps/fork.png --ppm 100 --start 0.9,2.1,-89.4
```

`--ppm` = pixels per metre, `--start x,y,heading_deg` = metres from the
bottom-left corner, heading counter-clockwise from +x. Cross-track error is
not available for PNG maps (no centreline); `margin_min_cm`/`off_road_s` are.

## 6. Which navigation.py / yaml

```bash
python tools/playground.py --nav ~/suhavi_wip/navigation.py --yaml ~/suhavi_wip/navigation.yaml --perception node
```

`--perception auto` (default): the node's `process_image`; while it returns
nothing, a reference yellow detector fills in so control can be tuned first.
`node` forbids the fallback, `ref` forces it.

## Model and limits

Ideal pinhole camera (`--cam-height 0.15 --cam-pitch 15 --hfov 70`, change to
match the mount), optional pixel noise / exposure drift / JPEG round trip,
image latency in 50 ms ticks (default 2), unicycle car with first-order
actuator lag (`--lag 0.15`), optional yaw-rate process noise. Track width
0.20 m and Low profile 0.15 m/s by default. No slip, no real motor dynamics,
no lighting — gains, signs, latency margins and fork logic transfer to the
car; colour thresholds must be checked on real frames (section 4).
