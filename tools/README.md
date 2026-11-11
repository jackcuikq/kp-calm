# tools/

Helpers that are not ROS packages; nothing here needs a rebuild.

| Script | Where | What |
| --- | --- | --- |
| `playground.py` | laptop | Simulated map + camera, closed-loop runs of the real `navigation.py`, replay of real frames, interactive tuning, parameter sweeps, plots. See `PLAYGROUND.md`. |
| `maps/` | laptop | Example PNG maps for `playground.py --map` (regenerate with `--make-maps`). |
| `record.sh` | Jetson | `ros2 bag record` of camera, IMU, encoders, requests, tread commands, safety status and `navigation/telemetry` into `~/bags/<timestamp>`. |
| `bag2video.py` | laptop | Bag → annotated `.mp4`, CSVs, delay/offset plots, and JPEG frames for `playground.py --replay`. `pip install rosbags`. |
| `fake_camera.py` | Jetson (or any ROS 2 Jazzy) | Publishes saved frames/video on `camera/image_raw` so the team launch runs without the base launch. Open-loop. |
| `cam_web.py` | Jetson | Browser view of the ROS camera and the perception overlay on `http://ROBOT_IP:8081/` while the base launch owns the camera. `--save DIR` dumps frames for offline work. |

Laptop needs: `pip install opencv-python numpy matplotlib` (playground only).
