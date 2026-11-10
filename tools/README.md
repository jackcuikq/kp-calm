# tools/

Helpers that are not ROS packages; nothing here needs a rebuild.

| Script | Where | What |
| --- | --- | --- |
| `sim.py` | laptop | Closed-loop simulator: runs the real `navigation.py` pipeline on a synthetic camera + road, moves a unicycle car. Tune gains, check signs/latency, make the offset-over-time plot. `python tools/sim.py --help` |
| `fake_camera.py` | Jetson (or any ROS 2 Jazzy) | Publishes saved frames/video on `camera/image_raw` so the team launch runs without the base launch. Open-loop. |
| `cam_web.py` | Jetson | Browser view of the ROS camera and the perception overlay on `http://ROBOT_IP:8081/` while the base launch owns the camera. `--save DIR` dumps frames for offline work. |

Laptop needs: `pip install opencv-python numpy matplotlib` (sim only).
