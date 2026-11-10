# Shared runtime source

These five helpers are the source of the device code already installed on each lab Jetson at `/opt/a2r-runtime`. The ROS base and the default camera/OLED services import them from there. They are included so you can read the code that appears in your tracebacks; you do not build or install this folder.

The passive demo-state reader (`a2r_state.py`) remains because the hardware, camera and OLED code import it; with no state file it reports inactive.

The third-party Sphero SDK is not redistributed here. The lab Jetsons carry it from Sphero's official repository at commit `cc82b799a70e583773a93b33c151a64ddce727c3`, under its own license.

Calibration is per robot and is never copied between robots.
