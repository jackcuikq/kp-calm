"""Source-layout and interface checks; no ROS or robot needed."""
from pathlib import Path

REPO=Path(__file__).resolve().parents[1]


def test_viewer_interface_copy_matches_robot():
    base=REPO/'robot/src/a2r_robot_interfaces'
    viewer=REPO/'viewer/src/a2r_robot_interfaces'
    for p in base.rglob('*'):
        if p.is_file() and '__pycache__' not in p.parts:
            assert p.read_bytes()==(viewer/p.relative_to(base)).read_bytes()


def test_public_request_matches_installed_robot_contract():
    text=(REPO/'robot/src/course_robot_interfaces/msg/MotionRequest.msg').read_text()
    fields=[line for line in text.splitlines() if line and not line.startswith('#')]
    assert fields==['std_msgs/Header header','float64 forward_speed_m_s','float64 yaw_rate_rad_s']
    starter=(REPO/'robot/src/team_navigation/team_navigation/navigation.py').read_text()
    assert 'msg.header.stamp =' in starter
    assert 'msg.stamp =' not in starter


def test_geometry_has_no_assumed_numeric_value():
    launch=(REPO/'robot/src/a2r_robot/launch/robot.launch.py').read_text()
    assert "DeclareLaunchArgument('track_width_m',default_value='0.0')" in launch
    assert 'tread_separation_m' not in launch


def test_camera_recorder_is_installed_and_uses_compressed_topic():
    setup=(REPO/'robot/src/team_navigation/setup.py').read_text()
    recorder=(REPO/'robot/src/team_navigation/team_navigation/record_camera.py').read_text()
    assert 'record_camera = team_navigation.record_camera:main' in setup
    assert 'camera/image_raw/compressed' in recorder
    assert 'MotionRequest' not in recorder
