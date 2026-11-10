FROM osrf/ros:jazzy-desktop
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-compressed-image-transport ros-jazzy-rmw-cyclonedds-cpp qtbase5-dev python3-colcon-common-extensions build-essential && rm -rf /var/lib/apt/lists/*
COPY src /opt/a2r-viewer-ws/src
RUN /bin/bash -c 'source /opt/ros/jazzy/setup.bash && cd /opt/a2r-viewer-ws && colcon build --packages-up-to a2r_rviz_panels --cmake-args -DCMAKE_BUILD_TYPE=Release' && rm -rf /opt/a2r-viewer-ws/build /opt/a2r-viewer-ws/log
COPY config /opt/a2r-viewer-config
