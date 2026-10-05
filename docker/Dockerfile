# Build and test image: ROS 2 Jazzy + the Nav2 packages this project uses + nlohmann json-schema-validator.
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-nav2-map-server ros-jazzy-nav2-planner ros-jazzy-nav2-controller \
      ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-behaviors ros-jazzy-nav2-lifecycle-manager \
      ros-jazzy-nav2-navfn-planner ros-jazzy-nav2-regulated-pure-pursuit-controller \
      ros-jazzy-nav2-costmap-2d ros-jazzy-nav2-msgs \
      ros-jazzy-nlohmann-json-schema-validator-vendor nlohmann-json3-dev python3-numpy \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /ws
COPY ros2_ws/src /ws/ros2_ws/src
RUN . /opt/ros/jazzy/setup.sh && cd /ws/ros2_ws && \
    colcon build --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release
CMD ["bash", "-c", ". /ws/ros2_ws/install/setup.sh && cd /ws/ros2_ws && colcon test --event-handlers console_direct+ --return-code-on-test-failure && colcon test-result --verbose"]
