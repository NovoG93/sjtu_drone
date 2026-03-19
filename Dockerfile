ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-core-noble
ARG ROS_DISTRO

RUN apt-get update \
    && apt-get install -y \
    lsb-release \
    mesa-utils \
    build-essential \
    && apt-get clean

# Install ROS Gz (Gazebo Harmonic) dependencies
RUN apt-get update \
    && apt-get install -y \
    ros-${ROS_DISTRO}-ros-gz \
    python3-colcon-common-extensions python3-rosdep --no-install-recommends \
    && apt-get clean
RUN rosdep init && rosdep update

RUN mkdir -p /ros2_ws/src
COPY ./sjtu_drone_description /ros2_ws/src/sjtu_drone_description
COPY ./sjtu_drone_bringup /ros2_ws/src/sjtu_drone_bringup
COPY ./sjtu_drone_control /ros2_ws/src/sjtu_drone_control

WORKDIR /ros2_ws
RUN apt-get update && \
    /bin/bash -c 'cd /ros2_ws/ \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build' && \
    apt-get clean

CMD ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py"]
