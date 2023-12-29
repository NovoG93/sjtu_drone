ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}-ros-core-jammy

RUN apt-get update \
    && apt-get install -y \
    wget curl unzip \
    lsb-release \
    mesa-utils \
    build-essential \
    && apt-get clean

# Get gazebo binaries
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
    && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
    && apt-get update \
    && apt-get install -y \
    gazebo \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    python3-colcon-common-extensions python3-rosdep --no-install-recommends \
    && apt-get clean
RUN rosdep init && rosdep update

RUN mkdir -p /ros2_ws/src
COPY ./sjtu_drone_description /ros2_ws/src/sjtu_drone_description
COPY ./sjtu_drone_bringup /ros2_ws/src/sjtu_drone_bringup
COPY ./sjtu_drone_control /ros2_ws/src/sjtu_drone_control

RUN curl -L https://github.com/osrf/gazebo_models/archive/refs/heads/master.zip -o /tmp/gazebo_models.zip \
    && unzip /tmp/gazebo_models.zip -d /tmp && mkdir -p ~/.gazebo/models/ && mv /tmp/gazebo_models-master/* ~/.gazebo/models/ \
    && rm -r /tmp/gazebo_models.zip

WORKDIR /ros2_ws
RUN /bin/bash -c 'cd /ros2_ws/ \
    && source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build'

CMD ["/bin/bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch sjtu_drone_bringup sjtu_drone_bringup.launch.py"]
