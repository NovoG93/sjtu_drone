# Overview

sjtu_drone is a quadrotor simulation program forked from [tum_simulator](http://wiki.ros.org/tum_simulator), developed using ROS + Gazebo.

The acronym 'sjtu' stands for Shanghai Jiao Tong University. This package has been used in the past for testing algorithms for the [UAV contest at SJTU](http://mediasoc.sjtu.edu.cn/wordpress)

# Requirements

This package is compatible with ROS Noetic version (Ubuntu 20.04). Existing versions on the internet support at most until Gazebo 7. After Gazebo 8.0, the API has gone significant changes; therefore, it was necessary to adapt the package to Gazebo 8.0+ API. As the default version of Gazebo coming with ROS Noetic is 11.0, it is suggested that do not use the full installation but the [desktop installation](http://wiki.ros.org/noetic/Installation/Ubuntu).

# Downloading and building

```
cd ~/git && git clone git@github.com:NovoG93/sjtu-drone.git 
cd ~/catkin_ws/src && ln -s ~/git/sjtu-drone
cd .. && catkin build
```
# Run

```
source ~/catkin_ws/devel/setpu.bash && rospack profile
roslaunch sjtu_drone start.launch
```

You sould now see the following:

![Gazebo](img/drone.jpg)

## Sensors
The folowing sensors are currently implemented:
- front_camera: /drone/front_camera/iage_raw
- downward_camera: /drone/down_camera/iage_raw
- sonar: /drone/sonar
- imu: /drone/imu

For more see the following image:
![rosgraph](./img/rosgraph.png)
