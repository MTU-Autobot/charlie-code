# Dependencies

* [razor_imu_9dof](https://github.com/KristofRobot/razor_imu_9dof)
* [ublox](https://github.com/KumarRobotics/ublox)
* [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper)

# Building & Installation

```bash
# Download.
cd catkin_ws/src
git clone https://github.com/MTU-Autobot/charlie-code.git
cd ..

# Install deb dependencies.
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

# Build.
catkin_make
```

# Razor IMU

To access the Razor IMU over USB, you may need to add yourself to the dialout group using.

```bash
sudo adduser ${USER} dialout
```
