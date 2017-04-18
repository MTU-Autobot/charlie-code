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

# Interface Board

To interface with the board you need to setup the udev rules found [here](https://www.pjrc.com/teensy/loader_linux.html)

# Networking

The robot has the static IP address ```141.219.120.14``` on Michigan Techs network

Add the robot to your ```\etc\hosts``` file with the lines

```
# Autobot Charlie
141.219.120.14 charlie
```

This will let you SSH into the robot by using ```ssh ubuntu@charlie``` instead of ```ssh ubuntu@141.219.120.14```

# Power Diagram
Here is a flowchart showing the power distribution on the robot
![Power Distribution](Power Diagram.svg)
