# USV Simulation 2D

Simlation of the rigid body dynamics of a simple differential drive USV in 2D, in ROS 2 (Humble).

## Requirements
Tested with:
- Ubuntu 22.04
- ROS 2 (Humble Hawksbill)

---

This package uses [SFML](https://www.sfml-dev.org/index.php) and [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page), which are build alongside this package using CMake's FetchContent.
SFML depends on system packages which have to be installed before building this package:

```
sudo apt update
sudo apt install \
    libxrandr-dev \
    libxcursor-dev \
    libudev-dev \
    libopenal-dev \
    libflac-dev \
    libvorbis-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libdrm-dev \
    libgbm-dev
```

## Build

Clone this package to a ROS 2 workspace and build with colcon:
```
cd ~/ros_ws/src/
git clone git@github.com:Aaveq-Robotics/usv_sim_2d.git
cd ~/ros_ws/
source /opt/ros/humble/setup.bash
colcon build
```

## Content

### Node: sim

#### Topics

| Name      |Type   | I/O   |
| ---       | ---   | ---   |
| control_output | [`aaveq_ros_interfaces::msg::ControlOutput`](https://github.com/Aaveq-Robotics/aaveq_ros_interfaces/blob/main/msg/ControlOutput.msg) | Subscriber |
| sim_state | [`aaveq_ros_interfaces::msg::SimState`](https://github.com/Aaveq-Robotics/aaveq_ros_interfaces/blob/main/msg/SimState.msg) | Publisher |


## Usage

Run the node after sourcing the workspace:
```
ros2 run usv_sim_2d sim
```

