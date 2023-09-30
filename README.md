# USV Simulation 2D

Simlation of the rigid body dynamics of a simple differential drive USV in 2D, in ROS 2 (Humble).

<!-- vscode-markdown-toc -->
* 1. [Cloning](#Cloning)
* 2. [Requirements](#Requirements)
* 3. [Build](#Build)
* 4. [Content](#Content)
	* 4.1. [Node: sim](#Node:sim)
		* 4.1.1. [Topics](#Topics)
		* 4.1.2. [Parameters](#Parameters)
* 5. [Usage](#Usage)

<!-- vscode-markdown-toc-config
	numbering=true
	autoSave=true
	/vscode-markdown-toc-config -->
<!-- /vscode-markdown-toc -->


##  1. <a name='Cloning'></a>Cloning

Clone this package to a ROS 2 workspace:
```
cd ~/ros_ws/src/
git clone git@github.com:Aaveq-Robotics/usv_sim_2d.git
cd ~/ros_ws/
```

##  2. <a name='Requirements'></a>Requirements
Tested with:
- Ubuntu 22.04 (Jammy Jellyfish)
- ROS 2 (Humble Hawksbill)

---

Package dependencies are handled with [rosdep](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Rosdep.html). In the workspace, run:

```
rosdep install --from-paths src -y --ignore-src
```

**NOTE:** if this is the first time using rosdep, it must first be initialized via:
```
sudo rosdep init
rosdep update
```

There is also a dependency to our own [aaveq_ros_interfaces](https://github.com/Aaveq-Robotics/aaveq_ros_interfaces) package, which needs to be cloned to `src/` as well:
```
cd ~/ros_ws/src/
git clone git@github.com:Aaveq-Robotics/aaveq_ros_interfaces.git
cd ~/ros_ws/
```

##  3. <a name='Build'></a>Build

Build with colcon:
```
cd ~/ros_ws/
source /opt/ros/humble/setup.bash
colcon build
```

##  4. <a name='Content'></a>Content

###  4.1. <a name='Node:sim'></a>Node: sim

####  4.1.1. <a name='Topics'></a>Topics

| Name      |Type   | I/O   |
| ---       | ---   | ---   |
| control_output | [`aaveq_ros_interfaces::msg::ControlOutput`](https://github.com/Aaveq-Robotics/aaveq_ros_interfaces/blob/main/msg/ControlOutput.msg) | Subscriber |
| sim_state | [`aaveq_ros_interfaces::msg::SimState`](https://github.com/Aaveq-Robotics/aaveq_ros_interfaces/blob/main/msg/SimState.msg) | Publisher |

####  4.1.2. <a name='Parameters'></a>Parameters
| Name      	| Description   					|Type   	| Default	|
| ---       	| ---   							| ---   	| ---		|
| sim_freq 		| Simulation frequency 				| `double` 	| 60.0		|
| vessel_config | Path to vessel configuration file	| `string` 	| ""		|


##  5. <a name='Usage'></a>Usage

Run the node after sourcing the workspace, with the vessel_config parameter set:
```
ros2 run usv_sim_2d sim --ros-args -p vessel_config:=src/usv_sim_2d/vessel_configs/diff_drive_simple.json
```


