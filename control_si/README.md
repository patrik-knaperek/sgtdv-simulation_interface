# **ControlSimulationInterface package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Interface between AMZ FSSIM workspace and SGT-DV workspace for state and command topics.

___

`control_si` node allows for controlling the FSSIM vehicle by commands from `path_tracking` node and provides conversion of state topics (pose and velocity) in the other direction for feedback.

**[Requires AMZ FSD skeleton & FSSIM installed](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/master/README.md)**

### Related packages
* `path_tracking` (SGT-DV)
* `control_pure_pursuit` (AMZ)

### Topic conversions
* `/control/pure_pursuit/center_line [geometry_msgs::PolygonStamped] → path_planning/trajectory [sgtdv_msgs::Point2DArr]`
* `/estimation/slam/state [fsd_common_msgs::CarState] → odometry/pose [sgtdv_msgs::CarPose]` 
* `/estimation/velocity_estimation/velocity_estimation [fsd_common_msgs::CarStateDt] → odometry/velocity [sgtdv_msgs::CarVel]`
* `path_tracking/cmd [sgtdv_msgs::Control] → sgt/control_command [fsd_common_msgs::ControlCommand]`

## Compilation
* compile standalone
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin clean
$ source ${FSD_ROOT}/devel/setup.bash
$ catkin build control_si
```
* compile with all packages needed for runnung with FSSIM
```sh
${SGT_ROOT}/ros_implementation/build_sim.sh
```
## Setup
In `fsd_skeleton/src/fssim_interface/config/config.yaml` file set parameter `fsd/cmd` to `"sgt/control_command"`.

## Launch
* run `control_si` node separately:
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ rosrun control_si control_si
```
* run with other nodes in FSSIM setup
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ roslaunch master trackdrive_sim.launch
```

Launch FSSIM:

```sh
$ source ${FSD_ROOT}/devel/setup.bash
$ roslaunch fssim_interface fssim.launch
```
In new terminal:
```sh
$ source ${SGT_ROOT}/devel/setup.bash
$ roslaunch control_meta trackdrive.launch
```

For launching along with `path_tracking` node, check [this README](../../path_tracking/README.md).