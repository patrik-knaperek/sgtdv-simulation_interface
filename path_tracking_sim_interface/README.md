# **PathTrackingSimInterface package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Interface between AMZ FSSIM workspace and SGT-DV workspace for state and command topics.

___

`pathTrackingSimInterface` node allows for controlling the FSSIM vehicle by commands from `pathTracking` node and provides conversion of state topics (pose and velocity) in the other direction for feedback.

**[Requires AMZ FSD skeleton & FSSIM installed](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/sgt-noetic-devel/SGT-DV_install_man.md)**

### Related packages
* `path_tracking` (SGT-DV)
* `control_pure_pursuit` (AMZ)

### Topic conversions
* `/control/pure_pursuit/center_line [geometry_msgs::PolygonStamped] → pathplanning_trajectory [sgtdv_msgs::Point2DArr]`
* `/estimation/slam/state [fsd_common_msgs::CarState] → pose_estimate [sgtdv_msgs::CarPose]` 
* `/estimation/velocity_estimation/velocity_estimation [fsd_common_msgs::CarStateDt] → /velocity_estimate [sgtdv_msgs::CarVel]`
* `pathtracking_commands [sgtdv_msgs::Control] → sgt/control_command [fsd_common_msgs::ControlCommand]`

## Compilation
* compile standalone
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin clean
$ source ${FSD_ROOT}/devel/setup.bash
$ catkin build path_tracking_sim_interface
```
* compile with all packages needed for runnung with FSSIM
```sh
${SGT_ROOT}/ros_implementation/build_sim.sh
```
## Setup
In `fsd_skeleton/src/fssim_interface/config/config.yaml` file set parameter `fsd/cmd` to `"sgt/control_command"`.

## Launch
* run `pathTrackingSimInterface` node separately:
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ rosrun path_tracking_sim_interface path_tracking_sim_interface
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

For launching along with pathTracking node, check [this README](../../path_tracking/README.md).