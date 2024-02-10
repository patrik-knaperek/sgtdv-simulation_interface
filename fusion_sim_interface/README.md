# **FusionSimInterface package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Interface between AMZ FSSIM workspace and SGT-DV workspace for cone detections.

___

`fusionSimInterface` node converts cone detection topics from FSSIM into SGT topics.

**[Requires AMZ FSD skeleton & FSSIM installed](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/sgt-noetic-devel/SGT-DV_install_man.md)**

### Related packages
* [`fusion`](../../fusion/README.md)
* [`calibration`](../../calibration/README.md)

### Topic conversions
* `fssim/camera/cones [sensor_msgs::PointCloud2] → camera_cones [sgtdv_msgs::ConeStampedArr]`
* `fssim/lidar/cones [sensor_msgs::PointCloud2] → lidar_cones [sgtdv_msgs::Point2DStampedArr]` 

## Compilation
* compile standalone
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin clean
$ source ${FSD_ROOT}/devel/setup.bash
$ catkin build fusion_sim_interface
```
* compile with all packages needed for runnung with FSSIM
```sh
$ ${SGT_ROOT}/ros_implementation/build_sim.sh
```
## Configuration
* `${FSD_ROOT}/src/fssim_interface/fssim_config/sensors/sensors_1.yaml` : set parameters of generated detections

## Launch
* run `fusionSimInterface` node separately:
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ rosrun fusion_sim_interface fusion_sim_interface
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

For launching along with `fusion` node, check [this README](../../fusion/README.md).