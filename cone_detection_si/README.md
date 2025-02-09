# **ConeDetectionSimulationInterface package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Interface between AMZ FSSIM workspace and SGT-DV workspace for cone detections.

___

`cone_detection_si` node converts cone detection topics from FSSIM into SGT topics.

**[Requires AMZ FSD skeleton & FSSIM installed](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/master/README.md)**

### Related packages
* [`fusion`](../../fusion/README.md)
* [`calibration`](../../calibration/README.md)

### Topic conversions
* `fssim/camera/cones [sensor_msgs::PointCloud2] → camera/cones [sgtdv_msgs::ConeStampedArr]`
* `fssim/lidar/cones [sensor_msgs::PointCloud2] → lidar/cones [sgtdv_msgs::Point2DStampedArr]` 

## Compilation
* compile standalone
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin clean
$ source ${FSD_ROOT}/devel/setup.bash
$ catkin build cone_detection_si
```
* compile with all packages needed for runnung with FSSIM
```sh
$ ${SGT_ROOT}/ros_implementation/build_sim.sh
```
## Configuration
* `${FSD_ROOT}/src/fssim_interface/fssim_config/sensors/sensors_1.yaml` : set parameters of generated detections

## Launch
* run `cone_detection_si` node separately:
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ rosrun cone_detection_si cone_detection_si
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