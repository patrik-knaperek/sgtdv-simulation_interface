# **SLAMSimulationInterface package**

___

© **SGT Driverless**

**Authors:** Patrik Knaperek

**Objective:** Interface between AMZ FSSIM and SGT-DV workspaces to substitue SLAM algorithm for Path Planning testing.
___

FSSIM SLAM node latches the whole map on the `estimation/slam/map` topic. This node can simulate behavior of SLAM in the first lap, when the map is extended and published gradually as the vehicle is crossing the track. After the first lap, it detects loop closure and publishes the whole map.

*Note: If only global navigation is being tested, set the `map_ready_` and `loop_closure_` initial value to `true`.*
```cpp
SlamSI::SlamSI(ros::NodeHandle& nh) :
  ...
  cones_blue_count_(0), cones_yellow_count_(0), map_ready_(false), loop_closure_(false)
{
 ...
}
```

**[Requires AMZ FSD skeleton & FSSIM installed](https://gitlab.com/sgt-driverless/simulation/fsd_skeleton/-/blob/master/README.md)**

### Related packages
* [`path_planning`](../../path_planning/README.md) (SGT-DV)
* `slam_slam` (AMZ)

### Topic conversions
* `/estimation/slam/map [fsd_common_msgs::Map] → /slam/map [sgtdv_msgs::ConeArr]`
* `/estimation/slam/state [fsd_common_msgs::CarState] → /slam/pose [sgtdv_msgs::CarPose], odometry/velocity [sgtdv_msgs::CarVel]`

## Compilation
* compile standalone
```sh
$ cd ${SGT_ROOT}/ros_implementation
$ catkin clean
$ source ${FSD_ROOT}/devel/setup.bash
$ catkin build slam_si
```
* compile fith other packages for running with FSSIM
```sh
$ ${SGT_ROOT}/build_sim.sh
```

## Launch
Launch FSSIM first:
```sh
$ source ${FSD_ROOT}/devel/setup.bash
$ roslaunch fssim_interface fssim.launch
```

Run SLAMSimInterface node separately:
```sh
$ source ${SGT_ROOT}/ros_implementation/devel/setup.bash
$ rosrun slam_si slam_si
```

Run with whole FSSIM workflow
```sh
$ source ${SGT_ROOT}/devel/setup.bash
$ roslaunch master trackdrive_sim.launch
```

For launching along with PathPlanning node instead, check [this README](../../path_planning/README.md).