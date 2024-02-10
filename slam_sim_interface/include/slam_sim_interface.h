/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#pragma once

/* ROS */
#include <ros/ros.h>
#include <std_msgs/Empty.h>

/* AMZ FSSIM */
#include <fsd_common_msgs/Map.h>
#include <fsd_common_msgs/CarState.h>

/* SGT-DV */
#include <sgtdv_msgs/ConeArr.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include <sgtdv_msgs/Point2DArr.h>

class SLAMSimInterface
{
public:
  SLAMSimInterface();
  ~SLAMSimInterface() = default;

  // Setters
  void setPublishers(const ros::Publisher &map_pub, const ros::Publisher &pose_pub
                      , const ros::Publisher &loop_close_pub/*, const ros::Publisher &velPub*/)
  {
    map_pub_ = map_pub;
    pose_pub_ = pose_pub;
    loop_close_pub_ = loop_close_pub;
    //m_velocityPublisher = velPub;
  };

  // Callbacks
  void mapCallback(const fsd_common_msgs::Map::ConstPtr &msg);
  void stateCallback(const fsd_common_msgs::CarState::ConstPtr &msg);

  void publishMap();

private:
  static constexpr float LOOK_AHEAD_DISTANCE = 15.0;

  // SGT-DV --> FSSIM

  // FSSIM --> SGT-DV
  void addToMap(const fsd_common_msgs::Cone &cone);
  int findLookAheadConeIdx(std::vector<fsd_common_msgs::Cone> cones);
  void actualizeMap();
  void loopClosure();
  std::vector<fsd_common_msgs::Cone> cones_blue_, cones_yellow_;
  ros::Publisher map_pub_, pose_pub_, loop_close_pub_/*, m_velocityPublisher*/;

  sgtdv_msgs::ConeArr map_;
  sgtdv_msgs::CarPose car_pose_;

  int cones_blue_count_, cones_yellow_count_;
  bool map_ready_, loop_closure_;
};