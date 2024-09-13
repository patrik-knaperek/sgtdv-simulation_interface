/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/slam_si.h"

SlamSI::SlamSI(ros::NodeHandle& nh) :
  /* ROS interface init */
  map_pub_(nh.advertise<sgtdv_msgs::ConeArr>("slam/map", 1, true)),
  pose_pub_(nh.advertise<sgtdv_msgs::CarPose>("slam/pose", 1)),
  loop_close_pub_(nh.advertise<std_msgs::Empty>("slam/loop_closure", 1, true)),
  
  map_sub_(nh.subscribe("estimation/slam/map", 1, &SlamSI::mapCallback, this)),
  pose_sub_(nh.subscribe("estimation/slam/state", 1, &SlamSI::stateCallback, this)),

  cones_blue_count_(0), cones_yellow_count_(0), map_ready_(false), loop_closure_(false)
{
}

void SlamSI::mapCallback(const fsd_common_msgs::Map::ConstPtr &msg)
{
  if(!map_ready_)
  {
    map_ready_ = true;
  }

  map_.cones.reserve(msg->cone_blue.size() + msg->cone_yellow.size() + msg->cone_orange.size());

  cones_blue_ = msg->cone_blue;
  cones_yellow_ = msg->cone_yellow;

  for(const auto &i : msg->cone_orange)
  {
    addToMap(i);
  }

  if(loop_closure_)
  {
    for(const auto &i : msg->cone_blue)
    {
      addToMap(i);
    }

    for(const auto &i : msg->cone_yellow)
    {
      addToMap(i);
    }
  }
    
}

void SlamSI::stateCallback(const fsd_common_msgs::CarState::ConstPtr &msg)
{
  car_pose_.position.x = msg->car_state.x;
  car_pose_.position.y = msg->car_state.y;
  car_pose_.yaw = msg->car_state.theta;
  
  pose_pub_.publish(car_pose_);

  if(!loop_closure_)
  {
    actualizeMap();
  }
}

void SlamSI::addToMap(const fsd_common_msgs::Cone &coneMsg)
{
  sgtdv_msgs::Cone cone;
  cone.coords.x = coneMsg.position.x;
  cone.coords.y = coneMsg.position.y;

  if(coneMsg.color.data == "blue")
  {
    cone.color = 'b';
  } else if(coneMsg.color.data == "yellow")
  {
    cone.color = 'y';
  } else if(coneMsg.color.data == "orange")
  {
    cone.color = 'g';
  }

  map_.cones.emplace_back(cone);
}

int SlamSI::findLookAheadConeIdx(std::vector<fsd_common_msgs::Cone> cones) const
{
  const auto closest_it  = std::min_element(cones.begin(), cones.end(),
                                          [&](const fsd_common_msgs::Cone &a,
                                          const fsd_common_msgs::Cone &b) {
                                          const double da = std::hypot(car_pose_.position.x - a.position.x,
                                                                      car_pose_.position.y - a.position.y);
                                          const double db = std::hypot(car_pose_.position.x - b.position.x,
                                                                      car_pose_.position.y - b.position.y);

                                          return da < db;
                                          });
  const auto closest_idx =  std::distance(cones.begin(), closest_it);

  const auto size = cones.size();
  static int offset;
  static int next_idx;
  double distance;
    
  offset = 0;
  do
  {
    next_idx = (closest_idx + offset++);
    if(next_idx > size - 1) break;

    distance = std::sqrt(std::pow(cones[next_idx].position.x - car_pose_.position.x,2) 
            + std::pow(cones[next_idx].position.y - car_pose_.position.y,2));
  
  } while(distance < LOOK_AHEAD_DISTANCE);
  return next_idx;
}

void SlamSI::actualizeMap()
{
  const int cone_blue_lookahead_idx = findLookAheadConeIdx(cones_blue_);
  const int cone_yellow_lookahead_idx = findLookAheadConeIdx(cones_yellow_);

  while(cones_blue_count_ < cone_blue_lookahead_idx && cones_blue_count_ < cones_blue_.size())
  {
    addToMap(cones_blue_[cones_blue_count_++]);
  }

  while(cones_yellow_count_ < cone_yellow_lookahead_idx && cones_yellow_count_ < cones_yellow_.size())
  {
    addToMap(cones_yellow_[cones_yellow_count_++]);
  }

  if(cones_blue_count_ == cones_blue_.size() && cones_yellow_count_ == cones_yellow_.size())
  {
    loopClosure();
    return;
  }
}

void SlamSI::loopClosure()
{
  loop_closure_ = true;
  loop_close_pub_.publish(std_msgs::Empty());
}

void SlamSI::publishMap() const
{
  if(map_ready_)
    map_pub_.publish(map_);
}