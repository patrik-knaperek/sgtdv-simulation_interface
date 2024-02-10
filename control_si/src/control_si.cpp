/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/control_si.h"

ControlSI::ControlSI(ros::NodeHandle& nh) :
  /* ROS interface init */
  cmd_pub_(nh.advertise<fsd_common_msgs::ControlCommand>("sgt/control_command", 1)), // this topic name has to be set in fssim_interface/config/config.yaml as parameter fsd/cmd to obtain SGT control commands by FSSIM
  trajectory_pub_(nh.advertise<sgtdv_msgs::Point2DArr>("pathplanning_trajectory", 1, true)),
  pose_pub_(nh.advertise<sgtdv_msgs::CarPose>("pose_estimate", 1)),
  velocity_pub_(nh.advertise<sgtdv_msgs::CarVel>("velocity_estimate", 1)),
  
  cmd_sub_(nh.subscribe("pathtracking_commands", 1, &ControlSI::cmdCallback, this)),
  trajectory_sub_(nh.subscribe("control/pure_pursuit/center_line", 1, &ControlSI::trajectoryCallback, this)),
  pose_sub_(nh.subscribe("estimation/slam/state", 1, &ControlSI::poseCallback, this)),
  velocity_sub_(nh.subscribe("estimation/velocity_estimation/velocity_estimation", 1, &ControlSI::velocityCallback, this))
{
}

void ControlSI::cmdCallback(const sgtdv_msgs::Control::ConstPtr &msg) const
{
  fsd_common_msgs::ControlCommand cmd_msg;

  cmd_msg.header.stamp = msg->stamp;
  cmd_msg.steering_angle.data = msg->steering_angle;
  cmd_msg.throttle.data = msg->speed / 100.f;

  cmd_pub_.publish(cmd_msg);
}
void ControlSI::trajectoryCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg) const
{
  sgtdv_msgs::Point2DArr trajectory_msg;
  trajectory_msg.points.reserve(msg->polygon.points.size());

  sgtdv_msgs::Point2D point;
  for(const auto& msg_point : msg->polygon.points)
  {
    point.x = msg_point.x;
    point.y = msg_point.y;

    trajectory_msg.points.push_back(point);
  }

  trajectory_pub_.publish(trajectory_msg);
}
void ControlSI::poseCallback(const fsd_common_msgs::CarState::ConstPtr &msg) const
{
  sgtdv_msgs::CarPose car_pose_msg;

  car_pose_msg.position.x = msg->car_state.x;
  car_pose_msg.position.y = msg->car_state.y;
  car_pose_msg.yaw = msg->car_state.theta;

  pose_pub_.publish(car_pose_msg);
}

void ControlSI::velocityCallback(const fsd_common_msgs::CarStateDt::ConstPtr &msg) const
{
  sgtdv_msgs::CarVel car_vel_msg;

  car_vel_msg.speed = msg->car_state_dt.x;
  car_vel_msg.yaw_rate = msg->car_state_dt.theta;

  velocity_pub_.publish(car_vel_msg);
}