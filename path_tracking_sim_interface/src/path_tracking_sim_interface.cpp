/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/path_tracking_sim_interface.h"

PathTrackingSimInterface::PathTrackingSimInterface(ros::NodeHandle& nh) :
  /* ROS interface init */
  cmd_pub_(nh.advertise<fsd_common_msgs::ControlCommand>("sgt/control_command", 1)), // this topic name has to be set in fssim_interface/config/config.yaml as parameter fsd/cmd to obtain SGT control commands by FSSIM
  trajectory_pub_(nh.advertise<sgtdv_msgs::Point2DArr>("pathplanning_trajectory", 1, true)),
  pose_pub_(nh.advertise<sgtdv_msgs::CarPose>("pose_estimate", 1)),
  velocity_pub_(nh.advertise<sgtdv_msgs::CarVel>("velocity_estimate", 1)),
  
  cmd_sub_(nh.subscribe("pathtracking_commands", 1, &PathTrackingSimInterface::cmdCallback, this)),
  trajectory_sub_(nh.subscribe("control/pure_pursuit/center_line", 1, &PathTrackingSimInterface::trajectoryCallback, this)),
  pose_sub_(nh.subscribe("estimation/slam/state", 1, &PathTrackingSimInterface::poseCallback, this)),
  velocity_sub_(nh.subscribe("estimation/velocity_estimation/velocity_estimation", 1, &PathTrackingSimInterface::velocityCallback, this))
{
}

void PathTrackingSimInterface::cmdCallback(const sgtdv_msgs::Control::ConstPtr &msg) const
{
  fsd_common_msgs::ControlCommandPtr cmd(new fsd_common_msgs::ControlCommand);

  cmd->header.stamp = ros::Time::now();
  cmd->steering_angle.data = msg->steering_angle;
  cmd->throttle.data = msg->speed / 100.f;

  cmd_pub_.publish(cmd);
}
void PathTrackingSimInterface::trajectoryCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg) const
{
  sgtdv_msgs::Point2DArrPtr trajectory_msg(new sgtdv_msgs::Point2DArr);
  trajectory_msg->points.reserve(msg->polygon.points.size());

  for(auto msg_point : msg->polygon.points)
  {
    sgtdv_msgs::Point2D point;
    //point.header = msg->header;
    point.x = msg_point.x;
    point.y = msg_point.y;

    trajectory_msg->points.push_back(point);
  }

  trajectory_pub_.publish(trajectory_msg);
}
void PathTrackingSimInterface::poseCallback(const fsd_common_msgs::CarState::ConstPtr &msg) const
{
  sgtdv_msgs::CarPosePtr car_pose_msg(new sgtdv_msgs::CarPose);

  //carPose->position.header = msg->header;
  car_pose_msg->position.x = msg->car_state.x;
  car_pose_msg->position.y = msg->car_state.y;
  car_pose_msg->yaw = msg->car_state.theta;

  pose_pub_.publish(car_pose_msg);
}

void PathTrackingSimInterface::velocityCallback(const fsd_common_msgs::CarStateDt::ConstPtr &msg) const
{
  sgtdv_msgs::CarVelPtr car_vel_msg(new sgtdv_msgs::CarVel);

  car_vel_msg->speed = msg->car_state_dt.x;
  car_vel_msg->yaw_rate = msg->car_state_dt.theta;

  velocity_pub_.publish(car_vel_msg);
}