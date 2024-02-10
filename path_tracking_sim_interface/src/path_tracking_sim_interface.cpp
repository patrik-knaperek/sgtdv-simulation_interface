/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/path_tracking_sim_interface.h"

PathTrackingSimInterface::PathTrackingSimInterface()
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