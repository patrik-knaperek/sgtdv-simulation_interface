/*****************************************************/
//Organization: Stuba Green Team
//Authors: Patrik Knaperek
/*****************************************************/

#include "../include/PathTrackingSimInterface.h"

PathTrackingSimInterface::PathTrackingSimInterface()
{

}

PathTrackingSimInterface::~PathTrackingSimInterface()
{

}

void PathTrackingSimInterface::DoCmd(const sgtdv_msgs::Control::ConstPtr &msg)
{
    fsd_common_msgs::ControlCommandPtr cmd(new fsd_common_msgs::ControlCommand);

    cmd->header.stamp = ros::Time::now();
    cmd->steering_angle.data = msg->steeringAngle;
    cmd->throttle.data = msg->speed;

    m_cmdPublisher.publish(cmd);
}
void PathTrackingSimInterface::DoTrajectory(const geometry_msgs::PolygonStamped::ConstPtr &msg)
{
    sgtdv_msgs::Point2DArrPtr trajectory(new sgtdv_msgs::Point2DArr);
    trajectory->points.reserve(msg->polygon.points.size());

    for(auto msgPoint : msg->polygon.points)
    {
        sgtdv_msgs::Point2D point;
        point.header = msg->header;
        point.x = msgPoint.x;
        point.y = msgPoint.y;

        trajectory->points.push_back(point);
    }

    m_trajectoryPublisher.publish(trajectory);
}
void PathTrackingSimInterface::DoPose(const fsd_common_msgs::CarState::ConstPtr &msg)
{
    sgtdv_msgs::CarPosePtr carPose(new sgtdv_msgs::CarPose);

    carPose->position.header = msg->header;
    carPose->position.x = msg->car_state.x;
    carPose->position.y = msg->car_state.y;
    carPose->yaw = msg->car_state.theta;

    m_posePublisher.publish(carPose);
}