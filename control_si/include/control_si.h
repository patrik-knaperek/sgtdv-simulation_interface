/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

/* ROS */
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>

/* AMZ FSSIM */
#include <fsd_common_msgs/ControlCommand.h>
#include <fsd_common_msgs/CarState.h>
#include <fsd_common_msgs/CarStateDt.h>

/* SGT-DV */
#include <sgtdv_msgs/Control.h>
#include <sgtdv_msgs/CarPose.h>
#include <sgtdv_msgs/CarVel.h>
#include <sgtdv_msgs/Point2DArr.h>

class ControlSI
{
public:
  ControlSI(ros::NodeHandle& nh);
  ~ControlSI() = default;

  /* Callbacks */
  void cmdCallback(const sgtdv_msgs::Control::ConstPtr &msg) const;
  void trajectoryCallback(const geometry_msgs::PolygonStamped::ConstPtr &msg) const;
  void poseCallback(const fsd_common_msgs::CarState::ConstPtr &msg) const;
  void velocityCallback(const fsd_common_msgs::CarStateDt::ConstPtr &msg) const;

private:
  /* SGT-DV --> FSSIM */
  ros::Publisher cmd_pub_;

  /* FSSIM --> SGT-DV */
  ros::Publisher trajectory_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber velocity_sub_;
};