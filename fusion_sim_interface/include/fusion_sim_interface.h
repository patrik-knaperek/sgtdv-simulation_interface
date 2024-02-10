/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

/* ROS */
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/* SGT */
#include <sgtdv_msgs/ConeStampedArr.h>
#include <sgtdv_msgs/Point2DStampedArr.h>
#include "../../../SGT_Macros.h"
#include <sgtdv_msgs/DebugState.h>

class FusionSimInterface
{
public:
  FusionSimInterface(ros::NodeHandle& nh);
  ~FusionSimInterface() = default;

  void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) const;
  void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) const;
  
private:
  ros::Publisher lidar_pub_;
  ros::Publisher camera_pub_;

  ros::Subscriber lidar_sub_;
  ros::Subscriber camera_sub_;

#ifdef SGT_DEBUG_STATE
  ros::Publisher lidar_vis_debug_pub_;
  ros::Publisher camera_vis_debug_pub_;
#endif    
};
