/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/cone_detection_si.h"

ConeDetectionSI::ConeDetectionSI(ros::NodeHandle& nh) :
  /* ROS interface init */
  camera_pub_(nh.advertise<sgtdv_msgs::ConeStampedArr>("camera/cones", 1)),
  lidar_pub_(nh.advertise<sgtdv_msgs::Point2DStampedArr>("lidar/cones", 1)),
  
  camera_sub_(nh.subscribe("fssim/camera/cones", 1, &ConeDetectionSI::cameraCallback, this)),
  lidar_sub_(nh.subscribe("fssim/lidar/cones", 1, &ConeDetectionSI::lidarCallback, this))

#ifdef SGT_DEBUG_STATE
  , lidar_vis_debug_pub_(nh.advertise<sgtdv_msgs::DebugState>("lidar/debug_state", 2))
  , camera_vis_debug_pub_(nh.advertise<sgtdv_msgs::DebugState>("camera/debug_state", 2))
#endif
{   
}

void ConeDetectionSI::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) const
{   
#ifdef SGT_DEBUG_STATE
  sgtdv_msgs::DebugState state;
  state.stamp = ros::Time::now();
  state.working_state = 1;
  lidar_vis_debug_pub_.publish(state);
#endif    
    
  const int points_count = msg->width;
  sgtdv_msgs::Point2DStampedArr lidar_cones_msg;
  lidar_cones_msg.points.reserve(points_count);

  float const *temp;

  sgtdv_msgs::Point2DStamped cone;
  cone.header = msg->header;

  for(int i = 0; i < points_count; i++)
  {
    temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);

    cone.x = *temp;
    cone.y = *(temp + 1);

    lidar_cones_msg.points.emplace_back(cone);
  }

  lidar_pub_.publish(lidar_cones_msg);

#ifdef SGT_DEBUG_STATE
  state.stamp = ros::Time::now();
  state.num_of_cones = lidar_cones_msg.points.size();
  state.working_state = 0;
	lidar_vis_debug_pub_.publish(state);
#endif
}

void ConeDetectionSI::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) const
{
#ifdef SGT_DEBUG_STATE
  sgtdv_msgs::DebugState state;
  state.stamp = ros::Time::now();
  state.working_state = 1;
  camera_vis_debug_pub_.publish(state);
#endif
    
  const int cones_count = msg->width;
  sgtdv_msgs::ConeStampedArr camera_cones_msg;
  camera_cones_msg.cones.reserve(cones_count);
  
  float const *temp;
  sgtdv_msgs::ConeStamped cone;
  cone.coords.header = msg->header;
  
  for(int i = 0; i < cones_count; i++)
  {
    temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
    
    cone.coords.x = *temp;
    cone.coords.y = *(temp + 1);

    if(*(temp + 9) > 0.85)
      cone.color = 'b';           // blue cone
    else if(*(temp + 10) > 0.85)
      cone.color = 'y';           // yellow cone
    else if(*(temp + 11) > 0.85)
      cone.color = 'g';           // orange cone big
    else cone.color = 'u';        // unknown color

    camera_cones_msg.cones.emplace_back(cone);
  }
  
  camera_pub_.publish(camera_cones_msg);

#ifdef SGT_DEBUG_STATE
  state.stamp = ros::Time::now();
  state.num_of_cones = camera_cones_msg.cones.size();
  state.working_state = 0;
  camera_vis_debug_pub_.publish(state);
#endif
}
