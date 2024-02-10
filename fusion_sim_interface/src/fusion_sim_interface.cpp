/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/fusion_sim_interface.h"

FusionSimInterface::FusionSimInterface()
{
    
}

FusionSimInterface::~FusionSimInterface()
{

}

void FusionSimInterface::lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{   
#ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.stamp = ros::Time::now();
    state.working_state = 1;
    lidar_vis_debug_pub_.publish(state);
#endif    
    
    int points_count = msg->width;
    sgtdv_msgs::Point2DStampedArrPtr lidar_cones_msg(new sgtdv_msgs::Point2DStampedArr);
    lidar_cones_msg->points.reserve(points_count);

    float const *temp;

    for(int i = 0; i < points_count; i++)
    {
        temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
        sgtdv_msgs::Point2DStamped cone = sgtdv_msgs::Point2DStamped();

        cone.header = msg->header;
        cone.x = *temp;
        cone.y = *(temp + 1);

        lidar_cones_msg->points.push_back(cone);
    }

    lidar_pub_.publish(lidar_cones_msg);

#ifdef SGT_DEBUG_STATE
    state.stamp = ros::Time::now();
    state.num_of_cones = lidar_cones_msg->points.size();
    state.working_state = 0;
	lidar_vis_debug_pub_.publish(state);
#endif
}

void FusionSimInterface::cameraCallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
#ifdef SGT_DEBUG_STATE
    sgtdv_msgs::DebugState state;
    state.stamp = ros::Time::now();
    state.working_state = 1;
    camera_vis_debug_pub_.publish(state);
#endif
    
    int cones_count = msg->width;
    sgtdv_msgs::ConeStampedArrPtr camera_cones_msg(new sgtdv_msgs::ConeStampedArr);
    camera_cones_msg->cones.reserve(cones_count);
    
    float const *temp;

    for(int i = 0; i < cones_count; i++)
    {
        temp = reinterpret_cast<const float*>(&msg->data[i*msg->point_step]);
        sgtdv_msgs::ConeStamped cone;

        cone.coords.header = msg->header;
        cone.coords.x = *temp;
        cone.coords.y = *(temp + 1);

        if(*(temp + 9) > 0.85)
            cone.color = 'b';       // blue cone
        else if(*(temp + 10) > 0.85)
            cone.color = 'y';       // yellow cone
        else if(*(temp + 11) > 0.85)
            cone.color = 'g';       // orange cone big
        else cone.color = 'u';      // unknown color

        camera_cones_msg->cones.push_back(cone);
    }
    
    camera_pub_.publish(camera_cones_msg);

#ifdef SGT_DEBUG_STATE
    state.stamp = ros::Time::now();
    state.num_of_cones = camera_cones_msg->cones.size();
    state.working_state = 0;
	camera_vis_debug_pub_.publish(state);
#endif
}
