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
        FusionSimInterface();
        ~FusionSimInterface();

        // Setters
        void setPublishers(ros::Publisher cameraPub, ros::Publisher lidarPub)
        {
            this->camera_pub_ = cameraPub;
            this->lidar_pub_ = lidarPub;
        };

    #ifdef SGT_DEBUG_STATE
        void SetVisDebugPublishers(ros::Publisher lidarVisDebugPublisher, ros::Publisher cameraVisDebugPublisher)
        { 
            lidar_vis_debug_pub_ = lidarVisDebugPublisher;
            camera_vis_debug_pub_ = cameraVisDebugPublisher;
        };
    #endif
        
        void lidarCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void cameraCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);
        
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
