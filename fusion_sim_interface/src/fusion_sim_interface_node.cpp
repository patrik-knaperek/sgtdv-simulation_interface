/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/fusion_sim_interface.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fusionSimInterface");
    ros::NodeHandle handle;
    
    FusionSimInterface sim_interface;

    ros::Publisher lidarPub = handle.advertise<sgtdv_msgs::Point2DStampedArr>("lidar_cones", 1);
    ros::Publisher cameraPub = handle.advertise<sgtdv_msgs::ConeStampedArr>("camera_cones", 1);
    sim_interface.setPublishers(cameraPub, lidarPub);

#ifdef SGT_DEBUG_STATE
    ros::Publisher lidarDebugStatePublisher = handle.advertise<sgtdv_msgs::DebugState>("lidar_cone_detection_debug_state", 1);
    ros::Publisher cameraDebugStatePublisher = handle.advertise<sgtdv_msgs::DebugState>("camera_cone_detection_debug_state", 1);
    sim_interface.SetVisDebugPublishers(lidarDebugStatePublisher, cameraDebugStatePublisher);
#endif

    ros::Subscriber cameraSub = handle.subscribe("fssim/camera/cones", 1, &FusionSimInterface::cameraCallback, &sim_interface);
    ros::Subscriber lidarSub =  handle.subscribe("fssim/lidar/cones", 1, &FusionSimInterface::lidarCallback, &sim_interface);

    ros::spin();

    return 0;
}
