/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/path_tracking_sim_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pathTrackingSimInterface");
  ros::NodeHandle handle;

  PathTrackingSimInterface sim_interface(handle);

  ros::spin();

  return 0;
}