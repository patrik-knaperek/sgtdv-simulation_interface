/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/fusion_sim_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fusionSimInterface");
  ros::NodeHandle handle;
  
  FusionSimInterface sim_interface(handle);
  
  ros::spin();

  return 0;
}
