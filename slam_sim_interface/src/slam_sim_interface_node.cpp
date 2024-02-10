/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/slam_sim_interface.h"

constexpr int FPS = 5;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slamSimInterface");
  ros::NodeHandle handle;

  SLAMSimInterface sim_interface(handle);

  ros::Rate rate(FPS);
  while(ros::ok())
  {
    ros::spinOnce();
    sim_interface.publishMap();
    rate.sleep();
  }

  return 0;
}