/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/slam_si.h"

constexpr int FPS = 5;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_si");
  ros::NodeHandle handle;

  SlamSI slam_SI(handle);

  ros::Rate rate(FPS);
  while(ros::ok())
  {
    ros::spinOnce();
    slam_SI.publishMap();
    rate.sleep();
  }

  return 0;
}