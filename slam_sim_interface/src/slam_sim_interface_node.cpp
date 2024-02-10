/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/slam_sim_interface.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slamSimInterface");
  ros::NodeHandle handle;

  SLAMSimInterface sim_interface;

  ros::Publisher mapPub = handle.advertise<sgtdv_msgs::ConeArr>("slam/map", 1, true);
  ros::Publisher posePub = handle.advertise<sgtdv_msgs::CarPose>("slam/pose", 1);
  ros::Publisher loopClosePub = handle.advertise<std_msgs::Empty>("slam/loop_closure", 1, true);
  //ros::Publisher velPub = handle.advertise<sgtdv_msgs::CarVel>("velocity_estimate", 1);
  sim_interface.setPublishers(mapPub, posePub, loopClosePub/*, velPub*/);

  ros::Subscriber mapSub = handle.subscribe("estimation/slam/map", 1, &SLAMSimInterface::mapCallback, &sim_interface);
  ros::Subscriber poseSub = handle.subscribe("estimation/slam/state", 1, &SLAMSimInterface::stateCallback, &sim_interface);
  
  ros::Rate rate(5);
  while(ros::ok())
  {
    ros::spinOnce();
    sim_interface.publishMap();
    rate.sleep();
  }

  return 0;
}