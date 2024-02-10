/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/control_si.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_si");
  ros::NodeHandle handle;

  ControlSI control_SI(handle);

  ros::spin();

  return 0;
}