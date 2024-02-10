/*****************************************************/
/* Organization: Stuba Green Team
/* Authors: Patrik Knaperek
/*****************************************************/

#include "../include/cone_detection_si.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cone_detection_si");
  ros::NodeHandle handle;
  
  ConeDetectionSI cone_detection_SI(handle);
  
  ros::spin();

  return 0;
}
