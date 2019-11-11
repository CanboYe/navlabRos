#include "ros/ros.h"
#include "std_msgs/String.h"
//#include <stdio.h>

#include <iostream>
#include <fstream>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <sstream>
#include <vector>
#include <math.h>
#include "navlab.h"
#include "utils.h"

#define PI 3.14159265

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener2");
  ros::NodeHandle nodeHandle("~");


  Navlab myNavlab(nodeHandle);
  
  myNavlab.runROS();

  return 0;
}