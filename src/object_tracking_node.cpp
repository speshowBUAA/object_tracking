// headers in STL
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
// headers in 3rd-part
#include "object_tracking.h"
#include <gtest/gtest.h>
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_tracking");
  ros::NodeHandle nh("~");
  Tracking t(nh);
  t.createROSPubSub();
  ros::spin();
  return 1;
}