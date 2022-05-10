// headers in STL
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
// headers in 3rd-part
#include "../include/object_tracker/object_tracker.h"
#include <gtest/gtest.h>
#include <glog/logging.h>
using namespace std;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_tracking");
  ros::NodeHandle nh("~");
  std::string filter;
  LOG(INFO) << "tracking node started.";
  
  ObjectTracker *ot_ = new ObjectTracker(nh);
  map<long, vector<geometry_msgs::Pose>> tracks;
  map<long, vector<geometry_msgs::Pose>> predicts;
  map<int, int> assignments_cluster_track;
  ros::Time timestamp = ros::Time::now();
  vector<autoware_msgs::DetectedObject> clusters_center(10);
  for (size_t i = 0; i < clusters_center.size(); i++)
  {
    autoware_msgs::DetectedObject& cluster_center = clusters_center[i];
    cluster_center.pose.position.x = 0.0f;
    cluster_center.pose.position.y = 0.0f;
    cluster_center.pose.position.z = 0.0f;
  }
  ot_->detect(clusters_center, timestamp.toSec(), 0, tracks, predicts,
             assignments_cluster_track);
  return 0;
}