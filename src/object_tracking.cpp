// headers in STL
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

// headers in local files
#include "object_tracking.h"

Tracking::Tracking(const ros::NodeHandle& n)
{
  n.getParam("/tracking_classes", trk_cls_);
  for (size_t cls = 0 ; cls < trk_cls_.size(); cls++)
  {
    ObjectTracker *ot = new ObjectTracker(n);
    ot_.push_back(ot);
    std::map<long, autoware_msgs::DetectedObject> detect_track_info;
    detect_track_info_.push_back(detect_track_info);
  }
}

void Tracking::createROSPubSub()
{
  det_objects_ = nh_.subscribe<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1, &Tracking::trackCallback, this);
  pub_track_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/tracking_objects", 1);
}

void Tracking::trackCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &detect_objects)
{
  autoware_msgs::DetectedObjectArray pub_objects;
  pub_objects.header = detect_objects->header;
  for (size_t cls = 0 ; cls < trk_cls_.size(); cls++)
  {
    vector<autoware_msgs::DetectedObject> detect_objs;
    for (auto const &object: detect_objects->objects)
    {
      if (IsObjectValid(object) && object.label == trk_cls_[cls])
      {
        detect_objs.push_back(object);
        // ROS_INFO("object_center %.2f %.2f %.2f rotation %.2f %.2f %.2f %.2f...", object.pose.position.x, object.pose.position.y, object.pose.position.z, object.pose.orientation.x, object.pose.orientation.y, object.pose.orientation.z, object.pose.orientation.w);
      }
    }

    std::chrono::time_point<std::chrono::system_clock> t0 = std::chrono::system_clock::now();
    map<long, vector<geometry_msgs::Pose>> tracks;
    map<int, int> assignments_detect_track;
    ot_[cls]->detect(detect_objs, detect_objects->header.stamp.toSec(), cls, tracks,
          assignments_detect_track); // assignment < observation, target >
    LOG(INFO) << "detect_objs.size(): " << detect_objs.size() << ", tracks.size(): " << tracks.size() << ", assignments_detect_track.size(): " << assignments_detect_track.size();
    std::chrono::time_point<std::chrono::system_clock> t1 = std::chrono::system_clock::now();
    LOG(INFO) << "ot_->detect: " << std::chrono::duration<double>(t1 - t0).count() * 1000 << " ms.";

    for (size_t i = 0; i < detect_objs.size(); i++)
    {
      auto const &object = detect_objs[i];
      auto it = assignments_detect_track.find(i);
      if (it != assignments_detect_track.end())
      {
        detect_track_info_[cls][it->second] = object;
      }
    }


    std::map<long, autoware_msgs::DetectedObject> detect_track_info;
    
    for (map<long, vector<geometry_msgs::Pose>>::const_iterator it =
          tracks.begin();
        it != tracks.end(); ++it)
    {
      const autoware_msgs::DetectedObject& track_info = detect_track_info_[cls][it->first];
      autoware_msgs::DetectedObject obj;
      obj.header = detect_objects->header;
      obj.valid = true;
      obj.pose_reliable = true;

      obj.id = it->first;
      obj.pose.position = it->second[0].position;
      
      obj.velocity.linear.x = it->second[1].position.x;
      obj.velocity.linear.y = it->second[1].position.y;
      obj.velocity.linear.z = it->second[1].position.z;
      obj.dimensions= track_info.dimensions;
      obj.pose.position.z = track_info.pose.position.z;
      obj.pose.orientation = it->second[0].orientation;
      obj.score = track_info.score;
      obj.label = track_info.label;
      pub_objects.objects.push_back(obj);
      // ROS_INFO("object id %d center %.2f %.2f %.2f rotation %.2f %.2f %.2f %.2f...", obj.id, obj.pose.position.x, obj.pose.position.y, obj.pose.position.z, obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w);
      detect_track_info[it->first] = track_info;
    }
    detect_track_info_[cls] = detect_track_info;
  }
  pub_track_objects_.publish(pub_objects);
}

bool Tracking::IsObjectValid(const autoware_msgs::DetectedObject &in_object)
{
  if (!in_object.valid ||
      std::isnan(in_object.pose.orientation.x) ||
      std::isnan(in_object.pose.orientation.y) ||
      std::isnan(in_object.pose.orientation.z) ||
      std::isnan(in_object.pose.orientation.w) ||
      std::isnan(in_object.pose.position.x) ||
      std::isnan(in_object.pose.position.y) ||
      std::isnan(in_object.pose.position.z) ||
      (in_object.pose.position.x == 0.) ||
      (in_object.pose.position.y == 0.) ||
      (in_object.dimensions.x <= 0.) ||
      (in_object.dimensions.y <= 0.) ||
      (in_object.dimensions.z <= 0.)
    )
  {
    return false;
  }
  return true;
}//end IsObjectValid