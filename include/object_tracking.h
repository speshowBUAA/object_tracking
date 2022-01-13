/**
* @file multihead_ros.h
* @brief ROS interface for tracking_object
* @author Xiubo Ye
* @date 2021/12/23
*/

#pragma once
#include <glog/logging.h>
// headers in ROS
#include <ros/ros.h>
#include "object_tracker/object_tracker.h"
#include "autoware_msgs/DetectedObjectArray.h"

class Tracking
{
private:

    // initializer list
    ros::NodeHandle private_nh_;
    // end initializer list
    ObjectTracker* ot_;
    std::vector<std::string> trk_cls_;
    ros::NodeHandle nh_;
    ros::Subscriber det_objects_;
    ros::Publisher pub_track_objects_;
    std::vector<std::map<long, autoware_msgs::DetectedObject>> detect_track_info_; // x_size, y_size, z_size, yaw

    void trackCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &msg);
    bool IsObjectValid(const autoware_msgs::DetectedObject &in_object);

public:
    Tracking(const ros::NodeHandle& n);
    void createROSPubSub();
};