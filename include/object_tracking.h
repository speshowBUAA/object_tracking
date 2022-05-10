/**
* @file multihead_ros.h
* @brief ROS interface for tracking_object
* @author Xiubo Ye
* @date 2021/12/23
*/

#pragma once
#include <glog/logging.h>
#include <eigen_conversions/eigen_msg.h>
// headers in ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include "object_tracker/object_tracker.h"
#include "autoware_msgs/DetectedObjectArray.h"

class Tracking
{
private:
    float velocity_limit_;
    // initializer list
    ros::NodeHandle private_nh_;
    // end initializer list
    std::vector<ObjectTracker*> ot_;
    std::vector<std::string> trk_cls_;
    ros::NodeHandle nh_;
    ros::Subscriber det_objects_;
    ros::Publisher pub_track_objects_;
    ros::Publisher pub_predict_objects_;
    std::vector<std::map<long, autoware_msgs::DetectedObject>> detect_track_info_; // x_size, y_size, z_size, yaw
    std::string cloud_track_frame_id_;
    std::string cloud_base_frame_id_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;

    void trackCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &msg);
    bool IsObjectValid(const autoware_msgs::DetectedObject &in_object);
    bool lookupTransform(const std::string& target_frame, const std::string& source_frame,
                        const ros::Time& time, const ros::Duration timeout, Eigen::Isometry3d& transform);
public:
    Tracking(const ros::NodeHandle& n);
    void createROSPubSub();
};