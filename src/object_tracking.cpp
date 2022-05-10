// headers in STL
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>

// headers in local files
#include "object_tracking.h"

Tracking::Tracking(const ros::NodeHandle& n) : nh_(n), tfBuffer_(ros::Duration(60.)), tfListener_(tfBuffer_)
{
  n.getParam("/tracking_classes", trk_cls_);
  for (size_t cls = 0 ; cls < trk_cls_.size(); cls++)
  {
    ObjectTracker* ot = new ObjectTracker(n);
    std::map<long, autoware_msgs::DetectedObject> detect_track_info;
    detect_track_info_.push_back(detect_track_info);
    ot_.push_back(ot);
  }

  n.getParam("/velocity_limit", velocity_limit_);
  ROS_INFO("velocity_limit: %f ", velocity_limit_);
}

void Tracking::createROSPubSub()
{
  det_objects_ = nh_.subscribe<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", 1, &Tracking::trackCallback, this);
  pub_track_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/tracking_objects_sim", 1);
  pub_predict_objects_ = nh_.advertise<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/predict_objects_sim", 1);
}

void Tracking::trackCallback(const autoware_msgs::DetectedObjectArray::ConstPtr &detect_objects)
{
  cloud_track_frame_id_ = "odom";
  cloud_base_frame_id_ = "base_footprint";
  // lookup tf for sensor2odom: odom need to be gravity aligned
  Eigen::Isometry3d base_to_odom = Eigen::Isometry3d::Identity();
  if (!lookupTransform(cloud_track_frame_id_, cloud_base_frame_id_,
                       detect_objects->header.stamp, ros::Duration(0.1), base_to_odom))
    LOG(WARNING) << "Look TF failed, base_footprint_to_odom, RETURN!";
  Eigen::Matrix3d rotation_matrix = base_to_odom.rotation();
  Eigen::Quaterniond base_to_odom_q(rotation_matrix);

  autoware_msgs::DetectedObjectArray pub_track_objects;
  autoware_msgs::DetectedObjectArray pub_predict_objects;
  pub_track_objects.header = detect_objects->header;
  pub_predict_objects.header = detect_objects->header;

  for (size_t cls = 0 ; cls < trk_cls_.size(); cls++)
  {
    vector<autoware_msgs::DetectedObject> detect_objs;
    for (auto const &object: detect_objects->objects)
    {
      if (IsObjectValid(object) && object.label == trk_cls_[cls])
      {
        autoware_msgs::DetectedObject obj = object;
        // geometry_msgs::PoseStamped obj_centroid;
        // geometry_msgs::PoseStamped obj_centroid_odom;
        // obj_centroid.header = detect_objects->header;
        // obj_centroid.pose = obj.pose;
        Eigen::Vector4d vec_obj_centroid;
        vec_obj_centroid(0) = object.pose.position.x;
        vec_obj_centroid(1) = object.pose.position.y;
        vec_obj_centroid(2) = object.pose.position.z;
        vec_obj_centroid(3) = 1;
        
        Eigen::Vector4d centroid_in_odom = base_to_odom * vec_obj_centroid;
        Eigen::Quaterniond vec_obj_q(object.pose.orientation.w, object.pose.orientation.x,
                            object.pose.orientation.y, object.pose.orientation.z);
        Eigen::Quaterniond centroid_in_odom_q = base_to_odom_q * vec_obj_q;

        // try{
        //     obj_centroid_odom = tfBuffer_.transform(obj_centroid, cloud_track_frame_id_);
        //    }
        // catch (tf2::TransformException& ex) {
        //       ROS_ERROR("%s",ex.what());
        //       ros::Duration(0.1).sleep();
        // }
        // ROS_INFO("before:(%.2f,%.2f,%.2f), q:(%.2f,%.2f,%.2f,%.2f) frame_id:%s",obj_centroid.pose.position.x,obj_centroid.pose.position.y,obj_centroid.pose.position.z,object.pose.orientation.x, object.pose.orientation.y, object.pose.orientation.z, object.pose.orientation.w, obj_centroid.header.frame_id.c_str());
        // ROS_INFO("before eigen:(%.2f,%.2f,%.2f), q:(%.2f,%.2f,%.2f,%.2f) frame_id:%s", vec_obj_centroid(0),vec_obj_centroid(1),vec_obj_centroid(2),vec_obj_q.vec()[0],vec_obj_q.vec()[1],vec_obj_q.vec()[2],vec_obj_q.vec()[3],detect_objects->header.frame_id.c_str());
        // ROS_INFO("after:(%.2f,%.2f,%.2f), q:(%.2f,%.2f,%.2f,%.2f) frame_id:%s",obj_centroid_odom.pose.position.x,obj_centroid_odom.pose.position.y,obj_centroid_odom.pose.position.z,obj_centroid_odom.pose.orientation.x,obj_centroid_odom.pose.orientation.y,obj_centroid_odom.pose.orientation.z,obj_centroid_odom.pose.orientation.w,obj_centroid_odom.header.frame_id.c_str());
        // ROS_INFO("after eigen:(%.2f,%.2f,%.2f), q:(%.2f,%.2f,%.2f,%.2f) frame_id:%s",centroid_in_odom(0),centroid_in_odom(1),centroid_in_odom(2),centroid_in_odom_q.vec()[0],centroid_in_odom_q.vec()[1],centroid_in_odom_q.vec()[2],centroid_in_odom_q.vec()[3],cloud_track_frame_id_.c_str());
      
        // obj.pose = obj_centroid_odom.pose;
        obj.pose.position.x = centroid_in_odom(0);
        obj.pose.position.y = centroid_in_odom(1);
        obj.pose.position.z = centroid_in_odom(2);
        obj.pose.orientation.x = centroid_in_odom_q.vec()[0];
        obj.pose.orientation.y = centroid_in_odom_q.vec()[1];
        obj.pose.orientation.z = centroid_in_odom_q.vec()[2];
        obj.pose.orientation.w = centroid_in_odom_q.vec()[3];
        obj.header.frame_id = cloud_track_frame_id_;
        // obj.pose.position.x = vec_obj_centroid(0);
        // obj.pose.position.y = vec_obj_centroid(1);
        // obj.pose.position.z = vec_obj_centroid(2);
        // obj.pose.orientation.x = vec_obj_q.vec()[0];
        // obj.pose.orientation.y = vec_obj_q.vec()[1];
        // obj.pose.orientation.z = vec_obj_q.vec()[2];
        // obj.pose.orientation.w = vec_obj_q.vec()[3];
        // obj.header.frame_id = cloud_base_frame_id_;
        
        // if((obj.pose.position.x < -47) && (obj.pose.position.x > -49) && (obj.pose.position.y > 11))   //debug
        detect_objs.push_back(obj);
      }
    }

    std::chrono::time_point<std::chrono::system_clock> t0 = std::chrono::system_clock::now();
    map<long, vector<geometry_msgs::Pose>> tracks;
    map<long, vector<geometry_msgs::Pose>> predicts;
    map<int, int> assignments_detect_track;
    ot_[cls]->detect(detect_objs, detect_objects->header.stamp.toSec(), cls, tracks, predicts,
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
    
    // get track objects info
    for (map<long, vector<geometry_msgs::Pose>>::const_iterator it =
          tracks.begin();
        it != tracks.end(); ++it)
    {
      const autoware_msgs::DetectedObject& track_info = detect_track_info_[cls][it->first];
      autoware_msgs::DetectedObject obj;
      obj.header = detect_objects->header;
      obj.header.frame_id = cloud_track_frame_id_;
      // obj.header.frame_id = cloud_base_frame_id_;
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

      if((abs(obj.velocity.linear.x) > velocity_limit_) || (abs(obj.velocity.linear.y) > velocity_limit_)) continue;

      // if((obj.id == 31) || (obj.id == 43))
      pub_track_objects.objects.push_back(obj);
      tf::Quaternion q(obj.pose.orientation.x,
                        obj.pose.orientation.y, 
                        obj.pose.orientation.z,
                        obj.pose.orientation.w);
      double roll, pitch, yaw;

      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
      // ROS_INFO("object id %d center %.2f %.2f %.2f rotation %.2f %.2f %.2f %.2f... frame_id:%s", obj.id, obj.pose.position.x, obj.pose.position.y, obj.pose.position.z, obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w, obj.header.frame_id.c_str());
      detect_track_info[it->first] = track_info;
    }
    detect_track_info_[cls] = detect_track_info;

    //get predict objects info
    for (map<long, vector<geometry_msgs::Pose>>::const_iterator it =
            predicts.begin();
          it != predicts.end(); ++it)
    {
      const autoware_msgs::DetectedObject& track_info = detect_track_info_[cls][it->first];
      autoware_msgs::DetectedObject obj;
      obj.header = detect_objects->header;
      obj.header.frame_id = cloud_track_frame_id_;
      // obj.header.frame_id = cloud_base_frame_id_;
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

      pub_predict_objects.objects.push_back(obj);
      tf::Quaternion q(obj.pose.orientation.x,
                        obj.pose.orientation.y, 
                        obj.pose.orientation.z,
                        obj.pose.orientation.w);
      double roll, pitch, yaw;

      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
      // ROS_INFO("object id %d center %.2f %.2f %.2f rotation %.2f %.2f %.2f %.2f... frame_id:%s", obj.id, obj.pose.position.x, obj.pose.position.y, obj.pose.position.z, obj.pose.orientation.x, obj.pose.orientation.y, obj.pose.orientation.z, obj.pose.orientation.w, obj.header.frame_id.c_str());
      detect_track_info[it->first] = track_info;
    }
  }

  pub_track_objects.header.frame_id = cloud_track_frame_id_;
  // pub_track_objects.header.frame_id = cloud_base_frame_id_;
  pub_predict_objects.header.frame_id = cloud_track_frame_id_;
  // pub_predict_objects.header.frame_id = cloud_base_frame_id_;
  pub_track_objects_.publish(pub_track_objects);
  pub_predict_objects_.publish(pub_predict_objects);
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

bool Tracking::lookupTransform(const std::string& target_frame,
                                 const std::string& source_frame,
                                 const ros::Time& time,
                                 const ros::Duration timeout,
                                 Eigen::Isometry3d& transform)
{
  while (1)
  {
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
      transform_stamped =
          tfBuffer_.lookupTransform(target_frame, source_frame, time);
    }
    catch (tf2::TransformException& ex)
    {
      if (ros::Time::now() - time > timeout)
      {
        LOG(WARNING) << "Look TF failed: " << ex.what();
        return false;
      }

      usleep(1000);
      continue;
    }
    tf::transformMsgToEigen(transform_stamped.transform, transform);
    break;
  }

  return true;
}