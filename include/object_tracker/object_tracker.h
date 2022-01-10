#ifndef PEDESTRIANLOCALISATION_H
#define PEDESTRIANLOCALISATION_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>

#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <bayes_tracking/BayesFilter/bayesFlt.hpp>

#include <XmlRpcValue.h>

#include <string.h>
#include <vector>
#include <math.h>

#include "object_tracker/flobot_tracking.h"
#include "object_tracker/asso_exception.h"

#define INVALID_ID -1

class ObjectTracker
{
public:
  ObjectTracker(const ros::NodeHandle& n);
  ~ObjectTracker();

  void detect(const std::vector<autoware_msgs::DetectedObject>& ppl,
              const double time_stamp, const size_t detector_idx,
              std::map<long, std::vector<geometry_msgs::Pose>>& tracks, std::map<int, int>& assignments);

private:
  void parseParams(ros::NodeHandle);

  ros::NodeHandle nh;
  std::vector<std::string> detector_names;
  XmlRpc::XmlRpcValue detectors;
  
  SimpleTracking<EKFilter>* ekf = NULL;
  SimpleTracking<UKFilter>* ukf = NULL;
  SimpleTracking<PFilter>* pf = NULL;
};

#endif // PEDESTRIANLOCALISATION_H
