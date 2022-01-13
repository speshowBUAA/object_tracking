#include "object_tracker/object_tracker.h"

using namespace MTRK;
using namespace std;

ObjectTracker::ObjectTracker(const ros::NodeHandle& n) : nh(n)
{
  // Initialize node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run
  // simultaneously
  // while using different parameters.
  parseParams(nh);
}

ObjectTracker::~ObjectTracker() {}

void ObjectTracker::parseParams(ros::NodeHandle n)
{
  std::string filter;
  n.getParam("/filter_type", filter);
  ROS_INFO_STREAM("Found filter type: " << filter);
  if (filter == "EKF")
  {
    if (n.hasParam("/std_limit"))
    {
      double stdLimit;
      n.getParam("/std_limit", stdLimit);
      ROS_INFO("std_limit: %f ", stdLimit);
      ekf = new SimpleTracking<EKFilter>(stdLimit);
    }
    else
    {
      ekf = new SimpleTracking<EKFilter>();
    }
  }
  else if (filter == "UKF")
  {
    if (n.hasParam("/std_limit"))
    {
      double stdLimit;
      n.getParam("/std_limit", stdLimit);
      ROS_INFO("std_limit: %f ", stdLimit);
      ukf = new SimpleTracking<UKFilter>(stdLimit);
    }
    else
    {
      ukf = new SimpleTracking<UKFilter>();
    }
  }
  else if (filter == "PF")
  {
    if (n.hasParam("/std_limit"))
    {
      double stdLimit;
      n.getParam("/std_limit", stdLimit);
      ROS_INFO("std_limit: %f ", stdLimit);
      pf = new SimpleTracking<PFilter>(stdLimit);
    }
    else
    {
      pf = new SimpleTracking<PFilter>();
    }
  }
  else
  {
    ROS_FATAL_STREAM("Filter type " << filter << " is not specified. Unable to "
                                                "create the tracker. Please "
                                                "use either EKF, UKF or PF.");
    return;
  }

  XmlRpc::XmlRpcValue cv_noise;
  n.getParam("/cv_noise_params", cv_noise);
  ROS_ASSERT(cv_noise.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  ROS_INFO_STREAM("Constant Velocity Model noise: " << cv_noise);
  if (ekf == NULL)
  {
    if (ukf == NULL)
    {
      pf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
    }
    else
    {
      ukf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
    }
  }
  else
  {
    ekf->createConstantVelocityModel(cv_noise["x"], cv_noise["y"]);
  }
  ROS_INFO_STREAM(
      "Created " << filter
                << " based tracker using constant velocity prediction model.");

  n.getParam("/detectors", detectors);
  ROS_ASSERT(detectors.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  ROS_INFO_STREAM("Get detectors: " << detectors);
  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = detectors.begin();
      it != detectors.end(); ++it)
  {
    ROS_INFO_STREAM("Found detector: " << (std::string)(it->first) << " ==> "
                                      << detectors[it->first]);

    detector_names.push_back(it->first);

    try
    {
      if (ekf == NULL)
      {
        if (ukf == NULL)
        {
          if (detectors[it->first].hasMember("create_seq_size") &&
              detectors[it->first].hasMember("create_seq_time") &&
              detectors[it->first].hasMember("prune_seq_size"))
          {
            int create_seq_size = detectors[it->first]["create_seq_size"];
            int prune_seq_size = detectors[it->first]["prune_seq_size"];
            double create_seq_time = detectors[it->first]["create_seq_time"];
            pf->addDetectorModel(
                it->first,
                detectors[it->first]["matching_algorithm"] == "NN"
                    ? NN
                    : detectors[it->first]["matching_algorithm"] == "NNJPDA"
                          ? NNJPDA
                          : throw(asso_exception()),
                detectors[it->first]["observation_model"] == "CARTESIAN"
                    ? CARTESIAN
                    : detectors[it->first]["observation_model"] == "POLAR"
                          ? POLAR
                          : detectors[it->first]["observation_model"] == "CARTESIAN3D"
                              ? CARTESIAN3D
                              : detectors[it->first]["observation_model"] == "CARTESIAN3DYaw"
                                ? CARTESIAN3DYaw
                                : throw(observ_exception()),
                detectors[it->first]["noise_params"]["x"],
                detectors[it->first]["noise_params"]["y"],
                (unsigned int)create_seq_size, create_seq_time, (unsigned int)prune_seq_size);
          }
          else
          {
            pf->addDetectorModel(
                it->first,
                detectors[it->first]["matching_algorithm"] == "NN"
                    ? NN
                    : detectors[it->first]["matching_algorithm"] == "NNJPDA"
                          ? NNJPDA
                          : throw(asso_exception()),
                detectors[it->first]["observation_model"] == "CARTESIAN"
                    ? CARTESIAN
                    : detectors[it->first]["observation_model"] == "POLAR"
                          ? POLAR
                          : detectors[it->first]["observation_model"] == "CARTESIAN3D"
                              ? CARTESIAN3D
                              : detectors[it->first]["observation_model"] == "CARTESIAN3DYaw"
                                ? CARTESIAN3DYaw
                                : throw(observ_exception()),
                detectors[it->first]["noise_params"]["x"],
                detectors[it->first]["noise_params"]["y"]);
          }
        }
        else
        {
          if (detectors[it->first].hasMember("create_seq_size") &&
              detectors[it->first].hasMember("create_seq_time") &&
              detectors[it->first].hasMember("prune_seq_size"))
          {
            int create_seq_size = detectors[it->first]["create_seq_size"];
            int prune_seq_size = detectors[it->first]["prune_seq_size"];
            double create_seq_time = detectors[it->first]["create_seq_time"];
            ukf->addDetectorModel(
                it->first,
                detectors[it->first]["matching_algorithm"] == "NN"
                    ? NN
                    : detectors[it->first]["matching_algorithm"] == "NNJPDA"
                          ? NNJPDA
                          : throw(asso_exception()),
                detectors[it->first]["observation_model"] == "CARTESIAN"
                    ? CARTESIAN
                    : detectors[it->first]["observation_model"] == "POLAR"
                          ? POLAR
                          : detectors[it->first]["observation_model"] == "CARTESIAN3D"
                              ? CARTESIAN3D
                              : detectors[it->first]["observation_model"] == "CARTESIAN3DYaw"
                                ? CARTESIAN3DYaw
                                : throw(observ_exception()),
                detectors[it->first]["noise_params"]["x"],
                detectors[it->first]["noise_params"]["y"],
                (unsigned int)create_seq_size, create_seq_time, (unsigned int)prune_seq_size);
          }
          else
          {
            ukf->addDetectorModel(
                it->first,
                detectors[it->first]["matching_algorithm"] == "NN"
                    ? NN
                    : detectors[it->first]["matching_algorithm"] == "NNJPDA"
                          ? NNJPDA
                          : throw(asso_exception()),
                detectors[it->first]["observation_model"] == "CARTESIAN"
                    ? CARTESIAN
                    : detectors[it->first]["observation_model"] == "POLAR"
                          ? POLAR
                          : detectors[it->first]["observation_model"] == "CARTESIAN3D"
                              ? CARTESIAN3D
                              : detectors[it->first]["observation_model"] == "CARTESIAN3DYaw"
                                ? CARTESIAN3DYaw
                                : throw(observ_exception()),
                detectors[it->first]["noise_params"]["x"],
                detectors[it->first]["noise_params"]["y"]);
          }
        }
      }
      else
      {
        if (detectors[it->first].hasMember("create_seq_size") &&
            detectors[it->first].hasMember("create_seq_time") &&
            detectors[it->first].hasMember("prune_seq_size"))
        {
          int create_seq_size = detectors[it->first]["create_seq_size"];
          int prune_seq_size = detectors[it->first]["prune_seq_size"];
          double create_seq_time = detectors[it->first]["create_seq_time"];
          ekf->addDetectorModel(
              it->first,
              detectors[it->first]["matching_algorithm"] == "NN"
                  ? NN
                  : detectors[it->first]["matching_algorithm"] == "NNJPDA"
                        ? NNJPDA
                        : throw(asso_exception()),
              detectors[it->first]["observation_model"] == "CARTESIAN"
                  ? CARTESIAN
                  : detectors[it->first]["observation_model"] == "POLAR"
                        ? POLAR
                        : detectors[it->first]["observation_model"] == "CARTESIAN3D"
                            ? CARTESIAN3D
                            : detectors[it->first]["observation_model"] == "CARTESIAN3DYaw"
                              ? CARTESIAN3DYaw
                              : throw(observ_exception()),
              detectors[it->first]["noise_params"]["x"],
              detectors[it->first]["noise_params"]["y"], (unsigned int)create_seq_size,
              create_seq_time, (unsigned int)prune_seq_size);
        }
        else
        {
          ekf->addDetectorModel(
              it->first,
              detectors[it->first]["matching_algorithm"] == "NN"
                  ? NN
                  : detectors[it->first]["matching_algorithm"] == "NNJPDA"
                        ? NNJPDA
                        : throw(asso_exception()),
              detectors[it->first]["observation_model"] == "CARTESIAN"
                  ? CARTESIAN
                  : detectors[it->first]["observation_model"] == "POLAR"
                        ? POLAR
                        : detectors[it->first]["observation_model"] == "CARTESIAN3D"
                            ? CARTESIAN3D
                            : detectors[it->first]["observation_model"] == "CARTESIAN3DYaw"
                              ? CARTESIAN3DYaw
                              : throw(observ_exception()),
              detectors[it->first]["noise_params"]["x"],
              detectors[it->first]["noise_params"]["y"]);
        }
      }
    }
    catch (asso_exception& e)
    {
      ROS_FATAL_STREAM(
          "" << e.what() << " " << detectors[it->first]["matching_algorithm"]
            << " is not specified. Unable to add " << (std::string)(it->first)
            << " to the tracker. Please use either NN or NNJPDA as "
                "association algorithms.");
      return;
    }
    catch (observ_exception& e)
    {
      ROS_FATAL_STREAM(
          "" << e.what() << " " << detectors[it->first]["observation_model"]
            << " is not specified. Unable to add " << (std::string)(it->first)
            << " to the tracker. Please use either CARTESIAN or POLAR as "
                "observation models.");
      return;
    }
  }
}

void ObjectTracker::detect(const vector<autoware_msgs::DetectedObject>& ppl,
                           const double time_stamp, const size_t detector_idx,
                           map<long, vector<geometry_msgs::Pose>>& tracks,
                           map<int, int>& assignments)
{
  // std::cerr << "[people_tacker] got " << pta->poses.size() << " poses, from "
  // << detector << std::endl;

  if (ppl.size())
  {
    if (ekf == NULL)
    {
      if (ukf == NULL)
      {
        pf->addObservation(detector_names[detector_idx], ppl, time_stamp,
                           assignments);
        tracks = detectors[detector_names[detector_idx]]["observation_model"] == "CARTESIAN3D" ? pf->getTracks3D()
                  : detectors[detector_names[detector_idx]]["observation_model"] == "CARTESIAN3DYaw" ? pf->getTracks3Dyaw()
                  : pf->getTracks();
      }
      else
      {
        ukf->addObservation(detector_names[detector_idx], ppl, time_stamp,
                            assignments);
        tracks = detectors[detector_names[detector_idx]]["observation_model"] == "CARTESIAN3D" ? ukf->getTracks3D()
                 : detectors[detector_names[detector_idx]]["observation_model"] == "CARTESIAN3DYaw" ? ukf->getTracks3Dyaw()
                 : ukf->getTracks();
      }
    }
    else
    {
      ekf->addObservation(detector_names[detector_idx], ppl, time_stamp,
                          assignments);
      tracks = detectors[detector_names[detector_idx]]["observation_model"] == "CARTESIAN3D" ? ekf->getTracks3D()
                : detectors[detector_names[detector_idx]]["observation_model"] == "CARTESIAN3DYaw" ? ekf->getTracks3Dyaw()
                : ekf->getTracks();
    }
  }
}
