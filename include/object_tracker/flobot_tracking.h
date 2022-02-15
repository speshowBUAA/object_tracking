/***************************************************************************
 *   Copyright (C) 2011 by Nicola Bellotto                                 *
 *   nbellotto@lincoln.ac.uk                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef SIMPLE_TRACKING_H
#define SIMPLE_TRACKING_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Pose.h>
#include <autoware_msgs/DetectedObject.h>
#include <bayes_tracking/models.h>
#include <bayes_tracking/ekfilter.h>
#include <bayes_tracking/ukfilter.h>
#include <bayes_tracking/pfilter.h>
#include <cstdio>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/optional.hpp>
#include <math.h>
#include <tf/transform_datatypes.h>

#include "object_tracker/multitracker.h"

using namespace MTRKYaw;

template <typename FilterType> class SimpleTracking
{
public:
  SimpleTracking(double sLimit = 1.0)
  {
    time = getTime();
    observation = new FM::Vec(2);
    observation3d = new FM::Vec(3);
    stdLimit = sLimit;
  }
  ~SimpleTracking() {}

  void createConstantVelocityModel(double vel_noise_x, double vel_noise_y, double vel_noise_z)
  {
    cvm = new Models::CVModel(vel_noise_x, vel_noise_y);
    cvm3d = new Models::CVModel3D(vel_noise_x, vel_noise_y, vel_noise_z);
    cvm3dyaw = new Models::CVModel3DYaw(vel_noise_x, vel_noise_y, vel_noise_z);
  }

  void addDetectorModel(std::string name, association_t alg,
                        observ_model_t om_flag, double pos_noise_x,
                        double pos_noise_y, double pos_noise_z, unsigned int createSeqSize = 5,
                        double seqTime = 0.2, unsigned int pruneSeqSize = 5)
  {
    ROS_INFO("Adding detector model for: %s.", name.c_str());
    detector_model det;
    det.om_flag = om_flag;
    det.alg = alg;
    if (om_flag == CARTESIAN)
      det.ctm = new Models::CartesianModel(pos_noise_x, pos_noise_y);
    if (om_flag == CARTESIAN3D)
      det.ctm3d = new Models::CartesianModel3D(pos_noise_x, pos_noise_y, pos_noise_z);
    if (om_flag == CARTESIAN3DYaw)
      det.ctm3dyaw = new Models::CartesianModel3DYaw(pos_noise_x, pos_noise_y, pos_noise_z);
    if (om_flag == POLAR)
      det.plm = new Models::PolarModel(pos_noise_x, pos_noise_y);
    det.createSeqSize = createSeqSize;
    det.seqTime = seqTime;
    det.pruneSeqSize = pruneSeqSize;
    detectors[name] = det;
  }

  std::map<long, std::vector<geometry_msgs::Pose>>
  track(double* track_time = NULL)
  {
    boost::mutex::scoped_lock lock(mutex);
    std::map<long, std::vector<geometry_msgs::Pose>> result;
    dt = getTime() - time;
    time += dt;
    if (track_time)
      *track_time = time;

    // prediction
    cvm->update(dt);
    mtrk.template predict<Models::CVModel>(*cvm);

    //=========================== @todo what's this?
    //===========================//
    detector_model dummy_det;
    mtrk.process(*(dummy_det.ctm));
    mtrk.cleanup();
    //
    //        for(typename std::map<std::string, detector_model>::const_iterator
    //        it = detectors.begin();
    //            it != detectors.end();
    //            ++it) {
    //            // process observations (if available) and update tracks
    //            mtrk.process(*(it->second.ctm), it->second.alg,
    //            it->second.seqSize,  it->second.seqTime);
    //        }
    //==========================================================================//

    for (int i = 0; i < mtrk.size(); i++)
    {
      double theta = atan2(mtrk[i].filter->x[3], mtrk[i].filter->x[1]);
      ROS_DEBUG("trk_%ld: Position: (%f, %f), Orientation: %f, Std Deviation: %f, %f",
      mtrk[i].id,
      mtrk[i].filter->x[0], mtrk[i].filter->x[2], //x, y
      theta, //orientation
      sqrt(mtrk[i].filter->X(0,0)),
      sqrt(mtrk[i].filter->X(2,2))//std dev
      );

      geometry_msgs::Pose pose, vel, var; // position, velocity, variance

      pose.position.x = mtrk[i].filter->x[0];
      pose.position.y = mtrk[i].filter->x[2];
      pose.orientation.z = sin(theta / 2);
      pose.orientation.w = cos(theta / 2);
      result[mtrk[i].id].push_back(pose);

      vel.position.x = mtrk[i].filter->x[1];
      vel.position.y = mtrk[i].filter->x[3];
      result[mtrk[i].id].push_back(vel);

      var.position.x = mtrk[i].filter->X(0, 0);
      var.position.y = mtrk[i].filter->X(2, 2);
      var.orientation.x = mtrk[i].filter->X(0, 2);
      var.orientation.y = mtrk[i].filter->X(2, 0);
      result[mtrk[i].id].push_back(var);
    }
    return result;
  }

  void addObservation(std::string detector_name,
                      std::vector<autoware_msgs::DetectedObject> obsv, double obsv_time, std::map<int, int>& assignments)
  {
    boost::mutex::scoped_lock lock(mutex);
    ROS_DEBUG("Adding new observations for detector: %s",
              detector_name.c_str());
    // add last observation/s to tracker
    detector_model det;
    try
    {
      det = detectors.at(detector_name);
    }
    catch (std::out_of_range& exc)
    {
      ROS_ERROR("Detector %s was not registered!", detector_name.c_str());
      return;
    }


    dt = getTime() - time;
    time += dt;

    // prediction
    cvm->update(dt);
    cvm3d->update(dt);
    cvm3dyaw->update(dt);
    mtrk.template predict<Models::CVModel>(*cvm);
    mtrk3d.template predict<Models::CVModel3D>(*cvm3d);
    mtrk3dyaw.template predict<Models::CVModel3DYaw>(*cvm3dyaw);
    // mtrk.process(*(det.ctm), det.alg); @todo can we remove this?
    // mtrk.cleanup();

    for (std::vector<autoware_msgs::DetectedObject>::iterator li = obsv.begin();
         li != obsv.end(); ++li)
    {
      if (det.om_flag == CARTESIAN)
      {
        (*observation)[0] = li->pose.position.x;
        (*observation)[1] = li->pose.position.y;
        mtrk.addObservation(*observation, obsv_time, li->pose.position.z);
      }
      if (det.om_flag == CARTESIAN3D)
      {
        tf::Quaternion q(li->pose.orientation.x,
                  li->pose.orientation.y, 
                  li->pose.orientation.z,
                  li->pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        (*observation3d)[0] = li->pose.position.x;
        (*observation3d)[1] = li->pose.position.y;
        (*observation3d)[2] = yaw;
        mtrk3d.addObservation(*observation3d, obsv_time, li->pose.position.z);
      }
      if (det.om_flag == CARTESIAN3DYaw)
      {
        tf::Quaternion q(li->pose.orientation.x,
                  li->pose.orientation.y, 
                  li->pose.orientation.z,
                  li->pose.orientation.w);
        double roll, pitch, yaw;
        tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
        (*observation3d)[0] = li->pose.position.x;
        (*observation3d)[1] = li->pose.position.y;
        (*observation3d)[2] = yaw;
        mtrk3dyaw.addObservation(*observation3d, obsv_time, li->pose.position.z);
      }
      if (det.om_flag == POLAR)
      {
        (*observation)[0] = atan2(li->pose.position.y, li->pose.position.x);                 // bearing
        (*observation)[1] = sqrt(pow(li->pose.position.x, 2) + pow(li->pose.position.y, 2)); // range
        mtrk.addObservation(*observation, obsv_time, li->pose.position.z);
      }
    }

    if (det.om_flag == CARTESIAN)
    {
      mtrk.process(*(det.ctm), det.om_flag, det.alg, det.createSeqSize, det.seqTime, det.pruneSeqSize,
                   stdLimit);
      assignments = mtrk.getAssignments();
      mtrk.cleanup();
    }
    if (det.om_flag == CARTESIAN3D)
    {
      mtrk3d.process(*(det.ctm3d), det.om_flag, det.alg, det.createSeqSize, det.seqTime, det.pruneSeqSize,
                   stdLimit);
      assignments = mtrk3d.getAssignments();
      mtrk3d.cleanup();
    }
    if (det.om_flag == CARTESIAN3DYaw)
    {
      mtrk3dyaw.process(*(det.ctm3dyaw), det.om_flag, det.alg, det.createSeqSize, det.seqTime, det.pruneSeqSize,
                   stdLimit);
      assignments = mtrk3dyaw.getAssignments();
      mtrk3dyaw.cleanup();
    }
    if (det.om_flag == POLAR)
    {
      // det.plm->update(robot_pose.position.x, robot_pose.position.y,
      // robot_pose.orientation.w);
      det.plm->update(0, 0, 0);
      mtrk.process(*(det.plm), det.om_flag, det.alg, det.createSeqSize, det.seqTime, det.pruneSeqSize,
                   stdLimit);
      assignments = mtrk.getAssignments();
      mtrk.cleanup();
    }
  }

  std::map<long, std::vector<geometry_msgs::Pose>> getTracks()
  {
    boost::mutex::scoped_lock lock(mutex);
    std::map<long, std::vector<geometry_msgs::Pose>> result;

    // std::cout << "mtrk.size: " << mtrk.size() << std::endl;

    for (int i = 0; i < mtrk.size(); i++)
    {
      //std::cout << "id: " << mtrk[i].id << std::endl;

      double theta = atan2(mtrk[i].filter->x[3], mtrk[i].filter->x[1]);
      geometry_msgs::Pose pose, vel, var; // position, velocity, variance

      pose.position.x = mtrk[i].filter->x[0];
      pose.position.y = mtrk[i].filter->x[2];
      pose.orientation.z = sin(theta / 2);
      pose.orientation.w = cos(theta / 2);
      result[mtrk[i].id].push_back(pose);

      vel.position.x = mtrk[i].filter->x[1];
      vel.position.y = mtrk[i].filter->x[3];
      result[mtrk[i].id].push_back(vel);

      var.position.x = mtrk[i].filter->X(0, 0);
      var.position.y = mtrk[i].filter->X(2, 2);
      var.orientation.x = mtrk[i].filter->X(0, 2);
      var.orientation.y = mtrk[i].filter->X(2, 0);
      result[mtrk[i].id].push_back(var);
    }
    return result;
  }

  std::map<long, std::vector<geometry_msgs::Pose>> getTracks3D()
  {
    boost::mutex::scoped_lock lock(mutex);
    std::map<long, std::vector<geometry_msgs::Pose>> result;
    // std::cout << "mtrk3d.size: " << mtrk3d.size() << std::endl;

    for (int i = 0; i < mtrk3d.size(); i++)
    {
      //std::cout << "id: " << mtrk3d[i].id << std::endl;
      double yaw = mtrk3d[i].filter->x[4];
      geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);
      geometry_msgs::Pose pose, vel, var; // position, velocity, variance

      pose.position.x = mtrk3d[i].filter->x[0];
      pose.position.y = mtrk3d[i].filter->x[2];
      pose.orientation = q;
      result[mtrk3d[i].id].push_back(pose);

      vel.position.x = mtrk3d[i].filter->x[1];
      vel.position.y = mtrk3d[i].filter->x[3];
      result[mtrk3d[i].id].push_back(vel);

      var.position.x = mtrk3d[i].filter->X(0, 0);
      var.position.y = mtrk3d[i].filter->X(2, 2);
      var.orientation.x = mtrk3d[i].filter->X(0, 2);
      var.orientation.y = mtrk3d[i].filter->X(2, 0);
      result[mtrk3d[i].id].push_back(var);
    }
    return result;
  }

  std::map<long, std::vector<geometry_msgs::Pose>> getTracks3Dyaw()
  {
    boost::mutex::scoped_lock lock(mutex);
    std::map<long, std::vector<geometry_msgs::Pose>> result;
    // std::cout << "mtrk3dyaw.size: " << mtrk3dyaw.size() << std::endl;

    for (int i = 0; i < mtrk3dyaw.size(); i++)
    {
      //std::cout << "id: " << mtrk3dyaw[i].id << std::endl;
      double yaw = mtrk3dyaw[i].filter->x[4];
      geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(yaw);
      geometry_msgs::Pose pose, vel, var; // position, velocity, variance

      pose.position.x = mtrk3dyaw[i].filter->x[0];
      pose.position.y = mtrk3dyaw[i].filter->x[2];
      pose.orientation = q;
      result[mtrk3dyaw[i].id].push_back(pose);

      vel.position.x = mtrk3dyaw[i].filter->x[1];
      vel.position.y = mtrk3dyaw[i].filter->x[3];
      result[mtrk3dyaw[i].id].push_back(vel);

      var.position.x = mtrk3dyaw[i].filter->X(0, 0);
      var.position.y = mtrk3dyaw[i].filter->X(2, 2);
      var.orientation.x = mtrk3dyaw[i].filter->X(0, 2);
      var.orientation.y = mtrk3dyaw[i].filter->X(2, 0);
      result[mtrk3dyaw[i].id].push_back(var);
    }
    return result;
  }
private:
  FM::Vec* observation; // observation [x, y]
  FM::Vec* observation3d; // observation3d [x, y, yaw]
  double dt, time;
  boost::mutex mutex;
  Models::CVModel* cvm;                   // CV model
  Models::CVModel3D* cvm3d;
  Models::CVModel3DYaw* cvm3dyaw;
  MultiTracker<FilterType, 4> mtrk; // state [x, v_x, y, v_y]
  MultiTracker<FilterType, 6> mtrk3d; // state [x, v_x, y, v_y, yaw, v_yaw]
  MultiTracker<FilterType, 6> mtrk3dyaw; // state [x, v_x, y, v_y, yaw, v_yaw]
  double stdLimit; // upper limit for the variance of estimation position

  typedef struct
  {
    Models::CartesianModel* ctm;  // Cartesian observation model
    Models::CartesianModel3D* ctm3d;  // Cartesian3D observation model
    Models::CartesianModel3DYaw* ctm3dyaw;  // Cartesian3DYaw observation model
    Models::PolarModel* plm;      // Polar observation model
    observ_model_t om_flag; // Observation model flag
    association_t alg;      // Data association algorithm
    unsigned int
        createSeqSize; // Minimum number of observations for new track creation
    double
        seqTime; // Minimum interval between observations for new track creation
    unsigned int
        pruneSeqSize; // Minimum number of observations for new track creation
  } detector_model;
  std::map<std::string, detector_model> detectors;

  double getTime() { return ros::Time::now().toSec(); }
};
#endif // SIMPLE_TRACKING_H
