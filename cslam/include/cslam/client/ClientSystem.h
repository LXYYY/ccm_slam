/**
 * This file is part of CCM-SLAM.
 *
 * Copyright (C): Patrik Schmuck <pschmuck at ethz dot ch> (ETH Zurich)
 * For more information see <https://github.com/patriksc/CCM-SLAM>
 *
 * CCM-SLAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * CCM-SLAM is based in the monocular version of ORB-SLAM2 by Raúl Mur-Artal.
 * CCM-SLAM partially re-uses modules of ORB-SLAM2 in modified or unmodified
 * condition. For more information see <https://github.com/raulmur/ORB_SLAM2>.
 *
 * CCM-SLAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with CCM-SLAM. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CSLAM_CLIENTSYSTEM_H_
#define CSLAM_CLIENTSYSTEM_H_

// C++
#include <cv_bridge/cv_bridge.h>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

#include <time.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// CSLAM
#include <cslam/ClientHandler.h>
#include <cslam/Database.h>
#include <cslam/Datatypes.h>
#include <cslam/Map.h>
#include <cslam/config.h>
#include <cslam/estd.h>

using namespace std;

namespace cslam {

class ClientSystem {
 public:
  typedef boost::shared_ptr<ClientHandler> chptr;
  typedef boost::shared_ptr<KeyFrameDatabase> dbptr;
  typedef boost::shared_ptr<Map> mapptr;

 public:
  ClientSystem(ros::NodeHandle Nh, ros::NodeHandle NhPrivate,
               const string& strVocFile, const string& strCamFile,
               eSensor sensor = MONOCULAR);

 private:
  void LoadVocabulary(const string& strVocFile);

  // ROS infrastructure
  ros::NodeHandle mNh;
  ros::NodeHandle mNhPrivate;
  size_t mClientId;

  // Input sensor
  eSensor mSensor;

  // MCP-SLAM Infrastructure
  vocptr mpVoc;
  const string mstrCamFile;
  dbptr mpKFDB;
  mapptr mpMap;
  chptr mpAgent;

  const uidptr mpUID;
};

}  // namespace cslam

#endif
