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

#include <ccmslam_msgs/LoopClosure.h>
#include <cslam/Datatypes.h>
#include <cslam/server/ServerSystem.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <voxgraph_msgs/LoopClosure.h>

#include <string>
#include <vector>

// TODO(mikexyl): based on some doc, publish should be thread safe already
class LoopClosureSendFunctor {
 public:
  LoopClosureSendFunctor(const ros::NodeHandle& nh) : nh_(nh) {
    for (int i = 0; i < kMaxClientNum; i++) {
      loop_closure_pub_.emplace_back(nh_.advertise<voxgraph_msgs::LoopClosure>(
          "loop_closure_out_" + std::to_string(i), 10, true));
    }
    map_fusion_pub_ =
        nh_.advertise<ccmslam_msgs::LoopClosure>("map_fusion_out", 10, true);
  }

  bool operator()(const size_t& from_client_id, const double& from_timestamp,
                  const size_t& to_client_id, const double& to_timestamp,
                  const cv::Mat& R, const cv::Mat& t) {
    tf2::Matrix3x3 tf2_rot(
        R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
        R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
        R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2));
    tf2::Quaternion tf2_quaternion;
    tf2_rot.getRotation(tf2_quaternion);

    ccmslam_msgs::LoopClosure map_fusion_msg;
    map_fusion_msg.from_client_id = from_client_id;
    map_fusion_msg.from_timestamp = ros::Time(from_timestamp);
    map_fusion_msg.to_client_id = to_client_id;
    map_fusion_msg.to_timestamp = ros::Time(to_timestamp);
    map_fusion_msg.transform.rotation = tf2::toMsg(tf2_quaternion);
    map_fusion_msg.transform.translation.x = t.at<float>(0);
    map_fusion_msg.transform.translation.y = t.at<float>(1);
    map_fusion_msg.transform.translation.z = t.at<float>(2);
    map_fusion_pub_.publish(map_fusion_msg);
    if (from_client_id == to_client_id) {
      ROS_INFO(
          "Loop Closure Message Published, from client %d time %d, to "
          "time %d",
          from_client_id, map_fusion_msg.from_timestamp,
          map_fusion_msg.to_timestamp);
    } else {
      ROS_INFO(
          "Map Fusion Message Published, from client %d time %d, to client "
          "%d time %d ",
          from_client_id, map_fusion_msg.from_timestamp, to_client_id,
          map_fusion_msg.to_timestamp);
    }
    return true;
  }

 private:
  ros::NodeHandle nh_;
  std::vector<ros::Publisher> loop_closure_pub_;
  ros::Publisher map_fusion_pub_;

  constexpr static int kMaxClientNum = 4;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "CSLAM server node");

  if (argc != 2) {
    cerr << endl << "Usage: rosrun cslam clientnode path_to_vocabulary" << endl;
    ros::shutdown();
    return 1;
  }

  ros::NodeHandle Nh;
  ros::NodeHandle NhPrivate("~");

  LoopClosureSendFunctor loop_closure_send_functor(Nh);
  cslam::fLoopSendFunc loop_closure_send_func = loop_closure_send_functor;

  boost::shared_ptr<cslam::ServerSystem> pSSys{
      new cslam::ServerSystem(Nh, NhPrivate, argv[1])};
  pSSys->InitializeClients();
  pSSys->SetLoopSendFunc(loop_closure_send_func);

  ROS_INFO("started CSLAM server node...");

  ros::MultiThreadedSpinner MSpin(2);

  MSpin.spin();

  ros::waitForShutdown();

  return 0;
}
