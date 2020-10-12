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

// TODO(mikexyl): based on some doc, publish should be thread safe already
class LoopClosureSendFunctor {
 public:
  LoopClosureSendFunctor(const ros::Publisher& loop_closure_pub)
      : loop_closure_pub_(loop_closure_pub) {}
  bool operator()(const size_t& from_client_id, const double& from_timestamp,
                  const double& to_client_id, const double& to_timestamp,
                  const cv::Mat& R, const cv::Mat& t) {
    ccmslam_msgs::LoopClosure loop_closure_msg;
    loop_closure_msg.from_client_id = from_client_id;
    loop_closure_msg.from_timestamp = ros::Time(from_timestamp);
    loop_closure_msg.to_client_id = to_client_id;
    loop_closure_msg.to_timestamp = ros::Time(to_timestamp);
    tf2::Matrix3x3 tf2_rot(
        R.at<float>(0, 0), R.at<float>(0, 1), R.at<float>(0, 2),
        R.at<float>(1, 0), R.at<float>(1, 1), R.at<float>(1, 2),
        R.at<float>(2, 0), R.at<float>(2, 1), R.at<float>(2, 2));
    tf2::Quaternion tf2_quaternion;
    tf2_rot.getRotation(tf2_quaternion);
    loop_closure_msg.transform.rotation = tf2::toMsg(tf2_quaternion);
    loop_closure_msg.transform.translation.x = t.at<float>(0);
    loop_closure_msg.transform.translation.y = t.at<float>(1);
    loop_closure_msg.transform.translation.z = t.at<float>(2);
    loop_closure_pub_.publish(loop_closure_msg);

    ROS_INFO(
        "Loop Closure Message Published, from time %d, to "
        "time %d",
        loop_closure_msg.from_timestamp, loop_closure_msg.to_timestamp);
  }

 private:
  ros::NodeHandle node_handle_;
  const ros::Publisher& loop_closure_pub_;
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

  ros::Publisher loop_closure_pub =
      Nh.advertise<ccmslam_msgs::LoopClosure>("loop_closure_input", 1);
  LoopClosureSendFunctor loop_closure_send_functor(loop_closure_pub);
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
