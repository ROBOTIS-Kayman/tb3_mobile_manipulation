/*******************************************************************************
* Copyright 2020 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman Jung */

#ifndef TB3_MOBILE_MANIPULATION_TASK_MANAGER_H
#define TB3_MOBILE_MANIPULATION_TASK_MANAGER_H

#include <ros/ros.h>
#include <ros/package.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>

#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/MarkerArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include <yaml-cpp/yaml.h>

#include <boost/thread.hpp>

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/LU>
#include <eigen3/Eigen/QR>

#include "service.h"

namespace tb3_mobile_manipulation
{

class TaskManager
{
public:
  // enum

  // const

  // constructor
  TaskManager();

  // method
  bool run_task(const std::string& task_name);
  void run_task_thread(Service *current_service);

  // variable

 protected:

  // enum

  // const

  // method
  void callback_thread();

  void get_quaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion& quaternion);
  void get_euler_angle(const geometry_msgs::Quaternion& quaternion, double& roll, double& pitch, double& yaw);
  bool get_target_pose(const std::string& target_name, geometry_msgs::Pose& target_pose);
  void load_task_data(const std::string& path);

  void approach_target(const std::string &target_name);
  void approach_target_thread(const geometry_msgs::Pose2D &present_pose, const geometry_msgs::Pose2D& target_pose);
  void leave_target(const std::string &command);
  void leave_target_thread(const geometry_msgs::Pose2D &present_pose, const geometry_msgs::Pose2D& target_pose);
  void nav_to_target(const std::string& target_name);
  void nav_to_target(const geometry_msgs::Pose &target_pose);
  void nav_to_target(const std::string& target_name, const std::string& real_target);
  void nav_to_target(const geometry_msgs::Pose &target_pose, const std::string& real_target);
  void nav_to_target_thread(const geometry_msgs::Pose &target_pose, const std::string &real_target);
  void look_around(const std::string& target_name);
  void look_around_thread(int direction, const std::string& target_name);

  void publish_goal_nav_msg(const geometry_msgs::PoseStamped& goal_msg);
  void publish_cmd_vel_msg(const geometry_msgs::Twist& msg);
  void publish_marker(bool clear, const std::vector<geometry_msgs::Pose2D> &pose_list);

  void command_msg_callback(const std_msgs::String::ConstPtr& msg);
  void navigation_result_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg);



  // variable
  // ros node handle
//  ros::NodeHandle nh_;
//  ros::NodeHandle priv_nh_;
  bool is_running_thread_;
  boost::thread* moving_thread_;
  boost::thread* task_thread_;
  int navigation_status_;

  std::vector<geometry_msgs::Pose2D> approach_pose_list_;
  std::vector<std::string> marker_name_list_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  std::map<std::string, Service*> room_service_list_;

  // publisher/subscriber
  ros::Publisher goal_nav_pub_;
  ros::Publisher cmd_vel_pub_;
  ros::Publisher debug_marker_pub_;
  ros::Subscriber cmd_sub_;
  ros::Subscriber navigation_result_sub_;
  
  // service
};
}

#endif // TB3_MOBILE_MANIPULATION_TASK_MANAGER_H
