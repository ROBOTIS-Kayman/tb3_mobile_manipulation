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

#include "tb3_mobile_manipulation_manager/task_manager.h"

namespace tb3_mobile_manipulation
{

// ========================================
// ==========     Manipulation
// ========================================
bool TaskManager::set_actuator_state(bool state)
{
  open_manipulator_msgs::SetActuatorState srv;
  srv.request.set_actuator_state = state;

  if(set_actuator_state_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void TaskManager::move_arm_joint(const std::string& target_pose)
{
  auto find_it = arm_pose_list_.find(target_pose);

  if(find_it != arm_pose_list_.end())
  {
    double path_time = 2.0;
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;

    for(auto element : find_it->second)
    {
      joint_name.push_back(element.first);
      joint_angle.push_back(element.second);
      //      std::cout << "joint : " << element.first << ", angle : " << element.second << std::endl;
    }

    if((joint_name.size() != 0) && (joint_name.size() == joint_angle.size()))
    {
      ROS_INFO_STREAM_COND(DEBUG, "move arm : " << target_pose);
      moving_thread_ = new boost::thread(boost::bind(&TaskManager::move_arm_joint_space_thread, this, joint_name, joint_angle, path_time));
      delete moving_thread_;
    }
  }
}

void TaskManager::move_arm_task(const std::string& target_pose)
{

}

void TaskManager::move_arm_task(const geometry_msgs::Point &target_position)
{
  double path_time = 2.0;

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::move_arm_task_space_thread, this, target_position, path_time));
  delete moving_thread_;
}

void TaskManager::move_arm_joint_space_thread(const std::vector<std::string>& joint_name, const std::vector<double>& joint_angle, double path_time)
{
  is_running_sub_task_thread_ = true;
  bool result;

  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    result = srv.response.is_planned;
    if(result == false)
      ROS_ERROR("Planning is failed");
  }
  else
  {
    ROS_ERROR("No response from server for manipulation");
    result = false;
  }

  if(result == true)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(int(path_time * 1000)));

  is_running_sub_task_thread_ = false;
}

void TaskManager::move_arm_task_space_thread(const geometry_msgs::Point& kinematics_position, double path_time)
{
  is_running_sub_task_thread_ = true;
  bool result;

  open_manipulator_msgs::SetKinematicsPose srv;

  srv.request.end_effector_name = "gripper";

  srv.request.kinematics_pose.pose.position = kinematics_position;

  srv.request.kinematics_pose.pose.orientation.w = 1;
  srv.request.kinematics_pose.pose.orientation.x = 0;
  srv.request.kinematics_pose.pose.orientation.y = 0;
  srv.request.kinematics_pose.pose.orientation.z = 0;

  srv.request.path_time = path_time;

  if(goal_task_space_path_position_only_client_.call(srv))
  {
    result = srv.response.is_planned;
  }
  else
    result = false;

  if(result == true)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(int(path_time * 1000)));

  is_running_sub_task_thread_ = false;
}

void TaskManager::open_gripper()
{
  double gripper_position = 0.013;
  ROS_INFO_COND(DEBUG, "open gripper");

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::move_gripper_thread, this, gripper_position));
  delete moving_thread_;
}

void TaskManager::close_gripper()
{
  double gripper_position = -0.013;
  ROS_INFO_COND(DEBUG, "close gripper");

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::move_gripper_thread, this, gripper_position));
  delete moving_thread_;
}

void TaskManager::move_gripper_thread(double gripper_position)
{
  is_running_sub_task_thread_ = true;
  bool result;
  int moving_time = 2000;

  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back("gripper");
  srv.request.joint_position.position.push_back(gripper_position);

  if(goal_tool_control_client_.call(srv))
  {
    result =  srv.response.is_planned;
  }
  else
  {
    result = false;
  }

  if(result == true)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time));

  // check gripper status

  is_running_sub_task_thread_ = false;
}

}
