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

#ifdef TB3_MOBILE_MANIPULATION_MOVEIT

namespace tb3_mobile_manipulation
{

// ========================================
// ==========     Manipulation
// ========================================
void TaskManager::init_manipulation()
{
  // Moveit
  //  ros::AsyncSpinner spinner(1);
  //  spinner.start();

  // Move group arm
  std::string planning_group_name = "arm";
  move_group_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name);

  // Move group gripper
  std::string planning_group_name2 = "gripper";
  move_group2_ = new moveit::planning_interface::MoveGroupInterface(planning_group_name2);
}

void TaskManager::move_arm_joint(const std::string& target_pose)
{
  auto find_it = arm_pose_list_.find(target_pose);

  if(find_it != arm_pose_list_.end())
  {
    double path_time = 3.0;
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
  double path_time = 3.0;

  // changed target_position from base_footprint
  geometry_msgs::Point position_from_footprint;
  position_from_footprint.x = target_position.x + (-0.092);
  position_from_footprint.y = target_position.y + 0.0;
  position_from_footprint.z = target_position.z + 0.101;

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::move_arm_task_space_thread, this, position_from_footprint, path_time));
  delete moving_thread_;
}

void TaskManager::move_arm_joint_space_thread(const std::vector<std::string>& joint_name, const std::vector<double>& joint_angle, double path_time)
{
  is_running_sub_task_thread_ = true;
  bool result;
  int interval_ms = 2000;

  // move
  //  ros::AsyncSpinner spinner(1);
  //  spinner.start();

  // Next get the current set of joint values for the group.
  //  const robot_state::JointModelGroup* joint_model_group =
  //      move_group_->getCurrentState()->getJointModelGroup("arm");

  //  moveit::core::RobotStatePtr current_state = move_group_->getCurrentState();

//  robot_state::RobotState start_state(*move_group_->getCurrentState());
//  move_group_->setStartState(start_state);

  //  std::vector<double> joint_group_positions;
  //  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  //  joint_group_positions[0] = joint_angle.at(0);  // radians
  //  joint_group_positions[1] = joint_angle.at(1);  // radians
  //  joint_group_positions[2] = joint_angle.at(2);  // radians
  //  joint_group_positions[3] = joint_angle.at(3);  // radians
  move_group_->setJointValueTarget(joint_angle);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  task_result_ = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (task_result_ == true)
  {
    //    move_group_->move();

    ROS_INFO("JOINT : Start moving");
    double moving_time = my_plan.trajectory_.joint_trajectory.points.rbegin()->time_from_start.toSec();
    task_result_ = move_group_->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    //    task_result_ = move_group_->asyncExecute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    ROS_INFO("JOINT : Done executed");
    boost::this_thread::sleep_for(boost::chrono::milliseconds(int(moving_time * 1000) + interval_ms));
    ROS_INFO("JOINT : Done moved");
  }
  else
    ROS_ERROR("Failed to plan for arm : joint space");

  //  spinner.stop();

  is_running_sub_task_thread_ = false;
}

void TaskManager::move_arm_task_space_thread(const geometry_msgs::Point& kinematics_position, double path_time)
{
  is_running_sub_task_thread_ = true;
  bool result;
  int interval_ms = 2000;

  //  ros::AsyncSpinner spinner(1);
  //  spinner.start();

  // Uncomment below to keep the end-effector parallel to the ground
  //  moveit_msgs::OrientationConstraint oc;
  //  oc.link_name = "end_effector_link";
  //  oc.header.frame_id = "link1";
  //  oc.orientation.w = 1.0;
  //  oc.absolute_x_axis_tolerance = 0.1 * M_PI;
  //  oc.absolute_y_axis_tolerance = 0.1 * M_PI;
  //  oc.absolute_z_axis_tolerance = 2.0 * M_PI;
  //  oc.weight = 1.0;
  //  moveit_msgs::Constraints constraints;
  //  constraints.orientation_constraints.push_back(oc);
  //  move_group_->setPathConstraints(constraints);

/*  std::string base_frame_id = robot_name_ + "/base_footprint";
  move_group_->setPoseReferenceFrame(base_frame_id);

  robot_state::RobotState start_state(*move_group_->getCurrentState());
  move_group_->setStartState(start_state)*/;

  geometry_msgs::Pose target_pose;
  target_pose.position = kinematics_position;
  target_pose.orientation.w = 1.0;
  move_group_->setPoseTarget(target_pose); // Cannot use setPoseTarget as the robot has only 4DOF.

//  move_group_->setGoalPositionTolerance(0.005);
  move_group_->setGoalOrientationTolerance(2.0 * M_PI / 180.0);
  //  move_group_->setGoalJointTolerance(5.0 * M_PI / 180.0);

  //  move_group_->setPositionTarget(
  //        kinematics_position.x,
  //        kinematics_position.y,
  //        kinematics_position.z);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  task_result_ = (move_group_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (task_result_ == true)
  {
    ROS_INFO_STREAM("TASK : Start moving [" << kinematics_position.x << ", " << kinematics_position.y << ", " << kinematics_position.z << "]");
    if(my_plan.trajectory_.joint_trajectory.points.size() != 0)
    {

      double moving_time = my_plan.trajectory_.joint_trajectory.points.rbegin()->time_from_start.toSec();

      std::cout << "tra value : ";
      for(auto it = my_plan.trajectory_.joint_trajectory.points.begin(); it != my_plan.trajectory_.joint_trajectory.points.end(); ++it)
        for(auto it_sub = it->positions.begin(); it_sub != it->positions.end(); ++it_sub)
          std::cout << *it_sub << ", ";
      std::cout << std::endl << "time : " << moving_time << std::endl;
    }
    double moving_time = my_plan.trajectory_.joint_trajectory.points.rbegin()->time_from_start.toSec();
    task_result_ = move_group_->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    //    task_result_ = move_group_->asyncExecute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    if(task_result_ == true)
    {
      ROS_INFO("TASK : Done executed");

      // wait for finishing job
      boost::this_thread::sleep_for(boost::chrono::milliseconds(int(moving_time * 1000) + interval_ms));
      ROS_INFO("TASK : Done moved");
    }
    else
      ROS_ERROR("Fail to execute the plan : task space");
  }
  else
    ROS_ERROR("Failed to plan for arm : task space");

  //  spinner.stop();

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
  int moving_time = 1500;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Next get the current set of joint values for the group.
  const robot_state::JointModelGroup* joint_model_group =
      move_group2_->getCurrentState()->getJointModelGroup("gripper");

  moveit::core::RobotStatePtr current_state = move_group2_->getCurrentState();

  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

  // Now, let's modify one of the joints, plan to the new joint space goal and visualize the plan.
  joint_group_positions[0] = gripper_position;  // radians
  move_group2_->setJointValueTarget(joint_group_positions);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  result = (move_group2_->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (result == true)
  {
    //    moving_time = int(my_plan.trajectory_.joint_trajectory.points.rbegin()->time_from_start.toSec() * 1000);
    task_result_ = move_group2_->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS;
    //    move_group2_->move();

    //  if(result == true)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time));
  }
  else
    ROS_ERROR("Failed to plan for gripper");

  spinner.stop();
  // check gripper status

  is_running_sub_task_thread_ = false;
}

}

#endif
