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
// ==========      Mission
// ========================================
void TaskManager::start_mission()
{
  if(is_running_mission_ == true)
    return;

  ROS_INFO_STREAM("Start mission!!!!!");

  // ready

  boost::thread *misstion_thread = new boost::thread(boost::bind(&TaskManager::mission_thread, this, ""));
  delete misstion_thread;
}

void TaskManager::pause_mission()
{  
  if(is_running_mission_ == true)
    is_pause_ = true;
}

void TaskManager::resume_mission()
{
  if(is_running_mission_ == true)
    is_pause_ = false;
}

void TaskManager::restart_mission(const std::string& mission_name)
{
  if(is_running_mission_ == true)
    return;

  ROS_INFO_STREAM("Restart mission!!!!! : " << mission_name);

  // ready

  boost::thread *misstion_thread = new boost::thread(boost::bind(&TaskManager::mission_thread, this, mission_name));
  delete misstion_thread;

}

void TaskManager::stop_mission()
{  
  if(is_running_mission_ == true)
  {
    is_stop_ = true;
    is_stop_mission_ = true;
  }
  else
    finish_task();
}

void TaskManager::on_start_mission()
{
  if(is_ready_mission_ == true)
    return;

  ready_task();

  sleep_for(100, 0, is_running_task_thread_, is_pause_, is_stop_);
}

void TaskManager::on_finish_mission()
{
  finish_task();

  sleep_for(100, 0, is_running_task_thread_, is_pause_, is_stop_);
}

void TaskManager::mission_thread(const std::string& start_mission)
{
  is_running_mission_ = true;
  mission_result_ = true;

  on_start_mission();

  int sleep_ms = 100;
  bool is_restart = false;

  if(start_mission != "")
    is_restart = true;

  //  for(auto service_element : room_service_list_)
  //  {
  //    if(is_restart == true)
  //    {
  //      if(service_element.first != start_mission)
  //        continue;
  //      else
  //        is_restart = false;
  //    }

  //    run_task(service_element.first);
  run_task("");

  bool continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_task_thread_, is_pause_mission_, is_stop_mission_);
  if(continue_result == false)
  {
    on_stop_mission();
    return;
  }

  //    // check mission result
  //    if(mission_result_ == false)
  //    {
  //      ROS_ERROR_STREAM("Mission is failed : " << service_element.first);
  //      on_stop_mission();
  //      return;
  //    }
  //  }

  on_finish_mission();
  is_running_mission_ = false;
}
// ========================================
// ==========      Task
// ========================================
void TaskManager::control_task(COMMAND command)
{
  switch(command)
  {
  case STOP:
    if(is_running_task_thread_ == true)
      is_stop_ = true;
    break;

  case PAUSE:
    if(is_running_task_thread_ == true && is_pause_ == false)
    {
      ROS_INFO("The current task will be paused.");
      is_pause_ = true;
    }
    break;

  case RESUME:
    if(is_running_task_thread_ == true && is_pause_ == true)
    {
      ROS_INFO("The current task will be resumed.");
      is_pause_ = false;
    }
    break;

  case READY:
    if(is_running_task_thread_ == false)
    {
      ready_task();
    }
    break;

  default:
    return;
  }
}

void TaskManager::ready_task()
{
  moving_thread_ = new boost::thread(boost::bind(&TaskManager::ready_task_thread, this));
  delete moving_thread_;
}

void TaskManager::ready_task_thread()
{
  is_running_task_thread_ = true;

  // reset odom
  publish_reset_turtlebot();

  sleep_for(100, 3000, is_running_sub_task_thread_, is_pause_, is_stop_);

  // move arm to init pose
  move_arm_joint("home_with_object");

  sleep_for(100, 0, is_running_sub_task_thread_, is_pause_, is_stop_);

  open_gripper();

  sleep_for(100, 0, is_running_sub_task_thread_, is_pause_, is_stop_);

  // reset init pose
  geometry_msgs::PoseWithCovariance init_pose;
  init_pose.pose.position.x = 0;
  init_pose.pose.position.y = 0;
  init_pose.pose.position.z = 0;
  init_pose.pose.orientation.x = 0;
  init_pose.pose.orientation.y = 0;
  init_pose.pose.orientation.z = 0;
  init_pose.pose.orientation.w = 1;

  init_pose.covariance.at(0) = 0.25;
  init_pose.covariance.at(7) = 0.25;
  init_pose.covariance.at(35) = 0.06853891945200942;
  publish_init_pose(init_pose);

  sleep_for(100, 3000, is_running_sub_task_thread_, is_pause_, is_stop_);

  ROS_INFO("Ready to run the task");

  is_ready_mission_ = true;
  is_running_task_thread_ = false;
}

void TaskManager::finish_task()
{
  ROS_INFO("prepared to finish the job.");

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::finish_task_thread, this));
  delete moving_thread_;

  is_ready_mission_ = false;
}

void TaskManager::finish_task_thread()
{
  is_running_task_thread_ = true;

  // move arm to init pose
  open_gripper();

  sleep_for(100, 0, is_running_sub_task_thread_, is_pause_, is_stop_);

  move_arm_joint("termination_pose");

  ROS_INFO("Done a task for finish");

  is_running_task_thread_ = false;
}

bool TaskManager::run_task(const std::string& task_name)
{
  //  Service *current_service;
  //  auto it = room_service_list_.find(task_name);
  //  if(it == room_service_list_.end())
  //  {
  //    ROS_ERROR_STREAM("No service for " << task_name);
  //    return false;
  //  }

  //  if(is_running_task_thread_ == true)
  //  {
  //    ROS_ERROR("Already running the task!");
  //    return false;
  //  }

  //  current_service = it->second;

  //  task_thread_ = new boost::thread(boost::bind(&TaskManager::run_task_thread, this, current_service));
  //  delete task_thread_;


  task_thread_ = new boost::thread(boost::bind(&TaskManager::scenario_thread, this));
  delete task_thread_;

  return true;
}

void TaskManager::run_task_thread(Service* current_service)
{
  is_running_task_thread_ = true;

  int sleep_ms = 100;
  //  int approach_repeat = 3;
  bool continue_result = true;

  ROS_WARN_STREAM("Start Task : " << current_service->get_name());

  // nav to object
  std::string object_marker_name = current_service->get_object();
  std::string target_marker_name = current_service->get_target();

  ROS_WARN("Nav to Object");
  int nav_result = navigation(object_marker_name, true);

  // failed to navigation or received stop command
  if(nav_result == TASK::FAIL)
  {
    on_stop_task();
    return;
  }

  // wait for detecting ar marker
  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

  // approach object
  ROS_WARN("Approach to Object");
  for(int ix = 0; ix < repeat_times_; ix++)
  {
    bool approach_result = approach_target(object_marker_name, repeat_times_, ix + 1);

    if(approach_result == false)
    {
      // go back and find the target
      // leave
      leave_target("leave_back_inter");

      continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
      if(continue_result == false)
      {
        on_stop_task();
        return;
      }

      // wait 1 sec
      boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

      // try again
      approach_result = approach_target(object_marker_name, repeat_times_, ix + 1);

      if(approach_result == false)
      {
        ROS_ERROR("Failed to approach");
        return;
      }
    }

    // wait for approaching
    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }

    // fail to apprach because of obstacle, try again
    if(task_result_ == false)
    {
      task_result_ = true;
      ix--;
    }

    if(ix == (repeat_times_ - 1))
      break;

    // leave
    leave_target("leave_back_inter");

    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }
  }

  // manipulation : pick
  ROS_WARN("Manipulatin :Pick");
  geometry_msgs::Point object_position;
  current_service->get_object_position(object_position.x, object_position.y, object_position.z);
  move_arm_task(object_position);

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  if(task_result_ == false)
  {
    task_result_ = true;
    move_arm_joint("target_pose");

    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }
  }

  // manipulation :: close gripper
  close_gripper();
  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // manipulation : move to via pose
  //  move_arm_joint("via_pose");
  object_position.z += 0.05;
  move_arm_task(object_position);
  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // manipulation : move to init pose
  move_arm_joint("home_with_object");
  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // leave
  ROS_WARN("Leave Back");
  leave_target("leave_back");

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // turn to next target
  geometry_msgs::Pose room_pose;
  bool result = current_service->get_room_center(room_pose.position.x, room_pose.position.y);
  if(result == false)
    return;

  turn_to_target(room_pose);

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // nav to room
  ROS_WARN("Nav to Room");
  double yaw = atan2(room_pose.position.y, room_pose.position.x);
  get_quaternion(0, 0, yaw, room_pose.orientation);

  nav_result = navigation(room_pose, target_marker_name, false);

  // failed to navigation or received stop command
  if(nav_result == TASK::FAIL)
  {
    on_stop_task();
    return;
  }
  //  }

  // find target
  ROS_WARN("Try to find the target");
  look_around(target_marker_name);

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // nav to target
  ROS_WARN("Nav to Target");
  nav_result = navigation(target_marker_name, false);

  // failed to navigation or received stop command
  if(nav_result == TASK::FAIL)
  {
    on_stop_task();
    return;
  }

  // wait for detecting ar marker
  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

  // approach target
  ROS_WARN("Approach Target");
  int approach_repeat = 1;
  for(int ix = 0; ix < approach_repeat; ix++)
  {
    // approach object
    bool approach_result = approach_target(target_marker_name, approach_repeat, ix + 1);

    if(approach_result == false)
    {
      // go back and find the target
      // leave
      leave_target("leave_back_inter");

      continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
      if(continue_result == false)
      {
        on_stop_task();
        return;
      }

      // wait 1 sec
      boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

      // try again
      approach_result = approach_target(object_marker_name, repeat_times_, ix + 1);

      if(approach_result == false)
      {
        ROS_ERROR("Failed to approach");
        return;
      }
    }

    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }

    if(ix == (approach_repeat - 1))
      break;

    // leave
    leave_target("leave_back_inter");

    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }
  }

  // manipulation : move to via pose
  ROS_WARN("Manipulation : Place");
  //  move_arm_joint("via_pose");

  //  object_position.x -= 0.05;
  move_arm_task(object_position);
  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // manipulation : place
  geometry_msgs::Point target_position;
  current_service->get_target_position(target_position.x, target_position.y, target_position.z);
  move_arm_task(target_position);

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  if(task_result_ == false)
  {
    task_result_ = true;
    move_arm_joint("target_pose");

    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }
  }

  // manipulation :: open gripper
  open_gripper();
  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // manipulation : move to init pose
  move_arm_joint("home_with_object");
  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }


  // leave
  leave_target("leave_back");

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // turn to next target
  geometry_msgs::Pose start_pose;
  start_pose.position.x = 0;
  start_pose.position.y = 0;

  turn_to_target(start_pose);

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // nav to start position
  //  nav_to_target("nav_start");

  //  continue_result = sleep_for(sleep_ms, 0, is_running_sub_task_thread_, is_pause_, is_stop_);
  //  if(continue_result == false)
  //  {
  //    on_stop_task();
  //    return;
  //  }
  ROS_WARN("Nav to Start");
  nav_result = navigation("nav_start", false);

  // failed to navigation or received stop command
  if(nav_result == TASK::FAIL)
  {
    on_stop_task();
    return;
  }

  ROS_WARN_STREAM("Task " << current_service->get_name() << " is done.");

  is_running_task_thread_ = false;
}

// ========================================
// ==========     Thread
// ========================================
void TaskManager::on_stop_mission()
{
  ROS_WARN("Running mission is stopped.");
  is_pause_mission_ = false;
  is_stop_mission_ = false;

  is_running_mission_ = false;
}

void TaskManager::on_stop_task()
{
  ROS_WARN("Running task is stopped.");
  is_pause_ = false;
  is_stop_ = false;
  //  moving_thread_->interrupt();
  is_running_task_thread_ = false;
}

bool TaskManager::sleep_for(int sleep_interval, int after_interval, bool &running_condition, bool& pause_condition, bool& termination_condition)
{
  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_interval));

  //  ROS_WARN_STREAM("[START] thread id : " << boost::this_thread::get_id());

  while(running_condition || pause_condition)
  {
    if(termination_condition == true)
      return false;

    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_interval));
  }

  if(termination_condition == true)
    return false;

  boost::this_thread::sleep_for(boost::chrono::milliseconds(after_interval));
  return true;
}

void TaskManager::scenario_thread()
{
  is_running_task_thread_ = true;

  // laod scenario
  std::string scenario_path = ros::package::getPath(ROS_PACKAGE_NAME) + "/script/scenario.yaml";

  YAML::Node doc;
  int scenario_result;

  try
  {
    doc = YAML::LoadFile(scenario_path.c_str());

    YAML::Node start_node = doc["start"];
    std::string next_scenario = start_node["next_scenario"].as<std::string>();

    YAML::Node current_node = doc[next_scenario];
    int retry_times = current_node["retry_times"].as<int>();
    int play_times = 0;
    bool is_retry = false;

    while(next_scenario != "finish")
    {
      play_times += 1;
      is_retry = false;

      // run each task
      if(current_node != NULL)
      {
        scenario_result = handle_scenario(current_node);

        if(scenario_result == TASK::FAIL)
        {
          ROS_ERROR_STREAM("Fail to play a scenario : " << next_scenario << ", retry times : " << retry_times);
          if(retry_times != 0 && play_times <= retry_times)
            is_retry = true;
          else
            next_scenario = current_node["scenario_on_failure"].as<std::string>();
          //            break;
        }

        if(scenario_result == TASK::SUCCESS)
        {
          next_scenario = current_node["next_scenario"].as<std::string>();
        }

        if(scenario_result == TASK::STOP)
        {
          ROS_INFO("Scenario is stopped!");
          break;
        }
      }
      else
      {
        ROS_ERROR_STREAM("No scenario to play" << next_scenario);
        break;
      }

      // wait for 500 ms
      boost::this_thread::sleep_for(boost::chrono::milliseconds(SLEEP_MS * 5));

      // init
      if(is_retry == false)
      {
        current_node = doc[next_scenario];
        retry_times = current_node["retry_times"].as<int>();
        play_times  = 0;
      }
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Fail to load scenario. : " << e.what());
  }

  is_running_task_thread_ = false;
}

TASK::STATUS TaskManager::handle_scenario(const YAML::Node &current_scenario)
{
  task_result_ = true;
  TASK::STATUS scenario_status = TASK::SUCCESS;

  std::string task_name = current_scenario["task"].as<std::string>();

  if(task_name == "find_object")
  {
    std::vector<std::string> args = current_scenario["args"].as< std::vector<std::string> >();
    std::string room = args.at(0);
    std::string target_type = args.at(1);
    std::string marker_name;
    if(target_type == "object")
      marker_name = room_service_list_[room]->get_object();
    else if(target_type == "target")
      marker_name = room_service_list_[room]->get_target();

    int retry_times = current_scenario["retry_times"].as<int>();

    ROS_WARN_STREAM("Find Object : " << marker_name);
//    for(int ix = 0; ix < retry_times; ix ++)
//    {
      scenario_status = find_target(marker_name);
//      if(result == false)
//        status = SCENARIO::FAIL;
//        break;
//    }
  }
  else if(task_name == "navigation")
  {
    std::vector<std::string> args = current_scenario["args"].as< std::vector<std::string> >();
    std::string room = args.at(0);
    std::string target_type = args.at(1);
    std::string marker_name;

    std::string real_target = "";
    std::string real_target_room = args.at(2);
    std::string real_target_type = args.at(3);
    if(real_target_room != "none")
    {
      if(real_target_type == "object")
        real_target = room_service_list_[real_target_room]->get_object();
      else if(real_target_type == "target")
        real_target = room_service_list_[real_target_room]->get_target();
    }

    // use geometry_msgs::Pose
    if(target_type == "none")
    {
      if(room == "start")
      {
        ROS_WARN_STREAM("Nav to Start");
        scenario_status = navigation("nav_start", false);
      }
      else
      {
        geometry_msgs::Pose target_pose;
        bool result = room_service_list_[room]->get_room_center(target_pose.position.x, target_pose.position.y);

        if(result == false)
          return TASK::FAIL;

        // nav to room
        ROS_WARN_STREAM("Nav to Room : " << room);
        double yaw = atan2(target_pose.position.y, target_pose.position.x);
        get_quaternion(0, 0, yaw, target_pose.orientation);

        scenario_status = navigation(target_pose, real_target, false);
      }
    }
    // use ar marker
    else
    {
      if(target_type == "object")
        marker_name = room_service_list_[room]->get_object();
      else if(target_type == "target")
        marker_name = room_service_list_[room]->get_target();

      ROS_WARN_STREAM("Nav to : " << room << ", " << target_type);
      scenario_status = navigation(marker_name, real_target, false);
    }
  }
  else if(task_name == "approach")
  {
    std::vector<std::string> args = current_scenario["args"].as< std::vector<std::string> >();
    std::string room = args.at(0);
    std::string target_type = args.at(1);
    int repeat_number = stoi(args.at(2));
    std::string marker_name;

    if(target_type == "object")
      marker_name = room_service_list_[room]->get_object();
    else if(target_type == "target")
      marker_name = room_service_list_[room]->get_target();

    ROS_WARN_STREAM("Approach to Object : " << repeat_number);

    scenario_status = approach(marker_name, repeat_number);
  }
  else if(task_name == "leave")
  {
    std::vector<std::string> args = current_scenario["args"].as< std::vector<std::string> >();
    double distance = stod(args.at(0));

    ROS_WARN_STREAM("Leave Target : " << distance);

    scenario_status = leave(distance);
  }
  else if(task_name == "turn")
  {
    geometry_msgs::Pose room_pose;
    std::vector<std::string> args = current_scenario["args"].as< std::vector<std::string> >();
    std::string room = args.at(0);

    if(room != "start")
    {
      bool result = room_service_list_[room]->get_room_center(room_pose.position.x, room_pose.position.y);
      if(result == false)
        return TASK::FAIL;
    }
    ROS_WARN_STREAM("Turn to : " << room);

    scenario_status = turn(room_pose);
  }
  else if(task_name == "manipulation_task")
  {
    std::vector<std::string> args = current_scenario["args"].as< std::vector<std::string> >();
    std::string room = args.at(0);
    std::string target_type = args.at(1);
    geometry_msgs::Point target_position;
    bool result;

    if(target_type == "object")
      result = room_service_list_[room]->get_object_position(target_position.x, target_position.y, target_position.z);
    else if(target_type == "target")
      result = room_service_list_[room]->get_target_position(target_position.x, target_position.y, target_position.z);

    ROS_WARN_STREAM("Manipulation - Task : " << room << ", " << target_type);
    if(result == false)
      return TASK::FAIL;

    scenario_status = manipulation_task(target_position);
  }
  else if(task_name == "manipulation_task_offset")
  {
    std::vector<std::string> args = current_scenario["args"].as< std::vector<std::string> >();
    std::string room = args.at(0);
    std::string target_type = args.at(1);
    geometry_msgs::Point offset_position;
    geometry_msgs::Point target_position;
    offset_position.x = stod(args.at(2));
    offset_position.y = stod(args.at(3));
    offset_position.z = stod(args.at(4));
    bool result;

    if(target_type == "object")
      result = room_service_list_[room]->get_object_position(target_position.x, target_position.y, target_position.z);
    else if(target_type == "target")
      result = room_service_list_[room]->get_target_position(target_position.x, target_position.y, target_position.z);

    target_position.x = target_position.x + offset_position.x;
    target_position.y = target_position.y + offset_position.y;
    target_position.z = target_position.z + offset_position.z;

    ROS_WARN_STREAM("Manipulation - Task/Offset : " << room << ", " << target_type << " | "
                    << offset_position.x << ", " << offset_position.y << ", " << offset_position.z);

    if(result == false)
      return TASK::FAIL;

    scenario_status = manipulation_task(target_position);
  }
  else if(task_name == "manipulation_joint")
  {
    std::vector<std::string> args = current_scenario["args"].as< std::vector<std::string> >();
    std::string pose_name = args.at(0);

    ROS_WARN_STREAM("Manipulation - Joint : " << pose_name);

    scenario_status = manipulation_joint(pose_name);
  }
  else if(task_name == "gripper")
  {
    std::vector<std::string> args = current_scenario["args"].as< std::vector<std::string> >();
    std::string mode = args.at(0);

    if(mode == "open")
      gripper(true);
    else if(mode == "close")
      gripper(false);
  }
  else
  {
    ROS_ERROR_STREAM("There is no service for current task : " << task_name);
    task_result_ = false;
    scenario_status = TASK::FAIL;
  }

  return scenario_status;
}

}
