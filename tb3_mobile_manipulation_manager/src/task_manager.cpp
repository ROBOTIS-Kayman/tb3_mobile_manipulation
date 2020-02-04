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
TaskManager::TaskManager()
  : is_running_sub_task_thread_(false),
    is_stop_(false),
    is_pause_(false),
    interval_sleep_ms_(500),
    linear_vel_(0.05),
    angular_vel_(0.1),
    navigation_status_(actionlib_msgs::GoalStatus::PENDING)
{
  robot_name_ = "/tb3_manipulation";

  marker_name_list_.push_back("ar_marker_0");
  marker_name_list_.push_back("ar_marker_1");
  marker_name_list_.push_back("ar_marker_2");
  marker_name_list_.push_back("ar_marker_3");
  marker_name_list_.push_back("ar_marker_4");
  marker_name_list_.push_back("ar_marker_5");
  marker_name_list_.push_back("ar_marker_6");
  marker_name_list_.push_back("ar_marker_7");

  boost::thread queue_thread = boost::thread(boost::bind(&TaskManager::callback_thread, this));

  ros::NodeHandle p_nh("~");
  std::string task_data_path = p_nh.param<std::string>("task_data_path", ros::package::getPath(ROS_PACKAGE_NAME) + "/config/room.yaml");
  load_task_data(task_data_path);
  std::string config_data_path = p_nh.param<std::string>("config_data_path", ros::package::getPath(ROS_PACKAGE_NAME) + "/config/config.yaml");
  load_config(config_data_path);
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
  ROS_INFO("Ready to run the task");

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::ready_task_thread, this));
  delete moving_thread_;
}

void TaskManager::ready_task_thread()
{
  // reset odom
  publish_reset_turtlebot();

  // move arm to init pose
  move_arm_joint("home_with_object");

  sleep_for(100, 0, is_running_sub_task_thread_, is_pause_, is_stop_);

  open_gripper();
}

bool TaskManager::run_task(const std::string& task_name)
{
  Service *current_service;
  auto it = room_service_list_.find(task_name);
  if(it == room_service_list_.end())
  {
    ROS_ERROR_STREAM("No service for " << task_name);
    return false;
  }

  if(is_running_task_thread_ == true)
  {
    ROS_ERROR("Already running the task!");
    return false;
  }

  current_service = it->second;

  task_thread_ = new boost::thread(boost::bind(&TaskManager::run_task_thread, this, current_service));
  delete task_thread_;

  return true;
}

void TaskManager::run_task_thread(Service* current_service)
{
  is_running_task_thread_ = true;

  int sleep_ms = 100;
  int approach_repeat = 3;
  bool continue_result = true;

  ROS_WARN_STREAM("Start Task : " << current_service->get_name());

  // nav to object
  std::string object_name = current_service->get_object();
  std::string target_name = current_service->get_target();

  bool result = nav_to_target(object_name);

  if(result == false)
  {
    // go to start point
    nav_to_target("nav_start", object_name);

    continue_result = sleep_for(sleep_ms, 0, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }

    result = nav_to_target(object_name);
    if(result == false)
    {
      ROS_ERROR("Failed to find target");

      on_stop_task();
      return;
    }
  }

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // wait for detecting ar marker
  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

  for(int ix = 0; ix < approach_repeat; ix++)
  {
    // approach object
    approach_target(object_name, approach_repeat, ix + 1);

    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }

    if(ix == (approach_repeat - 1))
      break;

    // leave
    leave_target("leave_back");

    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }
  }

  // manipulation : pick
  geometry_msgs::Point object_position;
  current_service->get_object_position(object_position.x, object_position.y, object_position.z);
  move_arm_task(object_position);

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
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
  move_arm_joint("via_pose");
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

  // nav to room
  geometry_msgs::Pose room_pose;
  result = current_service->get_room_center(room_pose.position.x, room_pose.position.y);
  if(result == true)
  {
    double yaw = atan2(room_pose.position.y, room_pose.position.x);
    get_quaternion(0, 0, yaw, room_pose.orientation);
    //    room_pose.orientation.w = 1;
    //    room_pose.orientation.x = 0;
    //    room_pose.orientation.y = 0;
    //    room_pose.orientation.z = 0;

    nav_to_target(room_pose, target_name);


    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }
  }

  // find target
  look_around(target_name);

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // nav to target
  nav_to_target(target_name);

  continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  // wait for detecting ar marker
  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

  // approach target
  approach_repeat = 2;
  for(int ix = 0; ix < approach_repeat; ix++)
  {
    // approach object
    approach_target(target_name, approach_repeat, ix + 1);

    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }

    if(ix == (approach_repeat - 1))
      break;

    // leave
    leave_target("leave_back");

    continue_result = sleep_for(sleep_ms, sleep_ms * 10, is_running_sub_task_thread_, is_pause_, is_stop_);
    if(continue_result == false)
    {
      on_stop_task();
      return;
    }
  }

  // manipulation : move to via pose
  move_arm_joint("via_pose");
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

  // manipulation :: close gripper
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

  // nav to start position
  nav_to_target("nav_start");

  continue_result = sleep_for(sleep_ms, 0, is_running_sub_task_thread_, is_pause_, is_stop_);
  if(continue_result == false)
  {
    on_stop_task();
    return;
  }

  ROS_INFO_STREAM("Task " << current_service->get_name() << " is done.");

  is_running_task_thread_ = false;
}

// ========================================
// ==========     Thread
// ========================================
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

  while(running_condition || pause_condition)
  {
    if(termination_condition == true)
      return false;

    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_interval));
  }

  boost::this_thread::sleep_for(boost::chrono::milliseconds(after_interval));
  return true;
}

void TaskManager::callback_thread()
{
  // subscriber & publisher
  ros::NodeHandle nh;
  //  ros::NodeHandle priv_nh_;
  goal_nav_pub_ = nh.advertise<geometry_msgs::PoseStamped>(robot_name_ + "/move_base_simple/goal", 0);
  cancel_nav_pub_ = nh.advertise<actionlib_msgs::GoalID>(robot_name_ + "/move_base/cancel", 0);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(robot_name_ + "/cmd_vel", 0);
  debug_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>(robot_name_ + "/marker", 0);
  reset_turtlebot_pub_ = nh.advertise<std_msgs::Empty>(robot_name_ + "/reset", 0);

  cmd_sub_ = nh.subscribe(robot_name_ + "/command", 1, &TaskManager::command_msg_callback, this);
  navigation_result_sub_ = nh.subscribe(robot_name_ + "/move_base/status", 1, &TaskManager::navigation_result_callback, this);

  // service client
  goal_joint_space_path_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name_ + "/goal_joint_space_path");
  goal_task_space_path_position_only_client_ = nh.serviceClient<open_manipulator_msgs::SetKinematicsPose>(robot_name_ + "/goal_task_space_path_position_only");
  goal_tool_control_client_ = nh.serviceClient<open_manipulator_msgs::SetJointPosition>(robot_name_ + "/goal_tool_control");
  set_actuator_state_client_ = nh.serviceClient<open_manipulator_msgs::SetActuatorState>(robot_name_ + "/set_actuator_state");

  tf_listener_.reset( new tf::TransformListener());

  ros::Duration dur(0.01);

  while (nh.ok())
  {
    ros::spinOnce();

    dur.sleep();
  }
}


// ========================================
// ==========      Common
// ========================================
void TaskManager::get_quaternion(double roll, double pitch, double yaw, geometry_msgs::Quaternion& quaternion)
{
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  quaternion.w = cy * cp * cr + sy * sp * sr;
  quaternion.x = cy * cp * sr - sy * sp * cr;
  quaternion.y = sy * cp * sr + cy * sp * cr;
  quaternion.z = sy * cp * cr - cy * sp * sr;
}

void TaskManager::get_euler_angle(const geometry_msgs::Quaternion& quaternion, double& roll, double& pitch, double& yaw)
{
  // roll (x-axis rotation)
  double sinr_cosp = +2.0 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
  double cosr_cosp = +1.0 - 2.0 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
  roll = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = +2.0 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
  if (fabs(sinp) >= 1)
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
    pitch = asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = +2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
  double cosy_cosp = +1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
  yaw = atan2(siny_cosp, cosy_cosp);
}

bool TaskManager::get_target_pose(const std::string& target_name, geometry_msgs::Pose& target_pose)
{
  tf::StampedTransform desired_transform;

  std::string base_name = "/map";
  try
  {
    tf_listener_->lookupTransform(base_name, target_name, ros::Time(0), desired_transform);
    Eigen::Vector3d transform_position(desired_transform.getOrigin().x(),
                                       desired_transform.getOrigin().y(),
                                       desired_transform.getOrigin().z());
    Eigen::Quaterniond transform_orientation(desired_transform.getRotation().w(),
                                             desired_transform.getRotation().x(),
                                             desired_transform.getRotation().y(),
                                             desired_transform.getRotation().z());

    tf::pointEigenToMsg(transform_position, target_pose.position);
    tf::quaternionEigenToMsg(transform_orientation, target_pose.orientation);

    //    std::cout << object_name << " : " << transform_orientation.w() << ", " << transform_orientation.x() << ", " << transform_orientation.y() << ", " << transform_orientation.z() << std::endl;
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s",ex.what());
    return false;
  }

  return true;
}

void TaskManager::load_task_data(const std::string& path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());

    room_service_list_.clear();

    for (YAML::iterator yaml_it = doc.begin(); yaml_it != doc.end(); ++yaml_it)
    {
      std::string task_name;

      task_name = yaml_it->first.as<std::string>();

      YAML::Node sub_node = yaml_it->second;
      std::string name = sub_node["name"].as<std::string>();

      YAML::Node object_node = sub_node["object"];
      std::string object_name = object_node["marker"].as<std::string>();
      std::vector<double> object_position = object_node["position"].as< std::vector<double> >();

      YAML::Node target_node = sub_node["target"];
      std::string target_name = target_node["marker"].as<std::string>();
      std::vector<double> target_position = target_node["position"].as< std::vector<double> >();

      std::vector<double> room_x = sub_node["x"].as< std::vector<double> >();
      std::vector<double> room_y = sub_node["y"].as< std::vector<double> >();

      if(room_x.size() != 2 || room_y.size() != 2)
        return;

      Service *service = new Service(task_name);
      service->set_object(object_name);
      service->set_object_position(object_position);
      service->set_target(target_name);
      service->set_target_position(target_position);
      service->set_room_x(room_x.at(0), room_x.at(1));
      service->set_room_y(room_y.at(0), room_y.at(1));

      room_service_list_[task_name] = service;
    }
  }
  catch (const std::exception& e)
  {
    ROS_ERROR("Fail to load task data.");
    room_service_list_.clear();
    return;
  }
}

void TaskManager::load_config(const std::string &path)
{
  YAML::Node doc;
  try
  {
    doc = YAML::LoadFile(path.c_str());

    // arm pose
    YAML::Node arm_pose_node = doc["arm_pose"];
    arm_pose_list_.clear();

    for (YAML::iterator yaml_it = arm_pose_node.begin(); yaml_it != arm_pose_node.end(); ++yaml_it)
    {
      std::string pose_name;

      pose_name = yaml_it->first.as<std::string>();

      YAML::Node sub_node = yaml_it->second;

      std::map<std::string, double> pose;

      for (YAML::iterator sub_it = sub_node.begin(); sub_it != sub_node.end(); ++sub_it)
      {
        std::string joint_name = sub_it->first.as<std::string>();
        double joint_angle = sub_it->second.as<double>() * M_PI / 180.0;

        pose[joint_name] = joint_angle;
      }

      arm_pose_list_[pose_name] = pose;
    }

    // approach
    YAML::Node approach_node = doc["approach"];
    linear_vel_ = approach_node["linear_vel"].as<double>();
    angular_vel_ = approach_node["angular_vel"].as<double>();
    interval_sleep_ms_ = approach_node["interval_sleep_ms"].as<int>();

  }
  catch (const std::exception& e)
  {
    ROS_ERROR_STREAM("Fail to load config data." << e.what());
    arm_pose_list_.clear();
    return;
  }
}


// ========================================
// ==========     Mobile
// ========================================
void TaskManager::approach_target(const std::string &target_name)
{
  approach_target(target_name, 1, 1);
}

void TaskManager::approach_target(const std::string& target_name, int total_count, int present_count)
{
  // approach ar_marker
  std::size_t pos = target_name.find("ar_marker");
  if(pos != std::string::npos)
  {
    std::string marker_name = target_name.substr(pos);
    auto find_it = std::find(marker_name_list_.begin(), marker_name_list_.end(), marker_name);
    if(find_it != marker_name_list_.end())
    {
      geometry_msgs::Pose target_pose, present_pose;

      std::string base_frame_id = robot_name_ + "/base_footprint";

      bool result = get_target_pose(marker_name, target_pose) &&
          get_target_pose(base_frame_id, present_pose);
      if(result == false)
      {
        ROS_ERROR("Couldn't find the target or present footprint");
        return;
      }

      double final_offset = 0.1 + (total_count - present_count) * 0.05;

      Eigen::Vector3d offset(0, 0, final_offset);

      Eigen::Quaterniond target_orientation;
      Eigen::Vector3d object_position, target_position;
      Eigen::VectorXd global_offset;

      // target position
      tf::quaternionMsgToEigen(target_pose.orientation, target_orientation);
      tf::pointMsgToEigen(target_pose.position, object_position);
      global_offset = target_orientation.toRotationMatrix() * offset;
      global_offset.coeffRef(2) = 0.0;
      target_position = object_position + global_offset;

      tf::pointEigenToMsg(target_position, target_pose.position);

      // target orientation : global yaw
      double target_yaw = atan2(-global_offset.coeff(1), -global_offset.coeff(0));

      geometry_msgs::Pose2D present_pose_2d, target_pose_2d;
      present_pose_2d.x = present_pose.position.x;
      present_pose_2d.y = present_pose.position.y;
      double p_roll, p_pitch, p_yaw;
      get_euler_angle(present_pose.orientation, p_roll, p_pitch, p_yaw);
      present_pose_2d.theta = p_yaw;

      target_pose_2d.x = target_pose.position.x;
      target_pose_2d.y = target_pose.position.y;
      target_pose_2d.theta = target_yaw;

      //publish start and target
      approach_pose_list_.clear();
      approach_pose_list_.push_back(present_pose_2d);
      approach_pose_list_.push_back(target_pose_2d);
      publish_marker(false, approach_pose_list_);

      moving_thread_ = new boost::thread(boost::bind(&TaskManager::approach_target_thread, this, present_pose_2d, target_pose_2d));
      delete moving_thread_;
    }
  }
}

void TaskManager::approach_target_thread(const geometry_msgs::Pose2D& present_pose, const geometry_msgs::Pose2D& target_pose)
{
  is_running_sub_task_thread_ = true;
  int additional_sleep_ms = 0;

  ROS_WARN_STREAM("present : " << present_pose.x << ", " << present_pose.y << " | " << present_pose.theta);
  ROS_WARN_STREAM("target : " << target_pose.x << ", " << target_pose.y << " | " << target_pose.theta);

  // LINEAR_MAX_VELOCITY = (WHEEL_RADIUS * 2 * M_PI * WAFFLE / 60)       #m/s  (WHEEL_RADIUS = 0.033, BURGER : 61[rpm], WAFFLE : 77[rpm])
  // ANGULAR_MAX_VELOCITY = (MAX_LINEAR_VELOCITY / WAFFLE_TURNING_RADIUS)   #rad/s (WAFFLE_TURNING_RADIUS = 0.1435)

  double diff_x = target_pose.x - present_pose.x;
  double diff_y = target_pose.y - present_pose.y;

  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);
  double yaw_1 = atan2(diff_y, diff_x) - present_pose.theta;
  double yaw_2 = target_pose.theta - atan2(diff_y, diff_x);

  ROS_INFO_STREAM("yaw_1 : " << (yaw_1 * 180 / M_PI) << ", distance : " << distance << ", yaw_2 : " << (yaw_2 * 180 / M_PI));

  // turn to yaw_1
  geometry_msgs::Twist approach_msg;
  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = (yaw_1 > 0) ? angular_vel_ : - angular_vel_;
  publish_cmd_vel_msg(approach_msg);

  // wait to turn
  int moving_time = yaw_1 * 1000 / approach_msg.angular.z;
  boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time + additional_sleep_ms));
  ROS_WARN_STREAM("turn 1 : " << (yaw_1 * 180 / M_PI) << ", time(ms) : " << moving_time);

  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(interval_sleep_ms_));

  // go to target
  approach_msg.linear.x = linear_vel_;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  // wait to turn
  moving_time = distance * 1000 / approach_msg.linear.x;
  boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time + additional_sleep_ms));
  ROS_WARN_STREAM("go straight : " << distance << ", time(ms) : " << moving_time);

  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(interval_sleep_ms_));

  // turn to target theta
  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = (yaw_2 > 0) ? angular_vel_ : - angular_vel_;
  publish_cmd_vel_msg(approach_msg);

  // wait to turn
  moving_time = yaw_2 * 1000 / approach_msg.angular.z;
  boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time + additional_sleep_ms));
  ROS_WARN_STREAM("turn 2 : " << (yaw_2 * 180 / M_PI) << ", time(ms) : " << moving_time);

  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  // debug
  //  boost::this_thread::sleep_for(boost::chrono::milliseconds(interval_sleep_ms));
  //  std::string base_frame_id = "tb3_mobile_manipulation/base_footprint";
  //  geometry_msgs::Pose target;
  //  get_target_pose(base_frame_id, target);
  //  double r,p,y;
  //  get_euler_angle(target.orientation, r, p, y);
  //  ROS_WARN_STREAM("target from TF : " << target.position.x << ", " << target.position.y << ", theta : " << y);

  is_running_sub_task_thread_ = false;
}

void TaskManager::leave_target(const std::string& command)
{
  if(command.find("back") != std::string::npos)
  {
    publish_marker(true, approach_pose_list_);

    geometry_msgs::Pose2D pose_1, pose_2;
    pose_2.x = -0.2;

    moving_thread_ = new boost::thread(boost::bind(&TaskManager::leave_target_thread, this, pose_1, pose_2));
    delete moving_thread_;
  }
  else
  {
    if(approach_pose_list_.size() != 2)
    {
      ROS_ERROR("No approach data!!!");
      return;
    }

    // clear marker
    //  std::vector<geometry_msgs::Pose2D> pose_list;
    publish_marker(true, approach_pose_list_);

    //  double distance = 0.3;
    //  approach_thread_ = new boost::thread(boost::bind(&TaskManager::leave_target_thread, this, distance));
    moving_thread_ = new boost::thread(boost::bind(&TaskManager::leave_target_thread, this, approach_pose_list_.at(1), approach_pose_list_.at(0)));
    delete moving_thread_;
  }
}

void TaskManager::leave_target_thread(const geometry_msgs::Pose2D &present_pose, const geometry_msgs::Pose2D& target_pose)
{
  is_running_sub_task_thread_ = true;

  int interval_sleep_ms = 500;

  double linear_vel = -0.1;
  double angular_vel = 0.1;

  double diff_x = present_pose.x - target_pose.x;
  double diff_y = present_pose.y - target_pose.y;

  double distance = sqrt(diff_x * diff_x + diff_y * diff_y);
  double yaw_1 = atan2(diff_y, diff_x) - present_pose.theta;
  double yaw_2 = target_pose.theta - atan2(diff_y, diff_x);

  // turn to yaw_1
  geometry_msgs::Twist approach_msg;
  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = (yaw_1 > 0) ? angular_vel : - angular_vel;
  publish_cmd_vel_msg(approach_msg);

  // wait to turn
  int moving_time = yaw_1 * 1000 / approach_msg.angular.z;
  boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time));
  //  ROS_WARN_STREAM("turn 1 : " << (yaw_1 * 180 / M_PI) << ", time(ms) : " << moving_time);

  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(interval_sleep_ms));

  // go to target
  approach_msg.linear.x = linear_vel;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  // wait to turn
  moving_time = - distance * 1000 / approach_msg.linear.x;
  boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time));
  //  ROS_WARN_STREAM("go straight : " << distance << ", time(ms) : " << moving_time);

  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(interval_sleep_ms));

  // turn to target theta
  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = (yaw_2 > 0) ? angular_vel : - angular_vel;
  publish_cmd_vel_msg(approach_msg);

  // wait to turn
  moving_time = yaw_2 * 1000 / approach_msg.angular.z;
  boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time));
  //  ROS_WARN_STREAM("turn 2 : " << (yaw_2 * 180 / M_PI) << ", time(ms) : " << moving_time);

  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  is_running_sub_task_thread_ = false;
}

void TaskManager::cancel_nav()
{
  actionlib_msgs::GoalID cancel_msg;
  cancel_msg.id = navigation_goal_id_;

  cancel_nav_pub_.publish(cancel_msg);

  ROS_INFO("Nav goal is canceled.");
}

bool TaskManager::nav_to_target(const std::string& target_name)
{
  return nav_to_target(target_name, "");
}

bool TaskManager::nav_to_target(const std::string& target_name, const std::string& real_target)
{
  // go ar_marker
  geometry_msgs::Pose *target_pose = nullptr;

  std::size_t pos = target_name.find("ar_marker");
  if(pos != std::string::npos)
  {
    std::string marker_name = target_name.substr(pos);
    auto find_it = std::find(marker_name_list_.begin(), marker_name_list_.end(), marker_name);
    if(find_it != marker_name_list_.end())
    {
      target_pose = new geometry_msgs::Pose;

      bool result = get_target_pose(marker_name, *target_pose);
      if(result == false)
      {
        ROS_WARN("Failed to find correct ar marker, It will go to start point.");

        target_pose = nullptr;

        return false;
      }

      Eigen::Vector3d offset(0, 0, 0.35);

      Eigen::Quaterniond target_orientation;
      Eigen::Vector3d object_position, target_position;
      Eigen::VectorXd global_offset;

      // target position
      tf::quaternionMsgToEigen(target_pose->orientation, target_orientation);
      tf::pointMsgToEigen(target_pose->position, object_position);
      global_offset = target_orientation.toRotationMatrix() * offset;
      global_offset.coeffRef(2) = 0.0;
      target_position = object_position + global_offset;

      tf::pointEigenToMsg(target_position, target_pose->position);

      // target orientation : global yaw
      double yaw = atan2(-global_offset.coeff(1), -global_offset.coeff(0));

      get_quaternion(0.0, 0.0, yaw, target_pose->orientation);

      //      nav_to_target_thread(*target_pose);
    }
  }

  // test code
  if(target_name == "nav_1")
  {
    target_pose = new geometry_msgs::Pose;

    target_pose->position.x = 0.8;
    target_pose->position.y = 0.39;
    target_pose->position.z = 0;

    target_pose->orientation.w = 1;
    target_pose->orientation.x = 0;
    target_pose->orientation.y = 0;
    target_pose->orientation.z = 0;

    //    publish_goal_nav_msg(nav_msg);
  }

  if(target_name == "nav_1_goal")
  {
    target_pose = new geometry_msgs::Pose;

    target_pose->position.x = 0.95;
    target_pose->position.y = 0.95;
    target_pose->position.z = 0;

    geometry_msgs::Quaternion orientation;
    get_quaternion(0, 0, M_PI * 0.25, orientation);
    target_pose->orientation = orientation;

    //    publish_goal_nav_msg(nav_msg);
  }

  if(target_name == "nav_2")
  {
    target_pose = new geometry_msgs::Pose;

    target_pose->position.x = 0.8;
    target_pose->position.y = 0.13;
    target_pose->position.z = 0;

    target_pose->orientation.w = 1;
    target_pose->orientation.x = 0;
    target_pose->orientation.y = 0;
    target_pose->orientation.z = 0;

    //    publish_goal_nav_msg(nav_msg);
  }

  if(target_name == "nav_2_goal")
  {
    target_pose = new geometry_msgs::Pose;

    target_pose->position.x = 0.95;
    target_pose->position.y = -0.95;
    target_pose->position.z = 0;

    geometry_msgs::Quaternion orientation;
    get_quaternion(0, 0, -M_PI * 0.25, orientation);
    target_pose->orientation = orientation;

    //    publish_goal_nav_msg(nav_msg);
  }

  if(target_name == "nav_start")
  {
    target_pose = new geometry_msgs::Pose;

    target_pose->position.x = 0;
    target_pose->position.y = 0;
    target_pose->position.z = 0;

    target_pose->orientation.w = 1;
    target_pose->orientation.x = 0;
    target_pose->orientation.y = 0;
    target_pose->orientation.z = 0;
  }

  if(target_pose != nullptr)
  {
    moving_thread_ = new boost::thread(boost::bind(&TaskManager::nav_to_target_thread, this, *target_pose, real_target));
    delete moving_thread_;
  }

  return true;
}

bool TaskManager::nav_to_target(const geometry_msgs::Pose &target_pose)
{
  return nav_to_target(target_pose, "");
}

bool TaskManager::nav_to_target(const geometry_msgs::Pose &target_pose, const std::string& real_target)
{
  moving_thread_ = new boost::thread(boost::bind(&TaskManager::nav_to_target_thread, this, target_pose, real_target));
  delete moving_thread_;

  return true;
}

void TaskManager::nav_to_target_thread(const geometry_msgs::Pose& target_pose, const std::string& real_target)
{
  is_running_sub_task_thread_ = true;

  int sleep_ms = 100;

  geometry_msgs::PoseStamped nav_msg;
  nav_msg.header.stamp = ros::Time::now();
  nav_msg.header.frame_id = "map";
  nav_msg.pose = target_pose;

  publish_goal_nav_msg(nav_msg);

  // wait for accept
  //  ros::Duration dur(0.1);
  while(navigation_status_ != move_base_msgs::MoveBaseActionResult::_status_type::ACTIVE)
    //    dur.sleep();
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));

  ROS_INFO_STREAM("Navigation message is accepted. : " << navigation_status_);

  // wait for finishing navigation
  geometry_msgs::Pose real_target_pose;
  while(navigation_status_ == move_base_msgs::MoveBaseActionResult::_status_type::ACTIVE)
  {
    if(real_target != "")
    {
      bool result = get_target_pose(real_target, real_target_pose);
      if(result == true)
      {
        cancel_nav();
        break;
      }
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
  }

  ROS_INFO_STREAM("Navigation is finished. : " << navigation_status_);

  is_running_sub_task_thread_ = false;
}

void TaskManager::look_around(const std::string& target_name)
{
  std::size_t pos = target_name.find("ar_marker");
  if(pos != std::string::npos)
  {
    std::string marker_name = target_name.substr(pos);
    auto find_it = std::find(marker_name_list_.begin(), marker_name_list_.end(), marker_name);
    if(find_it != marker_name_list_.end())
    {
      moving_thread_ = new boost::thread(boost::bind(&TaskManager::look_around_thread, this, 1, marker_name));
      delete moving_thread_;
    }
  }
}

void TaskManager::look_around_thread(int direction, const std::string& target_name)
{
  if(target_name == "")
  {
    ROS_ERROR("No target name");
    return;
  }

  is_running_sub_task_thread_ = true;

  double linear_vel = 0.0;
  double angular_vel = 0.5;

  // turn around
  geometry_msgs::Twist approach_msg;
  approach_msg.linear.x = linear_vel;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = angular_vel * direction;
  publish_cmd_vel_msg(approach_msg);

  // check to find the target object
  geometry_msgs::Pose target_pose;

  int moving_time = 2 * M_PI * 1000 / angular_vel;
  int sleep_time = 200; //ms
  int total_moving_index = moving_time / sleep_time;

  for(int ix = 0; ix < total_moving_index; ix++)
  {
    if(get_target_pose(target_name, target_pose))
    {
      ROS_INFO("Success to find the target object");
      break;
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_time));
  }

  // stop moving
  approach_msg.linear.x = 0.0;
  approach_msg.linear.y = 0.0;
  approach_msg.linear.z = 0.0;
  approach_msg.angular.x = 0.0;
  approach_msg.angular.y = 0.0;
  approach_msg.angular.z = 0.0;
  publish_cmd_vel_msg(approach_msg);

  is_running_sub_task_thread_ = false;
}

void TaskManager::compensation_localization()
{

}

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
      std::cout << "joint : " << element.first << ", angle : " << element.second << std::endl;
    }

    if((joint_name.size() != 0) && (joint_name.size() == joint_angle.size()))
    {
      ROS_INFO_STREAM("move arm : " << target_pose);
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

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::move_gripper_thread, this, gripper_position));
  delete moving_thread_;
}

void TaskManager::close_gripper()
{
  double gripper_position = -0.013;

  moving_thread_ = new boost::thread(boost::bind(&TaskManager::move_gripper_thread, this, gripper_position));
  delete moving_thread_;
}

void TaskManager::move_gripper_thread(double gripper_position)
{
  is_running_sub_task_thread_ = true;
  bool result;

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

  is_running_sub_task_thread_ = false;
}

void TaskManager::publish_goal_nav_msg(const geometry_msgs::PoseStamped& goal_msg)
{
  ROS_INFO_STREAM("Publish Nav msg");
  goal_nav_pub_.publish(goal_msg);
}

void TaskManager::publish_cmd_vel_msg(const geometry_msgs::Twist& msg)
{
  ROS_INFO_STREAM("Publish Cmd_vel msg : " << msg.linear.x << ", " << msg.angular.z);
  cmd_vel_pub_.publish(msg);
}

void TaskManager::publish_marker(bool clear, const std::vector<geometry_msgs::Pose2D>& pose_list)
{
  visualization_msgs::MarkerArray marker_array;
  ros::Time now = ros::Time::now();
  visualization_msgs::Marker arrow_marker;
  visualization_msgs::Marker cube_marker;

  arrow_marker.header.frame_id = "map";
  arrow_marker.header.stamp = now;
  arrow_marker.ns = "approach_marker";
  cube_marker = arrow_marker;

  arrow_marker.id = 0;
  cube_marker.id = 100;
  //  arrow_marker.type = visualization_msgs::Marker::CUBE;
  arrow_marker.type = visualization_msgs::Marker::ARROW;
  cube_marker.type = visualization_msgs::Marker::CUBE;
  arrow_marker.action = (clear == false) ? visualization_msgs::Marker::ADD : visualization_msgs::Marker::DELETEALL;
  cube_marker.action = arrow_marker.action;

  if(clear == true)
  {
    marker_array.markers.push_back(arrow_marker);
    debug_marker_pub_.publish(marker_array);
    return;
  }

  arrow_marker.scale.x = 0.15;
  arrow_marker.scale.y = 0.015;
  arrow_marker.scale.z = 0.015;

  arrow_marker.color.r = 0.0;
  arrow_marker.color.g = 1.0;
  arrow_marker.color.b = 0.0;
  arrow_marker.color.a = 1.0;

  cube_marker.scale.x = 0.03;
  cube_marker.scale.y = 0.03;
  cube_marker.scale.z = 0.03;

  cube_marker.color.r = 0.0;
  cube_marker.color.g = 1.0;
  cube_marker.color.b = 1.0;
  cube_marker.color.a = 1.0;

  for(auto it = pose_list.begin(); it != pose_list.end(); ++it)
  {
    arrow_marker.pose.position.x = it->x;
    arrow_marker.pose.position.y = it->y;
    arrow_marker.pose.position.z = 0.15;

    get_quaternion(0.0, 0.0, it->theta, arrow_marker.pose.orientation);
    marker_array.markers.push_back(arrow_marker);
    cube_marker.pose = arrow_marker.pose;
    marker_array.markers.push_back(cube_marker);

    arrow_marker.id++;
    cube_marker.id++;
  }

  debug_marker_pub_.publish(marker_array);
}

void TaskManager::publish_reset_turtlebot()
{
  std_msgs::Empty msg;

  reset_turtlebot_pub_.publish(msg);
  ROS_INFO("RESET Turtlebot!!");
}

void TaskManager::command_msg_callback(const std_msgs::String::ConstPtr& msg)
{
  // check command for controlling the task
  COMMAND current_command = NONE;
  if(msg->data == "stop")
    current_command = STOP;
  else if(msg->data == "pause")
    current_command = PAUSE;
  else if(msg->data == "resume")
    current_command = RESUME;
  else if(msg->data == "ready")
    current_command = READY;

  if(current_command != NONE)
  {
    control_task(current_command);
    return;
  }

  // task command
  if(is_running_task_thread_ == true)
  {
    ROS_WARN("Task thread is running, command is ignored.");

    return;
  }
  else
    run_task(msg->data);

  // nav command for testing
  if(is_running_sub_task_thread_ == true)
  {
    ROS_WARN("Moving thread is running, command is ignored.");

    return;
  }
  else
  {
    // navigation
    if(msg->data.find("nav") != std::string::npos)
      nav_to_target(msg->data);

    // approach
    else if(msg->data.find("approach") != std::string::npos)
      approach_target(msg->data);

    else if(msg->data.find("leave") != std::string::npos)
      leave_target(msg->data);

    else if(msg->data.find("find") != std::string::npos)
      look_around(msg->data);
  }
}

void TaskManager::navigation_result_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  if(msg->status_list.size() == 0)
    return;

  navigation_goal_id_ = msg->status_list.rbegin()->goal_id.id;
  navigation_status_ = msg->status_list.rbegin()->status;
  //  std::cout << "navigation : " << navigation_status_ << std::endl;
  //  navigation_status_ = msg->status.status;
}

}
