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
  : is_running_thread_(false),
    navigation_status_(actionlib_msgs::GoalStatus::PENDING)
{
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

  current_service = it->second;

  task_thread_ = new boost::thread(boost::bind(&TaskManager::run_task_thread, this, current_service));
  delete task_thread_;

  return true;
}

void TaskManager::run_task_thread(Service* current_service)
{
  int sleep_ms = 100;
  int approach_repeat = 3;

  ROS_WARN_STREAM("Start Task : " << current_service->get_name());

  // nav to object
  std::string object_name = current_service->get_object();
  std::string target_name = current_service->get_target();

  nav_to_target(object_name);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
  while(is_running_thread_)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

  for(int ix = 0; ix < approach_repeat; ix++)
  {
    // approach object
    approach_target(object_name);

    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
    while(is_running_thread_)
      boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));

    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

    // leave
    leave_target("leave_back");

    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
    while(is_running_thread_)
      boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));

    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));
  }

  // nav to room
  geometry_msgs::Pose room_pose;
  bool result = current_service->get_room_center(room_pose.position.x, room_pose.position.y);
  if(result == true)
  {
    double yaw = atan2(room_pose.position.y, room_pose.position.x);
    get_quaternion(0, 0, yaw, room_pose.orientation);
    //    room_pose.orientation.w = 1;
    //    room_pose.orientation.x = 0;
    //    room_pose.orientation.y = 0;
    //    room_pose.orientation.z = 0;

    nav_to_target(room_pose, target_name);


    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
    while(is_running_thread_)
      boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));

    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));
  }

  // find target
  look_around(target_name);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
  while(is_running_thread_)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

  // nav to target
  nav_to_target(target_name);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
  while(is_running_thread_)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

  // approach target
  approach_target(target_name);

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
  while(is_running_thread_)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

  // leave
  leave_target("leave_back");

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
  while(is_running_thread_)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms * 10));

  // nav to start position
  nav_to_target("nav_start");

  boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
  while(is_running_thread_)
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));

  ROS_INFO_STREAM("Task " << current_service->get_name() << " is done.");
}

void TaskManager::callback_thread()
{
  // subscriber & publisher
  ros::NodeHandle nh;
  //  ros::NodeHandle priv_nh_;
  goal_nav_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/tb3_mobile_manipulation/move_base_simple/goal", 0);
  cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/tb3_mobile_manipulation/cmd_vel", 0);
  debug_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/tb3_mobile_manipulation/marker", 0);

  cmd_sub_ = nh.subscribe("/tb3_mobile_manipulation/command", 1, &TaskManager::command_msg_callback, this);
  navigation_result_sub_ = nh.subscribe("/tb3_mobile_manipulation/move_base/status", 1, &TaskManager::navigation_result_callback, this);

  tf_listener_.reset( new tf::TransformListener());

  ros::Duration dur(0.01);

  while (nh.ok())
  {
    ros::spinOnce();

    dur.sleep();
  }
}

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

    YAML::Node id_sub_node = doc["id_joint"];
    for (YAML::iterator yaml_it = doc.begin(); yaml_it != doc.end(); ++yaml_it)
    {
      std::string task_name;

      task_name = yaml_it->first.as<std::string>();

      YAML::Node sub_node = yaml_it->second;
      std::string name = sub_node["name"].as<std::string>();
      std::string object_name = sub_node["object"].as<std::string>();
      std::string target_name = sub_node["target"].as<std::string>();
      std::vector<double> room_x = sub_node["x"].as< std::vector<double> >();
      std::vector<double> room_y = sub_node["y"].as< std::vector<double> >();

      if(room_x.size() != 2 || room_y.size() != 2)
        return;

      Service *service = new Service(task_name);
      service->set_object(object_name);
      service->set_target(target_name);
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

void TaskManager::approach_target(const std::string& target_name)
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

      std::string base_frame_id = "tb3_mobile_manipulation/base_footprint";

      bool result = get_target_pose(marker_name, target_pose) &&
          get_target_pose(base_frame_id, present_pose);
      if(result == false)
      {
        ROS_ERROR("Couldn't find the target or present footprint");
        return;
      }

      Eigen::Vector3d offset(0, 0, 0.10);

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
  is_running_thread_ = true;
  int additional_sleep_ms = 0;
  int interval_sleep_ms = 1000;

  ROS_WARN_STREAM("present : " << present_pose.x << ", " << present_pose.y << " | " << present_pose.theta);
  ROS_WARN_STREAM("target : " << target_pose.x << ", " << target_pose.y << " | " << target_pose.theta);

  // LINEAR_MAX_VELOCITY = (WHEEL_RADIUS * 2 * M_PI * WAFFLE / 60)       #m/s  (WHEEL_RADIUS = 0.033, BURGER : 61[rpm], WAFFLE : 77[rpm])
  // ANGULAR_MAX_VELOCITY = (MAX_LINEAR_VELOCITY / WAFFLE_TURNING_RADIUS)   #rad/s (WAFFLE_TURNING_RADIUS = 0.1435)
  double linear_vel = 0.05; //0.1;
  double angular_vel = 0.1; //0.1;

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
  approach_msg.angular.z = (yaw_1 > 0) ? angular_vel : - angular_vel;
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

  is_running_thread_ = false;
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
  is_running_thread_ = true;

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

  is_running_thread_ = false;

  //  // leave target
  //  geometry_msgs::Twist approach_msg;
  //  approach_msg.linear.x = - 0.1;
  //  approach_msg.linear.y = 0.0;
  //  approach_msg.linear.z = 0.0;
  //  approach_msg.angular.x = 0.0;
  //  approach_msg.angular.y = 0.0;
  //  approach_msg.angular.z = 0.0;
  //  publish_cmd_vel_msg(approach_msg);

  //  // wait to turn
  //  int moving_time = distance * 1000 / 0.1;
  //  boost::this_thread::sleep_for(boost::chrono::milliseconds(moving_time));
  //  ROS_WARN_STREAM("go back : " << distance << ", time(ms) : " << moving_time);

  //  approach_msg.linear.x = 0.0;
  //  approach_msg.linear.y = 0.0;
  //  approach_msg.linear.z = 0.0;
  //  approach_msg.angular.x = 0.0;
  //  approach_msg.angular.y = 0.0;
  //  approach_msg.angular.z = 0.0;
  //  publish_cmd_vel_msg(approach_msg);
}

void TaskManager::nav_to_target(const std::string& target_name)
{
  nav_to_target(target_name, "");
}

void TaskManager::nav_to_target(const std::string& target_name, const std::string& real_target)
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
        //        target_pose->position.x = 0;
        //        target_pose->position.y = 0;
        //        target_pose->position.z = 0;

        //        target_pose->orientation.w = 1;
        //        target_pose->orientation.x = 0;
        //        target_pose->orientation.y = 0;
        //        target_pose->orientation.z = 0;

        //        nav_to_target_thread(*target_pose);
        return;
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
}

void TaskManager::nav_to_target(const geometry_msgs::Pose &target_pose)
{
  nav_to_target(target_pose, "");
}

void TaskManager::nav_to_target(const geometry_msgs::Pose &target_pose, const std::string& real_target)
{
  moving_thread_ = new boost::thread(boost::bind(&TaskManager::nav_to_target_thread, this, target_pose, real_target));
  delete moving_thread_;
}

void TaskManager::nav_to_target_thread(const geometry_msgs::Pose& target_pose, const std::string& real_target)
{
  is_running_thread_ = true;

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
        break;
    }
    boost::this_thread::sleep_for(boost::chrono::milliseconds(sleep_ms));
  }

  ROS_INFO_STREAM("Navigation is finished. : " << navigation_status_);

  is_running_thread_ = false;
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

  is_running_thread_ = true;

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

  is_running_thread_ = false;
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

void TaskManager::command_msg_callback(const std_msgs::String::ConstPtr& msg)
{
  if(is_running_thread_ == true)
  {
    ROS_WARN("Moving thread is running, command is ignored.");
    return;
  }

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

  else
    run_task(msg->data);
}

void TaskManager::navigation_result_callback(const actionlib_msgs::GoalStatusArray::ConstPtr& msg)
{
  if(msg->status_list.size() == 0)
    return;

  navigation_status_ = msg->status_list.rbegin()->status;
  //  std::cout << "navigation : " << navigation_status_ << std::endl;
  //  navigation_status_ = msg->status.status;
}

}
