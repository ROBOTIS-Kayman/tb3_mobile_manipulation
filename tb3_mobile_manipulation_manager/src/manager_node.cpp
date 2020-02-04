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

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "tb3_mobile_manipulation_manager/task_manager.h"

// void buttonHandlerCallback(const std_msgs::String::ConstPtr& msg);
// void goInitPose();
// void playSound(const std::string &path);
// void setLED(int led);
// bool checkManagerRunning(std::string& manager_name);
// void dxlTorqueChecker();

// void demoModeCommandCallback(const std_msgs::String::ConstPtr &msg);

const int SPIN_RATE = 30;
// const bool DEBUG_PRINT = false;

// ros::Publisher init_pose_pub;
// ros::Publisher play_sound_pub;
// ros::Publisher led_pub;
// ros::Publisher dxl_torque_pub;

// std::string default_mp3_path = "";
// int current_status = Ready;
// int desired_status = Ready;
// bool apply_desired = false;

//node main
int main(int argc, char **argv)
{
  //init ros
  ros::init(argc, argv, "tb3_mobile_manipulation_manager");

  //create ros wrapper object
  // robotis_op::OPDemo *current_demo = NULL;
  // robotis_op::SoccerDemo *soccer_demo = new robotis_op::SoccerDemo();
  // robotis_op::ActionDemo *action_demo = new robotis_op::ActionDemo();
  // robotis_op::VisionDemo *vision_demo = new robotis_op::VisionDemo();

  ros::NodeHandle nh(ros::this_node::getName());

  // init_pose_pub = nh.advertise<std_msgs::String>("/robotis/base/ini_pose", 0);
  // play_sound_pub = nh.advertise<std_msgs::String>("/play_sound_file", 0);
  // led_pub = nh.advertise<robotis_controller_msgs::SyncWriteItem>("/robotis/sync_write_item", 0);
  // dxl_torque_pub = nh.advertise<std_msgs::String>("/robotis/dxl_torque", 0);
  // ros::Subscriber buttuon_sub = nh.subscribe("/robotis/open_cr/button", 1, buttonHandlerCallback);
  // ros::Subscriber mode_command_sub = nh.subscribe("/robotis/mode_command", 1, demoModeCommandCallback);

  // default_mp3_path = ros::package::getPath("op3_demo") + "/data/mp3/";

  ros::start();

  //set node loop rate
  ros::Rate loop_rate(SPIN_RATE);

  tb3_mobile_manipulation::TaskManager *task_manager = new tb3_mobile_manipulation::TaskManager();

  // wait for starting of manager
  // std::string manager_name = "/op3_manager";
  // while (ros::ok())
  // {
    // ros::Duration(1.0).sleep();

    // if (checkManagerRunning(manager_name) == true)
    // {
      // break;
      // ROS_INFO_COND(DEBUG_PRINT, "Succeed to connect");
    // }
    // ROS_WARN("Waiting for op3 manager");
  // }

  // init procedure
  // playSound(default_mp3_path + "Demonstration ready mode.mp3");
  // turn on R/G/B LED
  // setLED(0x01 | 0x02 | 0x04);

  ROS_INFO("Start task manager!");

  task_manager->ready_task();

  //node loop
  while (ros::ok())
  {
    //execute pending callbacks
    ros::spinOnce();

    //relax to fit output rate
    loop_rate.sleep();
  }

  //exit program
  return 0;
}

