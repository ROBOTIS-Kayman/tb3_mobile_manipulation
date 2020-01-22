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

#include "tb3_mobile_manipulation_manager/service.h"

namespace tb3_mobile_manipulation
{

Service::Service(std::string name)
  : name_(name)
{

}

void Service::set_object(const std::string& object_marker)
{
  object_marker_ = object_marker;
}

void Service::set_target(const std::string& target_marker)
{
  target_marker_ = target_marker;
}

void Service::set_room_x(double x_max, double x_min)
{
  room_x_.clear();

  room_x_.push_back(x_max);
  room_x_.push_back(x_min);
}

void Service::set_room_y(double y_max, double y_min)
{
  room_y_.clear();

  room_y_.push_back(y_max);
  room_y_.push_back(y_min);
}

bool Service::get_room_center(double& x, double&y)
{
  if(room_x_.size() != 2 || room_y_.size() != 2)
    return false;

  x = (room_x_.at(0) + room_x_.at(1)) * 0.5;
  y = (room_y_.at(0) + room_y_.at(1)) * 0.5;

  return true;
}
}
