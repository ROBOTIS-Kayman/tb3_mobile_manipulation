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

#define TB3_MOBILE_MANIPULATION_MOVEIT

#ifdef TB3_MOBILE_MANIPULATION_MOVEIT
#include "tb3_mobile_manipulation_manager/task_manager_moveit.h"
#else
#include "tb3_mobile_manipulation_manager/task_manager_open_manipulator.h"
#endif



#endif // TB3_MOBILE_MANIPULATION_TASK_MANAGER_H
