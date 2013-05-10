/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Julian Cerruti */

#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <shape_tools/solid_primitive_dims.h>
#include <moveit/robot_state/joint_state_group.h>

static const std::string ROBOT_DESCRIPTION="robot_description";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "creativa_tutorial", ros::init_options::AnonymousName);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::cout << "Starting process" << std::endl;

  move_group_interface::MoveGroup arm("right_arm");
  arm.setPlanningTime(45.0);

  arm.setStartStateToCurrentState();
  geometry_msgs::PoseStamped current_pose = arm.getCurrentPose();
  std::cout << "Arm currently at " << current_pose << std::endl;

  move_group_interface::MoveGroup::Plan plan_result;
  for(int i=0;i<10;i++) {
    current_pose = arm.getRandomPose();
    std::cout << "How about using this random pose? " << current_pose << std::endl;
    arm.setPoseTarget( current_pose);
    std::cout << "About to plan" << std::endl;
    arm.plan(plan_result);
    std::cout << "Plan done" << std::endl;
  }

  //ros::waitForShutdown();
  ros::shutdown();
  return 0;
}
