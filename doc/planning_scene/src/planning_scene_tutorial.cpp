/*********************************************************************
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
 *   * Neither the name of Willow Garage nor the names of its
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

/* Author: Sachin Chitta, Michael Lautman */
/* Translator: Zhiyu YANG */

#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

// BEGIN_SUB_TUTORIAL stateFeasibilityTestExample
//
// 用户自定义的约束也可以指定给 PlanningScene 类。
// 这是通过使用 setStateFeasibilityPredicate 函数指定回调来完成的。
// 下面是一个用户自定义回调的简单示例，该回调函数检查 Panda 机器人的 "panda_joint1" 值是正的还是负的：
bool stateFeasibilityTestExample(const moveit::core::RobotState& kinematic_state, bool /*verbose*/)
{
  const double* joint_values = kinematic_state.getJointPositions("panda_joint1");
  return (joint_values[0] > 0.0);
}
// END_SUB_TUTORIAL

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // 设置
  // ^^^^^
  //
  // 可以使用 :moveit_core:`RobotModel` 或 URDF 和 SRDF 文件轻松设置和配置 :planning_scene:`PlanningScene` 类。 
  // 但是，建议使用 :planning_scene_monitor:`PlanningSceneMonitor` 方法来实例化 PlanningScene 来维护当前的 planning scene 。
  // 该方法使用来自机器人关节和传感器的数据来创建和维护当前的 planning scene （将在下一节中详细讨论）。
  // 在本教程中，我们将直接实例化 PlanningScene 类，但是这种实例化方法仅作本教程阐述之用。

  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  // 碰撞检查
  // ^^^^^^^^^^^^^^^^^^
  //
  // 自碰撞检查
  // ~~~~~~~~~~~~~~~~~~~~~~~
  //
  // 我们要做的第一件事是检查机器人当前是否处于 *自碰撞* 状态，即机器人的当前配置是否会导致机器人的零件相互碰撞。 
  // 为此，我们将构造一个 :collision_detection_struct:`CollisionRequest` 对象和一个
  // :collision_detection_struct:`CollisionResult` 对象，并将它们传递给碰撞检查函数。
  // 注意，机器人是否处于自碰撞状态的结果都包含在结果中。 自碰撞检查使用的是机器人的
  // *未添加边距(unpadded)* 版本，即它直接使用 URDF 中提供的碰撞网格，而没有添加额外的边距。

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 1: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // 修改 state
  // ~~~~~~~~~~~~~~~~
  //
  // 现在，让我们更改机器人的当前状态。
  // planning_scene 在内部维护当前机器人状态。
  // 我们可以获取它的引用并进行更改，然后检查新机器人外形的碰撞情况。
  // 特别要注意的是，在发出新的碰撞检查请求之前，我们需要清除碰 collision_result 。

  moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 2: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // 检查一个 planning group
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // 现在，我们将仅对 Panda 的手部进行碰撞检查。
  // 也就是说，我们将检查手部与机器人自身其他部位之间是否存在任何碰撞。
  // 我们可以通过将组名称 "hand" 添加到碰撞检查的请求中，以进行特定的碰撞检查。

  collision_request.group_name = "hand";
  current_state.setToRandomPositions();
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 3: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // 获取关联信息 (Contact Information)
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // 首先，将 Panda 臂手动设置到我们已知会发生自碰撞的位置。
  // 请注意，这个位置实际上已经超出了熊猫的关节限位，但是我们仍然可以对其进行检查。

  std::vector<double> joint_values = { 0.0, 0.0, 0.0, -2.9, 0.0, 1.4, 0.0 };
  const moveit::core::JointModelGroup* joint_model_group = current_state.getJointModelGroup("panda_arm");
  current_state.setJointGroupPositions(joint_model_group, joint_values);
  ROS_INFO_STREAM("Test 4: Current state is "
                  << (current_state.satisfiesBounds(joint_model_group) ? "valid" : "not valid"));

  // 现在，我们可以获得在给定状态配置的 Panda 臂上可能发生的任何碰撞的关联信息。
  // 我们可以通过在碰撞检查请求中的 contacts 字段中填写 true 并在
  // max_contacts 字段里指定要返回的最大关联数量来请求关联信息。

  collision_request.contacts = true;
  collision_request.max_contacts = 1000;

  //

  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result);
  ROS_INFO_STREAM("Test 5: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");
  collision_detection::CollisionResult::ContactMap::const_iterator it;
  for (it = collision_result.contacts.begin(); it != collision_result.contacts.end(); ++it)
  {
    ROS_INFO("Contact between: %s and %s", it->first.first.c_str(), it->first.second.c_str());
  }

  // 修改允许碰撞矩阵 (Allowed Collision Matrix)
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // :collision_detection_class:`AllowedCollisionMatrix` （ACM）提供了一种机制，
  // 用于忽略某些对象之间的碰撞：机器人上的部分和世界中的对象。
  // 我们可以告诉碰撞检查器忽略上面报告的 link 之间的所有碰撞。也就是说，即使
  // link 实际上处于碰撞状态，碰撞检查器也将忽略它们，不返回碰撞信息。 
  // 
  // 在此例中还要注意我们是如何制作允许碰撞矩阵和当前状态的副本的，并将它们传递给碰撞检查函数。

  collision_detection::AllowedCollisionMatrix acm = planning_scene.getAllowedCollisionMatrix();
  moveit::core::RobotState copied_state = planning_scene.getCurrentState();

  collision_detection::CollisionResult::ContactMap::const_iterator it2;
  for (it2 = collision_result.contacts.begin(); it2 != collision_result.contacts.end(); ++it2)
  {
    acm.setEntry(it2->first.first, it2->first.second, true);
  }
  collision_result.clear();
  planning_scene.checkSelfCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // 检查所有碰撞
  // ~~~~~~~~~~~~~~~~~~~~~~~
  //
  // 当我们一直在检查自碰撞时，我们可以改用 checkCollision 函数，
  // 它将同时检查自碰撞和与环境物体（当前为空）的碰撞。
  // 这是在规划器（planner）中最常使用的一组碰撞检查功能。
  // 请注意，与环境物体的碰撞检查将使用机器人的带边距（padded）版本。
  // 增加边距有助于使机器人远离环境中的障碍物。
  collision_result.clear();
  planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
  ROS_INFO_STREAM("Test 7: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

  // 检查约束
  // ^^^^^^^^^^^^^^^^^^^
  //
  // PlanningScene 类还包括用于检查约束的函数调用。
  // 约束可以有两种类型：（a）从集合 :kinematic_constraints:`KinematicConstraint` 中，如
  // :kinematic_constraints:`JointConstraint`,
  // :kinematic_constraints:`PositionConstraint`,
  // :kinematic_constraints:`OrientationConstraint` 和
  // :kinematic_constraints:`VisibilityConstraint` ；
  // （b）通过回调指定的自定义约束。我们首先来看一个使用 KinematicConstraint 的简单示例。
  //
  // 检查运动学约束
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // 我们将首先在 Panda 机器人的 planning group "panda_arm" 里的末端执行器上定义一个简单的位置和方向约束。
  // 请注意这里使用了比较方便的方便函数来构造约束（这些功能可以在 moveit_core 的 kinematic_constraints 目录里的 :moveit_core_files:`utils.h<utils_8h>` 文件中找到）。

  std::string end_effector_name = joint_model_group->getLinkModelNames().back();

  geometry_msgs::PoseStamped desired_pose;
  desired_pose.pose.orientation.w = 1.0;
  desired_pose.pose.position.x = 0.3;
  desired_pose.pose.position.y = -0.185;
  desired_pose.pose.position.z = 0.5;
  desired_pose.header.frame_id = "panda_link0";
  moveit_msgs::Constraints goal_constraint =
      kinematic_constraints::constructGoalConstraints(end_effector_name, desired_pose);

  // 现在，我们可以使用 PlanningScene 类中的 isStateConstrained 函数来检查此状态下的约束。
  
  copied_state.setToRandomPositions();
  copied_state.update();
  bool constrained = planning_scene.isStateConstrained(copied_state, goal_constraint);
  ROS_INFO_STREAM("Test 8: Random state is " << (constrained ? "constrained" : "not constrained"));

  // 检查约束有一种更有效的的方法（当您想一遍又一遍地检查相同的约束时，例如在规划器 planner 内部）。
  // 我们首先构造一个 KinematicConstraintSet 来快速预处理 ROS 的约束消息。

  kinematic_constraints::KinematicConstraintSet kinematic_constraint_set(kinematic_model);
  kinematic_constraint_set.add(goal_constraint, planning_scene.getTransforms());
  bool constrained_2 = planning_scene.isStateConstrained(copied_state, kinematic_constraint_set);
  ROS_INFO_STREAM("Test 9: Random state is " << (constrained_2 ? "constrained" : "not constrained"));

  // 有一种使用 KinematicConstraintSet 类直接完成的方法

  kinematic_constraints::ConstraintEvaluationResult constraint_eval_result =
      kinematic_constraint_set.decide(copied_state);
  ROS_INFO_STREAM("Test 10: Random state is " << (constraint_eval_result.satisfied ? "constrained" : "not constrained"));

  // 用户定义的约束
  // ~~~~~~~~~~~~~~~~~~~~~~~~
  //
  // CALL_SUB_TUTORIAL stateFeasibilityTestExample

  // 现在，每当调用 isStateFeasible 时，都会调用用户定义的回调函数。

  planning_scene.setStateFeasibilityPredicate(stateFeasibilityTestExample);
  bool state_feasible = planning_scene.isStateFeasible(copied_state);
  ROS_INFO_STREAM("Test 11: Random state is " << (state_feasible ? "feasible" : "not feasible"));

  // 每当调用 isStateValid 时，都会执行三项检查：（a）碰撞检查；（b）约束检查；（c）使用用户定义的回调函数进行可行性检查。

  bool state_valid = planning_scene.isStateValid(copied_state, kinematic_constraint_set, "panda_arm");
  ROS_INFO_STREAM("Test 12: Random state is " << (state_valid ? "valid" : "not valid"));

  // 请注意，通过 MoveIt 和 OMPL 可用的所有规划期（planner）都将执行碰撞检查、约束检查，以及使用用户定义的回调函数执行可行性检查。
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
