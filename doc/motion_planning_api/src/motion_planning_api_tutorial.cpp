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

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <boost/scoped_ptr.hpp>

int main(int argc, char** argv)
{
  const std::string node_name = "motion_planning_tutorial";
  ros::init(argc, argv, node_name);
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // BEGIN_TUTORIAL
  // 开始
  // ^^^^^
  // 设置并开始使用规划器 planner 非常容易。
  // 规划器 planner 在 MoveIt 中被设置为插件，您可以使用 ROS 的 pluginlib 接口来加载要使用的任何规划器程序。
  // 在加载 planner 之前，我们需要两个对象，即 RobotModel 和 PlanningScene 。
  // 我们将从实例化 `RobotModelLoader`_ 对象开始，该对象将在 ROS 的参数服务器上查找 robot description ，
  // 并构造一个 :moveit_core:`RobotModel` 供我们使用。
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/noetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  const std::string PLANNING_GROUP = "panda_arm";
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
  /* 创建一个 RobotState 和 JointModelGroup 来跟踪当前的机器人位姿和 planning group */
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

  // 使用 :moveit_core:`RobotModel` 构造一个 :planning_scene:`PlanningScene` 来维护
  // 世界（包括机器人）的状态
  planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));

  // 配置一个有效的机器人状态
  planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

  // 现在，我们将构造一个加载器，以按名字加载规划器。
  // 注意这里我们使用 ROS pluginlib 库。
  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  // 我们将从 ROS 的参数服务器中获取要加载的规划插件 planning plugin 的名称，然后加载规划器。
  // 同时需要确保捕获所有异常。
  if (!node_handle.getParam("planning_plugin", planner_plugin_name))
    ROS_FATAL_STREAM("Could not find planner plugin name");
  try
  {
    planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL_STREAM("Exception while creating planning plugin loader " << ex.what());
  }
  try
  {
    planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
    if (!planner_instance->initialize(robot_model, node_handle.getNamespace()))
      ROS_FATAL_STREAM("Could not initialize planner instance");
    ROS_INFO_STREAM("Using planning interface '" << planner_instance->getDescription() << "'");
  }
  catch (pluginlib::PluginlibException& ex)
  {
    const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
    std::stringstream ss;
    for (const auto& cls : classes)
      ss << cls << " ";
    ROS_ERROR_STREAM("Exception while loading planner '" << planner_plugin_name << "': " << ex.what() << std::endl
                                                         << "Available plugins: " << ss.str());
  }

  // 可视化
  // ^^^^^^^^^^^^^
  // 
  // MoveItVisualTools 包提供了 RViz 里许多将物体、机器人
  // 和轨迹可视化的功能，还提供了一些调试工具，如脚本的逐步运行。
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.loadRobotStatePub("/display_robot_state");
  visual_tools.enableBatchPublishing();
  visual_tools.deleteAllMarkers();  // 清除所有旧的 markers
  visual_tools.trigger();

  /*远程控制是一种自我审查工具，允许用户通过 RViz 中的按钮和快捷键来逐步地执行高级脚本。 */
  visual_tools.loadRemoteControl();

  /* RViz 提供了许多类型的 markers，在本 demo 中，我们将使用文本，圆柱体和球体 */
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning API Demo", rvt::WHITE, rvt::XLARGE);

  /* 批量发布上述数据，用于减少发送给 RViz 进行大型可视化的消息数量 */
  visual_tools.trigger();

  /* 我们还可以使用 visual_tools 来待用户输入 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // 目标位姿
  // ^^^^^^^^^
  //
  // 现在，我们将为 Panda 机械臂创建一个运动规划的请求，并将末端执行器的所需位姿指定为输入。
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.trigger();
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.4;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // 位置容许误差为 0.01 m，姿态容许误差为 0.01 弧度。
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  // 我们使用 `kinematic_constraints`_ 包中提供的一个辅助函数来创建带约束的规划请求。
  //
  // .. _kinematic_constraints:
  //     http://docs.ros.org/noetic/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

  req.group_name = PLANNING_GROUP;
  req.goal_constraints.push_back(pose_goal);

  // 现在，我们构建一个运动规划的上下文，其中封装了场景，请求和响应。我们使用此规划上下文来调用规划器
  planning_interface::PlanningContextPtr context =
      planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // 可视化规划结果
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* 可视化运动轨迹 */
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  /* 将规划场景中的状态设置为上一次运动规划的最终状态 */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // 显示目标状态
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.publishAxisLabeled(pose.pose, "goal_1");
  visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* 我们还可以使用 visual_tools 来等待用户输入 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 关节空间里的目标位置
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 现在, 设置一个关节空间里的目标位置
  moveit::core::RobotState goal_state(robot_model);
  std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);
  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // 调用规划器并可视化运动轨迹
  /* 重新构造规划上下文 */
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  /* 调用规划器 */
  context->solve(res);
  /* 检查运动规划是否成功 */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /* 可视轨迹 */
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);

  /* 现在您应该看到两个运动规划的运动轨迹 */
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  /* 我们将设置更多目标。但是首先，我们将规划场景中的状态设置为上一次运动规划的最终状态 */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // 可视化目标状态
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.publishAxisLabeled(pose.pose, "goal_2");
  visual_tools.publishText(text_pose, "Joint Space Goal (2)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* 等待用户输入 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  /* 现在，我们回到第一个目标，为旋转受限的运动规划做准备 */
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  res.getMessage(response);

  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  /* 将规划场景中的状态设置为上一次运动规划的最终状态 */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // 可视化目标状态
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.trigger();

  /* 等待用户输入 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 添加路径约束
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // 让我们再次添加一个新的目标位姿。这次我们还将向运动规划添加路径约束。
  /* 创建一个新的位姿目标 */

  pose.pose.position.x = 0.32;
  pose.pose.position.y = -0.25;
  pose.pose.position.z = 0.65;
  pose.pose.orientation.w = 1.0;
  moveit_msgs::Constraints pose_goal_2 =
      kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);

  /* 现在, 让我们尝试向这个新的位姿目标移动 */
  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal_2);

  /* 但是，让我们对运动规划施加路径限制。在这里，我们要求末端执行器始终保持水平 */
  geometry_msgs::QuaternionStamped quaternion;
  quaternion.header.frame_id = "panda_link0";
  quaternion.quaternion.w = 1.0;
  req.path_constraints = kinematic_constraints::constructGoalConstraints("panda_link8", quaternion);

  // 由于施加了路径约束，规划器需要在末端执行器（机器人的工作空间）的可达位置空间中进行规划，
  // 因此，我们还需要为允许的规划方案指定一个限制条件。
  // 注意：默认界限由 WorkspaceBounds 请求适配器（属于 OMPL 管道的一部分，但在本示例中未使用）自动填充。
  // 我们使用的限制条件显然包括手臂的可到达空间，因为在规划时不会在不可达空间中进行采样；
  // 限制条件仅用于确定采样的配置是否有效。
  req.workspace_parameters.min_corner.x = req.workspace_parameters.min_corner.y =
      req.workspace_parameters.min_corner.z = -2.0;
  req.workspace_parameters.max_corner.x = req.workspace_parameters.max_corner.y =
      req.workspace_parameters.max_corner.z = 2.0;

  // 调用规划器 planner 并可视化目前为止创建的所有运动规划结果。
  context = planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
  context->solve(res);
  res.getMessage(response);
  display_trajectory.trajectory.push_back(response.trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();
  display_publisher.publish(display_trajectory);

  /* 将规划场景中的状态设置为上一次运动规划的最终状态 */
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  planning_scene->setCurrentState(*robot_state.get());

  // 可视化目标状态
  visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
  visual_tools.publishAxisLabeled(pose.pose, "goal_3");
  visual_tools.publishText(text_pose, "Orientation Constrained Motion Plan (3)", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // END_TUTORIAL
  /* 等待用户输入 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to exit the demo");
  planner_instance.reset();

  return 0;
}
