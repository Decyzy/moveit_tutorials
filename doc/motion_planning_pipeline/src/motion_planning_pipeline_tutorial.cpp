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

/* Author: Sachin Chitta, Mike Lautman*/

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_group_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::NodeHandle node_handle("~");

  // BEGIN_TUTORIAL
  // 开始
  // ^^^^^
  // 设置并开始使用规划管道非常容易。
  // 在加载计划器之前，我们需要两个对象，即 RobotModel 和 PlanningScene 。
  //
  // 我们将从实例化一个 `RobotModelLoader`_ 对象开始，该对象将在 ROS 参数服务器上查找 robot description ，并构建一个供我们使用的 :moveit_core:`RobotModel` 。
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/noetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoaderPtr robot_model_loader(
      new robot_model_loader::RobotModelLoader("robot_description"));

  // 通过使用 RobotModelLoader ，我们可以构造一个规划场景监视器（planing scene monitor），该监视器将创建一个规划场景，监视规划场景变更，并将变更应用于其内部的规划场景对象（planning scene）。 然后，我们调用 startSceneMonitor ，startWorldGeometryMonitor 和 startStateMonitor 以完全初始化规划场景监视器。
  planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));

  /* 监听主题 /XXX 上的规划场景消息，并将其应用于对应的内部规划场景对象 */
  psm->startSceneMonitor();
  /* 监听世界几何体，碰撞对象和 octomaps（可选）*/
  psm->startWorldGeometryMonitor();
  /* 监听关节状态更新以及附加的碰撞对象的更改，并相应地更新内部规划场景对象 */
  psm->startStateMonitor();

  /* 我们还可以使用 RobotModelLoader 获得包含机器人运动学信息的机器人模型 ，即 RobotModel 对象 */
  moveit::core::RobotModelPtr robot_model = robot_model_loader->getModel();

  /* 我们可以通过锁定内部规划场景并读取来从 PlanningSceneMonitor 中获取最新的机器人状态。
  这个锁定可确保在我们读取基础场景时，该对象不会被更新。
  在许多其他用途中，RobotState 可用于计算机器人的正向和反向运动学。 */
  moveit::core::RobotStatePtr robot_state(
      new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

  /* 创建一个 JointModelGroup 来跟踪当前的机器人位姿和 planning group。 Joint Model
     group 可用于一次处理一组关节，例如处理左臂或末端执行器 */
  const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("panda_arm");

  // 现在，我们可以设置 PlanningPipeline 对象，该对象将使用 ROS 参数服务器来确定请求适配器集 request adapter 和要使用的规划插件。
  planning_pipeline::PlanningPipelinePtr planning_pipeline(
      new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));

  // 可视化
  // ^^^^^^^^^^^^^
  // MoveItVisualTools 包提供了 RViz 里许多将物体、机器人
  // 和轨迹可视化的功能，还提供了一些调试工具，如脚本的逐步运行。
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  /* 远程控制是一种自我审查工具，允许用户通过 RViz 中的按钮和快捷键来逐步地执行高级脚本。 */
  visual_tools.loadRemoteControl();

  /* RViz 提供了许多类型的 markers，在本 demo 中，我们将使用文本，圆柱体和球体 */
  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);

  /* 批量发布上述数据，用于减少发送给 RViz 进行大型可视化的消息数量 */
  visual_tools.trigger();

  /* 我们还可以使用 visual_tools 来待用户输入 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // 目标位姿
  // ^^^^^^^^^
  // 现在，我们将为 Panda 的右臂创建一个运动规划的请求，并将末端执行器的所需位姿指定为输入。 
  planning_interface::MotionPlanRequest req;
  planning_interface::MotionPlanResponse res;
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "panda_link0";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.75;
  pose.pose.orientation.w = 1.0;

  // 位置容许误差为 0.01 m，姿态容许误差为 0.01 弧度。
  std::vector<double> tolerance_pose(3, 0.01);
  std::vector<double> tolerance_angle(3, 0.01);

  //  我们使用 `kinematic_constraints`_ 包中提供的一个辅助函数来创建带约束的规划请求。
  //
  // .. _kinematic_constraints:
  //     http://docs.ros.org/noetic/api/moveit_core/html/namespacekinematic__constraints.html#a88becba14be9ced36fefc7980271e132
  req.group_name = "panda_arm";
  moveit_msgs::Constraints pose_goal =
      kinematic_constraints::constructGoalConstraints("panda_link8", pose, tolerance_pose, tolerance_angle);
  req.goal_constraints.push_back(pose_goal);

  // 在执行规划之前，我们需要在规划场景上加只读锁，以便在规划时不会被修改 world representation 。
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* 现在，调用规划管道并检查规划是否成功。 */
    planning_pipeline->generatePlan(lscene, req, res);
  }
  /* 现在，调用规划管道并检查规划是否成功。 */
  /* 检查规划是否成功 */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }

  // 可视化结果
  // ^^^^^^^^^^^^^^^^^^^^
  ros::Publisher display_publisher =
      node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  /* 可视化运动轨迹 */
  ROS_INFO("Visualizing the trajectory");
  moveit_msgs::MotionPlanResponse response;
  res.getMessage(response);

  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();

  /* 等待用户输入 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 关节空间的目标位置
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  /* 首先，将规划场景中的状态设置为上一次运动规划的最终状态 */
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // 现在, 设置一个关节空间下的目标点 。
  moveit::core::RobotState goal_state(*robot_state);
  std::vector<double> joint_values = { -1.0, 0.7, 0.7, -1.5, -0.7, 2.0, 0.0 };
  goal_state.setJointGroupPositions(joint_model_group, joint_values);
  moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(joint_goal);

  // 在执行规划之前，我们需要在规划场景上加只读锁，以便在规划时不会被修改 world representation 。
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* 现在, 调用规划管道并检查规划是否成功。 */
    planning_pipeline->generatePlan(lscene, req, res);
  }
  /* 检查规划是否成功。 */
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /* 可视化轨迹 */
  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  // 现在您应该看到两个规划的轨迹
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();

  /* 等待用户输入 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 使用规划请求适配器 Planning Request Adapter
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 通过规划请求适配器 Planning Request Adapter ，我们可以指定在进行规划之前或完成规划之后在结果路径上应该执行的一系列操作 。

  /* 首先, 将规划场景中的状态设置为上一次运动规划的最终状态 */
  robot_state = planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentStateUpdated(response.trajectory_start);
  robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
  moveit::core::robotStateToRobotStateMsg(*robot_state, req.start_state);

  // 现在, 设置其中一个将其中一个关节值设置为略高于其上限 。
  const moveit::core::JointModel* joint_model = joint_model_group->getJointModel("panda_joint3");
  const moveit::core::JointModel::Bounds& joint_bounds = joint_model->getVariableBounds();
  std::vector<double> tmp_values(1, 0.0);
  tmp_values[0] = joint_bounds[0].min_position_ - 0.01;
  robot_state->setJointPositions(joint_model, tmp_values);

  req.goal_constraints.clear();
  req.goal_constraints.push_back(pose_goal);

  // 在执行规划之前，我们需要在规划场景上加只读锁，以便在规划时不会被修改 world representation 。
  {
    planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
    /* 现在, 调用规划管道并检查规划是否成功 */
    planning_pipeline->generatePlan(lscene, req, res);
  }
  if (res.error_code_.val != res.error_code_.SUCCESS)
  {
    ROS_ERROR("Could not compute plan successfully");
    return 0;
  }
  /* 可视化轨迹 */
  ROS_INFO("Visualizing the trajectory");
  res.getMessage(response);
  display_trajectory.trajectory_start = response.trajectory_start;
  display_trajectory.trajectory.push_back(response.trajectory);
  /*现在您应该可以看到三个规划的轨迹 */
  display_publisher.publish(display_trajectory);
  visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
  visual_tools.trigger();

  /* 等待用户输入 */
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to finish the demo");

  ROS_INFO("Done");
  return 0;
}
