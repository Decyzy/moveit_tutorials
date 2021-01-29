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

/* Author: Sachin Chitta, Michael Lautman*/
/* Translator: Zhiyu YANG */

#include <ros/ros.h>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_model_and_robot_state_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  // 开始
  // ^^^^^^^^^^^^^
  // 设置 RobotModel 类非常容易。
  // 通常，您会发现大多数更高级别的组件都将返回指向 RobotModel 的共享指针。
  // 您应该尽可能使用共享指针。
  // 在此示例中，我们将从此类的共享指针开始，且仅讨论基础的 API 。
  // 您可以查看这些类的 API 源码，以获取更多有关如何使用这些类的所提供的更多功能的信息。
  //
  // 我们将从实例化 `RobotModelLoader`_ 对象开始，
  // 该对象将在 ROS 参数服务器上查找 robot description，
  // 并构造一个供我们使用的 :moveit_core:`RobotModel` 类。
  //
  // .. _RobotModelLoader:
  //     http://docs.ros.org/noetic/api/moveit_ros_planning/html/classrobot__model__loader_1_1RobotModelLoader.html
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

  // 使用 :moveit_core:`RobotModel`, 我们可以构建一个保存机器人配置的 :moveit_core:`RobotState` 。
  // 我们将此状态下的所有关节都设置为其默认值。 然后我们可以得到一个 :moveit_core:`JointModelGroup`,
  // 它代表特定 planning group 的机器人模型，例如 Panda 机器人的 "panda_arm" 。
  moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();
  const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup("panda_arm");

  const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();

  // 获得关节位置
  // ^^^^^^^^^^^^^^^^^^^^
  // 我们可以检索存储在机器人状态（kinematic_state）中的 Panda 机械臂的当前关节位置。
  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  for (std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // 关节限位
  // ^^^^^^^^^^^^^^^^^^^^
  // setJointGroupPositions() 函数本身不会限制关节位置，但是调用 enforceBounds() 函数可以做到这点。
  /* 设置 Panda 臂的一个关节位置为超出其限位的值 */
  joint_values[0] = 5.57;
  kinematic_state->setJointGroupPositions(joint_model_group, joint_values);

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  kinematic_state->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));

  // 正向运动学
  // ^^^^^^^^^^^^^^^^^^
  // 现在，我们可以为一组随机关节位置进行正向运动学计算。
  // 请注意，我们想找到 "panda_link8" 的位姿，
  // 它是机器人 "panda_arm" 这个 planning group 中最远端的连杆 (link) 。
  kinematic_state->setToRandomPositions(joint_model_group);
  const Eigen::Isometry3d& end_effector_state = kinematic_state->getGlobalLinkTransform("panda_link8");

  /* 打印末端执行器的位姿。请记住，这是在机器人模型坐标系中的表示 */
  ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation() << "\n");
  ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation() << "\n");

  // 逆向运动学
  // ^^^^^^^^^^^^^^^^^^
  // 现在，我们可以解决 Panda 机器人的逆运动学问题（IK）。要解决 IK ，我们将需要以下内容： 
  //
  //  * 末端执行器的所需位姿（默认情况下，末端执行器是 "panda_arm" 连杆链中的最后一个 link），
  //    即我们在以上步骤中计算得到的 end_effector_state 。
  //  * 求解超时设置 timeout: 0.1 s
  double timeout = 0.1;
  bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

  // 现在，我们可以打印出找到的 IK 解 (如果找到的话):
  if (found_ik)
  {
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  // 获得雅克比矩阵
  // ^^^^^^^^^^^^^^^^^^^^^^^^
  // 我们还能从 :moveit_core:`RobotState` 中获得雅克比。
  Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
  Eigen::MatrixXd jacobian;
  kinematic_state->getJacobian(joint_model_group,
                               kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position, jacobian);
  ROS_INFO_STREAM("Jacobian: \n" << jacobian << "\n");
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}
