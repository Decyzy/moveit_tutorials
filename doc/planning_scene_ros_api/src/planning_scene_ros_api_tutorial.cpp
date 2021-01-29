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

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "planning_scene_ros_api_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  // BEGIN_TUTORIAL
  //
  // 可视化
  // ^^^^^^^^^^^^^
  // 软件包 MoveItVisualTools 提供了许多用于可视化 RViz 中的对象、机械手和轨迹的功能。其还提供了一些调试工具，例如脚本的逐步自检。
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();

  // ROS API
  // ^^^^^^^
  // planning scene 发布者的 ROS API 是通过使用 "diffs" 的话题接口实现的。
  // planning scene diff 是当前规划场景（由 move_group 节点维护）与用户所需的新规划场景之间的差异。
  //
  // 发布所需的话题
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 我们创建一个发布者并等待订阅者。请注意，可能需要在 launch 文件中重映射此话题。
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // 定义固连物体的消息
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 我们将使用此消息从世界中添加或移除对象，并将该对象固连到机器人上。 
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "panda_hand";
  /* 头必须包含有效的 TF 坐标系 */
  attached_object.object.header.frame_id = "panda_hand";
  /* 对象的id */
  attached_object.object.id = "box";

  /* 一个默认位姿 */
  geometry_msgs::Pose pose;
  pose.position.z = 0.11;
  pose.orientation.w = 1.0;

  /* 定义一个要附加的 box */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.075;
  primitive.dimensions[1] = 0.075;
  primitive.dimensions[2] = 0.075;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

  // 请注意，将对象固连到机器人上需要将相应的操作指定为 ADD 操作。
  attached_object.object.operation = attached_object.object.ADD;

  // 由于我们将对象固连到机械手上来模拟拾取对象，因此我们希望碰撞检查器忽略对象和机械手之间的碰撞。
  attached_object.touch_links = std::vector<std::string>{ "panda_hand", "panda_leftfinger", "panda_rightfinger" };

  // 将一个对象添加到环境里
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 将对象添加到环境中的方法是将其添加到规划场景的 "world" 部分中的一组碰撞对象中。
  // 注意，这里我们仅使用 attached_object 消息的 "object" 字段。
  ROS_INFO("Adding the object into the world at the location of the hand.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 插曲：同步与异步更新
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 有两种单独的机制可使用变更信息与 move_group 节点进行交互：
  //
  // * 通过 rosservice 调用发送变更信息并阻塞，直到变更信息被接受并应用（同步更新）
  // * 通过话题发送变更信息，即使变更信息可能尚未生效也继续（异步更新） 
  //
  // 尽管本教程的大部分内容都使用后一种机制（考虑到为可视化而插入了长时间的睡眠，异步更新不会有问题），
  // 但完全可以用以下 service 客户端替换 planning_scene_diff_publisher：
  ros::ServiceClient planning_scene_diff_client =
      node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  // 并通过 service 调用 call() 方法来将变更信息发送到规划场景：
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);
  // 请注意，直到我们确定变更信息已被接受并应用后，此操作才会继续。
  //
  // 将对象固连到机器人
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 当机器人从环境中拾取物体时，我们需要将物体 "attach" 到机器人，
  // 以便任何处理机器人模型的组件都知道要考虑新附加的物体，例如碰撞检查。
  //
  // 添加对象需要两项操作
  // 
  //  * 从环境中删除原始对象；
  //  * 将对象固连到机器人。

  /* 首先, 定义 REMOVE 物体的信息 */
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "panda_hand";
  remove_object.operation = remove_object.REMOVE;

  // 请注意我们是如何通过首先清除 collision_objects 来确保变更消息不包含其他附加对象或碰撞对象的。

  /* 执行 REMOVE + ATTACH 操作 */
  ROS_INFO("Attaching the object to the hand and removing it from the world.");
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 从机器人上取走物体
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 从机器人上取走物体需要两项操作：
  // 
  //  * 从机器人上取走物体
  //  * 将该物体重新引入环境

  /* 首先, 定义 DETACH 物体的信息 */
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "panda_hand";
  detach_object.object.operation = attached_object.object.REMOVE;

  // 请注意我们是如何通过首先清除 collision_objects 来确保变更消息不包含其他附加对象或碰撞对象的。

  /* 执行 DETACH + ADD 操作 */
  ROS_INFO("Detaching the object from the robot and returning it to the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.robot_state.is_diff = true;
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

  // 从碰撞体世界中移除物体
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // 从碰撞体世界中删除对象仅需要使用前面定义的删除对象消息。
  // 请注意我们是如何通过首先清除 collision_objects 来确保变更消息不包含其他附加对象或碰撞对象的。
  ROS_INFO("Removing the object from the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher.publish(planning_scene);
  // END_TUTORIAL

  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to end the demo");

  ros::shutdown();
  return 0;
}
