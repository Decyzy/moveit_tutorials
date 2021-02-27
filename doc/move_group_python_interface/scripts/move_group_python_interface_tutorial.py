#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman
# Translator: Zhiyu YANG

## BEGIN_SUB_TUTORIAL imports
##
## 要使用 MoveIt 的 Python 接口, 我们将 import `moveit_commander`_ 命名空间。
## 这个命名空间为我们提供了一个 `MoveGroupCommander`_ 类, 一个 `PlanningSceneInterface`_ 类,
## 和一个 `RobotCommander`_ 类。 此外，我们还将 import `rospy`_ 和一些我们将使用的 messages ：
##

# Python 2/3 compatibility imports
from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, dist, fabs, cos
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if the values in two lists are within a tolerance of each other.
  For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle 
  between the identical orientations q and -q is calculated correctly).
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
    x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
    # Euclidean distance
    d = dist((x1, y1, z1), (x0, y0, z0))
    # phi = angle between orientations
    cos_phi_half = fabs(qx0*qx1 + qy0*qy1 + qz0*qz1 + qw0*qw1)
    return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

  return True


class MoveGroupPythonInterfaceTutorial(object):
  """MoveGroupPythonInterfaceTutorial"""
  def __init__(self):
    super(MoveGroupPythonInterfaceTutorial, self).__init__()

    ## BEGIN_SUB_TUTORIAL setup
    ##
    ## 首先初始化 `moveit_commander`_ 和 `rospy`_ 节点:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial', anonymous=True)

    ## 实例化 `RobotCommander`_ 对象。 其提供诸如机器人运动学模型、
    ## 机器人当前关节值等信息
    robot = moveit_commander.RobotCommander()

    ## 实例化一个 `PlanningSceneInterface`_ 对象。其提供了一个远程接口， 
    ## 用于获取、设置和更新机器人
    ## 对其周围环境的认知：
    scene = moveit_commander.PlanningSceneInterface()

    ## 实例化一个 `MoveGroupCommander`_ 对象。
    ## 该对象是 planning group (由一组关节组成) 的一个接口。在本教程中，
    ## 这个 planning group 是 Panda 机器人里主要的关机，因此我们设置这个 planning group 的名称为 "panda_arm" 。
    ## 如果当前使用其他机器人，请将此值更改为该机器人机械臂部分的 planning group 名字。
    ## 该接口可用于运动规划和执行动作：
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    ## 创建一个名为 `DisplayTrajectory`_ 的 ROS publisher ，用于在 Rviz 中显示轨迹：
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    ## END_SUB_TUTORIAL

    ## BEGIN_SUB_TUTORIAL basic_info
    ##
    ## 获取基础信息
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^
    # 我们可以获得该机器人参考坐标系的名字： 
    planning_frame = move_group.get_planning_frame()
    print("============ Planning frame: %s" % planning_frame)

    # 我们还可以为该 planning group 打印末端执行器 link 的名字： 
    eef_link = move_group.get_end_effector_link()
    print("============ End effector link: %s" % eef_link)

    # 我们可以获得机器人中所有 planning group 的列表:
    group_names = robot.get_group_names()
    print("============ Available Planning Groups:", robot.get_group_names())

    # 有时输出机械臂的所有状态对于调试很有用:
    print("============ Printing robot state")
    print(robot.get_current_state())
    print("")
    ## END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names


  def go_to_joint_state(self):
    # 这里将类成员变量复制到局部变量是为了使本教程更加清楚。
    # 但是在实际使用中，除非有充分的理由，否则应该直接使用类成员变量。
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_joint_state
    ##
    ## 目标关节角度的运动规划
    ## ^^^^^^^^^^^^^^^^^^^^^^^^
    ## Panda 机械臂的未被修改过的初始位姿是在一个 `奇异状态 <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_ ，
    ## 所以我们要做的第一件事就是将其移到稍微更好的位姿。
    # 我们可以从 planning group 中获取并调整各关节角度:
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/4
    joint_goal[2] = 0
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = pi/3
    joint_goal[6] = 0

    # 使用关节值或位姿来调用 go 命令，
    # 在已经设置了 planning group 的目标位姿或或目标关节角度的情况下可以不带任何参数。
    move_group.go(joint_goal, wait=True)

    # 调用 ``stop()`` 以确保没有未完成的运动。
    move_group.stop()

    ## END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)


  def go_to_pose_goal(self):
    # 这里将类成员变量复制到局部变量是为了使本教程更加清楚。
    # 但是在实际使用中，除非有充分的理由，否则应该直接使用类成员变量。
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    ## 目标位姿的运动规划
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## 我们可以为这个 planning group 规划一个运动，
    ## 使其末端执行器达到所需的位姿:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    ## 现在，我们调用规划器 (planner) 来进行运动规划，然后执行规划的路径。
    plan = move_group.go(wait=True)
    # 调用 `stop()` 以确保没有未完成的运动。
    move_group.stop()
    # 对某一目标位姿进行运动规划以后，最好清除这个目标位姿。
    # 注意: 没有类似于 clear_joint_value_targets() 的函数
    move_group.clear_pose_targets()

    ## END_SUB_TUTORIAL

    # For testing:
    # Note that since this section of code will not be included in the tutorials
    # we use the class variable rather than the copied state variable
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)


  def plan_cartesian_path(self, scale=1):
    # 这里将类成员变量复制到局部变量是为了使本教程更加清楚。
    # 但是在实际使用中，除非有充分的理由，否则应该直接使用类成员变量。
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    ## 笛卡尔路径规划
    ## ^^^^^^^^^^^^^^^
    ## 您可以通过一系列末端执行器要通过的沿途路径点来直接进行笛卡尔路径规划。
    ## 如果想要在 Python shell 里交互执行，
    ## 设置 scale = 1.0 。
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # 1 向上 (z)
    wpose.position.y += scale * 0.2  # 以及侧边 (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # 2 向前/向后 in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # 3 侧边移动 (y)
    waypoints.append(copy.deepcopy(wpose))

    # 我们希望以 1 cm 的精度插值笛卡尔路径，
    # 这就是为什么我们在笛卡尔换算 compute_cartesian_path() 函数中将最大步长 eef_step 设置为 0.01 的原因。
    # 我们将 jump_threshold 指定为 0.0 以禁用跳跃阈值，即
    # 忽略检查关节空间中不可行的 jump 。这对本教程而言禁用跳跃阈值无妨。
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # 要求经过的沿途路径点
                                       0.01,        # eef_step, 末端执行器步长
                                       0.0)         # jump_threshold

    # 注意: 注意，我们只是在进行规划，而不是要求 move_group 真的让机器人运动。
    return plan, fraction

    ## END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # 这里将类成员变量复制到局部变量是为了使本教程更加清楚。
    # 但是在实际使用中，除非有充分的理由，否则应该直接使用类成员变量。
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    ## BEGIN_SUB_TUTORIAL display_trajectory
    ##
    ## 显示轨迹
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## 您可以要求 RViz 为您可视化一个规划结果（plan, 也称为轨迹）。但是
    ## group.plan() 方法会自动可视化轨迹，所以这里单独再显示一下有点多余。
    ## (它只是再次显示相同的轨迹):
    ##
    ## `DisplayTrajectory`_ 消息有两个主要的字段: trajectory_start 和 trajectory。
    ## 我们使用当前机器人状态填充 trajectory_start 以拷贝所有的
    ## AttachedCollisionObjects，同时将我们的规划结果加入到我 trajectory 字段里面。
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # 发布消息
    display_trajectory_publisher.publish(display_trajectory);

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # 这里将类成员变量复制到局部变量是为了使本教程更加清楚。
    # 但是在实际使用中，除非有充分的理由，否则应该直接使用类成员变量。
    move_group = self.move_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## 执行轨迹规划的结果
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 如果您希望机器人沿着已经规划好的轨迹运动，
    ## 请使用 execute 方法
    move_group.execute(plan, wait=True)

    ## **注意:** 机器人的当前关节状态必须在 `RobotTrajectory`_ 或 ``execute()`` 中的第一个沿途路径点的容许误差范围内，
    ## 否则执行 execute 将失败。
    ## END_SUB_TUTORIAL


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # 这里将类成员变量复制到局部变量是为了使本教程更加清楚。
    # 但是在实际使用中，除非有充分的理由，否则应该直接使用类成员变量。
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    ## 确保收到碰撞关系的更新消息
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 如果 Python 节点在发布冲突对象的更新消息之前死亡，
    ## 则该消息可能会丢失并且该 box 将不会出现。
    ## 为了确保更新生效，我们等到
    ## ``get_attached_objects()`` 和 ``get_known_object_names()`` 列表中出现更改的内容为止。
    ## 就本教程而言，我们在规划场景中添加、移除、附加或分离对象之后都将调用此函数。
    ## 然后我们等待更新完成，或超过 ``timeout`` 这么多秒。
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # 测试 box 是否在已附着的物体列表 attached_objects 中
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # 测试 box 是否在场景中。
      # 请注意，附加 box 的操作会删除 known_objects 中的 box
      is_known = box_name in scene.get_known_object_names()

      # 测试我们是否处于预期状态
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # 休眠一会，使处理器上可以调度运行其他线程
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # 如果我们退出了 while 循环而没有 return，那么就意味着超时了
    return False
    ## END_SUB_TUTORIAL


  def add_box(self, timeout=4):
    # 这里将类成员变量复制到局部变量是为了使本教程更加清楚。
    # 但是在实际使用中，除非有充分的理由，否则应该直接使用类成员变量。
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL add_box
    ##
    ## 将物体添加到规划场景里 (planning scene)
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 首先，我们在规划场景中的夹抓之间创建一个 box：
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_hand"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.11 # 在 panda_hand 坐标系上方
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

    ## END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # 这里将类成员变量复制到局部变量是为了使本教程更加清楚。
    # 但是在实际使用中，除非有充分的理由，否则应该直接使用类成员变量。
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    ## BEGIN_SUB_TUTORIAL attach_object
    ##
    ## 将物体固连到机器人
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 接下来, 我们将 box 固连到 Panda 机械臂的腕部。机械臂移动物体需要
    ## 其能够接触物体，但是当前规划场景会将这报告为发生碰撞。
    ## 因此，我们将 link 的名字添加到 ``touch_links`` 数组里，
    ## 让规划场景忽略那些 link 和 box 之间的碰撞。
    ## 对于 Panda 机器人，即设置 ``grasping_group = 'hand'`` 。如果使用其他机器人，
    ## 则应将该变量设置为对应的末端执行器 group 的名称。
    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    # 这里将类成员变量复制到局部变量是为了使本教程更加清楚。
    # 但是在实际使用中，除非有充分的理由，否则应该直接使用类成员变量。
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    ## BEGIN_SUB_TUTORIAL detach_object
    ##
    ## 从机器人上取下物体
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 我们还可以从规划场景中分离和移除对象:
    scene.remove_attached_object(eef_link, name=box_name)
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    # 这里将类成员变量复制到局部变量是为了使本教程更加清楚。
    # 但是在实际使用中，除非有充分的理由，否则应该直接使用类成员变量。
    box_name = self.box_name
    scene = self.scene

    ## BEGIN_SUB_TUTORIAL remove_object
    ##
    ## 从规划场景中删除对象
    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    ## 我们可以将 box 从世界里移走。
    scene.remove_world_object(box_name)

    ## **注意:** 必须先将物体分离，然后才能将其从世界上移除。
    ## END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    print("")
    print("----------------------------------------------------------")
    print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
    print("----------------------------------------------------------")
    print("Press Ctrl-D to exit at any time")
    print("")
    input("============ Press `Enter` to begin the tutorial by setting up the moveit_commander ...")
    tutorial = MoveGroupPythonInterfaceTutorial()

    input("============ Press `Enter` to execute a movement using a joint state goal ...")
    tutorial.go_to_joint_state()

    input("============ Press `Enter` to execute a movement using a pose goal ...")
    tutorial.go_to_pose_goal()

    input("============ Press `Enter` to plan and display a Cartesian path ...")
    cartesian_plan, fraction = tutorial.plan_cartesian_path()

    input("============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ...")
    tutorial.display_trajectory(cartesian_plan)

    input("============ Press `Enter` to execute a saved path ...")
    tutorial.execute_plan(cartesian_plan)

    input("============ Press `Enter` to add a box to the planning scene ...")
    tutorial.add_box()

    input("============ Press `Enter` to attach a Box to the Panda robot ...")
    tutorial.attach_box()

    input("============ Press `Enter` to plan and execute a path with an attached collision object ...")
    cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
    tutorial.execute_plan(cartesian_plan)

    input("============ Press `Enter` to detach the box from the Panda robot ...")
    tutorial.detach_box()

    input("============ Press `Enter` to remove the box from the planning scene ...")
    tutorial.remove_box()

    print("============ Python tutorial demo complete!")
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL
 