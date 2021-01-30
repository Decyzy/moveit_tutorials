规划场景监控器 PlanningSceneMonitor
====================================================================

建议使用 :planning_scene_monitor:`PlanningSceneMonitor` 接口来维护最新的规划场景 (planning scene) 。:moveit_core:`RobotState`, :planning_scene_monitor:`CurrentStateMonitor`, :planning_scene:`PlanningScene`, :planning_scene_monitor:`PlanningSceneMonitor`, 和 :planning_interface:`PlanningSceneInterface` 之间的关系一开始确实令人困惑。本教程旨在阐明这些关键概念。

RobotState 机器人状态
----------------------------------------
:moveit_core:`RobotState` 是机器人的一个快照。 它包含 :moveit_core:`RobotModel` 和一组关节值。

CurrentStateMonitor 当前状态监视器
---------------------------------------------------------
:planning_scene_monitor:`CurrentStateMonitor` (CSM) 可以看作是 RobotState 的 ROS 封装。 它订阅了发布 :sensor_msgs:`JointState` 消息的 ROS 话题，此消息为单自由度执行器（例如旋转或棱柱形关节）提供了最新的传感器值，并使用这些关节值更新 CSM 内部的 RobotState 。除了单自由度关节之外，机器人还可以具有多自由度关节，比如浮动关节和平面关节。为维护 link 和其他带有多自由度关节的坐标系的最新变换信息，CSM 存储了一个 TF2 :tf2:`Buffer`，该缓冲区使用TF2 :tf2:`TransformListener` 以在其内部数据中进行坐标系变换。

PlanningScene 规划场景
---------------------------------------------------
:planning_scene:`PlanningScene` 是世界的一个快照，其中包括 RobotState 和任意数量的碰撞体。Planning Scene 可用于检查碰撞以及获取有关环境的信息。

PlanningSceneMonitor 规划场景监视器
----------------------------------------------------------
:planning_scene_monitor:`PlanningSceneMonitor` 使用ROS接口封装了 PlanningScene，用于保持 PlanningScene 为最新状态。要访问 PlanningSceneMonitor 底层的 PlanningScene，请使用 :planning_scene_monitor:`LockedPlanningSceneRW` 类和 :planning_scene_monitor:`LockedPlanningSceneRO` 类。

PlanningSceneMonitor 具有以下对象，它们具都有自己的 ROS 接口来保持 planning scene 的子组件为最新状态： 

 * :planning_scene_monitor:`CurrentStateMonitor` 通过 ``robot_state_subscriber_`` 和 ``tf_buffer_`` 跟踪对 RobotState 的更新。同时还通过一个 planning scene 的订阅者收听来自其他发布者发布的规划场景变更 (planning scene diffs)。
 * OccupancyMapMonitor 用于通过 ROS 话题和服务来跟踪对 OccupancyMap 的变更。

PlanningSceneMonitor 有以下订阅者 (subscribers):

 * ``collision_object_subscriber_`` - 收听话题发布的关于在规划场景中添加、删除或修改碰撞对象的 :moveit_msgs:`CollisionObject` 消息，并将其传递到自身监视的规划场景中。
 * ``planning_scene_world_subscriber_`` - 收听话题发布的 :moveit_msgs:`PlanningSceneWorld` 消息，该消息可能包含碰撞对象信息以及 octomap 信息。这对于使规划场景监视器 (planning scene monitor) 保持同步非常有用。
 * ``attached_collision_object_subscriber_`` - 收听话题发布的 :moveit_msgs:`AttachedCollisionObject` 消息，这些消息决定了在 robot state 里附加/分离物体到 link 。

PlanningSceneMonitor 有以下服务 (service):

 * ``get_scene_service_`` - （可选）获取完整的规划场景状态。

PlanningSceneMonitor 初始化为：

 * ``startSceneMonitor`` - 启动 ``planning_scene_subscriber_``,
 * ``startWorldGeometryMonitor`` - 启动 ``collision_object_subscriber_``,  ``planning_scene_world_subscriber_``, 和 OccupancyMapMonitor,
 * ``startStateMonitor`` - 启动 CurrentStateMonitor 和 ``attached_collision_object_subscriber_``,
 * ``startPublishingPlanningScene`` - 开启一个线程以在一个被指定的话题上发布整个规划场景信息，以被其他的 PlanningSceneMonitors 订阅。
 * ``providePlanningSceneService`` - 启动 ``get_scene_service_``.

PlanningSceneInterface 规划场景接口
------------------------------------------------------------------
:planning_interface:`PlanningSceneInterface` 是一个有用的类，通过 C++ API 将更新发布到 MoveGroup 的 :planning_scene_monitor:`PlanningSceneMonitor` 中，而无需创建自己的 subscriber 和 service 客户端。它可能无法在没有 MoveGroup 或 MoveItCpp 的情况下工作。
