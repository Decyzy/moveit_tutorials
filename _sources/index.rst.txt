MoveIt 中文教程（个人翻译版）
=========================================

这些教程将使用 MoveIt 运动规划快速地为您和您的机器人提供帮助。

.. image:: doc/quickstart_in_rviz/rviz_plugin_head.png
   :width: 700px

在本教程中，我们使用 Franka Emika 的 Panda 机器人作为快速入门的演示案例。另外，你可以轻松使用已配置为可与 MoveIt 配合使用的任何机机械臂 - 通过 `已经运行 MoveIt 的机器人列表 <http://moveit.ros.org/robots/>`_ 来查看 MoveIt 是否已可用于您的机器人。否则，您可以在之后的 "与新机器人集成" 章节中设置 MoveIt ，从而用于您的自定义机器人。

MoveIt 和 RViz 入门
-------------------------------------
.. toctree::
   :maxdepth: 1

   doc/getting_started/getting_started
   doc/quickstart_in_rviz/quickstart_in_rviz_tutorial

MoveGroup - C++ 和 Python 封装的 ROS 接口
------------------------------------------------------------
通过脚本使用 MoveIt 的最简单方法是使用 ``move_group_interface`` 。该接口非常适合初学者，并提供了对 MoveIt 的许多功能的统一访问。

.. toctree::
   :maxdepth: 1

   doc/move_group_interface/move_group_interface_tutorial
   doc/move_group_python_interface/move_group_python_interface_tutorial
   doc/moveit_commander_scripting/moveit_commander_scripting_tutorial

通过 C ++ API 直接使用 MoveIt
---------------------------------------------------
使用 MoveIt 构建更复杂的应用程序通常需要开发人员深入研究 MoveIt 的 C ++ API 。另外，使用 C++ API 会直接跳过许多 ROS 服务或操作层，从而显着提高性能。

.. toctree::
   :maxdepth: 1

   doc/robot_model_and_robot_state/robot_model_and_robot_state_tutorial
   doc/planning_scene/planning_scene_tutorial
   doc/planning_scene_monitor/planning_scene_monitor_tutorial
   doc/planning_scene_ros_api/planning_scene_ros_api_tutorial
   doc/motion_planning_api/motion_planning_api_tutorial
   doc/motion_planning_pipeline/motion_planning_pipeline_tutorial
   doc/creating_moveit_plugins/plugin_tutorial
   doc/visualizing_collisions/visualizing_collisions_tutorial
   doc/time_parameterization/time_parameterization_tutorial
   doc/planning_with_approximated_constraint_manifolds/planning_with_approximated_constraint_manifolds_tutorial
   doc/pick_place/pick_place_tutorial
   doc/moveit_grasps/moveit_grasps_tutorial
   doc/moveit_task_constructor/moveit_task_constructor_tutorial
   doc/moveit_deep_grasps/moveit_deep_grasps_tutorial
   doc/subframes/subframes_tutorial
   doc/moveit_cpp/moveitcpp_tutorial
   doc/bullet_collision_checker/bullet_collision_checker

与新机器人集成
----------------------------
在尝试将新机器人与 MoveIt 集成之前，请检查是否已有设置好的机器人 (查看 `已经运行 MoveIt 机器人列表 <http://moveit.ros.org/robots/>`_)。否则，请按照本节中的教程将您的机器人与 MoveIt 集成在一起（并可在 MoveIt 邮件列表中分享您的结果）

.. toctree::
   :maxdepth: 1

   doc/setup_assistant/setup_assistant_tutorial
   doc/urdf_srdf/urdf_srdf_tutorial
   doc/controller_configuration/controller_configuration_tutorial
   doc/perception_pipeline/perception_pipeline_tutorial
   doc/hand_eye_calibration/hand_eye_calibration_tutorial
   doc/ikfast/ikfast_tutorial
   doc/trac_ik/trac_ik_tutorial

配置
-------------
.. toctree::
   :maxdepth: 1

   doc/kinematics_configuration/kinematics_configuration_tutorial
   doc/custom_constraint_samplers/custom_constraint_samplers_tutorial
   doc/ompl_interface/ompl_interface_tutorial
   doc/chomp_planner/chomp_planner_tutorial
   doc/stomp_planner/stomp_planner_tutorial
   doc/trajopt_planner/trajopt_planner_tutorial
   doc/pilz_industrial_motion_planner/pilz_industrial_motion_planner
   doc/planning_adapters/planning_adapters_tutorial.rst

其他
----------------------------

.. toctree::
   :maxdepth: 1

   doc/joystick_control_teleoperation/joystick_control_teleoperation_tutorial
   doc/realtime_servo/realtime_servo_tutorial
   doc/benchmarking/benchmarking_tutorial
   doc/tests/tests_tutorial

贡献
-----------

MoveIt 教程的主要贡献者按时间顺序列出为：Sachin Chitta，Dave Hershberger，Acorn Pooley，Dave Coleman，Michael Gorner，Francisco Suarez，Mike Lautman。请帮助我们改善这些文档，我们很乐意将您也包括在这里面！

在 Franka Emika 与 PickNik 合作赞助的 code sprint 中，这些教程在 2018 年进行了重大更新。 (`查看博客文章！ <http://moveit.ros.org/moveit!/ros/2018/02/26/tutorials-documentation-codesprint.html>`_)

.. image:: ./_static/franka_logo.png
   :width: 300px
