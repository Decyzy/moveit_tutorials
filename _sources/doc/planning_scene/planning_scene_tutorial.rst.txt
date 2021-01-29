规划场景 (PlanningScene)
========================================
:planning_scene:`PlanningScene` 类提供了用于干涉和约束检查的主要接口。在本教程中，我们将探索此类的 C++ 接口。

开始
---------------
请先确保已经完成了 `入门 <../getting_started/getting_started.html>`__ 里的步骤。

整个代码
---------------
整个代码见 :codedir:`here in the MoveIt GitHub project<planning_scene>` 。

.. tutorial-formatter:: ./src/planning_scene_tutorial.cpp

launch 文件
---------------
整个 launch 文件在 GitHub 上的 :codedir:`这里<planning_scene/launch/planning_scene_tutorial.launch>` 可见。本教程中的所有代码都可从 **moveit_tutorials** 包中运行，这个包是 MoveIt 安装的一部分。

运行代码
----------------
使用 roslaunch 启动 launch 文件以直接运行 moveit_tutorials 包里的代码： ::

 roslaunch moveit_tutorials planning_scene_tutorial.launch

预期输出
---------------

预期的输出将会是以下形式。因为我们使用了随机的关节值，所以数字可能和您看到的有出入： ::

 ros.moveit_tutorials: Test 1: Current state is not in self collision
 ros.moveit_tutorials: Test 2: Current state is not in self collision
 ros.moveit_tutorials: Test 3: Current state is not in self collision
 ros.moveit_tutorials: Test 4: Current state is valid
 ros.moveit_tutorials: Test 5: Current state is in self collision
 ros.moveit_tutorials: Contact between: panda_leftfinger and panda_link1
 ros.moveit_tutorials: Contact between: panda_link1 and panda_rightfinger
 ros.moveit_tutorials: Test 6: Current state is not in self collision
 ros.moveit_tutorials: Test 7: Current state is not in self collision
 ros.moveit_tutorials: Test 8: Random state is not constrained
 ros.moveit_tutorials: Test 9: Random state is not constrained
 ros.moveit_tutorials: Test 10: Random state is not constrained
 ros.moveit_tutorials: Test 11: Random state is feasible
 ros.moveit_tutorials: Test 12: Random state is not valid

**注意:** 如果您的输出具有不同的 ROS 控制台日志格式，请不要担心。您可以查看 `这篇博文 <http://dav.ee/blog/notes/archives/898>`__ 来自定义 ROS 控制台日志。
