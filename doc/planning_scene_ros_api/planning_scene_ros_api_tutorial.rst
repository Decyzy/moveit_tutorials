规划场景 PlanningScene 的 ROS API
========================================================

在本教程中，我们将研究如何执行两个操作来变更 Planning Scene：

 * 向 world 添加或移除对象；
 * 将物体附加到 world 里或从 world 里分离。

开始
---------------
请先确保已经完成了 `入门 <../getting_started/getting_started.html>`__ 里的步骤。

运行代码
----------------
打开两个 shell 。在第一个 shell 中启动 RViz ，并等待所有加载工作完成： ::

  roslaunch panda_moveit_config demo.launch

在另一个 shell 里使用为本示例启动 launch 文件： ::

  roslaunch moveit_tutorials planning_scene_ros_api_tutorial.launch

**注意:** 本教程使用 **RvizVisualToolsGui** 面板来逐步运行此 demo。想要在 RViz里添加这个面板，请按照 `可视化指南 <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html#rviz-visual-tools>`_ 中的说明。

RViz 窗口过一会儿就会出现，看起来和本页面顶部的窗口差不多。想要依次查看每个演示步骤，要么按下窗口底部 **RvizVisualToolsGui** 面板里的 **Next** 按钮，或者在 RViz 窗口聚焦状态下，选择窗口顶部 **Tools** 面板下的 **Key Tool** ，然后按下键盘上的 **N** 。

预期输出
---------------
在 RViz 里，我们应该能看到以下效果：

 * 物体出现在了规划场景里； 
 * 物体固连到了机器人上；
 * 物体从机器人上脱离；
 * 物体被从规划场景里移除。

.. role:: red

**注意:** 您可能会看到一条显示 :red:`Found empty JointState message` 的错误信息，这是一个已知的错误，将尽快修复。

整个代码
---------------
全部代码可以在 :codedir:`MoveIt GitHub project<planning_scene_ros_api>` 里找到。

.. tutorial-formatter:: ./src/planning_scene_ros_api_tutorial.cpp

launch 文件
---------------
整个 launch 文件在 GitHub 上的 :codedir:`这里 <planning_scene_ros_api/launch/planning_scene_ros_api_tutorial.launch>` 可见。本教程中的所有代码都可从 **moveit_tutorials** 包中运行，这个包是 MoveIt 安装的一部分。

调试和规划场景监视器
------------------------------------
为了帮助调试物体分离和固连，以下命令行工具可以帮助你检查： ::

  rosrun moveit_ros_planning moveit_print_planning_scene_info
