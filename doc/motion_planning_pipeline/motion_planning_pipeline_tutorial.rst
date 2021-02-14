运动规划管道 Motion Planning Pipeline
========================================================

在 MoveIt 中，运动规划器 motion planner 用于规划路径。然而，我们经常需要对运动规划请求进行预处理或后处理(例如用于时间参数化)。在这种情况下，我们使用规划管道  planning pipeline ，其将一个运动规划器与预处理和后处理阶段连接起来。所谓的前处理和后处理阶段（称为规划请求适配器 planning request adapter ），可以在 ROS 的参数服务器上按名称配置。在本教程中，我们将让您运行 c++ 代码来实例化和调用这样的 Planning Pipeline 。

开始
---------------
请先确保已经完成了 `入门 <../getting_started/getting_started.html>`__ 里的步骤。

运行代码
----------------
打开两个 shell 。在第一个 shell 中启动 RViz ，并等待所有加载工作完成： ::

  roslaunch panda_moveit_config demo.launch

在第二个 shell 里，运行 launch 文件： ::

 roslaunch moveit_tutorials motion_planning_pipeline_tutorial.launch

**注意:** 本教程使用 **RvizVisualToolsGui** 面板来逐步完成演示。 要将此面板添加到 RViz ，请参考 `可视化教程 <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html#rviz-visual-tools>`_ 。

RViz 窗口过一会儿就会出现，看起来和本页面顶部那张截图差不多。想要依次查看每个演示步骤，要么按下窗口底部 **RvizVisualToolsGui** 面板里的 **Next** 按钮，或者在 RViz 窗口聚焦状态下，选择窗口顶部 **Tools** 面板下的 **Key Tool** ，然后按下键盘上的 **N** 。

预期效果
---------------
在 RViz 里，我们最终应该能看到以下三个效果：

 1. 机器人移动它的右臂到其前方的一个目标位姿处，
 2. 机器人移动它的右臂到其一边的一个目标关节位置处，
 3. 机器人移动它的右臂到其前方的初始位姿处。

整个代码
---------------
全部代码可以在 :codedir:`MoveIt GitHub project<motion_planning_pipeline>` 里找到。

.. tutorial-formatter:: ./src/motion_planning_pipeline_tutorial.cpp

Launch 文件
---------------
整个 launch 文件在 GitHub 上的 :codedir:`这里<motion_planning_pipeline/launch/motion_planning_pipeline_tutorial.launch>` 可见。本教程中的所有代码都可从 **moveit_tutorials** 包中运行，这个包是 MoveIt 安装的一部分。

