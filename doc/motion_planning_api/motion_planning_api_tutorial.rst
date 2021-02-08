运动规划 Motion Planning 的 API
==================================
.. image:: motion_planning_api_tutorial_robot_move_arm_1st.png
   :width: 700px

MoveIt 以插件的形式来加载运动规划器 motion planner 。 这使 MoveIt 可以在运行时加载运动规划器。 在此示例中，我们将通过 C++ 代码来执行此操作。

开始
---------------
请先确保已经完成了 `入门 <../getting_started/getting_started.html>`__ 里的步骤。

运行 Demo
----------------
打开两个 shell 。在第一个 shell 中启动 RViz ，并等待所有加载工作完成： ::

  roslaunch panda_moveit_config demo.launch

在第二个 shell 里启动 launch 文件： ::

  roslaunch moveit_tutorials motion_planning_api_tutorial.launch

**注意:** 本教程使用 **RvizVisualToolsGui** 面板来逐步完成演示。 要将此面板添加到 RViz ，请参考 `可视化教程 <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html#rviz-visual-tools>`_ 。

RViz 窗口过一会儿就会出现，看起来和本页面顶部那张截图差不多。想要依次查看每个演示步骤，要么按下窗口底部 **RvizVisualToolsGui** 面板里的 **Next** 按钮，或者在 RViz 窗口聚焦状态下，选择窗口顶部 **Tools** 面板下的 **Key Tool** ，然后按下键盘上的 **N** 。

预期效果
---------------
在 RViz 中，我们最终应该能够看到四个重复播放的轨迹：


 1. 机器人将手臂移动到第一个目标位姿，

    |A|

 2. 机器人将手臂移动到目标关节位置，

    |B|

 3. 机器人将手臂移动最初的位姿，
 4. 机器人在保持末端执行器水平的情况下，将手臂移动到新的目标位姿。

    |C|

.. |A| image:: motion_planning_api_tutorial_robot_move_arm_1st.png
               :width: 200px
.. |B| image:: motion_planning_api_tutorial_robot_move_arm_2nd.png
               :width: 200px
.. |C| image:: motion_planning_api_tutorial_robot_move_arm_3rd.png
               :width: 200px

整个代码
---------------
全部代码可以在 :codedir:`moveit_tutorials GitHub project<motion_planning_api>` 里找到。接下来，我们逐步分析代码来解释其功能。

.. tutorial-formatter:: ./src/motion_planning_api_tutorial.cpp

Launch 文件
---------------
整个 launch 文件在 GitHub 上的 :codedir:`here <motion_planning_api/launch/motion_planning_api_tutorial.launch>` 可见。本教程中的所有代码都可从 **moveit_tutorials** 包中运行，这个包是 MoveIt 安装的一部分。
