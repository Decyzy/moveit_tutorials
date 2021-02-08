移动群 Move Group 的 C++ 接口
==================================
.. image:: move_group_interface_tutorial_start_screen.png
   :width: 700px

在 MoveIt 里，最简单的用户接口是 :planning_interface:`MoveGroupInterface` 类。这个类为用户想要执行的大多数操作提供了简便的功能，特别是设置关节或目标姿态，创建运动规划，移动机器人，添加对象到环境里和附加/分离对象到机器人上。此接口借助 ROS 的 topics 、services 和 actions 来和 `MoveGroup 节点 <http://docs.ros.org/noetic/api/moveit_ros_move_group/html/annotated.html>`_ 通信。

观看这个简短的 `YouTube 演示视频 <https://youtu.be/_5siHkFQPBQ>`__ ，看看 Move Group 接口的能力吧！

开始
---------------
请先确保已经完成了 `入门 <../getting_started/getting_started.html>`__ 里的步骤。

运行代码
----------------
打开两个 shell 。在第一个 shell 中启动 RViz ，并等待所有加载工作完成： ::

  roslaunch panda_moveit_config demo.launch

在第二个 shell 里，运行 launch 文件： ::

  roslaunch moveit_tutorials move_group_interface_tutorial.launch

**注意:** 本教程使用 **RvizVisualToolsGui** 面板来逐步完成演示。 要将此面板添加到 RViz ，请参考 `可视化教程 <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html#rviz-visual-tools>`_ 。

RViz 窗口过一会儿就会出现，看起来和本页面顶部那张截图差不多。想要依次查看每个演示步骤，要么按下窗口底部 **RvizVisualToolsGui** 面板里的 **Next** 按钮，或者在 RViz 窗口聚焦状态下，选择窗口顶部 **Tools** 面板下的 **Key Tool** ，然后按下键盘上的 **N** 。

预期效果
---------------
可以通过本页教程顶部的 `YouTube 视频 <https://youtu.be/_5siHkFQPBQ>`__ 来查看预期效果。在 RViz 里，我们应该能看到以下效果：

 1. 机器人将手臂移动到前面的目标位置。
 2. 机器人将其手臂移动到其一侧的目标关节位置处。
 3. 在保持末端执行器水平的同时，机器人将其手臂移动至一个新的目标姿态。
 4. 机器人沿着期望的笛卡尔路径移动手臂(沿着下、右、前左方的一个三角形路径)。
 5. 将一个 box 对象添加到手臂右侧的环境中。
    |B|

 6. 机器人将手臂移动到目标位置，同时避免了与 box 碰撞。
 7. 将一个物体固连到了手腕(其颜色将依次变为紫色、橙色、绿色)。
 8. 该物体与腕部分离(其颜色将恢复为绿色)。
 9. 将该物体从环境中移除。

.. |B| image:: ./move_group_interface_tutorial_robot_with_box.png

整个代码
---------------
全部代码可以在 :codedir:`MoveIt GitHub project<move_group_interface/src/move_group_interface_tutorial.cpp>` 里找到。接下来，我们逐步分析代码来解释其功能。

.. tutorial-formatter:: ./src/move_group_interface_tutorial.cpp

Launch 文件
---------------
整个 launch 文件在 GitHub 上的 :codedir:`这里<move_group_interface/launch/move_group_interface_tutorial.launch>` 可见。本教程中的所有代码都可从 **moveit_tutorials** 包中运行，这个包是 MoveIt 安装的一部分。


关于精度设置的说明
----------------------------
注意 `MoveGroupInterface <http://docs.ros.org/noetic/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html>`_ 中的 `setGoalTolerance()` 及其相关方法是为 **规划过程（planning）** 设置精度，而不是设置执行过程（execution）的精度。

如果你想配置执行过程中的精度，且使用了一个 FollowJointTrajectory 控制器（controller），你必须编辑 `controller.yaml` 文件，或者手动将其添加到从规划器（planner）生成的轨迹消息中。
