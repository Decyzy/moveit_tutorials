在 RViz 中快速入门 MoveIt
==========================
.. image:: rviz_plugin_head.png
   :width: 700px

本教程通过 RViz 和 MoveIt 插件来让你快速进行运动规划。Rviz 是 ROS 中主要的可视化工具，对于调试机器人来说非常有用。MoveIt 的 Rviz 插件允许你设置虚拟环境（场景），交互地设置机器人的初始和目标状态，测试各种运动规划方案，并可视化输出。开始吧!

开始
---------------
请先确保你已经完成了 `入门 <../getting_started/getting_started.html>`__ 。

步骤 1: 启动 Demo 并配置插件
------------------------------------------------

* 启动 demo: ::

   roslaunch panda_moveit_config demo.launch rviz_tutorial:=true

* 如果是初次打开，你应该会在 RViz 里看到一个空的场景。接下来需要为其添加在运动规划插件（Motion Planning Plugin）：

  * 你应该会在 RViz 里看到一个空的场景：

  |A|

  * 在 RViz 的 Displays 子窗口的底部选项卡里, 点击 *Add*：

  |B|

  * 选择 By display type 选项卡，在 moveit_ros_visualization 文件夹下, 选择 "MotionPlanning" 。 点击 "Ok"。

  |C|

  * 现在你应该能在 RViz 中看到 Panda 机械臂了：

  |D|

.. |A| image:: rviz_empty.png
               :width: 700px

.. |B| image:: rviz_click_add.png
               :width: 405px

.. |C| image:: rviz_plugin_motion_planning_add.png
               :width: 300px

.. |D| image:: rviz_start.png
               :width: 700px

* 一旦你加载了运动规划插件，我们就可以配置它了。在 "Displays" 子窗口中的 "Global Options" 选项卡中，将 **Fixed Frame** 字段设置为 ``/panda_link0`` 。

* 现在，你可以开始为你的机器人（本例中是 Panda 机械臂）配置插件了。点击"Displays" 子窗口中的 "MotionPlanning" 。

  * 确保 **Robot Description** 字段被设置为 ``robot_description`` 。

  * 确保 **Planning Scene Topic** 字段被设置为 ``move_group/monitored_planning_scene`` 。
    通过点击该字段的值来显示下拉菜单，从而选择正确的话题名称。

  * 确保 **Planned Path** 下的 **Trajectory Topic** 被设置为 ``/move_group/display_planned_path`` 。

  * 在 **Planning Request** 中， 将 **Planning Group** 修改为 ``panda_arm`` 。 你也可以在左下角的 MotionPlanning 面板中的 Planning 选项卡下看到这一点。


.. image:: rviz_plugin_start.png
   :width: 700px


步骤 2: 控制可视化机器人
---------------------------------------
这里将出现四种不同的、可能互相重叠的显示效果：

#. 在 ``move_group/monitored_planning_scene`` 规划环境中的机器人配置（默认显示）。

#. 为机器人规划好的路径（默认显示）。

#. 绿色：运动规划的起始状态（默认隐藏）。

#. 橙色: 运动规划的目标状态（默认显示）。

每一种可视化的显示状态都可以使用复选框来开启和关闭：

#. 场景规划机器人，通过 **Scene Robot** 项下的 **Show Robot Visual** 复选框控制。

#. 规划好的路径，通过 **Planned Path** 项下的 **Show Robot Visual** 复选框控制。

#. 起始状态， 通过 **Planning Request** 项下的 **Query Start State** 复选框控制。

#. 目标状态，通过 **Planning Request** 项下的 **Query Goal State** 复选框控制。

* 试一试开关这些复选框来展现不同的可视化效果。

.. image:: rviz_plugin_visualize_robots.png
   :width: 700px

步骤 3： 和 Panda 机械臂交互
-------------------------------

在接下来的步骤中，我们只需要场景机器人、起始状态和目标状态：

#. 勾选 **Planned Path** 项下的 **Show Robot Visual** 复选框。

#. 取消勾选 **Scene Robot** 项下的 **Show Robot Visual** 复选框。

#. 勾选 the **Planning Request** 项下的 **Query Goal State** 复选框。

#. 勾选 the **Planning Request** 项下的 **Query Start State** 复选框。

现在应该有两个交互式标记。现在应该有两个交互式标记。与橙色机械臂对应的那个是用来设置运动规划的“目标状态”；另一个与绿色手臂对应，用于设置运动规划的“起始状态”。如果没有看到交互标记，按下 RViz 顶部菜单中的 **Interact** （注意：有些工具可能是隐藏的，按下顶部菜单中的 **"+"** ，添加如下所示的 **Interact** 工具）。

.. image:: rviz_plugin_interact.png
   :width: 700px

现在，你应该能够使用这些标记来拖动机械臂并改变它的方向了。试他一试！

当移动至碰撞状态时
+++++++++++++++++++++

看看当你试图移动其中一个机械臂与另一个机械臂相碰撞时会发生什么。碰撞中的两个 links 将变成红色。

.. image:: rviz_plugin_collision.png
   :width: 700px

MotionPlanning 插件中 Planning 选项卡下的 "Use Collision-Aware IK （使用碰撞感知的逆运动学）" 复选框允许您切换 IK 求解器的模式。当复选框被选中时，求解器将继续尝试为期望的末端执行器姿态找到一个无碰撞的解。当这个复选框没有被选中时，求解器将允许得到会发生碰撞和干涉的解。无论复选框的状态如何，被碰撞的 link 将始终显示为红色。

.. image:: rviz_plugin_collision_aware_ik_checkbox.png
   :width: 700px

当试图移动至机械臂工作空间以外时
+++++++++++++++++++++++++++++++++

看看当你试图将机械臂的末端执行器移出其可到达的工作空间时会发生什么。

.. image:: rviz_plugin_invalid.png
   :width: 700px

Moving Joints or in Null Space
++++++++++++++++++++++++++++++
你可以使用 **Joints** 选项卡来移动单个关节以及7自由度机器人的冗余关节。尝试移动 "null space exploration" 滑块，将如下面的动画所示。

.. raw:: html

    <video width="700px" nocontrols="true" autoplay="true" loop="true">
        <source src="../../_static/rviz_joints_nullspace.webm" type="video/webm">
        The joints moving while the end effector stays still
    </video>


步骤 4: 给 Panda 机械臂做运动规划
-------------------------------------------------

* 现在，你可以开始在 MoveIt RViz插件里为 Panda 机械臂做运动规划。

  * 移动起始状态（绿色）至需要的位置。

  * 移动目标状态（橙色）至需要的位置。

  * 确保上述两个状态都没有发生自碰撞。

  * 确保规划好的路径是显示状态： displays 子窗口内 **Planned Path** 选项下的 **Show Trail** 复选框应该是勾选状态。

* 在 **MotionPlanning** 窗口下的 **Planning** 选项卡里, 点击 **Plan** 按钮。你应该能看到一个机械臂移动和其移动踪迹的可视化效果。

.. image:: rviz_plugin_planned_path.png
   :width: 700px

检查轨迹路线点
++++++++++++++++++++++++++++++++++

你可以在 RViz 中逐点查看轨迹。

* 从 "`Panels`" 菜单里, 选择 "`MotionPlanning - Slider`" 。你将在 RViz 里看到一个带滑动条的新面板。

* 设置你想要的目标姿态，然后点击 `Plan` 按钮以运行。

* 尝试操作 "`Slider`" 面板, 比如移动滑块，点击 "`Play`" 按钮等。

注意: 一旦你将末端执行器移动至一个新的目标位姿，请确保先点击运行 `Plan` ，然后再点击运行 `Play` -- 否则你将会看到为前一个目标位姿（如果可用的话）规划的路径。

.. image:: rviz_plugin_slider.png
   :width: 700px

笛卡尔运动规划
++++++++++++++++++++++

如果勾选了 "Use Cartesian Path （使用笛卡尔路径） " 复选框，则机器人将尝试在笛卡儿空间内线性移动末端执行器。

.. image:: rviz_plan_free.png
   :width: 700px

.. image:: rviz_plan_cartesian.png
   :width: 700px


沿轨迹运动和速度调整
+++++++++++++++++++++++++++++++++++++++

在一个成功进行了轨迹规划后，点击 "Plan & Execute" 或 "Execute" 按钮将会将轨迹发送给机器人 - 在本教程中，因为用的是 `demo.launch` ，因此机器人是仿真环境下的。

初始状态下，默认速度和加速度都被设置为机器人对应最大值的 10% (`0.1`) 。你可以在下图里的 Planning 选项卡中更改这些缩放因子，或者更改机器人的 `moveit_config` (在 `joint_limits.yaml` 文件中)里的这些默认值。

.. image:: rviz_plugin_collision_aware_ik_checkbox.png
   :width: 700px


下一步
----------

RViz Visual Tools
+++++++++++++++++

许多教程使用 ``moveit_visual_tools`` 来进行逐步演示。在继续下一个教程之前，建议启用 **RvizVisualToolsGui** 。

从 "`Panels`" 菜单里, 选择 "`RvizVisualToolsGui`" 。你会在 RViz 里看到一个新添加的面板。

.. image:: rviz_panels.png
   :width: 700px

保存配置
+++++++++++++++++++++++++
RViz 提供了 ``File->Save Config`` 来保存界面布局等配置。在继续学习下一个教程之前，你应该保存一下。

接下来的教程
++++++++++++++
* 要使用 c++ 来轻松地控制机器人，请查看 `移动组 (Move Group) 的 C++ 接口 <../move_group_interface/move_group_interface_tutorial.html>`_

* 要使用 Python 来轻松地控制机器人，请查看 `移动组 (Move Group) 的 Python 接口 <../move_group_python_interface/move_group_python_interface_tutorial.html>`_

* 要创建你自己的 ``*_moveit_config`` 包，请查看 `设置助手 <../setup_assistant/setup_assistant_tutorial.html>`_
