机器人模型和机器人状态 
===================================

.. image:: panda_tf.png
   :width: 700px
   :alt: "That is a sweet robot" you might say.

在本节中，我们将引导您逐步了解在 MoveIt 中运动学的 C ++ API。

RobotModel 类和 RobotState 类
----------------------------------------------------
:moveit_core:`RobotModel` 类和 :moveit_core:`RobotState` 类是您使用机器人运动学的两个核心类。

:moveit_core:`RobotModel` 类包含了所有 link 和关节之间的关系，包括从 URDF 加载的关节限位信息。RobotModel 类还将机器人的 link 和关节从 SRDF 文件里定义的 planning group 里分开。关于 URDF 和 SRDF 在 `URDF 和 SRDF 教程 <../urdf_srdf/urdf_srdf_tutorial.html>`__ 里有单独介绍。

:moveit_core:`RobotState` 类包含了特定时间点上和机器人有关的信息，并将关节位置以及可选的速度和加速度储存在一些 vector 里。此信息可用于获取机器人的运动学信息，其由机器人的当前状态决定，例如末端执行器的雅可比行列式。

RobotState 类还包含一些辅助程序，用于根据末端执行器位置（笛卡尔姿态）设置机械臂位置并计算笛卡尔轨迹。

在本示例中，我们将逐步介绍如何将这些类在 Panda 机器人上使用。

开始
---------------
请先确保已经完成了 `入门 <../getting_started/getting_started.html>`__ 里的步骤。

运行代码
----------------
本教程中所有的代码都可从 MoveIt 安装程序里包含的``moveit_tutorials`` 包里编译运行。

使用 roslaunch 启动 launch 文件以直接运行 moveit_tutorials 里的代码： ::

 roslaunch moveit_tutorials robot_model_and_robot_state_tutorial.launch

预期输出
---------------
预期的输出将会是以下形式。因为我们使用了随机的关节值，因此数字可能和您看到的有出入： ::

 ros.moveit_tutorials: Model frame: /panda_link0
 ros.moveit_tutorials: Joint panda_joint1: 0.000000
 ros.moveit_tutorials: Joint panda_joint2: 0.000000
 ros.moveit_tutorials: Joint panda_joint3: 0.000000
 ros.moveit_tutorials: Joint panda_joint4: 0.000000
 ros.moveit_tutorials: Joint panda_joint5: 0.000000
 ros.moveit_tutorials: Joint panda_joint6: 0.000000
 ros.moveit_tutorials: Joint panda_joint7: 0.000000
 ros.moveit_tutorials: Current state is not valid
 ros.moveit_tutorials: Current state is valid
 ros.moveit_tutorials: Translation:
 -0.541498
 -0.592805
  0.400443

 ros.moveit_tutorials: Rotation:
 -0.395039  0.600666 -0.695086
  0.299981 -0.630807 -0.715607
 -0.868306 -0.491205 0.0690048

 ros.moveit_tutorials: Joint panda_joint1: -2.407308
 ros.moveit_tutorials: Joint panda_joint2: 1.555370
 ros.moveit_tutorials: Joint panda_joint3: -2.102171
 ros.moveit_tutorials: Joint panda_joint4: -0.011156
 ros.moveit_tutorials: Joint panda_joint5: 1.100545
 ros.moveit_tutorials: Joint panda_joint6: 3.230793
 ros.moveit_tutorials: Joint panda_joint7: -2.651568
 ros.moveit_tutorials: Jacobian:
     0.592805   -0.0500638    -0.036041     0.366761   -0.0334361     0.128712 -4.33681e-18
    -0.541498   -0.0451907    0.0417049    -0.231187    0.0403683   0.00288573  3.46945e-18
            0    -0.799172    0.0772022    -0.247151    0.0818336    0.0511662            0
            0     0.670056    -0.742222     0.349402    -0.748556    -0.344057    -0.695086
            0     -0.74231    -0.669976    -0.367232    -0.662737     0.415389    -0.715607
            1  4.89669e-12    0.0154256     0.862009     0.021077     0.842067    0.0690048

**注意:** 如果您的输出具有不同的 ROS 控制台日志格式，请不要担心。您可以查看 `这篇博文 <http://dav.ee/blog/notes/archives/898>`__ 来自定义 ROS 控制台日志。

整个代码
---------------
整个代码见 :codedir:`MoveIt GitHub project<robot_model_and_robot_state/src/robot_model_and_robot_state_tutorial.cpp>` 。

.. tutorial-formatter:: ./src/robot_model_and_robot_state_tutorial.cpp

Launch 文件
^^^^^^^^^^^^^^^
要运行代码，您将需要一个包含执行以下两项操作的 launch 文件：

 * 将 Panda 机器人的 URDF 文件和 SRDF 文件加载到参数服务器上；
 * 将由 MoveIt 配置助手生成的 kinematics_solver 配置放到 ROS 参数服务器上，并处于本教程中的节点实例的命名空间下。

.. literalinclude:: ./launch/robot_model_and_robot_state_tutorial.launch

调试机器人的状态
-------------------------
为了帮助调试机器人状态 ，以下命令行工具将帮你检查： ::

  rosrun moveit_ros_planning moveit_print_planning_model_info
