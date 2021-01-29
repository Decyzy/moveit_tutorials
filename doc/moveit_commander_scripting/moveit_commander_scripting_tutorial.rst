MoveIt 命令行脚本
===========================
.. image:: moveit_commander_scripting.png
   :width: 700px

Python 包 `moveit_commander <http://wiki.ros.org/moveit_commander>`_ 封装了 MoveIt 的功能。其简单的交互接口可用于运动规划，笛卡尔路径的计算以及物体拾取和放置。 ``moveit_commander`` 同时也包含一个命令行接口，即 ``moveit_commander_cmdline.py`` 。

开始
---------------
请先确保已经完成了这些步骤 `入门 <../getting_started/getting_started.html>`__ 里的步骤。

启动 RViz 和命令行工具
---------------------------------------
打开两个 shell 。在第一个 shell 中启动 RViz ，并等待所有加载工作完成： ::

  roslaunch panda_moveit_config demo.launch

然后在第二个 shell 里初始化 ``moveit_commander`` 接口： ::

 rosrun moveit_commander moveit_commander_cmdline.py

使用 MoveIt 命令行工具
---------------------------------------------
您输入的第一个命令应该是： ::

 use panda_arm

这里的 ``panda_arm`` 是你想要操作的 planning group 名字。这将会连接到正在运行的 move_group 节点实例。 现在，您可以在该 planning group 上执行命令。

接下来的这个 ``current`` 命令，将向您显示当前 planning group 的状态： ::

 current

要以特定名称记录该状态，您只需键入： ::

 rec c

这将把当前 planning group 里的各关节值记录在名字 ``c`` 里面。可以使用类似 Matlab 的语法来修改关节值。

要想使机器人移动，您可以输入： ::

 goal = c
 goal[0] = 0.2
 go goal

上面的代码将 ``c`` 里的值复制到一个名为 ``goal`` 的新变量里。我们在 ``goal`` 里把第一个关节值修改为 ``0.2`` 。您可能需要使用其他值而不是 ``0.2`` （值必须在容许范围内，且不会引起碰撞） 。命令 ``go`` 将规划一次运动路径并执行。

除了调用 ``go`` 命令，您也可以这样输入： ::

 goal[0] = 0.2
 goal[1] = 0.2
 plan goal
 execute

这样做效率有点低，但好处是通过使用 ``plan`` 命令，可以在实际发出执行命令之前在 RViz 中可视化规划出来的运动路径。 

您可以键入 ``help`` 来查看受支持的命令列表。想要退出 ``moveit_commander`` 交互接口，您可以输入 ``quit`` 。
