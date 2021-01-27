移动组 (Move Group) 的 Python 接口
================================================
.. image:: move_group_python_interface.png
   :width: 700px

最简单的 MoveIt 用户接口之一是基于 python 的移动组 (Move Group) 接口。这些 python 包装 (wrappers) 为用户想要执行的大多数操作提供了简便的功能，特别是设置关节或目标姿态，创建运动规划，移动机器人，添加对象到环境里和附加/分离对象到机器人上。

观看这个简短的 `YouTube 演示视频 <https://youtu.be/3MA5ebXPLsc>`_ 看看 Move Group 的 python 接口的能力吧!

开始
---------------
请先确保已经完成了这些步骤 `入门 <../getting_started/getting_started.html>`_ 里的步骤。

打开 RViz 和 MoveGroup 节点
-----------------------------
打开两个 shell 。在第一个 shell 中启动 RViz ，并等待所有加载工作完成： ::

  roslaunch panda_moveit_config demo.launch

现在直接在另一个 shell 里使用 ``rosrun`` 运行 Python 代码。注意在某些情况下，你可能需要让 python 脚本拥有可执行权限：::

 rosrun moveit_tutorials move_group_python_interface_tutorial.py

预期效果
---------------
在 RViz 里，我们应该能看到以下效果：

当你在 shell 终端里执行完 ``rosrun`` 命令后，在每个步骤间敲 *<回车>* 。
 #. 机器人规划路径，并将手臂移动到目标关节位置处。
 #. 机器人规划了一条到目标位姿的路径。
 #. 机器人规划了一条笛卡尔路径。
 #. 机器人再次演示了笛卡尔路径规划.
 #. 机器人执行了笛卡尔路径规划.
 #. 一个 box 出现在 Panda 的末端执行器位置处。
 #. box 的颜色变了，以显示其当前被固连了。
 #. 机器人在保持 box 固连的情况下，规划并执行了一个笛卡尔路径。
 #. box 的颜色再次变了，以显示其当前脱离连接了。
 #. box 消失。

整个代码
---------------
注意: 整个代码见 :codedir:`本教程的 GitHub 仓库<move_group_python_interface/scripts/move_group_python_interface_tutorial.py>` 。

.. tutorial-formatter:: ./scripts/move_group_python_interface_tutorial.py

Launch 文件
---------------
整个 launch 文件在 GitHub 上的 :codedir:`这里<move_group_python_interface/launch/move_group_python_interface_tutorial.launch>` 可见。本教程中的所有代码都可从 **moveit_tutorials** 包中运行，这个包是 MoveIt 安装的一部分。