入门
===============

本教程将安装 MoveIt，并创建一个工作区来运行本教程和示例机器人。

安装 ROS 和 Catkin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`安装 ROS Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu>`_ 。

在学习 ROS 安装指南时，很容易漏掉一些步骤。如果你在后面的几个步骤中遇到了报错，那么最好从这里重新开始，以确保你正确地安装了 ROS 。

一旦你安装好了 ROS ，确保有最新的软件包： ::

  rosdep update
  sudo apt update
  sudo apt dist-upgrade

安装 `catkin <http://wiki.ros.org/catkin>`_  ，即 ROS 构建系统（ROS build system）： ::

  sudo apt install ros-noetic-catkin python3-catkin-tools

Install `wstool <http://wiki.ros.org/wstool>`_ : ::

  sudo apt install python3-wstool

创建一个 Catkin 工作空间以及下载 MoveIt 源码
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
本教程依赖于 MoveIt 的 master 分支。我们需要从源代码构建它。你将需要设置一个 `catkin <http://wiki.ros.org/catkin>`_ 工作空间: ::

  mkdir -p ~/ws_moveit/src
  cd ~/ws_moveit/src

  wstool init .
  wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
  wstool update -t .

下载示例代码
^^^^^^^^^^^^^^^^^^^^^
为了方便学习，你需要一个类似于叫做 **ROBOT_moveit_config** 的包。我们默认使用的演示机器人是 Panda 机械臂（来自 Franka Emika ）。我们建议你从源代码安装 **panda_moveit_config** 包以获得一个 working package （译者：活动包？）。

在你的 `catkin <http://wiki.ros.org/catkin>`_ 工作空间里, 下载本教程以及 ``panda_moveit_config`` 包: ::

  cd ~/ws_moveit/src
  git clone https://github.com/ros-planning/moveit_tutorials.git -b master
  git clone https://github.com/ros-planning/panda_moveit_config.git -b melodic-devel

.. Note:: 现在我们将使用预先生成的 ``panda_moveit_config`` 包，但是之后在 `MoveIt 设置助手教程 <../setup_assistant/setup_assistant_tutorial.html>`_ 中，将介绍如何制作我们自己的包。

构建和编译 Catkin 工作空间
^^^^^^^^^^^^^^^^^^^^^^^^^^^
以下代码将从 Debian 中安装你工作空间要求但尚未安装的依赖： ::

  cd ~/ws_moveit/src
  rosdep install -y --from-paths . --ignore-src --rosdistro noetic
  
**注意** 如果标准 ROS 软件源中还没有上游包，或者在这些包的安装中遇到任何构建错误，请尝试使用 `ROS 测试软件源 <http://wiki.ros.org/TestingRepository>`_ ： ::

        sudo sh -c 'echo "deb http://packages.ros.org/ros-testing/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
        sudo apt update

下一个命令将配置你的 catkin 工作空间： ::

  cd ~/ws_moveit
  catkin config --extend /opt/ros/${ROS_DISTRO} --cmake-args -DCMAKE_BUILD_TYPE=Release
  catkin build

使能（ source ） catkin 工作空间配置: ::

  source ~/ws_moveit/devel/setup.bash

可选: 将上述命令添加到您的 ``.bashrc``: ::

   echo 'source ~/ws_moveit/devel/setup.bash' >> ~/.bashrc

.. note:: 在 ``~/.bashrc`` 中自动使能（ source ） ``setup.bash`` 不是必须的，且很多老手会同时使用不止一个 catkin 工作空间。但这样做会简单许多，因此我们仍然这样建议。

下一步
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
`在 RViz 中可视化一个机器人和运动规划的交互插件 <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_
