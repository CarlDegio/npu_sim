# npu_sim

机械臂小车仿真，使用环境ubuntu20+rosfoxy+gazebo11

其他前置

```
sudo apt install ros-foxy-xacro
sudo apt install ros-foxy-gazebo-ros2-control
```

## 文件系统

config：

存放rviz2配置，ros2 control的yaml配置



launch：

存放启动文件，遵循ros2约定的py形式，主要在使用的是carlaunch_gazebo.py。



urdf：

car.xacro是车外形的描述文件，在launch中再被转化为urdf。

car.gazebo.xacro为车增添了gazebo中的插件与颜色配置，后缀是.xacro只是为了能够在pycharm下语法标记。



world：

世界文件，使用的是default.world，其中包含了两根地图的柱子，以检验相机和雷达功能。

## 运行

在工作空间下编译，之后即可运行

```
ros2 launch npu_sim carlaunch_gazebo.py
```

同时打开gazebo和rviz2，能看到雷达线即表明成功。



传感：

相机的传感主要有`/car/camera_high`和`/car/camera_low`两个空间之下的，前者是在机械臂上固定的深度相机，后者是车载的可以被驱动绕y旋转的RGB相机。可以通过rqt中的image看到不同的图片。

雷达的传感是`/car/scan`，扫描角度为[-1.5,1.5]弧度，一循环采样500个样点，一秒采集10次，最大距离10m最小距离0.1m。在rviz中即可看到。

imu的传感是`/car/imu`，给出各向线加速度，角速度，角，包含重力和噪声。



控制：

车底盘采用`libgazebo_ros_planar_move.so`，这是一个偷懒插件，以绝对性的移动替代了轮子，对应的控制话题是`/car/cmd_vel`，也可以通过rqt的topic publisher看到其中包含的数据类型等。以对应的数据类型发布速度即可使车运动或旋转。**存在的bug：planar和ros2_control貌似都跳过了动力学，直接设置速度而加速度置为0，从而复杂模型之间的交互力会错误。导致存在其中一个插件时，仿真的运动会变得错误（如期望自转速度1rad/s，实际无法到达），表现为整体仿真运动的更慢，需要更大的力才能正常运动**

机械臂带有三个旋转关节，车载相机带有一个旋转关节（代表现实的舵机）。机械臂的控制话题为`/arm_joint_controller/commands`可以控制机械臂每个关节的速度，ABB构型，底座是绕z旋转的，其余两个绕y旋转，x向前，旋转的正方向符合右手定则；车载相机控制话题为`/camera_joint_controller/commands`。这种格式的话题构造目前我只探索出了命令发布（代码也可以），给出一个例子。

```
ros2 topic pub /joint_camera_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5
- 0.1
- -0.1"
```

注意这里的冒号后有换行，大概是直接发布的double数组，但还不知道在rqt中如何构造这样的数据类型。

可以通过`/dynamic_joint_states`和`/joint_states`两个话题接收四个关节数据，两个应该没有本质差别。

## 拓展

gazebo11安装好后，`/opt/ros/foxy/share/gazebo_plugins/worlds`中有一些sdf案例，如各种车的仿真控制，传感器信息接收等，附带有可以尝试的指令，可以从这里了解到一些常用的组件的sdf代码。使用前最好预先[下载](https://github.com/osrf/gazebo_models)model到~/.gazebo/models

`ros-foxy-gazebo-ros2-control-demos`这个包包含了ros2 control在gazebo中的使用案例，是位置、速度、力控制的cartpole，可以试试向他们发送话题等，[仓库链接](https://github.com/ros-simulation/gazebo_ros2_control)。ros2的controller和ros1不太一样，还在摸索中。另一个教学型[案例](https://github.com/ros-controls/ros2_control_demos)以及官方[doc](https://ros-controls.github.io/control.ros.org/index.html)，内容还很少。



ros2 仿真建模实例：https://www.guyuehome.com/6788，mobot系列。

four_macnum是他人的麦轮小车，[链接](https://github.com/MapleHan/four_macnum_car)
