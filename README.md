# npu_sim

机械臂小车仿真，使用环境ubuntu20+rosfoxy+gazebo11

其他前置

```
sudo apt install ros-foxy-xacro
sudo apt install ros-foxy-gazebo-ros2-control
```



待补充

## 文件系统

config：

存放rviz2配置，ros2 control的yaml配置



launch：

存放启动文件，遵循ros2约定的py形式，主要在使用的是carlaunch_gazebo.py。

carlaunch.py是一个rviz内的模型预览，

carlaunch_xacro.py是转化xacro为urdf并在rivz内预览。



urdf：

模型文件，现在主要使用的是car_gazebo.urdf，有时间再改成xacro，使得路径自动化find。

mobot.example是借鉴其他人的一个四轮小车模型，链接[古月居](https://www.guyuehome.com/6788)，是一个雷达+相机的四轮小车。

four_macnum也是借鉴他人的麦轮小车，[链接](https://github.com/MapleHan/four_macnum_car)

car.urdf和car.xacro只有一个方块，供carlaunch.py和carlaunch_xacro.py检验用。



world：

世界文件，现在主要使用的是default.world，其中包含了两根地图的柱子，以检验相机和雷达功能。

car.world是一个将模型和外界都写在world内的尝试，但发现雷达有问题，雷达信息所属的frame无法获得。暂时搁置了。

## 运行

包clone在工作空间src下，修改car_gazebo.urdf文件内这部分内容：

```xml
<parameters>/home/demons/dev_ws/install/npu_sim/share/npu_sim/config/joint_controller.yaml</parameters>
```

这里demons改为自己的用户名（我的电脑用户名是demons），dev_ws改成自己的工作空间名（或者你的工作空间在其他路径，总之install之前的路径在不同环境下可能不一样）。这是一个将修正的问题，等我用xacro替代了这个urdf，它应当会被解决。

然后编译，之后即可运行

```
ros2 launch npu_sim carlaunch_gazebo.py
```

同时打开gazebo和rviz2，能看到雷达线即表明成功。

还可使用rqt查看相机图像。

控制：

车底盘采用`libgazebo_ros_planar_move.so`，这是一个偷懒插件，以绝对性的移动替代了轮子，对应的控制话题是`/car/cmd_vel`，也可以通过rqt的topic publisher看到其中包含的数据类型等。以对应的数据类型发布速度即可使车运动或旋转。

相机还带有一个旋转关节，控制话题为`/joint_camera_controller/commands`可以直接控制相机角度，它的话题构造目前我只探索出了命令发布，给出一个例子。

```
ros2 topic pub /joint_camera_controller/commands std_msgs/msg/Float64MultiArray "data:
- 0.5"
```

注意这里的冒号后有换行，大概是直接发布的double数组，但还不知道在rqt中如何构造这样的数据类型。



## 拓展

gazebo11安装好后，`/opt/ros/foxy/share/gazebo_plugins/worlds`中有一些sdf案例，如各种车的仿真控制，传感器信息接收等，附带有可以尝试的指令，可以从这里了解到一些常用的组件的sdf代码。使用前最好预先[下载](https://github.com/osrf/gazebo_models)model到~/.gazebo/models

`ros-foxy-gazebo-ros2-control-demos`这个包包含了ros2 control在gazebo中的使用案例，是位置、速度、力控制的cartpole，可以试试向他们发送话题等，[仓库链接](https://github.com/ros-simulation/gazebo_ros2_control)。ros2的controller和ros1不太一样，还在摸索中。另一个教学型[案例](https://github.com/ros-controls/ros2_control_demos)以及官方[doc](https://ros-controls.github.io/control.ros.org/index.html)，内容还很少。
