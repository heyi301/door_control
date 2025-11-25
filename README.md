# 这是一个ROS2的服务，作为ROS2和硬件设备之间的桥梁，提供向货仓下达指令的功能。
# 运行在ubuntu22.04+ros2 humble/foxy中


## 服务接口类型：
`int8 box_id`    #货箱id  
`int8 box_status`#货箱的状态  
`int8 door_status`#门的状态  
`---`  
`int8 cmd_status`#命令执行结果
## Action接口：
`````bash
# 1.目标数据
int8 box_id #货箱ID（0/1）
int8 door_cmd #货箱门开启/关闭（1/0）
---
# 2.结果
bool result #成功/失败 （1/0）
---
# 3.连续反馈
int8 current_action
`````
## 安装CH340驱动，卸载brltty驱动，防止冲突：
### 卸载brltty
>sudo apt autoremove --purge brltty
### 安装驱动：
进入download文件夹：
> cd ~/Downloads  




克隆仓库：  
> git clone https://github.com/heyi301/ch341_driver.git  


编译安装驱动：  
> cd ch341_driver/driver/  

> make

> sudo insmod ch341.ko  && sudo make install

如果没有报错，说明成功安装。

# 使用方法：

## 1.安装依赖：

````bash
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
sudo apt install ros-$ROS_DISTRO-rosidl-generator-dds-idl
`````

## 2.注释ros2环境变量，在.bashrc中。具体参考宇树ROS2服务接口：
>https://support.unitree.com/home/zh/developer/ROS2_service

## 3.编译cyclonedds
````bash
cd cyclonedds_ws/src
#克隆cyclonedds仓库
git clone https://github.com/ros2/rmw_cyclonedds -b $ROS_DISTRO
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x 
cd ..
colcon build --packages-select cyclonedds #编译cyclonedds
`````
## 4.运行脚本，编译功能包：  
> cd your/ws  
> source setup.sh  
> sudo chmod 777 ./scripts/build.sh  
> ./scripts/build.sh  


## 5.运行脚本，绑定USB设备。
>sudo chmod 777 ./scripts/door_usb.sh  
>sudo ./scripts/door_usb.sh

## 6.将货箱与电脑之间使用USB线连接好
<!-- ### 查询USB设备串口号请参考该文章的第二种方法：https://blog.csdn.net/qq_35386301/article/details/84566214
### 查询到serial串口名后，在./scripts/door_usb.sh中修改对应的serial串口名，保存，运行该脚本，实现USB设备的绑定。
>sudo chmod 777 ./scripts/door_usb.sh  
>sudo ./scripts/door_usb.sh -->
### 运行节点，启动节点：
> ros2 launch door_control door_control_launch.py

### 测试服务：
> ros2 service call /door_control door_interface/srv/DoorControl "{box_id: 1, box_status: 1, door_cmd: 0}"  
> ros2 action send_goal /door_control_action door_interface/action/DoorControl "{box_id: 1,door_cmd: 1}"