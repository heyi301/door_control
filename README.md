# 这是一个ROS2的服务，作为ROS2和硬件设备之间的桥梁，提供向货仓下达指令的功能。
# 运行在ubuntu22.04+ros2 humble中
# 准备工作：
## （可选1）将door_interface功能包加入你的工作空间，在CMakeLists中添加对应依赖，从而使用该接口文件
>find_package(door_interface REQUIRED)  
>ament_target_dependencies(your_project rclcpp std_msgs ... door_interface)
## （可选2）在你的工作空间中添加DoorControl.srv接口文件，从而使用该服务接口。
## 服务接口类型：
`int8 box_id`    #货箱id  
`int8 box_status`#货箱的状态  
`int8 door_status`#门的状态  
`---`  
`int8 cmd_status`#命令执行结果

## 安装CH340驱动，卸载brltty驱动，防止冲突：
### 卸载brltty
>sudo apt autoremove --purge brltty
### 安装驱动请参考：
>https://blog.csdn.net/m0_55260921/article/details/150052863  

# 使用方法：
## 1.运行脚本，编译功能包：

> cd your/ws  
> sudo chmod 777 ./scripts/build.sh  
> ./scripts/build.sh  

## 2.将货箱与电脑之间使用USB线连接好
## 3.运行脚本，绑定USB设备，启动服务节点。
### 查询USB设备串口号请参考该文章的第二种方法：https://blog.csdn.net/qq_35386301/article/details/84566214
### 查询到serial串口名后，在./scripts/door_usb.sh中修改对应的serial串口名，保存，运行该脚本，实现USB设备的绑定。
>sudo chmod 777 ./scripts/door_usb.sh  
>sudo ./scripts/door_usb.sh
### 运行节点，启动服务：
> ros2 run door_control door_control_node