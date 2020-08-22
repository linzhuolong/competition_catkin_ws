# competition_catkin_ws

*源码为UTF-8带BOM格式，建议用该编码格式打开*
*编译环境请参考.vscode*
*ubuntu version--16.04，ROS version--kinetic*

## 项目介绍
* 本项目为ROS移动机器人项目，采用软硬分离的设计方案。
* 项目主要分为三大部分:
* 1.底层硬件部分，主要控制驱动电机以及完成陀螺仪、里程计的数据采集;
* 2.视觉部分，通过摄像头对篮球、排球以及标定柱进行视觉识别，同时结合激光雷达数据，计算出目标物的种类、角度以及距离信息;
* 3.决策部分，决策部分主要接受底层和视觉的信息，控制机器人完成六个回合的比赛。

## 数据传输
* 数据传输主要以ROS的topic方式进行传输，以决策部分为例，在decisionmaking.cpp的9、11、13和15行分别定义了相应话题的订阅和发布对象:
```cpp
9|        detect_sub = nh.subscribe("/detect_result", 1, &DecisionMaking::detect_callback, this);//订阅感知部分传过来的数据
```
```cpp
11|       serial_sub = nh.subscribe("/dsp_to_pc", 1, &DecisionMaking::serial_callback, this);//订阅底层部分传过来的数据
```
```cpp
13|       detect_pub = nh.advertise<std_msgs::Int32>("/target_type", 1);//发布视觉感知命令
```
```cpp
15|       serial_pub = nh.advertise<const_msg::pc_to_dsp>("/pc_to_dsp", 1);//发布底层控制命令
```
## 相关依赖
* 激光依赖包－－hokoyo
* 串口依赖包－－serial
* usb摄像头包－－usb_cam
* 深度摄像头包－－realsense
* 视觉处理依赖－－sudo apt-get install ros-kinetic-perception

**参考本项目使用**

---

## Current Problem :
* 视觉处理包hsv运行结果不对，需要进一步调试，把逻辑理顺
* 决策处理包decisionmaking回合函数数据传输存在问题

## TODO :
* 当前的视觉处理采用的色块分割识别，光线变化时效果存疑，采用yolov3的深度学习视觉识别后续更新...


---
## src/demopkg:
* 该程序包主要包含了一些测试小demo，主要包含深度相机的启动测试:realsense2_demo.cpp，共享内存的测试:share_mem_demo.cpp，多线程自定义订阅的测试:thread_demo.cpp，以及为了测试主程序包的发布订阅测试节点
* boost_serial.cpp是用来测试boost::asio的串口库，但是read函数存在问题，所以运行报错:what():  read: End of file，等待后续更新...

