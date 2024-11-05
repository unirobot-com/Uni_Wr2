# Uni_Wr2
- 时间：2024.11.5
- 作者：Unirobot
- 简介：哈哈哈，让世界都开源吧！
- 环境：
  - 硬件：树霉派，编码电机*2，PC机(虚拟机ubuntu20.04)，路由器(局域网)
  - 软件：【下位机：Ubuntu20.04，Ros-noetic】【上位机：虚拟机VMware，Ubuntu20.04,Ros-noetic】
- 故事背景：我是unirobot第一款小车，名为Wr2，我会Gmapping-Hector-Cartographer导航哦，信我，不挂科。

## 文件结构
- catkin_ws: 小车底层代码(双轮差速小车，使用了PID调速)
- uni_robot: 上位机虚拟机导航相关代码(Gmapping,Hector,Cartographer)
- cartographer_ws：cartographer导航包(涵盖了Cartographer导航的算法，依赖包)/；

## 下面就是我的学习指令了
===================================================================================
## 【Gmapping Slam】2D激光雷达导航. 60*60厘米小场地 ---局部路径规划器 dwa 2024年-09月-25日
  ### 地图构建测试 注意：gmapping建图需要小车速度慢一点
  1. 远程启动底层驱动 `roslaunch device uni_car.launch `
  2. 启动gmapping建图服务  `roslaunch slam uni_slam_gmapping.launch `
  3. 启动键盘  `rosrun teleop_twist_keyboard teleop_twist_keyboard.py `
  4. 遥控小车在环境中走一圈
  5. 生成2D地图 `roslaunch slam save_gmapping_map.launch `

 ### 导航测试
  1. 远程启动底层驱动 `roslaunch device uni_car.launch `
  2. 启动导航 `roslaunch navi uni_car_navigation_gmapping.launch `


=================================================================================
## 【Hector Slam】2D激光雷达导航. 60*60厘米小场地 ---局部路径规划器 dwa 2024年-09月-22日
  ### 地图构建测试 注意：hector建图需要小车速度慢一点
  1. 远程启动底层驱动 `roslaunch device device_actual.launch `
  2. 启动hector建图服务  `roslaunch slam uni_gmapping_hector.launch `
  3. 启动键盘  `rosrun teleop_twist_keyboard teleop_twist_keyboard.py `
  4. 遥控小车在环境中走一圈
  5. 生成2D地图 `roslaunch slam save_hector_map.launch `

 ### 导航测试
  1. 远程启动底层驱动 `roslaunch device device_actual.launch `
  2. 启动导航 `roslaunch navi uni_car_navigation_hector.launch `



=================================================================================
## 【Cartographer】2D激光雷达导航. 60*60厘米小场地 ---局部路径规划器 dwa 2024年-09月-22日
  ### 地图构建测试
  1. 远程启动底层驱动 `roslaunch device device.launch `
  2. 录制雷达包 `roslaunch slam uni_record_bag.launch `
  3. 启动键盘   `rosrun teleop_twist_keyboard teleop_twist_keyboard.py `
  4. 遥控小车在环境中走一圈
  5. 处理bao包数据 `roslaunch cartographer_ros uni_process_minimap.launch `
  6. 停止建图 `rosservice call /finish_trajectory 0 `
  7. 保存建图过程文件 
    ` rosservice call /write_state "{filename: '${HOME}/uni_robot/src/slam/bag/data.bag.pbstream', include_unfinished_submaps: "true"}" `
  8. 生成2D地图 `roslaunch cartographer_ros uni_create_minimap.launch `

 ### 导航测试
  1. 远程启动底层驱动 `roslaunch device device_actual.launch `
  2. 启动导航 `roslaunch navi uni_car_navigation_cartographer.launch `


=================================================================================
## 既然都看到这里了，再看看下面的内容又如何，哈哈哈
Uni小车介绍bilibili：https://www.bilibili.com/video/BV1NfDHY6ED8/?spm_id_from=333.337.search-card.all.click&vd_source=89624656ad0eb6477d8606070fc1e081
某宝：https://4vhhasmxqjt25cg7za43qs6podckjow.taobao.com/