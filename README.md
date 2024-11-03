# Uni_Wr2
When the little potatoes arrive, all move away, Ha ha ha.
===============================================================
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


===============================================================
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



===============================================================
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
