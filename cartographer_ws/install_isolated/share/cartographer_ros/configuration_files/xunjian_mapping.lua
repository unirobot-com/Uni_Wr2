include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,  --
  map_frame = "map",
  tracking_frame = "gyro_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,  --这个布尔值决定是否自动发布里程计帧
  publish_frame_projected_to_2d = false, --是否应把发布的帧投影到地面（仅在2D模式下适用）。
  use_pose_extrapolator = true,
  use_odometry = false, --是否使用里程计
  use_nav_sat = false,  --是否使用卫星
  use_landmarks = false, --是否使用地标
  num_laser_scans = 0,   --单线雷达
  num_multi_echo_laser_scans = 0, --多雷达数量
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.8,  --在查找传感器数据与tracking_frame之间的转换时的超时时间。
  submap_publish_period_sec = 0.3,     --子图发布的时间间隔。
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- MAP_BUILDER.use_trajectory_builder_3d = true  --列的参数负责调整激光雷达数据的预处理和扫描匹配。
--TRAJECTORY_BUILDER_3D:系列的参数负责调整激光雷达数据的预处理和扫描匹配。

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1 --这个参数决定了将多少个激光帧累积在一起形成
            --一个点云，然后进行处理。如果你的环境比较复杂，或者你的移动速度比较快，你可能需要将多个激光帧
            --叠加在一起，也就是将此参数设为一个大于1的数字。

TRAJECTORY_BUILDER_3D.submaps.num_range_data = 400 -- 决定生成新子图需要的激光帧数。如果你想要与
            --环境更密切地交互，可以减少此参数。然而，如果设置得过小，可能导致地图过于分散，难以捕捉到环境的整
            --体特征。 400
-- TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.25      --对数据进行体素滤波以降低数据量。体素滤波的大小对应
            --于各点云的粒度，设置得过大会导致丢失小尺度的地图特征，设置得过小则会使处理速度降低。
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 2
-- TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 100
-- TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 150
-- TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.15

TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.25 --设置生成子图分辨率,子图用于的全局优化（回环检测）
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.1 --设置生成子图分辨率,子图用于局部优化和数据插入
-- TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.06

MAP_BUILDER.use_trajectory_builder_3d = true  --列的参数负责调整激光雷达数据的预处理和扫描匹配。
MAP_BUILDER.num_background_threads = 4


--POSE_GRAPH系列的参数主要涉及到回环检测和全局优化。
POSE_GRAPH.optimize_every_n_nodes = 320
POSE_GRAPH.global_sampling_ratio = 0.003
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5  --决定了用于回环检测的子图样本的比例，设置得过小
            --可能导致回环检测效果变差，过大则增大了计算量。
-- POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 50
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 5
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(20.)
POSE_GRAPH.global_constraint_search_after_n_seconds = 10
POSE_GRAPH.max_num_final_iterations = 100

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 10
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 10

return options