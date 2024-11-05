 
include "map_builder.lua"
include "trajectory_builder.lua"
 
options = {
    map_builder = MAP_BUILDER,
    trajectory_builder = TRAJECTORY_BUILDER,
    map_frame = "map",
    tracking_frame = "laser",
    published_frame = "base_link",
    odom_frame = "odom",
    provide_odom_frame = false, --true
    publish_frame_projected_to_2d = false, --true
    use_odometry = false,
    use_nav_sat = false,
    use_landmarks = false,
    num_laser_scans = 1,
    num_multi_echo_laser_scans = 0,
    num_subdivisions_per_laser_scan = 1,
    num_point_clouds = 0,
    lookup_transform_timeout_sec = 0.2,
    submap_publish_period_sec = 0.3,
    pose_publish_period_sec = 5e-3,
    trajectory_publish_period_sec = 30e-3,
    rangefinder_sampling_ratio = 1.,
    odometry_sampling_ratio = 1.,
    fixed_frame_pose_sampling_ratio = 1.,
    imu_sampling_ratio = 1.,
    landmarks_sampling_ratio = 1.,
}
 
MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads =2
 
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --积累几帧激光数据作为一个标准单位scan
TRAJECTORY_BUILDER_2D.min_range = 0.01  --激光的最近有效距离
TRAJECTORY_BUILDER_2D.max_range = 2.0   --激光最远的有效距离
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5. --无效激光数据设置距离为该数值
TRAJECTORY_BUILDER_2D.use_imu_data = false  --是否使用imu数据
 
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1 
--线距离搜索框，在这个框的大小内，搜索最佳scan匹配  减小该参数可以增强实时的建图效果，
--降低闭环优化的效果，形成闭环时，产生的重影较多
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher. angular_search_window = math.rad(10.) --角度搜索框的大小
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 20.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 2e-1
--影响的是过程中的效果，间接会影响最后的优化时间长
 
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 30.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 30.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 20
 
 
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 40
--num_range_data设置的值与CPU有这样一种关系，值小(10)，CPU使用率比较稳定，整体偏高，值大时，CPU短暂爆发使用(插入子图的时候)，平时使用率低，呈现极大的波动状态。
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49
 
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.05   --//尽量小点  // 如果移动距离过小, 或者时间过短, 不进行地图的更新
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.3)
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
 
POSE_GRAPH.optimization_problem.huber_scale = 1e2  --鲁棒核函数，去噪
 
POSE_GRAPH.optimize_every_n_nodes = 35   --后端优化节点
POSE_GRAPH.global_constraint_search_after_n_seconds = 10 
 
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 15  --优化迭代步数
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 1
 
POSE_GRAPH.constraint_builder.max_constraint_distance = 15.
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.min_score = 0.50
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth = 5. --搜索方法，界定分支法，求解问题构成一个搜索树，depth是构造树的深度
POSE_GRAPH.global_sampling_ratio = 0.001
 
return options