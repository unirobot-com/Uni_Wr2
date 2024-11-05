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

MAP_BUILDER.use_trajectory_builder_2d = true  -- 使用2D轨迹构建器
TRAJECTORY_BUILDER_2D.min_range = 0.01 --雷达的最大最小距离
TRAJECTORY_BUILDER_2D.max_range = 1.3

TRAJECTORY_BUILDER_2D.missing_data_ray_length = 2.0 -- 丢失数据时的射线长度.
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05  --体素滤波参数
TRAJECTORY_BUILDER_2D.use_imu_data = false -- 是否使用imu数据

-- TRAJECTORY_BUILDER_2D.submaps.resolution = 0.01
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.005 -- 栅格地图分辨率
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 10 -- 设置2D轨迹构建器子图的激光雷达数据数量
--在SLAM系统中，地图是由多个子图组成的。每个子图包含一定数量的激光雷达数据，这些数据用于构建局部地图。
--这个参数指定每个子图中包含的激光雷达数据帧数。即，当收集到35帧激光雷达数据后，会生成一个新的子图。
--子图生成频率：这个参数直接影响子图的生成频率。较小的值会更频繁地生成子图，从而可能提高地图的更新频率和精细度，但也会增加计算负担。
--子图质量：每个子图包含的激光雷达数据帧数越多，生成的子图可能会更稳定和准确，因为它包含了更多的观测数据。然而，过多的数据也会增加处理时间。
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.insert_free_space: true
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.hit_probability
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.miss_probability
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability: 0.9
-- TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability: 0.1

TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 1.0  --0.65
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49 --0.49

--ceres地图的扫描，平移，旋转的权重，影响建图效果，其他基本上是影响计算量等
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 50.
--扫描匹配点云和地图匹配程度，值越大，点云和地图匹配置信度越高
--这个参数用于调整在优化过程中占据空间（即由激光雷达测量到的障碍物）对总误差的贡献。较高的值表示占据空间对误差的影响更大，使得优化过程更关注与实际测量数据匹配的准确性。

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 50.
--残差平移、旋转分量，值越大，越不相信和地图匹配的效果，而是越相信先验位姿的结果
--这个参数用于调整在优化过程中平移变换误差对总误差的贡献。权重越大，系统对平移变换误差的敏感度越高，优化时将更加重视平移误差的最小化。

TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 50.
-- 如果imu不好，接入后地图旋转厉害，可以将这里的旋转权重减小
-- 这个参数用于调整在优化过程中旋转变换误差对总误差的贡献。权重越大，系统对旋转变换误差的敏感度越高，优化时将更加重视旋转误差的最小化。


TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 是否使用实时相关扫描匹配,建图时打开，定位时不要开，板子算不过来
-- 启用后，系统将使用实时的相关扫描匹配算法来进行扫描匹配，以提高位姿估计的准确性。


TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.01 --0.05
-- 实时相关扫描匹配的线性搜索窗口
-- 这个参数决定了在进行扫描匹配时，沿直线方向的搜索范围。较大的值可以增加搜索范围，但也会增加计算开销。


TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
-- 实时相关扫描匹配的平移变换代价权重
-- 这个参数用于调整扫描匹配过程中平移误差的代价。较高的值会增加平移误差的代价，从而影响匹配结果的优化方向。


TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-2
-- 实时相关扫描匹配的旋转变换代价权重
-- 这个参数用于调整扫描匹配过程中旋转误差的代价。较低的值会减少旋转误差的代价，使得系统在优化时更偏向于减少旋转误差。



-- 运动过滤器，这一个对于最后的地图的质量影响比较大，建议使用默认参数,这个可以实测，越小图越精细（在传感器牛逼时），计算量大。
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)
-- 建图时这个值越小子图生成越快，但是相应的质量降低，除非你的传感器非常牛逼

TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.01
-- 对于线性方向的过滤 (m)，建图时这个值越小子图生成越快，但是相应的质量降低，除非你的传感器非常牛逼
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.3)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --积累几帧激光数据作为一个标准单位scan


--POSE_GRAPH.optimization_problem.huber_scale = 1e2  
-- 优化问题的Huber缩放
--POSE_GRAPH.optimize_every_n_nodes = 35 
-- 每N个节点优化一次， 每35个有效帧组成一个子图，子图构建完成要闭环检测一次，这个数越小，闭环检测越频繁
--回环检测阈值，如果不稳定有错误匹配，可以提高这两个值，场景重复可以降低或者关闭回环
--POSE_GRAPH.constraint_builder.min_score = 0.48
--POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
POSE_GRAPH.optimize_every_n_nodes = 300
POSE_GRAPH.constraint_builder.min_score = 0.8
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.1
POSE_GRAPH.optimization_problem.huber_scale = 1e2  --1e3


return options
