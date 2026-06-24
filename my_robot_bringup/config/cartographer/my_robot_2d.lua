include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- Cartographer 建立的全局坐标系
  map_frame = "map",

  -- 用于跟踪机器人运动的坐标系
  tracking_frame = "base_link",

  -- Cartographer 对外发布的机器人坐标系
  published_frame = "base_footprint",

  -- 里程计坐标系
  odom_frame = "odom",

  -- 已有 Gazebo odom -> base_link，因此不要让 Cartographer
  -- 再发布一个 odom 坐标系，避免 TF 冲突
  provide_odom_frame = false,

  publish_frame_projected_to_2d = true,

  use_pose_extrapolator = true,

  -- 使用 /odom
  use_odometry = true,

  use_nav_sat = false,
  use_landmarks = false,

  -- 使用一个 LaserScan
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,

  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 使用 2D Cartographer
MAP_BUILDER.use_trajectory_builder_2d = true

-- 不使用 3D Cartographer
MAP_BUILDER.use_trajectory_builder_3d = false

-- 你的 2D 雷达最远 10m
TRAJECTORY_BUILDER_2D.min_range = 0.25
TRAJECTORY_BUILDER_2D.max_range = 10.0

-- 超过 max_range 的点作为自由空间射线
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 10.0

-- Gazebo 里通常没有 IMU，关闭 IMU
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- 使用在线相关扫描匹配，提高初始阶段鲁棒性
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 累积一帧雷达后插入
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- 体素滤波参数
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- 子图分辨率
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05

-- 每个子图积累的扫描数量
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- 扫描匹配权重
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.0

-- 每隔一定节点做一次全局优化
POSE_GRAPH.optimize_every_n_nodes = 35

-- 回环检测参数
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60

return options
