include "map_builder.lua"  
include "trajectory_builder.lua"  

options = {
  map_builder = MAP_BUILDER,  
  trajectory_builder = TRAJECTORY_BUILDER,  
  map_frame = "map",  -- 地图帧的名称
  tracking_frame = "base_footprint",  -- 跟踪帧的名称
  published_frame = "odom",  -- 发布帧的名称
  odom_frame = "odom",  -- 里程计帧的名称
  provide_odom_frame = false,  -- 是否提供里程计帧
  publish_frame_projected_to_2d = false,  -- 是否发布2d姿态
  use_pose_extrapolator = false,
  use_odometry = true,  -- 是否使用里程计
  use_nav_sat = false,  -- 是否使用导航卫星
  use_landmarks = false,  -- 是否使用地标
  num_laser_scans = 1,  -- 激光雷达的数量
  num_multi_echo_laser_scans = 0,  -- 多回波激光雷达的数量
  num_subdivisions_per_laser_scan = 1,  -- 每个激光扫描的细分数量
  num_point_clouds = 0,  -- 点云的数量
  lookup_transform_timeout_sec = 0.2,  -- 查找变换的超时时间（秒）
  submap_publish_period_sec = 0.3,  -- 子地图发布周期（秒）
  pose_publish_period_sec = 5e-3,  -- 姿态发布周期（秒）
  trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期（秒）
  rangefinder_sampling_ratio = 1.,  -- 测距仪采样比率
  odometry_sampling_ratio = 1.,  -- 里程计采样比率
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定帧姿态采样比率
  imu_sampling_ratio = 1.,  -- IMU采样比率
  landmarks_sampling_ratio = 1.,  -- 地标采样比率
}
 
MAP_BUILDER.use_trajectory_builder_2d = true  -- 是否启动2D SLAM
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- 2D轨迹构建器中子地图的范围数据数量
TRAJECTORY_BUILDER_2D.min_range = 0.1  -- 限制在雷达最小扫描范围，比机器人半径小的都忽略
TRAJECTORY_BUILDER_2D.max_range = 3.5  -- 限制在雷达最大扫描范围
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.  -- 限制在雷达最大扫描范围
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- 是否使用IMU数据
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- 是否使用实时回环检测扫描匹配

TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- 1.0改成0.1,提高对运动的敏感度
POSE_GRAPH.constraint_builder.min_score = 0.65  -- 0.55改成0.65,Fast csm的最低分数，高于此分数才进行优化。
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  --0.6改成0.7,全局定位最小分数，低于此分数则认为目前全局定位不准确

return options
