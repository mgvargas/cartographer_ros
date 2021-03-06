-- Configuration file for boxy, currently a copy of PR2.lua

include "map_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  sensor_bridge = {
    horizontal_laser_min_range = 0.06,
    horizontal_laser_max_range = 30.,
    horizontal_laser_missing_echo_ray_length = 5.,
    constant_odometry_translational_variance = 0.1,
    constant_odometry_rotational_variance = 0.1,
  },
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  use_odometry = true,
  use_laser_scan = true,
  use_multi_echo_laser_scan = false,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
SPARSE_POSE_GRAPH.optimization_problem.huber_scale = 1e2

return options
