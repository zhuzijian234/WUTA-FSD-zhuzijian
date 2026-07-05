#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wuta_msgs/msg/mission_state.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

namespace ndt_localization
{

/**
 * NDT 定位节点。
 *
 * 该节点负责把预先保存好的点云地图与当前 LiDAR 扫描帧做配准，
 * 从而估计车辆在地图坐标系中的位姿。它是“竞速阶段高精度定位”的
 * 核心模块，输出结果会被 localization_manager 统一转发给规划与控制。
 *
 * 工作流程：
 * 1. 订阅 /system/mission_state，只有在 LOC_NDT 模式下才启用 NDT；
 * 2. 订阅 /initialpose，作为初始匹配位姿；
 * 3. 订阅 /hesai/pandar，使用 PCL/NDT 对每帧点云做配准；
 * 4. 发布 /ndt/pose、/ndt/path、/ndt/aligned_cloud 供后续模块使用。
 *
 * TODO（实际车上调试时需要继续完善）：
 * - 根据 Hesai 128 线雷达调优 ndt_resolution、step_size、max_iterations；
 * - 补齐 LiDAR 到 base_link 的外参；
 * - 验证 map → odom → base_link 的 TF 链是否完整。
 */
class NdtLocalization : public rclcpp::Node
{
public:
  explicit NdtLocalization(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void loadMap();
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);

  void runNDT(const sensor_msgs::msg::PointCloud2::SharedPtr scan);
  void publishPose(const Eigen::Matrix4f & transform, const rclcpp::Time & stamp);
  void publishPath(const geometry_msgs::msg::PoseStamped & pose);

  // NDT
  using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
  PointCloud::Ptr map_cloud_;

  // State
  bool map_loaded_{false};
  bool initial_pose_set_{false};
  bool active_{false};   // true only when LOC_NDT mode
  Eigen::Matrix4f current_transform_{Eigen::Matrix4f::Identity()};
  nav_msgs::msg::Path path_history_;

  // Parameters
  std::string map_path_{"/tmp/wuta_lidar_map.pcd"};

  // NDT 调参 —— TODO: 需要在真实车辆上进一步标定
  double ndt_resolution_{1.0};       // m —— NDT 栅格地图的体素尺寸
  double step_size_{0.1};            // m — Newton step size
  double transform_epsilon_{0.01};   // convergence criterion
  int    max_iterations_{30};

  // Downsampling input scan before NDT (speeds up matching)
  double scan_voxel_size_{0.5};      // m

  // Subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_cloud_pub_;  // debug
};

}  // namespace ndt_localization
