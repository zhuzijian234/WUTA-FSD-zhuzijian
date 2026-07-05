#pragma once

#include "lidar_detection/detector_base.hpp"

namespace lidar_detection
{

struct TraditionalDetectorConfig
{
  // 地面去除
  double ground_z_threshold{-0.8};     // 相对传感器高度低于该值的点被视为地面
  double ransac_distance_threshold{0.2}; // RANSAC 采样内点距离阈值（m）
  bool use_ransac{true};               // true=使用 RANSAC，false=使用简单高度阈值

  // 聚类前体素下采样
  double voxel_leaf_size{0.1};         // m

  // 欧式聚类
  double cluster_tolerance{0.4};       // m —— 同一簇内点的最大距离
  int min_cluster_size{3};
  int max_cluster_size{200};

  // 锥桶形状筛选（基于聚类包围盒）
  double max_cone_width{0.5};          // m
  double max_cone_height{0.6};         // m
  double min_cone_height{0.1};         // m

  // 检测范围
  double max_detection_range{20.0};    // m —— 相对传感器原点的最大检测距离
};

/**
 * 传统 PCL 版本的锥桶检测器。
 *
 * 通过范围滤波、地面去除、体素下采样、欧氏聚类和尺寸筛选
 * 的流程，从原始点云中提取候选锥桶。它不依赖深度学习模型，
 * 适合做基线检测与调试。
 */
class TraditionalDetector : public IDetector
{
public:
  explicit TraditionalDetector(const TraditionalDetectorConfig & config);

  wuta_msgs::msg::ConeArray detect(const PointCloud::ConstPtr & cloud) override;

private:
  TraditionalDetectorConfig cfg_;

  PointCloud::Ptr removeGround(const PointCloud::ConstPtr & cloud) const;
  PointCloud::Ptr voxelDownsample(const PointCloud::ConstPtr & cloud) const;
  std::vector<PointCloud::Ptr> euclideanCluster(const PointCloud::ConstPtr & cloud) const;
  bool isConeShape(const PointCloud::Ptr & cluster) const;
};

}  // namespace lidar_detection
