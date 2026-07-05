#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "wuta_msgs/msg/cone_array.hpp"

namespace lidar_detection
{

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

/**
 * 检测器抽象接口。
 * 通过实现该接口可以切换不同的检测后端（传统 PCL 或深度学习）。
 */
class IDetector
{
public:
  virtual ~IDetector() = default;

  /**
   * Detect cones from a raw point cloud.
   * @param cloud  Input point cloud in sensor frame
   * @return       ConeArray (positions in sensor frame, color=UNKNOWN)
   */
  virtual wuta_msgs::msg::ConeArray detect(const PointCloud::ConstPtr & cloud) = 0;
};

}  // namespace lidar_detection
