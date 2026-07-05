#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "wuta_msgs/msg/cone_array.hpp"
#include "lidar_detection/detector_base.hpp"

namespace lidar_detection
{

/**
 * LiDAR 锥桶检测节点。
 *
 * 该节点是整个感知链路的入口之一：订阅点云数据，调用当前选定
 * 的检测器后端（传统 PCL 或深度学习）得到锥桶检测结果，并将结果
 * 发布给建图和融合模块。
 */
class LidarDetectionNode : public rclcpp::Node
{
public:
  explicit LidarDetectionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void publishVisualization(const wuta_msgs::msg::ConeArray & cones,
                            const std_msgs::msg::Header & header);

  std::unique_ptr<IDetector> detector_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<wuta_msgs::msg::ConeArray>::SharedPtr cone_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

}  // namespace lidar_detection
