#pragma once

#include "lidar_detection/detector_base.hpp"

namespace lidar_detection
{

/**
 * 深度学习锥桶检测的占位接口。
 *
 * 未来可扩展的实现方案：
 *   - PointPillars: 基于 pillar 的 3D 目标检测
 *   - CenterPoint: 基于中心点热力图的检测
 *
 * 接入步骤：
 *   1. 实现模型加载逻辑（ONNX / Horizon BPU .bin）
 *   2. 在 lidar_detection.yaml 中将 detector_type 设为 "dl"
 *   3. LidarDetectionNode 会自动使用该后端
 */
class DLDetector : public IDetector
{
public:
  explicit DLDetector(const std::string & model_path);

  wuta_msgs::msg::ConeArray detect(const PointCloud::ConstPtr & cloud) override;

private:
  std::string model_path_;
  // TODO: 模型句柄（例如 Horizon BPU 上下文、TensorRT 引擎、ONNX 会话）
};

}  // namespace lidar_detection
