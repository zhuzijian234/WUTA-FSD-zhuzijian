#include "lidar_detection/dl_detector.hpp"
#include <stdexcept>

namespace lidar_detection
{

DLDetector::DLDetector(const std::string & model_path)
: model_path_(model_path)
{
  // TODO: 加载深度学习模型
  // 可选实现方案：
  //   - Horizon BPU: 使用 bpu_predict 或 libdnn API 加载 .bin 模型
  //   - TensorRT: 使用 nvinfer1::IRuntime 加载 .engine
  //   - ONNX Runtime: 使用 Ort::Session 加载 .onnx
  throw std::runtime_error("DLDetector not yet implemented. model_path: " + model_path_);
}

wuta_msgs::msg::ConeArray DLDetector::detect(const PointCloud::ConstPtr & /*cloud*/)
{
  // TODO: 实现完整的推理流程
  // 1. 预处理：把点云体素化为 pillar 或生成 BEV 特征图
  // 2. 在 BPU/GPU 上执行模型推理
  // 3. 后处理：将热力图或锚框解码为目标框
  // 4. 执行 NMS 去重
  // 5. 将检测结果转换为 ConeArray

  return wuta_msgs::msg::ConeArray{};
}

}  // namespace lidar_detection
