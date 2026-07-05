#pragma once
#include "controller/vehicle_state.hpp"

namespace controller
{

/**
 * 控制输出的安全滤波器。
 *
 * Pure Pursuit 计算出的原始命令可能会过于激进，因此这里增加了三层保护：
 * - 加速时采用缓慢上升，降低打滑风险；
 * - 减速时响应更快，提升安全性；
 * - 转向角严格限幅，避免超过车辆物理允许范围；
 * - 紧急状态下直接置零，确保停车安全。
 */
class TwistFilter
{
public:
  explicit TwistFilter(const VehicleParams & params);

  struct FilteredCommand
  {
    double steering_angle{0.0};  // degrees, clamped
    double velocity{0.0};        // m/s, smoothed
    bool   emergency{false};
  };

  FilteredCommand filter(double raw_angle, double raw_velocity,
                         bool emergency = false);

  void reset() { last_velocity_ = 0.0; }

private:
  VehicleParams params_;
  double last_velocity_{0.0};
};

}  // namespace controller
