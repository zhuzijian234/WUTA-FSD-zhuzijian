#include "controller/twist_filter.hpp"
#include <algorithm>
#include <cmath>

namespace controller
{

TwistFilter::TwistFilter(const VehicleParams & params)
: params_(params) {}

// 此滤波器不改变控制逻辑的主体，只为最终输出增加安全保护。
// 对于自动驾驶车辆来说，这一层非常重要，因为它能避免控制器输出太激进。

TwistFilter::FilteredCommand TwistFilter::filter(
  double raw_angle, double raw_velocity, bool emergency)
{
  FilteredCommand out;
  out.emergency = emergency;

  // 紧急情况下直接停车，避免继续执行可能危险的控制命令。
  if (emergency) {
    out.velocity = 0.0;
    out.steering_angle = 0.0;
    last_velocity_ = 0.0;
    return out;
  }

  // 速度平滑：加速时采用较慢的增速，减速时响应更快，增加安全性。
  if (raw_velocity >= last_velocity_) {
    // 加速时使用较小的增益，避免轮胎打滑或控制抖动。
    out.velocity = 0.9 * last_velocity_ + 0.1 * raw_velocity;
  } else {
    // 减速时使用更大的增益，快速降低车速以提高安全性。
    out.velocity = 0.3 * last_velocity_ + 0.7 * raw_velocity;
  }
  last_velocity_ = out.velocity;

  // 转向角限幅：防止控制器输出超出车辆物理允许范围的角度。
  out.steering_angle = std::clamp(
    raw_angle,
    -params_.max_steer_angle,
     params_.max_steer_angle);

  return out;
}

}  // namespace controller
