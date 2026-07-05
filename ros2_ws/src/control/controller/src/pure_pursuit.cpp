#include "controller/pure_pursuit.hpp"
#include <algorithm>
#include <limits>

namespace controller
{

PurePursuit::PurePursuit(const VehicleParams & params, const Config & cfg)
: params_(params), cfg_(cfg) {}

// 这里的实现是一个较轻量的 Pure Pursuit 版本，适合在 ROS2 上作为底盘控制基线。
// 如果后续需要更高精度跟踪，可以在这里加入速度自适应、角度滤波或曲线拟合。

ControlCommand PurePursuit::compute(
  const VehicleState & state,
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints)
{
  ControlCommand cmd;
  if (waypoints.empty()) return cmd;

  // 1. 根据当前车速计算前视距离，速度越快，控制器看得越远。
  lookahead_dist_ = std::clamp(
    std::abs(state.velocity) * cfg_.ld_ratio,
    cfg_.min_lookahead,
    cfg_.max_lookahead);

  // 2. 在参考路径中找到离当前车位置最近且在前视距离内的目标点。
  //    这一步相当于“在路径中选一个当前最值得跟踪的参考点”。
  target_idx_ = findTargetIndex(state, waypoints, lookahead_dist_);
  if (target_idx_ < 0) {
    target_idx_ = static_cast<int>(waypoints.size()) - 1;
  }

  const auto & target = waypoints[target_idx_];
  const double tx = target.pose.pose.position.x;
  const double ty = target.pose.pose.position.y;

  // 3. 计算当前车辆到目标点的距离；距离太近时不再计算，避免除零。
  const double dist = planeDist(tx, ty, state.x, state.y);
  if (dist < 1e-6) return cmd;

  // 4. 把目标点转换到车体坐标系，求出横向偏移量 x_body。
  //    这里的 x_body 代表目标点在车辆前进方向左/右偏了多少。
  //    对于跟踪控制来说，x_body 越大，说明车辆应该转得越多。
  const double x_body = lateralOffset(tx, ty, state.x, state.y, state.yaw);

  // Numerical stabilization: amplify very small lateral offset (straight-ahead case)
  double numerator = 2.0 * x_body;
  if (std::abs(numerator) < 0.1) {
    numerator = 10.0 * std::copysign(1.0, numerator) * numerator;
  }

  // 5. 根据横向偏移和距离计算曲率 kappa，曲率越大表示转弯越急。
  const double kappa = numerator / (dist * dist);

  // 6. 由曲率换算成转向角，采用前轮转向自行车模型近似。
  cmd.steering_angle = std::atan(params_.wheel_base * kappa) * 180.0 / M_PI;

  // 7. 速度直接从路径点的速度属性取值，作为纵向控制参考。
  cmd.velocity = target.twist.twist.linear.x;

  cmd.valid = true;
  return cmd;
}

int PurePursuit::findTargetIndex(
  const VehicleState & state,
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
  double ld) const
{
  // 从路径尾部向前扫描，找到第一个处于当前前视距离内的路径点。
  // 这是一种简化版的目标点选择逻辑，能让控制器跟踪接近车辆的路径段。
  for (int i = static_cast<int>(waypoints.size()) - 1; i >= 1; --i) {
    const double d = planeDist(
      waypoints[i].pose.pose.position.x,
      waypoints[i].pose.pose.position.y,
      state.x, state.y);
    if (d < ld) return i;
  }
  return static_cast<int>(waypoints.size()) - 1;
}

double PurePursuit::lateralOffset(
  double target_x, double target_y,
  double car_x,    double car_y, double car_yaw)
{
  const double dx = target_x - car_x;
  const double dy = target_y - car_y;
  // Body frame x = lateral (left positive), y = longitudinal (forward positive)
  // x_body = -dx·sin(yaw) + dy·cos(yaw)
  return -dx * std::sin(car_yaw) + dy * std::cos(car_yaw);
}

double PurePursuit::planeDist(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace controller
