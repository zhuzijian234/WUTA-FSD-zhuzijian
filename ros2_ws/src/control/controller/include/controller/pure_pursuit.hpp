#pragma once

#include <autoware_msgs/msg/lane.hpp>
#include <autoware_msgs/msg/waypoint.hpp>
#include "controller/vehicle_state.hpp"
#include <cmath>
#include <vector>

namespace controller
{

struct ControlCommand
{
  // 输出的转向角，单位为度，正值表示左转。
  double steering_angle{0.0};
  // 输出的目标速度，单位为 m/s。
  double velocity{0.0};
  // 表示当前控制命令是否有效，防止空路径或无效状态下输出错误命令。
  bool   valid{false};
};

/**
 * Pure Pursuit 跟踪控制器。
 *
 * 该算法通过“选取当前前视距离内的目标路径点，再求该点相对车辆
 * 的横向偏移”来计算转向角。它是经典的路径跟踪算法之一，适合在
 * 赛道跟踪场景下使用。
 *
 * 核心步骤：
 * 1. 根据当前车速计算 lookahead 距离；
 * 2. 从参考路径中找出当前目标点；
 * 3. 把目标点变换到车体坐标系，得到横向偏移；
 * 4. 利用几何关系计算曲率，再换算为转向角；
 * 5. 纵向速度直接使用路径点上的速度参考。
 */
class PurePursuit
{
public:
  struct Config
  {
    // 前视距离 = 当前速度 × ld_ratio。
    double ld_ratio{2.0};
    // 低速时的下限，避免前视距离过小。
    double min_lookahead{2.0};
    // 高速时的上限，避免前视距离过大。
    double max_lookahead{20.0};
  };

  explicit PurePursuit(const VehicleParams & params, const Config & cfg);

  /**
   * Compute control command for one cycle.
   * @param state    Current vehicle state (pose + velocity)
   * @param waypoints  Reference path (autoware_msgs Lane waypoints)
   */
  ControlCommand compute(const VehicleState & state,
                         const std::vector<autoware_msgs::msg::Waypoint> & waypoints);

  // Accessors for diagnostics
  double lookaheadDistance() const { return lookahead_dist_; }
  int    targetIndex()       const { return target_idx_; }

private:
  int findTargetIndex(const VehicleState & state,
                      const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
                      double ld) const;

  // Transform global point to vehicle body frame, return lateral offset x
  static double lateralOffset(double target_x, double target_y,
                               double car_x, double car_y, double car_yaw);

  static double planeDist(double ax, double ay, double bx, double by);

  VehicleParams params_;
  Config cfg_;

  double lookahead_dist_{0.0};
  int    target_idx_{0};
};

}  // namespace controller
