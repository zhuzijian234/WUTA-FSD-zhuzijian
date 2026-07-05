#pragma once
#include <cmath>

namespace controller
{

struct VehicleParams
{
  // 轴距，影响 Pure Pursuit 的转向角换算。
  double wheel_base{1.53};
  // 重心到前轴距离，用于车辆动力学建模。
  double lf{0.8};
  // 最大允许转角，单位为度。
  double max_steer_angle{25.0};
};

struct VehicleState
{
  // 当前车辆的世界坐标 x。
  double x{0.0};
  // 当前车辆的世界坐标 y。
  double y{0.0};
  // 当前车辆航向角，单位为弧度。
  double yaw{0.0};
  // 当前车辆速度，单位为 m/s。
  double velocity{0.0};
};

}  // namespace controller
