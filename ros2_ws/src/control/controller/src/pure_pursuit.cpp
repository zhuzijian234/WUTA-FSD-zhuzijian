#include "controller/pure_pursuit.hpp"
#include <algorithm>
#include <limits>

namespace controller
{

PurePursuit::PurePursuit(const VehicleParams & params, const Config & cfg)
: params_(params), cfg_(cfg) {}

ControlCommand PurePursuit::compute(
  const VehicleState & state,
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints)
{
  ControlCommand cmd;
  if (waypoints.empty()) return cmd;

  // 1. Compute lookahead distance — velocity-proportional, clamped
  lookahead_dist_ = std::clamp(
    std::abs(state.velocity) * cfg_.ld_ratio,
    cfg_.min_lookahead,
    cfg_.max_lookahead);

  // 2. Find target waypoint
  target_idx_ = findTargetIndex(state, waypoints, lookahead_dist_);
  if (target_idx_ < 0) {
    target_idx_ = static_cast<int>(waypoints.size()) - 1;
  }

  const auto & target = waypoints[target_idx_];
  const double tx = target.pose.pose.position.x;
  const double ty = target.pose.pose.position.y;

  // 3. Distance to target
  const double dist = planeDist(tx, ty, state.x, state.y);
  if (dist < 1e-6) return cmd;

  // 4. Lateral offset in vehicle body frame (x_body = how far left/right target is)
  const double x_body = lateralOffset(tx, ty, state.x, state.y, state.yaw);

  const double numerator = 2.0 * x_body;

  // 5. Curvature: kappa = 2·x_body / dist²
  const double kappa = numerator / (dist * dist);

  // 6. Steering angle (Ackermann bicycle model): δ = atan(L × kappa)
  cmd.steering_angle = std::atan(params_.wheel_base * kappa) * 180.0 / M_PI;

  // 7. Velocity from target waypoint
  cmd.velocity = target.twist.twist.linear.x;

  cmd.valid = true;
  return cmd;
}

int PurePursuit::findTargetIndex(
  const VehicleState & state,
  const std::vector<autoware_msgs::msg::Waypoint> & waypoints,
  double ld)
{
  const int N = static_cast<int>(waypoints.size());
  if (N == 0) return -1;

  // Guard: if waypoints were replaced with a smaller set (e.g. mode switch
  // or data race on the subscription), clamp target_idx_ back into range.
  // Without this, search_start >= N and the loop never executes, returning
  // a bogus target deep in the new path.
  if (target_idx_ >= N) {
    target_idx_ = N - 1;
  }

  // Pure forward search — NO closest-waypoint lookup.
  // Closest-waypoint searches break on self-intersecting paths (e.g. the
  // skidpad figure-8) because multiple waypoints share the same coordinates
  // at different path indices, and floating-point precision picks the wrong
  // lap.  Instead, we simply scan forward from a few waypoints behind the
  // current target for the first point at distance >= lookahead.
  // This enforces strict monotonic progress.

  const int search_start = std::max(0, target_idx_ - 5);

  for (int i = search_start; i < N; ++i) {
    const double d = planeDist(
      waypoints[i].pose.pose.position.x,
      waypoints[i].pose.pose.position.y,
      state.x, state.y);
    if (d >= ld) return i;
  }

  // All remaining waypoints are within LD — aim for the finish
  return N - 1;
}

double PurePursuit::lateralOffset(
  double target_x, double target_y,
  double car_x,    double car_y, double car_yaw)
{
  const double dx = target_x - car_x;
  const double dy = target_y - car_y;
  return -dx * std::sin(car_yaw) + dy * std::cos(car_yaw);
}

double PurePursuit::planeDist(double ax, double ay, double bx, double by)
{
  const double dx = ax - bx;
  const double dy = ay - by;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace controller
