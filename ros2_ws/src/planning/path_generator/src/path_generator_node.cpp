#include "path_generator/path_generator_node.hpp"
#include <cmath>

namespace path_generator
{

using State = wuta_msgs::msg::MissionState;

PathGeneratorNode::PathGeneratorNode(const rclcpp::NodeOptions & options)
: Node("path_generator_node", options)
{
  trackdrive_velocity_    = declare_parameter("trackdrive_velocity",    trackdrive_velocity_);
  skidpad_radius_         = declare_parameter("skidpad_radius",         skidpad_radius_);
  skidpad_velocity_       = declare_parameter("skidpad_velocity",       skidpad_velocity_);
  skidpad_points_         = declare_parameter("skidpad_points",         skidpad_points_);
  acceleration_length_    = declare_parameter("acceleration_length",    acceleration_length_);
  acceleration_velocity_  = declare_parameter("acceleration_velocity",  acceleration_velocity_);

  // Subscribers
  mission_sub_ = create_subscription<State>(
    "/system/mission_state", 10,
    std::bind(&PathGeneratorNode::onMissionState, this, std::placeholders::_1));

  centerline_sub_ = create_subscription<autoware_msgs::msg::Lane>(
    "/planning/centerline", 10,
    std::bind(&PathGeneratorNode::onCenterline, this, std::placeholders::_1));

  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&PathGeneratorNode::onPose, this, std::placeholders::_1));

  // Publisher — final_waypoints consumed by controller
  waypoints_pub_ = create_publisher<autoware_msgs::msg::Lane>("/planning/final_waypoints", 10);

  // Visualization — LINE_STRIP through planned waypoints
  viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/final_waypoints_viz", 10);

  // Visualization — driven trajectory growing behind the vehicle
  trajectory_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planning/driven_trajectory_viz", 10);

  RCLCPP_INFO(get_logger(), "PathGeneratorNode ready.");
}

void PathGeneratorNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  current_pose_ = *msg;
  pose_ready_ = true;

  // Accumulate driven trajectory (skip if position unchanged to avoid duplicates)
  geometry_msgs::msg::Point pt;
  pt.x = msg->pose.position.x;
  pt.y = msg->pose.position.y;
  pt.z = msg->pose.position.z;

  if (trajectory_.empty() ||
      std::abs(pt.x - last_trajectory_point_.x) > 0.01 ||
      std::abs(pt.y - last_trajectory_point_.y) > 0.01)
  {
    trajectory_.push_back(pt);
    last_trajectory_point_ = pt;

    // Publish trajectory every few points — always publish so RViz can discover
    // the topic even before subscribing (ADD with same ns/id replaces in place)
    if (trajectory_.size() % 3 == 0)
    {
      publishTrajectory();
    }
  }
}

void PathGeneratorNode::onMissionState(const State::SharedPtr msg)
{
  // Detect mode change → invalidate cached path so the new mode regenerates
  const bool mode_changed = (msg->mission_mode != mission_mode_);
  if (mode_changed) {
    path_generated_ = false;
    RCLCPP_INFO(get_logger(), "Mode changed -> invalidated cached path");
  }

  mission_mode_  = msg->mission_mode;
  system_state_  = msg->state;

  // Trigger non-trackdrive paths when system is active
  if (system_state_ != State::EXPLORE && system_state_ != State::RACE) return;
  if (!pose_ready_) return;

  if (mission_mode_ == State::MISSION_SKIDPAD) {
    if (!path_generated_) {
      cached_lane_ = generateSkidpadPath();
      path_generated_ = true;
      RCLCPP_INFO(get_logger(), "Skidpad path cached (%zu waypoints)", cached_lane_.waypoints.size());
    }
    auto lane = cached_lane_;
    lane.header.stamp    = now();
    lane.header.frame_id = "map";
    waypoints_pub_->publish(lane);
    publishVisualization(lane, 0.0f, 1.0f, 1.0f);  // cyan for skidpad
  } else if (mission_mode_ == State::MISSION_ACCELERATION) {
    if (!path_generated_) {
      cached_lane_ = generateAccelerationPath();
      path_generated_ = true;
      RCLCPP_INFO(get_logger(), "Acceleration path cached (%zu waypoints)", cached_lane_.waypoints.size());
    }
    auto lane = cached_lane_;
    lane.header.stamp    = now();
    lane.header.frame_id = "map";
    waypoints_pub_->publish(lane);
    publishVisualization(lane, 1.0f, 0.5f, 0.0f);  // orange for acceleration
  }
  // TRACKDRIVE: forwarded by onCenterline callback
}

void PathGeneratorNode::onCenterline(const autoware_msgs::msg::Lane::SharedPtr msg)
{
  // Only forward trackdrive centerline
  if (mission_mode_ != State::MISSION_TRACKDRIVE) return;
  if (system_state_ != State::EXPLORE && system_state_ != State::RACE) return;

  // Update velocity for trackdrive
  auto lane = *msg;
  for (auto & wp : lane.waypoints) {
    wp.twist.twist.linear.x = trackdrive_velocity_;
  }
  waypoints_pub_->publish(lane);
  publishVisualization(lane, 0.0f, 1.0f, 0.0f);  // green for trackdrive
}

autoware_msgs::msg::Lane PathGeneratorNode::generateSkidpadPath() const
{
  autoware_msgs::msg::Lane lane;

  // FSG Skidpad: two circles of radius 9.125m
  // Right circle first (standard FSG direction), then left circle
  // Start at vehicle position
  const double cx = current_pose_.pose.position.x;
  const double cy = current_pose_.pose.position.y;
  const double z  = current_pose_.pose.position.z;

  // Vehicle heading
  const auto & q = current_pose_.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Circle centers: perpendicular to heading, offset by radius
  const double right_cx = cx + skidpad_radius_ * std::sin(yaw);
  const double right_cy = cy - skidpad_radius_ * std::cos(yaw);
  const double left_cx  = cx - skidpad_radius_ * std::sin(yaw);
  const double left_cy  = cy + skidpad_radius_ * std::cos(yaw);

  const double d_theta = 2.0 * M_PI / skidpad_points_;

  // Two laps right circle, two laps left circle (FSG rules)
  // Right circle: CW from vehicle tangent point (θ=π/2), theta decreases
  for (int i = 0; i <= 2 * skidpad_points_; ++i) {
    const double theta = M_PI_2 - i * d_theta;
    autoware_msgs::msg::Waypoint wp;
    wp.pose.pose.position.x = right_cx + skidpad_radius_ * std::cos(theta);
    wp.pose.pose.position.y = right_cy + skidpad_radius_ * std::sin(theta);
    wp.pose.pose.position.z = z;
    wp.pose.pose.orientation.w = 1.0;
    wp.twist.twist.linear.x = skidpad_velocity_;
    lane.waypoints.push_back(wp);
  }
  // Left circle: CCW from vehicle tangent point (θ=-π/2), theta increases
  for (int i = 0; i <= 2 * skidpad_points_; ++i) {
    const double theta = -M_PI_2 + i * d_theta;
    autoware_msgs::msg::Waypoint wp;
    wp.pose.pose.position.x = left_cx + skidpad_radius_ * std::cos(theta);
    wp.pose.pose.position.y = left_cy + skidpad_radius_ * std::sin(theta);
    wp.pose.pose.position.z = z;
    wp.pose.pose.orientation.w = 1.0;
    wp.twist.twist.linear.x = skidpad_velocity_;
    lane.waypoints.push_back(wp);
  }

  RCLCPP_INFO(get_logger(), "SKIDPAD V2 pts=%d -> %zu waypoints", skidpad_points_, lane.waypoints.size());
  return lane;
}

autoware_msgs::msg::Lane PathGeneratorNode::generateAccelerationPath() const
{
  autoware_msgs::msg::Lane lane;

  const double cx  = current_pose_.pose.position.x;
  const double cy  = current_pose_.pose.position.y;
  const double z   = current_pose_.pose.position.z;

  // Vehicle heading direction
  const auto & q = current_pose_.pose.orientation;
  const double yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  const double dx = std::cos(yaw);
  const double dy = std::sin(yaw);

  // Waypoints every 1m along straight line
  const int num_points = static_cast<int>(acceleration_length_);
  for (int i = 0; i <= num_points; ++i) {
    autoware_msgs::msg::Waypoint wp;
    wp.pose.pose.position.x = cx + i * dx;
    wp.pose.pose.position.y = cy + i * dy;
    wp.pose.pose.position.z = z;
    wp.pose.pose.orientation.w = 1.0;
    // Ramp down velocity in last 10m
    const double remaining = acceleration_length_ - i;
    wp.twist.twist.linear.x = (remaining < 10.0)
      ? acceleration_velocity_ * (remaining / 10.0)
      : acceleration_velocity_;
    lane.waypoints.push_back(wp);
  }

  RCLCPP_INFO(get_logger(), "Acceleration path generated: %zu waypoints", lane.waypoints.size());
  return lane;
}

void PathGeneratorNode::publishVisualization(
  const autoware_msgs::msg::Lane & lane,
  float r, float g, float b)
{
  visualization_msgs::msg::MarkerArray arr;

  // LINE_STRIP through all waypoints — ADD with same ns/id replaces in place
  visualization_msgs::msg::Marker line;
  line.header = lane.header;
  line.ns     = "planned_path";
  line.id     = 0;
  line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.08;  // line width
  line.color.r = r;
  line.color.g = g;
  line.color.b = b;
  line.color.a = 0.9f;

  for (const auto & wp : lane.waypoints) {
    geometry_msgs::msg::Point p;
    p.x = wp.pose.pose.position.x;
    p.y = wp.pose.pose.position.y;
    p.z = wp.pose.pose.position.z;
    line.points.push_back(p);
  }
  arr.markers.push_back(line);
  viz_pub_->publish(arr);
}

void PathGeneratorNode::publishTrajectory()
{
  if (trajectory_.size() < 2) return;

  visualization_msgs::msg::MarkerArray arr;

  // LINE_STRIP of driven positions — ADD with same ns/id replaces previous marker
  visualization_msgs::msg::Marker line;
  line.header.frame_id = "map";
  line.header.stamp    = now();
  line.ns     = "driven_trajectory";
  line.id     = 0;
  line.type   = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.scale.x = 0.06;  // slightly thinner than planned path
  line.color.r = 1.0f;
  line.color.g = 0.85f;
  line.color.b = 0.0f;
  line.color.a = 0.9f;
  line.points = trajectory_;

  arr.markers.push_back(line);
  trajectory_viz_pub_->publish(arr);
}

}  // namespace path_generator

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_generator::PathGeneratorNode>());
  rclcpp::shutdown();
  return 0;
}
