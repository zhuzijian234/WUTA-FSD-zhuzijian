#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <autoware_msgs/msg/lane.hpp>
#include <autoware_msgs/msg/command.hpp>

#include "wuta_msgs/msg/mission_state.hpp"
#include "controller/vehicle_state.hpp"
#include "controller/pure_pursuit.hpp"
#include "controller/twist_filter.hpp"

namespace controller
{

/**
 * 控制器节点主类。
 *
 * 该节点负责把规划层输出的参考路径和定位层输出的当前位姿
 * 组合成一条控制命令，发布给下游执行层。它是整个系统中最靠近
 * 车辆执行器的一层，承担“根据当前状态跟踪参考轨迹”的职责。
 */
class ControllerNode : public rclcpp::Node
{
public:
  // 控制器节点构造函数，负责初始化订阅、发布和定时器。
  explicit ControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 回调函数：分别处理定位、速度、路径和任务状态输入。
  void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void onWaypoints(const autoware_msgs::msg::Lane::SharedPtr msg);
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);
  void controlLoop();

  // 生成目标点和前视圈的可视化消息。
  void publishVisualization(double target_x, double target_y);

  // 算法对象：Pure Pursuit 和安全滤波器。
  std::unique_ptr<PurePursuit>  pure_pursuit_;
  std::unique_ptr<TwistFilter>  twist_filter_;

  // State
  VehicleState vehicle_state_;
  std::vector<autoware_msgs::msg::Waypoint> waypoints_;
  bool pose_ready_{false};
  bool waypoints_ready_{false};
  bool enabled_{false};  // 仅在任务状态为 EXPLORE 或 RACE 时运行控制逻辑

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_;
  rclcpp::Subscription<autoware_msgs::msg::Lane>::SharedPtr waypoints_sub_;
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;

  // Publishers
  rclcpp::Publisher<autoware_msgs::msg::Command>::SharedPtr cmd_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr target_viz_pub_;

  // Control loop timer
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace controller
