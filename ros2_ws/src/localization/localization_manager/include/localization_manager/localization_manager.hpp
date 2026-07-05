#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "wuta_msgs/msg/mission_state.hpp"

namespace localization_manager
{

/**
 * 定位管理器。
 *
 * 该节点负责接收两种定位源的输出：
 *   - EXPLORE 模式：EKF 输出（KISS-ICP + CG-410 融合）
 *   - RACE 模式：NDT 地图匹配输出
 *
 * 最终统一发布到 /localization/pose 主题，
 * 规划与控制模块只需要订阅这一条统一接口。
 *
 * 模式切换由 MissionManager 通过 /system/mission_state 驱动。
 */
class LocalizationManager : public rclcpp::Node
{
public:
  explicit LocalizationManager(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onMissionState(const wuta_msgs::msg::MissionState::SharedPtr msg);
  void onEkfOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onNdtPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  void publishLocalizationReady(bool ready);

  uint8_t active_mode_{wuta_msgs::msg::MissionState::LOC_KISS_ICP};

  // Subscriptions
  rclcpp::Subscription<wuta_msgs::msg::MissionState>::SharedPtr mission_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ekf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr ndt_sub_;

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub_;
};

}  // namespace localization_manager
