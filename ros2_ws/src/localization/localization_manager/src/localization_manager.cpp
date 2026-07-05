#include "localization_manager/localization_manager.hpp"

namespace localization_manager
{

using MissionState = wuta_msgs::msg::MissionState;

LocalizationManager::LocalizationManager(const rclcpp::NodeOptions & options)
: Node("localization_manager", options)
{
  // 订阅任务状态，决定当前采用 EKF 还是 NDT 定位输出。
  mission_sub_ = create_subscription<MissionState>(
    "/system/mission_state", 10,
    std::bind(&LocalizationManager::onMissionState, this, std::placeholders::_1));

  // EKF 输出订阅：在 EXPLORE 模式下启用，表示定位使用 KISS-ICP + EKF。
  ekf_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odometry/filtered", 10,
    std::bind(&LocalizationManager::onEkfOdom, this, std::placeholders::_1));

  // NDT 输出订阅：在 RACE 模式下启用，表示定位使用地图匹配。
  ndt_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/ndt/pose", 10,
    std::bind(&LocalizationManager::onNdtPose, this, std::placeholders::_1));

  // 发布者：统一输出给规划和控制层的定位结果。
  // 统一对外输出当前可用于规划和控制的定位结果，屏蔽底层 EKF/NDT 的差异。
  pose_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("/localization/pose", 10);
  ready_pub_ = create_publisher<std_msgs::msg::Bool>("/system/localization_ready", 10);

  RCLCPP_INFO(get_logger(), "LocalizationManager ready. Default mode: KISS-ICP + EKF");
}

void LocalizationManager::onMissionState(const MissionState::SharedPtr msg)
{
  if (msg->localization_mode == active_mode_) return;

  active_mode_ = msg->localization_mode;

  if (active_mode_ == MissionState::LOC_KISS_ICP) {
    RCLCPP_INFO(get_logger(), "Localization mode: KISS-ICP + EKF (EXPLORE)");
  } else {
    RCLCPP_INFO(get_logger(), "Localization mode: NDT map matching (RACE)");
  }
}

void LocalizationManager::onEkfOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (active_mode_ != MissionState::LOC_KISS_ICP) return;

  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.header.frame_id = "map";
  pose.pose = msg->pose.pose;
  pose_pub_->publish(pose);

  publishLocalizationReady(true);
}

void LocalizationManager::onNdtPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  if (active_mode_ != MissionState::LOC_NDT) return;

  pose_pub_->publish(*msg);
  publishLocalizationReady(true);
}

void LocalizationManager::publishLocalizationReady(bool ready)
{
  std_msgs::msg::Bool msg;
  msg.data = ready;
  ready_pub_->publish(msg);
}

}  // namespace localization_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<localization_manager::LocalizationManager>());
  rclcpp::shutdown();
  return 0;
}
