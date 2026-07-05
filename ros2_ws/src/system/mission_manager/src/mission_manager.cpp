#include "mission_manager/mission_manager.hpp"

namespace mission_manager
{

using State = wuta_msgs::msg::MissionState;

MissionManager::MissionManager(const rclcpp::NodeOptions & options)
: Node("mission_manager", options)
{
  // 从参数服务器读取比赛任务模式，默认是 trackdrive。
  const std::string mode_str = declare_parameter<std::string>("mission_mode", "trackdrive");
  if (mode_str == "skidpad")       mission_mode_ = State::MISSION_SKIDPAD;
  else if (mode_str == "acceleration") mission_mode_ = State::MISSION_ACCELERATION;
  else                             mission_mode_ = State::MISSION_TRACKDRIVE;

  // 发布者：把当前系统状态持续广播给所有模块。
  // 任务管理器是整个系统的状态总线，所有模块都通过它获取当前任务阶段与模式。
  state_pub_ = create_publisher<State>("/system/mission_state", 10);
  inspection_result_pub_ = create_publisher<std_msgs::msg::String>(
    "/system/inspection_result", 10);  // 预留，车检结果输出

  // 订阅者：这些输入决定当前系统是否进入建图、竞速、应急停机等阶段。
  // 这些输入决定当前系统是否进入建图、竞速、应急停机等阶段。
  cone_map_sub_ = create_subscription<wuta_msgs::msg::ConeMap>(
    "/mapping/cone_map", 10,
    std::bind(&MissionManager::onConeMap, this, std::placeholders::_1));

  emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/emergency", 10,
    std::bind(&MissionManager::onEmergency, this, std::placeholders::_1));

  mission_mode_sub_ = create_subscription<std_msgs::msg::String>(
    "/system/mission_mode_cmd", 10,
    std::bind(&MissionManager::onMissionModeCmd, this, std::placeholders::_1));

  lidar_status_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/lidar_ready", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      lidar_ready_ = msg->data;
      if (lidar_ready_ && localization_ready_ && current_state_ == State::IDLE) {
        transitionTo(State::READY);
      }
    });

  localization_status_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/localization_ready", 10,
    [this](const std_msgs::msg::Bool::SharedPtr msg) {
      localization_ready_ = msg->data;
      if (lidar_ready_ && localization_ready_ && current_state_ == State::IDLE) {
        transitionTo(State::READY);
      }
    });

  // ---------------------------------------------------------------------------
  // INSPECTION interface — 预留，暂不接其他模块
  // 发布 true 到此 topic 触发车检流程
  // ---------------------------------------------------------------------------
  inspection_trigger_sub_ = create_subscription<std_msgs::msg::Bool>(
    "/system/inspection_trigger", 10,
    std::bind(&MissionManager::onInspectionTrigger, this, std::placeholders::_1));

  // 周期性广播任务状态，频率 10Hz，确保所有模块都能及时知道当前阶段。
  state_timer_ = create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&MissionManager::publishState, this));

  RCLCPP_INFO(get_logger(), "Mission Manager initialized. mode=%s state=IDLE",
    mode_str.c_str());
}

void MissionManager::transitionTo(uint8_t new_state)
{
  // 状态迁移函数：负责变化系统状态，并同步设置定位模式。
  const auto state_name = [](uint8_t s) -> std::string {
    switch (s) {
      case State::IDLE:         return "IDLE";
      case State::READY:        return "READY";
      case State::INSPECTION:   return "INSPECTION";
      case State::EXPLORE:      return "EXPLORE";
      case State::MAPPING_DONE: return "MAPPING_DONE";
      case State::RACE:         return "RACE";
      case State::FINISH:       return "FINISH";
      case State::EMERGENCY:    return "EMERGENCY";
      default:                  return "UNKNOWN";
    }
  };

  RCLCPP_INFO(get_logger(), "State: %s → %s",
    state_name(current_state_).c_str(), state_name(new_state).c_str());

  current_state_ = new_state;

  // 进入 RACE 时，切换到 NDT 定位；进入 EXPLORE 时，切换到 KISS-ICP + EKF 定位。
  if (new_state == State::RACE) {
    localization_mode_ = State::LOC_NDT;
    RCLCPP_INFO(get_logger(), "Localization: NDT map matching");
  } else if (new_state == State::EXPLORE) {
    localization_mode_ = State::LOC_KISS_ICP;
    RCLCPP_INFO(get_logger(), "Localization: KISS-ICP + EKF");
  }

  publishState();
}

void MissionManager::publishState()
{
  State msg;
  msg.header.stamp    = now();
  msg.state           = current_state_;
  msg.mission_mode    = mission_mode_;
  msg.localization_mode = localization_mode_;
  state_pub_->publish(msg);
}

void MissionManager::onConeMap(const wuta_msgs::msg::ConeMap::SharedPtr msg)
{
  if (msg->is_closed && !map_closed_ && current_state_ == State::EXPLORE) {
    map_closed_ = true;
    RCLCPP_INFO(get_logger(), "Cone map closed. %zu blue + %zu yellow cones.",
      msg->blue_cones.size(), msg->yellow_cones.size());
    transitionTo(State::MAPPING_DONE);
    // TODO: 等待 NDT 地图生成完成后，再切换到 RACE 状态
  }
}

void MissionManager::onEmergency(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    RCLCPP_ERROR(get_logger(), "EMERGENCY triggered!");
    transitionTo(State::EMERGENCY);
  }
}

void MissionManager::onMissionModeCmd(const std_msgs::msg::String::SharedPtr msg)
{
  if (current_state_ != State::IDLE && current_state_ != State::READY) {
    RCLCPP_WARN(get_logger(), "Cannot change mission mode in state %d", current_state_);
    return;
  }
  if (msg->data == "trackdrive")    mission_mode_ = State::MISSION_TRACKDRIVE;
  else if (msg->data == "skidpad")  mission_mode_ = State::MISSION_SKIDPAD;
  else if (msg->data == "acceleration") mission_mode_ = State::MISSION_ACCELERATION;
  else {
    RCLCPP_WARN(get_logger(), "Unknown mission mode: %s", msg->data.c_str());
    return;
  }
  RCLCPP_INFO(get_logger(), "Mission mode set to: %s", msg->data.c_str());
  publishState();
}

// ---------------------------------------------------------------------------
// INSPECTION — 预留接口，暂未接入其他模块
// ---------------------------------------------------------------------------

void MissionManager::onInspectionTrigger(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (!msg->data) return;

  if (current_state_ != State::IDLE && current_state_ != State::READY) {
    RCLCPP_WARN(get_logger(), "Inspection only available in IDLE/READY state.");
    return;
  }

  RCLCPP_INFO(get_logger(), "Inspection triggered.");
  transitionTo(State::INSPECTION);
  runInspection();
  sendInspectionCAN();

  // Return to READY after inspection
  transitionTo(State::READY);
}

void MissionManager::runInspection()
{
  // TODO: 检查各传感器 topic 是否在线（LiDAR、相机、CG-410）
  // TODO: 检查 TF tree 是否完整
  // TODO: 发布检查结果到 /system/inspection_result

  RCLCPP_INFO(get_logger(), "[INSPECTION] 传感器检查 — 尚未实现。");

  std_msgs::msg::String result;
  result.data = "INSPECTION_NOT_IMPLEMENTED";
  inspection_result_pub_->publish(result);
}

void MissionManager::sendInspectionCAN()
{
  // TODO: 通过 CAN 接口向 VCU 发送车检测试报文
  // 建议通过 can_msgs::msg::Frame 发布到 /can/tx topic
  // 具体报文格式需根据 VCU 协议文档确定

  RCLCPP_INFO(get_logger(), "[INSPECTION] VCU CAN 测试 — 尚未实现。");
}

}  // namespace mission_manager

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_manager::MissionManager>());
  rclcpp::shutdown();
  return 0;
}
