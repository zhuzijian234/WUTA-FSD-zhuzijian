#include "controller/controller_node.hpp"
#include <visualization_msgs/msg/marker.hpp>

namespace controller
{

using MissionState = wuta_msgs::msg::MissionState;

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: Node("controller_node", options)
{
  // 构造函数中完成控制器所有依赖组件的初始化：
  // 1. 读取车辆参数和 Pure Pursuit 参数；
  // 2. 创建 Pure Pursuit 和 TwistFilter 对象；
  // 3. 初始化订阅者/发布者和固定频率控制定时器。
  // 车辆参数：控制器需要知道轴距、重心位置和最大转角，才能计算转向角。
  VehicleParams vp;
  vp.wheel_base       = declare_parameter("wheel_base",       vp.wheel_base);
  vp.lf               = declare_parameter("lf",               vp.lf);
  vp.max_steer_angle  = declare_parameter("max_steer_angle",  vp.max_steer_angle);

  // Pure Pursuit 的参数：lookahead 距离决定控制器向前看多远。
  PurePursuit::Config pp_cfg;
  pp_cfg.ld_ratio       = declare_parameter("ld_ratio",       pp_cfg.ld_ratio);
  pp_cfg.min_lookahead  = declare_parameter("min_lookahead",  pp_cfg.min_lookahead);
  pp_cfg.max_lookahead  = declare_parameter("max_lookahead",  pp_cfg.max_lookahead);

  // 创建控制算法对象和安全滤波器。
  pure_pursuit_ = std::make_unique<PurePursuit>(vp, pp_cfg);
  twist_filter_ = std::make_unique<TwistFilter>(vp);

  // 控制频率，单位 Hz，表示每秒执行多少次控制循环。
  const int rate_hz = declare_parameter("control_rate_hz", 50);

  // 订阅者：控制器的输入端，分别接收定位、速度、规划路径和任务状态。
  // 这是控制闭环的输入端：定位、速度、规划路径和任务状态都会影响控制输出。
  pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/localization/pose", 10,
    std::bind(&ControllerNode::onPose, this, std::placeholders::_1));

  vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "/localization/velocity", 10,
    std::bind(&ControllerNode::onVelocity, this, std::placeholders::_1));

  waypoints_sub_ = create_subscription<autoware_msgs::msg::Lane>(
    "/planning/final_waypoints", 10,
    std::bind(&ControllerNode::onWaypoints, this, std::placeholders::_1));

  mission_sub_ = create_subscription<MissionState>(
    "/system/mission_state", 10,
    std::bind(&ControllerNode::onMissionState, this, std::placeholders::_1));

  // 发布者：输出最终控制指令和可视化目标点。
  // 控制器把最终的速度/转角指令发布给下游执行层，供 VCU 或车辆控制模块使用。
  cmd_pub_ = create_publisher<autoware_msgs::msg::Command>("/control/command", 10);
  target_viz_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/control/target_viz", 10);

  // 定时器驱动控制循环：固定频率触发 controlLoop，形成闭环控制。
  control_timer_ = create_wall_timer(
    std::chrono::milliseconds(1000 / rate_hz),
    std::bind(&ControllerNode::controlLoop, this));

  RCLCPP_INFO(get_logger(), "ControllerNode ready. rate=%dHz, LD_ratio=%.1f",
    rate_hz, pp_cfg.ld_ratio);
}

void ControllerNode::onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // 定位消息是控制闭环的基础输入。控制器依赖它知道“车辆现在在哪里、朝哪个方向”。
  // 从定位消息中提取车辆当前的平面位置。
  vehicle_state_.x = msg->pose.position.x;
  vehicle_state_.y = msg->pose.position.y;

  // 把四元数转换成 yaw 角，便于 Pure Pursuit 计算车头方向。
  const auto & q = msg->pose.orientation;
  vehicle_state_.yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // 标记定位信息已准备好，控制循环才可以开始工作。
  pose_ready_ = true;
}

void ControllerNode::onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // 速度消息给出线速度分量，取向量模长作为当前车速。
  const double vx = msg->twist.linear.x;
  const double vy = msg->twist.linear.y;
  vehicle_state_.velocity = std::sqrt(vx * vx + vy * vy);
}

void ControllerNode::onWaypoints(const autoware_msgs::msg::Lane::SharedPtr msg)
{
  // 路径消息来自路径生成器，里面包含一串参考点；控制器通过它决定下一步该往哪里走。
  // 路径生成器会发布一组参考路径点，控制器用它们形成跟踪目标。
  waypoints_ = msg->waypoints;
  waypoints_ready_ = !waypoints_.empty();
}

void ControllerNode::onMissionState(const MissionState::SharedPtr msg)
{
  // 只有在 EXPLORE 或 RACE 阶段才允许控制器输出控制命令。
  enabled_ = (msg->state == MissionState::EXPLORE || msg->state == MissionState::RACE);

  if (!enabled_) {
    // 若不在可控状态，则重置滤波器并发布停止指令，避免继续跟踪旧路径。
    twist_filter_->reset();
    autoware_msgs::msg::Command stop;
    stop.speed = 0.0;
    stop.angle = 0.0;
    stop.dv_state = 4;
    cmd_pub_->publish(stop);
  }
}

void ControllerNode::controlLoop()
{
  // 控制循环相当于整个控制器的主函数：
  // 每隔固定周期就根据当前状态重新计算一条控制指令。
  // 只有在系统允许控制、定位已就绪、路径已收到时才执行控制计算。
  if (!enabled_ || !pose_ready_ || !waypoints_ready_) return;

  // 1. 利用 Pure Pursuit 生成原始控制指令：转角和目标速度。
  auto raw_cmd = pure_pursuit_->compute(vehicle_state_, waypoints_);
  if (!raw_cmd.valid) return;

  // 2. 使用安全滤波器对控制指令做平滑和限幅，避免急转弯或剧烈加速。
  auto filtered = twist_filter_->filter(raw_cmd.steering_angle, raw_cmd.velocity);

  // 3. 把滤波后的控制命令封装成 autoware 的 Command 消息并发布。
  autoware_msgs::msg::Command cmd;
  cmd.speed    = filtered.velocity;
  cmd.angle    = filtered.steering_angle;
  cmd.dv_state = filtered.emergency ? 6 : 4;  // 4=normal, 6=emergency
  cmd_pub_->publish(cmd);

  // 4. 如果有订阅者，发布目标点的可视化标记，便于 RViz 查看控制目标。
  if (target_viz_pub_->get_subscription_count() > 0 &&
      pure_pursuit_->targetIndex() < static_cast<int>(waypoints_.size()))
  {
    const auto & wp = waypoints_[pure_pursuit_->targetIndex()];
    publishVisualization(
      wp.pose.pose.position.x,
      wp.pose.pose.position.y);
  }
}

void ControllerNode::publishVisualization(double target_x, double target_y)
{
  // 可视化消息集合，包含目标点和当前 lookahead 圆。
  visualization_msgs::msg::MarkerArray arr;
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp    = now();
  m.ns     = "pp_target";
  m.id     = 0;
  m.type   = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x  = target_x;
  m.pose.position.y  = target_y;
  m.pose.position.z  = 0.5;
  m.pose.orientation.w = 1.0;
  m.scale.x = 0.5; m.scale.y = 0.5; m.scale.z = 0.5;
  m.color.r = 1.0f; m.color.g = 0.3f; m.color.b = 0.0f; m.color.a = 1.0f;
  arr.markers.push_back(m);

  // 生成 lookahead 圆，表示当前控制器向前看的范围，便于观察跟踪性能。
  visualization_msgs::msg::Marker circle;
  circle.header = m.header;
  circle.ns   = "pp_lookahead";
  circle.id   = 1;
  circle.type = visualization_msgs::msg::Marker::CYLINDER;
  circle.action = visualization_msgs::msg::Marker::ADD;
  circle.pose.position.x = vehicle_state_.x;
  circle.pose.position.y = vehicle_state_.y;
  circle.pose.position.z = 0.0;
  circle.pose.orientation.w = 1.0;
  const double ld = pure_pursuit_->lookaheadDistance();
  circle.scale.x = ld * 2; circle.scale.y = ld * 2; circle.scale.z = 0.05;
  circle.color.r = 0.0f; circle.color.g = 0.6f; circle.color.b = 1.0f; circle.color.a = 0.3f;
  arr.markers.push_back(circle);

  target_viz_pub_->publish(arr);
}

}  // namespace controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<controller::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
