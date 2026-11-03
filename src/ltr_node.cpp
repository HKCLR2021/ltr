#include "ltr/ltr_node.hpp"
#include "ltr/pub_utils.hpp"

#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

namespace ltr
{

using std::placeholders::_1;

namespace
{
constexpr double kCmdEps = 1e-4;  // float round-trip tolerance

ltr::msg::VelCmd make_vel_msg(double vx, double vy, double vz,
                              double wx, double wy, double wz)
{
  ltr::msg::VelCmd m;
  m.lin_vx = static_cast<float>(vx);
  m.lin_vy = static_cast<float>(vy);
  m.lin_vz = static_cast<float>(vz);
  m.ang_wx = static_cast<float>(wx);
  m.ang_wy = static_cast<float>(wy);
  m.ang_wz = static_cast<float>(wz);
  return m;
}

bool vel_cmd_equal(const ltr::msg::VelCmd& a, const ltr::msg::VelCmd& b) noexcept
{
  return std::fabs(a.lin_vx - b.lin_vx) < kCmdEps &&
         std::fabs(a.lin_vy - b.lin_vy) < kCmdEps &&
         std::fabs(a.lin_vz - b.lin_vz) < kCmdEps &&
         std::fabs(a.ang_wx - b.ang_wx) < kCmdEps &&
         std::fabs(a.ang_wy - b.ang_wy) < kCmdEps &&
         std::fabs(a.ang_wz - b.ang_wz) < kCmdEps;
}

bool sport_cmd_equal(const ltr::msg::SportModeCmd& a, const ltr::msg::SportModeCmd& b) noexcept
{
  return a.device_id == b.device_id && a.mode == b.mode;
}
} // namespace

// ---------- QoS ----------
rclcpp::QoS LtrNode::cmd_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1))
    .reliable()
    .durability_volatile()
    .lifespan(rclcpp::Duration(100ms));
}

rclcpp::QoS LtrNode::ack_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1))
    .reliable()
    .durability_volatile();
}

// ---------- ctor / dtor ----------
LtrNode::LtrNode(const rclcpp::NodeOptions& opts)
: rclcpp::Node("ltr_bridge", opts)
{
  ack_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  vel_topic_         = declare_parameter<std::string>("vel_cmd_topic",     "/ltr/vel_cmd");
  sport_topic_       = declare_parameter<std::string>("sport_mode_topic",  "/ltr/sportmode_cmd");
  vel_ack_topic_     = declare_parameter<std::string>("vel_ack_topic",     "/ltr/vel_cmd_received");
  sport_ack_topic_   = declare_parameter<std::string>("sport_ack_topic",   "/ltr/sportmode_cmd_received");
  power_state_topic_ = declare_parameter<std::string>("power_state_topic", "/ltr/power_state");

  vel_ack_timeout_ms_   = declare_parameter<int>("vel_ack_timeout_ms",   25);
  sport_ack_timeout_ms_ = declare_parameter<int>("sport_ack_timeout_ms", 50);

  const auto cmdq = cmd_qos();
  const auto ackq = ack_qos();

  vel_pub_       = create_publisher<ltr::msg::VelCmd>       (vel_topic_,   cmdq);
  sport_pub_     = create_publisher<ltr::msg::SportModeCmd> (sport_topic_, cmdq);
  bat_state_pub_ = create_publisher<sensor_msgs::msg::BatteryState>("/bottom/battery_state", 1);

  rclcpp::SubscriptionOptions ack_sub_options;
  ack_sub_options.callback_group = ack_cbg_;

  vel_ack_sub_ = create_subscription<ltr::msg::VelCmd>(
    vel_ack_topic_, 
    ackq,
    std::bind(&LtrNode::vel_ack_cb, this, _1),
    ack_sub_options);

  sport_ack_sub_ = create_subscription<ltr::msg::SportModeCmd>(
    sport_ack_topic_, 
    ackq,
    std::bind(&LtrNode::sport_mode_ack_cb, this, _1),
    ack_sub_options);

  bat_state_sub_ = create_subscription<ltr::msg::PowerState>(
    power_state_topic_, 1,
    std::bind(&LtrNode::power_state_cb, this, _1));

  RCLCPP_INFO(get_logger(), "LTR bridge up.");
}

LtrNode::~LtrNode() = default;

void LtrNode::vel_cmd(double vx, double vy, double vz,
                      double wx, double wy, double wz)
{
  vel_pub_->publish(make_vel_msg(vx, vy, vz, wx, wy, wz));
}

void LtrNode::sport_mode_cmd(uint8_t device_id, uint8_t mode)
{
  ltr::msg::SportModeCmd msg;
  msg.device_id = device_id;
  msg.mode      = mode;

  sport_pub_->publish(msg);
}

// ---------- publishers ----------
bool LtrNode::vel_cmd_with_ack(double vx, double vy, double vz,
                               double wx, double wy, double wz)
{
  std::lock_guard<std::mutex> call_lock(vel_call_mutex_);

  const auto msg = make_vel_msg(vx, vy, vz, wx, wy, wz);

  {
    std::lock_guard<std::mutex> lk(ack_mutex_);
    pending_vel_cmd_  = msg;
    vel_ack_received_ = false;
  }

  const auto t_start = std::chrono::steady_clock::now();
  vel_pub_->publish(msg);

  std::unique_lock<std::mutex> lk(ack_mutex_);
  const bool ok = ack_cv_.wait_for(
    lk,
    std::chrono::milliseconds(vel_ack_timeout_ms_),
    [this] { return vel_ack_received_; });

  const auto ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_start).count();

  if (ok) 
  {
    RCLCPP_INFO(get_logger(),
                "vel_cmd_with_ack(lin=[%.3f,%.3f,%.3f], ang=[%.3f,%.3f,%.3f]): "
                "ack in %.2f ms",
                vx, vy, vz, wx, wy, wz, ms);
  } 
  else 
  {
    RCLCPP_WARN(get_logger(),
                "vel_cmd_with_ack(lin=[%.3f,%.3f,%.3f], ang=[%.3f,%.3f,%.3f]): "
                "TIMEOUT after %.2f ms (limit %d ms)",
                vx, vy, vz, wx, wy, wz, ms, vel_ack_timeout_ms_);
  }

  pending_vel_cmd_.reset();
  return ok;
}

bool LtrNode::sport_mode_cmd_with_ack(uint8_t device_id, uint8_t mode)
{
  std::lock_guard<std::mutex> call_lock(sport_call_mutex_);

  ltr::msg::SportModeCmd msg;
  msg.device_id = device_id;
  msg.mode      = mode;

  {
    std::lock_guard<std::mutex> lk(ack_mutex_);
    pending_sport_cmd_  = msg;
    sport_ack_received_ = false;
  }

  const auto t_start = std::chrono::steady_clock::now();
  sport_pub_->publish(msg);

  std::unique_lock<std::mutex> lk(ack_mutex_);
  const bool ok = ack_cv_.wait_for(
    lk,
    std::chrono::milliseconds(sport_ack_timeout_ms_),
    [this] { return sport_ack_received_; });

  const auto ms = std::chrono::duration<double, std::milli>(
    std::chrono::steady_clock::now() - t_start).count();

  if (ok) 
  {
    RCLCPP_INFO(get_logger(),
                "sport_mode_cmd_with_ack(device_id=%u, mode=%u): ack in %.2f ms",
                static_cast<unsigned>(device_id),
                static_cast<unsigned>(mode), ms);
  } 
  else 
  {
    RCLCPP_WARN(get_logger(),
                "sport_mode_cmd_with_ack(device_id=%u, mode=%u): TIMEOUT after "
                "%.2f ms (limit %d ms)",
                static_cast<unsigned>(device_id),
                static_cast<unsigned>(mode), ms, sport_ack_timeout_ms_);
  }

  pending_sport_cmd_.reset();
  return ok;
}

// ---------- ack callbacks ----------
void LtrNode::vel_ack_cb(const ltr::msg::VelCmd::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(ack_mutex_);
  if (!pending_vel_cmd_) 
  {
    return;
  }
  if (vel_cmd_equal(*msg, *pending_vel_cmd_)) 
  {
    vel_ack_received_ = true;
    ack_cv_.notify_all();
  }
}

void LtrNode::sport_mode_ack_cb(const ltr::msg::SportModeCmd::SharedPtr msg)
{
  std::lock_guard<std::mutex> lk(ack_mutex_);
  if (!pending_sport_cmd_) 
  {
    return;
  }
  if (sport_cmd_equal(*msg, *pending_sport_cmd_)) 
  {
    sport_ack_received_ = true;
    ack_cv_.notify_all();
  }
}

void LtrNode::power_state_cb(const ltr::msg::PowerState::SharedPtr msg)
{
  sensor_msgs::msg::BatteryState bat_msg;

  bat_msg.header.stamp    = get_clock()->now();
  bat_msg.header.frame_id = "base_link";

  bat_msg.voltage = msg->input_voltage;
  bat_msg.current = msg->output_current;

  bat_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  bat_msg.present = true;

  bat_state_pub_->publish(bat_msg);
}

} // namespace ltr