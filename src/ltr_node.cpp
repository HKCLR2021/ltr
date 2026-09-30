#include "ltr/ltr_node.hpp"
#include "ltr/pub_utils.hpp"

using namespace std::chrono_literals;

namespace ltr 
{

// ---------- QoS ----------
// Command topics: low-latency, latest-wins, don't retry stale commands.
rclcpp::QoS LtrNode::cmd_qos() 
{
  return rclcpp::QoS(rclcpp::KeepLast(1))
    .reliable()
    .durability_volatile()
    .lifespan(rclcpp::Duration(200ms));
}

// State topics: best-effort, latest sample only.
rclcpp::QoS LtrNode::state_qos() 
{
  return rclcpp::QoS(rclcpp::KeepLast(1))
    .best_effort()
    .durability_volatile();
}

// ---------- ctor / dtor ----------
LtrNode::LtrNode(const rclcpp::NodeOptions& opts)
: rclcpp::Node("ltr_bridge", opts)
{
  // Topic names — match whatever the vendor firmware uses.
  // Configurable so it's easy to remap / test.
  vel_topic_         = declare_parameter<std::string>("vel_cmd_topic",        "ltr/vel_cmd");
  sport_topic_       = declare_parameter<std::string>("sport_mode_topic",     "ltr/sportmode_cmd");
  vel_state_topic_   = declare_parameter<std::string>("vel_state_topic",      "ltr/vel_state");
  sport_state_topic_ = declare_parameter<std::string>("sport_state_topic",    "ltr/sport_mode_state");

  const auto qos = cmd_qos();

  vel_pub_   = create_publisher<ltr::msg::VelCmd>      (vel_topic_,   qos);
  sport_pub_ = create_publisher<ltr::msg::SportModeCmd>(sport_topic_, qos);

  RCLCPP_INFO(get_logger(), "ltr bridge up.");
}

LtrNode::~LtrNode() = default;

// ---------- publishers ----------
void LtrNode::vel_cmd(double vx, double vy, double vz, double wx, double wy, double wz)
{
  ltr::msg::VelCmd msg;
  msg.lin_vx = static_cast<float>(vx);
  msg.lin_vy = static_cast<float>(vy);
  msg.lin_vz = static_cast<float>(vz);
  msg.ang_wx = static_cast<float>(wx);
  msg.ang_wy = static_cast<float>(wy);
  msg.ang_wz = static_cast<float>(wz);
  vel_pub_->publish(msg);
}

void LtrNode::sport_mode_cmd(uint8_t device_id, uint8_t mode)
{
  ltr::msg::SportModeCmd msg;
  msg.device_id = device_id;
  msg.mode      = mode;
  sport_pub_->publish(msg);
}

// ---------- subscriptions (lazy: created on first callback registration) ----------
void LtrNode::on_vel_state(VelStateCb cb)
{
  std::lock_guard<std::mutex> lk(cb_mtx_);
  vel_state_cb_ = std::move(cb);
  if (vel_state_cb_ && !vel_sub_)
  {
    vel_sub_ = create_subscription<ltr::msg::VelCmd>(
      vel_state_topic_, 
      state_qos(),
      [this](ltr::msg::VelCmd::ConstSharedPtr msg) {
        VelStateCb cb_local;
        {
          std::lock_guard<std::mutex> lk(cb_mtx_);
          cb_local = vel_state_cb_;
        }
        if (!cb_local) 
          return;
        const double v[6] = { 
          msg->lin_vx, msg->lin_vy, msg->lin_vz, 
          msg->ang_wx, msg->ang_wy, msg->ang_wz 
        };
        cb_local(v);
      });
  }
}

void LtrNode::on_sport_mode(SportModeCb cb)
{
  std::lock_guard<std::mutex> lk(cb_mtx_);
  sport_mode_cb_ = std::move(cb);
  if (sport_mode_cb_ && !sport_sub_) 
  {
    sport_sub_ = create_subscription<ltr::msg::SportModeCmd>(
      sport_state_topic_, state_qos(),
      [this](ltr::msg::SportModeCmd::ConstSharedPtr msg) {
        SportModeCb cb_local;
        {
          std::lock_guard<std::mutex> lk(cb_mtx_);
          cb_local = sport_mode_cb_;
        }
        if (cb_local) 
          cb_local(msg->device_id, msg->mode);
      });
  }
}

} // namespace ltr