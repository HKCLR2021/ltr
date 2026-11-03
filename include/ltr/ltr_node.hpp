#pragma once

#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "sensor_msgs/msg/battery_state.hpp"

#include "ltr/ltr_api.hpp"
#include "ltr/msg/vel_cmd.hpp"
#include "ltr/msg/power_state.hpp"
#include "ltr/msg/sport_mode_cmd.hpp"

namespace ltr
{

class LtrNode : public rclcpp::Node, public LtrApi
{
public:
  explicit LtrNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~LtrNode() override;

  void vel_cmd(double vx, double vy, double vz, double wx, double wy, double wz) override;
  void sport_mode_cmd(uint8_t device_id, uint8_t mode) override;

  bool vel_cmd_with_ack(double vx, double vy, double vz,
                        double wx, double wy, double wz);
  bool sport_mode_cmd_with_ack(uint8_t device_id, uint8_t mode);

  void vel_ack_cb(const ltr::msg::VelCmd::SharedPtr msg);
  void sport_mode_ack_cb(const ltr::msg::SportModeCmd::SharedPtr msg);
  void power_state_cb(const ltr::msg::PowerState::SharedPtr msg);

  rclcpp::Publisher<ltr::msg::VelCmd>::SharedPtr vel_pub()   const noexcept { return vel_pub_; }
  rclcpp::Publisher<ltr::msg::SportModeCmd>::SharedPtr sport_pub() const noexcept { return sport_pub_; }

private:
  // --- QoS helpers ---
  static rclcpp::QoS cmd_qos();
  static rclcpp::QoS ack_qos();

  rclcpp::CallbackGroup::SharedPtr ack_cbg_;

  // --- publishers ---
  rclcpp::Publisher<ltr::msg::VelCmd>::SharedPtr        vel_pub_;
  rclcpp::Publisher<ltr::msg::SportModeCmd>::SharedPtr  sport_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr bat_state_pub_;

  // --- subscribers ---
  rclcpp::Subscription<ltr::msg::VelCmd>::SharedPtr       vel_ack_sub_;
  rclcpp::Subscription<ltr::msg::SportModeCmd>::SharedPtr sport_ack_sub_;
  rclcpp::Subscription<ltr::msg::PowerState>::SharedPtr   bat_state_sub_;

  // --- command / ack synchronization -------------------------------
  // Two different mutexes on purpose: the caller may hold *_call_mutex_
  // while it waits, and the ack callback must be able to grab ack_mutex_.
  std::mutex vel_call_mutex_;
  std::mutex sport_call_mutex_;

  std::mutex              ack_mutex_;
  std::condition_variable ack_cv_;

  std::optional<ltr::msg::VelCmd>       pending_vel_cmd_;
  bool                                  vel_ack_received_  = false;

  std::optional<ltr::msg::SportModeCmd> pending_sport_cmd_;
  bool                                  sport_ack_received_ = false;

  int vel_ack_timeout_ms_   = 500;   // param: vel_ack_timeout_ms
  int sport_ack_timeout_ms_ = 1000;  // param: sport_ack_timeout_ms (mode switch can be slow)

  // topic / frame names (overridable via params)
  std::string vel_topic_;
  std::string sport_topic_;
  std::string vel_ack_topic_;
  std::string sport_ack_topic_;
  std::string power_state_topic_;
};

} // namespace ltr