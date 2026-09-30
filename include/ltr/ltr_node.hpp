#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include "ltr/ltr_api.hpp"
#include "ltr/msg/vel_cmd.hpp"
#include "ltr/msg/sport_mode_cmd.hpp"

namespace ltr 
{

class LtrNode : public rclcpp::Node, public LtrApi 
{
public:
  explicit LtrNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());
  ~LtrNode() override;

  void vel_cmd(double vx, double vy, double vz, double wx, double wy, double wz) override;
  void sport_mode_cmd(uint8_t mode, uint8_t sub_mode) override;

  void on_vel_state(VelStateCb cb) override;
  void on_sport_mode(SportModeCb cb) override;

  rclcpp::Publisher<ltr::msg::VelCmd>::SharedPtr       vel_pub()   const noexcept { return vel_pub_; }
  rclcpp::Publisher<ltr::msg::SportModeCmd>::SharedPtr sport_pub() const noexcept { return sport_pub_; }

private:
  // --- QoS helpers ---
  static rclcpp::QoS cmd_qos();
  static rclcpp::QoS state_qos();

  // --- publishers ---
  rclcpp::Publisher<ltr::msg::VelCmd>::SharedPtr        vel_pub_;
  rclcpp::Publisher<ltr::msg::SportModeCmd>::SharedPtr  sport_pub_;

  // --- subscribers (created lazily when a callback is registered) ---
  rclcpp::Subscription<ltr::msg::VelCmd>::SharedPtr       vel_sub_;
  rclcpp::Subscription<ltr::msg::SportModeCmd>::SharedPtr sport_sub_;

  // --- user callbacks ---
  VelStateCb  vel_state_cb_;
  SportModeCb sport_mode_cb_;
  std::mutex  cb_mtx_;

  // topic / frame names (overridable via params)
  std::string vel_topic_;
  std::string sport_topic_;
  std::string vel_state_topic_;
  std::string sport_state_topic_;
};

} // namespace ltr