#pragma once
#include <rclcpp/rclcpp.hpp>
#include "ltr/ltr_api.hpp"

namespace ltr 
{

LtrApiPtr make_api(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions());

} // namespace ltr