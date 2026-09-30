#pragma once

#include <cstddef>
#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace ltr {

template <typename MsgT>
inline std::size_t get_subscription_count(
  const std::shared_ptr<rclcpp::Publisher<MsgT>>& pub) noexcept
{
  return pub ? pub->get_subscription_count() : 0u;
}


} // namespace ltr