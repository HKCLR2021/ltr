#include "ltr/ltr.hpp"
#include "ltr/ltr_node.hpp"

namespace ltr 
{

LtrApiPtr make_api(const rclcpp::NodeOptions& opts) 
{
  return std::make_shared<LtrNode>(opts);
}

} // namespace ltr 