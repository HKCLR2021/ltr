#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include <memory>

namespace ltr 
{

class LtrApi 
{
public:
  virtual ~LtrApi() = default;

  // ---- publishers ----
  virtual void vel_cmd(double vx, double vy, double vz, double wx, double wy, double wz) = 0;
  virtual void sport_mode_cmd(uint8_t device_id, uint8_t mode) = 0;
};

using LtrApiPtr = std::shared_ptr<LtrApi>;

} // namespace ltr