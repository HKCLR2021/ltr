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

  // ---- publishers (fire-and-forget) ----
  virtual void vel_cmd(double vx, double vy, double vz, double wx, double wy, double wz) = 0;
  virtual void sport_mode_cmd(uint8_t mode, uint8_t sub_mode) = 0;

  // ---- optional subscribers (set before spinning) ----
  using VelStateCb  = std::function<void(const double v[6])>;
  using SportModeCb = std::function<void(uint8_t mode, uint8_t sub_mode)>;
  virtual void on_vel_state(VelStateCb cb)    = 0;
  virtual void on_sport_mode(SportModeCb cb)  = 0;
};

using LtrApiPtr = std::shared_ptr<LtrApi>;

} // namespace ltr