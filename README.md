# ltr

ROS 2 (Humble) C++ library that wraps the LTR topics behind a small,
stable API so other packages can command the robot.

The package provides:

- The LTR message definitions (`ltr/msg/VelCmd`, `ltr/msg/SportModeCmd`).
- A concrete `rclcpp::Node` (`ltr::LtrNode`) that owns the publishers and
  subscriptions.
- A narrow abstract interface (`ltr::LtrApi`) that downstream packages link
  against — no `rclcpp::Publisher` in sight.
- A factory (`ltr::make_api()`) that hands back a ready-to-spin node.

---

## Contents

- [Quick start](#quick-start)
- [Topics and QoS](#topics-and-qos)

---

## Quick start

```cpp
#include <rclcpp/rclcpp.hpp>
#include "ltr/ltr.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto api = ltr::make_api(); // shared_ptr<LtrApi>

  // fire-and-forget commands
  api->vel_cmd(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  api->sport_mode_cmd(0, 0);

  // optional feedback from the vendor
  api->on_vel_state([](const double v[6]) {
    // v[0..2] = vx, vy, vz ; v[3..5] = wx, wy, wz
  });
  api->on_sport_mode([](uint8_t mode, uint8_t sub_mode) {
    // ...
  });

  // LtrNode derives from rclcpp::Node, so it can be spun normally.
  auto node = std::dynamic_pointer_cast<rclcpp::Node>(api);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
```

---

## Topics and QoS

| Direction | Topic (default) | Type | QoS |
|---|---|---|---|
| Publish | `ltr/vel_cmd` | `ltr/msg/VelCmd` | `KeepLast(1)`, reliable, volatile, `lifespan(200 ms)` |
| Publish | `ltr/sport_mode_cmd` | `ltr/msg/SportModeCmd` | `KeepLast(1)`, reliable, volatile, `lifespan(200 ms)` |
| Subscribe | `ltr/vel_state` | `ltr/msg/VelCmd` | `KeepLast(1)`, best-effort, volatile |
| Subscribe | `ltr/sport_mode_state` | `ltr/msg/SportModeCmd` | `KeepLast(1)`, best-effort, volatile |

Parameters declared by `LtrNode`:

| Parameter | Default |
|---|---|
| `vel_cmd_topic` | `ltr/vel_cmd` |
| `sport_mode_topic` | `ltr/sport_mode_cmd` |
| `vel_state_topic` | `ltr/vel_state` |
| `sport_state_topic` | `ltr/sport_mode_state` |

> **Note on QoS matching.** ROS 2 endpoints must have compatible QoS to
> connect. If the vendor publishes best-effort, keep the command publisher
> best-effort too. If your setup uses a different `rmw` that doesn't support
> `lifespan`, remove it — the command still works, only the "drop stale
> commands" behaviour changes.

---