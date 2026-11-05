# ltr

ROS 2 (Humble) C++ library that wraps the LTR topics behind a small,
stable API so other packages can command the robot.

The package provides:

- The LTR message definitions.
- A concrete `rclcpp::Node` (`ltr::LtrNode`) that owns the publishers and
  subscriptions.
- A narrow abstract interface (`ltr::LtrApi`) that downstream packages link
  against — no `rclcpp::Publisher` in sight.
- A factory (`ltr::make_api()`) that hands back a ready-to-spin node.

---

## Prerequisites

- **ROS 2 Humble** (tested on Humble; other distros likely work but are not
  supported).
- **`rmw_cyclonedds_cpp` must be the active RMW implementation.**
  The LTR endpoints speak CycloneDDS on the wire. Running `LtrNode`
  under `rmw_fastrtps_cpp` (or any other RMW) will not match the
  endpoints. See [Enabling CycloneDDS](#enabling-cyclonedds) below.
- CycloneDDS and CycloneDDS-C++ installed:

  ```bash
  sudo apt install ros-humble-rmw-cyclonedds-cpp

---
