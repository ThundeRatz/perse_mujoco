#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <mujoco/mujoco.h>
#include "perse_mujoco/MujocoIpcBridge.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace perse_mujoco {

class TestSimNode : public rclcpp::Node {
public:
  TestSimNode();
  ~TestSimNode() override;

private:
  void stepSimulation();

  mjModel *m_{nullptr};
  mjData *d_{nullptr};
  std::mutex sim_mutex_;
  std::unique_ptr<MujocoIpcBridge> ipc_bridge_;

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_wall_time_;
  bool publish_clock_{true};
};

} // namespace perse_mujoco
