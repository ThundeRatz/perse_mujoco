#pragma once

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include "perse_mujoco/ActuatorBridge.hpp"
#include "perse_mujoco/MujocoIpcBridge.hpp"
#include "perse_mujoco/SensorBridge.hpp"
#include "perse_mujoco/SyntheticVision.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

namespace perse_mujoco {

class PerseMujocoNode : public rclcpp::Node {
public:
  PerseMujocoNode();
  ~PerseMujocoNode() override;

private:
  void stepSimulation();
  std::string resolveModelPath(const std::string &default_model_path, const std::string &generate_service_name);

  mjModel *m_{nullptr};
  mjData *d_{nullptr};
  std::mutex sim_mutex_;

  std::unique_ptr<MujocoIpcBridge> ipc_bridge_;
  std::unique_ptr<ActuatorBridge> actuator_bridge_;
  std::shared_ptr<SyntheticVision> synthetic_vision_;
  std::unique_ptr<SensorBridge> sensor_bridge_;

  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point last_wall_time_;
  std::chrono::steady_clock::time_point last_fps_time_;
  int steps_since_fps_{0};
  double time_accumulator_{0.0};

  bool publish_clock_{true};
};

} // namespace perse_mujoco
