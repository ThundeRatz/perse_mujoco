#pragma once

#include "rerun_client.hpp"

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace perse_mujoco {

class MujocoRerun : public rclcpp::Node {
public:
  MujocoRerun();
  ~MujocoRerun() override;

  bool init();

private:
  void ipcSyncLoop();
  void logModel();
  void logData();
  void logGeomWithFrame(const std::string &entity_path, int geom_id,
                        const std::string &parent_frame);

  std::string model_path_;
  std::string sim_socket_path_;
  std::string entity_root_;
  double publish_rate_hz_{60.0};

  mjModel *m_{nullptr};
  mjData *d_{nullptr};

  std::shared_ptr<RerunClient> rerun_client_;
  std::mutex data_mutex_;
  std::atomic<bool> running_{false};
  std::thread sync_thread_;
};

} // namespace perse_mujoco
