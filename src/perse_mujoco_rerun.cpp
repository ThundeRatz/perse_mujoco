#include "perse_mujoco/MujocoRerun.hpp"

#include <csignal>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv) {
  signal(SIGPIPE, SIG_IGN);

  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<perse_mujoco::MujocoRerun>();
    if (node->init()) {
      rclcpp::spin(node);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("perse_mujoco_rerun"), "Exceção fatal: %s",
                 e.what());
  }
  rclcpp::shutdown();
  return 0;
}
