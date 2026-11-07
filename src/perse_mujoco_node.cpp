#include "perse_mujoco/PerseMujocoNode.hpp"

#include <csignal>

int main(int argc, char **argv) {
  // Ignora SIGPIPE para evitar que quedas de sockets derrubem o processo
  signal(SIGPIPE, SIG_IGN);

  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<perse_mujoco::PerseMujocoNode>();
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("perse_mujoco"), "Exceção fatal: %s",
                 e.what());
  }
  rclcpp::shutdown();
  return 0;
}
