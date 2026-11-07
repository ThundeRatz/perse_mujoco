#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <mutex>

namespace perse_mujoco {

class ActuatorBridge {
public:
  ActuatorBridge(rclcpp::Node *node, mjModel *m);
  ~ActuatorBridge() = default;

  // Aplica os comandos de controle mais recentes nos atuadores do MuJoCo
  void applyControls(const mjModel *m, mjData *d);

  // Reinicia os alvos de atuadores para zero
  void reset();

  // Getters para estado dos comandos
  double getLinearVelocity() const;
  double getSteerAngle() const;
  bool getLedState() const;
  double getWheelRadius() const;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void ledCallback(const std_msgs::msg::Bool::SharedPtr msg);

  rclcpp::Node *node_{nullptr};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr led_sub_;

  mutable std::mutex control_mutex_;
  double target_linear_x_{0.0};
  double target_angular_z_{0.0};
  bool led_state_{false};

  // IDs dos atuadores no MuJoCo
  int steer_fl_act_id_{-1};
  int steer_fr_act_id_{-1};
  int drive_rl_act_id_{-1};
  int drive_rr_act_id_{-1};

  // Parâmetros geométricos e dinâmicos do chassi
  double wheelbase_{0.257};     // Distância entre eixos (m)
  double trackwidth_{0.226};    // Bitola entre rodas (m)
  double wheel_radius_{0.0714}; // Raio da roda (m)
  double max_steer_{0.8};       // Ângulo máximo de esterçamento (rad)
  double max_forward_speed_{
      1.2}; // Velocidade linear correspondente a PWM 1.0 (m/s)
  double max_reverse_speed_{
      0.8}; // Velocidade linear a ré correspondente a PWM -1.0 (m/s)
};

template <typename T> constexpr int sgn(T val) {
  return (T(0) < val) - (val < T(0));
}

} // namespace perse_mujoco
