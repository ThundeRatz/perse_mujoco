#include "perse_mujoco/ActuatorBridge.hpp"

#include <algorithm>
#include <cmath>

namespace perse_mujoco {

ActuatorBridge::ActuatorBridge(rclcpp::Node *node, mjModel *m) : node_(node) {
  // Lê limites de velocidade, limites mecânicos e geometria de perse_properties
  max_forward_speed_ =
      node_
          ->declare_parameter("properties.limits.speed.forward",
                              rclcpp::PARAMETER_DOUBLE)
          .get<double>();
  max_reverse_speed_ =
      node_
          ->declare_parameter("properties.limits.speed.backward",
                              rclcpp::PARAMETER_DOUBLE)
          .get<double>();
  max_steer_ = node_
                   ->declare_parameter("properties.limits.steer",
                                       rclcpp::PARAMETER_DOUBLE)
                   .get<double>();
  wheel_radius_ = node_
                      ->declare_parameter("properties.wheel.wheel_radius",
                                          rclcpp::PARAMETER_DOUBLE)
                      .get<double>();

  // Mapeia os IDs dos atuadores no modelo MuJoCo
  steer_fl_act_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "steer_fl_actuator");
  steer_fr_act_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "steer_fr_actuator");
  drive_rl_act_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "drive_rl_actuator");
  drive_rr_act_id_ = mj_name2id(m, mjOBJ_ACTUATOR, "drive_rr_actuator");

  if (steer_fl_act_id_ < 0 || steer_fr_act_id_ < 0 || drive_rl_act_id_ < 0 ||
      drive_rr_act_id_ < 0) {
    RCLCPP_WARN(
        node_->get_logger(),
        "Alguns atuadores não foram encontrados pelo nome no modelo MJCF! "
        "steer_fl: %d, steer_fr: %d, drive_rl: %d, drive_rr: %d",
        steer_fl_act_id_, steer_fr_act_id_, drive_rl_act_id_, drive_rr_act_id_);
  }

  // Lê tópicos de atuadores de topics.yaml
  std::string cmd_vel_topic =
      node_->declare_parameter("topics.cmd_vel", rclcpp::PARAMETER_STRING)
          .get<std::string>();
  std::string led_topic =
      node_->declare_parameter("topics.led", rclcpp::PARAMETER_STRING)
          .get<std::string>();

  // Cria subscrições de atuadores
  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::SensorDataQoS().reliable(),
      std::bind(&ActuatorBridge::cmdVelCallback, this, std::placeholders::_1));

  led_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      led_topic, rclcpp::SensorDataQoS().reliable(),
      std::bind(&ActuatorBridge::ledCallback, this, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "ActuatorBridge inicializada com sucesso!");
}

void ActuatorBridge::cmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(control_mutex_);
  target_linear_x_ = std::clamp(msg->linear.x, -1.0, 1.0);
  target_angular_z_ =
      std::clamp(sgn(msg->linear.x) * msg->angular.z, -max_steer_, max_steer_);
}

void ActuatorBridge::reset() {
  std::lock_guard<std::mutex> lock(control_mutex_);
  target_linear_x_ = 0.0;
  target_angular_z_ = 0.0;
  led_state_ = false;
}

void ActuatorBridge::ledCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(control_mutex_);
  led_state_ = msg->data;
}

double ActuatorBridge::getLinearVelocity() const {
  std::lock_guard<std::mutex> lock(control_mutex_);
  return target_linear_x_;
}

double ActuatorBridge::getSteerAngle() const {
  std::lock_guard<std::mutex> lock(control_mutex_);
  return target_angular_z_;
}

bool ActuatorBridge::getLedState() const {
  std::lock_guard<std::mutex> lock(control_mutex_);
  return led_state_;
}

double ActuatorBridge::getWheelRadius() const { return wheel_radius_; }

void ActuatorBridge::applyControls(const mjModel *m, mjData *d) {
  (void)m;
  double cmd_linear = 0.0;
  double cmd_steer = 0.0;

  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    cmd_linear = target_linear_x_;
    cmd_steer = target_angular_z_;
  }

  // 1. Cinemática Ackermann para as rodas dianteiras
  double delta_fl = 0.0;
  double delta_fr = 0.0;

  if (std::abs(cmd_steer) < 1e-4) {
    delta_fl = 0.0;
    delta_fr = 0.0;
  } else if (cmd_steer > 0.0) {
    // Virando para a esquerda (delta > 0)
    double r_center = wheelbase_ / std::tan(cmd_steer);
    delta_fl = std::atan2(wheelbase_, r_center - (trackwidth_ / 2.0));
    delta_fr = std::atan2(wheelbase_, r_center + (trackwidth_ / 2.0));
  } else {
    // Virando para a direita (delta < 0)
    double abs_steer = -cmd_steer;
    double r_center = wheelbase_ / std::tan(abs_steer);
    delta_fl = -std::atan2(wheelbase_, r_center + (trackwidth_ / 2.0));
    delta_fr = -std::atan2(wheelbase_, r_center - (trackwidth_ / 2.0));
  }

  delta_fl = std::clamp(delta_fl, -max_steer_, max_steer_);
  delta_fr = std::clamp(delta_fr, -max_steer_, max_steer_);

  if (steer_fl_act_id_ >= 0) {
    d->ctrl[steer_fl_act_id_] = delta_fl;
  }
  if (steer_fr_act_id_ >= 0) {
    d->ctrl[steer_fr_act_id_] = delta_fr;
  }

  // 2. Tração traseira proporcional à velocidade de cruzeiro
  double target_speed_mps = 0.0;
  if (cmd_linear >= 0.0) {
    target_speed_mps = cmd_linear * max_forward_speed_;
  } else {
    target_speed_mps = cmd_linear * max_reverse_speed_;
  }

  double wheel_angular_velocity = target_speed_mps / wheel_radius_;

  if (drive_rl_act_id_ >= 0) {
    d->ctrl[drive_rl_act_id_] = wheel_angular_velocity;
  }
  if (drive_rr_act_id_ >= 0) {
    d->ctrl[drive_rr_act_id_] = wheel_angular_velocity;
  }
}

} // namespace perse_mujoco
