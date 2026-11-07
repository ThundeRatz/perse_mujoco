#include "perse_mujoco/SensorBridge.hpp"

#include <cmath>
#include <limits>

namespace perse_mujoco {

SensorBridge::SensorBridge(rclcpp::Node *node, const mjModel *m,
                           std::shared_ptr<SyntheticVision> synthetic_vision,
                           double wheel_radius)
    : node_(node), synthetic_vision_(synthetic_vision), wheel_radius_(wheel_radius) {
  // Lê tópicos de sensores do arquivo topics.yaml
  std::string imu_topic =
      node_->declare_parameter("topics.imu", rclcpp::PARAMETER_STRING).get<std::string>();
  std::string encoder_topic =
      node_->declare_parameter("topics.encoder", rclcpp::PARAMETER_STRING).get<std::string>();
  std::string bumper_topic =
      node_->declare_parameter("topics.bumper", rclcpp::PARAMETER_STRING).get<std::string>();
  std::string inductive_topic =
      node_->declare_parameter("topics.inductive_sensor", rclcpp::PARAMETER_STRING).get<std::string>();
  std::string lidar_topic =
      node_->declare_parameter("topics.lidar", rclcpp::PARAMETER_STRING).get<std::string>();
  std::string odom_gt_topic =
      node_->declare_parameter("topics.odom_gt", rclcpp::PARAMETER_STRING).get<std::string>();

  // Inicialização dos publicadores ROS 2
  clock_pub_ = node_->create_publisher<rosgraph_msgs::msg::Clock>(
      "/clock", rclcpp::ClockQoS());

  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS().reliable());

  encoder_pub_ = node_->create_publisher<std_msgs::msg::Float64>(
      encoder_topic, rclcpp::SensorDataQoS().reliable());

  bumper_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      bumper_topic, rclcpp::SensorDataQoS().reliable());

  inductive_pub_ = node_->create_publisher<std_msgs::msg::Bool>(
      inductive_topic, rclcpp::SensorDataQoS().reliable());

  lidar_pub_ = node_->create_publisher<sensor_msgs::msg::LaserScan>(
      lidar_topic, rclcpp::SensorDataQoS().reliable());

  odom_gt_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>(
      odom_gt_topic, rclcpp::SensorDataQoS().reliable());

  // Mapeamento de IDs no modelo MuJoCo
  imu_accel_id_ = mj_name2id(m, mjOBJ_SENSOR, "imu_accel");
  imu_gyro_id_ = mj_name2id(m, mjOBJ_SENSOR, "imu_gyro");
  imu_site_id_ = mj_name2id(m, mjOBJ_SITE, "imu_site");
  chassis_body_id_ = mj_name2id(m, mjOBJ_BODY, "chassis");

  wheel_rl_jnt_id_ = mj_name2id(m, mjOBJ_JOINT, "wheel_rl_joint");
  if (wheel_rl_jnt_id_ >= 0) {
    wheel_rl_qpos_adr_ = m->jnt_qposadr[wheel_rl_jnt_id_];
  }

  bumper_jnt_id_ = mj_name2id(m, mjOBJ_JOINT, "bumper_joint");
  if (bumper_jnt_id_ >= 0) {
    bumper_qpos_adr_ = m->jnt_qposadr[bumper_jnt_id_];
  }
  bumper_body_id_ = mj_name2id(m, mjOBJ_BODY, "bumper");

  inductive_site_id_ = mj_name2id(m, mjOBJ_SITE, "inductive_site");
  lidar_site_id_ = mj_name2id(m, mjOBJ_SITE, "lidar_site");

  // Publica valores iniciais nos tópicos latchados
  std_msgs::msg::Float64 enc_init;
  enc_init.data = 0.0;
  encoder_pub_->publish(enc_init);

  std_msgs::msg::Bool bump_init;
  bump_init.data = false;
  bumper_pub_->publish(bump_init);

  std_msgs::msg::Bool ind_init;
  ind_init.data = false;
  inductive_pub_->publish(ind_init);

  RCLCPP_INFO(node_->get_logger(), "SensorBridge inicializada com sucesso!");
}

void SensorBridge::publishClock(const mjData *d) {
  if (!clock_pub_) return;
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = rclcpp::Time(static_cast<int64_t>(d->time * 1e9), RCL_ROS_TIME);
  clock_pub_->publish(clock_msg);
}

void SensorBridge::updateAndPublish(const mjModel *m, const mjData *d, const rclcpp::Time &stamp) {
  // Publica relógio a cada invocação do step
  publishClock(d);

  // Detecta se houve reset do relógio de simulação (t voltou para 0)
  if (stamp < last_imu_time_) {
    last_imu_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_encoder_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_bumper_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_inductive_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_lidar_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
    last_odom_gt_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  }

  // 1. IMU (100 Hz)
  if ((stamp - last_imu_time_).seconds() >= 1.0 / imu_rate_hz_) {
    publishImu(m, d, stamp);
    last_imu_time_ = stamp;
  }

  // 2. Encoders (100 Hz ou sob variação)
  if ((stamp - last_encoder_time_).seconds() >= 1.0 / encoder_rate_hz_) {
    publishEncoder(m, d, stamp);
    last_encoder_time_ = stamp;
  }

  // 3. Bumper (100 Hz)
  if ((stamp - last_bumper_time_).seconds() >= 1.0 / bumper_rate_hz_) {
    publishBumper(m, d, stamp);
    last_bumper_time_ = stamp;
  }

  // 4. Sensor Indutivo (50 Hz)
  if ((stamp - last_inductive_time_).seconds() >= 1.0 / inductive_rate_hz_) {
    publishInductive(m, d, stamp);
    last_inductive_time_ = stamp;
  }

  // 5. LiDAR 2D (10 Hz)
  if ((stamp - last_lidar_time_).seconds() >= 1.0 / lidar_rate_hz_) {
    publishLidar(m, d, stamp);
    last_lidar_time_ = stamp;
  }

  // 6. Ground Truth Odometry (50 Hz)
  if ((stamp - last_odom_gt_time_).seconds() >= 1.0 / odom_gt_rate_hz_) {
    publishOdomGt(m, d, stamp);
    last_odom_gt_time_ = stamp;
  }
}

void SensorBridge::publishImu(const mjModel *m, const mjData *d, const rclcpp::Time &stamp) {
  if (!imu_pub_) return;

  sensor_msgs::msg::Imu msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "imu_link";

  // Orientação em quaternion extraída do chassis (MuJoCo: w, x, y, z)
  if (chassis_body_id_ >= 0) {
    const mjtNum *q = &d->xquat[chassis_body_id_ * 4];
    msg.orientation.w = q[0];
    msg.orientation.x = q[1];
    msg.orientation.y = q[2];
    msg.orientation.z = q[3];
  } else {
    msg.orientation.w = 1.0;
  }
  msg.orientation_covariance[0] = 1e-4;
  msg.orientation_covariance[4] = 1e-4;
  msg.orientation_covariance[8] = 1e-4;

  // Giroscópio (velocidade angular rad/s)
  if (imu_gyro_id_ >= 0) {
    int adr = m->sensor_adr[imu_gyro_id_];
    msg.angular_velocity.x = d->sensordata[adr + 0];
    msg.angular_velocity.y = d->sensordata[adr + 1];
    msg.angular_velocity.z = d->sensordata[adr + 2];
  }
  msg.angular_velocity_covariance[0] = 1e-3;
  msg.angular_velocity_covariance[4] = 1e-3;
  msg.angular_velocity_covariance[8] = 1e-3;

  // Acelerômetro (aceleração m/s^2)
  if (imu_accel_id_ >= 0) {
    int adr = m->sensor_adr[imu_accel_id_];
    msg.linear_acceleration.x = d->sensordata[adr + 0];
    msg.linear_acceleration.y = d->sensordata[adr + 1];
    msg.linear_acceleration.z = d->sensordata[adr + 2];
  }
  msg.linear_acceleration_covariance[0] = 1e-2;
  msg.linear_acceleration_covariance[4] = 1e-2;
  msg.linear_acceleration_covariance[8] = 1e-2;

  imu_pub_->publish(msg);
}

void SensorBridge::publishEncoder(const mjModel *m, const mjData *d, const rclcpp::Time &stamp) {
  (void)m;
  (void)stamp;
  if (!encoder_pub_ || wheel_rl_qpos_adr_ < 0) return;

  double wheel_angle = d->qpos[wheel_rl_qpos_adr_];
  double raw_dist = wheel_angle * wheel_radius_;
  double quantized_dist = std::round(raw_dist / encoder_resolution_) * encoder_resolution_;

  std_msgs::msg::Float64 msg;
  msg.data = quantized_dist;
  encoder_pub_->publish(msg);
  last_encoder_dist_ = quantized_dist;
}

void SensorBridge::publishBumper(const mjModel *m, const mjData *d, const rclcpp::Time &stamp) {
  (void)stamp;
  if (!bumper_pub_) return;

  bool pressed = false;

  // 1. Verificação da contração da junta linear
  if (bumper_qpos_adr_ >= 0) {
    double q = d->qpos[bumper_qpos_adr_];
    if (q < bumper_trigger_threshold_) {
      pressed = true;
    }
  }

  // 2. Verificação de contato ativo no corpo do parachoque
  if (!pressed && bumper_body_id_ >= 0) {
    for (int i = 0; i < d->ncon; ++i) {
      int b1 = m->geom_bodyid[d->contact[i].geom1];
      int b2 = m->geom_bodyid[d->contact[i].geom2];
      if ((b1 == bumper_body_id_ || b2 == bumper_body_id_) &&
          (b1 != chassis_body_id_ && b2 != chassis_body_id_)) {
        pressed = true;
        break;
      }
    }
  }

  std_msgs::msg::Bool msg;
  msg.data = pressed;
  bumper_pub_->publish(msg);
  last_bumper_state_ = pressed;
}

void SensorBridge::publishInductive(const mjModel *m, const mjData *d, const rclcpp::Time &stamp) {
  (void)m;
  (void)stamp;
  if (!inductive_pub_ || inductive_site_id_ < 0 || !synthetic_vision_) return;

  const mjtNum *site_pos = &d->site_xpos[inductive_site_id_ * 3];
  bool reading = false;
  double half_plate = inductive_plate_size_ / 2.0;

  for (const auto &cone : synthetic_vision_->getCones()) {
    double cx = cone.x;
    double cy = cone.y;
    if (cone.body_id >= 0) {
      cx = d->xpos[cone.body_id * 3 + 0];
      cy = d->xpos[cone.body_id * 3 + 1];
    }

    if (std::abs(site_pos[0] - cx) <= half_plate &&
        std::abs(site_pos[1] - cy) <= half_plate) {
      reading = true;
      break;
    }
  }

  std_msgs::msg::Bool msg;
  msg.data = reading;
  inductive_pub_->publish(msg);
  last_inductive_state_ = reading;
}

void SensorBridge::publishLidar(const mjModel *m, const mjData *d, const rclcpp::Time &stamp) {
  if (!lidar_pub_ || lidar_site_id_ < 0) return;

  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = "lidar_link";

  scan.angle_min = -M_PI;
  scan.angle_max = M_PI;
  const int num_beams = 360;
  scan.angle_increment = (2.0 * M_PI) / num_beams;
  scan.time_increment = 0.0;
  scan.scan_time = 1.0 / lidar_rate_hz_;
  scan.range_min = 0.15;
  scan.range_max = 12.0;

  scan.ranges.resize(num_beams);

  const mjtNum *lidar_pos = &d->site_xpos[lidar_site_id_ * 3];
  const mjtNum *lidar_mat = &d->site_xmat[lidar_site_id_ * 9];

  for (int i = 0; i < num_beams; ++i) {
    double angle = scan.angle_min + i * scan.angle_increment;
    double lx = std::cos(angle);
    double ly = std::sin(angle);

    // Converte vetor da direção do feixe local para o referencial de mundo
    mjtNum ray_dir[3] = {
      lidar_mat[0] * lx + lidar_mat[1] * ly,
      lidar_mat[3] * lx + lidar_mat[4] * ly,
      lidar_mat[6] * lx + lidar_mat[7] * ly
    };

    int hit_geom = -1;
    mjtNum dist = mj_ray(m, d, lidar_pos, ray_dir, nullptr, 1, chassis_body_id_, &hit_geom, nullptr);

    if (dist >= scan.range_min && dist <= scan.range_max) {
      scan.ranges[i] = static_cast<float>(dist);
    } else {
      scan.ranges[i] = std::numeric_limits<float>::infinity();
    }
  }

  lidar_pub_->publish(scan);
}

void SensorBridge::publishOdomGt(const mjModel *m, const mjData *d, const rclcpp::Time &stamp) {
  (void)m;
  if (!odom_gt_pub_) return;

  nav_msgs::msg::Odometry msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = "odom";
  msg.child_frame_id = "base_link";

  // Posicao e orientacao da raiz livre do robô
  msg.pose.pose.position.x = d->qpos[0];
  msg.pose.pose.position.y = d->qpos[1];
  msg.pose.pose.position.z = d->qpos[2];

  msg.pose.pose.orientation.w = d->qpos[3];
  msg.pose.pose.orientation.x = d->qpos[4];
  msg.pose.pose.orientation.y = d->qpos[5];
  msg.pose.pose.orientation.z = d->qpos[6];

  // Velocidades lineares e angulares no referencial do mundo
  msg.twist.twist.linear.x = d->qvel[0];
  msg.twist.twist.linear.y = d->qvel[1];
  msg.twist.twist.linear.z = d->qvel[2];

  msg.twist.twist.angular.x = d->qvel[3];
  msg.twist.twist.angular.y = d->qvel[4];
  msg.twist.twist.angular.z = d->qvel[5];

  odom_gt_pub_->publish(msg);
}

} // namespace perse_mujoco
