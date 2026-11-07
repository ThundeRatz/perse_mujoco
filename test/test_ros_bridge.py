#!/usr/bin/env python3
"""
Test script for verifying perse_mujoco ROS 2 integration.
Checks publication of all sensor topics, reception of actuator commands,
and physical response of the simulated robot.
"""

import time
import unittest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Float64, Bool
from vision_msgs.msg import Detection2DArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class TestPerseMujocoRosBridge(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_perse_mujoco_bridge")
        cls.received_msgs = {}

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        def make_cb(topic_name):
            return lambda msg: cls.received_msgs.update({topic_name: msg})

        clock_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        cls.subs = [
            cls.node.create_subscription(Clock, "/clock", make_cb("/clock"), clock_qos),
            cls.node.create_subscription(Imu, "/perse/sensors/imu", make_cb("/perse/sensors/imu"), qos),
            cls.node.create_subscription(Float64, "/perse/sensors/encoder_fusion/distance",
                                         make_cb("/perse/sensors/encoder_fusion/distance"), qos),
            cls.node.create_subscription(Bool, "/perse/sensors/bumper", make_cb("/perse/sensors/bumper"), qos),
            cls.node.create_subscription(Bool, "/perse/sensors/inductive_sensor",
                                         make_cb("/perse/sensors/inductive_sensor"), qos),
            cls.node.create_subscription(LaserScan, "/perse/sensors/lidar", make_cb("/perse/sensors/lidar"), qos),
            cls.node.create_subscription(Detection2DArray, "/perse/sensors/camera/recognitions",
                                         make_cb("/perse/sensors/camera/recognitions"), qos),
            cls.node.create_subscription(Odometry, "/perse/simulation/odom_gt",
                                         make_cb("/perse/simulation/odom_gt"), qos),
        ]

        cls.cmd_vel_pub = cls.node.create_publisher(Twist, "/perse/actuator/cmd_vel", qos)
        cls.led_pub = cls.node.create_publisher(Bool, "/perse/actuator/led_rod", qos)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_for(self, duration_sec):
        start = time.time()
        while time.time() - start < duration_sec:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def test_01_topics_published(self):
        """Check that all required sensor and state topics are actively publishing."""
        # Spin up to 3 seconds to gather messages
        self.spin_for(3.0)

        required_topics = [
            "/clock",
            "/perse/sensors/imu",
            "/perse/sensors/encoder_fusion/distance",
            "/perse/sensors/bumper",
            "/perse/sensors/inductive_sensor",
            "/perse/sensors/lidar",
            "/perse/sensors/camera/recognitions",
            "/perse/simulation/odom_gt",
        ]

        for topic in required_topics:
            self.assertIn(topic, self.received_msgs, f"Tópico {topic} não recebeu nenhuma mensagem!")
            msg = self.received_msgs[topic]
            self.assertIsNotNone(msg, f"Mensagem vazia em {topic}")

    def test_02_sensor_data_validity(self):
        """Verify ranges, fields, and validity of received sensor messages."""
        self.spin_for(1.0)

        # Check Clock
        clock_msg = self.received_msgs.get("/clock")
        self.assertIsNotNone(clock_msg)
        self.assertGreater(clock_msg.clock.sec + clock_msg.clock.nanosec * 1e-9, 0.0)

        # Check IMU
        imu_msg = self.received_msgs.get("/perse/sensors/imu")
        self.assertIsNotNone(imu_msg)
        self.assertEqual(imu_msg.header.frame_id, "imu_link")
        # Gravity should be roughly ~9.81 m/s^2 on Z when resting
        accel_z = imu_msg.linear_acceleration.z
        self.assertGreater(accel_z, 5.0, f"Aceleração Z inesperada: {accel_z}")

        # Check LiDAR
        lidar_msg = self.received_msgs.get("/perse/sensors/lidar")
        self.assertIsNotNone(lidar_msg)
        self.assertEqual(lidar_msg.header.frame_id, "lidar_link")
        self.assertEqual(len(lidar_msg.ranges), 360)
        self.assertAlmostEqual(lidar_msg.range_min, 0.15, places=3)
        self.assertAlmostEqual(lidar_msg.range_max, 12.0, places=2)

        # Check Camera Recognitions
        cam_msg = self.received_msgs.get("/perse/sensors/camera/recognitions")
        self.assertIsNotNone(cam_msg)
        self.assertEqual(cam_msg.header.frame_id, "camera_link")
        # There should be at least one cone visible ahead in test_scene.xml (cone_1 or cone_2)
        print(f"[TEST] Detections count: {len(cam_msg.detections)}")
        if len(cam_msg.detections) > 0:
            det = cam_msg.detections[0]
            self.assertGreater(det.bbox.size_x, 0.0)
            self.assertGreater(det.bbox.size_y, 0.0)
            print(f"[TEST] First cone bbox center: ({det.bbox.center.position.x:.1f}, {det.bbox.center.position.y:.1f}), size: ({det.bbox.size_x:.1f}, {det.bbox.size_y:.1f})")

        # Check Odometry Ground Truth
        odom_msg = self.received_msgs.get("/perse/simulation/odom_gt")
        self.assertIsNotNone(odom_msg)
        self.assertEqual(odom_msg.header.frame_id, "odom")
        self.assertEqual(odom_msg.child_frame_id, "base_link")
        print(f"[TEST] Robot initial GT pose: x={odom_msg.pose.pose.position.x:.3f}, y={odom_msg.pose.pose.position.y:.3f}, z={odom_msg.pose.pose.position.z:.3f}")

    def test_03_actuator_response(self):
        """Send cmd_vel commands and check forward motion and encoder accumulation."""
        import socket
        import json

        def send_ipc(payload):
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect("/tmp/perse_mujoco.sock")
            sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
            resp = json.loads(sock.recv(4096).decode("utf-8").strip().split("\n")[0])
            sock.close()
            return resp

        # Ensure zero command before resetting
        zero_twist = Twist()
        self.cmd_vel_pub.publish(zero_twist)

        send_ipc({"cmd": "reset"})
        send_ipc({"cmd": "resume"})
        self.received_msgs.clear()
        self.spin_for(0.5)

        initial_odom = self.received_msgs.get("/perse/simulation/odom_gt")
        initial_x = initial_odom.pose.pose.position.x if initial_odom else 0.0
        initial_enc = self.received_msgs.get("/perse/sensors/encoder_fusion/distance")
        initial_enc_dist = initial_enc.data if initial_enc else 0.0

        # Command forward motion: linear.x = 0.5 (approx 1.5 m/s)
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0

        print("[TEST] Sending forward cmd_vel linear.x = 0.5...")
        for _ in range(30):
            self.cmd_vel_pub.publish(twist)
            self.spin_for(0.05)

        # Stop robot
        stop_twist = Twist()
        self.cmd_vel_pub.publish(stop_twist)
        self.spin_for(0.5)

        final_odom = self.received_msgs.get("/perse/simulation/odom_gt")
        final_x = final_odom.pose.pose.position.x if final_odom else 0.0
        final_enc = self.received_msgs.get("/perse/sensors/encoder_fusion/distance")
        final_enc_dist = final_enc.data if final_enc else 0.0

        delta_x = final_x - initial_x
        delta_enc = final_enc_dist - initial_enc_dist

        print(f"[TEST] Motion delta_x = {delta_x:.3f} m, delta_encoder = {delta_enc:.3f} m")
        self.assertGreater(delta_x, 0.1, "O robô não se moveu para frente após comando de velocidade!")
        self.assertGreater(delta_enc, 0.05, "A distância do encoder não aumentou após movimento!")

    def test_04_speed_multiplier_scaling(self):
        """Test that simulation speed multiplier accurately scales sim_time progression against real wall time."""
        import socket
        import json

        def send_ipc(payload):
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect("/tmp/perse_mujoco.sock")
            sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
            resp = json.loads(sock.recv(4096).decode("utf-8").strip().split("\n")[0])
            sock.close()
            return resp

        # Ensure simulation is in free_run and resumed
        send_ipc({"cmd": "resume"})

        for target_speed in [0.5, 0.9, 1.5, 2.0]:
            send_ipc({"cmd": "set_speed", "speed": target_speed})
            time.sleep(0.1)  # settle

            st1 = send_ipc({"cmd": "get_status"})
            t_wall_1 = time.monotonic()
            t_sim_1 = st1["sim_time"]

            time.sleep(0.5)

            st2 = send_ipc({"cmd": "get_status"})
            t_wall_2 = time.monotonic()
            t_sim_2 = st2["sim_time"]

            dt_wall = t_wall_2 - t_wall_1
            dt_sim = t_sim_2 - t_sim_1
            measured_speed = dt_sim / dt_wall
            measured_hz = measured_speed / st2["timestep"]

            print(
                f"[SPEED TEST] Target={target_speed:.2f}x | Measured={measured_speed:.3f}x "
                f"({measured_hz:.1f} Hz) | dSim={dt_sim:.3f}s in dWall={dt_wall:.3f}s"
            )
            self.assertAlmostEqual(
                measured_speed,
                target_speed,
                delta=0.15,
                msg=f"Velocidade simulada {measured_speed:.2f}x diverge do alvo {target_speed:.2f}x!",
            )

        # Restore default 0.9x speed
        send_ipc({"cmd": "set_speed", "speed": 0.9})


if __name__ == "__main__":
    unittest.main()
