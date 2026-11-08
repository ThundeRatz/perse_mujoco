#!/usr/bin/env python3
"""
Node responsável por gerar mapas XML dinamicamente para o simulador MuJoCo.
Responde ao serviço ROS 2 'services.generate_map' (std_srvs/srv/Trigger).
"""

import os
import yaml
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


class MapGeneratorNode(Node):
    def __init__(self):
        super().__init__("map_generator_node")

        # Declara parâmetros de tópicos e caminhos conforme AGENTS.md
        srv_name_param = self.declare_parameter(
            "services.generate_map", "/perse/simulation/generate_map"
        ).get_parameter_value().string_value

        self.mission_points_path = self.declare_parameter(
            "mission_points", ""
        ).get_parameter_value().string_value

        if not self.mission_points_path:
            try:
                bringup_share = get_package_share_directory("perse_bringup")
                self.mission_points_path = os.path.join(
                    bringup_share, "config", "mission_points.yaml"
                )
            except Exception as e:
                self.get_logger().warn(f"Não foi possível localizar perse_bringup: {e}")

        # Caminho fixo do arquivo gerado dentro de share/perse_mujoco/model/generated_scene.xml
        try:
            mujoco_share = get_package_share_directory("perse_mujoco")
            self.output_xml_path = os.path.join(
                mujoco_share, "model", "generated_scene.xml"
            )
        except Exception:
            self.output_xml_path = "/tmp/generated_scene.xml"

        # Cria o serviço Trigger para atender aos pedidos de geração do simulador
        self.srv = self.create_service(Trigger, srv_name_param, self.generate_map_callback)
        self.get_logger().info(
            f"MapGeneratorNode inicializado. Aguardando requisições em '{srv_name_param}'..."
        )

    def generate_map_callback(self, request, response):
        self.get_logger().info("Recebida requisição para gerar mapa da simulação...")
        try:
            success, msg = self.build_scene_xml()
            response.success = success
            response.message = msg
            if success:
                self.get_logger().info(f"Mapa gerado com sucesso em: {self.output_xml_path}")
            else:
                self.get_logger().error(f"Falha ao gerar mapa: {msg}")
        except Exception as e:
            response.success = False
            response.message = f"Exceção ao gerar mapa: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def build_scene_xml(self):
        pts = []
        if os.path.exists(self.mission_points_path):
            with open(self.mission_points_path, "r") as f:
                cfg = yaml.safe_load(f)
                pts = cfg.get("points", [])
        else:
            self.get_logger().warn(
                f"Arquivo mission_points não encontrado em '{self.mission_points_path}'. Gerando sem elementos de missão."
            )

        xml_lines = [
            '<?xml version="1.0" ?>',
            '<mujoco model="perse_generated_scene">',
            '  <compiler angle="radian" meshdir="meshes/"/>',
            '  <option gravity="0 0 -9.81" integrator="implicitfast" timestep="0.002"/>',
            '  <visual>',
            '    <headlight ambient="0.25 0.25 0.25" diffuse="0.6 0.6 0.6" specular="0.1 0.1 0.1"/>',
            '    <rgba haze="0.15 0.25 0.35 1"/>',
            '    <global azimuth="140" elevation="-25" ipd="0" offheight="960" offwidth="1280"/>',
            '  </visual>',
            '',
            '  <!-- Robô Perse oficial (autocontido e parametrizado) -->',
            '  <include file="perse_robot.xml"/>',
            '',
            '  <asset>',
            '    <texture builtin="gradient" height="512" rgb1="0.3 0.5 0.7" rgb2="0.05 0.05 0.1" type="skybox" width="512"/>',
            '    <texture builtin="checker" height="512" name="grid" rgb1="0.12 0.15 0.18" rgb2="0.18 0.22 0.26" type="2d" width="512"/>',
            '    <material name="grid_mat" reflectance="0.1" texrepeat="20 20" texture="grid"/>',
            '    <material name="box_mat" reflectance="0.4" rgba="0.2 0.6 1.0 1.0"/>',
            '    <material name="cone_orange_mat" rgba="0.95 0.40 0.05 1.0" roughness="0.5" specular="0.3"/>',
            '    <material name="cone_white_stripe_mat" rgba="0.95 0.95 0.95 1.0" roughness="0.4" specular="0.6"/>',
            '    <material name="plate_mat" rgba="0.92 0.82 0.22 1.0" roughness="0.6" specular="0.2"/>',
            '    <mesh file="cone_body.obj" name="cone_body_mesh" scale="0.35 0.35 0.35"/>',
            '    <mesh file="cone_stripes.obj" name="cone_stripes_mesh" scale="0.35 0.35 0.35"/>',
            '  </asset>',
            '',
            '  <worldbody>',
            '    <light castshadow="true" diffuse="0.8 0.8 0.8" dir="0 0 -1" directional="true" pos="0 0 5"/>',
            '    <light castshadow="true" diffuse="0.4 0.4 0.4" dir="-0.5 0.5 -1" directional="false" pos="3 -3 4"/>',
            '    <geom friction="1.0 0.05 0.001" material="grid_mat" name="ground" size="30 30 0.1" type="plane"/>',
            '',
        ]

        # Gera chapas e cones correspondentes para cada ponto de missão
        for idx, pt in enumerate(pts):
            x = pt["x"]
            y = pt["y"]
            has_cone = pt.get("has_cone", False)

            # Instancia chapa metálica de 1x1m
            xml_lines.append(self._generate_metal_plate(f"mission_{idx}", x, y))

            # Se has_cone for True, instancia o cone com listras brancas
            if has_cone:
                xml_lines.append(self._generate_traffic_cone(f"mission_{idx}", x, y))

        xml_lines.append("  </worldbody>")
        xml_lines.append("</mujoco>\n")

        os.makedirs(os.path.dirname(self.output_xml_path), exist_ok=True)
        with open(self.output_xml_path, "w") as f:
            f.write("\n".join(xml_lines))

        return True, self.output_xml_path

    def _generate_metal_plate(self, id_str, x, y):
        return f"""    <body name="plate_{id_str}" pos="{x:.6f} {y:.6f} 0.005">
      <geom condim="3" friction="1.0 0.05 0.001" material="plate_mat" name="plate_geom_{id_str}" size="0.5 0.5 0.005" type="box"/>
    </body>"""

    def _generate_traffic_cone(self, id_str, x, y):
        return f"""    <body name="cone_{id_str}" pos="{x:.6f} {y:.6f} 0.01">
      <freejoint name="cone_joint_{id_str}"/>
      <geom conaffinity="0" contype="0" group="2" material="cone_orange_mat" mesh="cone_body_mesh" name="cone_body_vis_{id_str}" type="mesh"/>
      <geom conaffinity="0" contype="0" group="2" material="cone_white_stripe_mat" mesh="cone_stripes_mesh" name="cone_stripes_vis_{id_str}" type="mesh"/>
      <geom condim="3" friction="0.9 0.05 0.001" group="3" mass="0.4" name="cone_col_base_{id_str}" pos="0 0 0.015" rgba="0 0 0 0" size="0.17 0.17 0.015" type="box"/>
      <geom condim="3" friction="0.8 0.05 0.001" group="3" mass="0.6" name="cone_col_stem_{id_str}" pos="0 0 0.24" rgba="0 0 0 0" size="0.08 0.22" type="cylinder"/>
    </body>"""


def main(args=None):
    rclpy.init(args=args)
    node = MapGeneratorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
