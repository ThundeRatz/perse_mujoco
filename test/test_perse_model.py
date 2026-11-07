#!/usr/bin/env python3
"""Suite de testes físicos e dinâmicos para validação do modelo do robô Perse no MuJoCo.

Cenários avaliados:
1. Assentamento estático e balanço da suspensão (comportamento sob gravidade).
2. Esterçamento Ackermann (resposta em frequência e precisão angular dos servos).
3. Tração traseira e aceleração longitudinal (rolamento dos pneus e velocidade).
4. Colisão frontal e resposta mecânica do Bumper móvel.
"""

import sys
import unittest
import numpy as np
import mujoco


class TestPerseModel(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model_path = "/home/thor/perse_ws/install/perse_mujoco/share/perse_mujoco/model/test_scene.xml"
        cls.m = mujoco.MjModel.from_xml_path(cls.model_path)
        cls.d = mujoco.MjData(cls.m)

    def setUp(self):
        # Reset da simulação antes de cada teste
        mujoco.mj_resetData(self.m, self.d)

    def test_01_static_settling_and_suspension(self):
        """Teste 1: Verifica o assentamento estático e a distribuição de peso na suspensão."""
        # Avança 1 segundo (500 passos a dt=0.002s) sob gravidade
        for _ in range(500):
            mujoco.mj_step(self.m, self.d)

        # O chassi deve estar nivelado no solo
        chassis_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "chassis")
        chassis_z = self.d.xpos[chassis_id][2]
        
        # Como o raio da roda é ~0.05755m, com a deflexão de ~0.5cm da mola, o centro do chassi deve estar em [0.045, 0.065]m
        self.assertGreater(chassis_z, 0.045, f"Chassi muito baixo: z={chassis_z:.3f}m")
        self.assertLess(chassis_z, 0.065, f"Chassi muito alto: z={chassis_z:.3f}m")

        # Roll e Pitch do chassi devem ser próximos de zero (robô plano no chão)
        rot_mat = self.d.xmat[chassis_id].reshape(3, 3)
        z_axis = rot_mat[:, 2] # vetor Z local do chassi
        # Produto escalar com o Z global (0, 0, 1) deve ser > 0.99
        self.assertGreater(z_axis[2], 0.98, "O chassi não está horizontal no solo!")

        # Verificação da deflexão das 4 molas de suspensão
        susp_fl = self.d.joint("suspension_fl_joint").qpos[0]
        susp_fr = self.d.joint("suspension_fr_joint").qpos[0]
        susp_rl = self.d.joint("suspension_rl_joint").qpos[0]
        susp_rr = self.d.joint("suspension_rr_joint").qpos[0]

        print(f"\n[Teste 1] Altura CoM: {chassis_z:.4f}m | Suspensão FL={susp_fl*1000:.1f}mm, FR={susp_fr*1000:.1f}mm, RL={susp_rl*1000:.1f}mm, RR={susp_rr*1000:.1f}mm")
        
        # As molas devem estar dentro do curso útil (+-30mm)
        for name, val in [("FL", susp_fl), ("FR", susp_fr), ("RL", susp_rl), ("RR", susp_rr)]:
            self.assertGreater(val, -0.03, f"Batente inferior atingido na suspensão {name}")
            self.assertLess(val, 0.03, f"Batente superior atingido na suspensão {name}")

    def test_02_steering_kinematics(self):
        """Teste 2: Verifica a resposta angular dos servos de esterçamento dianteiro."""
        # Deixa o robô assentar 0.2s
        for _ in range(100):
            mujoco.mj_step(self.m, self.d)

        act_fl = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "steer_fl_actuator")
        act_fr = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "steer_fr_actuator")
        
        target_steer = 0.4 # ~23 graus para a esquerda
        self.d.ctrl[act_fl] = target_steer
        self.d.ctrl[act_fr] = target_steer

        # Simula 0.5s para os atuadores alcançarem a posição desejada
        for _ in range(250):
            mujoco.mj_step(self.m, self.d)

        steer_fl_pos = self.d.joint("steer_fl_joint").qpos[0]
        steer_fr_pos = self.d.joint("steer_fr_joint").qpos[0]

        print(f"\n[Teste 2] Esterçamento: Alvo={target_steer:.3f}rad | FL={steer_fl_pos:.3f}rad, FR={steer_fr_pos:.3f}rad")
        
        # Deve ter respondido na direção correta
        self.assertGreater(steer_fl_pos, 0.1, "Servo dianteiro esquerdo não esterçou adequadamente")
        self.assertGreater(steer_fr_pos, 0.1, "Servo dianteiro direito não esterçou adequadamente")

    def test_03_drive_and_longitudinal_acceleration(self):
        """Teste 3: Aplica tração nas rodas traseiras e valida avanço longitudinal."""
        # Assentamento
        for _ in range(100):
            mujoco.mj_step(self.m, self.d)

        initial_x = self.d.xpos[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "chassis")][0]

        act_rl = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "drive_rl_actuator")
        act_rr = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "drive_rr_actuator")

        # Aplica velocidade de 15 rad/s nas rodas traseiras (~1 m/s)
        self.d.ctrl[act_rl] = 15.0
        self.d.ctrl[act_rr] = 15.0

        for _ in range(500): # 1.0s de tração
            mujoco.mj_step(self.m, self.d)

        final_x = self.d.xpos[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "chassis")][0]
        vel_x = self.d.qvel[0] # Velocidade linear do chassis em X

        print(f"\n[Teste 3] Tração: Deslocamento={final_x - initial_x:.3f}m | Velocidade Vx={vel_x:.3f}m/s")
        
        # O carro deve ter se movido para frente
        self.assertGreater(final_x, initial_x + 0.3, "O robô não tracionou para frente conforme esperado!")
        self.assertGreater(vel_x, 0.2, "Velocidade longitudinal insuficiente sob comando de tração")

    def test_04_bumper_collision_and_spring_return(self):
        """Teste 4: Valida o curso linear do Bumper frontal sob impacto e o retorno elástico."""
        bumper_jnt = self.d.joint("bumper_joint")
        self.assertEqual(bumper_jnt.qpos[0], 0.0, "Bumper deve iniciar totalmente estendido em 0.0m")

        # Aplica força frontal direta contra o bumper no sentido -X
        bumper_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, "bumper")
        
        # Simula 50 passos com força compressiva de 100N
        for _ in range(50):
            self.d.xfrc_applied[bumper_id] = [-100.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            mujoco.mj_step(self.m, self.d)

        compressed_pos = bumper_jnt.qpos[0]
        print(f"\n[Teste 4] Bumper comprimido sob impacto: {compressed_pos*1000:.2f}mm")
        
        # O bumper deve ter recuado
        self.assertLess(compressed_pos, -0.01, "Bumper não recuou sob impacto frontal")
        self.assertGreaterEqual(compressed_pos, -0.032, "Bumper ultrapassou o limite mecânico de 3cm")

        # Remove a força e verifica retorno elástico da mola
        self.d.xfrc_applied[bumper_id] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        for _ in range(250):
            mujoco.mj_step(self.m, self.d)

        returned_pos = bumper_jnt.qpos[0]
        print(f"[Teste 4] Bumper após relaxamento elástico: {returned_pos*1000:.2f}mm")
        self.assertAlmostEqual(returned_pos, 0.0, delta=0.003, msg="Bumper não retornou à posição neutra")


if __name__ == "__main__":
    unittest.main()
