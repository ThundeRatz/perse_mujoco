# Arquitetura e Especificação Técnica: Simulação MuJoCo (`perse_mujoco`)

Este documento descreve detalhadamente o projeto, requisitos, modelagem física, subsistemas de sensores/atuadores, integração ROS 2, casos de borda (*corner cases*) e plano de implementação para o novo simulador baseado em **MuJoCo** do robô **Perse**, substituindo a antiga simulação em Webots (`perse_webots`).

---

## 1. Visão Geral e Motivação

O Perse é um robô autônomo baseado na plataforma **Traxxas Stampede** (tração traseira, diferencial, esterçamento Ackermann, suspensão de curso longo e centro de massa elevado).
A simulação anterior em Webots apresentava limitações críticas:
* Dificuldade de rodar em modo *headless* desacoplado de forma leve e com alto paralelismo.
* Restrições na modelagem precisa de contato dinâmico (derrapagem de rodas em partida sobre superfícies lisas, compressão de suspensão e atrito).
* Acoplamento excessivo do visualizador à simulação, dificultando o uso simultâneo de visualizadores nativos, web e ferramentas modernas de telemetria/logging (ex.: Rerun / Foxglove).
* Gargalos para treinamento de IA/RL ou baterias de testes automatizados com múltiplos ambientes simultâneos.

O **MuJoCo (Multi-Joint dynamics with Contact)** resolve esses problemas nativamente por ser uma biblioteca de física computacional C/C++ determinística, desacoplada de renderização, com excelente tratamento de contatos e restrições cinemáticas.

---

## 2. Requisitos Arquiteturais

| ID | Requisito | Solução Técnica no MuJoCo |
|---|---|---|
| **REQ-01** | **Desacoplamento Simulação / Visualizador** | O motor de física roda em loop fechado (`mj_step`) em processo independente / nó ROS 2 sem contexto gráfico (`DISPLAY` não necessário). Renderizadores e visualizadores conectam-se como observadores do estado (`mjData`). |
| **REQ-02** | **Multi-Visualizadores (Nativo & Web)** | Suporte a: <br>1. Visualizador Nativo MuJoCo (GLFW/C++ ou Python `mujoco.viewer`).<br>2. MuJoCo Studio Web Viewer (via WebAssembly/WebSocket padrão, exportando MJCF e transmitindo `qpos`).<br>3. Telemetria/Rerun e RViz2 via tópicos ROS 2. |
| **REQ-03** | **Paridade de Sensores com o Robô Real** | Sensores integrados no MJCF e no driver C++: IMU, Odometria de Encoders, Bumper físico (3 cm), Sensor Indutivo (placas), Câmera Stereo, Bounding Boxes dos cones (Ground-Truth sintético) e LiDAR 2D. |
| **REQ-04** | **Compatibilidade Total de Tópicos ROS 2** | Manter exatamente os mesmos nomes e interfaces de tópicos usados pelo robô real e pelo `perse_webots`. |
| **REQ-05** | **Execução Simultânea (Múltiplas Simulações)** | Isolamento completo através de instâncias separadas de `mjData` (com `mjModel` compartilhado ou individual) e suporte a ROS Namespaces (`/sim_0`, `/sim_1`, ...) para RL/testes em batch. |
| **REQ-06** | **Geração Dinâmica de Pistas e Spawning** | Pool estático de corpos configurável no MJCF (reposicionamento instantâneo sem recompilação) combinado com gerador procedural em Python capaz de criar mundos MJCF dinâmicos. |
| **REQ-07** | **Arquitetura de Código Híbrida (C++ & Python)** | **C++:** Núcleo de física, nós ROS 2 de sensores/atuadores de alta frequência (100–1000 Hz) e computação determinística.<br>**Python:** Geração de mapas, pipelines de IA/RL (Gymnasium), testes e orquestração. |

---

## 3. Modelo Físico do Robô (`perse.xml` - MJCF)

### 3.1. Chassi e Distribuição de Massa
* **Massa total:** ~2.5 kg.
* **Centro de Massa (CoM):**
  * O chassi Traxxas Stampede possui altura livre do solo relativamente alta (~4–5 cm).
  * A eletrônica (Jetson/computador de bordo, bateria LiPo, sensores e carcaça superior) concentra a maior parte da massa no volume central e ligeiramente elevado.
  * No MJCF, a massa é decomposta por sub-corpos ou especificada explicitamente via `<inertial pos="x y z" mass="2.5" fullinertia="..." />` para garantir comportamento dinâmico fiel (rolagem de carroceria em curvas acentuadas e afundamento em frenagens).

### 3.2. Suspensão
* Cada uma das 4 rodas possui um braço articulado independente com suspensão tipo *double wishbone* ou braço oscilante:
  * Junta prismática ou de rotação combinada com `<joint name="suspension_..." type="slide" axis="0 0 1" range="0 0.05" stiffness="k" damping="c" springref="0" />`.
  * Os valores de rigidez ($k$) e amortecimento ($c$) devem ser calibrados para suportar o peso estático com ~30-40% de deflexão (sag) inicial.

### 3.3. Direção Ackermann
* As mangas de eixo dianteiras (`steer_left_joint`, `steer_right_joint`) possuem eixo de giro vertical:
  * Relação de Ackermann: $\tan(\delta_i) = \frac{L}{R - W/2}$, $\tan(\delta_o) = \frac{L}{R + W/2}$ (onde $L \approx 0.257\,\text{m}$ é o wheelbase).
  * Implementação no MJCF: Atuação via um atuador virtual de direção que comanda ambas as juntas de forma vinculada via `<equality>` constraint ou controle cinemático no driver C++.

### 3.4. Tração Traseira, Diferencial e Atrito
* **Diferencial:**
  * O motor elétrico DC/BLDC (Traxxas Velineon 3500 / VESC) aplica torque que é distribuído igualmente para ambas as rodas traseiras:
  $$\tau_{\text{left}} = \frac{\tau_{\text{motor}}}{2}, \quad \tau_{\text{right}} = \frac{\tau_{\text{motor}}}{2}$$
  * Em curvas, o MuJoCo permite que cada roda gire em velocidades angulares livres $\omega_L \neq \omega_R$.
* **Modelo de Atrito e Derrapagem (Wheelspin):**
  * Configuração de contato no `<geom>` dos pneus: `friction="tangential torsional rolling"`.
  * Amortecimento de contato MuJoCo (`solref` e `solimp`) aliado a limite de torque realista do atuador.
  * Em superfícies lisas ou partidas com aceleração máxima (PWM linear = 1.0), o torque excede o atrito estático $\mu_s \cdot F_N$, resultando em derrapagem momentânea das rodas traseiras antes de engrenar a tração.

### 3.5. Bumper Frontal
* Para-choque móvel com curso de aproximadamente 3 cm:
  * Junta prismática `<joint name="bumper_joint" type="slide" axis="1 0 0" range="0 0.03" stiffness="200" damping="10" />`.
  * Estado acionado: detectado quando a deformação da junta ultrapassa o limiar de gatilho (ex.: `qpos > 0.005 m`) ou via sensor de toque `<touch>`.

### 3.6. Fita de LED
* Objeto geométrico sobre a tampa com material dinâmico:
  * Geometria no MJCF com tag `<material name="led_strip" emission="0.0" rgba="1 1 1 1" />`.
  * Em tempo de execução, comandos ROS no tópico `/perse/actuator/led_rod` atualizam o canal emissivo do material no visualizador.

---

## 4. Modelagem e Implementação dos Sensores

| Sensor | Tópico ROS 2 | Tipo de Mensagem | Método de Implementação no MuJoCo |
|---|---|---|---|
| **IMU** | `/perse/sensors/imu` | `sensor_msgs/msg/Imu` | Tags nativas `<accelerometer>` e `<gyro>` acopladas ao `base_link`. Inclusão de ruído gaussiano configurável. |
| **Odometria / Encoders** | `/perse/sensors/encoder_fusion/distance` | `std_msgs/msg/Float64` | Leitura de `<jointpos>` da roda traseira com quantização discreta (`ENCODER_RESOLUTION = 0.001 m`). |
| **Bumper** | `/perse/sensors/bumper` | `std_msgs/msg/Bool` | Leitura de `<jointpos>` da junta do para-choque com histerese de contato. |
| **Sensor Indutivo** | `/perse/sensors/inductive_sensor` | `std_msgs/msg/Bool` | Verificação espacial da distância euclidiana 2D entre o sensor e as placas metálicas associadas aos cones (`PLATE_SIZE = 1.0 m`). |
| **LiDAR 2D** | `/perse/sensors/lidar` | `sensor_msgs/msg/LaserScan` | Varredura de feixes via `mj_ray()` em loop C++ paralelo (OpenMP) com filtragem de alcance e ângulo. |
| **Câmera Stereo / Imagens** | *(Nenhum tópico ROS publicado)* | — | **Sem tráfego de imagens via ROS:** Para replicar fielmente o robô real (onde o processamento visual da OAK-D ocorre onboard para evitar saturação da rede), a simulação **não** publica tópicos de imagem ROS. Opcionalmente, imagens offscreen podem ser enviadas diretamente para ferramentas de logging externo (ex.: Rerun SDK via API C++/Python) sem passar pelo DDS/ROS. |
| **Bounding Boxes dos Cones** | `/perse/sensors/camera/recognitions` | `vision_msgs/msg/Detection2DArray` | **Ground-Truth Sintético:** Projeção direta das posições 3D dos cones ativos no plano da câmera 2D (`mj_camProjection`) com descarte de cones oclusos via `mj_ray`. Publica o array idêntico ao output do `oak_camera_node` (`Detection2DArray`), com zero sobrecarga de rede ou inferência pesada. |
| **Ground Truth Odom** | `/perse/simulation/odom_gt` | `nav_msgs/msg/Odometry` | Extração direta de posição, orientação (quaternion) e velocidades lineares/angulares da raiz do robô no `mjData`. |

---

## 5. Tópicos de Atuadores e Comandos Suportados

* **Movimentação:** `/perse/actuator/cmd_vel` (`geometry_msgs/msg/Twist`)
  * `linear.x`: Comando de aceleração/velocidade normalizado (PWM $[-1.0, 1.0]$ ou velocidade cruzeiro).
  * `angular.z`: Ângulo de esterçamento das rodas dianteiras (em radianos, limitado por `max_steer`).
* **Iluminação LED:** `/perse/actuator/led_rod` (`std_msgs/msg/Bool`)
* **Debug e Visualização de Trajetórias:**
  * Subscrição aos tópicos de caminho `/path_topics` (`nav_msgs/msg/Path`) e ponto de mira `/topics.look_ahead_index` (`std_msgs/msg/Int64`).

---

## 6. Casos de Borda (*Corner Cases*) e Mitigações

Aqui estão os cenários críticos e armadilhas comuns na física de simulação e como serão mitigados:

### 6.1. Instabilidade Numérica no Contato Pneu-Solo (Vibrações e Efeito "Trampolim")
* **Problema:** Pneus rígidos contra solo rígido podem oscilar em alta frequência quando parados, gerando jitter ou fazendo o robô "tremer" ou "quicar".
* **Mitigação:** 
  * Ajuste dos parâmetros de contato do MuJoCo: `solref="0.02 1.0"` (tempo de relaxamento e amortecimento crítico) e `solimp="0.9 0.95 0.001"`.
  * Utilizar passo de integração física (`timestep`) adequado (ex.: 0.001s ou 0.002s, ou seja, 500–1000 Hz) enquanto o ROS publica a taxas menores (50–100 Hz).

### 6.2. Capotamento / Rollover Devido ao CoM Elevado
* **Problema:** O Traxxas Stampede tem CoM alto; em curvas fechadas e alta velocidade, o robô pode tombar facilmente.
* **Mitigação:** 
  * Em vez de "travar" o robô artificialmente, calibrar com precisão os batentes de curso da suspensão e a barra estabilizadora (anti-roll bar) virtual via `<equality><weld>` ou molas de torção para que a dinâmica reflita o comportamento real do robô.

### 6.3. Oclusão e Falsos Positivos nas Bounding Boxes Sintéticas
* **Problema:** Cones que estão atrás de uma parede, fora do campo de visão (FoV) ou atrás de outro cone não podem ter suas bounding boxes publicadas cegamente.
* **Mitigação:**
  * O algoritmo de projeção sintética executará:
    1. *Frustum culling:* Verifica se as coordenadas do cone estão dentro de $x_{\min}, x_{\max}, y_{\min}, y_{\max}$ e $z > z_{\text{near}}$.
    2. *Raycasting de linha de visada (`mj_ray`):* Dispara um raio da câmera ao centro do cone; se o primeiro corpo atingido não for o cone, este é descartado por oclusão.

### 6.4. Sincronização Temporal (Sim Time vs Real Time)
* **Problema:** Em testes headless acelerados (ex.: 10x real-time), os nós ROS externos podem esperar relógio real e quebrar timeouts.
* **Mitigação:**
  * O nó C++ do MuJoCo publicará o relógio `/clock` (`rosgraph_msgs/msg/Clock`) e o parâmetro `use_sim_time: true` será configurado globalmente nos launch files.

### 6.5. Reset e Spawning em Tempo Real sem Falhas de Segmentação
* **Problema:** Criar/destruir corpos no grafo de cena do MuJoCo em tempo de execução invalida ponteiros internos do `mjModel`.
* **Mitigação:**
  * Arquitetura de **Pool de Objetos**: Um número predefinido de cones/obstáculos (ex.: 100 cones) é alocado estaticamente no XML do mundo. O gerador procedural em Python apenas altera a posição $X, Y, Z$ (`mjData->qpos`) dos cones ativos e move os inativos para baixo do plano do solo ($Z = -10\,\text{m}$), desabilitando colisões.

---

## 7. Estrutura de Pacotes Proposta (`perse_mujoco`)

Seguindo o padrão do restante do repositório (`perse_bringup`, `perse_drivers`), os **launch files principais serão em formato XML (`.launch`)**. Launch files em Python (`.launch.py`) serão usados apenas como fallback caso haja necessidade estrita de lógica procedural no startup:

```
perse_mujoco/
├── CMakeLists.txt
├── package.xml
├── README.md
├── SPEC.md
├── config/
│   ├── simulation_params.yaml       # Parâmetros de física, timestep, tópicos
│   └── arenas/                      # Arenas e mapas pré-configurados
│       └── circle_arena.yaml
├── model/
│   ├── perse.xml                    # Definição principal MJCF do robô
│   ├── world.xml                    # Mundo, solo, iluminação e pool de cones
│   └── meshes/                      # STLs reaproveitados (roda, chassi, tampa, etc.)
├── include/perse_mujoco/
│   ├── PerseMujocoDriver.hpp        # Classe principal do nó de simulação
│   ├── SensorBridge.hpp             # Conversores de dados MuJoCo -> ROS 2
│   └── SyntheticVision.hpp          # Projeção de bounding boxes e oclusão
├── src/
│   ├── PerseMujocoDriver.cpp
│   ├── SensorBridge.cpp
│   ├── SyntheticVision.cpp
│   └── perse_mujoco_node.cpp        # Entrypoint do executável C++
├── python/perse_mujoco/
│   ├── __init__.py
│   ├── arena_generator.py           # Gerador procedural de pistas / cones
│   ├── gym_env.py                   # Wrapper Gymnasium para treinamento de IA
│   └── web_server.py                # Servidor WebSocket para MuJoCo Studio Web
└── launch/
    ├── sim_headless.launch          # Simulação rápida sem GUI (XML nativo)
    ├── sim_native_viewer.launch     # Simulação com janela MuJoCo GLFW (XML nativo)
    └── sim_web_viewer.launch        # Simulação com bridge web (XML nativo)
```

---

## 8. Roteiro de Implementação

1. **Etapa 1: Modelo Físico MJCF (`perse.xml`)**
   * Migrar medidas e geometrias dos arquivos STL existentes (`perse_webots/resource/meshes/`).
   * Configurar juntas da suspensão, Ackermann, rodas e bumper com massas e inércias.
2. **Etapa 2: Driver C++ Core & Pontes ROS 2**
   * Criar o nó `perse_mujoco_node` em C++ com `rclcpp`.
   * Integrar subscrição de `/perse/actuator/cmd_vel` e `/perse/actuator/led_rod`.
   * Publicar IMU, Encoders, Bumper, Sensor Indutivo e Ground Truth Odometry.
3. **Etapa 3: Sensores Complexos (LiDAR e BBoxes Sintéticas)**
   * Implementar varredura de `mj_ray` para LiDAR 2D.
   * Implementar `SyntheticVision` para geração de `vision_msgs/msg/Detection2DArray` no tópico `/perse/sensors/camera/recognitions` via projeção geométrica com descarte por oclusão.
   * Manter hook direto opcional para envio de imagens ao Rerun via API C++ sem publicar imagens no ROS.
4. **Etapa 4: Geração Procedural e Scripts Python**
   * Desenvolver `arena_generator.py` para geração dinâmica de pistas a partir de YAML ou aleatórias via pool de objetos.
5. **Etapa 5: Launch Files XML e Validação**
   * Criar os launch files XML (`.launch`) padronizados com o repositório.
   * Validar visualizador nativo GLFW e integração com MuJoCo Studio Web.
   * Calibrar atrito e aceleração para reproduzir a derrapagem característica na largada.
