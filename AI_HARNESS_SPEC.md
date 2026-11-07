# Arquitetura do Harness e Workflow de IA para MuJoCo (`perse_mujoco_mcp`)

Este documento detalha o design, arquitetura e especificações de ferramentas do **Harness de IA** para o simulador MuJoCo do robô Perse. O objetivo central é permitir que um agente autônomo (como o Antigravity) interaja de forma bidirecional, programática e determinística com o ambiente de simulação em tempo real ou passo-a-passo.

---

## 1. Visão Geral e Filosofia do Harness

Para que um agente de IA seja capaz de debugar controladores, calibrar dinâmica física (ex.: atrito de pneus e rigidez de suspensão), gerar testes de regressão automatizados e validar novos nós ROS 2, ele precisa de:
1. **Controle Total do Loop Temporal:** Capacidade de rodar em tempo real livre (*free run*) ou avançar deterministicamente passo a passo (*step mode*).
2. **Inspeção Multimodal:** Acesso a séries temporais numéricas (juntas, velocidades, sensores) e snapshots visuais instantâneos (câmeras do robô e câmera livre de inspeção).
3. **Perturbação Ativa do Ambiente:** Habilidade de aplicar forças externas (simular colisões, empurrões, vento), injetar comandos de atuadores e spawnar/mover cones e obstáculos.
4. **Isolamento de Processo:** A simulação roda como nó ROS 2 / executável de física dedicado, comunicando-se com o servidor MCP via **IPC leve (Unix Domain Socket / ZeroMQ)** sem afetar a estabilidade do motor de física.

```
+-------------------------------------------------------------+
|                     Agente de IA (Antigravity)               |
+-------------------------------------------------------------+
                              | JSON-RPC (MCP)
                              v
+-------------------------------------------------------------+
|           Servidor MCP Dedicado: `perse_mujoco_mcp`         |
| (Expõe ferramentas de inspeção, controle e perturbação)     |
+-------------------------------------------------------------+
                              | IPC (Unix Domain Socket / ZeroMQ)
                              v
+-------------------------------------------------------------+
|             Nó Core de Simulação MuJoCo (C++)               |
|  - Física (mjModel*, mjData*) em 500-1000 Hz                |
|  - Thread de IPC não-bloqueante (Escuta MCP)                |
|  - Render Offscreen sob demanda (EGL/OSMesa)                |
|  - Publicação ROS 2 (se conectado ao ecossistema)           |
+-------------------------------------------------------------+
```

---

## 2. Protocolo de Comunicação e Arquitetura IPC

* **Tecnologia:** Unix Domain Socket (UDS) binário ou ZeroMQ (REQ/REP ou ROUTER/DEALER) utilizando JSON estruturado.
* **Segurança do Loop de Física:**
  * O loop do `mj_step` não bloqueia aguardando o MCP no modo *Free Run*. Comandos do MCP são colocados em uma fila *lock-free* thread-safe (ring buffer) consumida no início de cada step.
  * No modo *Step Mode*, o loop de física aguarda explicitamente o comando `step` do MCP antes de avançar.
* **Latência:** < 0.2 ms por requisição IPC, permitindo centenas de chamadas por segundo.

---

## 3. Catálogo de Ferramentas MCP (`perse_mujoco_mcp`)

O servidor MCP expõe o seguinte conjunto de ferramentas atômicas e compostas:

### 3.1. Controle de Simulação e Temporalidade

| Ferramenta | Argumentos | Retorno | Descrição |
|---|---|---|---|
| `sim_get_status` | *(nenhum)* | `status: {state, sim_time, real_time_factor, step_count, mode}` | Retorna se a simulação está pausada, rodando em tempo real ou em modo passo-a-passo. |
| `sim_set_mode` | `mode: "free_run" \| "step_mode"`, `real_time_factor: float` | `success: bool` | Alterna entre execução contínua em tempo real (com fator de velocidade opcional) ou modo passo-a-passo. |
| `sim_step` | `steps: int` (default: 1), `duration_ms: float` (opcional) | `sim_time: float, completed_steps: int` | Avança deterministicamente $N$ passos de física ou $T$ milissegundos e pausa imediatamente. |
| `sim_pause` | *(nenhum)* | `sim_time: float` | Pausa imediatamente o loop físico. |
| `sim_resume` | *(nenhum)* | `sim_time: float` | Retoma a execução no modo livre. |
| `sim_reset` | `reset_pose: bool`, `seed: int` (opcional) | `success: bool, sim_time: float` | Reseta a simulação para o estado inicial ($t = 0$) ou para uma pose especificada. |

---

### 3.2. Telemetria e Leitura de Sensores

| Ferramenta | Argumentos | Retorno | Descrição |
|---|---|---|---|
| `get_robot_state` | *(nenhum)* | `pose: {x,y,z, roll,pitch,yaw}, twist: {vx,vy,vz, wx,wy,wz}, mass: float, com: [x,y,z]` | Retorna o estado rígido 6DoF global do chassi do robô (`base_link`). |
| `get_joint_states` | `joint_names: string[]` (opcional) | `{joints: [{name, pos, vel, torque, limit_low, limit_high}]}` | Retorna as leituras angulares/lineares de todas as juntas (suspensão, direção Ackermann, rodas, bumper). |
| `get_sensor_readings` | `sensors: string[]` (opcional) | `{imu: {...}, encoders: {...}, bumper: bool, inductive: bool, lidar_ranges: [...]}` | Retorna os valores exatos de todos os sensores do robô no instante atual. |
| `get_contact_points` | `body_name: string` (default: "all") | `{contacts: [{geom1, geom2, pos, normal, distance, force}]}` | Inspeciona forças e pontos de contato (essencial para checar aderência dos pneus e acionamento do bumper). |

---

### 3.3. Inspeção Visual e Câmeras (Offscreen Snapshot)

| Ferramenta | Argumentos | Retorno | Descrição |
|---|---|---|---|
| `capture_snapshot` | `camera: "robot_stereo_left" \| "robot_stereo_right" \| "free_camera"`, `width: int`, `height: int`, `look_at: [x,y,z]`, `camera_pos: [x,y,z]` | `image_path: string, bboxes: [{class, x_min, y_min, x_max, y_max, distance, occluded}], metadata: {...}` | Renderiza um frame offscreen em alta fidelidade, salva em arquivo de artefato/scratch e retorna caminho local + anotações sintéticas 2D/3D dos cones visíveis. |

---

### 3.4. Atuação e Perturbação de Física

| Ferramenta | Argumentos | Retorno | Descrição |
|---|---|---|---|
| `send_actuator_command` | `velocity_pwm: float [-1.0, 1.0]`, `steer_angle: float [rad]` | `success: bool` | Comanda diretamente o acelerador/freio e o ângulo de esterçamento do robô (equivalente ao `/perse/actuator/cmd_vel`). |
| `set_led_state` | `state: bool` | `success: bool` | Aciona ou desliga a fita de LED (`/perse/actuator/led_rod`). |
| `apply_external_force` | `body_name: string`, `force: [fx,fy,fz]`, `torque: [tx,ty,tz]`, `duration_steps: int` | `success: bool` | Aplica uma força/torque externo no robô ou objeto (ex.: simular vento lateral, solavanco ou colisão). |
| `set_robot_pose` | `position: [x,y,z]`, `orientation_yaw: float` | `success: bool` | Teleporta o robô instantaneamente para uma nova coordenada (útil para testar recuperação de falhas). |

---

### 3.5. Manipulação de Objetos e Arenas

| Ferramenta | Argumentos | Retorno | Descrição |
|---|---|---|---|
| `spawn_cone` | `x: float`, `y: float`, `has_plate: bool`, `color: "blue" \| "yellow" \| "orange"` | `cone_id: int` | Ativa um cone do pool de objetos estáticos e posiciona na coordenada informada com sua respectiva placa indutiva. |
| `clear_cones` | *(nenhum)* | `cleared_count: int` | Move todos os cones ativos de volta para o limbo ($Z = -10\,\text{m}$), desabilitando colisões. |
| `load_arena_layout` | `arena_config_path: string` | `total_cones: int` | Carrega um arquivo YAML de pista (ex.: `circle_arena.yaml`) e popula os cones instantaneamente. |

---

## 4. Casos de Uso do Harness de IA no Desenvolvimento

### Caso 1: Calibração Automatizada de Atrito e Dinâmica de Rodagem
* **Objetivo:** O agente precisa ajustar os coeficientes de atrito do MuJoCo (`friction`) e limites de torque para que o robô derrape na largada lisa, mas ganhe tração sem empinar ou quicar.
* **Workflow:**
  1. O agente define a posição inicial do robô sobre o piso liso via `set_robot_pose`.
  2. Executa `send_actuator_command(velocity_pwm=1.0, steer_angle=0.0)`.
  3. Executa `sim_step(steps=200)` (200 ms de física).
  4. Coleta `get_joint_states` (velocidade angular da roda traseira) e `get_robot_state` (velocidade linear real do chassi).
  5. Calcula a razão de escorregamento ($s = \frac{\omega R - v}{\omega R}$).
  6. Se o escorregamento estiver fora da faixa experimental real, o agente ajusta os parâmetros no XML `model/perse.xml` e repete o teste de forma autônoma.

### Caso 2: Validação Sintética do Detector de Cones (`SyntheticVision`)
* **Objetivo:** Garantir que cones atrás de outros cones ou fora do FoV não gerem falsos positivos.
* **Workflow:**
  1. O agente usa `clear_cones` e depois `spawn_cone(x=2.0, y=0.0)` e `spawn_cone(x=4.0, y=0.0)` (um atrás do outro).
  2. Chama `capture_snapshot(camera="robot_stereo_left")`.
  3. Inspeciona o retorno `bboxes`: verifica se o cone traseiro foi devidamente marcado como `occluded: true` e se apenas o cone da frente gerou bounding box visível.

### Caso 3: Teste de Colisão e Gatilho do Bumper
* **Objetivo:** Validar o disparo do tópico `/perse/sensors/bumper` em diferentes ângulos de impacto.
* **Workflow:**
  1. Spawna um obstáculo rígido a 0.5 m da frente do robô.
  2. Envia velocidade máxima contínua e avança passo-a-passo via `sim_step`.
  3. Monitora `get_sensor_readings` e a deflexão de 3 cm da junta do bumper.
  4. Confirma se o gatilho booleano foi ativado antes que o chassi colidisse com o obstáculo.

### Caso 4: Geração Aleatória de Cenários de Stress (Fuzzing Físico)
* **Objetivo:** Verificar se o controlador de trajetória consegue lidar com pistas com espaçamentos irregulares de cones.
* **Workflow:**
  1. Script Python do agente gera uma pista aleatória com curvas fechadas.
  2. Carrega no simulador via `load_arena_layout`.
  3. Dispara a missão no nó ROS do robô e observa os tópicos de odometria e status da missão via ferramentas MCP.

---

## 5. Estrutura do Servidor MCP (`perse_mujoco_mcp`)

O servidor MCP será empacotado como um módulo Python standalone com suporte a stdio (padrão MCP Anthropic/Google):

```
perse_mujoco/
└── mcp_server/
    ├── __init__.py
    ├── server.py              # Entrypoint do servidor MCP (FastMCP / stdio JSON-RPC)
    ├── ipc_client.py          # Cliente Unix Domain Socket conectado ao nó MuJoCo
    ├── tools_sim.py           # Ferramentas de tempo, pausa, step e reset
    ├── tools_sensors.py       # Ferramentas de telemetria, juntas e forças
    ├── tools_vision.py        # Ferramentas de snapshot offscreen e projeção
    └── tools_world.py         # Ferramentas de cones, arena e forças externas
```

---

## 6. Configuração no Antigravity

Para registrar o servidor MCP no ecossistema Antigravity, basta adicionar a definição em `~/.gemini/antigravity/mcp/perse_mujoco/`:

```json
{
  "name": "perse_mujoco",
  "command": "python3",
  "args": ["-m", "perse_mujoco.mcp_server.server"],
  "cwd": "/home/alan/Documents/repos/perse"
}
```

Dessa forma, o Antigravity ganha acesso nativo imediato às ferramentas de física e visualização sob demanda durante qualquer sessão de desenvolvimento.
