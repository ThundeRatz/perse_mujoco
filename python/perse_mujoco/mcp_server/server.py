#!/usr/bin/env python3
"""Servidor MCP oficial usando o SDK oficial 'mcp' (MCPServer) para o harness de simulação e visualização MuJoCo."""

import os
import sys
from typing import Any, Dict, List, Optional

# Garante acesso ao módulo perse_mujoco
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..")))
from perse_mujoco.mcp_server.ipc_client import MujocoIpcClient

try:
    from mcp.server.mcpserver import MCPServer
except ImportError:
    from mcp.server.fastmcp import FastMCP as MCPServer

app = MCPServer("perse_mujoco_mcp")
client = MujocoIpcClient()


# ------------------------------------------------------------------------------
# Ferramentas de Controle da Simulação Física
# ------------------------------------------------------------------------------
@app.tool()
def sim_get_status() -> Dict[str, Any]:
    """Retorna o status da simulação física MuJoCo (modo, tempo simulado, se pausada, contagem de corpos e juntas)."""
    return client.get_status()


@app.tool()
def sim_step(steps: int = 1) -> Dict[str, Any]:
    """Avança N passos físicos na simulação MuJoCo e pausa o tempo (passo-a-passo determinístico)."""
    return client.step(steps=steps)


@app.tool()
def sim_pause() -> Dict[str, Any]:
    """Pausa imediatamente a execução da simulação."""
    return client.pause()


@app.tool()
def sim_resume() -> Dict[str, Any]:
    """Retoma a execução livre (Free Run) da simulação em tempo real."""
    return client.resume()


@app.tool()
def sim_set_speed(speed: float = 0.9) -> Dict[str, Any]:
    """Ajusta o multiplicador de velocidade da simulação física (ex.: 0.5 para metade da velocidade, 2.0 para o dobro)."""
    return client.set_speed(speed=speed)


@app.tool()
def sim_reset() -> Dict[str, Any]:
    """Reseta a simulação física para o estado inicial (t = 0)."""
    return client.reset()


@app.tool()
def sim_get_bodies() -> Dict[str, Any]:
    """Obtém as posições e orientações 3D (quaternions) de todos os corpos na simulação."""
    return client.get_bodies()


@app.tool()
def sim_get_joints() -> Dict[str, Any]:
    """Obtém posições (qpos) e velocidades (qvel) de todas as juntas da simulação."""
    return client.get_joints()


@app.tool()
def sim_apply_force(
    body: str,
    force: Optional[List[float]] = None,
    torque: Optional[List[float]] = None,
) -> Dict[str, Any]:
    """Aplica uma força 3D [Fx, Fy, Fz] e/ou torque 3D [Tx, Ty, Tz] sobre um corpo da simulação."""
    return client.apply_force(body=body, force=force, torque=torque)


# ------------------------------------------------------------------------------
# Ferramentas de Controle do Visualizador Nativo
# ------------------------------------------------------------------------------
@app.tool()
def viewer_get_status() -> Dict[str, Any]:
    """Retorna o status do visualizador nativo (se está conectado à simulação, parâmetros da câmera atual e tempo exibido)."""
    return client.get_viewer_status()


@app.tool()
def viewer_set_camera(
    lookat: Optional[List[float]] = None,
    distance: Optional[float] = None,
    azimuth: Optional[float] = None,
    elevation: Optional[float] = None,
) -> Dict[str, Any]:
    """Controla a posição e orientação da câmera no visualizador nativo (ponto de foco lookat [x,y,z], distância, azimute e elevação em graus)."""
    return client.set_camera(
        lookat=lookat, distance=distance, azimuth=azimuth, elevation=elevation
    )


@app.tool()
def viewer_focus_body(body: str, distance: float = 2.0) -> Dict[str, Any]:
    """Centraliza e foca a câmera do visualizador diretamente sobre um corpo especificado pelo nome."""
    return client.focus_body(body=body, distance=distance)


@app.tool()
def viewer_set_option(option: str, enabled: bool = True) -> Dict[str, Any]:
    """Ativa ou desativa opções visuais de depuração no visualizador (opções suportadas: 'contacts', 'joints', 'com', 'inertia', 'wireframe', 'shadow', 'transparent')."""
    return client.set_visualization_option(option=option, enabled=enabled)


@app.tool()
def viewer_capture_screenshot(
    path: str = "/tmp/perse_screenshot.png",
    width: int = 1200,
    height: int = 800,
) -> Dict[str, Any]:
    """Captura um screenshot em alta fidelidade diretamente da viewport do visualizador nativo no instante atual e salva na imagem indicada."""
    return client.capture_screenshot(path=path, width=width, height=height)


if __name__ == "__main__":
    app.run()
