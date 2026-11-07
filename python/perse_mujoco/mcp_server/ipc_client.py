"""Cliente IPC para comunicação com a simulação MuJoCo e com o Visualizador Nativo via Unix Domain Socket."""

import json
import socket
from typing import Any, Dict, List, Optional


class MujocoIpcClient:
    def __init__(
        self,
        sim_socket_path: str = "/tmp/perse_mujoco.sock",
        viewer_socket_path: str = "/tmp/perse_viewer.sock",
    ):
        self.sim_socket_path = sim_socket_path
        self.viewer_socket_path = viewer_socket_path

    def _send_command(
        self, payload: Dict[str, Any], socket_path: str, timeout: float = 3.0
    ) -> Dict[str, Any]:
        """Envia um comando em JSON terminado em quebra de linha e aguarda a resposta."""
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        try:
            sock.connect(socket_path)
            cmd_data = (json.dumps(payload) + "\n").encode("utf-8")
            sock.sendall(cmd_data)

            response_buffer = ""
            while True:
                chunk = sock.recv(4096).decode("utf-8")
                if not chunk:
                    break
                response_buffer += chunk
                if "\n" in response_buffer:
                    break

            lines = response_buffer.strip().split("\n")
            if not lines or not lines[0]:
                return {"status": "error", "message": "Resposta vazia do socket"}
            return json.loads(lines[0])
        except Exception as e:
            return {"status": "error", "message": f"Erro de comunicação IPC ({socket_path}): {str(e)}"}
        finally:
            sock.close()

    # Comandos da Simulação Física (perse_mujoco_node)
    def get_status(self) -> Dict[str, Any]:
        return self._send_command({"cmd": "get_status"}, self.sim_socket_path)

    def set_mode(self, mode: str) -> Dict[str, Any]:
        return self._send_command({"cmd": "set_mode", "mode": mode}, self.sim_socket_path)

    def pause(self) -> Dict[str, Any]:
        return self._send_command({"cmd": "pause"}, self.sim_socket_path)

    def resume(self) -> Dict[str, Any]:
        return self._send_command({"cmd": "resume"}, self.sim_socket_path)

    def step(self, steps: int = 1) -> Dict[str, Any]:
        return self._send_command({"cmd": "step", "steps": steps}, self.sim_socket_path)

    def reset(self) -> Dict[str, Any]:
        return self._send_command({"cmd": "reset"}, self.sim_socket_path)

    def get_bodies(self) -> Dict[str, Any]:
        return self._send_command({"cmd": "get_bodies"}, self.sim_socket_path)

    def get_joints(self) -> Dict[str, Any]:
        return self._send_command({"cmd": "get_joints"}, self.sim_socket_path)

    def apply_force(
        self, body: str, force: Optional[List[float]] = None, torque: Optional[List[float]] = None
    ) -> Dict[str, Any]:
        payload = {
            "cmd": "apply_force",
            "body": body,
            "force": force or [0.0, 0.0, 0.0],
            "torque": torque or [0.0, 0.0, 0.0],
        }
        return self._send_command(payload, self.sim_socket_path)

    # Comandos do Visualizador Nativo (perse_mujoco_viewer)
    def get_viewer_status(self) -> Dict[str, Any]:
        return self._send_command({"cmd": "get_viewer_status"}, self.viewer_socket_path)

    def set_camera(
        self,
        lookat: Optional[List[float]] = None,
        distance: Optional[float] = None,
        azimuth: Optional[float] = None,
        elevation: Optional[float] = None,
    ) -> Dict[str, Any]:
        payload: Dict[str, Any] = {"cmd": "set_camera"}
        if lookat is not None:
            payload["lookat"] = lookat
        if distance is not None:
            payload["distance"] = distance
        if azimuth is not None:
            payload["azimuth"] = azimuth
        if elevation is not None:
            payload["elevation"] = elevation
        return self._send_command(payload, self.viewer_socket_path)

    def focus_body(self, body: str, distance: float = 2.0) -> Dict[str, Any]:
        payload = {"cmd": "focus_body", "body": body, "distance": distance}
        return self._send_command(payload, self.viewer_socket_path)

    def set_visualization_option(self, option: str, enabled: bool = True) -> Dict[str, Any]:
        payload = {"cmd": "set_option", "option": option, "enabled": enabled}
        return self._send_command(payload, self.viewer_socket_path)

    def capture_screenshot(
        self, path: str = "/tmp/perse_screenshot.png", width: int = 1200, height: int = 800
    ) -> Dict[str, Any]:
        payload = {"cmd": "capture_screenshot", "path": path, "width": width, "height": height}
        return self._send_command(payload, self.viewer_socket_path)
