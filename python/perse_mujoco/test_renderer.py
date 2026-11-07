#!/usr/bin/env python3
"""Script para renderizar a cena de teste do MuJoCo e salvar imagens/vídeos de teste."""

import argparse
import os
import sys
import mujoco

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False


def render_scene(model_path: str, output_image_path: str, width: int = 640, height: int = 480, steps: int = 250):
    """Carrega o modelo, simula N passos e renderiza a imagem final."""
    if not os.path.exists(model_path):
        raise FileNotFoundError(f"Arquivo de modelo não encontrado: {model_path}")

    print(f"Carregando modelo MuJoCo: {model_path}")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    print(f"Modelo carregado. nq={model.nq}, nv={model.nv}. Simulando {steps} passos...")
    for i in range(steps):
        mujoco.mj_step(model, data)

    print(f"Simulação concluída. Tempo simulado: {data.time:.3f} s. Posição da caixa Z: {data.qpos[2]:.3f} m")

    print(f"Inicializando renderizador offscreen ({width}x{height})...")
    renderer = mujoco.Renderer(model, height=height, width=width)
    renderer.update_scene(data)
    rgb_array = renderer.render()

    os.makedirs(os.path.dirname(os.path.abspath(output_image_path)), exist_ok=True)
    if HAS_PIL:
        img = Image.fromarray(rgb_array)
        img.save(output_image_path)
        print(f"Imagem renderizada com sucesso e salva em: {output_image_path}")
    else:
        # Fallback simples em PPM caso PIL não esteja instalado
        with open(output_image_path.replace('.png', '.ppm'), 'wb') as f:
            f.write(bytearray(f"P6\n{width} {height}\n255\n", 'ascii'))
            f.write(rgb_array.tobytes())
        print(f"Imagem PPM salva em: {output_image_path.replace('.png', '.ppm')}")

    renderer.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Renderiza cena de teste do MuJoCo")
    parser.add_argument("--model", type=str, default="src/perse_mujoco/model/test_scene.xml", help="Caminho do modelo MJCF")
    parser.add_argument("--output", type=str, default="src/perse_mujoco/test_render.png", help="Caminho de saída da imagem")
    parser.add_argument("--steps", type=int, default=250, help="Passos de física antes de renderizar")
    args = parser.parse_args()

    render_scene(args.model, args.output, steps=args.steps)
