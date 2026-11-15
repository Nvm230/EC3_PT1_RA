#!/usr/bin/env python3
"""
===============================================================================
SIMULACIÓN Y ANIMACIÓN EN PYBULLET: SALTO SOBRE OBSTÁCULO (UNITREE A1)
===============================================================================
Carga la trayectoria óptima generada por CasADi / IPOPT, ejecuta el movimiento
en PyBullet con el obstáculo físico en 3D, dibuja las trayectorias de depuración
y genera videos MP4, animaciones GIF y capturas de alta resolución.
"""

import os
import sys
import time
import argparse
import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image, ImageDraw, ImageFont
from robot_descriptions import a1_description

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from modelo_srbm import A1SRBMModel


def get_font(size=20, bold=True):
    font_paths = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/dejavu/DejaVuSans-Bold.ttf",
        "/usr/local/share/fonts/DejaVuSans-Bold.ttf"
    ]
    for path in font_paths:
        if os.path.exists(path):
            try:
                return ImageFont.truetype(path, size)
            except Exception:
                pass
    return ImageFont.load_default()


def create_obstacle(p_client, x_min, x_max, height=0.35, width=0.80):
    """Crea una caja rígida como obstáculo en PyBullet."""
    length = x_max - x_min
    x_center = (x_min + x_max) / 2.0
    half_extents = [length / 2.0, width / 2.0, height / 2.0]
    
    col_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=half_extents)
    vis_id = p.createVisualShape(p.GEOM_BOX, halfExtents=half_extents,
                                 rgbaColor=[0.90, 0.30, 0.20, 1.0],
                                 specularColor=[0.4, 0.4, 0.4])
    
    obs_id = p.createMultiBody(
        baseMass=0,  # Estático
        baseCollisionShapeIndex=col_id,
        baseVisualShapeIndex=vis_id,
        basePosition=[x_center, 0.0, height / 2.0]
    )
    return obs_id


def run_simulation(gui=False, save_media=True):
    # Cargar datos de la optimización
    data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trayectoria_salto_opt.npy")
    if not os.path.exists(data_path):
        from opt_salto_obstaculo import solve_jump_trajectory
        data = solve_jump_trajectory()
    else:
        data = np.load(data_path, allow_pickle=True).item()

    time_arr = data["time"]
    px = data["state"]["px"]
    pz = data["state"]["pz"]
    theta = data["state"]["theta"]
    q_joints = data["q_joints"]
    obs_info = data["obstacle"]

    # Conectar a PyBullet
    conn_mode = p.GUI if gui else p.DIRECT
    cid = p.connect(conn_mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    plane_id = p.loadURDF("plane.urdf")
    
    # Crear obstáculo 3D
    obs_id = create_obstacle(cid, obs_info["x_min"], obs_info["x_max"], obs_info["h"], width=0.80)

    # Cargar Unitree A1
    robot_id = p.loadURDF(a1_description.URDF_PATH, [0, 0, 0.30], useFixedBase=False)

    # Índices de articulaciones actuadas en PyBullet:
    # FL: [7, 9, 10], FR: [2, 4, 5], RL: [17, 19, 20], RR: [12, 14, 15]
    pb_joint_indices = [7, 9, 10, 2, 4, 5, 17, 19, 20, 12, 14, 15]

    # Visualizar trayectoria de CoM en 3D
    com_points = np.column_stack([px, np.zeros_like(px), pz])
    for i in range(len(com_points) - 1):
        p.addUserDebugLine(com_points[i], com_points[i+1], lineColorRGB=[0.0, 0.9, 0.2], lineWidth=3)

    # Configuración de cámara para renderizado
    cam_target = [0.45, 0.0, 0.35]
    cam_dist = 1.65
    cam_yaw = 55.0
    cam_pitch = -18.0
    view_matrix = p.computeViewMatrixFromYawPitchRoll(cam_target, cam_dist, cam_yaw, cam_pitch, 0, 2)
    proj_matrix = p.computeProjectionMatrixFOV(fov=60.0, aspect=1024/768, nearVal=0.1, farVal=100.0)

    # Interpolación para animación fluida a 50 FPS
    fps = 50
    t_total = time_arr[-1]
    n_frames = int(t_total * fps)
    t_interp = np.linspace(0, t_total, n_frames)

    px_interp = np.interp(t_interp, time_arr, px)
    pz_interp = np.interp(t_interp, time_arr, pz)
    theta_interp = np.interp(t_interp, time_arr, theta)
    q_interp = np.zeros((n_frames, 12))
    for j in range(12):
        q_interp[:, j] = np.interp(t_interp, time_arr, q_joints[:, j])

    captured_frames = []
    keyframe_indices = [
        int(n_frames * 0.12),  # Agache (Crouch)
        int(n_frames * 0.28),  # Despegue (Takeoff)
        int(n_frames * 0.52),  # Vértice / Salto sobre obstáculo (Apex)
        int(n_frames * 0.92)   # Aterrizaje y estabilización (Landing)
    ]
    keyframe_labels = [
        "1. Fase Agache / Compresion",
        "2. Fase Impulso / Despegue",
        "3. Vertice sobre Obstaculo (Clearance)",
        "4. Aterrizaje y Estabilizacion"
    ]
    keyframe_images = []

    print(f">>> Ejecutando simulación y renderizando {n_frames} fotogramas a {fps} FPS...")
    
    for f_idx in range(n_frames):
        pos_base = [px_interp[f_idx], 0.0, pz_interp[f_idx]]
        # Orientación base: cabeceo (pitch)
        pitch = theta_interp[f_idx]
        orn_base = p.getQuaternionFromEuler([0.0, pitch, 0.0])

        p.resetBasePositionAndOrientation(robot_id, pos_base, orn_base)

        for j_pb, q_val in zip(pb_joint_indices, q_interp[f_idx]):
            p.resetJointState(robot_id, j_pb, q_val)

        if gui:
            p.stepSimulation()
            time.sleep(1.0 / fps)

        if save_media:
            img = p.getCameraImage(1024, 768, viewMatrix=view_matrix, projectionMatrix=proj_matrix,
                                   renderer=p.ER_TINY_RENDERER)
            rgba = np.reshape(img[2], (768, 1024, 4))
            pil_img = Image.fromarray(rgba[:, :, :3])

            # Anotación elegante
            draw = ImageDraw.Draw(pil_img)
            font_title = get_font(20, bold=True)
            font_badge = get_font(15, bold=True)

            # Barra superior
            draw.rectangle([(0, 0), (1024, 48)], fill=(18, 24, 38))
            draw.line([(0, 48), (1024, 48)], fill=(45, 60, 85), width=2)
            draw.text((20, 12), "Unitree A1 | Salto de Obstaculo Alto (TrajOpt CasADi/IPOPT)", fill=(255, 255, 255), font=font_title)

            # Badge
            badge_text = f"t = {t_interp[f_idx]:.2f} s | CoM Z: {pz_interp[f_idx]:.2f} m"
            draw.rectangle([(750, 8), (1004, 40)], fill=(40, 140, 210))
            draw.text((765, 13), badge_text, fill=(255, 255, 255), font=font_badge)

            captured_frames.append(pil_img)

            if f_idx in keyframe_indices:
                k_idx = keyframe_indices.index(f_idx)
                kf_copy = pil_img.copy()
                d_kf = ImageDraw.Draw(kf_copy)
                d_kf.rectangle([(20, 60), (450, 95)], fill=(30, 40, 60))
                d_kf.text((30, 67), keyframe_labels[k_idx], fill=(80, 220, 120), font=font_title)
                keyframe_images.append(kf_copy)

    p.disconnect()

    # Guardar GIF, MP4 y mosaico de capturas
    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "outputs"))
    os.makedirs(output_dir, exist_ok=True)

    if save_media and len(captured_frames) > 0:
        import imageio
        gif_path = os.path.join(output_dir, "p1_salto_obstaculo.gif")
        mp4_path = os.path.join(output_dir, "p1_salto_obstaculo.mp4")

        print(f">>> Guardando GIF animado en: {gif_path}")
        imageio.mimsave(gif_path, [np.array(img) for img in captured_frames[::2]], fps=25)

        print(f">>> Guardando video MP4 en: {mp4_path}")
        try:
            imageio.mimsave(mp4_path, [np.array(img) for img in captured_frames], fps=50)
        except Exception as e:
            print(f"Aviso MP4: {e}")

        # Crear mosaico 2x2 de keyframes
        if len(keyframe_images) == 4:
            grid_w, grid_h = 1024, 768
            mosaic = Image.new("RGB", (grid_w * 2, grid_h * 2))
            mosaic.paste(keyframe_images[0], (0, 0))
            mosaic.paste(keyframe_images[1], (grid_w, 0))
            mosaic.paste(keyframe_images[2], (0, grid_h))
            mosaic.paste(keyframe_images[3], (grid_w, grid_h))
            mosaic_path = os.path.join(output_dir, "p1_capturas_secuencia.png")
            mosaic.save(mosaic_path)
            print(f">>> Mosaico de keyframes guardado en: {mosaic_path}")

    print(">>> ¡Simulación y exportación de medios de la Parte 1 completada!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="Ejecutar con interfaz gráfica")
    args = parser.parse_args()
    run_simulation(gui=args.gui)
