#!/usr/bin/env python3
"""
===============================================================================
SIMULACIÓN Y ANIMACIÓN EN PYBULLET: BACKFLIP 360° (UNITREE A1)
===============================================================================
Carga la trayectoria óptima del salto mortal 360°, ejecuta la simulación
cinemática/dinámica en PyBullet, renderiza los fotogramas de alta resolución y
genera los archivos de animación (GIF y MP4) y el mosaico de keyframes.
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

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "part1_salto_obstaculo")))
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


def run_simulation(gui=False, save_media=True):
    data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trayectoria_backflip_opt.npy")
    if not os.path.exists(data_path):
        from opt_backflip import solve_backflip_trajectory
        data = solve_backflip_trajectory()
    else:
        data = np.load(data_path, allow_pickle=True).item()

    time_arr = data["time"]
    px = data["state"]["px"]
    pz = data["state"]["pz"]
    theta = data["state"]["theta"]
    omega = data["state"]["omega"]
    q_joints = data["q_joints"]

    conn_mode = p.GUI if gui else p.DIRECT
    cid = p.connect(conn_mode)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)

    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF(a1_description.URDF_PATH, [0, 0, 0.30], useFixedBase=False)

    # Articulaciones de Unitree A1 en PyBullet:
    # FL: [7, 9, 10], FR: [2, 4, 5], RL: [17, 19, 20], RR: [12, 14, 15]
    pb_joint_indices = [7, 9, 10, 2, 4, 5, 17, 19, 20, 12, 14, 15]

    # Visualizar arco del CoM en 3D
    com_points = np.column_stack([px, np.zeros_like(px), pz])
    for i in range(len(com_points) - 1):
        p.addUserDebugLine(com_points[i], com_points[i+1], lineColorRGB=[0.9, 0.6, 0.1], lineWidth=3)

    # Configuración de cámara para vista lateral cinematográfica del backflip
    cam_target = [0.0, 0.0, 0.40]
    cam_dist = 1.60
    cam_yaw = 80.0  # Vista lateral para apreciar claramente el giro 360 en cabeceo
    cam_pitch = -12.0
    view_matrix = p.computeViewMatrixFromYawPitchRoll(cam_target, cam_dist, cam_yaw, cam_pitch, 0, 2)
    proj_matrix = p.computeProjectionMatrixFOV(fov=60.0, aspect=1024/768, nearVal=0.1, farVal=100.0)

    # Interpolación a 50 FPS
    fps = 50
    t_total = time_arr[-1]
    n_frames = int(t_total * fps)
    t_interp = np.linspace(0, t_total, n_frames)

    px_interp = np.interp(t_interp, time_arr, px)
    pz_interp = np.interp(t_interp, time_arr, pz)
    theta_interp = np.interp(t_interp, time_arr, theta)
    omega_interp = np.interp(t_interp, time_arr, omega)

    q_interp = np.zeros((n_frames, 12))
    for j in range(12):
        q_interp[:, j] = np.interp(t_interp, time_arr, q_joints[:, j])

    captured_frames = []
    # Keyframes representativos: Agache, Despegue y cabeceo, Cumbre invertida a 180°, Aterrizaje
    keyframe_indices = [
        int(n_frames * 0.12),  # Agache
        int(n_frames * 0.30),  # Impulso angular
        int(n_frames * 0.54),  # Vértice invertido (-180°)
        int(n_frames * 0.94)   # Aterrizaje (-360°)
    ]
    keyframe_labels = [
        "1. Agache y Carga de Energia (0 deg)",
        "2. Impulso Asimetrico de Despegue (-55 deg)",
        "3. Vertice Aereo Invertido (-180 deg)",
        "4. Aterrizaje Cuadrupedo Completo (-360 deg)"
    ]
    keyframe_images = []

    print(f">>> Renderizando simulación de Backflip 360° ({n_frames} fotogramas)...")

    for f_idx in range(n_frames):
        pos_base = [px_interp[f_idx], 0.0, pz_interp[f_idx]]
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

            draw = ImageDraw.Draw(pil_img)
            font_title = get_font(20, bold=True)
            font_badge = get_font(15, bold=True)

            # Barra superior
            draw.rectangle([(0, 0), (1024, 48)], fill=(22, 18, 30))
            draw.line([(0, 48), (1024, 48)], fill=(65, 45, 80), width=2)
            draw.text((20, 12), "Unitree A1 | Salto Mortal 360 (Backflip Acrobatico)", fill=(255, 255, 255), font=font_title)

            # Badge con ángulo y altura
            pitch_deg = np.rad2deg(pitch)
            badge_text = f"t = {t_interp[f_idx]:.2f} s | Giro: {pitch_deg:.1f} deg | Z: {pz_interp[f_idx]:.2f} m"
            draw.rectangle([(710, 8), (1004, 40)], fill=(160, 60, 200))
            draw.text((720, 13), badge_text, fill=(255, 255, 255), font=font_badge)

            captured_frames.append(pil_img)

            if f_idx in keyframe_indices:
                k_idx = keyframe_indices.index(f_idx)
                kf_copy = pil_img.copy()
                d_kf = ImageDraw.Draw(kf_copy)
                d_kf.rectangle([(20, 60), (520, 95)], fill=(30, 25, 45))
                d_kf.text((30, 67), keyframe_labels[k_idx], fill=(240, 180, 80), font=font_title)
                keyframe_images.append(kf_copy)

    p.disconnect()

    output_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "outputs"))
    os.makedirs(output_dir, exist_ok=True)

    if save_media and len(captured_frames) > 0:
        import imageio
        gif_path = os.path.join(output_dir, "p2_backflip_360.gif")
        mp4_path = os.path.join(output_dir, "p2_backflip_360.mp4")

        print(f">>> Guardando GIF animado en: {gif_path}")
        imageio.mimsave(gif_path, [np.array(img) for img in captured_frames[::2]], fps=25)

        print(f">>> Guardando video MP4 en: {mp4_path}")
        try:
            imageio.mimsave(mp4_path, [np.array(img) for img in captured_frames], fps=50)
        except Exception as e:
            print(f"Aviso MP4: {e}")

        if len(keyframe_images) == 4:
            grid_w, grid_h = 1024, 768
            mosaic = Image.new("RGB", (grid_w * 2, grid_h * 2))
            mosaic.paste(keyframe_images[0], (0, 0))
            mosaic.paste(keyframe_images[1], (grid_w, 0))
            mosaic.paste(keyframe_images[2], (0, grid_h))
            mosaic.paste(keyframe_images[3], (grid_w, grid_h))
            mosaic_path = os.path.join(output_dir, "p2_capturas_secuencia.png")
            mosaic.save(mosaic_path)
            print(f">>> Mosaico de keyframes del Backflip guardado en: {mosaic_path}")

    print(">>> ¡Simulación y exportación de medios de la Parte 2 completada!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true", help="Ejecutar con interfaz gráfica")
    args = parser.parse_args()
    run_simulation(gui=args.gui)
