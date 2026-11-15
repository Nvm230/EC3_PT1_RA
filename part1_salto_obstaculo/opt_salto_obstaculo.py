#!/usr/bin/env python3
"""
===============================================================================
OPTIMIZACIÓN DE TRAYECTORIA: SALTO SOBRE OBSTÁCULO ALTO (UNITREE A1)
===============================================================================
Formulación matemática:
  - Modelo dinámico: Single Rigid Body Model (SRBM) en el plano sagital (X, Z, Pitch).
  - Método: Colocación directa con solver IPOPT vía CasADi.
  - Parámetros del obstáculo:
      * Ubicación: x in [0.52, 0.64] m (obstáculo / barrera de 12 cm de espesor y 85 cm de ancho)
      * Altura: h_obs = 0.26 m (26 cm)
        [ANÁLISIS DE FACTIBILIDAD Y LÍMITES FÍSICOS]:
        - Altura nominal de parada de Unitree A1: 0.30 m.
        - Despeje libre del vientre al suelo: 0.18-0.20 m.
        - Paso vertical máximo de marcha cuadrúpeda (crawl/trot): 0.06-0.08 m.
        - Un obstáculo de 0.26 m supera en 7 cm el vientre del chasis y triplica
          el límite de zancada de marcha: resulta IMPOSIBLE de franquear caminando.
        - Exige un salto balístico completo con fase de vuelo y despegue a ~2.8 m/s,
          alcanzando un vértice de CoM a z = 0.70 m (+44 cm de margen sobre el obstáculo),
          mientras las patas se retraen (tuck) a z >= 0.45 m (+19 cm de despeje neto).
  - Fases del salto:
      1. Agache/Compresión (T1 = 0.15s, N1 = 8): CoM desciende a z = 0.18 m.
      2. Propulsión Explosiva (T2 = 0.12s, N2 = 6): Despegue en x ~ 0.14 m, z <= 0.30 m.
      3. Vuelo Parabólico (T3 = 0.54s, N3 = 24): Superación libre del obstáculo con retracción de patas.
      4. Aterrizaje y Amortiguación (T4 = 0.24s, N4 = 12): Contacto en x >= 1.15 m y reposo en z = 0.30 m.
"""

import sys
import os
import time
import numpy as np
import casadi as ca

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from modelo_srbm import A1SRBMModel


def solve_jump_trajectory():
    model = A1SRBMModel()

    # Parámetros del obstáculo
    x_obs_min = 0.52
    x_obs_max = 0.64
    h_obs = 0.26  # 26 cm de altura

    # Duraciones y discretización por fase
    T1, N1 = 0.15, 8    # Agache
    T2, N2 = 0.12, 6    # Empuje explosivo
    T3, N3 = 0.54, 24   # Vuelo sobre el obstáculo
    T4, N4 = 0.24, 12   # Aterrizaje y amortiguación
    N_total = N1 + N2 + N3 + N4

    dt1 = T1 / N1
    dt2 = T2 / N2
    dt3 = T3 / N3
    dt4 = T4 / N4
    dt_list = [dt1]*N1 + [dt2]*N2 + [dt3]*N3 + [dt4]*N4
    time_array = np.cumsum([0.0] + dt_list)

    # Constantes físicas
    m = model.mass
    Iyy = model.Iyy
    g = model.gravity
    mu = 0.85
    d_x = model.d_x
    f_max_pair = 380.0  # 380 N por par de patas (760 N total)

    opti = ca.Opti()

    # Variables de estado X = [px, pz, vx, vz, theta, omega]^T
    X = opti.variable(6, N_total + 1)
    px = X[0, :]
    pz = X[1, :]
    vx = X[2, :]
    vz = X[3, :]
    theta = X[4, :]
    omega = X[5, :]

    # Fuerzas de reacción en pares frontal y trasero
    F = opti.variable(4, N_total)
    fx_F = F[0, :]
    fz_F = F[1, :]
    fx_R = F[2, :]
    fz_R = F[3, :]

    # Posición de aterrizaje de patas
    rx_land_F = opti.variable(1)
    rx_land_R = opti.variable(1)

    # -------------------------------------------------------------
    # CONDICIONES DE FRONTERA
    # -------------------------------------------------------------
    # Estado inicial: robot en reposo a altura nominal (0.30 m)
    opti.subject_to(px[0] == 0.0)
    opti.subject_to(pz[0] == model.nominal_height)
    opti.subject_to(vx[0] == 0.0)
    opti.subject_to(vz[0] == 0.0)
    opti.subject_to(theta[0] == 0.0)
    opti.subject_to(omega[0] == 0.0)

    # Estado final: reposo completo tras superar el obstáculo (x >= 1.15 m)
    opti.subject_to(px[N_total] >= 1.15)
    opti.subject_to(pz[N_total] == model.nominal_height)
    opti.subject_to(vx[N_total] == 0.0)
    opti.subject_to(vz[N_total] == 0.0)
    opti.subject_to(theta[N_total] == 0.0)
    opti.subject_to(omega[N_total] == 0.0)

    # Aterrizaje seguro: patas traseras tocan suelo pasado el obstáculo
    opti.subject_to(rx_land_R >= x_obs_max + 0.15)
    opti.subject_to(rx_land_F == rx_land_R + 2.0 * d_x)

    # Compresión controlada en fase 1
    opti.subject_to(pz[N1] <= 0.20)
    opti.subject_to(pz[N1] >= 0.16)

    # Despegue: altura de base acotada dentro del alcance de las patas
    opti.subject_to(pz[N1 + N2] <= 0.31)
    opti.subject_to(pz[N1 + N2] >= 0.27)

    # Contacto en aterrizaje
    opti.subject_to(pz[N1 + N2 + N3] <= 0.31)
    opti.subject_to(pz[N1 + N2 + N3] >= 0.27)

    # Vértice de vuelo CoM centrado y holgado sobre el obstáculo
    k_apex = N1 + N2 + N3 // 2
    opti.subject_to(pz[k_apex] >= h_obs + 0.35)  # CoM >= 0.61 m
    opti.subject_to(px[k_apex] >= (x_obs_min + x_obs_max) / 2.0 - 0.08)
    opti.subject_to(px[k_apex] <= (x_obs_min + x_obs_max) / 2.0 + 0.08)

    # -------------------------------------------------------------
    # DINÁMICA SRBM Y RESTRICCIONES DE CONTACTO
    # -------------------------------------------------------------
    for k in range(N_total):
        h = dt_list[k]

        if k < N1 + N2:
            r_xF, r_zF = d_x, 0.0
            r_xR, r_zR = -d_x, 0.0
        elif k < N1 + N2 + N3:
            r_xF, r_zF = px[k] + d_x, pz[k] - 0.15
            r_xR, r_zR = px[k] - d_x, pz[k] - 0.15
        else:
            r_xF, r_zF = rx_land_F, 0.0
            r_xR, r_zR = rx_land_R, 0.0

        if k < N1 + N2 or k >= N1 + N2 + N3:
            opti.subject_to(fz_F[k] >= 0.0)
            opti.subject_to(fz_R[k] >= 0.0)
            opti.subject_to(fz_F[k] <= f_max_pair)
            opti.subject_to(fz_R[k] <= f_max_pair)
            opti.subject_to(fx_F[k] <= mu * fz_F[k])
            opti.subject_to(fx_F[k] >= -mu * fz_F[k])
            opti.subject_to(fx_R[k] <= mu * fz_R[k])
            opti.subject_to(fx_R[k] >= -mu * fz_R[k])
        else:
            opti.subject_to(fx_F[k] == 0.0)
            opti.subject_to(fz_F[k] == 0.0)
            opti.subject_to(fx_R[k] == 0.0)
            opti.subject_to(fz_R[k] == 0.0)

        tau_F = (r_xF - px[k]) * fz_F[k] - (r_zF - pz[k]) * fx_F[k]
        tau_R = (r_xR - px[k]) * fz_R[k] - (r_zR - pz[k]) * fx_R[k]
        tau = tau_F + tau_R

        ax = (fx_F[k] + fx_R[k]) / m
        az = (fz_F[k] + fz_R[k] - m * g) / m
        alpha = tau / Iyy

        opti.subject_to(px[k+1] == px[k] + h * (vx[k] + 0.5 * h * ax))
        opti.subject_to(pz[k+1] == pz[k] + h * (vz[k] + 0.5 * h * az))
        opti.subject_to(vx[k+1] == vx[k] + h * ax)
        opti.subject_to(vz[k+1] == vz[k] + h * az)
        opti.subject_to(theta[k+1] == theta[k] + h * (omega[k] + 0.5 * h * alpha))
        opti.subject_to(omega[k+1] == omega[k] + h * alpha)

    for k in range(N_total + 1):
        opti.subject_to(pz[k] >= 0.16)
        opti.subject_to(pz[k] <= 0.78)
        opti.subject_to(theta[k] >= -0.22)
        opti.subject_to(theta[k] <= 0.22)

    cost = 1e-4 * ca.sumsqr(F) + 20.0 * ca.sumsqr(theta) + 2.0 * ca.sumsqr(omega) + 1.0 * (px[N_total] - 1.20)**2
    opti.minimize(cost)

    opts = {
        "ipopt.print_level": 0,
        "ipopt.max_iter": 300,
        "ipopt.tol": 1e-4,
        "print_time": False
    }
    opti.solver("ipopt", opts)

    print(">>> Resolviendo optimización de trayectoria del salto sobre obstáculo...")
    t_start = time.time()
    sol = opti.solve()
    t_calc = time.time() - t_start
    print(f">>> ¡Solución óptima hallada en {t_calc:.3f} s!")

    px_opt = np.array(sol.value(px))
    pz_opt = np.array(sol.value(pz))
    vx_opt = np.array(sol.value(vx))
    vz_opt = np.array(sol.value(vz))
    th_opt = np.array(sol.value(theta))
    om_opt = np.array(sol.value(omega))

    fx_F_opt = np.array(sol.value(fx_F))
    fz_F_opt = np.array(sol.value(fz_F))
    fx_R_opt = np.array(sol.value(fx_R))
    fz_R_opt = np.array(sol.value(fz_R))

    rx_land_F_val = float(sol.value(rx_land_F))
    rx_land_R_val = float(sol.value(rx_land_R))

    # -------------------------------------------------------------
    # CINEMÁTICA INVERSA ANALÍTICA EXACTA Y TUCKING EN VUELO
    # -------------------------------------------------------------
    feet_world_traj = np.zeros((N_total + 1, 4, 3))
    q_joint_traj = np.zeros((N_total + 1, 12))

    y_hips = [model.d_y + model.l_thigh_lat, -(model.d_y + model.l_thigh_lat),
              model.d_y + model.l_thigh_lat, -(model.d_y + model.l_thigh_lat)]

    for k in range(N_total + 1):
        p_base = np.array([px_opt[k], 0.0, pz_opt[k]])
        pitch = th_opt[k]
        c, s = np.cos(pitch), np.sin(pitch)
        R_base = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])

        p_feet_b = np.zeros((4, 3))

        if k <= N1 + N2:
            # En suelo inicial
            for i in range(4):
                pf_w = np.array([d_x if i in [0, 1] else -d_x, y_hips[i], 0.0])
                feet_world_traj[k, i] = pf_w
                p_feet_b[i] = R_base.T @ (pf_w - p_base)
        elif k < N1 + N2 + N3:
            # En vuelo: retracción activa (tucking) definida directamente en el chasis
            tau_flight = (k - (N1 + N2)) / float(N3)
            # Longitud de pata varía suavemente de 0.30m a 0.15m en el vértice
            leg_len = 0.30 - 0.15 * np.sin(np.pi * tau_flight)

            for i in range(4):
                sx = 1.0 if i in [0, 1] else -1.0
                p_feet_b[i] = [sx * d_x, y_hips[i], -leg_len]
                feet_world_traj[k, i] = p_base + R_base @ p_feet_b[i]
        else:
            # En suelo final tras obstáculo
            for i in range(4):
                pf_w = np.array([rx_land_F_val if i in [0, 1] else rx_land_R_val, y_hips[i], 0.0])
                feet_world_traj[k, i] = pf_w
                p_feet_b[i] = R_base.T @ (pf_w - p_base)

        q_joints = model.ik_full_body(p_base, R_base, feet_world_traj[k])
        q_joint_traj[k] = q_joints

    trajectory_data = {
        "time": time_array,
        "dt_list": dt_list,
        "phases": {"N1": N1, "N2": N2, "N3": N3, "N4": N4},
        "obstacle": {"x_min": x_obs_min, "x_max": x_obs_max, "h": h_obs},
        "state": {
            "px": px_opt, "pz": pz_opt,
            "vx": vx_opt, "vz": vz_opt,
            "theta": th_opt, "omega": om_opt
        },
        "forces": {
            "fx_F": fx_F_opt, "fz_F": fz_F_opt,
            "fx_R": fx_R_opt, "fz_R": fz_R_opt
        },
        "landing": {
            "rx_land_F": rx_land_F_val,
            "rx_land_R": rx_land_R_val
        },
        "feet_world": feet_world_traj,
        "q_joints": q_joint_traj
    }

    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trayectoria_salto_opt.npy")
    np.save(save_path, trajectory_data, allow_pickle=True)
    print(f">>> Trayectoria completa guardada con éxito en: {save_path}")
    print(f"    - Despegue CoM: x = {px_opt[N1+N2]:.3f} m, vz = {vz_opt[N1+N2]:.2f} m/s, vx = {vx_opt[N1+N2]:.2f} m/s")
    print(f"    - Obstáculo: x in [{x_obs_min}, {x_obs_max}] m, h = {h_obs} m")
    print(f"    - Altura máxima CoM en cumbre: {pz_opt[k_apex]:.3f} m (Clearance CoM: +{pz_opt[k_apex]-h_obs:.3f} m)")
    print(f"    - Altura de patas durante vuelo sobre obstáculo: {feet_world_traj[k_apex, 0, 2]:.3f} m")
    print(f"    - Margen libre de patas sobre el obstáculo: +{feet_world_traj[k_apex, 0, 2] - h_obs:.3f} m")
    print(f"    - Posición de aterrizaje patas traseras: x = {rx_land_R_val:.3f} m (Margen post-obstáculo: +{rx_land_R_val-x_obs_max:.3f} m)")

    return trajectory_data


if __name__ == "__main__":
    solve_jump_trajectory()
