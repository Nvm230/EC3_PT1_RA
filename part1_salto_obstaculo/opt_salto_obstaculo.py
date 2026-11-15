#!/usr/bin/env python3
"""
===============================================================================
OPTIMIZACIÓN DE TRAYECTORIA: SALTO SOBRE OBSTÁCULO ALTO (UNITREE A1)
===============================================================================
Formulación matemática:
  - Modelo dinámico: Single Rigid Body Model (SRBM) en el plano sagital (X, Z, Pitch).
  - Método: Colocación directa / Multiple Shooting con solver IPOPT vía CasADi.
  - Fases del salto:
      1. Agache/Compresión (Stance 1): acumulación de energía y descenso controlado del CoM.
      2. Empuje/Propulsión (Stance 2): aplicación explosiva de GRF con cono de fricción.
      3. Vuelo Parabólico (Flight): fuerzas nulas, retracción cinemática de patas (tuck)
         y superación holgada del obstáculo alto (h_obs = 0.35 m).
      4. Aterrizaje/Amortiguación (Stance 3): disipación de impacto en piso y estabilización.
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
    x_obs_min = 0.35
    x_obs_max = 0.65
    h_obs = 0.35  # 35 cm de altura (mayor que la altura nominal de 30 cm)

    # Duraciones y discretización por fase
    T1, N1 = 0.15, 8    # Agache
    T2, N2 = 0.15, 8    # Empuje explosivo
    T3, N3 = 0.45, 18   # Vuelo sobre el obstáculo
    T4, N4 = 0.20, 10   # Aterrizaje y absorción
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
    mu = model.mu
    d_x = model.d_x
    f_max_pair = 2.0 * model.f_max_per_leg  # 320 N por par de patas

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

    # Posición de aterrizaje de pies (optimizada)
    rx_land_F = opti.variable(1)
    rx_land_R = opti.variable(1)

    # -------------------------------------------------------------
    # CONDICIONES DE FRONTERA
    # -------------------------------------------------------------
    # Estado inicial: reposo a altura nominal (0.30 m)
    opti.subject_to(px[0] == 0.0)
    opti.subject_to(pz[0] == model.nominal_height)
    opti.subject_to(vx[0] == 0.0)
    opti.subject_to(vz[0] == 0.0)
    opti.subject_to(theta[0] == 0.0)
    opti.subject_to(omega[0] == 0.0)

    # Estado final: reposo tras superar el obstáculo
    opti.subject_to(px[N_total] >= 0.88)
    opti.subject_to(pz[N_total] == model.nominal_height)
    opti.subject_to(vx[N_total] == 0.0)
    opti.subject_to(vz[N_total] == 0.0)
    opti.subject_to(theta[N_total] == 0.0)
    opti.subject_to(omega[N_total] == 0.0)

    # Aterrizaje past the obstacle
    opti.subject_to(rx_land_R >= x_obs_max + 0.06)
    opti.subject_to(rx_land_F == rx_land_R + 2.0 * d_x)

    # Clearance en la cumbre del salto (apex sobre el obstáculo)
    k_apex = N1 + N2 + N3 // 2
    opti.subject_to(pz[k_apex] >= h_obs + 0.23)  # CoM >= 0.58 m
    opti.subject_to(pz[N1] <= 0.22)               # Agache previo

    # -------------------------------------------------------------
    # DINÁMICA SRBM Y RESTRICCIONES DE CONTACTO
    # -------------------------------------------------------------
    for k in range(N_total):
        h = dt_list[k]

        # Puntos de contacto según la fase
        if k < N1 + N2:
            r_xF, r_zF = d_x, 0.0
            r_xR, r_zR = -d_x, 0.0
        elif k < N1 + N2 + N3:
            r_xF, r_zF = px[k] + d_x, pz[k] - 0.15
            r_xR, r_zR = px[k] - d_x, pz[k] - 0.15
        else:
            r_xF, r_zF = rx_land_F, 0.0
            r_xR, r_zR = rx_land_R, 0.0

        # Restricciones de fuerza por fase
        if k < N1 + N2 or k >= N1 + N2 + N3:  # Soporte (Stance)
            opti.subject_to(fz_F[k] >= 0.0)
            opti.subject_to(fz_R[k] >= 0.0)
            opti.subject_to(fz_F[k] <= f_max_pair)
            opti.subject_to(fz_R[k] <= f_max_pair)
            opti.subject_to(fx_F[k] <= mu * fz_F[k])
            opti.subject_to(fx_F[k] >= -mu * fz_F[k])
            opti.subject_to(fx_R[k] <= mu * fz_R[k])
            opti.subject_to(fx_R[k] >= -mu * fz_R[k])
        else:  # Vuelo balístico libre
            opti.subject_to(fx_F[k] == 0.0)
            opti.subject_to(fz_F[k] == 0.0)
            opti.subject_to(fx_R[k] == 0.0)
            opti.subject_to(fz_R[k] == 0.0)

        # Torques respecto al CoM
        tau_F = (r_xF - px[k]) * fz_F[k] - (r_zF - pz[k]) * fx_F[k]
        tau_R = (r_xR - px[k]) * fz_R[k] - (r_zR - pz[k]) * fx_R[k]
        tau = tau_F + tau_R

        ax = (fx_F[k] + fx_R[k]) / m
        az = (fz_F[k] + fz_R[k] - m * g) / m
        alpha = tau / Iyy

        # Integración numérica
        opti.subject_to(px[k+1] == px[k] + h * (vx[k] + 0.5 * h * ax))
        opti.subject_to(pz[k+1] == pz[k] + h * (vz[k] + 0.5 * h * az))
        opti.subject_to(vx[k+1] == vx[k] + h * ax)
        opti.subject_to(vz[k+1] == vz[k] + h * az)
        opti.subject_to(theta[k+1] == theta[k] + h * (omega[k] + 0.5 * h * alpha))
        opti.subject_to(omega[k+1] == omega[k] + h * alpha)

    # Restricciones globales de seguridad
    for k in range(N_total + 1):
        opti.subject_to(pz[k] >= 0.16)
        opti.subject_to(pz[k] <= 0.75)
        opti.subject_to(theta[k] >= -0.30)
        opti.subject_to(theta[k] <= 0.30)

    # Función objetivo
    cost = 1e-4 * ca.sumsqr(F) + 20.0 * ca.sumsqr(theta) + 2.0 * ca.sumsqr(omega) + 1.0 * (px[N_total] - 0.90)**2
    opti.minimize(cost)

    # Configuración de IPOPT
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

    # Extraer valores numéricos óptimos
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
    # GENERACIÓN DE TRAYECTORIAS ARTICULARES (IK DE CUERPO COMPLETO)
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

        if k <= N1 + N2:
            # Fase 1 y 2: pies en suelo inicial
            rx_f = d_x
            rx_r = -d_x
            rz_f = 0.0
            rz_r = 0.0
        elif k < N1 + N2 + N3:
            # Fase 3 (Vuelo): retracción de patas (tuck)
            tau_flight = (k - (N1 + N2)) / float(N3)
            # Retracción hacia arriba de hasta 17 cm
            tuck_z = 0.17 * np.sin(np.pi * tau_flight)

            rx_f = (1.0 - tau_flight) * d_x + tau_flight * rx_land_F_val
            rx_r = (1.0 - tau_flight) * (-d_x) + tau_flight * rx_land_R_val
            rz_f = max(0.0, pz_opt[k] - 0.28 + tuck_z)
            rz_r = max(0.0, pz_opt[k] - 0.28 + tuck_z)
        else:
            # Fase 4: pies en suelo final
            rx_f = rx_land_F_val
            rx_r = rx_land_R_val
            rz_f = 0.0
            rz_r = 0.0

        p_feet = np.zeros((4, 3))
        p_feet[0] = [rx_f, y_hips[0], rz_f]
        p_feet[1] = [rx_f, y_hips[1], rz_f]
        p_feet[2] = [rx_r, y_hips[2], rz_r]
        p_feet[3] = [rx_r, y_hips[3], rz_r]

        feet_world_traj[k] = p_feet
        q_joints = model.ik_full_body(p_base, R_base, p_feet)
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
    print(f"    - Distancia total recorrida: {px_opt[-1]:.3f} m")
    print(f"    - Altura máxima en la cumbre: {pz_opt[k_apex]:.3f} m (Obstáculo: {h_obs} m)")
    print(f"    - Altura mínima de patas sobre el obstáculo: {np.min(feet_world_traj[N1+N2+2 : N1+N2+N3-2, :, 2]):.3f} m")
    print(f"    - Margen de seguridad sobre obstáculo: {np.min(feet_world_traj[N1+N2+2 : N1+N2+N3-2, :, 2]) - h_obs:.3f} m")

    return trajectory_data


if __name__ == "__main__":
    solve_jump_trajectory()
