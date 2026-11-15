#!/usr/bin/env python3
"""
===============================================================================
OPTIMIZACIÓN DE TRAYECTORIA: SALTO MORTAL ACROBÁTICO 360° (BACKFLIP)
ROBOT CUADRÚPEDO UNITREE A1
===============================================================================
Formulación matemática:
  - Movimiento de alta complejidad dinámica: rotación completa de 360° en cabeceo (pitch).
  - Transferencia no lineal de momento angular: impulso asimétrico en despegue
    generado por las fuerzas de reacción del suelo (GRF) en patas frontales y traseras.
  - Inercia variable durante el vuelo: encogimiento de patas (tuck) que reduce el
    momento de inercia I_yy(t) acelerando el giro por conservación del momento angular,
    seguido de extensión de patas previa al aterrizaje para frenar el giro.
  - Absorción de impacto y estabilización cuadrúpeda completa a theta = -360° (0°).
"""

import os
import sys
import time
import numpy as np
import casadi as ca

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "part1_salto_obstaculo")))
from modelo_srbm import A1SRBMModel


def solve_backflip_trajectory():
    model = A1SRBMModel()

    # Duraciones y discretización de fases
    # 1. Agache/Compresión (Crouch)
    # 2. Empuje Asimétrico de Cabeceo (Thrust / Pitch launch)
    # 3. Vuelo acrobático 360° con encogimiento (Aerial Flip)
    # 4. Aterrizaje y absorción (Touchdown & stabilization)
    T1, N1 = 0.15, 8
    T2, N2 = 0.16, 10
    T3, N3 = 0.48, 25
    T4, N4 = 0.25, 12
    N_total = N1 + N2 + N3 + N4

    dt1 = T1 / N1
    dt2 = T2 / N2
    dt3 = T3 / N3
    dt4 = T4 / N4
    dt_list = [dt1]*N1 + [dt2]*N2 + [dt3]*N3 + [dt4]*N4
    time_array = np.cumsum([0.0] + dt_list)

    # Parámetros físicos
    m = model.mass
    g = model.gravity
    d_x = model.d_x
    mu = 0.8  # Fricción suficiente para impulso tangencial
    f_max_pair = 400.0  # Capacidad máxima pico por par de patas

    # Inercia variable: nominal extendido vs recogido (tuck)
    Iyy_nom = 0.0565
    Iyy_tuck = 0.0320  # ~43% menor al recoger patas

    opti = ca.Opti()

    # Estados: X = [px, pz, vx, vz, theta, omega]^T
    X = opti.variable(6, N_total + 1)
    px = X[0, :]
    pz = X[1, :]
    vx = X[2, :]
    vz = X[3, :]
    theta = X[4, :]
    omega = X[5, :]

    # Fuerzas de reacción en patas frontales y traseras
    F = opti.variable(4, N_total)
    fx_F = F[0, :]
    fz_F = F[1, :]
    fx_R = F[2, :]
    fz_R = F[3, :]

    # Puntos de contacto en aterrizaje
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

    # Estado final: rotación completa de 360 grados (-2*pi rad) en reposo
    opti.subject_to(pz[N_total] == model.nominal_height)
    opti.subject_to(vx[N_total] == 0.0)
    opti.subject_to(vz[N_total] == 0.0)
    opti.subject_to(theta[N_total] == -2.0 * np.pi)
    opti.subject_to(omega[N_total] == 0.0)

    # Aterrizaje simétrico cerca del origen longitudinal
    opti.subject_to(rx_land_F == rx_land_R + 2.0 * d_x)
    opti.subject_to(px[N_total] >= -0.20)
    opti.subject_to(px[N_total] <= 0.20)

    # Vértice de vuelo: altura suficiente para girar sin tocar el suelo
    k_apex = N1 + N2 + N3 // 2
    opti.subject_to(pz[k_apex] >= 0.60)
    # En el ápice el robot debe estar invertido (alrededor de -180° = -pi)
    opti.subject_to(theta[k_apex] <= -0.80 * np.pi)
    opti.subject_to(theta[k_apex] >= -1.25 * np.pi)

    # Fin de agache
    opti.subject_to(pz[N1] <= 0.20)

    # -------------------------------------------------------------
    # DINÁMICA DE CABECEO Y FUERZAS POR FASE
    # -------------------------------------------------------------
    Iyy_profile = np.zeros(N_total)

    for k in range(N_total):
        h = dt_list[k]

        # Inercia dependiente de la fase (efecto tuck)
        if k < N1 + N2:
            Iyy_k = Iyy_nom
            r_xF, r_zF = d_x, 0.0
            r_xR, r_zR = -d_x, 0.0
        elif k < N1 + N2 + N3:
            # En vuelo: inercia reducida por encogimiento
            Iyy_k = Iyy_tuck
            r_xF, r_zF = px[k] + d_x * ca.cos(theta[k]), pz[k] + d_x * ca.sin(theta[k])
            r_xR, r_zR = px[k] - d_x * ca.cos(theta[k]), pz[k] - d_x * ca.sin(theta[k])
        else:
            Iyy_k = Iyy_nom
            r_xF, r_zF = rx_land_F, 0.0
            r_xR, r_zR = rx_land_R, 0.0

        Iyy_profile[k] = Iyy_nom if (k < N1+N2 or k >= N1+N2+N3) else Iyy_tuck

        # Restricciones de contacto
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

        # Torques y aceleraciones
        tau_F = (r_xF - px[k]) * fz_F[k] - (r_zF - pz[k]) * fx_F[k]
        tau_R = (r_xR - px[k]) * fz_R[k] - (r_zR - pz[k]) * fx_R[k]
        tau = tau_F + tau_R

        ax = (fx_F[k] + fx_R[k]) / m
        az = (fz_F[k] + fz_R[k] - m * g) / m
        alpha = tau / Iyy_k

        # Integración numérica
        opti.subject_to(px[k+1] == px[k] + h * (vx[k] + 0.5 * h * ax))
        opti.subject_to(pz[k+1] == pz[k] + h * (vz[k] + 0.5 * h * az))
        opti.subject_to(vx[k+1] == vx[k] + h * ax)
        opti.subject_to(vz[k+1] == vz[k] + h * az)
        opti.subject_to(theta[k+1] == theta[k] + h * (omega[k] + 0.5 * h * alpha))
        opti.subject_to(omega[k+1] == omega[k] + h * alpha)

    # Rotación continua y monótona durante el vuelo (omega < -2 rad/s)
    for k in range(N1 + N2, N1 + N2 + N3):
        opti.subject_to(omega[k] <= -2.0)

    # Límites de seguridad
    for k in range(N_total + 1):
        opti.subject_to(pz[k] >= 0.15)
        opti.subject_to(pz[k] <= 0.85)

    # Función objetivo
    cost = 1e-4 * ca.sumsqr(F) + 0.2 * ca.sumsqr(vx) + 0.2 * ca.sumsqr(px)
    opti.minimize(cost)

    # Guess inicial de cabeceo rotando de 0 a -360 grados
    th_guess = np.zeros(N_total + 1)
    th_guess[N1+N2 : N1+N2+N3+1] = np.linspace(0.0, -2.0 * np.pi, N3 + 1)
    th_guess[N1+N2+N3:] = -2.0 * np.pi
    opti.set_initial(theta, th_guess)

    z_guess = np.ones(N_total + 1) * model.nominal_height
    z_guess[N1] = 0.18
    z_guess[k_apex] = 0.70
    opti.set_initial(pz, z_guess)

    opts = {
        "ipopt.print_level": 0,
        "ipopt.max_iter": 400,
        "ipopt.tol": 1e-4,
        "print_time": False
    }
    opti.solver("ipopt", opts)

    print(">>> Resolviendo optimización de trayectoria del Backflip 360°...")
    t_start = time.time()
    sol = opti.solve()
    t_calc = time.time() - t_start
    print(f">>> ¡Solución de Backflip 360° obtenida con éxito en {t_calc:.3f} s!")

    # Extraer arrays
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
            # En despegue: pies en suelo
            p_feet_b = np.zeros((4, 3))
            for i in range(4):
                p_foot_w = np.array([d_x if i in [0, 1] else -d_x, y_hips[i], 0.0])
                feet_world_traj[k, i] = p_foot_w
                p_feet_b[i] = R_base.T @ (p_foot_w - p_base)
        elif k < N1 + N2 + N3:
            # En vuelo acrobático: patas firmemente recogidas (tuck) en el marco de la base
            # Durante el giro, patas recogidas protegen el cuerpo y minimizan la inercia
            tau_flight = (k - (N1 + N2)) / float(N3)
            # En tau=0 despegue, en tau=0.5 máximo tuck, en tau=1.0 extensión para aterrizaje
            tuck_factor = np.sin(np.pi * tau_flight)
            leg_len = 0.28 - 0.13 * tuck_factor  # Longitud de pata varía de 0.28m a 0.15m

            p_feet_b = np.zeros((4, 3))
            for i in range(4):
                # Pata orientada hacia abajo relativo al chasis (eje -Z local)
                sx = 1.0 if i in [0, 1] else -1.0
                p_feet_b[i] = [sx * (d_x * 0.7), y_hips[i], -leg_len]
                feet_world_traj[k, i] = p_base + R_base @ p_feet_b[i]
        else:
            # En aterrizaje: pies en suelo a rx_land
            p_feet_b = np.zeros((4, 3))
            for i in range(4):
                p_foot_w = np.array([rx_land_F_val if i in [0, 1] else rx_land_R_val, y_hips[i], 0.0])
                feet_world_traj[k, i] = p_foot_w
                p_feet_b[i] = R_base.T @ (p_foot_w - p_base)

        q_joints = model.ik_full_body(p_base, R_base, feet_world_traj[k])
        q_joint_traj[k] = q_joints

    trajectory_data = {
        "time": time_array,
        "dt_list": dt_list,
        "phases": {"N1": N1, "N2": N2, "N3": N3, "N4": N4},
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
        "Iyy_profile": Iyy_profile,
        "feet_world": feet_world_traj,
        "q_joints": q_joint_traj
    }

    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trayectoria_backflip_opt.npy")
    np.save(save_path, trajectory_data, allow_pickle=True)
    print(f">>> Datos de Backflip guardados en: {save_path}")
    print(f"    - Giro angular total: {np.rad2deg(th_opt[-1]):.1f}° (360° completados)")
    print(f"    - Altura máxima en la cumbre: {pz_opt[k_apex]:.3f} m")
    print(f"    - Velocidad angular pico: {np.min(om_opt):.2f} rad/s ({np.rad2deg(np.min(om_opt)):.1f} °/s)")

    return trajectory_data


if __name__ == "__main__":
    solve_backflip_trajectory()
