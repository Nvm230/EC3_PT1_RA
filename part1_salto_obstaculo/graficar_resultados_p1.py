#!/usr/bin/env python3
"""
===============================================================================
GENERACIÓN DE GRÁFICAS ANALÍTICAS: SALTO SOBRE OBSTÁCULO (PARTE 1)
===============================================================================
Genera figuras profesionales, nítidas y desahogadas de:
  1. Perfil espacial en el plano sagital (X vs Z) y evolución temporal
     desacoplada de posición y velocidad del CoM (sin solapamiento de leyendas).
  2. Fuerzas de reacción en el piso (GRF), conos de fricción de Coulomb
     y dinámica de orientación de cabeceo (pitch) con doble escala.
  3. Trayectorias articulares de las 4 patas con límites mecánicos del Unitree A1.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Configuración visual sobria y elegante con márgenes generosos
plt.rcParams.update({
    "font.size": 10.5,
    "axes.labelsize": 11.5,
    "axes.titlesize": 12.5,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 9,
    "figure.titlesize": 13.5,
    "grid.alpha": 0.35,
    "grid.linestyle": "--"
})


def generate_plots():
    data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trayectoria_salto_opt.npy")
    if not os.path.exists(data_path):
        from opt_salto_obstaculo import solve_jump_trajectory
        data = solve_jump_trajectory()
    else:
        data = np.load(data_path, allow_pickle=True).item()

    output_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "outputs"))
    os.makedirs(output_dir, exist_ok=True)

    t = data["time"]
    px = data["state"]["px"]
    pz = data["state"]["pz"]
    vx = data["state"]["vx"]
    vz = data["state"]["vz"]
    th = data["state"]["theta"]
    om = data["state"]["omega"]

    fx_F = data["forces"]["fx_F"]
    fz_F = data["forces"]["fz_F"]
    fx_R = data["forces"]["fx_R"]
    fz_R = data["forces"]["fz_R"]

    feet = data["feet_world"]
    q_joints = data["q_joints"]
    obs = data["obstacle"]
    phases = data["phases"]
    N1, N2, N3, N4 = phases["N1"], phases["N2"], phases["N3"], phases["N4"]

    t_crouch = t[N1]
    t_takeoff = t[N1 + N2]
    t_flight_end = t[N1 + N2 + N3]
    k_apex = N1 + N2 + N3 // 2
    idx_flight = slice(N1 + N2, N1 + N2 + N3 + 1)

    # =========================================================================
    # FIGURA 1: PERFIL ESPACIAL SAGITAL (X vs Z) Y CINEMÁTICA TEMPORAL
    # =========================================================================
    fig = plt.figure(figsize=(11, 10.5))
    gs = fig.add_gridspec(3, 1, height_ratios=[1.35, 0.85, 0.85], hspace=0.38)

    ax1 = fig.add_subplot(gs[0])
    ax2 = fig.add_subplot(gs[1])
    ax3 = fig.add_subplot(gs[2], sharex=ax2)

    # -------------------------------------------------------------
    # Subplot 1: Plano Sagital X-Z
    # -------------------------------------------------------------
    ax1.axhline(0.0, color="#2C3E50", lw=2, ls="-", label="Suelo (z = 0)")

    # Obstáculo rígido con franja de seguridad en amarillo
    obs_w = obs["x_max"] - obs["x_min"]
    rect_body = patches.Rectangle((obs["x_min"], 0.0), obs_w, obs["h"] - 0.02,
                                  linewidth=1.5, edgecolor="#1B2631", facecolor="#2C3E50",
                                  alpha=0.9, label=f"Obstaculo (h = {obs['h']:.2f} m)")
    rect_trim = patches.Rectangle((obs["x_min"], obs["h"] - 0.02), obs_w, 0.02,
                                  linewidth=1.2, edgecolor="#B7950B", facecolor="#F1C40F",
                                  alpha=1.0, label="Franja Seguridad Amarilla")
    ax1.add_patch(rect_body)
    ax1.add_patch(rect_trim)

    # Trayectoria parabólica de avance y vuelo del CoM (sin el hook estático de agache)
    idx_jump = slice(N1, N1 + N2 + N3 + 1)
    ax1.plot(px[idx_jump], pz[idx_jump], color="#1B4F72", lw=3.0, label="Trayectoria CoM (Base)")

    # Indicador de agache preparatorio en el origen (0, 0.30) -> (0, 0.18)
    ax1.annotate("", xy=(0.0, pz[N1]), xytext=(0.0, pz[0]),
                 arrowprops=dict(arrowstyle="->", lw=2.0, color="#1B4F72", ls=":"))
    ax1.scatter([0.0], [pz[0]], color="#1B4F72", s=50, marker="o")
    ax1.text(0.02, 0.24, "Agache\n$\\Delta z = 12$ cm", fontsize=8.5, color="#1B4F72", va="center")

    # Trayectorias aéreas de los pies (durante vuelo libre)
    ax1.plot(feet[idx_flight, 0, 0], feet[idx_flight, 0, 2], color="#27AE60", ls="--", lw=2.2, label="Pies Frontales Vuelo (FL/FR)")
    ax1.plot(feet[idx_flight, 2, 0], feet[idx_flight, 2, 2], color="#8E44AD", ls="--", lw=2.2, label="Pies Traseros Vuelo (RL/RR)")

    # Puntos de contacto en despegue y aterrizaje
    p_takeoff_F = [obs["x_min"] - 0.34, 0.0]
    p_takeoff_R = [obs["x_min"] - 0.70, 0.0]
    p_land_F = [data["landing"]["rx_land_F"], 0.0]
    p_land_R = [data["landing"]["rx_land_R"], 0.0]

    ax1.scatter([p_takeoff_F[0], p_takeoff_R[0]], [0.0, 0.0], color="#D35400", s=65, marker="o", zorder=6, label="Apoyos Despegue")
    ax1.scatter([p_land_F[0], p_land_R[0]], [0.0, 0.0], color="#2980B9", s=65, marker="s", zorder=6, label="Apoyos Aterrizaje")

    # Vértice del CoM y cotas de despeje
    ax1.scatter([px[k_apex]], [pz[k_apex]], color="#C0392B", s=90, zorder=7)
    clearance_com = pz[k_apex] - obs["h"]
    ax1.annotate(f"Vertice CoM: z = {pz[k_apex]:.3f} m\nDespeje CoM: +{clearance_com*100:.1f} cm",
                 xy=(px[k_apex], pz[k_apex]), xytext=(px[k_apex] - 0.22, pz[k_apex] + 0.14),
                 arrowprops=dict(arrowstyle="->", lw=1.5, color="#C0392B"),
                 bbox=dict(boxstyle="round,pad=0.3", fc="#FDEDEC", ec="#C0392B", lw=1.2))

    # Cota de margen libre de patas sobre el obstáculo
    foot_h_above_obs = np.min(feet[idx_flight, :, 2])
    clearance_foot = foot_h_above_obs - obs["h"]
    x_mid_obs = 0.5 * (obs["x_min"] + obs["x_max"])
    ax1.annotate(f"Despeje Neto de Patas\nMargen: +{clearance_foot*100:.1f} cm",
                 xy=(x_mid_obs, foot_h_above_obs), xytext=(x_mid_obs + 0.16, foot_h_above_obs + 0.10),
                 arrowprops=dict(arrowstyle="->", lw=1.3, color="#27AE60"),
                 bbox=dict(boxstyle="round,pad=0.3", fc="#E8F8F5", ec="#27AE60", lw=1.2))

    ax1.set_xlabel("Posicion Longitudinal X [m]")
    ax1.set_ylabel("Altura Z [m]")
    ax1.set_title("Plano Sagital: Salto Balistico y Superacion del Obstaculo Alto (Unitree A1)")
    # Leyenda situada fuera del área de conflicto en la esquina superior izquierda
    ax1.legend(loc="upper left", framealpha=0.92, ncol=2)
    ax1.grid(True)
    ax1.set_xlim(-0.15, 1.35)
    ax1.set_ylim(-0.04, 0.98)

    # -------------------------------------------------------------
    # Subplot 2: Posición del CoM vs Tiempo
    # -------------------------------------------------------------
    ax2.plot(t, px, color="#1F618D", lw=2.2, label=r"Avance Longitudinal $p_x(t)$ [m]")
    ax2.plot(t, pz, color="#117864", lw=2.2, label=r"Altura Vertical $p_z(t)$ [m]")
    ax2.axhline(obs["h"], color="#C0392B", ls=":", lw=1.2, label=f"Altura Obstaculo ({obs['h']} m)")

    for t_ph in [t_crouch, t_takeoff, t_flight_end]:
        ax2.axvline(t_ph, color="#7F8C8D", ls=":", lw=1.2)
        ax3.axvline(t_ph, color="#7F8C8D", ls=":", lw=1.2)

    y_lbl = 1.15
    ax2.text(t_crouch / 2, y_lbl, "Compresion", ha="center", fontsize=8.5, bbox=dict(fc="#F2F4F4", ec="gray", pad=1.5))
    ax2.text((t_crouch + t_takeoff) / 2, y_lbl, "Impulso", ha="center", fontsize=8.5, bbox=dict(fc="#F2F4F4", ec="gray", pad=1.5))
    ax2.text((t_takeoff + t_flight_end) / 2, y_lbl, "Vuelo Libre", ha="center", fontsize=8.5, bbox=dict(fc="#F2F4F4", ec="gray", pad=1.5))
    ax2.text((t_flight_end + t[-1]) / 2, y_lbl, "Aterrizaje", ha="center", fontsize=8.5, bbox=dict(fc="#F2F4F4", ec="gray", pad=1.5))

    ax2.set_ylabel("Posicion [m]")
    ax2.set_title("Evolucion Temporal de Posicion del Centro de Masa (CoM)")
    ax2.legend(loc="center left", framealpha=0.9)
    ax2.grid(True)
    ax2.set_ylim(-0.05, 1.35)

    # -------------------------------------------------------------
    # Subplot 3: Velocidad del CoM vs Tiempo
    # -------------------------------------------------------------
    ax3.plot(t, vx, color="#2980B9", lw=2.0, ls="-", label=r"Velocidad Horizontal $v_x(t)$ [m/s]")
    ax3.plot(t, vz, color="#16A085", lw=2.0, ls="-", label=r"Velocidad Vertical $v_z(t)$ [m/s]")
    ax3.axhline(0.0, color="gray", lw=0.8, ls="--")

    ax3.set_xlabel("Tiempo [s]")
    ax3.set_ylabel("Velocidad [m/s]")
    ax3.set_title("Evolucion Temporal de Velocidad del Centro de Masa (CoM)")
    ax3.legend(loc="lower left", framealpha=0.9)
    ax3.grid(True)
    ax3.set_ylim(-2.8, 3.5)
    ax3.set_xlim(0.0, t[-1])

    p1_traj_path = os.path.join(output_dir, "p1_trayectoria_salto.png")
    fig.savefig(p1_traj_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f">>> Grafica de trayectoria guardada en: {p1_traj_path}")

    # =========================================================================
    # FIGURA 2: FUERZAS DE REACCIÓN (GRF) Y DINÁMICA DE CABECEO
    # =========================================================================
    t_steps = t[:-1]
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    # 1. Fuerzas normales verticales
    ax1.plot(t_steps, fz_F, color="#2980B9", lw=2.2, label=r"$F_{z}$ Frontal (FL+FR) [N]")
    ax1.plot(t_steps, fz_R, color="#8E44AD", lw=2.2, label=r"$F_{z}$ Trasero (RL+RR) [N]")
    ax1.plot(t_steps, fz_F + fz_R, color="#2C3E50", lw=1.8, ls="--", label=r"$F_{z}$ Total [N]")
    ax1.axhline(12.0 * 9.81, color="#C0392B", ls=":", lw=1.3, label="Peso Robot ($mg = 117.7$ N)")
    ax1.set_ylabel("Fuerza Vertical [N]")
    ax1.set_title("Fuerzas de Reaccion del Suelo (GRF) - Componentes Verticales")
    ax1.legend(loc="upper right", ncol=4, framealpha=0.9)
    ax1.grid(True)
    ax1.set_ylim(-20, 800)

    # 2. Fuerzas tangenciales y conos de fricción
    mu = 0.7
    ax2.plot(t_steps, fx_F, color="#2980B9", lw=2.0, label=r"$F_x$ Frontal [N]")
    ax2.plot(t_steps, fx_R, color="#8E44AD", lw=2.0, label=r"$F_x$ Trasero [N]")
    ax2.plot(t_steps, mu * (fz_F + fz_R), color="#E67E22", ls=":", lw=1.4, label=r"Limite Cono Friccion $\pm\mu F_z$")
    ax2.plot(t_steps, -mu * (fz_F + fz_R), color="#E67E22", ls=":", lw=1.4)
    ax2.axhline(0.0, color="gray", lw=0.8, ls="--")
    ax2.set_ylabel("Fuerza Tangencial [N]")
    ax2.set_title("Fuerzas Horizontales Tangenciales y Conos de Friccion de Coulomb")
    ax2.legend(loc="upper right", ncol=3, framealpha=0.9)
    ax2.grid(True)
    ax2.set_ylim(-320, 320)

    # 3. Cabeceo con doble escala
    ax3_twin = ax3.twinx()
    l1 = ax3.plot(t, np.rad2deg(th), color="#1E8449", lw=2.2, label=r"Angulo Pitch $\theta(t)$ [deg]")
    l2 = ax3_twin.plot(t, np.rad2deg(om), color="#B03A2E", lw=1.8, ls="--", label=r"Vel. Angular Pitch $\omega_y(t)$ [deg/s]")

    ax3.set_xlabel("Tiempo [s]")
    ax3.set_ylabel("Pitch [deg]", color="#1E8449")
    ax3_twin.set_ylabel("Vel. Angular [deg/s]", color="#B03A2E")
    ax3.tick_params(axis="y", labelcolor="#1E8449")
    ax3_twin.tick_params(axis="y", labelcolor="#B03A2E")
    ax3.set_title("Dinamica de Orientacion de Cabeceo (Pitch)")

    lines = l1 + l2
    labels = [l.get_label() for l in lines]
    ax3.legend(lines, labels, loc="upper right", framealpha=0.9)
    ax3.grid(True)
    ax3.set_ylim(-0.45, 0.45)
    ax3_twin.set_ylim(-3.0, 3.0)

    for ax in [ax1, ax2, ax3]:
        for t_phase in [t_crouch, t_takeoff, t_flight_end]:
            ax.axvline(t_phase, color="#7F8C8D", ls=":", lw=1.2)

    p1_forces_path = os.path.join(output_dir, "p1_fuerzas_grf.png")
    fig.savefig(p1_forces_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f">>> Grafica de fuerzas guardada en: {p1_forces_path}")

    # =========================================================================
    # FIGURA 3: TRAYECTORIAS ARTICULARES DE LAS 4 PATAS (IK COMPLETA)
    # =========================================================================
    fig, axes = plt.subplots(3, 1, figsize=(11, 9.5), sharex=True)
    leg_names = ["FL (Frontal Izq)", "FR (Frontal Der)", "RL (Trasera Izq)", "RR (Trasera Der)"]
    colors = ["#1F77B4", "#FF7F0E", "#2CA02C", "#D62728"]

    # 1. Hip Roll
    for i in range(4):
        axes[0].plot(t, q_joints[:, 3*i], color=colors[i], lw=1.8, label=leg_names[i])
    axes[0].axhline(0.80, color="gray", ls="--", lw=1)
    axes[0].axhline(-0.80, color="gray", ls="--", lw=1, label="Limites Articulares")
    axes[0].set_ylabel("Hip Roll [rad]")
    axes[0].set_title("Evolucion de Articulaciones Unitree A1 durante el Salto (Cinematica Inversa)")
    axes[0].legend(loc="upper right", ncol=5, framealpha=0.9)
    axes[0].grid(True)
    axes[0].set_ylim(-1.0, 1.0)

    # 2. Thigh Pitch
    for i in range(4):
        axes[1].plot(t, q_joints[:, 3*i + 1], color=colors[i], lw=1.8, label=leg_names[i])
    axes[1].axhline(3.50, color="gray", ls="--", lw=1)
    axes[1].axhline(-1.00, color="gray", ls="--", lw=1, label="Limites Articulares")
    axes[1].set_ylabel("Thigh Pitch [rad]")
    axes[1].grid(True)
    axes[1].set_ylim(-1.2, 3.8)

    # 3. Calf Pitch
    for i in range(4):
        axes[2].plot(t, q_joints[:, 3*i + 2], color=colors[i], lw=1.8, label=leg_names[i])
    axes[2].axhline(-0.90, color="gray", ls="--", lw=1)
    axes[2].axhline(-2.70, color="gray", ls="--", lw=1, label="Limites Articulares")
    axes[2].set_ylabel("Calf Pitch [rad]")
    axes[2].set_xlabel("Tiempo [s]")
    axes[2].grid(True)
    axes[2].set_ylim(-2.9, -0.6)

    for ax in axes:
        for t_phase in [t_crouch, t_takeoff, t_flight_end]:
            ax.axvline(t_phase, color="#7F8C8D", ls=":", lw=1.2)

    p1_joints_path = os.path.join(output_dir, "p1_articulaciones.png")
    fig.savefig(p1_joints_path, dpi=200, bbox_inches="tight")
    plt.close(fig)
    print(f">>> Grafica de articulaciones guardada en: {p1_joints_path}")


if __name__ == "__main__":
    generate_plots()
