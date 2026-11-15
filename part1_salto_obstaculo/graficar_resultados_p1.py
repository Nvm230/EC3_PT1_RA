#!/usr/bin/env python3
"""
===============================================================================
GENERACIÓN DE GRÁFICAS ANALÍTICAS: SALTO SOBRE OBSTÁCULO (PARTE 1)
===============================================================================
Genera figuras profesionales de:
  1. Perfil espacial del salto en el plano sagital (X, Z) con obstáculo y pies.
  2. Fuerzas de reacción en el piso (GRF), fricción y momentos de cabeceo.
  3. Trayectorias articulares de las 4 patas con límites mecánicos.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Configuración visual elegante
plt.rcParams.update({
    "font.size": 11,
    "axes.labelsize": 12,
    "axes.titlesize": 13,
    "xtick.labelsize": 10,
    "ytick.labelsize": 10,
    "legend.fontsize": 10,
    "figure.titlesize": 14,
    "figure.autolayout": True,
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

    # =========================================================================
    # FIGURA 1: PERFIL ESPACIAL SAGITAL (X vs Z) Y CLEARANCE DE OBSTÁCULO
    # =========================================================================
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(11, 8), gridspec_kw={"height_ratios": [1.3, 1]})

    # Subplot 1: Plano X-Z
    ax1.plot(px, pz, "b-", lw=2.5, label="Trayectoria CoM (Base)")
    ax1.plot(feet[:, 0, 0], feet[:, 0, 2], "g--", lw=1.8, label="Pies Frontales (FL/FR)")
    ax1.plot(feet[:, 2, 0], feet[:, 2, 2], "m--", lw=1.8, label="Pies Traseros (RL/RR)")

    # Obstáculo
    rect = patches.Rectangle((obs["x_min"], 0.0), obs["x_max"] - obs["x_min"], obs["h"],
                             linewidth=2, edgecolor="#C0392B", facecolor="#E74C3C", alpha=0.5,
                             label=f"Obstaculo Alto (h = {obs['h']} m)")
    ax1.add_patch(rect)
    ax1.axhline(0.0, color="k", lw=1.5, ls="-", label="Suelo (z = 0)")

    # Anotaciones
    k_apex = N1 + N2 + N3 // 2
    ax1.scatter([px[k_apex]], [pz[k_apex]], color="blue", s=70, zorder=5)
    ax1.annotate(f"Vertice CoM: z={pz[k_apex]:.2f} m\nClearance={pz[k_apex]-obs['h']:.2f} m",
                 xy=(px[k_apex], pz[k_apex]), xytext=(px[k_apex]-0.05, pz[k_apex]+0.06),
                 arrowprops=dict(arrowstyle="->", lw=1.5, color="blue"),
                 bbox=dict(boxstyle="round,pad=0.3", fc="#EBF5FB", ec="blue", lw=1))

    ax1.set_xlabel("Posicion Longitudinal X [m]")
    ax1.set_ylabel("Altura Z [m]")
    ax1.set_title("Plano Sagital: Salto y Superacion del Obstaculo Alto con Unitree A1")
    ax1.legend(loc="upper right", framealpha=0.9)
    ax1.grid(True)
    ax1.set_xlim(-0.25, 1.15)
    ax1.set_ylim(-0.05, 0.82)

    # Subplot 2: Posiciones y Velocidades CoM vs Tiempo
    ax2.plot(t, px, "b-", lw=2, label="p_x (m)")
    ax2.plot(t, pz, "c-", lw=2, label="p_z (m)")
    ax2.plot(t, vx, "b--", lw=1.5, label="v_x (m/s)")
    ax2.plot(t, vz, "c--", lw=1.5, label="v_z (m/s)")

    for t_phase in [t_crouch, t_takeoff, t_flight_end]:
        ax2.axvline(t_phase, color="gray", ls=":", lw=1.2)

    ax2.text(t_crouch/2, 2.5, "Compresion", ha="center", fontsize=9, bbox=dict(fc="white", alpha=0.7))
    ax2.text((t_crouch + t_takeoff)/2, 2.5, "Impulso", ha="center", fontsize=9, bbox=dict(fc="white", alpha=0.7))
    ax2.text((t_takeoff + t_flight_end)/2, 2.5, "Vuelo Aereo", ha="center", fontsize=9, bbox=dict(fc="white", alpha=0.7))
    ax2.text((t_flight_end + t[-1])/2, 2.5, "Aterrizaje", ha="center", fontsize=9, bbox=dict(fc="white", alpha=0.7))

    ax2.set_xlabel("Tiempo [s]")
    ax2.set_ylabel("Estado CoM")
    ax2.set_title("Evolucion Temporal de Posicion y Velocidad")
    ax2.legend(loc="lower right", ncol=4, framealpha=0.9)
    ax2.grid(True)

    fig.tight_layout()
    p1_traj_path = os.path.join(output_dir, "p1_trayectoria_salto.png")
    fig.savefig(p1_traj_path, dpi=200)
    plt.close(fig)
    print(f">>> Grafica de trayectoria guardada en: {p1_traj_path}")

    # =========================================================================
    # FIGURA 2: FUERZAS DE REACCIÓN (GRF) Y CONOS DE FRICCIÓN
    # =========================================================================
    t_steps = t[:-1]
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    # Fuerzas verticales
    ax1.plot(t_steps, fz_F, "b-", lw=2, label="F_z Frontal (FL+FR)")
    ax1.plot(t_steps, fz_R, "m-", lw=2, label="F_z Trasero (RL+RR)")
    ax1.plot(t_steps, fz_F + fz_R, "k--", lw=1.5, label="F_z Total")
    ax1.axhline(data["state"]["px"].shape[0] * 0.0 + 12.0 * 9.81, color="red", ls=":", lw=1.2, label="Peso Robot (m*g = 117.7 N)")
    ax1.set_ylabel("Fuerza Vertical [N]")
    ax1.set_title("Fuerzas de Reaccion del Suelo (GRF) y Dinamica de Contacto")
    ax1.legend(loc="upper right", ncol=4, framealpha=0.9)
    ax1.grid(True)

    # Fuerzas horizontales y conos de fricción
    mu = 0.7
    ax2.plot(t_steps, fx_F, "b-", lw=2, label="F_x Frontal")
    ax2.plot(t_steps, fx_R, "m-", lw=2, label="F_x Trasero")
    ax2.plot(t_steps, mu * fz_F, "b:", lw=1.2, label=r"Limite Cono Friccion $\pm\mu F_z$")
    ax2.plot(t_steps, -mu * fz_F, "b:", lw=1.2)
    ax2.plot(t_steps, mu * fz_R, "m:", lw=1.2)
    ax2.plot(t_steps, -mu * fz_R, "m:", lw=1.2)
    ax2.set_ylabel("Fuerza Tangencial [N]")
    ax2.legend(loc="upper right", ncol=3, framealpha=0.9)
    ax2.grid(True)

    # Cabeceo y velocidad angular
    ax3.plot(t, np.rad2deg(th), "g-", lw=2, label="Angulo Pitch [deg]")
    ax3.plot(t, np.rad2deg(om), "r--", lw=1.5, label="Velocidad Angular Pitch [deg/s]")
    ax3.set_xlabel("Tiempo [s]")
    ax3.set_ylabel("Orientacion Pitch")
    ax3.legend(loc="upper right", framealpha=0.9)
    ax3.grid(True)

    for ax in [ax1, ax2, ax3]:
        for t_phase in [t_crouch, t_takeoff, t_flight_end]:
            ax.axvline(t_phase, color="gray", ls=":", lw=1.2)

    fig.tight_layout()
    p1_forces_path = os.path.join(output_dir, "p1_fuerzas_grf.png")
    fig.savefig(p1_forces_path, dpi=200)
    plt.close(fig)
    print(f">>> Grafica de fuerzas guardada en: {p1_forces_path}")

    # =========================================================================
    # FIGURA 3: TRAYECTORIAS ARTICULARES DE LAS 4 PATAS (IK COMPLETA)
    # =========================================================================
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    leg_names = ["FL", "FR", "RL", "RR"]
    colors = ["#1F77B4", "#FF7F0E", "#2CA02C", "#D62728"]

    # 1. Hip Roll
    for i in range(4):
        axes[0].plot(t, q_joints[:, 3*i], color=colors[i], lw=1.8, label=f"Pata {leg_names[i]}")
    axes[0].axhline(0.80, color="gray", ls="--", lw=1)
    axes[0].axhline(-0.80, color="gray", ls="--", lw=1, label="Limites Articulares")
    axes[0].set_ylabel("Hip Roll [rad]")
    axes[0].set_title("Evolucion de Articulaciones Unitree A1 durante el Salto (Cinematica Inversa)")
    axes[0].legend(loc="upper right", ncol=5, framealpha=0.9)
    axes[0].grid(True)

    # 2. Thigh Pitch
    for i in range(4):
        axes[1].plot(t, q_joints[:, 3*i + 1], color=colors[i], lw=1.8, label=f"Pata {leg_names[i]}")
    axes[1].axhline(3.50, color="gray", ls="--", lw=1)
    axes[1].axhline(-1.00, color="gray", ls="--", lw=1, label="Limites Articulares")
    axes[1].set_ylabel("Thigh Pitch [rad]")
    axes[1].grid(True)

    # 3. Calf Pitch
    for i in range(4):
        axes[2].plot(t, q_joints[:, 3*i + 2], color=colors[i], lw=1.8, label=f"Pata {leg_names[i]}")
    axes[2].axhline(-0.90, color="gray", ls="--", lw=1)
    axes[2].axhline(-2.70, color="gray", ls="--", lw=1, label="Limites Articulares")
    axes[2].set_ylabel("Calf Pitch [rad]")
    axes[2].set_xlabel("Tiempo [s]")
    axes[2].grid(True)

    for ax in axes:
        for t_phase in [t_crouch, t_takeoff, t_flight_end]:
            ax.axvline(t_phase, color="gray", ls=":", lw=1.2)

    fig.tight_layout()
    p1_joints_path = os.path.join(output_dir, "p1_articulaciones.png")
    fig.savefig(p1_joints_path, dpi=200)
    plt.close(fig)
    print(f">>> Grafica de articulaciones guardada en: {p1_joints_path}")


if __name__ == "__main__":
    generate_plots()
