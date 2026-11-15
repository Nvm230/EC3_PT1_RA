#!/usr/bin/env python3
"""
===============================================================================
GENERACIÓN DE GRÁFICAS ANALÍTICAS: SALTO MORTAL 360° (BACKFLIP) - PARTE 2
===============================================================================
Genera figuras profesionales de:
  1. Evolución cinemática y dinámica rotacional: ángulo pitch (0 a -360°),
     velocidad angular, trayectoria de CoM y reducción de inercia I_yy(t) (tucking).
  2. Fuerzas de reacción del suelo (GRF), torques asimétricos de cabeceo y fricción.
  3. Evolución de los 12 ángulos articulares de Unitree A1 durante el backflip.
"""

import os
import sys
import numpy as np
import matplotlib.pyplot as plt

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
    data_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trayectoria_backflip_opt.npy")
    if not os.path.exists(data_path):
        from opt_backflip import solve_backflip_trajectory
        data = solve_backflip_trajectory()
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

    Iyy_profile = data["Iyy_profile"]
    q_joints = data["q_joints"]
    phases = data["phases"]
    N1, N2, N3, N4 = phases["N1"], phases["N2"], phases["N3"], phases["N4"]

    t_crouch = t[N1]
    t_takeoff = t[N1 + N2]
    t_flight_end = t[N1 + N2 + N3]

    # =========================================================================
    # FIGURA 1: DINÁMICA ROTACIONAL 360°, TRAYECTORIA CoM E INERCIA VARIABLE
    # =========================================================================
    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True)

    # 1. Ángulo pitch (0° a -360°)
    axes[0].plot(t, np.rad2deg(th), "darkmagenta", lw=2.5, label=r"Angulo Pitch $\theta(t)$ [deg]")
    axes[0].axhline(0.0, color="gray", ls="--", lw=1)
    axes[0].axhline(-180.0, color="orange", ls=":", lw=1.2, label=r"Inversion Aerea (-180$^\circ$)")
    axes[0].axhline(-360.0, color="green", ls="--", lw=1.2, label=r"Rotacion Completa (-360$^\circ$)")
    axes[0].set_ylabel(r"Pitch $\theta$ [deg]")
    axes[0].set_title("Dinamica de Rotacion 360 y Trayectoria Espacial del Backflip Acrobatico")
    axes[0].legend(loc="lower left", framealpha=0.9)
    axes[0].grid(True)
    axes[0].set_ylim(-400, 40)

    # 2. Velocidad angular
    axes[1].plot(t, om, "r-", lw=2, label=r"$\omega_y(t)$ [rad/s]")
    axes[1].plot(t, np.rad2deg(om) / 10.0, "r:", lw=1.2, label=r"$\omega_y / 10$ [deg/s]")
    axes[1].set_ylabel(r"Vel. Angular [rad/s]")
    axes[1].legend(loc="upper right", framealpha=0.9)
    axes[1].grid(True)
    axes[1].set_ylim(-12.0, 3.0)

    # 3. Altura y avance CoM
    axes[2].plot(t, pz, "b-", lw=2.2, label="Altura CoM $p_z(t)$ [m]")
    axes[2].plot(t, px, "c--", lw=1.8, label="Posicion Long. $p_x(t)$ [m]")
    k_apex = N1 + N2 + N3 // 2
    axes[2].scatter([t[k_apex]], [pz[k_apex]], color="blue", s=50, zorder=5)
    axes[2].annotate(f"Vertice: z = {pz[k_apex]:.2f} m\n(Giro = {np.rad2deg(th[k_apex]):.0f} deg)",
                     xy=(t[k_apex], pz[k_apex]), xytext=(t[k_apex]+0.04, pz[k_apex]+0.07),
                     arrowprops=dict(arrowstyle="->", lw=1.2, color="blue"),
                     bbox=dict(boxstyle="round,pad=0.2", fc="#EBF5FB", ec="blue"))
    axes[2].set_ylabel("CoM [m]")
    axes[2].legend(loc="upper right", framealpha=0.9)
    axes[2].grid(True)
    axes[2].set_ylim(-0.15, 1.15)

    # 4. Inercia variable Iyy(t) (Efecto Tucking)
    axes[3].plot(t[:-1], Iyy_profile * 1000.0, "teal", lw=2.2, label=r"Momento Inercia $I_{yy}(t)$ [$g\cdot m^2$]")
    axes[3].annotate("Patas Recogidas (Tuck)\nReduccion Inercia 43%", xy=(0.5*(t_takeoff + t_flight_end), 32.0),
                     xytext=(0.5*(t_takeoff + t_flight_end)-0.12, 45.0),
                     arrowprops=dict(arrowstyle="->", lw=1.2, color="teal"),
                     bbox=dict(boxstyle="round,pad=0.2", fc="#E8F8F5", ec="teal"))
    axes[3].set_ylabel(r"$I_{yy}$ [$g\cdot m^2$]")
    axes[3].set_xlabel("Tiempo [s]")
    axes[3].legend(loc="lower left", framealpha=0.9)
    axes[3].grid(True)
    axes[3].set_ylim(25, 68)

    for ax in axes:
        for t_phase in [t_crouch, t_takeoff, t_flight_end]:
            ax.axvline(t_phase, color="gray", ls=":", lw=1.2)

    fig.tight_layout()
    p2_rot_path = os.path.join(output_dir, "p2_evolucion_backflip.png")
    fig.savefig(p2_rot_path, dpi=200)
    plt.close(fig)
    print(f">>> Grafica de rotacion backflip guardada en: {p2_rot_path}")

    # =========================================================================
    # FIGURA 2: FUERZAS DE REACCIÓN (GRF) Y TORQUE DE CABECEO
    # =========================================================================
    t_steps = t[:-1]
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 9), sharex=True)

    # Fuerzas normales verticales
    ax1.plot(t_steps, fz_F, "b-", lw=2, label="F_z Frontal (FL+FR)")
    ax1.plot(t_steps, fz_R, "m-", lw=2, label="F_z Trasero (RL+RR)")
    ax1.plot(t_steps, fz_F + fz_R, "k--", lw=1.5, label="F_z Total")
    ax1.set_ylabel("Fuerza Normal [N]")
    ax1.set_title("Fuerzas de Reaccion del Suelo (GRF) e Impulso Angular en Despegue")
    ax1.legend(loc="upper right", ncol=3, framealpha=0.9)
    ax1.grid(True)
    ax1.set_ylim(-20, 520)

    # Fuerzas tangenciales
    mu = 0.8
    ax2.plot(t_steps, fx_F, "b-", lw=2, label="F_x Frontal")
    ax2.plot(t_steps, fx_R, "m-", lw=2, label="F_x Trasero")
    ax2.plot(t_steps, mu * fz_F, "b:", lw=1.2, label=r"Cono Friccion $\pm\mu F_z$")
    ax2.plot(t_steps, -mu * fz_F, "b:", lw=1.2)
    ax2.plot(t_steps, mu * fz_R, "m:", lw=1.2)
    ax2.plot(t_steps, -mu * fz_R, "m:", lw=1.2)
    ax2.set_ylabel("Fuerza Tangencial [N]")
    ax2.legend(loc="upper right", ncol=3, framealpha=0.9)
    ax2.grid(True)
    ax2.set_ylim(-220, 220)

    # Torque neto de cabeceo tau_y
    d_x = 0.1805
    tau_y = np.zeros_like(t_steps)
    for k in range(len(t_steps)):
        if k < N1 + N2:
            r_xF, r_zF = d_x, 0.0
            r_xR, r_zR = -d_x, 0.0
        elif k < N1 + N2 + N3:
            r_xF, r_zF = px[k] + d_x*np.cos(th[k]), pz[k] + d_x*np.sin(th[k])
            r_xR, r_zR = px[k] - d_x*np.cos(th[k]), pz[k] - d_x*np.sin(th[k])
        else:
            r_xF, r_zF = data["landing"]["rx_land_F"], 0.0
            r_xR, r_zR = data["landing"]["rx_land_R"], 0.0
        tau_F = (r_xF - px[k]) * fz_F[k] - (r_zF - pz[k]) * fx_F[k]
        tau_R = (r_xR - px[k]) * fz_R[k] - (r_zR - pz[k]) * fx_R[k]
        tau_y[k] = tau_F + tau_R

    ax3.plot(t_steps, tau_y, "darkred", lw=2.2, label=r"Torque Neto Cabeceo $\tau_y(t)$ [N$\cdot$m]")
    ax3.annotate("Impulso Angular Despegue\n(Empuje Asimetrico Frontal/Trasero)", xy=(0.20, np.min(tau_y)),
                 xytext=(0.28, np.min(tau_y) + 7.0),
                 arrowprops=dict(arrowstyle="->", lw=1.2, color="darkred"),
                 bbox=dict(boxstyle="round,pad=0.2", fc="#FDEDEC", ec="darkred"))
    ax3.set_xlabel("Tiempo [s]")
    ax3.set_ylabel(r"Torque $\tau_y$ [N$\cdot$m]")
    ax3.legend(loc="upper right", framealpha=0.9)
    ax3.grid(True)
    ax3.set_ylim(-35, 15)

    for ax in [ax1, ax2, ax3]:
        for t_phase in [t_crouch, t_takeoff, t_flight_end]:
            ax.axvline(t_phase, color="gray", ls=":", lw=1.2)

    fig.tight_layout()
    p2_forces_path = os.path.join(output_dir, "p2_fuerzas_backflip.png")
    fig.savefig(p2_forces_path, dpi=200)
    plt.close(fig)
    print(f">>> Grafica de fuerzas backflip guardada en: {p2_forces_path}")

    # =========================================================================
    # FIGURA 3: TRAYECTORIAS ARTICULARES DE LAS 4 PATAS DURANTE EL BACKFLIP
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
    axes[0].set_title("Evolucion Articular durante el Backflip 360 (Unitree A1)")
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
    p2_joints_path = os.path.join(output_dir, "p2_articulaciones_backflip.png")
    fig.savefig(p2_joints_path, dpi=200)
    plt.close(fig)
    print(f">>> Grafica de articulaciones backflip guardada en: {p2_joints_path}")


if __name__ == "__main__":
    generate_plots()
