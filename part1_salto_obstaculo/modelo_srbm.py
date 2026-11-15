#!/usr/bin/env python3
"""
===============================================================================
MODELO DINÁMICO SRBM Y CINEMÁTICA ANALÍTICA DE CUADRÚPEDO (UNITREE A1)
===============================================================================
Contiene:
  - Parámetros físicos del robot cuadrúpedo Unitree A1.
  - Modelo dinámico de cuerpo rígido único (Single Rigid Body Model - SRBM).
  - Cinemática directa (FK) e inversa (IK) analítica cerrada de las 4 patas.
"""

import numpy as np

class A1SRBMModel:
    """Parámetros dinámicos y geométricos de Unitree A1."""
    def __init__(self):
        # Masa total y tensor de inercia
        self.mass = 12.0  # kg
        self.Ixx = 0.0168
        self.Iyy = 0.0565  # Inercia de cabeceo (pitch)
        self.Izz = 0.0640
        self.gravity = 9.81  # m/s^2

        # Geometría de cadera respecto al centro de masa (CoM)
        self.d_x = 0.1805   # mitad de distancia longitudinal caderas
        self.d_y = 0.0470   # mitad de ancho lateral de caderas
        self.l_thigh_lat = 0.0838  # offset lateral muslo
        self.l_thigh = 0.2000      # longitud muslo
        self.l_calf = 0.2000       # longitud pantorrilla

        # Signos de posición de caderas [FL, FR, RL, RR]
        # FL: (+1, +1), FR: (+1, -1), RL: (-1, +1), RR: (-1, -1)
        self.leg_signs = [
            (+1.0, +1.0),  # FL (0)
            (+1.0, -1.0),  # FR (1)
            (-1.0, +1.0),  # RL (2)
            (-1.0, -1.0),  # RR (3)
        ]
        self.leg_names = ["FL", "FR", "RL", "RR"]

        # Límites cinemáticos
        self.max_leg_length = 0.38  # m
        self.min_leg_length = 0.12  # m
        self.nominal_height = 0.30  # m
        self.crouch_height = 0.18   # m

        # Límites de fuerza y fricción
        self.mu = 0.6
        self.f_max_per_leg = 160.0  # N por pata (~640 N total, ~5.4 g)

    def get_hip_position_body(self, leg_idx):
        """Posición 3D de la cadera en el marco local del cuerpo."""
        sx, sy = self.leg_signs[leg_idx]
        return np.array([sx * self.d_x, sy * self.d_y, 0.0], dtype=np.float64)

    def fk_leg(self, leg_idx, q_leg):
        """
        Cinemática directa exacta de una pata respecto al marco de la base.
        q_leg = [q_hip_roll, q_thigh_pitch, q_calf_pitch]
        """
        sx, sy = self.leg_signs[leg_idx]
        p_hip = np.array([sx * self.d_x, sy * self.d_y, 0.0], dtype=np.float64)
        
        q1, q2, q3 = q_leg[0], q_leg[1], q_leg[2]
        c1, s1 = np.cos(q1), np.sin(q1)
        R1 = np.array([[1.0, 0.0, 0.0], [0.0, c1, -s1], [0.0, s1, c1]], dtype=np.float64)
        
        p_thigh = p_hip + R1 @ np.array([0.0, sy * self.l_thigh_lat, 0.0], dtype=np.float64)
        
        c2, s2 = np.cos(q2), np.sin(q2)
        R2 = R1 @ np.array([[c2, 0.0, s2], [0.0, 1.0, 0.0], [-s2, 0.0, c2]], dtype=np.float64)
        p_calf = p_thigh + R2 @ np.array([0.0, 0.0, -self.l_thigh], dtype=np.float64)
        
        c3, s3 = np.cos(q3), np.sin(q3)
        R3 = R2 @ np.array([[c3, 0.0, s3], [0.0, 1.0, 0.0], [-s3, 0.0, c3]], dtype=np.float64)
        p_foot = p_calf + R3 @ np.array([0.0, 0.0, -self.l_calf], dtype=np.float64)
        
        return p_foot

    def ik_leg(self, leg_idx, p_foot_b):
        """
        Cinemática inversa analítica exacta de una pata respecto a la base.
        p_foot_b: [x, y, z] en el marco base.
        Retorna: [q1, q2, q3]
        """
        sx, sy = self.leg_signs[leg_idx]
        p_hip = np.array([sx * self.d_x, sy * self.d_y, 0.0], dtype=np.float64)
        dp = p_foot_b - p_hip

        dy, dz = dp[1], dp[2]
        L_yz_sq = dy**2 + dz**2
        l_lat = sy * self.l_thigh_lat

        d_proj_sq = max(0.0, L_yz_sq - self.l_thigh_lat**2)
        d_proj = np.sqrt(d_proj_sq)

        q1 = np.arctan2(dy, -dz) - np.arctan2(l_lat, d_proj)

        # Rotar vector al marco intermedio tras q1
        c1, s1 = np.cos(q1), np.sin(q1)
        R1 = np.array([[1.0, 0.0, 0.0], [0.0, c1, -s1], [0.0, s1, c1]], dtype=np.float64)
        p_thigh = p_hip + R1 @ np.array([0.0, sy * self.l_thigh_lat, 0.0], dtype=np.float64)
        r_thigh = R1.T @ (p_foot_b - p_thigh)

        x_leg, z_leg = r_thigh[0], r_thigh[2]
        L_xz_sq = x_leg**2 + z_leg**2

        cos_q3 = (L_xz_sq - self.l_thigh**2 - self.l_calf**2) / (2.0 * self.l_thigh * self.l_calf)
        cos_q3 = np.clip(cos_q3, -1.0, 1.0)
        q3 = -np.arccos(cos_q3)  # rodilla flexionada hacia atrás

        sin_q3 = np.sin(q3)
        k1 = self.l_thigh + self.l_calf * cos_q3
        k2 = self.l_calf * sin_q3
        q2 = np.arctan2(x_leg, -z_leg) - np.arctan2(k2, k1)

        return np.array([q1, q2, q3], dtype=np.float64)

    def ik_full_body(self, p_base, R_base, p_feet_world):
        """
        Calcula los 12 ángulos articulares a partir de la pose de la base
        y las posiciones cartesianas en el mundo de los 4 pies.
        p_base: (3,)
        R_base: (3, 3)
        p_feet_world: (4, 3)
        """
        q_full = np.zeros(12, dtype=np.float64)
        for i in range(4):
            # Transformar pie del marco mundo al marco base: p_b = R_base^T * (p_w - p_base)
            p_foot_b = R_base.T @ (p_feet_world[i] - p_base)
            q_leg = self.ik_leg(i, p_foot_b)
            q_full[3*i : 3*i + 3] = q_leg
        return q_full
