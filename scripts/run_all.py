#!/usr/bin/env python3
"""
===============================================================================
SCRIPT MAESTRO DE EJECUCIÓN COMPLETA - EC3 ROBÓTICA AVANZADA
===============================================================================
Ejecuta de manera secuencial y automatizada:
  1. Optimización de trayectoria del salto sobre obstáculo (Parte 1).
  2. Simulación física y renderizado de video/GIF en PyBullet (Parte 1).
  3. Generación de gráficas analíticas del salto (Parte 1).
  4. Optimización no lineal del salto mortal 360° / Backflip (Parte 2).
  5. Simulación física y renderizado de video/GIF en PyBullet (Parte 2).
  6. Generación de gráficas analíticas del Backflip (Parte 2).
"""

import os
import sys
import time
import argparse
import subprocess

BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

def run_step(description, script_rel_path, extra_args=[]):
    script_path = os.path.join(BASE_DIR, script_rel_path)
    print("\n" + "="*80)
    print(f"PASO: {description}")
    print(f"Script: {script_path}")
    print("="*80)
    cmd = [sys.executable, script_path] + extra_args
    t0 = time.time()
    ret = subprocess.run(cmd)
    dt = time.time() - t0
    if ret.returncode != 0:
        print(f"ERROR: Falló la ejecución de {script_path} (código {ret.returncode})")
        sys.exit(ret.returncode)
    print(f"Completado exitosamente en {dt:.2f} s.")

def main():
    parser = argparse.ArgumentParser(description="Ejecución completa de actividades EC3")
    parser.add_argument("--gui", action="store_true", help="Abrir interfaz gráfica en simulaciones PyBullet")
    args = parser.parse_args()

    gui_flag = ["--gui"] if args.gui else []

    t_start = time.time()
    print("\n" + "#"*80)
    print("INICIANDO EJECUCIÓN TOTAL DE TAREAS - OPTIMIZACIÓN DE TRAYECTORIAS")
    print("#"*80)

    # PARTE 1
    run_step("Parte 1: Optimizacion de Salto sobre Obstaculo Alto (CasADi + IPOPT)",
             "part1_salto_obstaculo/opt_salto_obstaculo.py")
    run_step("Parte 1: Simulacion y Renderizado de Animacion PyBullet",
             "part1_salto_obstaculo/sim_salto_pybullet.py", gui_flag)
    run_step("Parte 1: Generacion de Graficas Analiticas de Resultados",
             "part1_salto_obstaculo/graficar_resultados_p1.py")

    # PARTE 2
    run_step("Parte 2: Optimizacion no lineal de Salto Mortal 360 (Backflip)",
             "part2_movimiento_complejo/opt_backflip.py")
    run_step("Parte 2: Simulacion y Renderizado de Animacion PyBullet",
             "part2_movimiento_complejo/sim_backflip_pybullet.py", gui_flag)
    run_step("Parte 2: Generacion de Graficas Analiticas del Backflip",
             "part2_movimiento_complejo/graficar_resultados_p2.py")

    t_total = time.time() - t_start
    print("\n" + "#"*80)
    print(f"TODAS LAS TAREAS SE COMPLETARON SATISFACTORIAMENTE EN {t_total:.2f} s")
    print(f"Archivos de salida disponibles en: {os.path.join(BASE_DIR, 'outputs')}")
    print("#"*80 + "\n")

if __name__ == "__main__":
    main()
