# EC3 Parte 1: Optimización de Trayectorias en Robótica Cuadrúpeda
### Curso: Robótica Avanzada — Universidad de Ingeniería y Tecnología (UTEC)

Este repositorio contiene la solución completa de la evaluación continua sobre **Optimización de Trayectorias** aplicada al robot cuadrúpedo **Unitree A1**, utilizando el Modelo de Cuerpo Rígido Único (**Single Rigid Body Model - SRBM**), optimización no lineal por colocación directa con **CasADi** y el solucionador **IPOPT**, cinemática inversa analítica de 12 GDL y simulación física en **PyBullet**.

---

## 1. Estructura del Repositorio

```text
EC3_PT1_RA/
├── README.md                      <- Guía de instalación, explicación teórica y ejecución
├── REPORTE.md                     <- Reporte técnico exhaustivo con formulación matemática y resultados
├── reporte.tex                    <- Documento fuente LaTeX del reporte académico
├── reporte.pdf                    <- Reporte formal compilado en PDF (12 páginas con figuras)
├── requirements.txt               <- Dependencias de Python para cualquier sistema operativo
├── part1_salto_obstaculo/         <- PROBLEMA 1: Salto dinámico sobre obstáculo alto
│   ├── modelo_srbm.py             <- Modelo dinámico SRBM y cinemática analítica directa/inversa
│   ├── opt_salto_obstaculo.py     <- Optimización de trayectoria por fases (CasADi + IPOPT)
│   ├── sim_salto_pybullet.py      <- Simulación física 3D en PyBullet y renderizado de medios
│   └── graficar_resultados_p1.py  <- Gráficas analíticas (plano sagital, fuerzas GRF, articulaciones)
├── part2_movimiento_complejo/     <- PROBLEMA 2: Salto mortal acrobático 360° (Backflip)
│   ├── opt_backflip.py            <- Optimización no lineal con rotación completa y tucking
│   ├── sim_backflip_pybullet.py   <- Simulación física 3D en PyBullet y renderizado de medios
│   └── graficar_resultados_p2.py  <- Gráficas analíticas (ángulo pitch, velocidad angular, fuerzas)
├── scripts/
│   └── run_all.py                 <- Script maestro para ejecutar todas las actividades en secuencia
└── outputs/                       <- Medios generados (figuras PNG, animaciones GIF y videos MP4)
    ├── p1_articulaciones.png
    ├── p1_capturas_secuencia.png
    ├── p1_fuerzas_grf.png
    ├── p1_salto_obstaculo.gif
    ├── p1_salto_obstaculo.mp4
    ├── p1_trayectoria_salto.png
    ├── p2_articulaciones_backflip.png
    ├── p2_backflip_360.gif
    ├── p2_backflip_360.mp4
    ├── p2_capturas_secuencia.png
    ├── p2_evolucion_backflip.png
    └── p2_fuerzas_backflip.png
```

---

## 2. Requisitos e Instalación

El código es multiplataforma y puede ejecutarse en cualquier computadora con Linux, macOS o Windows (WSL / nativo con Python 3.8 a 3.12).

### Opción A: Entorno Virtual Estándar (Cualquier PC)

1. **Clonar el repositorio:**
   ```bash
   git clone git@github.com:Nvm230/EC3_PT1_RA.git
   cd EC3_PT1_RA
   ```

2. **Crear y activar un entorno virtual de Python:**
   ```bash
   # En Linux / macOS:
   python3 -m venv venv
   source venv/bin/activate

   # En Windows (CMD o PowerShell):
   python -m venv venv
   venv\Scripts\activate
   ```

3. **Instalar dependencias:**
   ```bash
   pip install --upgrade pip
   pip install -r requirements.txt
   ```

### Opción B: Mediante Contenedor Distrobox / Podman / Docker (UTEC)

Si se utiliza el contenedor `ros2_humble` configurado en el sistema:
```bash
distrobox enter ros2_humble
cd /home/nvm/Documents/UTEC/EC3_pt1_Avanzada
```

O directamente sin sesión interactiva:
```bash
podman exec -u nvm ros2_humble python3 /home/nvm/Documents/UTEC/EC3_pt1_Avanzada/scripts/run_all.py
```

---

## 3. Instrucciones de Ejecución

### Opción 1: Ejecución Completa Automatizada (Recomendada)

El script maestro `scripts/run_all.py` ejecuta en secuencia:
1. Optimización del salto sobre obstáculo (Parte 1).
2. Simulación física en PyBullet y exportación de GIF/MP4 (Parte 1).
3. Generación de curvas analíticas de fuerzas, trayectoria y motores (Parte 1).
4. Optimización no lineal del backflip acrobático 360° (Parte 2).
5. Simulación física en PyBullet y exportación de GIF/MP4 (Parte 2).
6. Generación de curvas analíticas del backflip (Parte 2).

```bash
# Modo rápido headless (sin interfaz gráfica, apto para servidores y laptops):
python3 scripts/run_all.py

# Con ventana visual interactiva de PyBullet:
python3 scripts/run_all.py --gui
```

Tiempo estimado de ejecución: **16 segundos** en total.

---

### Opción 2: Ejecución Individual Paso a Paso

#### PARTE 1: Salto sobre Obstáculo Alto

1. **Resolver la optimización de trayectoria con CasADi + IPOPT:**
   ```bash
   python3 part1_salto_obstaculo/opt_salto_obstaculo.py
   ```
   *Resuelve el problema NLP en ~0.09 s y genera `trayectoria_salto_opt.npy`.*

2. **Ejecutar la simulación física en PyBullet y generar GIF/MP4:**
   ```bash
   # Modo headless:
   python3 part1_salto_obstaculo/sim_salto_pybullet.py

   # Con interfaz gráfica:
   python3 part1_salto_obstaculo/sim_salto_pybullet.py --gui
   ```
   *Verifica colisiones físicas paso a paso (contactos = 0) y exporta `p1_salto_obstaculo.gif` y `p1_salto_obstaculo.mp4`.*

3. **Generar figuras analíticas:**
   ```bash
   python3 part1_salto_obstaculo/graficar_resultados_p1.py
   ```

---

#### PARTE 2: Salto Mortal Acrobático 360° (Backflip)

1. **Resolver la optimización no lineal del giro 360° con CasADi + IPOPT:**
   ```bash
   python3 part2_movimiento_complejo/opt_backflip.py
   ```
   *Resuelve la rotación continua con inercia variable en ~0.13 s y genera `trayectoria_backflip_opt.npy`.*

2. **Ejecutar la simulación en PyBullet y generar GIF/MP4:**
   ```bash
   # Modo headless:
   python3 part2_movimiento_complejo/sim_backflip_pybullet.py

   # Con interfaz gráfica:
   python3 part2_movimiento_complejo/sim_backflip_pybullet.py --gui
   ```

3. **Generar figuras analíticas del Backflip:**
   ```bash
   python3 part2_movimiento_complejo/graficar_resultados_p2.py
   ```

---

## 4. Compilación del Reporte Académico en PDF

El documento formal de 12 páginas con toda la formulación matemática, figuras y tablas se encuentra en `reporte.tex`. Para compilarlo desde LaTeX:

```bash
pdflatex -interaction=nonstopmode reporte.tex
pdflatex -interaction=nonstopmode reporte.tex
```

El documento resultante es `reporte.pdf`.

---

## 5. Resumen Físico y Resultados Obtenidos

### Problema 1: Salto sobre Obstáculo Alto
- **Obstáculo:** Alto $h = 0.26\text{ m}$ ($26\text{ cm}$), ancho $12\text{ cm}$ en $x \in [0.52, 0.64]\text{ m}$.
- **Reto físico:** La altura nominal del robot es de $0.28 - 0.30\text{ m}$ y el vientre libre está a solo $0.18 - 0.20\text{ m}$. La zancada de paso en marcha no supera los $0.06 - 0.08\text{ m}$. Por tanto, un obstáculo de $26\text{ cm}$ colisiona frontalmente contra el vientre y las patas, imposibilitando cruzarlo a pie. Es obligatorio ejecutar un salto balístico.
- **Resultado de la optimización:**
  - Despegue CoM: $x = 0.121\text{ m}$, $v_z = 2.66\text{ m/s}$, $v_x = 1.71\text{ m/s}$.
  - Altura máxima CoM: **$0.665\text{ m}$** ($+40.5\text{ cm}$ de margen sobre la barrera).
  - Altura de patas durante el cruce: **$z \ge 0.515\text{ m}$** ($+25.5\text{ cm}$ de holgura limpia).
  - Aterrizaje suave en $x = 0.992\text{ m}$ ($+35.2\text{ cm}$ post-obstáculo).
  - **Cero colisiones físicas:** Monitoreo activo en PyBullet registró `Total de contactos con obstáculo = 0`.
  - Tiempo de resolución IPOPT: **0.090 s**.

### Problema 2: Salto Mortal Acrobático 360° (Backflip)
- **Reto físico:** Maniobra no lineal extrema con giro continuo de $\Delta \theta = -360.0^\circ$ ($-2\pi\text{ rad}$) en cabeceo.
- **Mecanismo:** Despegue con par asimétrico $\tau_y(t)$, conservación de momento angular $L_y = I_{yy} \omega_y$ en vuelo y modulación de inercia mediante encogimiento de patas (*tucking*), reduciendo $I_{yy}$ en un $43.4\%$ para acelerar la rotación y extendiéndolas antes del aterrizaje para amortiguar el impacto.
- **Simetría sagital perfecta:** La rotación se completa casi en su totalidad en el aire (touchdown a $-335^\circ$), permitiendo que las patas toquen el suelo orientadas hacia abajo, eliminando desviaciones laterales ($q_1 \equiv 0.00^\circ$ en todas las patas).
- **Resultado de la optimización:**
  - Giro final completado: **$-360.0^\circ$**.
  - Velocidad angular pico: **$-9.51\text{ rad/s}$** ($-545.1^\circ/\text{s}$).
  - Altura máxima: **$0.845\text{ m}$** con inversión a $-187.2^\circ$ en cumbre.
  - Tiempo de resolución IPOPT: **0.136 s**.

### Exportación Cinematográfica
Ambas animaciones fueron exportadas en cámara lenta cinematográfica (80 y 85 fotogramas a 25 FPS, ~3.3 segundos) acompañadas de una pausa de 0.6 segundos en reposo al aterrizar, asegurando una visualización nítida y elegante de cada etapa del movimiento.
