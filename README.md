# EC3 Parte 1: Optimización de Trayectorias en Robótica Cuadrúpeda
### Curso: Robótica Avanzada — Universidad de Ingeniería y Tecnología (UTEC)

Este repositorio contiene la solución completa de la evaluación continua sobre **Optimización de Trayectorias** aplicada al robot cuadrúpedo **Unitree A1**, utilizando el Modelo de Cuerpo Rígido Único (**Single Rigid Body Model - SRBM**), optimización no lineal por colocación directa con **CasADi** y el solver **IPOPT**, cinemática inversa analítica de 12 GDL y simulación física en **PyBullet**.

---

## 1. Estructura del Repositorio

```text
EC3_pt1_Avanzada/
├── README.md                      <- Guía de instalación, explicación teórica y ejecución
├── REPORTE.md                     <- Reporte técnico exhaustivo con formulación matemática y resultados
├── reporte.tex                    <- Documento fuente LaTeX del reporte académico
├── reporte.pdf                    <- Reporte formal compilado en PDF (11 páginas con figuras de alta resolución)
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

## 2. Requisitos y Entorno de Ejecución

El proyecto está diseñado para ejecutarse dentro del contenedor Distrobox `ros2_humble` configurado en el sistema, el cual cuenta con:
- **Python 3.10**
- **CasADi 3.8.2** + **IPOPT 3.14.19** con solucionador lineal MUMPS.
- **PyBullet 3.2.7**
- **Robot Descriptions 3.3.0** (URDF oficial de Unitree A1).
- **NumPy**, **SciPy**, **Matplotlib**, **ImageIO** y **Pillow**.

### Ingreso al contenedor:
```bash
distrobox enter ros2_humble
```

---

## 3. Instrucciones de Ejecución

### Opción 1: Ejecución Completa Automatizada (Recomendada)
Para correr de manera secuencial la optimización, simulaciones en PyBullet, generación de animaciones (GIF / MP4) y gráficas de ambas partes:

```bash
# Dentro de distrobox:
cd /home/nvm/Documents/UTEC/EC3_pt1_Avanzada
python3 scripts/run_all.py

# O con visualización en ventana gráfica de PyBullet:
python3 scripts/run_all.py --gui
```

Desde el sistema host sin entrar interactivamente:
```bash
podman exec -u nvm ros2_humble python3 /home/nvm/Documents/UTEC/EC3_pt1_Avanzada/scripts/run_all.py
```

---

### Opción 2: Ejecución Individual Paso a Paso

#### PARTE 1: Salto sobre Obstáculo Alto

1. **Resolver la optimización de trayectoria del salto:**
   ```bash
   python3 part1_salto_obstaculo/opt_salto_obstaculo.py
   ```
   *Resuelve el problema NLP con IPOPT en ~0.08 s y guarda `trayectoria_salto_opt.npy`.*

2. **Ejecutar la simulación en PyBullet y generar GIF/MP4:**
   ```bash
   # Modo headless rápido:
   python3 part1_salto_obstaculo/sim_salto_pybullet.py
   
   # Con ventana GUI interactiva:
   python3 part1_salto_obstaculo/sim_salto_pybullet.py --gui
   ```

3. **Generar las figuras analíticas de resultados:**
   ```bash
   python3 part1_salto_obstaculo/graficar_resultados_p1.py
   ```

---

#### PARTE 2: Movimiento Complejo (Backflip Acrobático 360°)

1. **Resolver la optimización de trayectoria del Backflip:**
   ```bash
   python3 part2_movimiento_complejo/opt_backflip.py
   ```
   *Resuelve la rotación no lineal de 360° en ~0.12 s y guarda `trayectoria_backflip_opt.npy`.*

2. **Ejecutar la simulación en PyBullet y generar GIF/MP4:**
   ```bash
   # Modo headless:
   python3 part2_movimiento_complejo/sim_backflip_pybullet.py
   
   # Con ventana GUI:
   python3 part2_movimiento_complejo/sim_backflip_pybullet.py --gui
   ```

3. **Generar las figuras analíticas del Backflip:**
   ```bash
   python3 part2_movimiento_complejo/graficar_resultados_p2.py
   ```

---

## 4. Compilación del Reporte en PDF

El reporte formal se encuentra disponible en formato LaTeX y se compila directamente con:
```bash
cd /home/nvm/Documents/UTEC/EC3_pt1_Avanzada
pdflatex -interaction=nonstopmode reporte.tex
pdflatex -interaction=nonstopmode reporte.tex
```
El documento resultante es `reporte.pdf`.

---

## 5. Resumen Teórico y Resultados Obtenidos

### Problema 1: Salto Dinámico sobre Obstáculo Alto
- **Obstáculo:** Alto $h = 0.35\text{ m}$, ancho $30\text{ cm}$ en $x \in [0.35, 0.65]\text{ m}$.
- **Reto físico:** La altura nominal del A1 es de $0.30\text{ m}$, por lo que el obstáculo supera la altura libre del chasis. No es posible caminarlo; se requiere un salto balístico completo con fase de vuelo y despegue a $F_z \approx 2.4\text{ g}$.
- **Resultado óptimo:**
  - Altura máxima CoM: **$0.686\text{ m}$** ($+33.6\text{ cm}$ sobre la barrera).
  - Altura mínima de patas en cruce: **$0.408\text{ m}$ a $0.577\text{ m}$** (margen de seguridad $> 5.8\text{ cm}$).
  - Aterrizaje suave en $x = 0.880\text{ m}$ con velocidad residual nula.
  - Tiempo de resolución IPOPT: **0.077 s**.

### Problema 2: Salto Mortal Acrobático 360° (Backflip)
- **Reto físico:** Maniobra no lineal extrema con giro continuo de $\Delta \theta = -360.0^\circ$ ($-2\pi\text{ rad}$) en cabeceo.
- **Mecanismo:** Despegue con par asimétrico $\tau_y(t)$, conservación de momento angular $L_y = I_{yy} \omega_y$ en vuelo y modulación de inercia mediante encogimiento de patas (*tucking*), reduciendo $I_{yy}$ en un $43.4\%$ para acelerar la rotación y extendiéndolas antes del aterrizaje para amortiguar el impacto.
- **Resultado óptimo:**
  - Giro final completado: **$-360.0^\circ$**.
  - Velocidad angular pico: **$-9.11\text{ rad/s}$** ($-521.9^\circ/\text{s}$).
  - Altura máxima: **$0.717\text{ m}$** con inversión a $-180^\circ$ en cumbre.
  - Tiempo de resolución IPOPT: **0.124 s**.

---

## 6. Publicación en GitHub

Para subir el repositorio completo a GitHub:
```bash
cd /home/nvm/Documents/UTEC/EC3_pt1_Avanzada

# Configurar credenciales (si no están configuradas)
git config user.name "TuNombre"
git config user.email "tu_correo@utec.edu.pe"

# Agregar archivos y hacer commit inicial
git add .
git commit -m "Solución completa EC3: Optimización de trayectorias (Salto obstáculo + Backflip 360)"

# Vincular con tu repositorio remoto de GitHub:
git remote add origin https://github.com/<TU_USUARIO>/EC3_pt1_Avanzada.git
git branch -M main
git push -u origin main
```
