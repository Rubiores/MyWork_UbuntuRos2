# MyWork_UbuntuRos2

[![Lenguaje C++](https://img.shields.io/badge/C++-17-blue)](#)
[![Lenguaje Python](https://img.shields.io/badge/Python-3.8+-yellow?logo=python)](#)
[![Sistema Operativo](https://img.shields.io/badge/Ubuntu-22.04-E95420?logo=ubuntu)](#)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](#)
[![SLAM Toolbox](https://img.shields.io/badge/SLAM-Toolbox-green)](#)
[![AMCL](https://img.shields.io/badge/Localización-AMCL-blueviolet)](#)
[![Nav2](https://img.shields.io/badge/Nav2-Planeado-lightgrey)](#)
[![CMake](https://img.shields.io/badge/CMake-3.16+-064F8C?logo=cmake)](#)
[![Colcon](https://img.shields.io/badge/Build-Colcon-22314E)](#)
[![Git](https://img.shields.io/badge/Git-2.34+-F05032?logo=git)](#)
[![VS Code](https://img.shields.io/badge/IDE-VS%20Code-007ACC?logo=visualstudiocode)](#)
[![Shell](https://img.shields.io/badge/Shell-Bash-4EAA25?logo=gnubash)](#)


# Robot Móvil con ROS 2, LiDAR A1 y Navegación

## Descripción General del Proyecto

Este repositorio contiene el desarrollo de un robot móvil diferencial con ROS 2 y un LiDAR A1. El paquete principal está dentro de la carpeta:

```
src/robot_nav
```

Capaz de:

* Realizar SLAM para construir un mapa en tiempo real

* Localizarse dentro de un mapa previamente generado

* Integrar un control PID ejecutado en microcontrolador (Raspberry Pi Pico)

* Visualizar todos los datos del sistema en RViz2

El proyecto nace como parte del PAE – Desarrollo de un robot móvil capaz de navegar en entornos dinámicos y constituye la base del Trabajo de Grado (TDG) donde se extenderá a navegación inteligente.

## Objetivos del Proyecto
**Objetivos logrados**

* Implementación completa del stack de ROS 2 para un robot diferencial.

* Integración física del robot: estructura, motores, encoders, LiDAR y Raspberry Pi.

* Publicación de odometría por diseño propio del nodo puente (pico_bridge).

* Visualización del LiDAR A1 en RViz2.

* SLAM funcional utilizando paquetes estándar de ROS 2.

* Localización mediante AMCL configurado acorde a la cinemática del robot.

* Corrección y estabilización del control del robot (PID en Raspberry Pi Pico).

* Estructura de trabajo clara: slam_launch, localization_launch, nav_launch.

**Trabajo futuro**

* Mejorar la navegación en ambientes dinámicos, ajustando parámetros del planner local.

* Implementación de técnicas básicas de IA para ajustar dinámicamente parámetros de navegación.

* Integrar una capa de decisiones basada en fuzzy logic (planeado para el TDG).

* Crear nodos para control avanzado y navegación.

 
## Launch Files Principales
### 🗺️ 1. SLAM Launch

**Archivo**: slam_launch.py

* Inicializa el nodo del LiDAR.

* Ejecuta SLAM Toolbox.

* Genera el mapa en tiempo real.

Uso:
```md
ros2 launch robot_nav slam_launch.py
```

### 📍 2. Localization Launch

Archivo: localization_launch.py

Carga un mapa previamente generado.

Ejecuta AMCL para localización.

Uso:
```md
ros2 launch robot_nav localization_launch.py
```

### 🧭 3. Navigation Launch (Preparando para TDG)

```md
ros2 launch robot_nav nav_launch.py
```

⚠️ Este launch está documentado y organizado, pero la navegación aún no está correctamente implementada.

Será usado para Nav2 una vez los módulos y parámetros estén definidos.

Está previsto que sea funcional para la fase inicial del TDG.


## 🧪 Resultados y Evidencias
### 🗺️ Mapa generado con SLAM

<p align="center"> <img src="images/slam.png" alt="Mapa generado con SLAM" width="70%"> </p>

### 📍 Localización del robot con AMCL

<p align="center"> <img src="images/localization.jpeg" alt="Localización con AMCL" width="70%"> </p>

## 🔧 Instalación y Uso
### 1️⃣ Clonar el repositorio
```md
git clone https://github.com/Rubiores/MyWork_UbuntuRos2.git
```
### 2️⃣ Compilar el workspace
```md
cd ~/MyWork_UbuntuRos2
colcon build
source install/setup.bash
```
### 3️⃣ Ejecutar SLAM o Localización

```md
ros2 launch robot_nav slam_launch.py
ros2 launch robot_nav localization_launch.py
```


