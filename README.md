# MyWork_UbuntuRos2

# Robot Móvil con ROS 2, LiDAR A1 y Navegación Autónoma

## Descripción General del Proyecto

Este proyecto desarrolla un robot móvil autónomo basado en ROS 2 y un sensor LiDAR A1, capaz de:

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

ros2 launch <paquete> localization_launch.py map:=/ruta/mapa.yaml

🧭 3. Navigation Launch (Preparado para TDG)

Archivo: nav_launch.py

⚠️ Este launch está documentado y organizado, pero la navegación aún no está implementada.

Será usado para Nav2 una vez los módulos y parámetros estén definidos.

Está previsto para la fase inicial del TDG.

Uso (futuro):

ros2 launch <paquete> nav_launch.py


