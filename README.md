# Terrabots_GUI

Panel de control y monitoreo de robots usando ROS2 y PySide6.

---

## Requisitos

- ROS2 Jazzy (desktop full) instalado
- Python 3.12+
- Colcon y herramientas básicas de ROS2

---

## Instalación

0. **Instalar dependencias necesarias**

```bash
pip3 install --no-cache-dir --break-system-packages PySide6==6.7.2
```

1. **Crear un workspace de ROS2**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. **Clonar el repositorio dentro**
   
```bash
git clone https://github.com/sergio100899/Terrabots_GUI.git
```

3. **Construir y activar el workspace**

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```
---

## Uso

```bash
ros2 run terrabots_gui gui_app
```

<img width="1337" height="835" alt="image" src="https://github.com/user-attachments/assets/843f0cbd-4abf-4af5-94b3-f665bac273ec" />

