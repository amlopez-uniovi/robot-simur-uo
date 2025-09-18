
# Guía didáctica: Jerarquía de robots diferenciales y cómo usarla

> ![Diagrama UML de la jerarquía de robots](robot_hierarchy.png)
>
> El objetivo aprender, desde cero, a **usar** y **extender** la jerarquía.

---

## 1) Objetivos de aprendizaje
- Controlar un robot diferencial mediante una **API estable** (simulador u hardware).
- Entender la separación entre **interfaz** (qué) e **implementación** (cómo).
- Extender la jerarquía creando nuevas clases concretas.
- Escribir pruebas básicas para validar su implementación.

---

## 2) Mapa mental de capas (sin repetir el UML)

1. **`IRobotBase`**: capacidades mínimas (pose, comando general, parada, ciclo de control).
2. **`IDifferentialRobot`**: especialización para robots de **dos ruedas motrices** (comandos `(wl, wr)` y utilidades).
3. **Base de plataforma** (p. ej. `WebotsBaseDifferentialRobot`): inicialización y utilidades de **una plataforma** concreta.
4. **Clases concretas** (`RosbotRobot`, `EPuckRobot`): detalles del robot (motores, sensores, conversiones).

**Regla de oro:** programa contra **interfaces**; cambia de robot sin tocar tu lógica de control.

---


## 3) Inicio rápido (5 min)

### La demo `_skeleton demo`: tu plantilla para nuevos proyectos

La carpeta `webots_demos/_skeleton demo` es una **plantilla mínima y funcional** para crear tus propios experimentos o prácticas con robots diferenciales en Webots. Incluye:

- Un mundo `.wbt` listo para usar (con EPuck y RosBot)
- Controladores de ejemplo para ambos robots (`skeleton_controller.py`, `epuck_skeleton_controller.py`, `rosbot_skeleton_controller.py`)
- Estructura de carpetas y archivos recomendada

#### ¿Cómo se usa?
1. **Copia la carpeta** `_skeleton demo` dentro de `webots_demos` y renómbrala según tu proyecto (por ejemplo, `mi_nueva_demo/`).
2. Renombra los controladores y el mundo si lo deseas.
3. Modifica el código del controlador para implementar tu lógica, usando la jerarquía de robots (por ejemplo, `from robot_simur_uo.webots.epuck_robot import EPuck`).
4. Abre el mundo en Webots y selecciona el controlador correspondiente para cada robot.

#### ¿Por qué es útil?
- Garantiza que **todas las demos** sigan la misma estructura y buenas prácticas.
- Permite empezar desde una base funcional, sin errores de configuración.
- Facilita la integración con la jerarquía de robots y la reutilización de código.

#### Integración en `webots_demos`
La carpeta `_skeleton demo` está pensada para **no ser modificada directamente**. Siempre crea una copia antes de trabajar. Así, si necesitas empezar otro proyecto, tendrás siempre una plantilla limpia y actualizada.

---

