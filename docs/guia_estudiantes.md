# Guía para estudiantes de **robot-simur-uo**

> Documento introductorio para comprender, instalar y usar la librería en asignaturas/prácticas. Mantén esta guía junto al código fuente.

---

## 1) ¿Qué es *robot-simur-uo*?

**robot-simur-uo** es una librería Python para **simulación y control de robots** en el contexto docente de la Universidad de Oviedo. Proporciona:

- **Controladores** de navegación (Bug0, lookahead, campos potenciales, random, waypoints).
- **Interfaces de robot** (base, diferencial, Ackermann).
- **Adaptadores Webots** (e-puck, RosBot).
- **Utilidades**: sensores, mapas de ocupación, coordenadas, waypoints, visualización ASCII.

Objetivo: que el alumnado pueda **experimentar rápido** con robots simulados y luego **profundizar** en el código.

---

## 2) Requisitos e instalación

### Requisitos
- Python ≥ 3.8 (probado con 3.10+).
- `pip` actualizado.
- **Opcional:** `venv`/`conda` para aislar dependencias.
- **Para demos:** [Webots](https://cyberbotics.com/) instalado (las demos viven en `webots_demos/`).
- **Para doc:** `sphinx`, `myst-parser` (si usas Markdown), y un tema (`furo` o similar).

> El proyecto usa **PyScaffold** y `setuptools_scm` para versionado. Las dependencias mínimas vienen declaradas en `setup.cfg` (no hay librerías pesadas por defecto).

### Instalación (editable)
```bash
# 1) Clona el repositorio
git clone https://github.com/amlopez-uniovi/robot-simur-uo.git
cd robot-simur-uo

# 2) (Opcional) Crea y activa entorno
python -m venv .venv
# Windows: .venv\Scripts\activate
# Linux/Mac:
source .venv/bin/activate

# 3) Instala en modo editable
pip install -U pip setuptools setuptools_scm wheel
pip install -e .
