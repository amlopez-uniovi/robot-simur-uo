import numpy as np

class DepthCameraManager:
    """
    Manager para la cámara de profundidad de Webots (Depth).
    Permite inicializar, capturar mapas de profundidad y configurar la cámara.
    """
    def __init__(self, robot, device_name="depth_camera", time_step=64):
        self.camera = robot.getDevice(device_name)
        self.camera.enable(time_step)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        self.time_step = time_step

    def get_depth_map(self) -> np.ndarray:
        """Devuelve el mapa de profundidad actual como un array numpy (H, W)."""
        depth_map = self.camera.getRangeImageArray()
        if depth_map is not None:
            return np.array(depth_map, dtype=np.float32)
        return None

    def get_resolution(self):
        return self.width, self.height
