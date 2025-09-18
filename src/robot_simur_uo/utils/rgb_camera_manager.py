import numpy as np

class RgbCameraManager:
    """
    Manager para la cámara RGB de Webots.
    Permite inicializar, capturar imágenes y configurar la cámara RGB.
    """
    def __init__(self, robot, device_name="camera", time_step=64):
        self.camera = robot.getDevice(device_name)
        self.camera.enable(time_step)
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        self.time_step = time_step

    def get_image(self) -> np.ndarray:
        """Devuelve la imagen RGB actual como un array numpy (H, W, 3)."""
        image = self.camera.getImageArray()
        if image is not None:
            return np.array(image, dtype=np.uint8)
        return None

    def get_resolution(self):
        return self.width, self.height
