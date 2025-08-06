class GpsManager:
    """
    Manager para el sensor GPS de Webots.
    Encapsula inicialización y acceso a la posición.
    """
    def __init__(self, robot, device_name="gps", time_step=32):
        self.gps = robot.getDevice(device_name)
        self.gps.enable(time_step)

    def get_position(self):
        """Devuelve la posición actual [x, y, z] en metros."""
        return self.gps.getValues()
