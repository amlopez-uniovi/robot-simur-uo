class CompassManager:
    """
    Manager para el sensor de brújula de Webots.
    Encapsula inicialización y acceso a la orientación.
    """
    def __init__(self, robot, device_name="compass", time_step=32):
        self.compass = robot.getDevice(device_name)
        self.compass.enable(time_step)

    def get_direction(self):
        """Devuelve el vector de dirección [x, y, z]."""
        return self.compass.getValues()
