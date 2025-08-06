class ImuManager:
    """
    Manager para el sensor IMU de Webots (acelerómetro, giroscopio y brújula).
    Encapsula inicialización y acceso a los valores de los sensores.
    """
    def __init__(self, robot, accel_name="imu accelerometer", gyro_name="imu gyro", compass_name="imu compass", time_step=32):
        self.accelerometer = robot.getDevice(accel_name)
        self.gyro = robot.getDevice(gyro_name)
        self.compass = robot.getDevice(compass_name)
        self.accelerometer.enable(time_step)
        self.gyro.enable(time_step)
        self.compass.enable(time_step)

    def get_accelerometer(self):
        """Devuelve los valores del acelerómetro [x, y, z] en m/s²."""
        return self.accelerometer.getValues()

    def get_gyro(self):
        """Devuelve los valores del giroscopio [x, y, z] en rad/s."""
        return self.gyro.getValues()

    def get_compass(self):
        """Devuelve los valores de la brújula [x, y, z]."""
        return self.compass.getValues()
