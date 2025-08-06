"""
Interfaz base para todos los tipos de robots.
"""

from abc import ABC, abstractmethod
from ..utils.coordinates import RobotPose


class IRobotBase(ABC):
    """
    Interfaz base para todos los robots.
    
    Define los métodos comunes que deben implementar todas las clases de robot
    independientemente de su tipo de locomoción.
    
    Implementa directamente la gestión de pose que es común para todos los robots.
    """
    
    def __init__(self):
        """Inicializa el robot con pose y estado básico."""
    
    # Métodos implementados (comunes para todos los robots)
    @abstractmethod
    def get_pose(self) -> RobotPose:
        """
        Obtiene la pose actual del robot.
        
        Returns:
            Pose actual del robot
        """
        return RobotPose(0.0, 0.0, 0.0)  # Valor por defecto, puede ser sobrescrito por subclases
    

    
    @abstractmethod
    def set_drive_command(self, forward_speed: float, steering_speed: float) -> None:
        """
        Establece la velocidad de avance y dirección simultáneamente (interfaz principal).
        
        Args:
            forward_speed: Velocidad lineal (m/s)
            steering_speed: Velocidad de dirección (rad/s)
        """
        pass
    

    
    def stop(self) -> None:
        """Detiene el robot."""
       
        self.set_drive_command(0.0, 0.0)
    
    def cleanup(self) -> None:
        """Limpieza del robot."""
        self.stop()

    
    def log_devices(self, to_terminal: bool = True, to_file: str = None) -> None:
        """
        Registra información de dispositivos del robot.
        
        Las subclases deben sobrescribir este método para generar información específica
        y almacenarla en self.log_message, luego llamar a este método base para manejar
        la salida a terminal y archivo.
        
        Args:
            to_terminal: Si True, imprime a la terminal
            to_file: Si se especifica, escribe al archivo indicado
        """
        import time
        import os
        
        # Si no hay mensaje generado por la subclase, crear uno básico
        if not hasattr(self, 'log_message') or not self.log_message:
            timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
            self.log_message = f"=== Robot Device Log - {timestamp} ===\nNo specific device information available.\n" + "=" * 70
        
        # Salida a terminal
        if to_terminal:
            print(self.log_message)
            
        # Salida a archivo con manejo avanzado
        if to_file:
            try:
                # Generar timestamp para operaciones de archivo
                timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
                
                # Verificar si el archivo existe
                file_exists = os.path.exists(to_file)
                
                # Crear directorio padre si no existe
                if os.path.dirname(to_file):  # Solo si hay un directorio padre
                    os.makedirs(os.path.dirname(to_file), exist_ok=True)
                
                # Escribir al archivo (crear o añadir)
                with open(to_file, 'a', encoding='utf-8') as f:
                    # Si es un archivo nuevo, agregar cabecera
                    if not file_exists:
                        f.write(f"=== Robot Device Log File Created - {timestamp} ===\n")
                        f.write(f"Log file: {to_file}\n")
                        f.write("=" * 70 + "\n\n")
                    
                    f.write(f"{self.log_message}\n\n")
                
                # Informar sobre la operación realizada
                if to_terminal:
                    if file_exists:
                        print(f"✅ Log añadido al archivo existente: {to_file}")
                    else:
                        print(f"✅ Archivo de log creado: {to_file}")
                        
            except Exception as e:
                if to_terminal:
                    print(f"❌ Error escribiendo al archivo {to_file}: {e}")
                    
        # Limpiar el mensaje después de usarlo (opcional)
        self.log_message = ""
            
    # Métodos abstractos (deben ser implementados por las subclases)
    @abstractmethod
    def step(self, dt: float) -> None:
        """
        Ejecuta un paso de simulación.
        
        Args:
            dt: Paso de tiempo en segundos
        """
        pass
