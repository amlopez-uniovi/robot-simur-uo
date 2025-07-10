"""
Controlador PID para control preciso de robots.
"""

import time
from typing import Optional


class PIDController:
    """Controlador PID para control de posición/velocidad de robots."""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_min: float = -1.0, output_max: float = 1.0):
        """
        Inicializa el controlador PID.
        
        Args:
            kp: Ganancia proporcional
            ki: Ganancia integral
            kd: Ganancia derivativa
            output_min: Valor mínimo de salida
            output_max: Valor máximo de salida
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        
        # Variables internas
        self.prev_error: Optional[float] = None
        self.integral = 0.0
        self.prev_time: Optional[float] = None
        
    def reset(self):
        """Reinicia el estado interno del controlador."""
        self.prev_error = None
        self.integral = 0.0
        self.prev_time = None
    
    def update(self, setpoint: float, measurement: float, dt: Optional[float] = None) -> float:
        """
        Calcula la salida del controlador PID.
        
        Args:
            setpoint: Valor deseado
            measurement: Valor medido actual
            dt: Tiempo transcurrido (se calcula automáticamente si es None)
            
        Returns:
            Salida del controlador
        """
        current_time = time.time()
        
        # Calcular error
        error = setpoint - measurement
        
        # Calcular dt si no se proporciona
        if dt is None:
            if self.prev_time is not None:
                dt = current_time - self.prev_time
            else:
                dt = 0.0
        
        # Término proporcional
        proportional = self.kp * error
        
        # Término integral
        if dt > 0:
            self.integral += error * dt
        integral_term = self.ki * self.integral
        
        # Término derivativo
        derivative = 0.0
        if self.prev_error is not None and dt > 0:
            derivative = self.kd * (error - self.prev_error) / dt
        
        # Calcular salida
        output = proportional + integral_term + derivative
        
        # Aplicar límites
        output = max(self.output_min, min(self.output_max, output))
        
        # Actualizar variables para la siguiente iteración
        self.prev_error = error
        self.prev_time = current_time
        
        return output
    
    def set_gains(self, kp: float, ki: float, kd: float):
        """
        Actualiza las ganancias del controlador.
        
        Args:
            kp: Nueva ganancia proporcional
            ki: Nueva ganancia integral
            kd: Nueva ganancia derivativa
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
    
    def set_output_limits(self, output_min: float, output_max: float):
        """
        Actualiza los límites de salida.
        
        Args:
            output_min: Nuevo límite mínimo
            output_max: Nuevo límite máximo
        """
        self.output_min = output_min
        self.output_max = output_max
    
    def get_error(self) -> Optional[float]:
        """Retorna el último error calculado."""
        return self.prev_error
    
    def get_integral(self) -> float:
        """Retorna el valor actual del término integral."""
        return self.integral


class DualPIDController:
    """Controlador PID dual para control independiente de motores izquierdo y derecho."""
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0):
        """
        Inicializa el controlador PID dual.
        
        Args:
            kp: Ganancia proporcional
            ki: Ganancia integral  
            kd: Ganancia derivativa
        """
        self.left_pid = PIDController(kp, ki, kd)
        self.right_pid = PIDController(kp, ki, kd)
    
    def update(self, left_setpoint: float, left_measurement: float,
               right_setpoint: float, right_measurement: float,
               dt: Optional[float] = None) -> tuple:
        """
        Actualiza ambos controladores PID.
        
        Args:
            left_setpoint: Velocidad deseada motor izquierdo
            left_measurement: Velocidad medida motor izquierdo
            right_setpoint: Velocidad deseada motor derecho
            right_measurement: Velocidad medida motor derecho
            dt: Intervalo de tiempo
            
        Returns:
            Tuple con (salida_izquierda, salida_derecha)
        """
        left_output = self.left_pid.update(left_setpoint, left_measurement, dt)
        right_output = self.right_pid.update(right_setpoint, right_measurement, dt)
        
        return left_output, right_output
    
    def reset(self):
        """Reinicia ambos controladores."""
        self.left_pid.reset()
        self.right_pid.reset()
    
    def set_gains(self, kp: float, ki: float, kd: float):
        """Actualiza las ganancias de ambos controladores."""
        self.left_pid.set_gains(kp, ki, kd)
        self.right_pid.set_gains(kp, ki, kd)
