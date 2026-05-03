# ========================================================
# SINIF: PIDController
# ========================================================
# Standart PID kontrolcü. Çizgi izleme hata sinyalini (line_error)
# direksiyon açısına dönüştürür.

class PIDController:
    def __init__(self, kp: float = 0.5, ki: float = 0.0, kd: float = 0.1,
                 output_min: float = -5.0, output_max: float = 5.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True

    def compute(self, error: float, dt: float) -> float:
        """Hata değerinden kontrol çıktısı hesapla."""
        if self._first_call:
            self._prev_error = error
            self._first_call = False

        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # Clamp
        output = max(self.output_min, min(self.output_max, output))

        self._prev_error = error
        return output

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._first_call = True
