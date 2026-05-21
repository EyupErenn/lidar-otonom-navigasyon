"""
robot_kinematics.py
-------------------
Non-holonomic mobil robot kinematik modelleri.
PDF gereksinimlerine uygun olarak hem Diferansiyel (Differential) 
hem de Ackermann (Araç tipi) sürüş modellerini barındırır.
"""

import numpy as np

class RobotState:
    """Robotun anlık durumunu (x, y, theta) tutan veri sınıfı."""
    def __init__(self, x=0.0, y=0.0, theta=0.0):
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

    def get_array(self):
        return np.array([self.x, self.y, self.theta])

class DifferentialDrive:
    """
    Diferansiyel Sürüş Modeli (Unicycle)
    Robot sadece baktığı yönde ilerleyebilir (yana kayamaz).
    Kontrol girdileri: v (ilerleme hızı), omega (dönüş hızı)
    """
    def __init__(self, start_x, start_y, start_theta, wheel_base=0.8):
        self.state = RobotState(start_x, start_y, start_theta)
        self.wheel_base = wheel_base

    def step(self, v, omega, dt):
        """Kinematik denklemlerle (Euler entegrasyonu) konumu günceller."""
        self.state.x += v * np.cos(self.state.theta) * dt
        self.state.y += v * np.sin(self.state.theta) * dt
        self.state.theta += omega * dt
        self.state.theta = self._normalize_angle(self.state.theta)
        return self.state.get_array()

    @staticmethod
    def _normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

class AckermannDrive:
    """
    Ackermann Sürüş Modeli (Araç Tipi)
    Araçlar gibi ön tekerleklerin dönme açısına (direksiyon) bağlıdır.
    Kontrol girdileri: v (ilerleme hızı), steer_angle (direksiyon açısı)
    """
    def __init__(self, start_x, start_y, start_theta, wheelbase=1.0):
        self.state = RobotState(start_x, start_y, start_theta)
        self.wheelbase = wheelbase  # Ön ve arka tekerlekler arası mesafe (L)

    def step(self, v, steer_angle, dt):
        """Ackermann kinematik denklemleri ile konumu günceller."""
        self.state.x += v * np.cos(self.state.theta) * dt
        self.state.y += v * np.sin(self.state.theta) * dt
        
        # Açısal hız, direksiyon açısı ve dingil mesafesine bağlıdır
        omega = (v / self.wheelbase) * np.tan(steer_angle)
        
        self.state.theta += omega * dt
        self.state.theta = self._normalize_angle(self.state.theta)
        return self.state.get_array()

    @staticmethod
    def _normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

class DeadReckoning:
    """
    Sadece Enkoder ve IMU verisi kullanılarak konum tahmini (Kalman/LiDAR olmadan).
    Sensör füzyonu başarısını kıyaslamak için kullanılır.
    """
    def __init__(self, start_x, start_y, start_theta):
        self.state = RobotState(start_x, start_y, start_theta)

    def update(self, v_meas, omega_meas, dt):
        """Gürültülü sensör ölçümleriyle konumu ilerletir."""
        self.state.x += v_meas * np.cos(self.state.theta) * dt
        self.state.y += v_meas * np.sin(self.state.theta) * dt
        self.state.theta += omega_meas * dt
        self.state.theta = self._normalize_angle(self.state.theta)
        return self.state.get_array()

    @staticmethod
    def _normalize_angle(angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi