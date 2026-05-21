"""
navigation.py
-------------
Yapay Potansiyel Alan (APF) tabanlı navigasyon modülü.
Non-holonomic robotlar için hedef çekimi ve engel itmesini hesaplayarak 
doğrusal hız (v) ve açısal hız (omega) veya direksiyon açısı kontrol sinyalleri üretir.
"""

import numpy as np

class APFNavigator:
    """
    Yapay Potansiyel Alan (Artificial Potential Field) yöneticisi.
    """
    def __init__(self, k_att, k_rep, d_obs, max_speed, robot_type="Diferansiyel Sürüş"):
        # Arayüzden gelen parametreler
        self.k_att = k_att
        self.k_rep = k_rep
        self.d_obs = d_obs
        self.max_speed = max_speed
        
        # Robot Modeli Seçimi
        self.robot_type = robot_type
        
        # Fiziksel Limitler
        self.max_omega = 1.5      # Maksimum dönme hızı (Diferansiyel için) [rad/s]
        self.max_steer = np.pi/4  # Maksimum direksiyon açısı (Ackermann için) [rad]
        self.wheelbase = 1.0      # Dingil mesafesi (Ackermann için) [m]
        
        self.goal_tolerance = 0.5 # Hedefe ulaşma toleransı [m]
        self.reached = False

    def compute_force(self, pos, goal, obstacles):
        """
        Mevcut konum (pos) için toplam APF kuvvet vektörünü hesaplar.
        """
        # 1. Çekici Kuvvet (Hedefe Doğru)
        dist_to_goal = np.linalg.norm(goal - pos)
        if dist_to_goal < 1e-6:
            f_att = np.array([0.0, 0.0])
        else:
            # Parabolik çekim (Hedefe yaklaştıkça kuvvet azalır)
            f_att = self.k_att * (goal - pos)
        
        # 2. İtici Kuvvet (Engellerden Uzağa)
        f_rep = np.array([0.0, 0.0])
        for obs in obstacles:
            # Engel türüne göre en yakın noktayı hesapla
            if obs['type'] == 'circle':
                cx, cy, cr = obs['params']
                closest_point = np.array([cx, cy])
                dist = np.linalg.norm(pos - closest_point) - cr
                
            elif obs['type'] == 'rectangle':
                rx, ry, rw, rh = obs['params']
                cx = max(rx, min(pos[0], rx + rw))
                cy = max(ry, min(pos[1], ry + rh))
                closest_point = np.array([cx, cy])
                dist = np.linalg.norm(pos - closest_point)
            
            # Eğer robot engelin etki alanı içindeyse itme kuvveti uygula
            if 0 < dist < self.d_obs:
                direction = (pos - closest_point) / np.linalg.norm(pos - closest_point)
                magnitude = self.k_rep * (1.0/dist - 1.0/self.d_obs) * (1.0/(dist**2))
                
                # Çok büyük kuvvetleri sınırla (matematiksel patlamayı önlemek için)
                magnitude = min(magnitude, 10.0) 
                f_rep += magnitude * direction
                
        return f_att + f_rep

    def get_control(self, x, y, theta, goal, obstacles):
        """
        APF kuvvetinden seçilen non-holonomic robot modeline göre kontrol sinyalleri üretir.
        Döndürür: (v, omega) 
        - Diferansiyel için: v (doğrusal hız), omega (açısal hız)
        - Ackermann için: v (doğrusal hız), steer_angle (direksiyon açısı - omega gibi döndürülür)
        """
        pos = np.array([x, y])
        
        # Hedef kontrolü
        if np.linalg.norm(goal - pos) < self.goal_tolerance:
            self.reached = True
            return 0.0, 0.0

        # Toplam Kuvvet Vektörü
        force = self.compute_force(pos, goal, obstacles)
        force_magnitude = np.linalg.norm(force)
        
        # İstenen Hareket Yönü (Kuvvet Vektörünün Açısı)
        desired_angle = np.arctan2(force[1], force[0])
        
        # Yönelim Hatası (Robotun baktığı yön ile gitmesi gereken yön arasındaki fark)
        angle_error = self._normalize_angle(desired_angle - theta)
        
        # --- Robot Modeline Göre Kontrol (Non-Holonomic Kısıtlar) ---
        
        # Hız Kontrolü (Açı farkı büyükse yavaşla ki dönüşü tamamlayabilsin)
        v = min(self.max_speed, 0.5 * force_magnitude)
        if abs(angle_error) > np.pi / 4: 
            v *= 0.2 
        
        if self.robot_type == "Diferansiyel Sürüş":
            # Omega Kontrolü: Basit Oransal (P) Kontrolcü
            omega = np.clip(2.5 * angle_error, -self.max_omega, self.max_omega)
            return v, omega
            
        elif self.robot_type == "Ackermann (Araç Tipi)":
            # Ackermann Direksiyon Kontrolü
            steer_angle = np.clip(1.5 * angle_error, -self.max_steer, self.max_steer)
            # Ackermann kinematiğinde açısal hız (omega) direksiyon açısına bağlıdır: omega = (v / L) * tan(delta)
            # Ancak ana simülasyon döngüsü (x_k+1) direkt omega beklediği için, Ackermann'ın ürettiği
            # efektif dönüş miktarını omega formatında döndürüyoruz.
            effective_omega = (v / self.wheelbase) * np.tan(steer_angle)
            return v, effective_omega

    @staticmethod
    def _normalize_angle(a):
        return (a + np.pi) % (2 * np.pi) - np.pi