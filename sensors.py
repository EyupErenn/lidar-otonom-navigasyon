"""
sensors.py
----------
LiDAR, IMU ve Tekerlek Enkoderi sensör simülasyonları.
Gerçekçi Gaussian gürültü modelleri ve fiziksel ışın izleme 
(ray-casting) algoritmalarını içerir.
"""

import numpy as np

# ─── LiDAR SENSÖRÜ ──────────────────────────────────────────────────────────

class LiDAR:
    """
    2B LiDAR sensörü simülasyonu.
    Belirlenen açı çözünürlüğünde ışınlar göndererek ortamdaki 
    dairesel ve dikdörtgen engellerin mesafesini ölçer.
    """
    def __init__(self, max_range=8.0, num_beams=180, noise_std=0.04):
        self.max_range = max_range
        self.num_beams = num_beams
        self.noise_std = noise_std
        # Işın açılarını 0 ile 2*PI arasında eşit dağıt
        self.angles = np.linspace(0, 2 * np.pi, num_beams, endpoint=False)

    def scan(self, robot_x, robot_y, robot_theta, obstacles, map_size):
        """
        Robotun bulunduğu konumdan ortamdaki engellere doğru tarama yapar.
        Bağımlılık enjeksiyonu ile (obstacles, map_size) ana döngüden alınır.
        """
        beam_angles = self.angles + robot_theta
        distances_true = np.full(self.num_beams, self.max_range)
        hit_points = []

        for i, angle in enumerate(beam_angles):
            d = self._ray_cast(robot_x, robot_y, angle, obstacles, map_size)
            distances_true[i] = d
            
            # Çarpma noktasının (x, y) koordinatlarını hesapla
            hx = robot_x + d * np.cos(angle)
            hy = robot_y + d * np.sin(angle)
            hit_points.append((hx, hy))

        # Sensör Gürültüsü Ekleme (Ham Veri = Gerçek + Gaussian Gürültü)
        noise = np.random.normal(0, self.noise_std, self.num_beams)
        distances_raw = np.clip(distances_true + noise, 0.1, self.max_range)

        return beam_angles, distances_raw, distances_true, hit_points

    def _ray_cast(self, rx, ry, angle, obstacles, map_size):
        """
        Tek bir ışını ilerletir ve ilk engele veya duvara çarpma mesafesini bulur.
        Hem daire hem de dikdörtgen engelleri destekler.
        """
        step = 0.1  # Işın ilerleme adımı (Çözünürlük)
        dist = 0.0
        cos_a = np.cos(angle)
        sin_a = np.sin(angle)

        while dist < self.max_range:
            dist += step
            px = rx + dist * cos_a
            py = ry + dist * sin_a

            # 1. Duvar (Harita Sınırı) Kontrolü
            if px <= 0 or px >= map_size[0] or py <= 0 or py >= map_size[1]:
                return dist

            # 2. Engel Kontrolü
            for obs in obstacles:
                if obs['type'] == 'circle':
                    cx, cy, cr = obs['params']
                    if np.hypot(px - cx, py - cy) <= cr:
                        return dist
                        
                elif obs['type'] == 'rectangle':
                    bx, by, bw, bh = obs['params']
                    if bx <= px <= bx + bw and by <= py <= by + bh:
                        return dist

        return self.max_range

    def filter_scan(self, distances_raw, window=5):
        """
        Ödev İsteri: Sensör verisini temizleme.
        Median filtresi uygulayarak ham LiDAR verisindeki anlık parazitleri temizler.
        """
        filtered = np.copy(distances_raw)
        
        # Dairesel median filtresi
        for i in range(len(distances_raw)):
            indices = np.arange(i - window // 2, i + window // 2 + 1) % len(distances_raw)
            filtered[i] = np.median(distances_raw[indices])
            
        return filtered


# ─── IMU SENSÖRÜ ────────────────────────────────────────────────────────────

class IMU:
    """
    IMU (İnersiyel Ölçüm Birimi) simülasyonu.
    Açısal hız ölçümlerine (Jiroskop) gürültü ve sürüklenme (drift) ekler.
    """
    def __init__(self, gyro_noise=0.008, bias_gyro=0.003):
        self.gyro_noise = gyro_noise
        self.bias_gyro = np.random.uniform(-bias_gyro, bias_gyro)

    def measure_omega(self, true_omega):
        """Gerçek açısal hıza gürültü ve bias ekleyerek ölçüm üret."""
        return true_omega + np.random.normal(0, self.gyro_noise) + self.bias_gyro


# ─── TEKERLEK ENKODERİ ──────────────────────────────────────────────────────

class WheelEncoder:
    """
    Tekerlek enkoderi simülasyonu.
    Robotun ilerlediği mesafeye kayma ve ölçüm gürültüsü ekler.
    """
    def __init__(self, noise_std=0.003):
        self.noise_std = noise_std

    def measure_velocity(self, true_v):
        """Gerçek ilerleme hızına enkoder okuma gürültüsü ekle."""
        return true_v + np.random.normal(0, self.noise_std)