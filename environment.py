"""
environment.py
--------------
Simülasyon ortamını, harita boyutlarını, engelleri ve çarpışma tespit 
algoritmalarını barındıran veri ve fizik modülü. Çizim işlemleri 
visualization.py modülüne devredilmiştir.
"""

import numpy as np

class Environment:
    """
    Robotun içinde hareket edeceği haritayı ve engelleri tanımlayan temel sınıf.
    """
    def __init__(self, name, map_size, start, goal, obstacles, robot_radius=0.4):
        self.name = name
        self.map_size = map_size  # (genişlik, yükseklik) metre cinsinden
        self.start = np.array(start, dtype=float)
        self.goal = np.array(goal, dtype=float)
        self.obstacles = obstacles  # Engel listesi (Sözlük yapısında)
        self.robot_radius = robot_radius

    def is_collision(self, x, y, margin=0.0):
        """
        Verilen (x,y) noktasının engellere veya harita sınırlarına 
        çarpıp çarpmadığını kontrol eder.
        """
        r = self.robot_radius + margin
        
        # 1. Duvar (Harita Sınırı) Kontrolü
        if x < r or x > self.map_size[0] - r:
            return True
        if y < r or y > self.map_size[1] - r:
            return True
            
        # 2. Dinamik Engel Kontrolü
        for obs in self.obstacles:
            if obs['type'] == 'circle':
                cx, cy, cr = obs['params']
                # Dairesel engel çarpışma tespiti (Öklid uzaklığı)
                if np.hypot(x - cx, y - cy) <= cr + r:
                    return True
                    
            elif obs['type'] == 'rectangle':
                rx, ry, rw, rh = obs['params']
                # Dikdörtgen engel (raf/palet) için en yakın noktayı bulma
                closest_x = max(rx, min(x, rx + rw))
                closest_y = max(ry, min(y, ry + rh))
                if np.hypot(x - closest_x, y - closest_y) <= r:
                    return True
                    
        return False

# ==========================================
# SENARYOLAR (Streamlit Arayüzü İçin)
# ==========================================

# 1. Depo Senaryosu (Palet ve Raflar - Dikdörtgen ve Daire karışık, En az 10 engel)
depo_engelleri = [
    {'type': 'rectangle', 'params': (2.0, 2.0, 2.0, 1.0)},  # x, y, genişlik, yükseklik
    {'type': 'rectangle', 'params': (7.0, 2.0, 1.0, 3.0)},
    {'type': 'circle', 'params': (4.0, 6.0, 0.8)},          # cx, cy, yarıçap
    {'type': 'rectangle', 'params': (10.0, 5.0, 3.0, 1.0)},
    {'type': 'circle', 'params': (14.0, 3.0, 1.0)},
    {'type': 'rectangle', 'params': (2.0, 10.0, 1.0, 4.0)},
    {'type': 'rectangle', 'params': (6.0, 9.0, 4.0, 1.0)},
    {'type': 'circle', 'params': (12.0, 10.0, 0.9)},
    {'type': 'rectangle', 'params': (16.0, 7.0, 1.0, 5.0)},
    {'type': 'circle', 'params': (8.0, 14.0, 0.7)},
    {'type': 'rectangle', 'params': (12.0, 15.0, 3.0, 1.0)},
    {'type': 'circle', 'params': (18.0, 18.0, 0.5)}, 
]

# 2. Keşif Senaryosu (Daha serbest, sadece dairesel engeller)
kesif_engelleri = [
    {'type': 'circle', 'params': (5.0, 5.0, 1.5)},
    {'type': 'circle', 'params': (10.0, 10.0, 2.0)},
    {'type': 'circle', 'params': (15.0, 5.0, 1.5)},
    {'type': 'circle', 'params': (5.0, 15.0, 1.5)},
    {'type': 'circle', 'params': (15.0, 15.0, 1.5)},
    {'type': 'circle', 'params': (10.0, 4.0, 1.0)},
    {'type': 'circle', 'params': (10.0, 16.0, 1.0)},
]

# Arayüzdeki dropdown'ın okuyacağı ana sözlük
SCENARIOS = {
    "Depo Senaryosu": Environment(
        name="Depo Senaryosu",
        map_size=(20.0, 20.0),
        start=(1.0, 1.0),
        goal=(19.0, 19.0),
        obstacles=depo_engelleri
    ),
    "Keşif Senaryosu": Environment(
        name="Keşif Senaryosu",
        map_size=(20.0, 20.0),
        start=(2.0, 2.0),
        goal=(18.0, 18.0),
        obstacles=kesif_engelleri
    )
}