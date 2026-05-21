"""
kalman_filter.py (veya sensor_fusion.py)
----------------
Genişletilmiş Kalman Filtresi (EKF) — Sensör Füzyonu Modülü

Ödev İsterlerine Uygunluk: 
- Tahmin (Predict) Adımı: Tekerlek Enkoderi (Hız) ve IMU (Açısal Hız) verileri ile yapılır.
- Güncelleme (Update) Adımı: LiDAR (Konum tespiti) verisi ile yapılır.
Bu sayede LiDAR, IMU ve Enkoder tam teşekküllü bir şekilde EKF içinde kaynaştırılır.
"""

import numpy as np

class ExtendedKalmanFilter:
    def __init__(self, start_x, start_y, start_theta, q_std, r_std, p_init=0.1):
        """
        Arayüzden (Streamlit) gelen Q, R ve P parametreleri ile EKF'yi başlatır.
        """
        # Durum vektörü: [x, y, theta]
        self.x_est = np.array([start_x, start_y, start_theta], dtype=float)
        
        # P: Hata Kovaryans Matrisi (Başlangıç belirsizliği)
        self.P = np.eye(3) * p_init
        
        # Q: Proses Gürültüsü Kovaryans Matrisi (Sistemin kendi dinamiğindeki hata)
        self.Q = np.diag([q_std**2, q_std**2, (q_std/2)**2])
        
        # R: Ölçüm Gürültüsü Kovaryans Matrisi (LiDAR sensöründen gelen hatanın varyansı)
        # Sadece x ve y konumlarını LiDAR'dan güncelleyeceğimiz için 2x2 boyutundadır
        self.R = np.diag([r_std**2, r_std**2])
        
        # H: Gözlem (Ölçüm) Matrisi. (3 durum değişkeninden sadece x ve y'yi ölçüyoruz)
        self.H = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0]
        ])

    def predict(self, v_enc, omega_imu, dt):
        """
        Tahmin Adımı (Predict)
        Enkoder (v_enc) ve IMU (omega_imu) kullanılarak robotun bir sonraki 
        konumu hareket denklemleri ile tahmin edilir.
        """
        theta = self.x_est[2]
        
        # 1. Non-holonomic hareket modeli ile durum tahmini
        self.x_est[0] += v_enc * np.cos(theta) * dt
        self.x_est[1] += v_enc * np.sin(theta) * dt
        self.x_est[2] += omega_imu * dt
        self.x_est[2] = self._normalize_angle(self.x_est[2])
        
        # 2. F: Sistemin Jacobian Matrisi (Durum geçişinin türevi)
        F = np.array([
            [1.0, 0.0, -v_enc * np.sin(theta) * dt],
            [0.0, 1.0,  v_enc * np.cos(theta) * dt],
            [0.0, 0.0, 1.0]
        ])
        
        # 3. Tahmin edilen kovaryans güncellemesi: P = F * P * F^T + Q
        self.P = F @ self.P @ F.T + self.Q

    def update(self, z_lidar_x, z_lidar_y):
        """
        Güncelleme Adımı (Update)
        LiDAR'dan elde edilen bağımsız (x, y) konum verisi ile enkoder tahmini düzeltilir.
        """
        # LiDAR ölçüm vektörü z
        z = np.array([z_lidar_x, z_lidar_y])
        
        # 1. Tahmin edilen ölçüm: z_pred = H * x_est
        z_pred = self.H @ self.x_est
        
        # 2. Yenilik (Innovation) vektörü: y = z - z_pred
        y = z - z_pred
        
        # 3. Yenilik kovaryansı: S = H * P * H^T + R
        S = self.H @ self.P @ self.H.T + self.R
        
        # 4. Kalman Kazancı: K = P * H^T * S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # 5. Durumun Kalman kazancı ile güncellenmesi: x = x + K * y
        self.x_est = self.x_est + K @ y
        self.x_est[2] = self._normalize_angle(self.x_est[2])
        
        # 6. Kovaryans matrisinin güncellenmesi: P = (I - K * H) * P
        I = np.eye(3)
        self.P = (I - K @ self.H) @ self.P

    @staticmethod
    def _normalize_angle(angle):
        """Açıyı sınırları içerisinde [-pi, pi] normalize eder."""
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle