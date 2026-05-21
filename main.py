"""
main.py
-------
Streamlit tabanlı profesyonel simülasyon arayüzü ve ana döngü.
Arayüz bileşenlerini oluşturur, parametreleri alır ve lokalizasyon
(EKF) ile navigasyon algoritmalarını koşturup görselleştirir.
"""

import streamlit as st
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import time

# Kendi yazdığımız modülleri içe aktarıyoruz
from environment import SCENARIOS
from kalman_filter import ExtendedKalmanFilter

# ==========================================
# 1. SAYFA VE ARAYÜZ AYARLARI (SIDEBAR)
# ==========================================
st.set_page_config(layout="wide", page_title="AutoNav Pro Simülatör")

st.sidebar.title("🤖 AutoNav Pro")
st.sidebar.subheader("Lokalizasyon & Navigasyon")

# Senaryo Seçimi
senaryo_secimi = st.sidebar.selectbox("Senaryo", list(SCENARIOS.keys()))
env = SCENARIOS[senaryo_secimi]

st.sidebar.markdown("---")
st.sidebar.markdown("### 📡 LiDAR PARAMETRELERİ")
lidar_menzil = st.sidebar.slider("Max Menzil (m)", 2.0, 15.0, 8.0)
lidar_isin = st.sidebar.slider("Işın Sayısı", 30, 360, 180)
lidar_gurultu = st.sidebar.slider("Gürültü σ (m)", 0.01, 0.20, 0.04)

st.sidebar.markdown("### 🧭 IMU & ENKODER")
imu_gurultu = st.sidebar.slider("Jiroskop σ (rad/s)", 0.001, 0.050, 0.008)
enc_gurultu = st.sidebar.slider("Enkoder σ (m)", 0.001, 0.050, 0.003)

st.sidebar.markdown("### 🧮 KALMAN FİLTRESİ (EKF)")
ekf_q = st.sidebar.slider("Q (proses σ)", 0.001, 0.100, 0.010)
ekf_r = st.sidebar.slider("R (ölçüm σ)", 0.010, 0.200, 0.050)
ekf_p = st.sidebar.slider("P_0 (başlangıç)", 0.010, 1.000, 0.100)

st.sidebar.markdown("### 🏎️ ROBOT & NAVİGASYON")
max_hiz = st.sidebar.slider("Maks Hız (m/s)", 0.5, 3.0, 1.2)
k_att = st.sidebar.slider("APF K_att (Çekim)", 0.5, 5.0, 1.5)
k_rep = st.sidebar.slider("APF K_rep (İtme)", 1.0, 10.0, 3.0)
d_obs = st.sidebar.slider("APF d_0 (Etki Mesafesi)", 0.5, 5.0, 2.0)

# Simülasyon Kontrol Butonları
col_btn1, col_btn2 = st.sidebar.columns(2)
baslat = col_btn1.button("▶️ Başlat", use_container_width=True)
sifirla = col_btn2.button("🔄 Sıfırla", use_container_width=True)

# ==========================================
# 2. NAVİGASYON (APF) VE SENSÖR YARDIMCI FONKSİYONLARI
# ==========================================
def calculate_apf_force(pos, goal, obstacles, k_att, k_rep, d_obs):
    """Yapay Potansiyel Alan (APF) ile yönelim kuvvetini hesaplar."""
    # Çekici Kuvvet (Hedefe doğru)
    f_att = k_att * (goal - pos)
    
    # İtici Kuvvet (Engellerden uzağa)
    f_rep = np.array([0.0, 0.0])
    for obs in obstacles:
        if obs['type'] == 'circle':
            cx, cy, cr = obs['params']
            dist = np.hypot(pos[0] - cx, pos[1] - cy) - cr
            direction = pos - np.array([cx, cy])
        elif obs['type'] == 'rectangle':
            rx, ry, rw, rh = obs['params']
            cx = max(rx, min(pos[0], rx + rw))
            cy = max(ry, min(pos[1], ry + rh))
            dist = np.hypot(pos[0] - cx, pos[1] - cy)
            direction = pos - np.array([cx, cy])
            
        if 0 < dist < d_obs:
            magnitude = k_rep * (1.0/dist - 1.0/d_obs) * (1.0/(dist**2))
            f_rep += magnitude * (direction / np.linalg.norm(direction))
            
    return f_att + f_rep

# ==========================================
# 3. ANA EKRAN YERLEŞİMİ
# ==========================================
st.title(f"🏭 {env.name} Simülasyonu")

# Üst Metrikler (Sıfır değerlerle başlat)
m1, m2, m3, m4 = st.columns(4)
metric_ekf = m1.empty()
metric_dr = m2.empty()
metric_dist = m3.empty()
metric_goal = m4.empty()

metric_ekf.metric("EKF RMSE (m)", "0.000")
metric_dr.metric("DR RMSE (m)", "0.000")
metric_dist.metric("Gidilen Mesafe (m)", "0.00")
metric_goal.metric("Hedefe Uzaklık (m)", f"{np.linalg.norm(env.start - env.goal):.2f}")

# Grafikler için alanlar
col_plot1, col_plot2 = st.columns([2, 1])
map_plot_area = col_plot1.empty()
error_plot_area = col_plot2.empty()

# ==========================================
# 4. SİMÜLASYON DÖNGÜSÜ
# ==========================================
if baslat:
    dt = 0.1
    
    # Robot ve EKF Başlatma
    x_true = np.array([env.start[0], env.start[1], 0.0])
    x_dr = np.copy(x_true)
    ekf = ExtendedKalmanFilter(env.start[0], env.start[1], 0.0, ekf_q, ekf_r, ekf_p)
    
    # Veri Kayıt Dizileri
    path_true, path_ekf, path_dr = [np.copy(x_true[:2])], [np.copy(x_true[:2])], [np.copy(x_dr[:2])]
    err_ekf_list, err_dr_list = [], []
    total_distance = 0.0
    
    # Gerçek zamanlı grafik objelerini bir kere oluştur
    fig_map, ax_map = plt.subplots(figsize=(8, 8))
    fig_err, ax_err = plt.subplots(figsize=(5, 4))
    
    for step in range(1000): # Maksimum adım
        # 1. Navigasyon (Hedef Yönelimi)
        force = calculate_apf_force(x_true[:2], env.goal, env.obstacles, k_att, k_rep, d_obs)
        v_cmd = np.clip(np.linalg.norm(force), 0, max_hiz)
        hedef_aci = np.arctan2(force[1], force[0])
        
        # Açısal hız (Basit P kontrolcü)
        aci_farki = ekf._normalize_angle(hedef_aci - x_true[2])
        omega_cmd = np.clip(aci_farki * 2.0, -1.0, 1.0)
        
        # 2. Gerçek Robot Hareketi
        x_true[0] += v_cmd * np.cos(x_true[2]) * dt
        x_true[1] += v_cmd * np.sin(x_true[2]) * dt
        x_true[2] = ekf._normalize_angle(x_true[2] + omega_cmd * dt)
        
        total_distance += v_cmd * dt
        hedefe_kalan = np.linalg.norm(x_true[:2] - env.goal)
        
        # 3. Sensör Ölçümleri (Gürültü Ekleme)
        v_meas = v_cmd + np.random.normal(0, enc_gurultu)
        omega_meas = omega_cmd + np.random.normal(0, imu_gurultu)
        z_lidar_x = x_true[0] + np.random.normal(0, lidar_gurultu)
        z_lidar_y = x_true[1] + np.random.normal(0, lidar_gurultu)
        
        # 4. Dead Reckoning (Sadece IMU ve Enkoder - EKF Yok)
        x_dr[0] += v_meas * np.cos(x_dr[2]) * dt
        x_dr[1] += v_meas * np.sin(x_dr[2]) * dt
        x_dr[2] = ekf._normalize_angle(x_dr[2] + omega_meas * dt)
        
        # 5. EKF Füzyonu
        ekf.predict(v_meas, omega_meas, dt)
        ekf.update(z_lidar_x, z_lidar_y)
        
        # Verileri Kaydet
        path_true.append(np.copy(x_true[:2]))
        path_dr.append(np.copy(x_dr[:2]))
        path_ekf.append(np.copy(ekf.x_est[:2]))
        
        err_dr_list.append(np.linalg.norm(x_true[:2] - x_dr[:2]))
        err_ekf_list.append(np.linalg.norm(x_true[:2] - ekf.x_est[:2]))

        # ==========================================
        # 5. GERÇEK ZAMANLI ÇİZİM (Her 5 adımda bir ekranı güncelle)
        # ==========================================
        if step % 5 == 0:
            # Metrikleri Güncelle
            metric_ekf.metric("EKF RMSE (m)", f"{err_ekf_list[-1]:.3f}")
            metric_dr.metric("DR RMSE (m)", f"{err_dr_list[-1]:.3f}")
            metric_dist.metric("Gidilen Mesafe (m)", f"{total_distance:.2f}")
            metric_goal.metric("Hedefe Uzaklık (m)", f"{hedefe_kalan:.2f}")
            
            # Haritayı Temizle ve Yeniden Çiz
            ax_map.clear()
            ax_map.set_facecolor('#0e1117')
            ax_map.set_xlim(0, env.map_size[0])
            ax_map.set_ylim(0, env.map_size[1])
            ax_map.grid(True, color='#333', linestyle='--', alpha=0.5)
            
            # Engelleri Çiz
            for obs in env.obstacles:
                if obs['type'] == 'circle':
                    cx, cy, cr = obs['params']
                    ax_map.add_patch(Circle((cx, cy), cr, color='#e94560', alpha=0.8))
                elif obs['type'] == 'rectangle':
                    rx, ry, rw, rh = obs['params']
                    ax_map.add_patch(Rectangle((rx, ry), rw, rh, color='#e94560', alpha=0.8))
            
            # Yolları Çiz (Numpy array'e çevirip çiziyoruz)
            pt = np.array(path_true)
            pe = np.array(path_ekf)
            pd = np.array(path_dr)
            
            ax_map.plot(pt[:, 0], pt[:, 1], 'g-', linewidth=2, label='Gerçek Yol')
            ax_map.plot(pd[:, 0], pd[:, 1], 'r--', alpha=0.6, label='Dead Reckoning')
            ax_map.plot(pe[:, 0], pe[:, 1], 'b-.', linewidth=2, label='EKF Tahmini')
            
            # Başlangıç ve Hedef Noktaları
            ax_map.plot(env.start[0], env.start[1], 'wo', markersize=8)
            ax_map.plot(env.goal[0], env.goal[1], 'y*', markersize=12)
            
            ax_map.legend(loc='upper left', facecolor='black', labelcolor='white')
            map_plot_area.pyplot(fig_map)
            
            # Hata Grafiğini Güncelle
            ax_err.clear()
            ax_err.plot(err_dr_list, 'r-', label='DR Hatası', alpha=0.7)
            ax_err.plot(err_ekf_list, 'b-', label='EKF Hatası', linewidth=2)
            ax_err.set_title("Zaman Serisi Hata Analizi")
            ax_err.set_xlabel("Zaman Adımı")
            ax_err.set_ylabel("Hata (m)")
            ax_err.legend()
            ax_err.grid(True)
            error_plot_area.pyplot(fig_err)
            
            # Çok hızlı dönmemesi için ufak bir bekleme (Animasyon etkisi)
            time.sleep(0.01)
            
        # Hedefe varıldıysa döngüyü kır
        if hedefe_kalan < 0.5:
            st.success("✅ Hedefe Başarıyla Ulaşıldı!")
            break