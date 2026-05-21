# LiDAR Tabanlı Otonom Navigasyon Simülasyonu
### Sensör Füzyonu ve Lokalizasyon — 2B Simülasyon (Python)

---

## 📋 Proje Özeti

Bu proje, bir **depo ortamında** otonom mobil robotun LiDAR, IMU ve tekerlek enkoderi kullanarak
engelleri algılamasını, konumunu tahmin etmesini (lokalizasyon) ve hedefe güvenle ulaşmasını simüle eder.

| Özellik | Değer |
|---|---|
| Ortam | 20×20 metre, 12 dairesel engel |
| Başlangıç | (1.0, 1.0) |
| Hedef | (18.0, 18.0) |
| Robot Modeli | Non-holonomic diferansiyel sürüş |
| Sensörler | 2B LiDAR (360°), IMU, Tekerlek Enkoderi |
| Lokalizasyon | Extended Kalman Filter (EKF) |
| Navigasyon | Yapay Potansiyel Alan (APF) + RRT yol planlaması |

---

## 🗂️ Dosya Yapısı

```
lidar_navigation/
├── environment.py      ← 2B harita, engeller, çarpışma kontrolü
├── robot.py            ← Non-holonomic unicycle robot modeli
├── sensors.py          ← LiDAR, IMU, enkoder simülasyonu + gürültü
├── kalman_filter.py    ← Extended Kalman Filter (EKF)
├── navigation.py       ← APF navigasyon + RRT yol planlaması
├── visualization.py    ← Tüm grafik çıktıları
├── main.py             ← Ana simülasyon döngüsü
├── outputs/            ← Grafik çıktıları (otomatik oluşturulur)
│   ├── 01_environment_map.png
│   ├── 02_path_plan.png
│   ├── 03_lidar_visualization.png
│   ├── 03b_lidar_timeseries.png
│   ├── 04_localization.png
│   ├── 04b_localization_2d.png
│   ├── 05_error_analysis.png
│   └── 06_control_signals.png
└── README.md
```

---

## ⚙️ Kurulum

### Gereksinimler

- Python 3.8 veya üzeri
- pip

### Adım 1 — Depoyu klonla

```bash
git clone https://github.com/KULLANICI_ADIN/lidar-navigation.git
cd lidar-navigation
```

### Adım 2 — Bağımlılıkları yükle

```bash
pip install numpy matplotlib
```

> Sanal ortam kullanmak istersen:
> ```bash
> python -m venv venv
> source venv/bin/activate        # Windows: venv\Scripts\activate
> pip install numpy matplotlib
> ```

### Adım 3 — Simülasyonu çalıştır

```bash
python main.py
```

Simülasyon tamamlandığında tüm grafikler `outputs/` klasörüne otomatik olarak kaydedilir.

---

## 🧩 Teknik Detaylar

### Robot Modeli (Non-holonomic)

Diferansiyel sürüş robotu, unicycle kinematik modeli ile hareket eder:

```
x'     = v · cos(θ)
y'     = v · sin(θ)
θ'     = ω
```

Kontrol sinyalleri: `v` (ilerleme hızı, m/s) ve `ω` (dönme hızı, rad/s).

### LiDAR Simülasyonu

- 360 ışın, 1° çözünürlük
- Maksimum menzil: 7 metre
- Gürültü: σ = 0.04 m (Gaussian)
- Filtreleme: Median filtre + outlier eleme

### Extended Kalman Filter (EKF)

Durum vektörü: `[x, y, θ]`

| Matris | Açıklama | Değer |
|---|---|---|
| **Q** | Proses gürültüsü | diag(0.01, 0.01, 0.005) |
| **R** | Ölçüm gürültüsü | diag(0.05, 0.05, 0.03) |
| **P₀** | Başlangıç kovaryansı | diag(0.1, 0.1, 0.05) |

**Tahmin adımı:** Enkoder ölçümü + Jacobian ile kovaryans güncelleme  
**Güncelleme adımı:** IMU açı ölçümü + Kalman kazancı ile düzeltme

### Navigasyon — APF

Toplam kuvvet = Çekim (hedefe doğru) + İtme (engellerden uzak):

```
F_toplam = F_çekim + F_itme
```

RRT ile ön-planlama yapılır, APF ile reaktif engel kaçınma gerçekleştirilir.

---

## 📊 Sonuçlar

| Metrik | EKF | Dead Reckoning |
|---|---|---|
| RMSE | **0.051 m** | 0.601 m |
| MAE | **0.045 m** | 0.500 m |

- Robot hedefe **620 adımda (31 saniye)** ulaştı
- Toplam kat edilen mesafe: **23.68 m**
- EKF, dead reckoning'e göre **~12× daha doğru** lokalizasyon sağladı

---

## 🤖 Yapay Zeka Kullanım Beyanı

**Kullanılan araçlar:** Claude (claude-sonnet-4-6)

**Kullanıldığı bölümler:**
- Kalman Filtresi kod iskeletinin oluşturulması
- APF navigasyon parametrelerinin ayarlanması
- Kodların hata ayıklaması
- README ve rapor metninin dil düzenlemesi

**Öğrencinin katkıları:**
- Proje senaryosu ve sistem mimarisinin tasarımı
- Kodların test edilmesi ve çalıştırılması
- Sonuç yorumları ve değerlendirme

---

## 📚 Kaynaklar

[1] V. Ušinskis et al., "Sensor-fusion based navigation for autonomous mobile robot," *Sensors*, vol. 25, no. 4, art. 1248, 2025.

[2] Y. Ou et al., "Autonomous navigation by mobile robot with sensor fusion based on deep reinforcement learning," *Sensors*, vol. 24, no. 12, art. 3895, 2024.

[3] B. Zhang and C. Li, "The optimization and application research of the RRT-APF-based path planning algorithm," *Electronics*, vol. 13, no. 24, art. 4963, 2024.
