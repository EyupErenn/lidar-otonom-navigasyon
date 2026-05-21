
# AutoNav Pro: LiDAR Tabanlı Otonom Navigasyon Simülasyonu

### Sensör Füzyonu ve Lokalizasyon — 2B Web Simülasyonu (Python & Streamlit)

---

## 📋 Proje Özeti

Bu proje, bir **depo ve keşif ortamında** otonom mobil robotun LiDAR, IMU ve tekerlek enkoderi kullanarak engelleri algılamasını, konumunu tahmin etmesini (lokalizasyon) ve hedefe güvenle ulaşmasını gerçek zamanlı olarak simüle eden web tabanlı bir arayüzdür.

| Özellik | Değer |
| --- | --- |
| **Platform** | Streamlit tabanlı dinamik web arayüzü |
| **Ortamlar** | Depo (12 raf/engel) ve Keşif Senaryoları |
| **Robot Modelleri** | Non-holonomic (Diferansiyel Sürüş & Ackermann) |
| **Sensörler** | 2B LiDAR (Ray-casting), IMU, Tekerlek Enkoderi |
| **Lokalizasyon** | Genişletilmiş Kalman Filtresi (EKF) — Sensör Füzyonu |
| **Navigasyon** | Yapay Potansiyel Alan (APF) |

---

## 🗂️ Dosya Yapısı (Modüler OOP Mimarisi)

```text
lidar_navigation/
├── main.py                 ← Streamlit ana arayüzü ve simülasyon döngüsü
├── environment.py          ← 2B haritalar, senaryolar ve engeller (Daire/Dikdörtgen)
├── robot_kinematics.py     ← Diferansiyel ve Ackermann robot kinematik modelleri
├── sensors.py              ← LiDAR (ışın izleme), IMU ve Enkoder gürültü simülasyonları
├── kalman_filter.py        ← EKF tabanlı sensör füzyonu (Tahmin: IMU+Enc, Güncelleme: LiDAR)
├── navigation.py           ← APF tabanlı yönelim ve engelden kaçınma algoritmaları
├── visualization.py        ← Çıktı raporları için yüksek çözünürlüklü statik grafik motoru
└── README.md

```

---

## ⚙️ Kurulum ve Çalıştırma

### Gereksinimler

* Python 3.8 veya üzeri
* pip (Paket Yöneticisi)

### Adım 1 — Depoyu Klonla

```bash
git clone https://github.com/KULLANICI_ADIN/lidar-navigation.git
cd lidar-navigation

```

### Adım 2 — Bağımlılıkları Yükle (Terminal)

```bash
pip install streamlit numpy matplotlib scipy

```

### Adım 3 — Simülasyonu Başlat

Uygulamayı web tarayıcısında (localhost) açmak için aşağıdaki komutu çalıştırın:

```bash
streamlit run main.py

```

> **Not:** Arayüzdeki sol panelden (Sidebar) simülasyon parametrelerini (gürültü, hız, EKF matrisleri vb.) canlı olarak değiştirebilir ve sonuçları eşzamanlı gözlemleyebilirsiniz.

---

## 🧩 Teknik Detaylar ve Matematiksel Modeller

### 1. Robot Kinematiği (Non-Holonomic)

Projede iki farklı non-holonomic sürüş modeli uygulanmıştır:

**Diferansiyel Sürüş Modeli:**


$$x_{k+1} = x_k + v \cdot \cos(\theta_k) \cdot dt$$

$$y_{k+1} = y_k + v \cdot \sin(\theta_k) \cdot dt$$

$$\theta_{k+1} = \theta_k + \omega \cdot dt$$

**Ackermann Sürüş Modeli (Araç Tipi):**


$$x_{k+1} = x_k + v \cdot \cos(\theta_k) \cdot dt$$

$$y_{k+1} = y_k + v \cdot \sin(\theta_k) \cdot dt$$

$$\theta_{k+1} = \theta_k + \frac{v}{L} \cdot \tan(\delta) \cdot dt$$


*(Burada $L$ dingil mesafesini, $\delta$ ise direksiyon açısını temsil eder.)*

### 2. Genişletilmiş Kalman Filtresi (EKF) ile Sensör Füzyonu

Sistem, üç farklı sensörün verisini birleştirerek hatayı minimize eder:

* **Tahmin (Predict) Adımı:** Tekerlek enkoderi ve IMU verileri ile hareket modeli yürütülür. Kovaryans güncellenir:

$$P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k$$


* **Güncelleme (Update) Adımı:** LiDAR'dan gelen (x, y) konum tespiti ile Kalman kazancı hesaplanarak tahmin düzeltilir:

$$K_k = P_{k|k-1} H_k^T (H_k P_{k|k-1} H_k^T + R_k)^{-1}$$



### 3. Navigasyon — Yapay Potansiyel Alan (APF)

Robotun rotası, hedefe olan çekici kuvvet ($F_{att}$) ve engellerin itici kuvvetinin ($F_{rep}$) vektörel toplamı ile belirlenir:


$$F_{toplam} = F_{att} + F_{rep}$$

---

## 📊 Örnek Sonuçlar

Streamlit arayüzünden alınan anlık simülasyon metrikleri (Örneklenen Değerler):

| Metrik | Sensör Füzyonu (EKF) | Sadece Enkoder (Dead Reckoning) |
| --- | --- | --- |
| RMSE (Hata) | **~0.045 m** | ~0.601 m |
| Doğruluk Farkı | **%92 Daha Kesin** | Giderek artan sürüklenme (drift) |

> *Detaylı Hata, Zaman Serisi ve LiDAR (Ham vs Filtreli) grafikleri uygulamanın canlı arayüzünde izlenebilmektedir.*

---

## 🤖 Yapay Zeka Kullanım Beyanı

**Kullanılan yapay zeka araçları:** Gemini)

**Yapay zekanın kullanıldığı bölümler:**

* Orijinal Python scriptlerinin Streamlit arayüzüne entegrasyonu ve modüler mimarinin (OOP) oluşturulması.
* Sensör füzyonu (EKF) ve APF navigasyon algoritmalarının matematiksel denklemlerinin koda dönüştürülmesi.
* Kinematik modellerin (Diferansiyel ve Ackermann) LaTeX formatında raporlanması.

**Öğrencinin kendi katkıları:**

* Proje gereksinimlerine göre "Depo/Palet Taşıma" senaryosunun tasarlanması.
* Simülasyonda kullanılan parametrelerin (Q, R matrisleri, gürültü varyansları) test edilmesi ve optimize edilmesi.
* EKF hatası ile Dead Reckoning sapmalarının karşılaştırmalı analizi ve değerlendirilmesi.

*Açıklama: Yapay zeka aracı bir asistan olarak kod organizasyonu için kullanılmıştır. Çıktılar, formüller ve nihai simülasyon testleri öğrenci tarafından doğrulanarak teslim edilmiştir.*

---

## 📚 Kaynaklar

[1] V. Ušinskis, M. Nowicki, A. Dzedzickis ve V. Bučinskas, "Sensor-fusion based navigation for autonomous mobile robot," *Sensors*, cilt 25, sayı 4, makale 1248, 2025, doi: 10.3390/s25041248.

[2] Y. Ou, Y. Cai, Y. Sun ve T. Qin, "Autonomous navigation by mobile robot with sensor fusion based on deep reinforcement learning," *Sensors*, cilt 24, sayı 12, makale 3895, 2024, doi: 10.3390/s24123895.

[3] B. Zhang ve C. Li, "The optimization and application research of the RRT-APF-based path planning algorithm," *Electronics*, cilt 13, sayı 24, makale 4963, 2024, doi: 10.3390/electronics13244963.