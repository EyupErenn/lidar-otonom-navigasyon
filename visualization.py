"""
visualization.py
----------------
Ödev raporu (PDF) için istenen yüksek çözünürlüklü grafikleri üretir.
Dinamik Environment nesnesi ile uyumlu çalışarak hem dairesel hem de 
dikdörtgen engelleri (depo rafları) çizebilir.
"""

import os
import numpy as np
import matplotlib
matplotlib.use('Agg') # Arayüzü dondurmaması için arkaplan (backend) motoru
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.gridspec import GridSpec

OUTPUT_DIR = "outputs"
os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Stil ve Renk Paleti ──────────────────────────────────────────────────────
DARK_BG     = '#0d1117'
PANEL_BG    = '#161b22'
GRID_COLOR  = '#30363d'
TRUE_COLOR  = '#58a6ff'
EKF_COLOR   = '#3fb950'
DR_COLOR    = '#f78166'
PLAN_COLOR  = '#d2a8ff'
LIDAR_RAW   = '#ff7b72'
LIDAR_FILT  = '#79c0ff'
TEXT_COLOR  = '#e6edf3'

plt.rcParams.update({
    'figure.facecolor' : DARK_BG,
    'axes.facecolor'   : PANEL_BG,
    'axes.edgecolor'   : GRID_COLOR,
    'axes.labelcolor'  : TEXT_COLOR,
    'xtick.color'      : TEXT_COLOR,
    'ytick.color'      : TEXT_COLOR,
    'text.color'       : TEXT_COLOR,
    'legend.facecolor' : PANEL_BG,
    'legend.edgecolor' : GRID_COLOR,
    'grid.color'       : GRID_COLOR,
    'grid.linestyle'   : '--',
    'grid.alpha'       : 0.4,
    'font.family'      : 'sans-serif',
})

# ─────────────────────────────────────────────────────────────────────────────
# ORTAK ÇİZİM FONKSİYONLARI
# ─────────────────────────────────────────────────────────────────────────────

def plot_environment(ax, env, show_title=True):
    """Dinamik haritayı, sınırları ve engelleri çizer."""
    ax.set_xlim(0, env.map_size[0])
    ax.set_ylim(0, env.map_size[1])
    ax.set_aspect('equal')
    
    # Duvarlar (Sınırlar)
    wall = patches.Rectangle((0, 0), env.map_size[0], env.map_size[1],
                             linewidth=2, edgecolor='#e94560',
                             facecolor='none', zorder=1)
    ax.add_patch(wall)

    # Engeller (Daire ve Dikdörtgen desteği)
    for i, obs in enumerate(env.obstacles):
        if obs['type'] == 'circle':
            cx, cy, cr = obs['params']
            patch = patches.Circle((cx, cy), cr, color='#e94560', alpha=0.85, zorder=2)
            ax.add_patch(patch)
            ax.text(cx, cy, f'E{i+1}', ha='center', va='center',
                    fontsize=7, color='white', fontweight='bold', zorder=3)
        elif obs['type'] == 'rectangle':
            rx, ry, rw, rh = obs['params']
            patch = patches.Rectangle((rx, ry), rw, rh, color='#e94560', alpha=0.85, zorder=2)
            ax.add_patch(patch)
            ax.text(rx + rw/2, ry + rh/2, f'E{i+1}', ha='center', va='center',
                    fontsize=7, color='white', fontweight='bold', zorder=3)

    # Başlangıç ve Hedef
    ax.plot(*env.start, marker='o', markersize=10, color='#00d4aa', label='Başlangıç', zorder=5)
    ax.plot(*env.goal,  marker='*', markersize=14, color='#ffd700', label='Hedef', zorder=5)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.grid(True)
    if show_title:
        ax.set_title(f'{env.name} — Ortam Haritası', color=TEXT_COLOR, fontsize=13)
    ax.legend(loc='upper left', facecolor=PANEL_BG, labelcolor='white')

def _draw_obstacles_mini(ax, env):
    """LiDAR ve küçük grafikler için basitleştirilmiş engel çizimi."""
    for obs in env.obstacles:
        if obs['type'] == 'circle':
            cx, cy, cr = obs['params']
            ax.add_patch(patches.Circle((cx, cy), cr, color='#e94560', alpha=0.5))
        elif obs['type'] == 'rectangle':
            rx, ry, rw, rh = obs['params']
            ax.add_patch(patches.Rectangle((rx, ry), rw, rh, color='#e94560', alpha=0.5))

# ─────────────────────────────────────────────────────────────────────────────
# GRAFİK ÜRETİM (RAPOR İÇİN)
# ─────────────────────────────────────────────────────────────────────────────

def plot_map(env):
    fig, ax = plt.subplots(figsize=(8, 8), facecolor=DARK_BG)
    plot_environment(ax, env)
    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, "01_environment_map.png")
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK_BG)
    plt.close()
    return path

def plot_path(results, env):
    true_pos = results['true_positions']
    ekf_pos  = results['ekf_positions']
    
    fig, ax = plt.subplots(figsize=(9, 9), facecolor=DARK_BG)
    plot_environment(ax, env, show_title=False)

    ex = [p[0] for p in ekf_pos]
    ey = [p[1] for p in ekf_pos]
    ax.plot(ex, ey, '-', color=EKF_COLOR, linewidth=1.5, label='EKF Tahmini Yol', alpha=0.85, zorder=5)

    tx = [p[0] for p in true_pos]
    ty = [p[1] for p in true_pos]
    ax.plot(tx, ty, '-', color=TRUE_COLOR, linewidth=2.5, label='Gerçek Yol', zorder=6)

    ax.set_title('Robot Yol Planı — Gerçek ve EKF Tahmini', color=TEXT_COLOR, fontsize=13, pad=12)
    ax.legend(loc='upper left')

    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, "02_path_plan.png")
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK_BG)
    plt.close()
    return path

def plot_lidar(scan_data, env):
    if not scan_data:
        return None

    fig = plt.figure(figsize=(16, 7), facecolor=DARK_BG)
    gs  = GridSpec(1, 3, figure=fig, wspace=0.35)

    angles = scan_data['angles']
    raw    = scan_data['raw']
    filt   = scan_data['filtered']
    rx, ry, rtheta = scan_data['robot_pos']

    # 1. Polar Görünüm
    ax_polar = fig.add_subplot(gs[0], polar=True, facecolor=PANEL_BG)
    ax_polar.plot(angles, raw,  color=LIDAR_RAW,  linewidth=0.7, alpha=0.6, label='Ham Veri')
    ax_polar.plot(angles, filt, color=LIDAR_FILT, linewidth=1.2, alpha=0.9, label='Filtrelenmiş')
    ax_polar.set_title('LiDAR Polar Tarama', color=TEXT_COLOR, fontsize=11, pad=15)
    ax_polar.tick_params(colors=TEXT_COLOR)
    ax_polar.legend(loc='upper right', fontsize=8, bbox_to_anchor=(1.3, 1.1))

    # 2. Kartezyen Ham
    ax_raw = fig.add_subplot(gs[1], facecolor=PANEL_BG)
    raw_x = rx + raw * np.cos(angles)
    raw_y = ry + raw * np.sin(angles)
    ax_raw.scatter(raw_x, raw_y, s=1.5, c=LIDAR_RAW, alpha=0.6)
    ax_raw.plot(rx, ry, 'o', color='yellow', markersize=8, label='Robot')
    _draw_obstacles_mini(ax_raw, env)
    ax_raw.set_xlim(0, env.map_size[0])
    ax_raw.set_ylim(0, env.map_size[1])
    ax_raw.set_title('Ham LiDAR Verisi', color=TEXT_COLOR, fontsize=11)
    ax_raw.set_aspect('equal')

    # 3. Kartezyen Filtrelenmiş
    ax_filt = fig.add_subplot(gs[2], facecolor=PANEL_BG)
    filt_x = rx + filt * np.cos(angles)
    filt_y = ry + filt * np.sin(angles)
    ax_filt.scatter(filt_x, filt_y, s=1.5, c=LIDAR_FILT, alpha=0.8)
    ax_filt.plot(rx, ry, 'o', color='yellow', markersize=8, label='Robot')
    _draw_obstacles_mini(ax_filt, env)
    ax_filt.set_xlim(0, env.map_size[0])
    ax_filt.set_ylim(0, env.map_size[1])
    ax_filt.set_title('Filtrelenmiş LiDAR Verisi', color=TEXT_COLOR, fontsize=11)
    ax_filt.set_aspect('equal')

    fig.suptitle('LiDAR Sensör Görselleştirmesi — Ham vs Filtrelenmiş', color=TEXT_COLOR, fontsize=13, y=1.01)
    path = os.path.join(OUTPUT_DIR, "03_lidar_visualization.png")
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK_BG)
    plt.close()
    return path

def plot_localization(results, env):
    t    = np.array(results['time_steps'])
    tx   = np.array([p[0] for p in results['true_positions']])
    ty   = np.array([p[1] for p in results['true_positions']])
    ex   = np.array([p[0] for p in results['ekf_positions']])
    ey   = np.array([p[1] for p in results['ekf_positions']])
    dx   = np.array([p[0] for p in results['dr_positions']])
    dy   = np.array([p[1] for p in results['dr_positions']])

    fig, axes = plt.subplots(2, 1, figsize=(10, 8), facecolor=DARK_BG, sharex=True)

    axes[0].plot(t, tx, color=TRUE_COLOR, linewidth=2,   label='Gerçek x')
    axes[0].plot(t, ex, color=EKF_COLOR,  linewidth=1.5, label='EKF x',  linestyle='--')
    axes[0].plot(t, dx, color=DR_COLOR,   linewidth=1.0, label='DR x',   linestyle=':', alpha=0.7)
    axes[0].set_ylabel('x (m)')
    axes[0].set_title('Lokalizasyon Karşılaştırması — X Ekseni', color=TEXT_COLOR)
    axes[0].legend()
    axes[0].grid(True)

    axes[1].plot(t, ty, color=TRUE_COLOR, linewidth=2,   label='Gerçek y')
    axes[1].plot(t, ey, color=EKF_COLOR,  linewidth=1.5, label='EKF y',  linestyle='--')
    axes[1].plot(t, dy, color=DR_COLOR,   linewidth=1.0, label='DR y',   linestyle=':', alpha=0.7)
    axes[1].set_ylabel('y (m)')
    axes[1].set_xlabel('Zaman Adımı')
    axes[1].set_title('Lokalizasyon Karşılaştırması — Y Ekseni', color=TEXT_COLOR)
    axes[1].legend()
    axes[1].grid(True)

    for ax in axes:
        ax.set_facecolor(PANEL_BG)

    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, "04_localization.png")
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK_BG)
    plt.close()
    return path

def plot_error(results):
    t       = np.array(results['time_steps'])
    err_ekf = np.array(results['err_ekf'])
    err_dr  = np.array(results['err_dr'])
    
    # NaN değerleri önlemek için güvenli ortalama (RMSE/MAE) hesaplama
    rmse_ekf = np.sqrt(np.mean(err_ekf**2)) if len(err_ekf) > 0 else 0
    rmse_dr  = np.sqrt(np.mean(err_dr**2)) if len(err_dr) > 0 else 0

    fig, ax = plt.subplots(figsize=(10, 5), facecolor=DARK_BG)
    ax.plot(t, err_ekf, color=EKF_COLOR, linewidth=1.5, label=f'EKF Hatası (RMSE: {rmse_ekf:.3f}m)')
    ax.plot(t, err_dr,  color=DR_COLOR,  linewidth=1.5, label=f'DR Hatası (RMSE: {rmse_dr:.3f}m)', linestyle='--', alpha=0.8)
    
    ax.set_ylabel('Konum Hatası (m)')
    ax.set_xlabel('Zaman Adımı')
    ax.set_title('Zaman Serisi Hata Analizi (RMSE)', color=TEXT_COLOR)
    ax.legend()
    ax.grid(True)
    ax.set_facecolor(PANEL_BG)

    plt.tight_layout()
    path = os.path.join(OUTPUT_DIR, "05_error_analysis.png")
    plt.savefig(path, dpi=150, bbox_inches='tight', facecolor=DARK_BG)
    plt.close()
    return path