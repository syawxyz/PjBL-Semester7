#!/usr/bin/env python3
"""Gambar jalur Dijkstra, A*, dan Theta* (data/jalur_*.csv) di atas peta turtlebot3_world.

Jalankan dari folder repo setelah ROS 2 di-source:
    python3 scripts/plot_jalur.py
Hasil: images/perbandingan-planner.png
"""
import csv
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from matplotlib.colors import BoundaryNorm, ListedColormap  # noqa: E402
from PIL import Image  # noqa: E402

SURFACE, INK, INK_2, MUTED = '#fcfcfb', '#0b0b0b', '#52514e', '#8a8984'
SERIES = [
    ('dijkstra', 'Dijkstra (NavFn, bawaan Nav2)', '#2a78d6', '-'),
    ('astar', 'A* (NavFn, use_astar: true)', '#eb6834', '--'),
    ('thetastar', 'Theta* (ThetaStarPlanner)', '#1baf7a', ':'),
]
START, GOAL = (-2.0, -0.5), (1.5, 0.5)


def main():
    map_yaml = os.path.join(get_package_share_directory('nav2_bringup'), 'maps', 'turtlebot3_world.yaml')
    meta = yaml.safe_load(open(map_yaml))
    img = np.array(Image.open(os.path.join(os.path.dirname(map_yaml), meta['image'])))
    h, w = img.shape
    res = meta['resolution']
    ox, oy = meta['origin'][:2]
    cells = np.where(img < 100, 0, np.where(img > 250, 2, 1))  # 0 halangan, 1 unknown, 2 bebas

    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10})
    fig, ax = plt.subplots(figsize=(8, 5.6), dpi=150)
    fig.patch.set_facecolor(SURFACE)
    ax.set_facecolor(SURFACE)
    ax.imshow(cells, cmap=ListedColormap(['#a3a29d', '#eeede9', SURFACE]),
              norm=BoundaryNorm([-0.5, 0.5, 1.5, 2.5], 3),
              extent=[ox, ox + w * res, oy, oy + h * res], origin='upper', interpolation='nearest')

    for key, label, color, style in SERIES:
        pts = [(float(r['x']), float(r['y'])) for r in csv.DictReader(open(f'data/jalur_{key}.csv'))]
        length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color=SURFACE, lw=4.2, solid_capstyle='round', zorder=3)
        ax.plot(xs, ys, color=color, lw=2.2, ls=style, solid_capstyle='round',
                dash_capstyle='round', zorder=4,
                label=f'{label}  —  {length:.2f} m'.replace('.', ','))

    for (x, y), text, ha in [(START, 'Start', 'right'), (GOAL, 'Goal', 'left')]:
        ax.plot(x, y, 'o', ms=9, color=INK, mec=SURFACE, mew=2, zorder=5)
        ax.annotate(text, (x, y), xytext=(-10 if ha == 'right' else 10, 0), textcoords='offset points',
                    ha=ha, va='center', color=INK, fontsize=10, fontweight='bold', zorder=6)

    ax.set_xlim(-2.75, 2.35)
    ax.set_ylim(-1.75, 1.85)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)', color=INK_2)
    ax.set_ylabel('y (m)', color=INK_2)
    ax.tick_params(colors=MUTED, labelcolor=INK_2, length=3)
    for spine in ax.spines.values():
        spine.set_color('#d6d5d0')
    ax.set_title('Perbandingan jalur global: Dijkstra, A*, dan Theta*', loc='left',
                 color=INK, fontsize=12, fontweight='bold', pad=26)
    ax.text(0, 1.015, 'Peta turtlebot3_world · start (−2,0; −0,5) → goal (1,5; 0,5) · abu-abu = halangan',
            transform=ax.transAxes, color=INK_2, fontsize=9, va='bottom')
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.11), frameon=False, labelcolor=INK, handlelength=3.2)
    fig.tight_layout()
    fig.savefig('images/perbandingan-planner.png', facecolor=SURFACE)


if __name__ == '__main__':
    main()
