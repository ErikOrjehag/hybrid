# Visualizer for snapped path, guide, ESDF, start & goal
# ------------------------------------------------------
# Usage:
#   python3 visualizer.py [--dir DIR] [--csv snapped.csv]
# Defaults to current directory; expects files written by the C++ demo:
#   - snapped.csv (required)
#   - guide.csv   (optional)
#   - esdf.csv    (optional)
#   - meta.json   (optional)
#
# Plots:
#   1) ESDF heatmap/contours
#   2) Guide path (dashed)
#   3) Snapped path with heading arrows
#   4) Start (green star) & Goal (red X)
#   5) Curvature over index

import os, sys, csv, json, math, argparse
import numpy as np
import matplotlib.pyplot as plt


def load_snapped(path):
    xs, ys, ps, ks = [], [], [], []
    with open(path, 'r', newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            ps.append(float(row['psi']))
            ks.append(float(row['kappa']))
    return np.array(xs), np.array(ys), np.array(ps), np.array(ks)


def load_guide(path):
    if not os.path.exists(path):
        return None, None
    xs, ys = [], []
    with open(path, 'r', newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
    return np.array(xs), np.array(ys)


def load_esdf(path):
    if not os.path.exists(path):
        return None
    xs, ys, ds = [], [], []
    with open(path, 'r', newline='') as f:
        r = csv.DictReader(f)
        for row in r:
            xs.append(float(row['x']))
            ys.append(float(row['y']))
            ds.append(float(row['dist']))
    xs, ys, ds = np.array(xs), np.array(ys), np.array(ds)
    # infer grid resolution/shape
    ux = np.unique(xs)
    uy = np.unique(ys)
    W, H = len(ux), len(uy)
    X = ux.reshape(1, W).repeat(H, axis=0)
    Y = uy.reshape(H, 1).repeat(W, axis=1)
    D = ds.reshape(H, W)
    return X, Y, D


def load_meta(path):
    if not os.path.exists(path):
        return None
    with open(path, 'r') as f:
        return json.load(f)


def plot_xy(ax, X, Y, D, guide_xy, snapped, meta):
    if D is not None:
        # heatmap (dist clipped for visualization)
        clipped = np.clip(D, -0.2, 1.0)
        im = ax.imshow(clipped, origin='lower', extent=[X.min(), X.max(), Y.min(), Y.max()], alpha=0.9)
        cs = ax.contour(X, Y, D, levels=[0.0, 0.2, 0.4, 0.6, 0.8], linewidths=0.8)
        ax.clabel(cs, inline=True, fontsize=8)

    if guide_xy[0] is not None:
        gx, gy = guide_xy
        ax.plot(gx, gy, '--', linewidth=1.5, label='Guide')

    xs, ys, ps, _ = snapped
    ax.plot(xs, ys, '-', linewidth=2.0, label='Snapped')

    step = max(1, len(xs)//40)
    ax.quiver(xs[::step], ys[::step], np.cos(ps[::step]), np.sin(ps[::step]), angles='xy', scale_units='xy', scale=10)

    if meta is not None:
        sx, sy = meta['start']['x'], meta['start']['y']
        gx, gy = meta['goal']['x'],  meta['goal']['y']
        ax.plot(sx, sy, marker='*', markersize=12, linestyle='None', label='Start')
        ax.plot(gx, gy, marker='x', markersize=10, linestyle='None', label='Goal')

    ax.set_aspect('equal', adjustable='box')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('ESDF + Guide + Snapped Path')
    ax.grid(True)
    ax.legend(loc='best')


def plot_curvature(ax, kappa):
    ax.plot(range(len(kappa)), kappa, linewidth=2)
    ax.set_title('Curvature along path (index as arc-length proxy)')
    ax.set_xlabel('sample index')
    ax.set_ylabel('kappa [1/m]')
    ax.grid(True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dir', default='.', help='Directory containing CSV/JSON files')
    ap.add_argument('--csv', default=None, help='Path to snapped.csv (overrides --dir)')
    args = ap.parse_args()

    base = args.dir
    snapped_path = args.csv if args.csv else os.path.join(base, 'snapped.csv')
    guide_path   = os.path.join(base, 'guide.csv')
    esdf_path    = os.path.join(base, 'esdf.csv')
    meta_path    = os.path.join(base, 'meta.json')

    if not os.path.exists(snapped_path):
        print(f"Missing snapped.csv at {snapped_path}")
        sys.exit(1)

    snapped = load_snapped(snapped_path)
    guide_xy = load_guide(guide_path)
    esdf = load_esdf(esdf_path)
    meta = load_meta(meta_path)

    fig1, ax1 = plt.subplots()
    if esdf is not None:
        X, Y, D = esdf
    else:
        X = Y = D = None
    plot_xy(ax1, X, Y, D, guide_xy, snapped, meta)

    fig2, ax2 = plt.subplots()
    plot_curvature(ax2, snapped[3])
    plt.show()


if __name__ == '__main__':
    main()
