"""
Turn odom_test runs into report material.

  ros2 run uet_amr_evaluation odom_report ~/amr_odom_tests
  ros2 run uet_amr_evaluation odom_report ~/amr_odom_tests --out ~/report_figs

Writes into <out> (default: <runs dir>/report):
  <run>.png        per-run figure (trajectory, progress, speed vs time)
  summary.png      error per test, reported vs measured
  summary.csv      one row per run, ready for a table
  summary.md       the same as a markdown table
Error is computed against the *measured* value in meta.json when present
(otherwise against the commanded target and flagged as such).
"""
import argparse
import csv
import json
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

# colorblind-safe, one colour per odometry source
COLORS = {'wheel': '#0072B2', 'ekf': '#D55E00', 'gyro': '#009E73'}
plt.rcParams.update({'axes.grid': True, 'grid.alpha': 0.3, 'figure.dpi': 120,
                     'axes.spines.top': False, 'axes.spines.right': False})


def load_runs(root):
    runs = []
    for name in sorted(os.listdir(root)):
        d = os.path.join(root, name)
        if not os.path.isfile(os.path.join(d, 'meta.json')):
            continue
        with open(os.path.join(d, 'meta.json')) as f:
            meta = json.load(f)
        data = np.genfromtxt(os.path.join(d, 'data.csv'), delimiter=',', names=True)
        runs.append((name, meta, data))
    return runs


def finals(meta):
    """Reported values {source: value} in the test's unit (m or deg)."""
    s = meta['summary']
    if meta['kind'] == 'straight':
        return {'wheel': s.get('wheel_reported'), 'ekf': s.get('ekf_reported')}
    return {'wheel': s.get('wheel_reported_deg'), 'ekf': s.get('ekf_reported_deg'),
            'gyro': s.get('gyro_reported_deg')}


def reference(meta):
    """Ground truth: the measured value, falling back to the commanded one."""
    if meta.get('measured') is not None:
        return meta['measured'], True
    return meta['target'], False


def plot_run(name, meta, d, path):
    straight = meta['kind'] == 'straight'
    t = d['t']
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    title = f"{meta['kind']} {meta['target']:g} {meta['unit']}"
    if meta.get('label'):
        title += f"  ({meta['label']})"
    fig.suptitle(title)

    # trajectory in the start frame
    for src in ('wheel', 'ekf'):
        x, y = d[f'{src}_x'], d[f'{src}_y']
        if np.isfinite(x).any() and np.ptp(x) + np.ptp(y) > 0:
            ax[0].plot(x, y, color=COLORS[src], label=src)
    ax[0].set_xlabel('x forward [m]')
    ax[0].set_ylabel('y left [m]')
    if not straight:
        ax[0].set_aspect('equal', adjustable='datalim')
    if ax[0].lines:
        ax[0].legend()
    ax[0].set_title('trajectory (start frame, y exaggerated)' if straight else 'trajectory (start frame)')

    # progress over time
    if straight:
        for src in ('wheel', 'ekf'):
            ax[1].plot(t, d[f'{src}_x'], color=COLORS[src], label=src)
        ax[1].set_ylabel('distance [m]')
    else:
        for src in ('wheel', 'ekf', 'gyro'):
            ax[1].plot(t, np.degrees(d[f'{src}_yaw']), color=COLORS[src], label=src)
        ax[1].set_ylabel('rotation [deg]')
    ref, measured = reference(meta)
    ax[1].axhline(ref, color='k', ls='--', lw=1, label='measured' if measured else 'target')
    ax[1].set_xlabel('time [s]')
    ax[1].legend()
    ax[1].set_title('progress')

    # commanded vs measured speed
    key = 'v' if straight else 'w'
    ax[2].plot(t, d["cmd"], color='0.6', ls='--', label='command')
    for src in ('wheel', 'ekf'):
        ax[2].plot(t, d[f'{src}_{key}'], color=COLORS[src], label=src)
    ax[2].set_xlabel('time [s]')
    ax[2].set_ylabel('speed [m/s]' if straight else 'rate [rad/s]')
    ax[2].legend()
    ax[2].set_title('speed')

    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def rows_for(runs):
    out = []
    for name, meta, _ in runs:
        ref, measured = reference(meta)
        row = {'run': name, 'kind': meta['kind'], 'target': meta['target'],
               'unit': meta['unit'], 'reference': ref,
               'reference_is_measured': measured, 'label': meta.get('label', '')}
        for src, val in finals(meta).items():
            row[f'{src}_reported'] = val
            if val is not None:
                row[f'{src}_error'] = val - ref
                row[f'{src}_error_pct'] = 100.0 * (val - ref) / ref if ref else float('nan')
        s = meta['summary']
        if meta['kind'] == 'straight':
            row['lateral_drift_m'] = s.get('wheel_lateral')
            row['heading_drift_deg'] = s.get('wheel_heading_drift_deg')
        else:
            row['position_drift_m'] = s.get('wheel_position_drift')
        row['duration_s'] = s.get('duration_s')
        out.append(row)
    return out


def write_tables(rows, out):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(os.path.join(out, 'summary.csv'), 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=keys)
        wr.writeheader()
        wr.writerows(rows)

    def fmt(v, p=2):
        return '-' if v is None or (isinstance(v, float) and math.isnan(v)) else f'{v:.{p}f}'

    lines = ['| test | reference | wheel | err % | ekf | err % | gyro | err % | drift |',
             '|---|---|---|---|---|---|---|---|---|']
    for r in rows:
        ref = f"{fmt(r['reference'])} {r['unit']}" + ('' if r['reference_is_measured'] else ' (target)')
        drift = (f"lat {fmt(r.get('lateral_drift_m'))} m, hdg {fmt(r.get('heading_drift_deg'), 1)}°"
                 if r['kind'] == 'straight' else f"{fmt(r.get('position_drift_m'))} m")
        lines.append(
            f"| {r['kind']} {r['target']:g} {r['unit']} | {ref} "
            f"| {fmt(r.get('wheel_reported'))} | {fmt(r.get('wheel_error_pct'))} "
            f"| {fmt(r.get('ekf_reported'))} | {fmt(r.get('ekf_error_pct'))} "
            f"| {fmt(r.get('gyro_reported'))} | {fmt(r.get('gyro_error_pct'))} | {drift} |")
    with open(os.path.join(out, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines) + '\n')


def plot_summary(rows, path):
    kinds = [k for k in ('straight', 'rotate') if any(r['kind'] == k for r in rows)]
    fig, axes = plt.subplots(1, len(kinds), figsize=(7 * len(kinds), 4.5), squeeze=False)
    for ax, kind in zip(axes[0], kinds):
        rs = sorted((r for r in rows if r['kind'] == kind), key=lambda r: abs(r['target']))
        labels = [f"{r['target']:g} {r['unit']}" for r in rs]
        srcs = [s for s in ('wheel', 'ekf', 'gyro')
                if any(r.get(f'{s}_error_pct') is not None for r in rs)]
        w = 0.8 / max(len(srcs), 1)
        x = np.arange(len(rs))
        for i, s in enumerate(srcs):
            vals = [r.get(f'{s}_error_pct', np.nan) for r in rs]
            ax.bar(x + i * w - 0.4 + w / 2, vals, w, color=COLORS[s], label=s)
        ax.axhline(0, color='k', lw=0.8)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=30, ha='right')
        ax.set_ylabel('odometry error vs reference [%]')
        ax.set_title('straight-line distance error' if kind == 'straight' else 'rotation error')
        ax.legend()
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('runs_dir', nargs='?', default=os.path.expanduser('~/amr_odom_tests'))
    p.add_argument('--out')
    args = p.parse_args()

    out = args.out or os.path.join(args.runs_dir, 'report')
    os.makedirs(out, exist_ok=True)
    runs = load_runs(args.runs_dir)
    if not runs:
        print(f'no runs found in {args.runs_dir}')
        return
    for name, meta, data in runs:
        plot_run(name, meta, data, os.path.join(out, f'{name}.png'))
    rows = rows_for(runs)
    write_tables(rows, out)
    plot_summary(rows, os.path.join(out, 'summary.png'))
    unmeasured = sum(not r['reference_is_measured'] for r in rows)
    print(f'{len(runs)} runs -> {out}')
    if unmeasured:
        print(f'note: {unmeasured} run(s) have no measured value; errors are vs commanded target')


if __name__ == '__main__':
    main()
