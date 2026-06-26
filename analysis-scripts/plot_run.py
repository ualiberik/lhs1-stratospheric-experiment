"""
Plot burst-means data with Newton's law cooling fit.

Usage:
    python plot_run.py <burst_means_csv> <fit_results_txt>

Output: two PNG files saved next to the CSV.
"""

import csv
import os
import re
import sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Load data ─────────────────────────────────────────────────────────────────

def load_csv(path):
    bursts, t, T_bmp, T_mcp, phases = [], [], [], [], []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            bursts.append(int(row['burst']))
            t.append(float(row['t_s']))
            T_bmp.append(float(row['T_bmp_C']))
            T_mcp.append(float(row['T_mcp_C']))
            phases.append(row['phase'])
    return (np.array(bursts), np.array(t), np.array(T_bmp),
            np.array(T_mcp), phases)


def load_fit(path):
    """Parse fit-results.txt for T_amb, A, tau for both sensors."""
    text = open(path).read()

    def grab(pattern):
        m = re.search(pattern, text)
        return float(m.group(1)) if m else None

    return {
        'T_amb_bmp': grab(r'T_amb_bmp\s*=\s*([\d.]+)'),
        'T_amb_mcp': grab(r'T_amb_mcp\s*=\s*([\d.]+)'),
        'A_bmp':     grab(r'BMP388[^\n]*A\s*=\s*([\d.]+)'),
        'tau_bmp':   grab(r'BMP388[^\n]*tau\s*=\s*([\d.]+)'),
        'err_bmp':   grab(r'BMP388[^\n]*\(\+-\s*([\d.]+)'),
        'A_mcp':     grab(r'MCP9808[^\n]*A\s*=\s*([\d.]+)'),
        'tau_mcp':   grab(r'MCP9808[^\n]*tau\s*=\s*([\d.]+)'),
        'err_mcp':   grab(r'MCP9808[^\n]*\(\+-\s*([\d.]+)'),
        'R':         grab(r'R\s*=\s*tau_mcp\s*/\s*tau_bmp\s*=\s*[\d.]+\s*/\s*[\d.]+\s*=\s*([\d.]+)'),
        'peak_burst': int(grab(r'Peak burst\s*:\s*(\d+)') or 0),
    }


# ── Plots ─────────────────────────────────────────────────────────────────────

BLUE  = '#2563EB'
RED   = '#DC2626'
LBLUE = '#93C5FD'
LRED  = '#FCA5A5'
GRAY  = '#6B7280'


def plot_full_series(bursts, t, T_bmp, T_mcp, phases, fit, stem, out_dir):
    peak_idx = np.where(bursts == fit['peak_burst'])[0][0]
    t_peak   = t[peak_idx]

    fig, ax = plt.subplots(figsize=(12, 5))

    # shaded regions
    ax.axvspan(t[0]/60,        t_peak/60,    alpha=0.06, color='orange', label='_nolegend_')
    ax.axvspan(t_peak/60,      t[-1]/60+2,   alpha=0.06, color='steelblue', label='_nolegend_')

    # data
    ax.plot(t/60, T_bmp, 'o-', color=BLUE, ms=4, lw=1.4, label='BMP388 (external)')
    ax.plot(t/60, T_mcp, 's-', color=RED,  ms=4, lw=1.4, label='MCP9808 (under regolith)')

    ax.axvline(t_peak/60, color=GRAY, lw=1, ls='--')
    ax.text(t_peak/60 + 0.3, T_bmp.max() + 0.5, 'peak', fontsize=9, color=GRAY)

    # T_amb lines
    ax.axhline(fit['T_amb_bmp'], color=BLUE, lw=0.8, ls=':', alpha=0.6)
    ax.axhline(fit['T_amb_mcp'], color=RED,  lw=0.8, ls=':', alpha=0.6)

    ax.set_xlabel('Time (min)', fontsize=11)
    ax.set_ylabel('Temperature (°C)', fontsize=11)
    ax.set_title(f'{stem} — full time series\n'
                 f'τ_bmp = {fit["tau_bmp"]:.0f} s, '
                 f'τ_mcp = {fit["tau_mcp"]:.0f} s, '
                 f'R = {fit["R"]:.3f}', fontsize=11)
    ax.legend(fontsize=10)
    ax.grid(True, alpha=0.3)

    orange_patch = mpatches.Patch(color='orange', alpha=0.3, label='Warming')
    blue_patch   = mpatches.Patch(color='steelblue', alpha=0.3, label='Cooling')
    ax.legend(handles=ax.get_legend_handles_labels()[0] + [orange_patch, blue_patch],
              labels=ax.get_legend_handles_labels()[1] + ['Warming', 'Cooling'],
              fontsize=9)

    fig.tight_layout()
    out = os.path.join(out_dir, f'{stem}_plot-series.png')
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f'Saved {out}')


def plot_cooling_fit(bursts, t, T_bmp, T_mcp, phases, fit, stem, out_dir):
    cool_mask = np.array([p == 'cooling' for p in phases])
    t_cool  = t[cool_mask]
    b_cool  = T_bmp[cool_mask]
    m_cool  = T_mcp[cool_mask]
    t_rel   = t_cool - t_cool[0]   # seconds since peak

    # fit curves
    t_fine = np.linspace(0, t_rel[-1] * 1.05, 500)
    curve_bmp = fit['T_amb_bmp'] + fit['A_bmp'] * np.exp(-t_fine / fit['tau_bmp'])
    curve_mcp = fit['T_amb_mcp'] + fit['A_mcp'] * np.exp(-t_fine / fit['tau_mcp'])

    fig, ax = plt.subplots(figsize=(11, 5))

    # data points
    ax.scatter(t_rel/60, b_cool, color=BLUE, s=25, zorder=5, label='BMP388 (data)')
    ax.scatter(t_rel/60, m_cool, color=RED,  s=25, zorder=5, label='MCP9808 (data)')

    # fit curves
    ax.plot(t_fine/60, curve_bmp, color=BLUE, lw=2,
            label=f'BMP388 fit: τ={fit["tau_bmp"]:.0f}±{fit["err_bmp"]:.0f} s')
    ax.plot(t_fine/60, curve_mcp, color=RED,  lw=2,
            label=f'MCP9808 fit: τ={fit["tau_mcp"]:.0f}±{fit["err_mcp"]:.0f} s')

    # T_amb lines
    ax.axhline(fit['T_amb_bmp'], color=BLUE, lw=0.8, ls=':', alpha=0.5,
               label=f'T_amb BMP = {fit["T_amb_bmp"]:.1f} °C')
    ax.axhline(fit['T_amb_mcp'], color=RED,  lw=0.8, ls=':', alpha=0.5,
               label=f'T_amb MCP = {fit["T_amb_mcp"]:.1f} °C')

    ax.set_xlabel('Time since peak (min)', fontsize=11)
    ax.set_ylabel('Temperature (°C)', fontsize=11)
    ax.set_title(f'{stem} — Newton\'s law cooling fit\n'
                 f'R = τ_mcp / τ_bmp = {fit["tau_mcp"]:.0f} / {fit["tau_bmp"]:.0f} = {fit["R"]:.3f}',
                 fontsize=11)
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3)

    fig.tight_layout()
    out = os.path.join(out_dir, f'{stem}_plot-cooling.png')
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f'Saved {out}')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 3:
        print('Usage: python plot_run.py <burst_means_csv> <fit_results_txt>')
        sys.exit(1)

    csv_path = os.path.abspath(sys.argv[1])
    txt_path = os.path.abspath(sys.argv[2])
    out_dir  = os.path.normpath(os.path.join(os.path.dirname(csv_path), '..', 'plots'))
    os.makedirs(out_dir, exist_ok=True)
    stem     = os.path.splitext(os.path.basename(csv_path))[0].replace('_burst-means', '')

    bursts, t, T_bmp, T_mcp, phases = load_csv(csv_path)
    fit = load_fit(txt_path)

    plot_full_series(bursts, t, T_bmp, T_mcp, phases, fit, stem, out_dir)
    plot_cooling_fit(bursts, t, T_bmp, T_mcp, phases, fit, stem, out_dir)


if __name__ == '__main__':
    main()
