"""
Burst cooling analysis — Newton's law fit for any experiment run.

Usage:
    python cooling_analysis.py <log_file> <sleep_s>

Arguments:
    log_file   Path to raw .log file
    sleep_s    SLEEP_TIME value used during data collection (seconds)

Output (written next to the log file in ../processed/):
    <log_stem>_burst-means.csv
    <log_stem>_fit-results.txt
"""

import csv
import os
import sys
import numpy as np
from scipy.optimize import curve_fit

AWAKE_S      = 60       # fixed in firmware
COL_BMP      = 2        # BMP388 temperature column (0-based)
COL_MCP      = 6        # MCP9808 temperature column (0-based)
SEPARATOR    = '00000000-000000'
MIN_EXCESS_C = 0.10     # minimum excess above T_amb to count a burst as meaningful


# ── Parsing ───────────────────────────────────────────────────────────────────

def parse_log(path):
    """Return (bursts, n_separators).
    bursts          — list of non-empty bursts, each a list of (bmp, mcp) floats
    n_separators    — total separator lines in file (includes empty wakeups)
    """
    bursts, current = [], []
    n_separators = 0
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('<<<'):
                continue
            if SEPARATOR in line:
                n_separators += 1
                if current:
                    bursts.append(current)
                current = []
                continue
            parts = line.split(',')
            if len(parts) < max(COL_BMP, COL_MCP) + 1:
                continue
            try:
                bmp = float(parts[COL_BMP])
                mcp = float(parts[COL_MCP])
                if -100 <= bmp <= 200 and -100 <= mcp <= 200:
                    current.append((bmp, mcp))
            except ValueError:
                continue
    if current:
        bursts.append(current)
    return bursts, n_separators


def burst_mean(burst):
    arr = np.array(burst)
    return arr[:, 0].mean(), arr[:, 1].mean()


# ── Fitting ───────────────────────────────────────────────────────────────────

def fit_newton(t, T, T_amb, cycle_s):
    """Fit T(t) = T_amb + A*exp(-t/tau). Returns (popt, perr) or (None, None)."""
    def model(t, A, tau):
        return T_amb + A * np.exp(-t / tau)

    if T[0] - T_amb <= 0:
        return None, None
    try:
        popt, pcov = curve_fit(
            model, t, T,
            p0=[T[0] - T_amb, cycle_s * 2],
            bounds=([0, 1], [500, 100000]),
            maxfev=5000
        )
        return popt, np.sqrt(np.diag(pcov))
    except (RuntimeError, ValueError):
        return None, None


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) < 3:
        print("Usage: python cooling_analysis.py <log_file> <sleep_s>")
        sys.exit(1)

    log_path = os.path.abspath(sys.argv[1])
    sleep_s  = int(sys.argv[2])
    cycle_s  = AWAKE_S + sleep_s

    log_stem = os.path.splitext(os.path.basename(log_path))[0]
    out_dir  = os.path.normpath(os.path.join(os.path.dirname(log_path), '..', 'processed'))
    os.makedirs(out_dir, exist_ok=True)

    csv_path = os.path.join(out_dir, f'{log_stem}_burst-means.csv')
    txt_path = os.path.join(out_dir, f'{log_stem}_fit-results.txt')

    # ── Parse ─────────────────────────────────────────────────────────────────
    bursts, n_sep = parse_log(log_path)
    n_empty = n_sep - len(bursts)

    print(f"Separators : {n_sep}  |  Non-empty bursts : {len(bursts)}  |  Empty : {n_empty}")

    if not bursts:
        print("No data found — aborting.")
        sys.exit(1)

    all_means   = [burst_mean(b) for b in bursts]
    T_bmp_all   = np.array([m[0] for m in all_means])
    T_mcp_all   = np.array([m[1] for m in all_means])

    # Auto-detect peak: cooling starts at the hottest burst
    peak_idx     = int(np.argmax(T_bmp_all))
    cool_means   = all_means[peak_idx:]
    cool_bursts  = bursts[peak_idx:]

    T_bmp = np.array([m[0] for m in cool_means])
    T_mcp = np.array([m[1] for m in cool_means])
    t_arr = np.array([k * cycle_s for k in range(len(cool_means))], dtype=float)

    T_amb_bmp = T_bmp.min()
    T_amb_mcp = T_mcp.min()
    exc_bmp   = T_bmp - T_amb_bmp
    exc_mcp   = T_mcp - T_amb_mcp
    n_meaningful = int(((exc_bmp > MIN_EXCESS_C) | (exc_mcp > MIN_EXCESS_C)).sum())

    # ── Burst means CSV ───────────────────────────────────────────────────────
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['burst', 't_s', 'T_bmp_C', 'T_mcp_C', 'phase', 'n_samples'])
        for k, (m, b) in enumerate(zip(all_means, bursts)):
            phase = 'warming' if k < peak_idx else 'cooling'
            w.writerow([k, k * cycle_s, f'{m[0]:.4f}', f'{m[1]:.4f}', phase, len(b)])
    print(f"Saved {csv_path}")

    # ── Fit ───────────────────────────────────────────────────────────────────
    fit_bmp, err_bmp = fit_newton(t_arr, T_bmp, T_amb_bmp, cycle_s)
    fit_mcp, err_mcp = fit_newton(t_arr, T_mcp, T_amb_mcp, cycle_s)

    bmp_cooled_out = (len(exc_bmp) > 1 and exc_bmp[1] <= MIN_EXCESS_C)

    tau_bmp = fit_bmp[1] if fit_bmp is not None and not bmp_cooled_out else None
    tau_mcp = fit_mcp[1] if fit_mcp is not None else None

    # ── Fit results TXT ───────────────────────────────────────────────────────
    lines = []
    lines.append(f"Cooling Analysis: {log_stem}")
    lines.append("=" * 60)
    lines.append(f"Cycle          : {cycle_s} s  (awake {AWAKE_S} s + sleep {sleep_s} s)")
    lines.append(f"Total separators : {n_sep}")
    lines.append(f"Non-empty bursts : {len(bursts)}")

    if n_empty > 0:
        lines.append(f"Empty bursts     : {n_empty}  *** ANOMALY ***")
        lines.append("  Device wrote separators without data for these wakeup cycles.")
        lines.append("  Data before the anomaly is unaffected.")

    lines.append(f"Peak burst     : {peak_idx}  (T_bmp = {T_bmp_all[peak_idx]:.2f} C)")
    if peak_idx > 0:
        lines.append(f"Warming bursts : 0 to {peak_idx - 1}")
    lines.append(f"Cooling bursts : {peak_idx} to {len(bursts) - 1}  ({len(cool_means)} total)")
    lines.append(f"Meaningful (excess > {MIN_EXCESS_C} C) : {n_meaningful}")
    lines.append("")

    lines.append(f"T_amb_bmp = {T_amb_bmp:.3f} C  (min observed)")
    lines.append(f"T_amb_mcp = {T_amb_mcp:.3f} C  (min observed)")
    if T_bmp[-1] - T_amb_bmp > 1.0:
        lines.append("NOTE: cube did not reach ambient — T_amb may be above true ambient.")
        lines.append("      Tau values are lower bounds.")
    lines.append("")

    if fit_mcp is not None:
        lines.append(f"MCP9808 : A = {fit_mcp[0]:.2f} C,  tau = {tau_mcp:.0f} s  (+-{err_mcp[1]:.0f} s)")
    else:
        lines.append("MCP9808 : fit did not converge.")

    if bmp_cooled_out:
        lines.append(f"BMP388  : cooled within first sleep window ({sleep_s} s) — tau indeterminate.")
    elif fit_bmp is not None:
        lines.append(f"BMP388  : A = {fit_bmp[0]:.2f} C,  tau = {tau_bmp:.0f} s  (+-{err_bmp[1]:.0f} s)")
    else:
        lines.append("BMP388  : fit did not converge.")

    lines.append("")
    if tau_bmp is not None and tau_mcp is not None:
        R = tau_mcp / tau_bmp
        lines.append(f"R = tau_mcp / tau_bmp = {tau_mcp:.0f} / {tau_bmp:.0f} = {R:.3f}")
    elif tau_mcp is not None and bmp_cooled_out:
        excess_b0 = exc_bmp[0]
        if excess_b0 > MIN_EXCESS_C:
            tau_bmp_upper = cycle_s / np.log(excess_b0 / MIN_EXCESS_C)
            lines.append(f"tau_mcp = {tau_mcp:.0f} s")
            lines.append(f"tau_bmp < {tau_bmp_upper:.0f} s  (upper bound from noise floor {MIN_EXCESS_C} C)")
            lines.append(f"R > {tau_mcp / tau_bmp_upper:.2f}  (lower bound)")
    else:
        lines.append("R cannot be computed.")

    lines.append("")
    lines.append(f"{'burst':>6}  {'t(s)':>6}  {'T_bmp':>8}  {'T_mcp':>8}  {'phase':>8}  {'n':>5}")
    for k, (m, b) in enumerate(zip(all_means, bursts)):
        phase = 'warming' if k < peak_idx else 'cooling'
        lines.append(f"{k:>6}  {k*cycle_s:>6}  {m[0]:>8.3f}  {m[1]:>8.3f}  {phase:>8}  {len(b):>5}")

    with open(txt_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"Saved {txt_path}")

    for line in lines:
        print(line)


if __name__ == '__main__':
    main()
