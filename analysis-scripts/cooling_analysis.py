"""
Burst cooling analysis — Newton's law fit for any experiment run.

Usage:
    python cooling_analysis.py <log_file> <sleep_s> [--fit-offset N]

Arguments:
    log_file       Path to raw .log file
    sleep_s        SLEEP_TIME value used during data collection (seconds)
    --fit-offset N Skip first N cooling bursts from the fit.
                   Use when the early cooling phase was in a different medium
                   (e.g. atmosphere before vacuum was applied). Default: 0.

Output (written next to the log file in ../processed/):
    <log_stem>_burst-means.csv
    <log_stem>_fit-results.txt
"""

import argparse
import csv
import os
import sys
import numpy as np
from scipy.optimize import curve_fit

AWAKE_S      = 60       # fixed in firmware
COL_BMP      = 2        # BMP388 temperature column (0-based)
COL_PRS      = 3        # BMP388 pressure column (0-based), hPa
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
                prs = float(parts[COL_PRS])
                mcp = float(parts[COL_MCP])
                if -100 <= bmp <= 200 and -100 <= mcp <= 200:
                    current.append((bmp, prs, mcp))
            except ValueError:
                continue
    if current:
        bursts.append(current)
    return bursts, n_separators


def burst_mean(burst):
    arr = np.array(burst)
    T_bmp = arr[:, 0].mean()
    # pressure: ignore zeros (sensor init), use only valid readings
    prs_valid = arr[:, 1][arr[:, 1] > 1.0]
    P_mean = prs_valid.mean() if len(prs_valid) > 0 else float('nan')
    T_mcp = arr[:, 2].mean()
    return T_bmp, P_mean, T_mcp


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
    parser = argparse.ArgumentParser(
        description='Burst cooling analysis — Newton\'s law fit.'
    )
    parser.add_argument('log_file', help='Path to raw .log file')
    parser.add_argument('sleep_s', type=int,
                        help='SLEEP_TIME value used during data collection (seconds)')
    parser.add_argument('--fit-offset', type=int, default=0, metavar='N',
                        help='Skip first N cooling bursts from the fit '
                             '(e.g. atmospheric equilibration before vacuum). Default: 0.')
    parser.add_argument('--fit-end', type=int, default=-1, metavar='N',
                        help='Last burst number (inclusive) to include in the fit. '
                             'Use to exclude re-pressurisation bursts. Default: -1 (use all).')
    args = parser.parse_args()

    log_path   = os.path.abspath(args.log_file)
    sleep_s    = args.sleep_s
    fit_offset = args.fit_offset
    fit_end    = args.fit_end
    cycle_s    = AWAKE_S + sleep_s

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

    all_means = [burst_mean(b) for b in bursts]
    T_bmp_all = np.array([m[0] for m in all_means])
    P_all     = np.array([m[1] for m in all_means])
    T_mcp_all = np.array([m[2] for m in all_means])

    # Auto-detect peak: cooling starts at the hottest burst
    peak_idx    = int(np.argmax(T_bmp_all))
    cool_means  = all_means[peak_idx:]

    T_bmp_cool = np.array([m[0] for m in cool_means])
    T_mcp_cool = np.array([m[2] for m in cool_means])

    T_amb_bmp = T_bmp_cool.min()
    T_amb_mcp = T_mcp_cool.min()

    exc_bmp = T_bmp_cool - T_bmp_cool.min()
    exc_mcp = T_mcp_cool - T_mcp_cool.min()
    n_meaningful = int(((exc_bmp > MIN_EXCESS_C) | (exc_mcp > MIN_EXCESS_C)).sum())

    # Fitting window: [peak_idx + fit_offset .. fit_end] (both inclusive burst numbers)
    fit_offset = min(fit_offset, len(cool_means) - 2)
    fit_start_burst = peak_idx + fit_offset
    fit_end_burst   = fit_end if (fit_end >= 0 and fit_end < len(bursts)) else len(bursts) - 1

    # Relative indices inside cool_means array
    rel_start = fit_offset
    rel_end   = fit_end_burst - peak_idx + 1   # exclusive upper bound
    rel_end   = max(rel_start + 1, min(rel_end, len(cool_means)))

    T_bmp_fit = T_bmp_cool[rel_start:rel_end]
    T_mcp_fit = T_mcp_cool[rel_start:rel_end]
    t_arr_fit = np.array([k * cycle_s for k in range(len(T_bmp_fit))], dtype=float)

    # T_amb: use only the fit window (avoids re-pressurisation bursts inflating min)
    T_amb_bmp = T_bmp_fit.min()
    T_amb_mcp = T_mcp_fit.min()

    # ── Burst means CSV ───────────────────────────────────────────────────────
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['burst', 't_s', 'T_bmp_C', 'P_hPa', 'T_mcp_C', 'phase', 'n_samples'])
        for k, (m, b) in enumerate(zip(all_means, bursts)):
            if k < peak_idx:
                phase = 'warming'
            elif k < fit_start_burst:
                phase = 'cooling-atm'
            elif k <= fit_end_burst:
                phase = 'cooling'
            else:
                phase = 'cooling-post'
            w.writerow([k, k * cycle_s, f'{m[0]:.4f}', f'{m[1]:.2f}', f'{m[2]:.4f}', phase, len(b)])
    print(f"Saved {csv_path}")

    # ── Fit ───────────────────────────────────────────────────────────────────
    fit_bmp, err_bmp = fit_newton(t_arr_fit, T_bmp_fit, T_amb_bmp, cycle_s)
    fit_mcp, err_mcp = fit_newton(t_arr_fit, T_mcp_fit, T_amb_mcp, cycle_s)

    bmp_cooled_out = (len(T_bmp_fit) > 1 and (T_bmp_fit[1] - T_amb_bmp) <= MIN_EXCESS_C)

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
    if fit_offset > 0 or fit_end >= 0:
        lines.append(f"Fit offset     : {fit_offset} bursts  "
                     f"(pre-vacuum phase, bursts {peak_idx}–{fit_start_burst - 1} excluded)")
        lines.append(f"Fit window     : bursts {fit_start_burst} to {fit_end_burst}  "
                     f"({len(T_bmp_fit)} bursts)")
        if fit_end >= 0:
            lines.append(f"Fit end        : burst {fit_end_burst}  "
                         f"(bursts {fit_end_burst + 1}–{len(bursts) - 1} excluded as post-vacuum)")
    lines.append(f"Meaningful (excess > {MIN_EXCESS_C} C) : {n_meaningful}")
    lines.append("")

    lines.append(f"T_amb_bmp = {T_amb_bmp:.3f} C  (min observed)")
    lines.append(f"T_amb_mcp = {T_amb_mcp:.3f} C  (min observed)")
    if T_bmp_fit[-1] - T_amb_bmp > 1.0:
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
        excess_b0 = T_bmp_fit[0] - T_amb_bmp
        if excess_b0 > MIN_EXCESS_C:
            tau_bmp_upper = cycle_s / np.log(excess_b0 / MIN_EXCESS_C)
            lines.append(f"tau_mcp = {tau_mcp:.0f} s")
            lines.append(f"tau_bmp < {tau_bmp_upper:.0f} s  (upper bound from noise floor {MIN_EXCESS_C} C)")
            lines.append(f"R > {tau_mcp / tau_bmp_upper:.2f}  (lower bound)")
    else:
        lines.append("R cannot be computed.")

    lines.append("")
    lines.append(f"{'burst':>6}  {'t(s)':>6}  {'T_bmp':>8}  {'P(hPa)':>8}  {'T_mcp':>8}  {'phase':>12}  {'n':>5}")
    for k, (m, b) in enumerate(zip(all_means, bursts)):
        if k < peak_idx:
            phase = 'warming'
        elif k < peak_idx + fit_offset:
            phase = 'cooling-atm'
        else:
            phase = 'cooling'
        p_str = f'{m[1]:>8.2f}' if m[1] == m[1] else '     nan'
        lines.append(f"{k:>6}  {k*cycle_s:>6}  {m[0]:>8.3f}  {p_str}  {m[2]:>8.3f}  {phase:>12}  {len(b):>5}")

    with open(txt_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"Saved {txt_path}")

    for line in lines:
        print(line)


if __name__ == '__main__':
    main()
