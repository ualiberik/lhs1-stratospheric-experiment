"""
EXP-04: Regolith Configuration — Atmosphere
Burst averaging + Newton's law fit for tau_bmp, tau_mcp, and R = tau_mcp / tau_bmp.
"""

import csv
import os
import sys
import numpy as np
from scipy.optimize import curve_fit

# ── Paths ────────────────────────────────────────────────────────────────────
BASE = os.path.dirname(os.path.abspath(__file__))
LOG_PATH = os.path.join(BASE, '../data/exp4-regolith-atmosphere/raw/'
                               'exp04_regolith-atm_2026-06-25_run1.log')
OUT_DIR  = os.path.join(BASE, '../data/exp4-regolith-atmosphere/processed')

# ── Firmware parameters (at time of collection) ───────────────────────────────
AWAKE_S  = 60
SLEEP_S  = 600   # was 600 at collection time; future runs use 60
CYCLE_S  = AWAKE_S + SLEEP_S

# ── Data stream column indices (0-based) ──────────────────────────────────────
COL_BMP = 2   # column 3: temp_bmp388_degC
COL_MCP = 6   # column 7: temp_mcp9808_degC (former ax field)

SEPARATOR = '00000000-000000'
MIN_EXCESS_C = 0.10   # threshold: burst considered "meaningful" if excess > this


# ── Parsing ───────────────────────────────────────────────────────────────────
def parse_bursts(path):
    bursts, current = [], []
    with open(path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('<<<'):
                continue
            if SEPARATOR in line:
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
    return bursts


def burst_mean(burst):
    arr = np.array(burst)
    return arr[:, 0].mean(), arr[:, 1].mean()


# ── Fitting ───────────────────────────────────────────────────────────────────
def fit_newton(t, T, T_amb):
    """Fit T(t) = T_amb + A*exp(-t/tau) with T_amb fixed. Returns (A, tau), (σ_A, σ_tau)."""
    def model(t, A, tau):
        return T_amb + A * np.exp(-t / tau)

    excess = T[0] - T_amb
    if excess <= 0:
        return None, None

    try:
        popt, pcov = curve_fit(
            model, t, T,
            p0=[excess, CYCLE_S * 2],
            bounds=([0, 1], [200, 50000]),
            maxfev=5000
        )
        return popt, np.sqrt(np.diag(pcov))
    except (RuntimeError, ValueError):
        return None, None


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    bursts = parse_bursts(LOG_PATH)
    print(f"Total bursts: {len(bursts)}")

    cooling = bursts[1:]   # discard burst 0 (equalization)
    if not cooling:
        print("No post-equalization bursts — aborting.")
        sys.exit(1)

    means = [burst_mean(b) for b in cooling]
    T_bmp = np.array([m[0] for m in means])
    T_mcp = np.array([m[1] for m in means])
    t_arr = np.array([k * CYCLE_S for k in range(len(means))], dtype=float)

    # Use minimum observed temperature as T_amb — robust when sensor
    # transiently overshoots ambient or room temperature drifts slightly.
    T_amb_bmp = T_bmp.min()
    T_amb_mcp = T_mcp.min()

    exc_bmp = T_bmp - T_amb_bmp
    exc_mcp = T_mcp - T_amb_mcp
    n_meaningful = int(((exc_bmp > MIN_EXCESS_C) | (exc_mcp > MIN_EXCESS_C)).sum())

    # ── Save burst means CSV ──────────────────────────────────────────────────
    os.makedirs(OUT_DIR, exist_ok=True)
    csv_path = os.path.join(OUT_DIR, 'exp04_burst_means.csv')
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['burst', 't_s', 'T_bmp_C', 'T_mcp_C',
                    'excess_bmp_C', 'excess_mcp_C', 'n_samples'])
        for k, ((T_b, T_m), t) in enumerate(zip(means, t_arr)):
            w.writerow([k + 1, int(t), f'{T_b:.4f}', f'{T_m:.4f}',
                        f'{T_b - T_amb_bmp:+.4f}', f'{T_m - T_amb_mcp:+.4f}',
                        len(cooling[k])])
    print(f"Saved {csv_path}")

    # ── Fit ───────────────────────────────────────────────────────────────────
    fit_bmp, err_bmp = fit_newton(t_arr, T_bmp, T_amb_bmp)
    fit_mcp, err_mcp = fit_newton(t_arr, T_mcp, T_amb_mcp)

    # ── Save results ──────────────────────────────────────────────────────────
    txt_path = os.path.join(OUT_DIR, 'exp04_fit_results.txt')
    lines = []
    lines.append("EXP-04  Newton Cooling Fit Results")
    lines.append("=" * 50)
    lines.append(f"Burst cycle : {CYCLE_S} s  (awake {AWAKE_S} s + sleep {SLEEP_S} s)")
    lines.append(f"Bursts total: {len(bursts)}  "
                 f"(burst 0 discarded as equalization)")
    lines.append(f"Cooling bursts: {len(cooling)}")
    lines.append(f"Bursts with excess > {MIN_EXCESS_C} °C: {n_meaningful}")
    lines.append(f"T_amb_bmp  = {T_amb_bmp:.3f} °C")
    lines.append(f"T_amb_mcp  = {T_amb_mcp:.3f} °C")
    lines.append("")

    if n_meaningful < 3:
        lines.append("WARNING: fewer than 3 bursts with meaningful excess.")
        lines.append("Almost all cooling occurred within the first sleep window")
        lines.append(f"({SLEEP_S} s). Inter-burst fit is poorly constrained.")
        lines.append("Re-run with SLEEP_TIME=60 s for reliable tau estimation.")
        lines.append("")

    bmp_cooled_out = (exc_bmp[1] <= MIN_EXCESS_C) if len(exc_bmp) > 1 else True

    if fit_mcp is not None:
        tau_mcp = fit_mcp[1]
        lines.append(f"MCP9808 : A = {fit_mcp[0]:.2f} °C,  "
                     f"tau = {tau_mcp:.0f} s  (+-{err_mcp[1]:.0f} s)")
    else:
        lines.append("MCP9808 : fit did not converge.")
        tau_mcp = None

    if bmp_cooled_out:
        lines.append(f"BMP388  : cooling complete within first sleep window "
                     f"({SLEEP_S} s).  tau_bmp << {SLEEP_S} s — indeterminate.")
        tau_bmp = None
    elif fit_bmp is not None:
        tau_bmp = fit_bmp[1]
        lines.append(f"BMP388  : A = {fit_bmp[0]:.2f} °C,  "
                     f"tau = {tau_bmp:.0f} s  (+-{err_bmp[1]:.0f} s)")
    else:
        lines.append("BMP388  : fit did not converge.")
        tau_bmp = None

    lines.append("")
    if tau_bmp is not None and tau_mcp is not None:
        R = tau_mcp / tau_bmp
        lines.append(f"R = tau_mcp / tau_bmp = {tau_mcp:.0f} / {tau_bmp:.0f} = {R:.3f}")
        if n_meaningful < 3:
            lines.append("(R has low confidence — see warning above)")
    elif tau_mcp is not None:
        # Upper bound on tau_bmp: at burst 2, BMP388 excess <= MIN_EXCESS_C
        # (detection floor). Solving A*exp(-t/tau) = MIN_EXCESS_C gives tau_max.
        excess_b1_bmp = T_bmp[0] - T_amb_bmp
        tau_bmp_upper = CYCLE_S / np.log(excess_b1_bmp / MIN_EXCESS_C)
        R_lower = tau_mcp / tau_bmp_upper
        lines.append(f"tau_mcp = {tau_mcp:.0f} s")
        lines.append(f"tau_bmp < {tau_bmp_upper:.0f} s  "
                     f"(upper bound from noise floor {MIN_EXCESS_C} degC)")
        lines.append(f"R = tau_mcp / tau_bmp > {R_lower:.2f}  (lower bound)")
    else:
        lines.append("R cannot be computed — fit failed.")

    lines.append("")
    lines.append("Burst means:")
    lines.append(f"{'burst':>6}  {'t(s)':>6}  {'T_bmp':>8}  {'T_mcp':>8}  "
                 f"{'exc_bmp':>9}  {'exc_mcp':>9}  {'n':>5}")
    for k, ((T_b, T_m), t) in enumerate(zip(means, t_arr)):
        lines.append(
            f"{k+1:>6}  {int(t):>6}  {T_b:>8.3f}  {T_m:>8.3f}  "
            f"{T_b-T_amb_bmp:>+9.3f}  {T_m-T_amb_mcp:>+9.3f}  "
            f"{len(cooling[k]):>5}"
        )

    with open(txt_path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    print(f"Saved {txt_path}")

    for line in lines:
        print(line)


if __name__ == '__main__':
    main()
