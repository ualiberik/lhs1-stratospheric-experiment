# LHS-1 Regolith Thermal Insulation — Ground Experiment Results

## Overview

Four bench experiments were conducted to characterize the thermal insulation effect of
LHS-1 Lunar Highlands Simulant regolith under atmospheric and near-vacuum conditions.
Cooling curves were fitted to Newton's law of cooling:

```
T(t) = T_amb + A × exp(−t / τ)
```

The primary metric is the **insulation ratio R = τ_mcp / τ_bmp**, where:
- τ_mcp — cooling time constant of the MCP9808 sensor (under regolith, or exposed)
- τ_bmp — cooling time constant of the BMP388 sensor (external reference)

R > 1 means the regolith-shielded sensor cools slower than the reference → insulation effect.

## Hardware

| Config | MCU | Firmware |
|---|---|---|
| EXP-04, EXP-05 | Adafruit ESP32-S3 Feather | ion_2026_berikov |
| EXP-02, EXP-03 | ESP32-C6 SuperMini | ion_no-regolith_c6 |

**Burst structure:** 60 s awake (600 samples at 100 ms) → 60 s sleep → repeat. Cycle = 120 s.

## Results

| Experiment | Conditions | τ_bmp (s) | τ_mcp (s) | R |
|---|---|---|---|---|
| EXP-02 | No regolith, atmosphere | 645 ± 13 | 645 ± 19 | **0.999** |
| EXP-04 | Regolith, atmosphere | 937 ± 14 | 1021 ± 24 | **1.090** |
| EXP-03 | No regolith, near-vacuum (~20 hPa) | 751 ± 15 | 709 ± 18 | **0.945** |
| EXP-05 | Regolith, near-vacuum (~25 hPa) | 879 ± 15 | 905 ± 27 | **1.029** |

EXP-04: 4 runs performed; run 4 used for final fit.  
EXP-05: run 1 discarded (unstable vacuum); run 2 used.

## Key Findings

**Net regolith contribution:**

| Medium | Δ = R_regolith − R_baseline | Interpretation |
|---|---|---|
| Atmosphere | 1.090 − 0.999 = **+0.091** | Regolith slows MCP cooling by ~9% |
| Near-vacuum | 1.029 − 0.945 = **+0.084** | Regolith slows MCP cooling by ~8% |

Both values are positive: **regolith provides measurable thermal insulation in both environments.**

The difference between Δ_atm and Δ_vac (0.007) is smaller than the propagated
uncertainty (~±0.047), so whether vacuum enhances the insulation effect is statistically
inconclusive at this sample size.

**Baseline asymmetry in vacuum (EXP-03, R = 0.945):** without regolith, the MCP9808
cools faster than the BMP388 under near-vacuum conditions. This reflects differences in
radiative coupling between the two sensors in the absence of convection, and is absent
at atmospheric pressure (EXP-02, R = 0.999).

## Firmware Notes

An I2C bus recovery sequence (9 SCL clock pulses before `Wire.begin()`, per NXP UM10204
§3.1.16) was identified and tested during bench experiments to prevent sensor
initialization failures after deep sleep.