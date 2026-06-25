# LHS-1 Stratospheric Experiment: 26.073B

## Overview
This project is the stratospheric experiment for a high-altitude balloon flight. The experiment studies thermal insulation efficiency of lunar regolith (simulant) collecting important data for the future colonization of the Moon. The experiment was selected by iEDU Cubes in Space program for the flight on the NASA RB-11 high-altitude balloon.

&nbsp;

<div align="center">
  <img width="450" height="264" alt="Payload Cubes Size Comparison" src="/hardware/payload-size-comparison.jpg" />
  <br>
  <em>Payload Cube Size Comparison</em>
</div>

&nbsp;

## Status
Currently, the experiment is assembled and is tested on the bench.

## Experiment Title
Thermal Insulation Efficiency of LHS-1 Lunar
Regolith Simulant as a Material for Lunar Habitats
in Stratospheric Near-Vacuum Conditions.

## Problem Statement
NASA and all the international space agencies highly
advocate the use of In-Situ Resource Utilization (ISRU), especially utilizing the lunar regolith as the main
material in the 3D printing of habitats. 

The main problem with the use of regolith in this regard is the alteration
in the thermal insulation capabilities of the regolith when it is exposed to vacuum conditions. This makes
regolith potentially an excellent insulator, but its exact performance in near-vacuum has not been sufficiently
tested under dynamic flight conditions on Earth. 

This gap is confirmed by reliable sources: Exolith Lab
(2023) LHS-1 Lunar Highlands Simulant Fact Sheet, NASA (2022) Lunar Surface Innovation Initiative – ISRU
Focus Area, and Zacny et al. (2012) “Lunar Regolith Mechanics and Its Relevance to ISRU” (Earth and Space
2012, ASCE).

## Hypothesis
_**IF**_ a high-accuracy temperature sensor (+-0.25°C accuracy, MCP9808) exposed to extremely low
temperatures (-50 °C to -60°C) and near-vacuum atmospheric pressure (0.01 ~ 0.1 atm corresponding to 15 -
25 km altitude) during a stratospheric balloon flight is shielded by 3 mm of lunar highlands regolith simulant
(LHS-1, with bulk density ~1.6 g/cm3),

_**THEN**_ the temperature drop under the protection of the regolith will be at least 30 - 50% slower (over a multi-hour exposure period), compared to an unprotected ambient sensor (BMP-388), proving the usefulness of
lunar regolith as a thermal insulation and shielding material for lunar surface bases,

_**BECAUSE**_ the extremely low atmospheric pressure (0.01 atm) will remove air between regolith particles,
significantly decreasing the efficiency of convective heat transfer and the overall thermal conductivity of the
regolith.

_This is theoretical hypothesis. It is to be changed after the bench tests._

## Variables

### Independent Variables
1. **External temperature (°C):** BMP-388 sensor measures it during flight. It directly affects the
under-regolith temperatures and launches the cooling process, imitating nightfall on the Moon.

2. **Atmospheric pressure (Pa):** BMP-388 sensor measures it during flight. The decrease of atmospheric
pressure brings experiment closer to the lunar conditions and affects regolith’s thermal insulation
properties.

The experiment has 2 independent variables. However, they do not interfere with each other: the decrease of
external temperature is a trigger of cooling. Analysis compares it with under-regolith temperature, while the
atmospheric pressure defines how effectively the regolith shield works.

### Dependent Variables
1. **Under-regolith temperature (°C):** MCP9808 temperature sensor measures it during flight. It is the
internal temperature isolated by the regolith shield.

## Methodology

5 datasets will be collected:

1. **The flight dataset** — main data collected from the RB-11 flight in stratospheric conditions.

2. **No regolith configuration (atmosphere)** — defines baseline difference between sensors in 1 atmosphere conditions.

3. **No regolith configuration (near-vacuum)** — defines baseline between sensors in vacuum chamber conditions.

4. **Regolith configuration (atmosphere)** — defines regolith's contribution to thermal conductivity in 1 atmosphere conditions.

5. **Regolith configuration (near-vacuum)** — defines regolith's contribution to thermal conductivity in vacuum chamber conditions — the most close to the real flight test.

## Experiment Design

The experiment studies thermal insulation properties of lunar regolith simulant LHS-1 in stratospheric
conditions being launched on a balloon. The experiment components will be divided into 3 groups:

1. **Basic electronics group** — includes Adafruit ESP32-S3 microcontroller and Li-ion Charger. It controls
other electronics, saves data, and supplies power. ESP32-S3 microcontroller collects and writes data
once a second during flight and saves it on its own.

2. **Experimental group** — includes Adafruit MCP9808 temperature sensor and LHS-1 (Lunar Highlands
Simulant, then will be called “regolith”, “simulant”, “lunar dust”, etc.) shield. Sensor measures
dependent variable (under-regolith temperature). The regolith shield represents the main point of the
experiment, covering the sensor and protecting it from cooling.

3. **Control group** — includes Adafruit BMP388 atmospheric pressure and temperature sensor. It measures
independent variables (external temperature, atmospheric pressure) and plays the role of a control
group being not protected by a regolith shield.


The 3D-printed PET-G (Polyethylene Terephthalate Glyco — one of the plastics used for 3D-printing) cube payload cube contains all 3 groups providing construction rigidity and allowing mount components.

&nbsp;

<div align="center">
  <img width="450" height="264" alt="Payload Cube with Lid Opened" src="/hardware/payload-2.jpg" />
  <br>
  <em>Payload Cube with Lid Opened</em>
</div>

&nbsp;

## Repository Structure
```text
lhs1-stratospheric-experiment/
├── README.md
├── LICENSE
├── firmware/          # electronics software
├── cad/               # Fusion 360 3D models and renders
|   ├── stl/           # 3D printed components stl files
|   ├── step/          # full final 3D models
|   ├── renders/       # 3D models render images
|   └── CHANGELOG.md   # design change history
├── hardware/          # experiment hardware and electronics
├── data/
│   ├── raw/           # test and flight upload dumps      
│   └── processed/     # processed data
├── analysis/          # analysis scripts, graphs
├── docs/              
└── assets/            # service files
```
## Author Contributions
1. **Uali Berik** — Conceptualization, Methodology, Mechanical design (CAD),
   Software (firmware), Hardware integration, Validation, Writing,
   Project administration.

2. **Arlan Tairov** — Problem statement, Background research.

3. **Amina Toxankyzy** — Communication plan, Documentation of assembly process.

4. **Nurzhan Kunakbayev** — Safety, Analysis plan.

5. **Arman Temirbulatov** — Assembly support.