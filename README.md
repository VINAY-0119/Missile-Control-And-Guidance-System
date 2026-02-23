# Missile Guidance, Navigation & Control (GNC) System
### MATLAB/Simulink Model

---

<p align="center">
  <img src="gnc%20Power%20.png" alt="" width="800">
</p>
---

## Overview

This project implements a complete **Missile Guidance, Navigation & Control (GNC)** system in MATLAB/Simulink. The model simulates the full flight loop of a surface-to-surface or air-to-surface missile, integrating a flight path angle (FPA) guidance law, a PID-based autopilot controller, and a 2D kinematic navigation model that propagates position in Latitude/Longitude/Altitude (LLA) coordinates.

The Simulink top-level model (`GNC.slx`) contains a primary flight control loop and a guidance command subsystem that together close the guidance, control, and navigation loop from target acquisition through terminal engagement.

---

## System Architecture

The model is organized into the following functional blocks:

### 1. Guidance Command Subsystem
Computes the desired **Flight Path Angle (FPA)** and **Range-to-Go** based on:
- Current missile position (LLA)
- Observer (OBS) position/range
- Target location (LAT\_TARGET, LON\_TARGET, ELEV\_TARGET)

Outputs: `FPA`, `RNG`, `D_OBS`, `ALT`, `WAN`

### 2. PID Controller
A classical **PID controller** that takes the FPA error (commanded vs. actual) and generates a control surface deflection command (`d_cmd`). A saturation block limits the output to physically achievable deflection limits.

### 3. FPA Subsystem (Inner Subsystem)
Accepts the saturated deflection command and models the missile aerodynamic response to produce:
- Actual FPA (`ghat`)
- Angle of attack (`alpha`)
- `Theta_dot` (pitch rate)

### 4. Kinematics / Navigation
Integrates the missile equations of motion using:
- Velocity (`vel`) — from a constant or scheduled source
- `sin` / `cos` decomposition of FPA into vertical and horizontal velocity components
- Two integrators to propagate downrange position and altitude
- Conversion block: **POS → LLA** (converts Cartesian position to geodetic coordinates)

### 5. Height Computation
- An `Abs` block and `Height` computation derive current altitude above ground
- Scope block displays altitude over time
- A display block shows real-time LLA position

### 6. Stop / Simulation Control
- A **Stop Simulation** block terminates the run when terminal conditions are met (e.g., impact or time limit)

---

## Signal Flow Summary

```
[Target LLA] ──► GUIDANCE COMMAND ──► FPA_cmd
                                          │
[Missile State] ──► ADD ──► PID ──► SAT ──► FPA Subsystem ──► ghat (actual FPA)
                     ▲                                              │
                     └──────────────── Feedback ───────────────────┘
                                                                    │
                                             ┌──────────────────────┘
                                             ▼
                                   Cos/Sin × Vel ──► Integrators ──► [X, H]
                                                                        │
                                                                   POS → LLA ──► Display
```

---

## Model Parameters

| Parameter | Description | Typical Value |
|---|---|---|
| `vel` | Missile velocity (m/s) | Mission-dependent |
| Simulation Stop Time | Total flight duration (s) | 10.0 s (configurable) |
| PID Gains | Kp, Ki, Kd | Tuned for FPA tracking |
| Saturation Limits | Max/min deflection angle | ±30° |
| LAT\_TARGET / LON\_TARGET | Target geodetic coordinates | User-defined |
| ELEV\_TARGET | Target elevation (m) | User-defined |

---

## File Structure

```
GNC/
├── GNC.slx              # Top-level Simulink model
├── Subsystem.slx        # Inner FPA/aerodynamics subsystem
├── init_params.m        # (Recommended) Workspace initialization script
└── README.md            # This file
```

---

## Requirements

- **MATLAB** R2020b or later
- **Simulink**
- **Aerospace Toolbox** (recommended, for LLA conversion)
- **Control System Toolbox** (for PID tuning)

---

## Getting Started

### 1. Initialize Workspace

Before running the model, set target and initial conditions in MATLAB:

```matlab
% Target coordinates
LAT_TARGET  = 32.5000;   % degrees
LON_TARGET  = 44.4000;   % degrees
ELEV_TARGET = 100;       % meters

% Observer position (launch site)
LAT_OBS = 32.0000;
LON_OBS = 44.0000;

% Missile velocity
vel = 300;               % m/s

% PID gains (tune as needed)
Kp = 2.0;
Ki = 0.5;
Kd = 0.1;
```

### 2. Open and Run the Model

```matlab
open_system('GNC');
sim('GNC');
```

### 3. View Results

After simulation:
- The **Scope** block shows altitude vs. time
- The **Display** block shows terminal LLA position
- Export logged signals via the Simulink Data Inspector

---

## Subsystem Details

### FPA Guidance Law
The guidance block computes the commanded FPA as:

```
FPA_cmd = atan2(ΔAlt, ΔRange)
```

Where `ΔAlt` and `ΔRange` are the altitude and downrange distance differences between the missile and the target.

### Navigation (POS → LLA)
Position integration uses a flat-Earth approximation:

```
Ẋ = V · cos(FPA)
Ḣ = V · sin(FPA)
```

The `POS → LLA` block converts flat-Earth Cartesian (X, H) back to geodetic (Lat, Lon, Alt) for display and guidance feedback.

### PID Autopilot
The controller minimizes the FPA tracking error:

```
e(t) = FPA_cmd(t) − FPA_actual(t)
u(t) = Kp·e + Ki·∫e dt + Kd·(de/dt)
```

---

## Guidance Modes

| Mode | Description |
|---|---|
| **Terminal Guidance** | Closes on `TAR_GET` using FPA law |
| **Observer-Relative** | Uses `CURR_ENT` (current entity) and `OBS` range for midcourse updates |

---

## Known Limitations

- The model currently implements a **2D (planar)** guidance law. Full 3D engagement geometry (lateral guidance) is not included.
- The aerodynamic model inside the FPA subsystem is simplified. For high-fidelity applications, replace with a full 6-DOF aero model.
- Flat-Earth navigation is accurate for ranges under ~100 km. For longer ranges, use a WGS-84 ellipsoid model.
- The `Stop Simulation` block terminates on a fixed condition; add impact detection logic for more accurate terminal events.

---

## Tuning Guidelines

- **Increase `Kp`** to improve FPA tracking speed (may cause overshoot)
- **Increase `Ki`** to eliminate steady-state FPA error
- **Increase `Kd`** to damp oscillations (watch for noise amplification)
- Use the **MATLAB PID Tuner** (`pidTuner`) on a linearized plant for systematic tuning

---

## License

This model is provided for educational and research purposes. Ensure compliance with all applicable export control regulations (ITAR/EAR) before sharing or modifying missile guidance code.

---

## Author / Contact

Developed using MATLAB/Simulink. For questions, open an issue or contact the project maintainer.
