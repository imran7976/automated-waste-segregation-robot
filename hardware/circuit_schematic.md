# Circuit Schematic & Wiring Architecture

## 1. System Overview
EcoSort is powered by an autonomous, regulated dual-rail power topology engineered to eliminate transient voltage sags and microcontroller brownouts caused by high-draw servo stall currents.

```
+---------------------------------------------------------------+
|                      Dual IMR 18650 Pack                      |
|                     (3.7V, 4000mAh Total)                     |
+-------------------------------+-------------------------------+
                                |
                                v
               +---------------------------------+
               |   LM2596 DC-DC Buck Converter   |
               |       (150 kHz, 2A Rated)       |
               +----------------+----------------+
                                |
             +------------------+------------------+
             | Regulated 5.0V Rail                 |
             v                                     v
+------------------------+             +------------------------+
| Arduino Uno R3 (Logic) |             |  4x MG90S Micro-Servos  |
|      (ATmega328P)      |             |   (High Current Rail)  |
+------------------------+             +------------------------+
             |                                     ^
             +-------------- Star Ground ----------+
                  (2200 µF Electrolytic Buffer)
```

---

## 2. Pin Interconnect Mapping

### A. Sensory Cluster Interfacing
| Sensor Module | Sensing Modality | Sensor Pin | Arduino Pin | Logic Level | Description |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **Active IR Obstacle Sensor** (LM393) | Ingress Specimen Arrival | Digital Out (OUT) | **Pin 2** (INT0) | Active-LOW | Hardware interrupt triggers classification routine |
| **LJ12A3-4-Z/BX Inductive Sensor** | Ferrous/Non-ferrous Metals | Digital Out (Black Wire) | **Pin 4** | Active-LOW (NPN-NO) | Detects metal conductivity within 4.0 mm |
| **FR-04 Moisture Sensor** (Electrolytic PCB) | Surface & Volume Moisture | Digital Out (D0) | **Pin 5** | Active-LOW | Comparator digital logic |
| **FR-04 Moisture Sensor** (Electrolytic PCB) | Moisture Gradient | Analog Out (A0) | **Pin A0** | Analog (0–5V) | 10-bit ADC channel ($V_{th} = 430 \approx 2.10\text{V}$) |

### B. Manipulator Actuator PWM Mapping
| Joint Label | Degree of Freedom | Actuator Model | PWM Pin | Rotation Envelope | Calibrated Home |
| :--- | :--- | :--- | :--- | :--- | :--- |
| **$J_1$ (Base)** | Azimuthal Rotation | TowerPro MG90S | **Pin 8** | $0^\circ - 180^\circ$ | $0^\circ$ |
| **$J_2$ (Shoulder)** | Vertical Pitch | TowerPro MG90S | **Pin 9** | $0^\circ - 180^\circ$ | $80^\circ$ |
| **$J_3$ (Elbow)** | Arm Extension Pitch | TowerPro MG90S | **Pin 10** | $0^\circ - 180^\circ$ | $40^\circ$ |
| **$J_4$ (Gripper)** | Parallel Jaw Clamping | TowerPro MG90S | **Pin 11** | $20^\circ - 90^\circ$ | $20^\circ$ (Closed) |

---

## 3. Power Isolation & Decoupling Strategy
1. **Isolated Regulated 5V Rail:** An LM2596 switching regulator steps down battery voltage directly to 5.0 V for the 4 servo motors.
2. **Brownout Suppression:** A $2200\,\mu\text{F}$ low-ESR electrolytic capacitor is placed in parallel across the servo $5\text{V} - \text{GND}$ rail to absorb instantaneous current spikes during simultaneous acceleration of all 4 servos.
3. **Common Star-Ground:** All ground lines (Arduino logic GND, LM2596 output GND, and sensor returns) converge at a single star-ground point to prevent ground loop noise from corrupting the high-resolution analog moisture readings on Pin A0.
