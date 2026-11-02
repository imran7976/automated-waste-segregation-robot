/**
 * @file config.h
 * @brief System Pinout, Kinematic Bounds, and Priority Constants for EcoSort 4-DOF Robot
 * @author Md. Al-Imran
 * @details Aligned with validated experimental parameters from conference validation.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ==========================================
// 1. PIN DEFINITIONS
// ==========================================
// Sensor Inputs
#define PIN_IR_SENSOR         2   // Active-LOW Ingress IR Obstacle Sensor (INT0)
#define PIN_INDUCTIVE_SENSOR  4   // LJ12A3-4-Z/BX NPN-NO Inductive Proximity Sensor
#define PIN_MOISTURE_DIGITAL  5   // FR-04 Moisture Comparator Digital Output (D0)
#define PIN_MOISTURE_ANALOG   A0  // FR-04 Moisture 10-bit ADC Analog Input (A0)

// Actuator PWM Outputs (TowerPro MG90S Micro-Servos)
#define PIN_SERVO_BASE        8   // Joint 1: Azimuthal Base Rotation
#define PIN_SERVO_SHOULDER    9   // Joint 2: Shoulder Pitch Link
#define PIN_SERVO_ELBOW       10  // Joint 3: Elbow Pitch Link
#define PIN_SERVO_GRIPPER     11  // Joint 4: Parallel-Jaw End-Effector Gripper

// ==========================================
// 2. SENSOR & CLASSIFICATION CONSTANTS
// ==========================================
// Ingress Stabilization Delay (eliminates mechanical bounce)
#define STABILIZATION_DELAY_MS  200

// Temporal Majority Sampling
#define SAMPLE_COUNT            3    // 3-pass majority voting
#define SAMPLE_INTERVAL_MS      75   // 75 ms between successive samples

// Moisture Threshold (10-bit ADC [0-1023], 430 corresponds to ~2.10V)
#define MOISTURE_ADC_THRESHOLD  430

// Waste Classification Classes
enum WasteClass {
    WASTE_NONE = 0,
    WASTE_METAL = 1,
    WASTE_WET = 2,
    WASTE_DRY = 3
};

// ==========================================
// 3. KINEMATIC & SERVO TRAJECTORY ANGLES
// ==========================================
// Home Stance
#define BASE_HOME_ANGLE         0
#define SHOULDER_HOME_ANGLE     80
#define ELBOW_HOME_ANGLE        40
#define GRIPPER_CLOSED_ANGLE    20
#define GRIPPER_OPEN_ANGLE      90

// Pick Routine Angles
#define SHOULDER_PICK_ANGLE     135
#define ELBOW_TRANSFER_ANGLE    100

// Target Destination Bin Azimuth Angles (Base Servo)
#define BIN_METAL_ANGLE         60   // Bin 1: Metallic Waste (60 deg)
#define BIN_WET_ANGLE           120  // Bin 2: Wet Organic Waste (120 deg)
#define BIN_DRY_ANGLE           180  // Bin 3: Dry Recyclable Waste (180 deg)

// Motion Interpolation Step Delay (prevents mechanical jerk & payload drop)
#define SERVO_STEP_DELAY_MS     15   // 15 ms per degree step

// ==========================================
// 4. DENAVIT-HARTENBERG (D-H) DIMENSIONS (mm)
// ==========================================
#define DH_D1   65.0f   // Base offset height (mm)
#define DH_A2   80.0f   // Shoulder link length (mm)
#define DH_A3   80.0f   // Elbow link length (mm)
#define DH_A4   60.0f   // Gripper end-effector reach (mm)

#define WORKSPACE_R_MIN  10.0f   // Minimum reach boundary |L2 - L3| (mm)
#define WORKSPACE_R_MAX  200.0f  // Maximum reach boundary L2 + L3 (mm)
#define TARGET_RADIUS    150.0f  // Deposition operating radius (mm)

#endif // CONFIG_H
