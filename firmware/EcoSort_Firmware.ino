/**
 * @file EcoSort_Firmware.ino
 * @brief Autonomous 4-DOF Robotic Manipulator with Priority-Based Multi-Sensor Fusion
 * @author Md. Al-Imran
 * @details Implements synchronous FSM, 4-phase pick-and-place routine, 
 *          temporal majority voting, and deterministic priority logic (Metal > Wet > Dry).
 */

#include <Servo.h>
#include "config.h"
#include "kinematics.h"

// ==========================================
// SERVO ACTUATOR INSTANCES & CURRENT ANGLES
// ==========================================
Servo servoBase;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

int currentBaseAngle     = BASE_HOME_ANGLE;
int currentShoulderAngle = SHOULDER_HOME_ANGLE;
int currentElbowAngle    = ELBOW_HOME_ANGLE;
int currentGripperAngle  = GRIPPER_CLOSED_ANGLE;

// Kinematics solver instance
RobotKinematics kinematics;

// Ingress detection interrupt flag
volatile bool objectDetected = false;

// Function declarations
void isrObjectIngress();
void muoviServo(Servo &servo, int &currentPos, int targetPos, int stepDelayMs = SERVO_STEP_DELAY_MS);
WasteClass sampleWasteMaterial();
void executeSortingRoutine(WasteClass wasteType);
void returnToHomeStance();

// ==========================================
// ARDUINO SETUP
// ==========================================
void setup() {
    // Initialize serial telemetry
    Serial.begin(9600);
    while (!Serial && millis() < 2000); // Wait up to 2 seconds for serial port

    Serial.println(F("=================================================="));
    Serial.println(F(" EcoSort: Autonomous 4-DOF Waste Segregation Robot"));
    Serial.println(F(" Firmware v2.1 | Author: Md. Al-Imran"));
    Serial.println(F("=================================================="));

    // Configure sensor GPIOs
    pinMode(PIN_IR_SENSOR, INPUT_PULLUP);
    pinMode(PIN_INDUCTIVE_SENSOR, INPUT_PULLUP);
    pinMode(PIN_MOISTURE_DIGITAL, INPUT);
    pinMode(PIN_MOISTURE_ANALOG, INPUT);

    // Attach hardware interrupt for IR Ingress detection (Active-LOW on Pin 2)
    attachInterrupt(digitalPinToInterrupt(PIN_IR_SENSOR), isrObjectIngress, FALLING);

    // Attach servo PWM pins
    servoBase.attach(PIN_SERVO_BASE);
    servoShoulder.attach(PIN_SERVO_SHOULDER);
    servoElbow.attach(PIN_SERVO_ELBOW);
    servoGripper.attach(PIN_SERVO_GRIPPER);

    // Initialize robot to calibrated home stance
    Serial.println(F("[INIT] Initializing 4-DOF arm to Home Stance..."));
    servoBase.write(BASE_HOME_ANGLE);
    servoShoulder.write(SHOULDER_HOME_ANGLE);
    servoElbow.write(ELBOW_HOME_ANGLE);
    servoGripper.write(GRIPPER_CLOSED_ANGLE);
    delay(1000);

    Serial.println(F("[SYSTEM READY] Entering IDLE polling state. Waiting for refuse specimen..."));
}

// ==========================================
// HARDWARE INTERRUPT SERVICE ROUTINE (ISR)
// ==========================================
void isrObjectIngress() {
    objectDetected = true;
}

// ==========================================
// ARDUINO MAIN LOOP
// ==========================================
void loop() {
    // Check if an object triggered the active-low IR ingress beam
    if (objectDetected || digitalRead(PIN_IR_SENSOR) == LOW) {
        objectDetected = false;

        Serial.println(F("\n[EVENT] Refuse specimen detected at inspection stage."));

        // Step 1: Mechanical Stabilization Delay (eliminates contact bounce)
        delay(STABILIZATION_DELAY_MS);

        // Step 2: Multi-pass Temporal Majority Sampling
        WasteClass classifiedClass = sampleWasteMaterial();

        // Step 3: Execute Automated 4-Phase Pick-and-Place Routine
        executeSortingRoutine(classifiedClass);

        // Re-arm system for next incoming specimen
        Serial.println(F("[IDLE] Cycle completed. Monitoring for next item..."));
    }

    delay(20);
}

// ==========================================
// TEMPORAL MAJORITY MULTI-SENSOR SAMPLING
// ==========================================
WasteClass sampleWasteMaterial() {
    int metalVoteCount = 0;
    int wetVoteCount   = 0;
    int dryVoteCount   = 0;

    Serial.println(F("[SENSOR] Commencing 3-pass majority sampling loop (75ms interval)..."));

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // Read inductive proximity sensor (Active-LOW: 0 = Metal present)
        int inductiveVal = digitalRead(PIN_INDUCTIVE_SENSOR);

        // Read FR-04 moisture sensor (10-bit ADC channel A0)
        int moistureAdcVal = analogRead(PIN_MOISTURE_ANALOG);

        Serial.print(F("  - Sample #"));
        Serial.print(i + 1);
        Serial.print(F(" | Inductive: "));
        Serial.print(inductiveVal == LOW ? F("METAL_DETECTED") : F("NO_METAL"));
        Serial.print(F(" | Moisture ADC: "));
        Serial.println(moistureAdcVal);

        // Hierarchical Evaluation per sample
        if (inductiveVal == LOW) {
            metalVoteCount++;
        } else if (moistureAdcVal >= MOISTURE_ADC_THRESHOLD) {
            wetVoteCount++;
        } else {
            dryVoteCount++;
        }

        if (i < SAMPLE_COUNT - 1) {
            delay(SAMPLE_INTERVAL_MS);
        }
    }

    // Deterministic Priority Decision (Metal > Wet > Dry)
    WasteClass finalDecision;
    if (metalVoteCount >= 2) {
        finalDecision = WASTE_METAL;
        Serial.println(F("[DECISION] Priority 1 Triggered: Classified as [METAL WASTE] (Confidence: High)"));
    } else if (wetVoteCount >= 2) {
        finalDecision = WASTE_WET;
        Serial.println(F("[DECISION] Priority 2 Triggered: Classified as [WET ORGANIC WASTE] (Confidence: High)"));
    } else {
        finalDecision = WASTE_DRY;
        Serial.println(F("[DECISION] Priority 3 Triggered: Classified as [DRY RECYCLABLE WASTE] (Default)"));
    }

    return finalDecision;
}

// ==========================================
// 4-PHASE PICK-AND-PLACE TRAJECTORY ROUTINE
// ==========================================
void executeSortingRoutine(WasteClass wasteType) {
    int targetBinAngle = BIN_DRY_ANGLE; // Default to Dry
    const char *binName = "Bin 3 (Dry Recyclables - 180 deg)";

    if (wasteType == WASTE_METAL) {
        targetBinAngle = BIN_METAL_ANGLE;
        binName = "Bin 1 (Metal - 60 deg)";
    } else if (wasteType == WASTE_WET) {
        targetBinAngle = BIN_WET_ANGLE;
        binName = "Bin 2 (Wet Organics - 120 deg)";
    }

    Serial.print(F("[ACTUATION] Routing to destination: "));
    Serial.println(binName);

    // ----------------------------------------------------
    // PHASE I: Grasp Alignment (Enveloping Specimen)
    // ----------------------------------------------------
    Serial.println(F("  -> Phase I: Grasp Alignment (Opening gripper, lowering shoulder)"));
    muoviServo(servoGripper, currentGripperAngle, GRIPPER_OPEN_ANGLE);
    muoviServo(servoShoulder, currentShoulderAngle, SHOULDER_PICK_ANGLE);
    delay(100);

    // ----------------------------------------------------
    // PHASE II: Securing & Elevation (Frictional Grasp)
    // ----------------------------------------------------
    Serial.println(F("  -> Phase II: Securing & Elevation (Closing jaw, lifting shoulder)"));
    muoviServo(servoGripper, currentGripperAngle, GRIPPER_CLOSED_ANGLE);
    delay(150); // Settle frictional grip
    muoviServo(servoShoulder, currentShoulderAngle, SHOULDER_HOME_ANGLE);
    delay(100);

    // ----------------------------------------------------
    // PHASE III: Transfer Rotation (Azimuthal Bin Transfer)
    // ----------------------------------------------------
    Serial.println(F("  -> Phase III: Transfer Rotation (Adjusting elbow, routing base)"));
    muoviServo(servoElbow, currentElbowAngle, ELBOW_TRANSFER_ANGLE);
    muoviServo(servoBase, currentBaseAngle, targetBinAngle);
    delay(150);

    // ----------------------------------------------------
    // PHASE IV: Discharge & Homing (Release and Reset)
    // ----------------------------------------------------
    Serial.println(F("  -> Phase IV: Discharge & Homing (Discharging item into bin)"));
    muoviServo(servoGripper, currentGripperAngle, GRIPPER_OPEN_ANGLE);
    delay(200); // Allow gravity discharge
    muoviServo(servoGripper, currentGripperAngle, GRIPPER_CLOSED_ANGLE);

    returnToHomeStance();
}

// ==========================================
// RESET ROBOT TO CALIBRATED HOME POSITION
// ==========================================
void returnToHomeStance() {
    Serial.println(F("  -> Resetting links to Home Stance (0 deg Base)..."));
    muoviServo(servoBase, currentBaseAngle, BASE_HOME_ANGLE);
    muoviServo(servoElbow, currentElbowAngle, ELBOW_HOME_ANGLE);
    muoviServo(servoShoulder, currentShoulderAngle, SHOULDER_HOME_ANGLE);
    muoviServo(servoGripper, currentGripperAngle, GRIPPER_CLOSED_ANGLE);
    delay(200);
}

// ==========================================
// SMOOTH SERVO MOTION INTERPOLATION
// ==========================================
void muoviServo(Servo &servo, int &currentPos, int targetPos, int stepDelayMs) {
    if (currentPos == targetPos) return;

    if (currentPos < targetPos) {
        for (int pos = currentPos; pos <= targetPos; pos++) {
            servo.write(pos);
            delay(stepDelayMs);
        }
    } else {
        for (int pos = currentPos; pos >= targetPos; pos--) {
            servo.write(pos);
            delay(stepDelayMs);
        }
    }
    currentPos = targetPos;
}
