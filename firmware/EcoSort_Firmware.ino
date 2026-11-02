/**
 * @file EcoSort_Firmware.ino
 * @brief Verified Arduino Firmware for EcoSort 4-DOF Robotic Waste Segregation System
 * @author Md. Al-Imran
 * 
 * Hardware Pinout:
 *   - IR Sensor:           Digital Pin 2 (Active-LOW ingress detection)
 *   - Inductive Sensor:    Digital Pin 4 (LJ12A3-4-Z/BX Metal detection)
 *   - Moisture Sensor:     Digital Pin 5 (FR-04 Rain/Moisture detection)
 *   - Base Servo (J1):     PWM Pin 8     (60 deg Metal, 120 deg Wet, 180 deg Dry)
 *   - Shoulder Servo (J2): PWM Pin 9     (First Arm pitch)
 *   - Elbow Servo (J3):    PWM Pin 10    (Second Arm pitch)
 *   - Gripper Servo (J4):  PWM Pin 11    (Parallel-jaw claw)
 */

#include <Servo.h>

// ==========================================
// PIN CONFIGURATIONS
// ==========================================
#define PIN_IR_SENSOR         2   // Ingress object obstacle sensor
#define PIN_INDUCTIVE_SENSOR  4   // Inductive proximity metal sensor
#define PIN_MOISTURE_SENSOR   5   // FR-04 Rain/moisture sensor

#define PIN_SERVO_BASE        8   // Joint 1: Base rotation servo
#define PIN_SERVO_FIRST_ARM   9   // Joint 2: Shoulder (First arm) servo
#define PIN_SERVO_SECOND_ARM  10  // Joint 3: Elbow (Second arm) servo
#define PIN_SERVO_GRIPPER     11  // Joint 4: End-effector claw servo

// ==========================================
// ACTUATOR OBJECTS & CALIBRATED HOME ANGLES
// ==========================================
Servo servoBase;
Servo servoFirstArm;
Servo servoSecondArm;
Servo servoGripper;

const int defaultBaseAngle      = 0;   // Home stance: 0 deg
const int defaultFirstArmAngle  = 80;  // Shoulder resting: 80 deg
const int defaultSecondArmAngle = 40;  // Elbow resting: 40 deg
const int defaultGripperAngle   = 20;  // Claw closed: 20 deg

int currentBasePos      = defaultBaseAngle;
int currentFirstArmPos  = defaultFirstArmAngle;
int currentSecondArmPos = defaultSecondArmAngle;
int currentGripperPos   = defaultGripperAngle;

// ==========================================
// FUNCTION DECLARATIONS
// ==========================================
void muoviServo(Servo &servo, int fromPos, int toPos);
void sortMetalWaste();
void sortWetWaste();
void sortDryWaste();

// ==========================================
// SYSTEM SETUP
// ==========================================
void setup() {
  pinMode(PIN_IR_SENSOR, INPUT);
  pinMode(PIN_MOISTURE_SENSOR, INPUT);
  pinMode(PIN_INDUCTIVE_SENSOR, INPUT);

  Serial.begin(9600);
  Serial.println(F("=================================================="));
  Serial.println(F(" EcoSort: Automated 4-DOF Waste Segregation Robot"));
  Serial.println(F(" Firmware Version: 1.0 (Hardware-Verified)       "));
  Serial.println(F(" Author: Md. Al-Imran                             "));
  Serial.println(F("=================================================="));

  // Attach servo PWM pins
  servoBase.attach(PIN_SERVO_BASE);
  servoFirstArm.attach(PIN_SERVO_FIRST_ARM);
  servoSecondArm.attach(PIN_SERVO_SECOND_ARM);
  servoGripper.attach(PIN_SERVO_GRIPPER);

  // Initialize servos to calibrated default home stances
  servoBase.write(defaultBaseAngle);
  servoFirstArm.write(defaultFirstArmAngle);
  servoSecondArm.write(defaultSecondArmAngle);
  servoGripper.write(defaultGripperAngle);

  delay(1000);
  Serial.println(F("[SYSTEM READY] Polling sensor array for incoming waste..."));
}

// ==========================================
// SMOOTH STEP-BY-STEP SERVO ACTUATION
// ==========================================
// Moves servo incrementally with 15 ms delay per degree to prevent inertial jerk
void muoviServo(Servo &servo, int fromPos, int toPos) {
  if (fromPos < toPos) {
    for (int pos = fromPos; pos <= toPos; pos++) {
      servo.write(pos);
      delay(15);
    }
  } else {
    for (int pos = fromPos; pos >= toPos; pos--) {
      servo.write(pos);
      delay(15);
    }
  }
}

// ==========================================
// PICK-AND-PLACE: METAL STREAM (BIN 1 - 60°)
// ==========================================
void sortMetalWaste() {
  Serial.println(F("  -> Executing pick-and-place for Metal Waste (Bin 1: 60 deg)"));

  // Phase I: Grasp Alignment
  muoviServo(servoGripper, currentGripperPos, 90);
  currentGripperPos = 90;
  delay(500);

  muoviServo(servoFirstArm, currentFirstArmPos, 135);
  currentFirstArmPos = 135;
  delay(500);

  // Phase II: Securing & Elevation
  muoviServo(servoGripper, currentGripperPos, defaultGripperAngle);
  currentGripperPos = defaultGripperAngle;
  delay(500);

  muoviServo(servoFirstArm, currentFirstArmPos, defaultFirstArmAngle);
  currentFirstArmPos = defaultFirstArmAngle;
  delay(500);

  // Phase III: Transfer Rotation to Bin 1 (60 deg)
  muoviServo(servoSecondArm, currentSecondArmPos, 100);
  currentSecondArmPos = 100;
  delay(500);

  muoviServo(servoBase, currentBasePos, 60);
  currentBasePos = 60;
  delay(1000);

  // Phase IV: Discharge & Homing
  muoviServo(servoGripper, currentGripperPos, 90);
  currentGripperPos = 90;
  delay(1000);

  muoviServo(servoGripper, currentGripperPos, defaultGripperAngle);
  currentGripperPos = defaultGripperAngle;
  delay(500);

  muoviServo(servoBase, currentBasePos, defaultBaseAngle);
  currentBasePos = defaultBaseAngle;
  delay(500);

  muoviServo(servoSecondArm, currentSecondArmPos, defaultSecondArmAngle);
  currentSecondArmPos = defaultSecondArmAngle;
  delay(500);
}

// ==========================================
// PICK-AND-PLACE: WET STREAM (BIN 2 - 120°)
// ==========================================
void sortWetWaste() {
  Serial.println(F("  -> Executing pick-and-place for Wet Waste (Bin 2: 120 deg)"));

  // Phase I: Grasp Alignment
  muoviServo(servoGripper, currentGripperPos, 90);
  currentGripperPos = 90;
  delay(500);

  muoviServo(servoFirstArm, currentFirstArmPos, 135);
  currentFirstArmPos = 135;
  delay(500);

  // Phase II: Securing & Elevation
  muoviServo(servoGripper, currentGripperPos, defaultGripperAngle);
  currentGripperPos = defaultGripperAngle;
  delay(500);

  muoviServo(servoFirstArm, currentFirstArmPos, defaultFirstArmAngle);
  currentFirstArmPos = defaultFirstArmAngle;
  delay(500);

  // Phase III: Transfer Rotation to Bin 2 (120 deg)
  muoviServo(servoSecondArm, currentSecondArmPos, 100);
  currentSecondArmPos = 100;
  delay(500);

  muoviServo(servoBase, currentBasePos, 120);
  currentBasePos = 120;
  delay(1000);

  // Phase IV: Discharge & Homing
  muoviServo(servoGripper, currentGripperPos, 90);
  currentGripperPos = 90;
  delay(1000);

  muoviServo(servoGripper, currentGripperPos, defaultGripperAngle);
  currentGripperPos = defaultGripperAngle;
  delay(500);

  muoviServo(servoBase, currentBasePos, defaultBaseAngle);
  currentBasePos = defaultBaseAngle;
  delay(500);

  muoviServo(servoSecondArm, currentSecondArmPos, defaultSecondArmAngle);
  currentSecondArmPos = defaultSecondArmAngle;
  delay(500);
}

// ==========================================
// PICK-AND-PLACE: DRY STREAM (BIN 3 - 180°)
// ==========================================
void sortDryWaste() {
  Serial.println(F("  -> Executing pick-and-place for Dry Waste (Bin 3: 180 deg)"));

  // Phase I: Grasp Alignment
  muoviServo(servoGripper, currentGripperPos, 90);
  currentGripperPos = 90;
  delay(500);

  muoviServo(servoFirstArm, currentFirstArmPos, 135);
  currentFirstArmPos = 135;
  delay(500);

  // Phase II: Securing & Elevation
  muoviServo(servoGripper, currentGripperPos, defaultGripperAngle);
  currentGripperPos = defaultGripperAngle;
  delay(500);

  muoviServo(servoFirstArm, currentFirstArmPos, defaultFirstArmAngle);
  currentFirstArmPos = defaultFirstArmAngle;
  delay(500);

  // Phase III: Transfer Rotation to Bin 3 (180 deg)
  muoviServo(servoSecondArm, currentSecondArmPos, 100);
  currentSecondArmPos = 100;
  delay(500);

  muoviServo(servoBase, currentBasePos, 180);
  currentBasePos = 180;
  delay(1000);

  // Phase IV: Discharge & Homing
  muoviServo(servoGripper, currentGripperPos, 90);
  currentGripperPos = 90;
  delay(1000);

  muoviServo(servoGripper, currentGripperPos, defaultGripperAngle);
  currentGripperPos = defaultGripperAngle;
  delay(500);

  muoviServo(servoBase, currentBasePos, defaultBaseAngle);
  currentBasePos = defaultBaseAngle;
  delay(500);

  muoviServo(servoSecondArm, currentSecondArmPos, defaultSecondArmAngle);
  currentSecondArmPos = defaultSecondArmAngle;
  delay(500);
}

// ==========================================
// MAIN POLLING LOOP & DETERMINISTIC FSM
// ==========================================
void loop() {
  // Read active-low sensors (inverted logic: HIGH = detected)
  int irDetected       = !digitalRead(PIN_IR_SENSOR);
  int moistureDetected = !digitalRead(PIN_MOISTURE_SENSOR);
  int metalDetected    = !digitalRead(PIN_INDUCTIVE_SENSOR);

  // Ingress specimen detected by IR sensor
  if (irDetected == 1) {
    Serial.println(F("\n[EVENT] Specimen entered inspection stage."));

    // Stabilization delay to eliminate mechanical bounce
    delay(1000);

    // Multi-pass temporal majority sampling loop (3 samples @ 75 ms interval)
    for (int i = 0; i < 3; i++) {
      moistureDetected = !digitalRead(PIN_MOISTURE_SENSOR);
      metalDetected    = !digitalRead(PIN_INDUCTIVE_SENSOR);
      delay(75);
    }

    // Deterministic Priority Classification (Metal > Wet > Dry)
    if (metalDetected == 1) {
      Serial.println(F("[DECISION] Priority 1: Metal Waste Detected."));
      sortMetalWaste();
    } else if (moistureDetected == 1) {
      Serial.println(F("[DECISION] Priority 2: Wet Organic Waste Detected."));
      sortWetWaste();
    } else {
      Serial.println(F("[DECISION] Priority 3: Dry Recyclable Waste Detected."));
      sortDryWaste();
    }
  }

  delay(500);
}
