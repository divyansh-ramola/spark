// =======================
// === PIN DEFINITIONS ===
// =======================

// Stepper 1 (TB6600 - Linear Axis)
#define TB_STEP_PIN     3    // Step pulse pin
#define TB_DIR_PIN      2    // Direction control pin
#define TB_ENABLE_PIN   4    // Enable pin (active LOW)
#define TB_LIMIT_PIN    22   // Limit switch pin (Normally Open + Common)

// Stepper 2 (DRV8825 - Gripper)
#define DRV_STEP_PIN    5    // Step pulse pin
#define DRV_DIR_PIN     6    // Direction control pin
#define DRV_ENABLE_PIN  7    // Enable pin (active LOW)
#define DRV_LIMIT_PIN   23   // Limit switch pin for gripper open detection (NO + C)

// Servo Gripper
#define SERVO_PIN       8    // PWM signal pin for servo
#define SERVO_LIMIT_PIN 24   // Limit switch pin for servo gripper (NO + C)

// Relay Control
#define RELAY_PIN       10   // Relay control pin

// ============================
// === USER CONFIGURATION  ====
// ============================

const float leadScrewPitchMM = 2.0;
const int stepsPerRevolution = 200;
const int microsteppingTB6600 = 2;
const int stepsPerMM = (stepsPerRevolution * microsteppingTB6600) / leadScrewPitchMM;
const int tbMoveSpeedUS = 50;
const int tbHomingSpeedUS = 50;
const bool tbHomingDir = LOW;
const bool tbWorkingDir = HIGH;
const float tbTravelAfterHomeMM = 69.28;

const int drvMicrostepping = 1;
const int drvStepsPerRev = 200;
const int drvStepsToClose = 80;
const int drvStepDelayUS = 30000;
const int drvPulseWidthUS = 20;
const bool gripperOpenDirection  = HIGH;
const bool gripperCloseDirection = LOW;

const int servoCloseRotationDegrees = 150;
const int servoStepDelay = 5;
const int servoStartAngle = 90;

bool relayState = LOW;

float x = 0.0;
float moveDistance = 70;
float moveDistance1 = 2.0;

// ====================
// === LIBRARIES ======
// ====================
#include <Servo.h>
Servo servoGripper;

// =====================
// === SETUP BLOCK =====
// =====================
void setup() {
  pinMode(TB_STEP_PIN, OUTPUT);
  pinMode(TB_DIR_PIN, OUTPUT);
  pinMode(TB_ENABLE_PIN, OUTPUT);
  pinMode(TB_LIMIT_PIN, INPUT_PULLUP);

  pinMode(DRV_STEP_PIN, OUTPUT);
  pinMode(DRV_DIR_PIN, OUTPUT);
  pinMode(DRV_ENABLE_PIN, OUTPUT);
  pinMode(DRV_LIMIT_PIN, INPUT_PULLUP);

  pinMode(SERVO_LIMIT_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, relayState);

  digitalWrite(TB_STEP_PIN, LOW);
  digitalWrite(DRV_STEP_PIN, LOW);

  digitalWrite(TB_ENABLE_PIN, LOW);
  digitalWrite(DRV_ENABLE_PIN, LOW);

  servoGripper.attach(SERVO_PIN);
  servoGripper.write(servoStartAngle);

  Serial.begin(9600);
  Serial.println("System ready for ROS2 commands...");
  Serial.println("Commands:");
  Serial.println("SEQ1 - Execute Sequence 1 (close stepper, open servo, home, close servo, move)");
  Serial.println("SEQ2 - Open both grippers + short move");
  Serial.println("SET_X <value> - Set x variable for distance calculation");
  Serial.println("TOGGLE_RELAY - Toggle relay state (LOW <-> HIGH)");
  Serial.print("Initial relay state: ");
  Serial.println(relayState ? "HIGH" : "LOW");
}

// =====================
// === MAIN LOOP =======
// =====================
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    processCommand(command);
  }

  // Ensure stepper driver remains enabled for holding torque
  digitalWrite(DRV_ENABLE_PIN, LOW);
}

// ===============================
// === PROCESS SERIAL COMMANDS ===
// ===============================
void processCommand(String cmd) {
  Serial.print("Received command: ");
  Serial.println(cmd);

  if (cmd == "SEQ1") {
    executeSequence1();
  } else if (cmd == "SEQ2") {
    executeSequence2();
  } else if (cmd == "TOGGLE_RELAY") {
    toggleRelay();
  } else if (cmd.startsWith("SET_X ")) {
    float newX = cmd.substring(6).toFloat();
    x = newX;
    moveDistance = 70.0;
    Serial.print("X variable set to: ");
    Serial.println(x);
  } else {
    Serial.println("Unknown command. Available: SEQ1, SEQ2, SET_X <val>, TOGGLE_RELAY");
  }
}

// ========================
// === RELAY CONTROL ======
// ========================
void toggleRelay() {
  relayState = !relayState;
  digitalWrite(RELAY_PIN, relayState);
  Serial.print("Relay toggled to: ");
  Serial.println(relayState ? "HIGH" : "LOW");
}

// ========================
// === SEQUENCE 1 =========
// ========================
void executeSequence1() {
  Serial.println("=== EXECUTING SEQUENCE 1 ===");

  
  openServoGripper(); delay(500);
  homeLinearAxis(); delay(500);
  closeStepperGripper(); delay(500);
  delay(500);
  closeServoGripper(); 
  

  Serial.print("Step 5: Moving linear axis by ");
  Serial.print(moveDistance);
  Serial.println(" mm...");
  moveLinearAxis(moveDistance);

  Serial.println("=== SEQUENCE 1 COMPLETE ===");
}

// ========================
// === SEQUENCE 2 =========
// ========================
void executeSequence2() {
  Serial.println("=== EXECUTING SEQUENCE 2 ===");

  openStepperGripper(); delay(200);
  openServoGripper(); delay(500);
  homeLinearAxis(); delay(500);
  moveLinearAxisShort(moveDistance1);

  Serial.println("=== SEQUENCE 2 COMPLETE ===");
}

// ============================
// === HOMING LOGIC ==========
// ============================
void homeLinearAxis() {
  digitalWrite(TB_DIR_PIN, tbHomingDir);
  Serial.println("Homing started...");

  while (true) {
    if (digitalRead(TB_LIMIT_PIN) == LOW) {
      delay(20);
      if (digitalRead(TB_LIMIT_PIN) == LOW) {
        Serial.println("Limit switch pressed — Homing complete.");
        break;
      }
    }
    digitalWrite(TB_STEP_PIN, HIGH); delayMicroseconds(tbHomingSpeedUS);
    digitalWrite(TB_STEP_PIN, LOW); delayMicroseconds(tbHomingSpeedUS);
  }
}

// ============================
// === LINEAR AXIS MOVE =======
// ============================
void moveLinearAxis(float mm) {
  long steps = mm * stepsPerMM;
  digitalWrite(TB_DIR_PIN, tbWorkingDir);
  for (long i = 0; i < steps; i++) {
    digitalWrite(TB_STEP_PIN, HIGH); delayMicroseconds(tbMoveSpeedUS);
    digitalWrite(TB_STEP_PIN, LOW); delayMicroseconds(tbMoveSpeedUS);
  }
  Serial.print("Linear axis moved ");
  Serial.print(mm);
  Serial.println(" mm");
}

void moveLinearAxisShort(float mm) {
  long steps = mm * stepsPerMM;
  digitalWrite(TB_DIR_PIN, tbWorkingDir);
  for (long i = 0; i < steps; i++) {
    digitalWrite(TB_STEP_PIN, HIGH); delayMicroseconds(tbMoveSpeedUS);
    digitalWrite(TB_STEP_PIN, LOW); delayMicroseconds(tbMoveSpeedUS);
  }
  Serial.print("Linear axis moved ");
  Serial.print(mm);
  Serial.println(" mm");
}

// ====================================
// === STEPPER GRIPPER - CLOSE ========
// ====================================
void closeStepperGripper() {
  Serial.println("Closing stepper gripper...");
  digitalWrite(DRV_DIR_PIN, gripperCloseDirection);

  for (int i = 0; i < drvStepsToClose; i++) {
    digitalWrite(DRV_STEP_PIN, HIGH); delayMicroseconds(drvPulseWidthUS);
    digitalWrite(DRV_STEP_PIN, LOW); delayMicroseconds(drvStepDelayUS);
    if (i % 20 == 0) {
      Serial.print("Step: "); Serial.println(i);
    }
  }
  Serial.println("Stepper gripper closed.");
}

// ====================================
// === STEPPER GRIPPER - OPEN =========
// ====================================
void openStepperGripper() {
  Serial.println("Opening stepper gripper...");
  digitalWrite(DRV_DIR_PIN, gripperOpenDirection);

  int stepCount = 0;
  while (digitalRead(DRV_LIMIT_PIN) != LOW) {
    digitalWrite(DRV_STEP_PIN, HIGH); delayMicroseconds(drvPulseWidthUS);
    digitalWrite(DRV_STEP_PIN, LOW); delayMicroseconds(drvStepDelayUS);
    stepCount++;
    if (stepCount > 1000) {
      Serial.println("Warning: Open limit not reached after 1000 steps!");
      break;
    }
    if (stepCount % 50 == 0) {
      Serial.print("Opening step: "); Serial.println(stepCount);
    }
  }
  Serial.print("Stepper gripper opened after "); Serial.println(stepCount);
}

// ====================================
// === SERVO GRIPPER - OPEN ===========
// ====================================
void openServoGripper() {
  Serial.println("Opening servo gripper...");
  int currentAngle = servoGripper.read();

  while (digitalRead(SERVO_LIMIT_PIN) == HIGH) {
    currentAngle--;
    currentAngle = constrain(currentAngle, 0, 180);
    servoGripper.write(currentAngle);
    delay(servoStepDelay);
    if (currentAngle <= 0) break;
  }

  if (digitalRead(SERVO_LIMIT_PIN) == LOW) {
    delay(20);
    if (digitalRead(SERVO_LIMIT_PIN) == LOW) {
      Serial.print("Servo opened at angle: "); Serial.println(currentAngle);
    }
  }
}

// ====================================
// === SERVO GRIPPER - CLOSE ==========
// ====================================
void closeServoGripper() {
  Serial.println("Closing servo gripper...");
  int currentAngle = servoGripper.read();

  while (digitalRead(SERVO_LIMIT_PIN) == HIGH) {
    currentAngle--;
    currentAngle = constrain(currentAngle, 0, 180);
    servoGripper.write(currentAngle);
    delay(servoStepDelay);
    if (currentAngle <= 0) break;
  }

  if (digitalRead(SERVO_LIMIT_PIN) == LOW) {
    delay(20);
    if (digitalRead(SERVO_LIMIT_PIN) == LOW) {
      Serial.print("Homed servo at angle: "); Serial.println(currentAngle);
    }
  }

  for (int step = 0; step < servoCloseRotationDegrees; step++) {
    currentAngle++;
    currentAngle = constrain(currentAngle, 0, 180);
    servoGripper.write(currentAngle);
    delay(servoStepDelay);
  }

  Serial.print("Servo gripper closed at angle: ");
  Serial.println(currentAngle);
}
