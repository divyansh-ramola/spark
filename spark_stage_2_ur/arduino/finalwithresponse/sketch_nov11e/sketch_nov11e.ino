/*
  Dual Stepper Motor Control for ROS2 (No Extra Steps After Homing)

  Commands:
  - 'h': Homes both motors in sequence
  - 'f': Moves Motor 0 in homing direction until away limit switch detected (pin 45 HIGH)
  - 'm': Moves Motor 0 away from home until away limit switch is triggered
  - 'n': Moves Motor 1 away from home
  - 'p': Moves Motor 0 towards home (fixed steps defined in config)
  - 'q': Moves Motor 1 towards home (fixed steps defined in config)
  - 'a': Moves Motor 1 forward (custom steps)
  - 'b': Moves Motor 1 backward (custom steps)
  
  Response: 1 = success, 0 = failure
*/

// --- MOTOR 0 PINS & CONFIG ---
#define STEP_PIN_0 3
#define DIR_PIN_0  4
#define LIMIT_SWITCH_PIN_0 32  // Home limit switch
#define AWAY_LIMIT_SWITCH_PIN_0 45  // Away limit switch (normally open)
#define HOMING_DIR_0 LOW
#define FIND_DIR_0 LOW  // Find direction (same as homing)
int stepDelay_0 = 800;
int stepDelay_0_find = 2000;
// --- FIND METAL CONFIG ---
#define MAX_FIND_STEPS_0 2200  // Maximum steps allowed for 'f' command (safety limit)

long maxHomingSteps_0 = 200000;

// --- MOTOR 1 PINS & CONFIG ---
#define STEP_PIN_1 6
#define DIR_PIN_1  7
#define LIMIT_SWITCH_PIN_1 31
#define HOMING_DIR_1 LOW
int stepDelay_1 = 800;
long maxHomingSteps_1 = 200000;

// --- SENSOR (MOTOR 1, reused for M0 find) ---
#define SENSOR_PIN_1 30
#define FIND_DIR_1 LOW

// --- MOVE AWAY CONFIG ---
#define MOVE_AWAY_STEPS_1 3000

// --- MOVE TOWARDS HOME CONFIG ---
#define MOVE_TOWARDS_HOME_STEPS_0 800
#define MOVE_TOWARDS_HOME_STEPS_1 500

// --- MANUAL MOVE CONFIG (NEW) ---
#define MOVE_MOTOR1_FORWARD_STEPS 400  // 'a' command steps
#define MOVE_MOTOR1_BACKWARD_STEPS 350  // 'b' command steps

// --- GLOBAL VARIABLES ---
bool motor0Stopped = false;
bool motor1Stopped = false;

// =========================================================================
void takeOneStep0() {
  digitalWrite(STEP_PIN_0, HIGH);
  delayMicroseconds(500);
  digitalWrite(STEP_PIN_0, LOW);
  delayMicroseconds(stepDelay_0);
}

void takeOneStep1() {
  digitalWrite(STEP_PIN_1, HIGH);
  delayMicroseconds(500);
  digitalWrite(STEP_PIN_1, LOW);
  delayMicroseconds(stepDelay_1);
}

// =========================================================================
bool homeMotor0() {
  long stepCount = 0;
  bool homingFailed = false;

  // If already pressed, back off
  if (digitalRead(LIMIT_SWITCH_PIN_0) == LOW) {
    digitalWrite(DIR_PIN_0, !HOMING_DIR_0);
    while (stepCount < maxHomingSteps_0) {
      takeOneStep0();
      stepCount++;
      if (digitalRead(LIMIT_SWITCH_PIN_0) == HIGH) break;
    }
    if (stepCount >= maxHomingSteps_0) homingFailed = true;
  }

  stepCount = 0;

  if (!homingFailed) {
    digitalWrite(DIR_PIN_0, HOMING_DIR_0);
    while (stepCount < maxHomingSteps_0) {
      takeOneStep0();
      stepCount++;
      if (digitalRead(LIMIT_SWITCH_PIN_0) == LOW) {
        motor0Stopped = true;
        break;
      }
    }
    if (!motor0Stopped) homingFailed = true;
  }

  motor0Stopped = true;
  return !homingFailed;
}

bool homeMotor1() {
  long stepCount = 0;
  bool homingFailed = false;

  if (digitalRead(LIMIT_SWITCH_PIN_1) == LOW) {
    digitalWrite(DIR_PIN_1, !HOMING_DIR_1);
    while (stepCount < maxHomingSteps_1) {
      takeOneStep1();
      stepCount++;
      if (digitalRead(LIMIT_SWITCH_PIN_1) == HIGH) break;
    }
    if (stepCount >= maxHomingSteps_1) homingFailed = true;
  }

  stepCount = 0;

  if (!homingFailed) {
    digitalWrite(DIR_PIN_1, HOMING_DIR_1);
    while (stepCount < maxHomingSteps_1) {
      takeOneStep1();
      stepCount++;
      if (digitalRead(LIMIT_SWITCH_PIN_1) == LOW) {
        motor1Stopped = true;
        break;
      }
    }
    if (!motor1Stopped) homingFailed = true;
  }

  motor1Stopped = true;
  return !homingFailed;
}

bool findMetalMotor0() {
  Serial.println("=== Finding Metal (Motor 0) ===");
  long stepCount = 0;
  motor0Stopped = false;

  if (digitalRead(SENSOR_PIN_1) == HIGH) {
    Serial.println("Metal already detected.");
    motor0Stopped = true;
    return true;
  }

  digitalWrite(DIR_PIN_0, FIND_DIR_0);
  Serial.println("M0 moving to find metal...");

  while (stepCount < MAX_FIND_STEPS_0) {  // <-- Limited by MAX_FIND_STEPS_0
    takeOneStep0();
    stepCount++;

    if (digitalRead(SENSOR_PIN_1) == HIGH) {
      Serial.println("M0 Metal Found! Stopping immediately.");
      motor0Stopped = true;
      break;
    }
  }

  if (!motor0Stopped) {
    Serial.println("!!! M0 FIND METAL FAILED (Max steps reached) !!!");
    motor0Stopped = true;
    return false;
  }

  return true;
}


bool moveAway0() {
  long stepCount = 0;
  bool movingFailed = false;

  if (digitalRead(AWAY_LIMIT_SWITCH_PIN_0) == HIGH) {
    digitalWrite(DIR_PIN_0, !HOMING_DIR_0);
    while (stepCount < maxHomingSteps_0) {
      takeOneStep0();
      stepCount++;
      if (digitalRead(AWAY_LIMIT_SWITCH_PIN_0) == LOW) break;
    }
    if (stepCount >= maxHomingSteps_0) movingFailed = true;
  }

  stepCount = 0;
  if (!movingFailed) {
    digitalWrite(DIR_PIN_0, HOMING_DIR_0);
    while (stepCount < maxHomingSteps_0) {
      takeOneStep0();
      stepCount++;
      if (digitalRead(AWAY_LIMIT_SWITCH_PIN_0) == HIGH) {
        motor0Stopped = true;
        break;
      }
    }
    if (!motor0Stopped) movingFailed = true;
  }

  motor0Stopped = true;
  return !movingFailed;
}

bool moveAway1() {
  digitalWrite(DIR_PIN_1, !HOMING_DIR_1);
  for (long i = 0; i < MOVE_AWAY_STEPS_1; i++) takeOneStep1();
  motor1Stopped = true;
  return true;
}

bool moveTowardsHome0() {
  digitalWrite(DIR_PIN_0, HOMING_DIR_0);
  for (long i = 0; i < MOVE_TOWARDS_HOME_STEPS_0; i++) {
    takeOneStep0();
    if (digitalRead(LIMIT_SWITCH_PIN_0) == LOW) break;
  }
  motor0Stopped = true;
  return true;
}

bool moveTowardsHome1() {
  digitalWrite(DIR_PIN_1, HOMING_DIR_1);
  for (long i = 0; i < MOVE_TOWARDS_HOME_STEPS_1; i++) {
    takeOneStep1();
    if (digitalRead(LIMIT_SWITCH_PIN_1) == LOW) break;
  }
  motor1Stopped = true;
  return true;
}

// --- NEW COMMANDS ---
// Move Motor 1 Forward
bool moveMotor1Forward() {
  digitalWrite(DIR_PIN_1, !HOMING_DIR_1);
  for (long i = 0; i < MOVE_MOTOR1_FORWARD_STEPS; i++) takeOneStep1();
  motor1Stopped = true;
  return true;
}

// Move Motor 1 Backward
bool moveMotor1Backward() {
  digitalWrite(DIR_PIN_1, HOMING_DIR_1);
  for (long i = 0; i < MOVE_MOTOR1_BACKWARD_STEPS; i++) takeOneStep1();
  motor1Stopped = true;
  return true;
}

// =========================================================================
void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN_0, OUTPUT);
  pinMode(DIR_PIN_0, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN_0, INPUT_PULLUP);
  pinMode(AWAY_LIMIT_SWITCH_PIN_0, INPUT_PULLUP);

  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN_1, INPUT_PULLUP);

  pinMode(SENSOR_PIN_1, INPUT);

  delay(2000);
  Serial.println("READY");
}

// =========================================================================
void loop() {
  if (Serial.available() > 0) {
    String commandStr = Serial.readStringUntil('\n');
    commandStr.trim();
    if (commandStr.length() == 0) return;

    char command = commandStr[0];
    bool success = false;

    // Execute command and send appropriate response
    if (command == 'h') {
      success = homeMotor0() && homeMotor1();
      if (success) Serial.println("HOME_COMPLETE");
      else Serial.println("HOME_FAILED");
    }
    else if (command == 'f') {
      success = findMetalMotor0();
      if (success) Serial.println("FIND_METAL_COMPLETE");
      else Serial.println("FIND_METAL_FAILED");
    }
    else if (command == 'm') {
      success = moveAway0();
      if (success) Serial.println("MOVE_M0_AWAY_COMPLETE");
      else Serial.println("MOVE_M0_AWAY_FAILED");
    }
    else if (command == 'n') {
      success = moveAway1();
      if (success) Serial.println("MOVE_M1_AWAY_COMPLETE");
      else Serial.println("MOVE_M1_AWAY_FAILED");
    }
    else if (command == 'p') {
      success = moveTowardsHome0();
      if (success) Serial.println("MOVE_M0_HOME_COMPLETE");
      else Serial.println("MOVE_M0_HOME_FAILED");
    }
    else if (command == 'q') {
      success = moveTowardsHome1();
      if (success) Serial.println("MOVE_M1_HOME_COMPLETE");
      else Serial.println("MOVE_M1_HOME_FAILED");
    }
    else if (command == 'a') {
      success = moveMotor1Forward();
      if (success) Serial.println("MOVE_M1_FORWARD_COMPLETE");
      else Serial.println("MOVE_M1_FORWARD_FAILED");
    }
    else if (command == 'b') {
      success = moveMotor1Backward();
      if (success) Serial.println("MOVE_M1_BACKWARD_COMPLETE");
      else Serial.println("MOVE_M1_BACKWARD_FAILED");
    }
    else {
      Serial.println("COMMAND_INVALID");
      return;
    }
  }
}
