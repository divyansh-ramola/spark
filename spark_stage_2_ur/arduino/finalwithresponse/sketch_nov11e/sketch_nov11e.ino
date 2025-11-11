/*
  Dual Stepper Motor Control for ROS2

  Commands:
  - 'h': Homes both motors in sequence
  - 'f': Moves Motor 1 until metal detected (pin 30 HIGH)
  - 'm': Moves Motor 0 away from home
  - 'n': Moves Motor 1 away from home
  - 'p': Moves Motor 0 towards home (fixed steps defined in config)
  - 'q': Moves Motor 1 towards home (fixed steps defined in config)
  
  Response: 1 = success, 0 = failure
*/

// --- MOTOR 0 PINS & CONFIG ---
#define STEP_PIN_0 3
#define DIR_PIN_0  4
#define LIMIT_SWITCH_PIN_0 31
#define HOMING_DIR_0 LOW
int stepDelay_0 = 800;
long maxHomingSteps_0 = 200000;

// --- MOTOR 1 PINS & CONFIG ---
#define STEP_PIN_1 6
#define DIR_PIN_1  7
#define LIMIT_SWITCH_PIN_1 32
#define HOMING_DIR_1 LOW
int stepDelay_1 = 800;
long maxHomingSteps_1 = 200000;

// --- SENSOR (MOTOR 1) ---
#define SENSOR_PIN_1 30
#define FIND_DIR_1 LOW
#define EXTRA_STEPS_AFTER_FIND 100

// --- MOVE AWAY CONFIG ---
#define MOVE_AWAY_STEPS_0 2000
#define MOVE_AWAY_STEPS_1 1000

// --- MOVE TOWARDS HOME CONFIG ---
#define MOVE_TOWARDS_HOME_STEPS_0 1000
#define MOVE_TOWARDS_HOME_STEPS_1 500

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

  if (digitalRead(LIMIT_SWITCH_PIN_0) == LOW) {
    digitalWrite(DIR_PIN_0, !HOMING_DIR_0);

    while (stepCount < maxHomingSteps_0) {
      takeOneStep0();
      stepCount++;
      if (digitalRead(LIMIT_SWITCH_PIN_0) == HIGH) {
        break;
      }
    }
    if (stepCount >= maxHomingSteps_0) {
      homingFailed = true;
    }
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
    if (!motor0Stopped) {
      homingFailed = true;
    }
  }

  if (homingFailed) {
    motor0Stopped = true;
    return false;
  }
  return true;
}

bool homeMotor1() {
  long stepCount = 0;
  bool homingFailed = false;

  if (digitalRead(LIMIT_SWITCH_PIN_1) == LOW) {
    digitalWrite(DIR_PIN_1, !HOMING_DIR_1);

    while (stepCount < maxHomingSteps_1) {
      takeOneStep1();
      stepCount++;
      if (digitalRead(LIMIT_SWITCH_PIN_1) == HIGH) {
        break;
      }
    }
    if (stepCount >= maxHomingSteps_1) {
      homingFailed = true;
    }
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
    if (!motor1Stopped) {
      homingFailed = true;
    }
  }

  if (homingFailed) {
    motor1Stopped = true;
    return false;
  }
  return true;
}

bool findMetalMotor1() {
  long stepCount = 0;
  motor1Stopped = false;

  if (digitalRead(SENSOR_PIN_1) == HIGH) {
    motor1Stopped = true;
    return true;
  }

  digitalWrite(DIR_PIN_1, FIND_DIR_1);

  while (stepCount < maxHomingSteps_1) {
    takeOneStep1();
    stepCount++;

    if (digitalRead(SENSOR_PIN_1) == HIGH) {
      delay(1000);

      for (long i = 0; i < EXTRA_STEPS_AFTER_FIND; i++) {
        takeOneStep1();
      }
      motor1Stopped = true;
      break;
    }
  }

  if (!motor1Stopped) {
    motor1Stopped = true;
    return false;
  }
  return true;
}

bool moveAway0() {
  digitalWrite(DIR_PIN_0, !HOMING_DIR_0);

  for (long i = 0; i < MOVE_AWAY_STEPS_0; i++) {
    takeOneStep0();
  }

  motor0Stopped = true;
  return true;
}

bool moveAway1() {
  digitalWrite(DIR_PIN_1, !HOMING_DIR_1);

  for (long i = 0; i < MOVE_AWAY_STEPS_1; i++) {
    takeOneStep1();
  }

  motor1Stopped = true;
  return true;
}

bool moveTowardsHome0() {
  digitalWrite(DIR_PIN_0, HOMING_DIR_0);

  for (long i = 0; i < MOVE_TOWARDS_HOME_STEPS_0; i++) {
    takeOneStep0();
    if (digitalRead(LIMIT_SWITCH_PIN_0) == LOW) {
      break;
    }
  }

  motor0Stopped = true;
  return true;
}

bool moveTowardsHome1() {
  digitalWrite(DIR_PIN_1, HOMING_DIR_1);

  for (long i = 0; i < MOVE_TOWARDS_HOME_STEPS_1; i++) {
    takeOneStep1();
    if (digitalRead(LIMIT_SWITCH_PIN_1) == LOW) {
      break;
    }
  }

  motor1Stopped = true;
  return true;
}

// =========================================================================
void setup() {
  Serial.begin(115200);

  digitalWrite(STEP_PIN_0, LOW);
  digitalWrite(DIR_PIN_0, LOW);
  digitalWrite(STEP_PIN_1, LOW);
  digitalWrite(DIR_PIN_1, LOW);

  pinMode(STEP_PIN_0, OUTPUT);
  pinMode(DIR_PIN_0, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN_0, INPUT_PULLUP);

  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN_1, INPUT_PULLUP);

  pinMode(SENSOR_PIN_1, INPUT);
  
  delay(2000);  // Wait for serial connection
  Serial.println("System ready for ROS2 commands...");
  Serial.println("READY");
}

// =========================================================================
void loop() {
  if (Serial.available() > 0) {
    String commandStr = Serial.readStringUntil('\n');
    commandStr.trim();
    
    if (commandStr.length() == 0) {
      return;  // Ignore empty commands
    }
    
    char command = commandStr[0];
    bool success = false;

    Serial.print("Received command: ");
    Serial.println(command);

    if (command == 'h' || command == 'H') {
      Serial.println("Starting home sequence...");
      motor0Stopped = false;
      motor1Stopped = false;
      bool m0 = homeMotor0();
      bool m1 = homeMotor1();
      success = m0 && m1;
      if (success) {
        Serial.println("HOME_COMPLETE");
      } else {
        Serial.println("HOME_FAILED");
      }
    }
    else if (command == 'f' || command == 'F') {
      Serial.println("Starting find metal sequence...");
      motor1Stopped = false;
      success = findMetalMotor1();
      if (success) {
        Serial.println("FIND_METAL_COMPLETE");
      } else {
        Serial.println("FIND_METAL_FAILED");
      }
    }
    else if (command == 'm' || command == 'M') {
      Serial.println("Moving Motor 0 away...");
      motor0Stopped = false;
      success = moveAway0();
      if (success) {
        Serial.println("MOVE_M0_AWAY_COMPLETE");
      } else {
        Serial.println("MOVE_M0_AWAY_FAILED");
      }
    }
    else if (command == 'n' || command == 'N') {
      Serial.println("Moving Motor 1 away...");
      motor1Stopped = false;
      success = moveAway1();
      if (success) {
        Serial.println("MOVE_M1_AWAY_COMPLETE");
      } else {
        Serial.println("MOVE_M1_AWAY_FAILED");
      }
    }
    else if (command == 'p' || command == 'P') {
      Serial.println("Moving Motor 0 towards home...");
      motor0Stopped = false;
      success = moveTowardsHome0();
      if (success) {
        Serial.println("MOVE_M0_HOME_COMPLETE");
      } else {
        Serial.println("MOVE_M0_HOME_FAILED");
      }
    }
    else if (command == 'q' || command == 'Q') {
      Serial.println("Moving Motor 1 towards home...");
      motor1Stopped = false;
      success = moveTowardsHome1();
      if (success) {
        Serial.println("MOVE_M1_HOME_COMPLETE");
      } else {
        Serial.println("MOVE_M1_HOME_FAILED");
      }
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(command);
      Serial.println("COMMAND_FAILED");
    }
  }
}