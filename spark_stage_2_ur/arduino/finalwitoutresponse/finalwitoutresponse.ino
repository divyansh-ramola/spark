/*
  Dual Stepper Motor Homing (With Serial Response Mechanism)

  This code waits for serial commands and sends acknowledgments:
  - 'h': Homes both motors in sequence
  - 'f': Moves Motor 0 until metal detected (pin 30 HIGH)
  - 'm': Moves Motor 0 away from home
  - 'n': Moves Motor 1 away from home
  - 'p': Moves Motor 0 towards home (fixed steps defined in config)
  - 'q': Moves Motor 1 towards home (fixed steps defined in config)
  
  Response Format:
  - ACK:<command> - Command received
  - DONE:<command> - Command completed successfully
  - ERROR:<command> - Command failed
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

// --- SENSOR (for Motor 0) ---
#define SENSOR_PIN_1 30
#define FIND_DIR_0 LOW
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
  Serial.println("=== Homing Motor 0 ===");
  long stepCount = 0;
  bool homingFailed = false;

  if (digitalRead(LIMIT_SWITCH_PIN_0) == LOW) {
    Serial.println("WARNING: M0 Switch pressed. Backing off...");
    digitalWrite(DIR_PIN_0, !HOMING_DIR_0);

    while (stepCount < maxHomingSteps_0) {
      takeOneStep0();
      stepCount++;
      if (digitalRead(LIMIT_SWITCH_PIN_0) == HIGH) {
        Serial.println("M0 Switch clear (HIGH).");
        break;
      }
    }
    if (stepCount >= maxHomingSteps_0) {
      homingFailed = true;
    }
  }

  stepCount = 0;

  if (!homingFailed) {
    Serial.println("M0 Moving toward switch (looking for LOW)...");
    digitalWrite(DIR_PIN_0, HOMING_DIR_0);

    while (stepCount < maxHomingSteps_0) {
      takeOneStep0();
      stepCount++;
      if (digitalRead(LIMIT_SWITCH_PIN_0) == LOW) {
        Serial.println("M0 Limit switch HIT (LOW). Homing complete.");
        motor0Stopped = true;
        break;
      }
    }
    if (!motor0Stopped) {
      homingFailed = true;
    }
  }

  if (homingFailed) {
    Serial.println("!!! M0 HOMING FAILED (Max steps) !!!");
    motor0Stopped = true;
    return false;
  }
  return true;
}

bool homeMotor1() {
  Serial.println("=== Homing Motor 1 ===");
  long stepCount = 0;
  bool homingFailed = false;

  if (digitalRead(LIMIT_SWITCH_PIN_1) == LOW) {
    Serial.println("WARNING: M1 Switch pressed. Backing off...");
    digitalWrite(DIR_PIN_1, !HOMING_DIR_1);

    while (stepCount < maxHomingSteps_1) {
      takeOneStep1();
      stepCount++;
      if (digitalRead(LIMIT_SWITCH_PIN_1) == HIGH) {
        Serial.println("M1 Switch clear (HIGH).");
        break;
      }
    }
    if (stepCount >= maxHomingSteps_1) {
      homingFailed = true;
    }
  }

  stepCount = 0;

  if (!homingFailed) {
    Serial.println("M1 Moving toward switch (looking for LOW)...");
    digitalWrite(DIR_PIN_1, HOMING_DIR_1);

    while (stepCount < maxHomingSteps_1) {
      takeOneStep1();
      stepCount++;
      if (digitalRead(LIMIT_SWITCH_PIN_1) == LOW) {
        Serial.println("M1 Limit switch HIT (LOW). Homing complete.");
        motor1Stopped = true;
        break;
      }
    }
    if (!motor1Stopped) {
      homingFailed = true;
    }
  }

  if (homingFailed) {
    Serial.println("!!! M1 HOMING FAILED (Max steps) !!!");
    motor1Stopped = true;
    return false;
  }
  return true;
}

// =========================================================================
// 🆕 Modified function — Find Metal using Motor 0
bool findMetalMotor0() {
  Serial.println("=== Finding Metal (Motor 0) ===");
  long stepCount = 0;
  motor0Stopped = false;

  if (digitalRead(SENSOR_PIN_1) == HIGH) {
    Serial.println("WARNING: Metal already detected (Pin 30 is HIGH).");
    motor0Stopped = true;
    return true;
  }

  Serial.println("M0 Moving to find metal (looking for HIGH on pin 30)...");
  digitalWrite(DIR_PIN_0, FIND_DIR_0);

  while (stepCount < maxHomingSteps_0) {
    takeOneStep0();
    stepCount++;

    if (digitalRead(SENSOR_PIN_1) == HIGH) {
      Serial.println("M0 Metal Found (HIGH).");
      Serial.print("Moving ");
      Serial.print(EXTRA_STEPS_AFTER_FIND);
      Serial.println(" extra steps...");

      delay(1000);

      for (long i = 0; i < EXTRA_STEPS_AFTER_FIND; i++) {
        takeOneStep0();
      }
      Serial.println("Extra steps complete. Stopping.");
      motor0Stopped = true;
      break;
    }
  }

  if (!motor0Stopped) {
    Serial.println("!!! M0 FIND METAL FAILED (Max steps) !!!");
    motor0Stopped = true;
    return false;
  }
  return true;
}
// =========================================================================

bool moveAway0() {
  Serial.println("=== Moving Motor 0 Away ===");
  digitalWrite(DIR_PIN_0, !HOMING_DIR_0);
  Serial.print("M0 Moving ");
  Serial.print(MOVE_AWAY_STEPS_0);
  Serial.println(" steps...");

  for (long i = 0; i < MOVE_AWAY_STEPS_0; i++) {
    takeOneStep0();
  }

  Serial.println("M0 Move complete.");
  motor0Stopped = true;
  return true;
}

bool moveAway1() {
  Serial.println("=== Moving Motor 1 Away ===");
  digitalWrite(DIR_PIN_1, !HOMING_DIR_1);
  Serial.print("M1 Moving ");
  Serial.print(MOVE_AWAY_STEPS_1);
  Serial.println(" steps...");

  for (long i = 0; i < MOVE_AWAY_STEPS_1; i++) {
    takeOneStep1();
  }

  Serial.println("M1 Move complete.");
  motor1Stopped = true;
  return true;
}

bool moveTowardsHome0() {
  Serial.println("=== Moving Motor 0 Towards Home ===");
  digitalWrite(DIR_PIN_0, HOMING_DIR_0);
  Serial.print("M0 Moving ");
  Serial.print(MOVE_TOWARDS_HOME_STEPS_0);
  Serial.println(" steps towards home...");

  for (long i = 0; i < MOVE_TOWARDS_HOME_STEPS_0; i++) {
    takeOneStep0();
    if (digitalRead(LIMIT_SWITCH_PIN_0) == LOW) {
      Serial.println("M0 Limit switch hit during movement. Stopping.");
      break;
    }
  }

  Serial.println("M0 Move towards home complete.");
  motor0Stopped = true;
  return true;
}

bool moveTowardsHome1() {
  Serial.println("=== Moving Motor 1 Towards Home ===");
  digitalWrite(DIR_PIN_1, HOMING_DIR_1);
  Serial.print("M1 Moving ");
  Serial.print(MOVE_TOWARDS_HOME_STEPS_1);
  Serial.println(" steps towards home...");

  for (long i = 0; i < MOVE_TOWARDS_HOME_STEPS_1; i++) {
    takeOneStep1();
    if (digitalRead(LIMIT_SWITCH_PIN_1) == LOW) {
      Serial.println("M1 Limit switch hit during movement. Stopping.");
      break;
    }
  }

  Serial.println("M1 Move towards home complete.");
  motor1Stopped = true;
  return true;
}

// =========================================================================
void setup() {
  Serial.begin(115200);
  Serial.println("READY");

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

  Serial.println("Setup complete.");
  Serial.println("Commands: h=home, f=find metal (M0), m=move M0 away, n=move M1 away");
  Serial.println("          p=M0 towards home, q=M1 towards home");
}

// =========================================================================
void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    bool success = false;

    if (command == 'h' || command == 'H') {
      Serial.println("ACK:h");
      motor0Stopped = false;
      motor1Stopped = false;
      bool m0 = homeMotor0();
      bool m1 = homeMotor1();
      success = m0 && m1;
      if (success) Serial.println("DONE:h");
      else Serial.println("ERROR:h");
    }

    else if (command == 'f' || command == 'F') {
      Serial.println("ACK:f");
      motor0Stopped = false;
      success = findMetalMotor0();   // 👈 now uses Motor 0
      if (success) Serial.println("DONE:f");
      else Serial.println("ERROR:f");
    }

    else if (command == 'm' || command == 'M') {
      Serial.println("ACK:m");
      motor0Stopped = false;
      success = moveAway0();
      if (success) Serial.println("DONE:m");
      else Serial.println("ERROR:m");
    }

    else if (command == 'n' || command == 'N') {
      Serial.println("ACK:n");
      motor1Stopped = false;
      success = moveAway1();
      if (success) Serial.println("DONE:n");
      else Serial.println("ERROR:n");
    }

    else if (command == 'p' || command == 'P') {
      Serial.println("ACK:p");
      motor0Stopped = false;
      success = moveTowardsHome0();
      if (success) Serial.println("DONE:p");
      else Serial.println("ERROR:p");
    }

    else if (command == 'q' || command == 'Q') {
      Serial.println("ACK:q");
      motor1Stopped = false;
      success = moveTowardsHome1();
      if (success) Serial.println("DONE:q");
      else Serial.println("ERROR:q");
    }
  }
}
