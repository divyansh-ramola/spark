/*
  Dual Stepper Motor Homing (Robust Logic on Serial Command)

  This code waits for serial commands:
  - 'h': Homes both motors in sequence (back-off logic).
  - 'f': Moves Motor 1 until a HIGH signal is read on pin 30,
         then moves a few extra steps.
  
  --------------------------------------------------------------------------
  --- PIN DEFINITIONS & CONFIGURATION ---
  --------------------------------------------------------------------------
  ... (Wiring notes are the same) ...
*/

// --- MOTOR 0 PINS & CONFIG ---
#define STEP_PIN_0 3
#define DIR_PIN_0  4
#define LIMIT_SWITCH_PIN_0 31 // Homing switch

#define HOMING_DIR_0 LOW // The one direction it moves *toward* the switch

int stepDelay_0 = 5000; // Delay in microseconds
long maxHomingSteps_0 = 200000; 


// --- MOTOR 1 PINS & CONFIG ---
#define STEP_PIN_1 6 
#define DIR_PIN_1  7 
#define LIMIT_SWITCH_PIN_1 32 // Homing switch

#define HOMING_DIR_1 LOW // The one direction it moves *toward* the switch

int stepDelay_1 = 1000; // Delay in microseconds
long maxHomingSteps_1 = 200000; 


// --- NEW SENSOR (MOTOR 1) ---
#define SENSOR_PIN_1 30 // Inductive sensor pin
#define FIND_DIR_1 LOW // Direction for M1 to move to find metal
#define EXTRA_STEPS_AFTER_FIND 100 // <-- **** NEW **** How many steps to move after trigger


// --- GLOBAL VARIABLES ---
bool motor0Stopped = false; 
bool motor1Stopped = false; 

// =========================================================================
//  HELPER FUNCTION: Motor 0 - Take one step
// =========================================================================
void takeOneStep0() {
  digitalWrite(STEP_PIN_0, HIGH);
  delayMicroseconds(500); 
  digitalWrite(STEP_PIN_0, LOW);
  delayMicroseconds(stepDelay_0);
}

// =========================================================================
//  HELPER FUNCTION: Motor 1 - Take one step
// =========================================================================
void takeOneStep1() {
  digitalWrite(STEP_PIN_1, HIGH);
  delayMicroseconds(500); 
  digitalWrite(STEP_PIN_1, LOW);
  delayMicroseconds(stepDelay_1);
}

// =========================================================================
//  ROBUST HOMING FUNCTION (MOTOR 0) - (Unchanged)
// =========================================================================
void homeMotor0() {
  Serial.println("=== Homing Motor 0 ===");
  long stepCount = 0;
  bool homingFailed = false;

  // --- 1. Check if switch is ALREADY pressed (LOW) ---
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

  // Reset step count
  stepCount = 0;

  // --- 2. Main Homing: Move toward switch (LOW) ---
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

  // --- 3. Final Status ---
  if (homingFailed) {
    Serial.println("!!! M0 HOMING FAILED (Max steps) !!!");
    motor0Stopped = true; 
  }
}

// =========================================================================
//  ROBUST HOMING FUNCTION (MOTOR 1) - (Unchanged)
// =========================================================================
void homeMotor1() {
  Serial.println("=== Homing Motor 1 ===");
  long stepCount = 0;
  bool homingFailed = false;

  // --- 1. Check if switch is ALREADY pressed (LOW) ---
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

  // Reset step count
  stepCount = 0;

  // --- 2. Main Homing: Move toward switch (LOW) ---
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

  // --- 3. Final Status ---
  if (homingFailed) {
    Serial.println("!!! M1 HOMING FAILED (Max steps) !!!");
    motor1Stopped = true; 
  }
}

// =========================================================================
//  MODIFIED FUNCTION: Find Metal (MOTOR 1) + Extra Steps
// =========================================================================
void findMetalMotor1() {
  Serial.println("=== Finding Metal (Motor 1) ===");
  long stepCount = 0;
  motor1Stopped = false; // Reset the flag

  // Check if metal is *already* detected
  if (digitalRead(SENSOR_PIN_1) == HIGH) {
    Serial.println("WARNING: Metal already detected (Pin 30 is HIGH).");
    motor1Stopped = true;
    return; // Don't move
  }

  // --- Main Move: Move toward sensor (HIGH) ---
  Serial.println("M1 Moving to find metal (looking for HIGH on pin 30)...");
  digitalWrite(DIR_PIN_1, FIND_DIR_1); 
    
  while (stepCount < maxHomingSteps_1) {
    takeOneStep1(); // Use Motor 1's step function
    stepCount++;
    
    // Check for the TARGET state (HIGH)
    if (digitalRead(SENSOR_PIN_1) == HIGH) {
      Serial.println("M1 Metal Found (HIGH).");
      
      // --- *** NEW BLOCK *** ---
      Serial.print("Moving ");
      Serial.print(EXTRA_STEPS_AFTER_FIND);
      Serial.println(" extra steps...");

      delay(1000);
      
      // Direction is already set, just run the steps
      for (long i = 0; i < EXTRA_STEPS_AFTER_FIND; i++) {
        takeOneStep1();
      }
      Serial.println("Extra steps complete. Stopping.");
      // --- *** END NEW BLOCK *** ---

      motor1Stopped = true; 
      break; // Exit the while loop
    }
  }

  // Check if the loop finished *without* hitting the switch
  if (!motor1Stopped) {
    Serial.println("!!! M1 FIND METAL FAILED (Max steps) !!!");
    motor1Stopped = true; // Stop all movement for this motor
  }
}


// =========================================================================
//  SETUP FUNCTION: Runs once when the Arduino boots up - (Unchanged)
// =========================================================================
void setup() {
  Serial.begin(9600);
  Serial.println("Dual Stepper Homing Code (Robust Logic)");

  // --- FIX: Set all motor pins LOW *before* setting mode ---
  digitalWrite(STEP_PIN_0, LOW);
  digitalWrite(DIR_PIN_0, LOW);
  digitalWrite(STEP_PIN_1, LOW);
  digitalWrite(DIR_PIN_1, LOW);

  // Set Motor 0 pin modes
  pinMode(STEP_PIN_0, OUTPUT);
  pinMode(DIR_PIN_0, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN_0, INPUT_PULLUP); // Homing switch

  // Set Motor 1 pin modes
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN_1, INPUT_PULLUP); // Homing switch

  // --- NEW SENSOR PIN (FOR MOTOR 1) ---
  pinMode(SENSOR_PIN_1, INPUT); 

  // Homing is no longer done at setup.
  Serial.println("Setup complete.");
  Serial.println("Send 'h' to start homing sequence.");
  Serial.println("Send 'f' to find metal with Motor 1.");
}

// =========================================================================
//  MAIN LOOP: Runs over and over - (Unchanged)
// =========================================================================
void loop() {
  
  // Check for incoming serial data
  if (Serial.available() > 0) {
    char command = Serial.read(); // Read the incoming byte

    // Check if the command is 'h'
    if (command == 'h' || command == 'H') {
      Serial.println("=== Homing Command Received ===");
      motor0Stopped = false;
      motor1Stopped = false;
      homeMotor0();
      homeMotor1();
      Serial.println("=== Homing Sequence Complete ===");
      Serial.println("Send 'h' to home again.");
    }
    
    // Check if the command is 'f'
    else if (command == 'f' || command == 'F') {
      Serial.println("=== Find Metal Command Received ===");
      findMetalMotor1(); // Calls the function for Motor 1
      Serial.println("=== Find Metal Sequence Complete ===");
    }
  }
  
}