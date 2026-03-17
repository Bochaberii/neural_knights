// Motor Movement with Live Encoder Feedback
// IBT-2 Motor Control + Continuous Encoder/RPM Display

//// --- Motor A (IBT-2 #1) ---
const int RPWM_A = 5;   // Left motor
const int LPWM_A = 6;
const int REN_A  = 22;
const int LEN_A  = 23;
const int ENC_A1 = 2;   // Left encoder
const int ENC_A2 = 3;

//// --- Motor B (IBT-2 #2) ---
const int RPWM_B = 9;   // Right motor
const int LPWM_B = 10;
const int REN_B  = 24;
const int LEN_B  = 25;
const int ENC_B1 = 18;  // Right encoder
const int ENC_B2 = 19;

// Movement parameters
int motorSpeed = 120;      // Default motor speed (0-255)
unsigned long moveDuration = 3000; // Move for 3 seconds
unsigned long moveStartTime = 0;
bool isMoving = false;
String currentMovement = "";

// Encoder counters
volatile long pulsesA = 0;
volatile long pulsesB = 0;
long prevPulsesA = 0;
long prevPulsesB = 0;

// Timing for continuous display
unsigned long lastDisplayTime = 0;
const unsigned long DISPLAY_INTERVAL = 500; // Display every 500ms

// Motor specs
const int PPR = 20;
const int GEAR_RATIO = 30;
const int CPR = PPR * GEAR_RATIO;

void setup() {
  Serial.begin(57600);

  // Motor pins setup
  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  pinMode(REN_A, OUTPUT);
  pinMode(LEN_A, OUTPUT);
  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);
  pinMode(REN_B, OUTPUT);
  pinMode(LEN_B, OUTPUT);

  // Enable motor drivers
  digitalWrite(REN_A, HIGH);
  digitalWrite(LEN_A, HIGH);
  digitalWrite(REN_B, HIGH);
  digitalWrite(LEN_B, HIGH);

  // Encoder pins
  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);

  // Attach encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, RISING);

  stopMotors();

  Serial.println("=== MOTOR MOVEMENT WITH LIVE ENCODER FEEDBACK ===");
  Serial.println("Commands:");
  Serial.println("  F - Move FORWARD for 3 seconds");
  Serial.println("  B - Move BACKWARD for 3 seconds");
  Serial.println("  L - Turn LEFT for 3 seconds");
  Serial.println("  R - Turn RIGHT for 3 seconds");
  Serial.println("  S - STOP immediately");
  Serial.println("  SPEED XXX - Set motor speed (0-255)");
  Serial.println("  RESET - Reset encoder counts");
  Serial.println("  AUTO - Run automatic test sequence");
  Serial.println("===============================================");
  Serial.print("Current speed: ");
  Serial.println(motorSpeed);
  Serial.println();
  
  lastDisplayTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Handle automatic movement timeout
  if (isMoving && (currentTime - moveStartTime >= moveDuration)) {
    stopMotors();
    Serial.println(">>> MOVEMENT COMPLETED <<<");
  }
  
  // Continuous encoder display
  if (currentTime - lastDisplayTime >= DISPLAY_INTERVAL) {
    displayEncoderData();
    lastDisplayTime = currentTime;
  }
  
  // Handle serial commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "F" || cmd == "FORWARD") {
      moveForward();
    }
    else if (cmd == "B" || cmd == "BACKWARD") {
      moveBackward();
    }
    else if (cmd == "L" || cmd == "LEFT") {
      turnLeft();
    }
    else if (cmd == "R" || cmd == "RIGHT") {
      turnRight();
    }
    else if (cmd == "S" || cmd == "STOP") {
      stopMotors();
    }
    else if (cmd.startsWith("SPEED ")) {
      int newSpeed = cmd.substring(6).toInt();
      if (newSpeed >= 0 && newSpeed <= 255) {
        motorSpeed = newSpeed;
        Serial.print(">>> Speed set to: ");
        Serial.println(motorSpeed);
      } else {
        Serial.println("Speed must be 0-255");
      }
    }
    else if (cmd == "RESET") {
      resetEncoders();
    }
    else if (cmd == "AUTO") {
      runAutoTest();
    }
    else if (cmd == "HELP") {
      Serial.println("F/B/L/R/S - Move Forward/Back/Left/Right/Stop");
      Serial.println("SPEED XXX - Set speed, AUTO - Auto test, RESET - Reset encoders");
    }
  }
}

void moveForward() {
  setMotor(RPWM_A, LPWM_A, motorSpeed);  // Left forward
  setMotor(RPWM_B, LPWM_B, motorSpeed);  // Right forward
  startMovement("FORWARD");
}

void moveBackward() {
  setMotor(RPWM_A, LPWM_A, -motorSpeed); // Left backward
  setMotor(RPWM_B, LPWM_B, -motorSpeed); // Right backward
  startMovement("BACKWARD");
}

void turnLeft() {
  setMotor(RPWM_A, LPWM_A, -motorSpeed); // Left backward
  setMotor(RPWM_B, LPWM_B, motorSpeed);  // Right forward
  startMovement("LEFT TURN");
}

void turnRight() {
  setMotor(RPWM_A, LPWM_A, motorSpeed);  // Left forward
  setMotor(RPWM_B, LPWM_B, -motorSpeed); // Right backward
  startMovement("RIGHT TURN");
}

void stopMotors() {
  setMotor(RPWM_A, LPWM_A, 0);
  setMotor(RPWM_B, LPWM_B, 0);
  isMoving = false;
  currentMovement = "STOPPED";
  Serial.println(">>> MOTORS STOPPED <<<");
}

void startMovement(String movement) {
  currentMovement = movement;
  isMoving = true;
  moveStartTime = millis();
  Serial.print(">>> MOVING ");
  Serial.print(movement);
  Serial.print(" at speed ");
  Serial.print(motorSpeed);
  Serial.println(" <<<");
}

void setMotor(int RPWM, int LPWM, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    analogWrite(RPWM, speed);
    analogWrite(LPWM, 0);
  } else if (speed < 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -speed);
  } else {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

void displayEncoderData() {
  noInterrupts();
  long currentPulsesA = pulsesA;
  long currentPulsesB = pulsesB;
  interrupts();
  
  // Calculate RPM (pulses in last 0.5 seconds * 120 to get per minute)
  long deltaPulsesA = currentPulsesA - prevPulsesA;
  long deltaPulsesB = currentPulsesB - prevPulsesB;
  float rpmA = (deltaPulsesA * 120.0) / CPR; // *120 because we measure every 0.5s
  float rpmB = (deltaPulsesB * 120.0) / CPR;
  
  Serial.print("[");
  Serial.print(currentMovement);
  Serial.print("] Counts: L=");
  Serial.print(currentPulsesA);
  Serial.print(" R=");
  Serial.print(currentPulsesB);
  Serial.print(" | RPM: L=");
  Serial.print(rpmA, 1);
  Serial.print(" R=");
  Serial.println(rpmB, 1);
  
  prevPulsesA = currentPulsesA;
  prevPulsesB = currentPulsesB;
}

void resetEncoders() {
  noInterrupts();
  pulsesA = 0;
  pulsesB = 0;
  interrupts();
  prevPulsesA = 0;
  prevPulsesB = 0;
  Serial.println(">>> ENCODERS RESET <<<");
}

void runAutoTest() {
  Serial.println(">>> STARTING AUTO TEST SEQUENCE <<<");
  Serial.println("Will test: Forward -> Right -> Backward -> Left -> Stop");
  
  // Forward
  Serial.println("Testing FORWARD...");
  moveForward();
  delay(3000);
  
  // Right turn
  Serial.println("Testing RIGHT TURN...");
  turnRight();
  delay(2000);
  
  // Backward
  Serial.println("Testing BACKWARD...");
  moveBackward();
  delay(3000);
  
  // Left turn
  Serial.println("Testing LEFT TURN...");
  turnLeft();
  delay(2000);
  
  // Stop
  stopMotors();
  Serial.println(">>> AUTO TEST COMPLETED <<<");
}

// Encoder interrupt functions
void encoderA_ISR() {
  pulsesA++;
}

void encoderB_ISR() {
  pulsesB++;
}