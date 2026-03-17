// Dual IBT-2 Motor Driver Robot Control with RPM Feedback (Arduino Mega)
// Motor A: RPWM=5, LPWM=6, REN=22, LEN=23, Encoders on pins 2,3
// Motor B: RPWM=9, LPWM=10, REN=24, LEN=25, Encoders on pins 18,19

//// --- Motor A (IBT-2 #1) ---
const int RPWM_A = 5;
const int LPWM_A = 6;
const int REN_A  = 22;
const int LEN_A  = 23;
const int ENC_A1 = 2;   // Interrupt
const int ENC_A2 = 3;   // Interrupt

//// --- Motor B (IBT-2 #2) ---
const int RPWM_B = 9;
const int LPWM_B = 10;
const int REN_B  = 24;
const int LEN_B  = 25;
const int ENC_B1 = 18;  // Interrupt
const int ENC_B2 = 19;  // Interrupt

// Default robot speed
int defaultSpeed = 180;

// Encoder counters
volatile long pulsesA = 0;
volatile long pulsesB = 0;

// Motor specs (change to match your encoder)
const int PPR = 20;        // pulses per revolution of encoder (example: 20 CPR)
const int GEAR_RATIO = 30; // gearbox ratio (example: 30:1)
const int CPR = PPR * GEAR_RATIO; // counts per shaft revolution

// --- Function Prototypes ---
void setMotor(int RPWM, int LPWM, int speed);
void stopAll();
void encoderA_ISR();
void encoderB_ISR();

void setup() {
  Serial.begin(57600);  // Changed to 57600 for consistency

  // Motor A pins
  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  pinMode(REN_A, OUTPUT);
  pinMode(LEN_A, OUTPUT);

  // Motor B pins
  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);
  pinMode(REN_B, OUTPUT);
  pinMode(LEN_B, OUTPUT);

  // Enable drivers
  digitalWrite(REN_A, HIGH);
  digitalWrite(LEN_A, HIGH);
  digitalWrite(REN_B, HIGH);
  digitalWrite(LEN_B, HIGH);

  // Encoder pins
  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, RISING);

  stopAll();

  Serial.println("=== IBT-2 Robot Control with Encoder Test ===");
  Serial.println("Commands:");
  Serial.println("  E or ENC - Show encoder counts");
  Serial.println("  FB - Show RPM (resets counters)"); 
  Serial.println("  RESET - Reset encoder counts to 0");
  Serial.println("  F/BK/L/R/S - Move forward/back/left/right/stop");
  Serial.println("  A 150 - Set motor A speed");
  Serial.println("  B -100 - Set motor B speed");
  Serial.println();
}

void loop() {
  // --- Handle Serial Commands ---
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    int sp = line.indexOf(' ');
    String cmd = (sp == -1) ? line : line.substring(0, sp);
    String rest = (sp == -1) ? "" : line.substring(sp + 1);
    cmd.trim();
    rest.trim();

    if (cmd.equalsIgnoreCase("E") || cmd.equalsIgnoreCase("ENC")) {
      noInterrupts();
      long countA = pulsesA;
      long countB = pulsesB;
      interrupts();

      Serial.print("[ENCODERS] Left=");
      Serial.print(countA);
      Serial.print("  Right=");
      Serial.println(countB);
    }
    else if (cmd.equalsIgnoreCase("A")) {
      int val = rest.toInt();
      setMotor(RPWM_A, LPWM_A, val);
      Serial.print("Motor A set to ");
      Serial.println(val);
    }
    else if (cmd.equalsIgnoreCase("B")) {
      int val = rest.toInt();
      setMotor(RPWM_B, LPWM_B, val);
      Serial.print("Motor B set to ");
      Serial.println(val);
    }
    else if (cmd.equalsIgnoreCase("F") || cmd.equalsIgnoreCase("FORWARD")) {
      setMotor(RPWM_A, LPWM_A, defaultSpeed);
      setMotor(RPWM_B, LPWM_B, defaultSpeed);
      Serial.println("Robot: FORWARD");
    }
    else if (cmd.equalsIgnoreCase("BK") || cmd.equalsIgnoreCase("BACK") || cmd.equalsIgnoreCase("BACKWARD")) {
      setMotor(RPWM_A, LPWM_A, -defaultSpeed);
      setMotor(RPWM_B, LPWM_B, -defaultSpeed);
      Serial.println("Robot: BACKWARD");
    }
    else if (cmd.equalsIgnoreCase("L") || cmd.equalsIgnoreCase("LEFT")) {
      setMotor(RPWM_A, LPWM_A, -defaultSpeed);
      setMotor(RPWM_B, LPWM_B, defaultSpeed);
      Serial.println("Robot: LEFT");
    }
    else if (cmd.equalsIgnoreCase("R") || cmd.equalsIgnoreCase("RIGHT")) {
      setMotor(RPWM_A, LPWM_A, defaultSpeed);
      setMotor(RPWM_B, LPWM_B, -defaultSpeed);
      Serial.println("Robot: RIGHT");
    }
    else if (cmd.equalsIgnoreCase("S") || cmd.equalsIgnoreCase("STOP")) {
      stopAll();
      Serial.println("Robot: STOP");
    }
    else if (cmd.equalsIgnoreCase("V")) {
      if (rest.length() > 0) {
        defaultSpeed = constrain(rest.toInt(), 0, 255);
        Serial.print("Default speed set to ");
        Serial.println(defaultSpeed);
      } else {
        Serial.print("Default speed is ");
        Serial.println(defaultSpeed);
      }
    }
    else if (cmd.equalsIgnoreCase("FB") || cmd.equalsIgnoreCase("FEEDBACK")) {
      noInterrupts();
      long countA = pulsesA; pulsesA = 0;
      long countB = pulsesB; pulsesB = 0;
      interrupts();

      float rpmA = (countA * 60.0) / CPR;  // RPM Motor A
      float rpmB = (countB * 60.0) / CPR;  // RPM Motor B

      Serial.print("[FEEDBACK] RPM_A=");
      Serial.print(rpmA);
      Serial.print("  RPM_B=");
      Serial.println(rpmB);
    }
    else if (cmd.equalsIgnoreCase("RESET")) {
      noInterrupts();
      pulsesA = 0;
      pulsesB = 0;
      interrupts();
      Serial.println("Encoder counts reset to 0");
    }
    else {
      Serial.println("Unknown command. Try: E, FB, F, BK, L, R, S, RESET");
    }
  }
}

// --- Motor Control Functions ---
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

void stopAll() {
  setMotor(RPWM_A, LPWM_A, 0);
  setMotor(RPWM_B, LPWM_B, 0);
}

// --- Encoder ISRs ---
void encoderA_ISR() {
  pulsesA++;
}
void encoderB_ISR() {
  pulsesB++;
}