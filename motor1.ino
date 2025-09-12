// Dual IBT-2 Motor Driver Robot Control with On-Demand Feedback (Arduino Mega)

//// --- Motor A (IBT-2 #1) ---
const int RPWM_A = 5;
const int LPWM_A = 6;
const int REN_A  = 22;
const int LEN_A  = 23;
const int ENC_A1 = 2;   // Interrupt
const int ENC_A2 = 3;   // Interrupt
const int ISRA   = A0;
const int ISLA   = A1;

//// --- Motor B (IBT-2 #2) ---
const int RPWM_B = 9;
const int LPWM_B = 10;
const int REN_B  = 24;
const int LEN_B  = 25;
const int ENC_B1 = 18;  // Interrupt
const int ENC_B2 = 19;  // Interrupt
const int ISRB   = A2;
const int ISLB   = A3;

// Default robot speed
int defaultSpeed = 180;

// Encoder counters
volatile long pulsesA = 0;
volatile long pulsesB = 0;

// Motor specs (adjust to your encoders)
const int PPR = 20;       // pulses per revolution of encoder
const int GEAR_RATIO = 30; // gearbox ratio
const int CPR = PPR * GEAR_RATIO; // counts per shaft revolution

// --- Function Prototypes ---
void setMotor(int RPWM, int LPWM, int speed);
void stopAll();
void encoderA_ISR();
void encoderB_ISR();
void reportFeedback();

void setup() {
  Serial.begin(9600);

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

  Serial.println("IBT-2 Robot Control with On-Demand Feedback Ready.");
  Serial.println("Commands: A <speed>, B <speed>, F, BK, L, R, S, V <speed>, FB");
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

    if (cmd.equalsIgnoreCase("A")) {
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
    else if (cmd.equalsIgnoreCase("FB")) {
      reportFeedback();
    }
    else {
      Serial.print("Unknown command: ");
      Serial.println(line);
    }
  }
}

// --- Feedback Function ---
void reportFeedback() {
  noInterrupts();
  long countA = pulsesA; pulsesA = 0;
  long countB = pulsesB; pulsesB = 0;
  interrupts();

  float rpmA = (countA * 60.0) / CPR;
  float rpmB = (countB * 60.0) / CPR;

  int rawA = analogRead(ISRA) + analogRead(ISLA);
  int rawB = analogRead(ISRB) + analogRead(ISLB);

  float currentA = (rawA / 1023.0) * 5.0 * 6.0; // Example scaling
  float currentB = (rawB / 1023.0) * 5.0 * 6.0;

  Serial.print("[FEEDBACK] RPM_A=");
  Serial.print(rpmA);
  Serial.print(" RPM_B=");
  Serial.print(rpmB);
  Serial.print(" CurrentA=");
  Serial.print(currentA, 2);
  Serial.print("A CurrentB=");
  Serial.print(currentB, 2);
  Serial.println("A");
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
void encoderA_ISR() { pulsesA++; }
void encoderB_ISR() { pulsesB++; }

