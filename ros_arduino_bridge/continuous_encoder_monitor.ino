// Continuous Encoder and RPM Monitor
// Motor A: Encoders on pins 2,3 | Motor B: Encoders on pins 18,19

//// --- Encoder Pins ---
const int ENC_A1 = 2;   // Left motor encoder A
const int ENC_A2 = 3;   // Left motor encoder B  
const int ENC_B1 = 18;  // Right motor encoder A
const int ENC_B2 = 19;  // Right motor encoder B

// Encoder counters
volatile long pulsesA = 0;
volatile long pulsesB = 0;

// Previous counts for RPM calculation
long prevPulsesA = 0;
long prevPulsesB = 0;

// Timing for RPM calculation
unsigned long lastTime = 0;
const unsigned long RPM_INTERVAL = 1000; // Calculate RPM every 1000ms (1 second)

// Motor specs (adjust to match your encoder)
const int PPR = 20;        // pulses per revolution of encoder
const int GEAR_RATIO = 30; // gearbox ratio
const int CPR = PPR * GEAR_RATIO; // counts per shaft revolution

void setup() {
  Serial.begin(57600);

  // Encoder pins as inputs with pullups
  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);

  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, RISING);

  Serial.println("=== CONTINUOUS ENCODER & RPM MONITOR ===");
  Serial.println("Left Encoder: Pins 2,3  |  Right Encoder: Pins 18,19");
  Serial.println("Spin wheels to see live encoder counts and RPM");
  Serial.println("Format: [Counts] Left=XXX Right=XXX  |  [RPM] Left=X.X Right=X.X");
  Serial.println("========================================================");
  
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Calculate and display every second
  if (currentTime - lastTime >= RPM_INTERVAL) {
    
    // Get current encoder counts (disable interrupts briefly)
    noInterrupts();
    long currentPulsesA = pulsesA;
    long currentPulsesB = pulsesB;
    interrupts();
    
    // Calculate pulses in the last interval
    long deltaPulsesA = currentPulsesA - prevPulsesA;
    long deltaPulsesB = currentPulsesB - prevPulsesB;
    
    // Calculate RPM (pulses per minute / counts per revolution)
    float rpmA = (deltaPulsesA * 60.0) / CPR;
    float rpmB = (deltaPulsesB * 60.0) / CPR;
    
    // Display results
    Serial.print("[COUNTS] Left=");
    Serial.print(currentPulsesA);
    Serial.print(" Right=");
    Serial.print(currentPulsesB);
    Serial.print("  |  [RPM] Left=");
    Serial.print(rpmA, 1);
    Serial.print(" Right=");
    Serial.println(rpmB, 1);
    
    // Update for next calculation
    prevPulsesA = currentPulsesA;
    prevPulsesB = currentPulsesB;
    lastTime = currentTime;
  }
  
  // Handle simple commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd.equalsIgnoreCase("RESET") || cmd.equalsIgnoreCase("R")) {
      noInterrupts();
      pulsesA = 0;
      pulsesB = 0;
      interrupts();
      prevPulsesA = 0;
      prevPulsesB = 0;
      Serial.println(">>> ENCODER COUNTS RESET TO 0 <<<");
    }
    else if (cmd.equalsIgnoreCase("HELP") || cmd.equalsIgnoreCase("H")) {
      Serial.println("Commands:");
      Serial.println("  RESET or R - Reset encoder counts to 0");
      Serial.println("  HELP or H - Show this help");
    }
  }
  
  delay(50); // Small delay to prevent overwhelming serial output
}

// --- Encoder Interrupt Service Routines ---
void encoderA_ISR() {
  pulsesA++;
}

void encoderB_ISR() {
  pulsesB++;
}