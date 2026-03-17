// RIGHT ENCODER DIAGNOSTIC TEST
// This will help us find why the right encoder stays at 0

// Pin definitions
const int LEFT_A = 2;    // Working encoder
const int LEFT_B = 3;
const int RIGHT_A = 18;  // Problem encoder
const int RIGHT_B = 19;

// Test counters
volatile long leftCount = 0;
volatile long rightCount = 0;

void setup() {
  Serial.begin(57600);
  
  // Setup pins
  pinMode(LEFT_A, INPUT_PULLUP);
  pinMode(LEFT_B, INPUT_PULLUP);
  pinMode(RIGHT_A, INPUT_PULLUP);
  pinMode(RIGHT_B, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_A), leftISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_A), rightISR, RISING);
  
  Serial.println("=== RIGHT ENCODER DIAGNOSTIC ===");
  Serial.println("Testing why right encoder stays at 0");
  Serial.println("Commands:");
  Serial.println("  T - Test raw pin readings");
  Serial.println("  C - Show interrupt counts");
  Serial.println("  R - Reset counts");
  Serial.println("=====================================");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();
    
    if (cmd == "T") {
      testRawPins();
    }
    else if (cmd == "C") {
      showCounts();
    }
    else if (cmd == "R") {
      noInterrupts();
      leftCount = 0;
      rightCount = 0;
      interrupts();
      Serial.println("Counts reset to 0");
    }
  }
}

void testRawPins() {
  Serial.println("=== RAW PIN TEST ===");
  Serial.println("Spin wheels and watch for pin changes:");
  Serial.println("Left should change, right might not...");
  
  for (int i = 0; i < 20; i++) {  // Test for 10 seconds
    int leftA = digitalRead(LEFT_A);
    int leftB = digitalRead(LEFT_B);
    int rightA = digitalRead(RIGHT_A);
    int rightB = digitalRead(RIGHT_B);
    
    Serial.print("Left(2,3): ");
    Serial.print(leftA);
    Serial.print(",");
    Serial.print(leftB);
    Serial.print("  Right(18,19): ");
    Serial.print(rightA);
    Serial.print(",");
    Serial.print(rightB);
    
    // Check if pins are stuck
    if (rightA == 0 && rightB == 0) {
      Serial.print("  << RIGHT PINS BOTH LOW - NO POWER?");
    } else if (rightA == 1 && rightB == 1) {
      Serial.print("  << RIGHT PINS BOTH HIGH - CHECK CONNECTIONS");
    }
    
    Serial.println();
    delay(500);
  }
  Serial.println("=== RAW PIN TEST COMPLETE ===");
}

void showCounts() {
  noInterrupts();
  long left = leftCount;
  long right = rightCount;
  interrupts();
  
  Serial.print("Interrupt Counts: Left=");
  Serial.print(left);
  Serial.print("  Right=");
  Serial.println(right);
  
  if (right == 0) {
    Serial.println(">>> RIGHT ENCODER NOT TRIGGERING INTERRUPTS <<<");
    Serial.println("Possible causes:");
    Serial.println("1. Pin 18 not connected to encoder signal");
    Serial.println("2. Right encoder has no power (5V/GND)");
    Serial.println("3. Right encoder is faulty");
    Serial.println("4. Wrong pin assignment");
  }
}

void leftISR() {
  leftCount++;
}

void rightISR() {
  rightCount++;
}