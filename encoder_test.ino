// Simple Encoder Test Code
// Upload this to your Arduino to test encoders

// Define encoder pins (adjust to match your wiring)
#define LEFT_ENC_PIN_A  2
#define LEFT_ENC_PIN_B  3
#define RIGHT_ENC_PIN_A 18
#define RIGHT_ENC_PIN_B 19

// Encoder counters
volatile long leftCount = 0;
volatile long rightCount = 0;

// Interrupt functions for left encoder
void leftEncoderA() {
  if (digitalRead(LEFT_ENC_PIN_A) == digitalRead(LEFT_ENC_PIN_B)) {
    leftCount++;
  } else {
    leftCount--;
  }
}

void leftEncoderB() {
  if (digitalRead(LEFT_ENC_PIN_A) != digitalRead(LEFT_ENC_PIN_B)) {
    leftCount++;
  } else {
    leftCount--;
  }
}

// Interrupt functions for right encoder
void rightEncoderA() {
  if (digitalRead(RIGHT_ENC_PIN_A) == digitalRead(RIGHT_ENC_PIN_B)) {
    rightCount++;
  } else {
    rightCount--;
  }
}

void rightEncoderB() {
  if (digitalRead(RIGHT_ENC_PIN_A) != digitalRead(RIGHT_ENC_PIN_B)) {
    rightCount++;
  } else {
    rightCount--;
  }
}

void setup() {
  Serial.begin(57600);
  
  // Set encoder pins as inputs
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), leftEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_B), leftEncoderB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), rightEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_B), rightEncoderB, CHANGE);
  
  Serial.println("Encoder Test Started");
  Serial.println("Spin the wheels and watch the counts change");
  Serial.println("Format: Left_Count Right_Count");
}

void loop() {
  // Print encoder values every 500ms
  Serial.print("Encoders: ");
  Serial.print(leftCount);
  Serial.print(" ");
  Serial.println(rightCount);
  
  delay(500);
}

// Test procedure:
// 1. Upload this code to Arduino
// 2. Open Serial Monitor (57600 baud)
// 3. Manually spin each wheel
// 4. Watch if numbers change
// 5. If numbers don't change, check wiring/pins