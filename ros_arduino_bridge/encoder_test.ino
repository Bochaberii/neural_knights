/*
 * Simple Encoder Pin Test
 * This code reads the raw digital values from encoder pins
 * Upload this to test if encoders are wired correctly
 */

// Encoder pins
#define LEFT_ENC_PIN_A 2
#define LEFT_ENC_PIN_B 3
#define RIGHT_ENC_PIN_A 18
#define RIGHT_ENC_PIN_B 19

void setup() {
  Serial.begin(57600);
  
  // Set encoder pins as inputs with pullups
  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);
  
  Serial.println("=== Encoder Pin Test ===");
  Serial.println("Left encoder: pins 2,3");
  Serial.println("Right encoder: pins 18,19");
  Serial.println("Spin wheels and watch for pin changes");
  Serial.println();
}

void loop() {
  // Read all encoder pins
  int leftA = digitalRead(LEFT_ENC_PIN_A);
  int leftB = digitalRead(LEFT_ENC_PIN_B);
  int rightA = digitalRead(RIGHT_ENC_PIN_A);
  int rightB = digitalRead(RIGHT_ENC_PIN_B);
  
  // Print pin states
  Serial.print("Left(2,3): ");
  Serial.print(leftA);
  Serial.print(",");
  Serial.print(leftB);
  Serial.print("  Right(18,19): ");
  Serial.print(rightA);
  Serial.print(",");
  Serial.println(rightB);
  
  delay(250); // Update 4 times per second
}