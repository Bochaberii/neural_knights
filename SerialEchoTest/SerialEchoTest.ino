/*
 * Simple Serial Echo Sketch
 * This will help verify basic serial communication with the Arduino.
 */

void setup() {
  // Initialize serial communication
  Serial.begin(57600);  // Match the baud rate in the ROSArduinoBridge sketch
  
  // Wait a moment for the serial connection to establish
  delay(1000);
  
  // Send a startup message
  Serial.println("Arduino Serial Echo Test - Ready");
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming byte
    char inByte = Serial.read();
    
    // Echo the received character
    Serial.print("Received: ");
    Serial.println(inByte);
    
    // Also send a simple "OK" response similar to ROSArduinoBridge
    Serial.println("OK");
  }
}