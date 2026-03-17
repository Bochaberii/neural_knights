/*
 * Servo Control for Loading/Unloading System
 * This sketch keeps the servo at a holding angle until commanded to unload
 */

#include <Servo.h>

// Define constants
#define SERVO_PIN 11        // Servo control pin
#define HOLDING_ANGLE 90    // Default holding position (adjust as needed)
#define UNLOAD_ANGLE 0      // Angle to drop/unload (adjust as needed)
#define DELAY_MS 800        // Time to wait in unload position before returning

// Create servo object
Servo myServo;

// Variables to store incoming serial data
String inputString = "";
boolean stringComplete = false;
boolean unloadSequenceActive = false;

void setup() {
  // Initialize serial communication
  Serial.begin(57600);
  
  // Wait for serial port to connect
  delay(2000);
  
  // Print startup message
  Serial.println("Servo Loading/Unloading System Ready");
  Serial.println("Commands: 'h' to move to holding position");
  Serial.println("          'u' to perform unload sequence");
  Serial.println("          '0-180' to move to specific angle");
  
  // Attach servo
  myServo.attach(SERVO_PIN);
  
  // Move to holding position initially
  myServo.write(HOLDING_ANGLE);
  Serial.println("Servo moved to holding position");
}

void loop() {
  // Process any incoming serial data
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    // Add character to input string if it's not a newline
    if (inChar != '\n' && inChar != '\r') {
      inputString += inChar;
    }
    
    // If we get a newline or carriage return, process the command
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
    }
  }
  
  // If we have a complete command, process it
  if (stringComplete) {
    processCommand();
    
    // Clear the string for the next command
    inputString = "";
    stringComplete = false;
  }
}

void processCommand() {
  // Remove any whitespace
  inputString.trim();
  
  // If the command is empty, do nothing
  if (inputString.length() == 0) {
    return;
  }
  
  // Check for 'h' to move to holding position
  if (inputString == "h") {
    myServo.write(HOLDING_ANGLE);
    Serial.print("Moved to holding position (");
    Serial.print(HOLDING_ANGLE);
    Serial.println(" degrees)");
    return;
  }
  
  // Check for 'u' to perform unload sequence
  if (inputString == "u") {
    performUnloadSequence();
    return;
  }
  
  // Otherwise, try to parse as an angle
  int angle = inputString.toInt();
  
  // Validate angle range
  if (angle >= 0 && angle <= 180) {
    myServo.write(angle);
    Serial.print("Moved to ");
    Serial.print(angle);
    Serial.println(" degrees");
  } else {
    Serial.println("Invalid command. Use 'h' for holding, 'u' for unloading, or 0-180 for specific angle.");
  }
}

void performUnloadSequence() {
  Serial.println("Starting unload sequence");
  
  // Move to unload position
  myServo.write(UNLOAD_ANGLE);
  Serial.print("Moved to unload position (");
  Serial.print(UNLOAD_ANGLE);
  Serial.println(" degrees)");
  
  // Wait for a moment
  delay(DELAY_MS);
  
  // Return to holding position
  myServo.write(HOLDING_ANGLE);
  Serial.print("Returned to holding position (");
  Serial.print(HOLDING_ANGLE);
  Serial.println(" degrees)");
  
  Serial.println("Unload sequence completed");
}