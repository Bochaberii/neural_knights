/*
 * Simple Servo Control Sketch
 * For testing basic servo functionality
 */

#include <Servo.h>

// Define constants
#define SERVO_PIN 11        // Servo control pin
#define SERVO_REST_POS 90   // Default resting position

// Create servo object
Servo myServo;

// Variables to store incoming serial data
String inputString = "";
boolean stringComplete = false;

void setup() {
  // Initialize serial communication
  Serial.begin(57600);
  
  // Wait for serial port to connect
  delay(2000);
  
  // Print startup message
  Serial.println("Simple Servo Control Ready");
  Serial.println("Commands: '0' to '180' to move servo to that angle");
  Serial.println("          'r' to return to rest position");
  
  // Attach servo but DON'T move it yet
  myServo.attach(SERVO_PIN);
  
  // Important: Don't automatically move the servo on startup
  // Just leave it wherever it currently is
  
  Serial.println("Servo attached but not moved. Send a command to move.");
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
  
  // Check for 'r' to return to rest position
  if (inputString == "r") {
    myServo.write(SERVO_REST_POS);
    Serial.print("Moved to rest position (");
    Serial.print(SERVO_REST_POS);
    Serial.println(" degrees)");
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
    Serial.println("Invalid angle. Use 0-180 or 'r' for rest position.");
  }
}