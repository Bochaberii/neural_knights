#ifdef USE_SERVOS

// Global variables
SweepServo servos[N_SERVOS];
int servoPins[N_SERVOS] = {CUBE_SERVO_PIN}; 
int servoInitPosition[N_SERVOS] = {SERVO_REST_POS};
int stepDelay[N_SERVOS] = {0};

// Drop system variables
bool objectDetected = false;  // Whether an object is detected by the IR sensor
bool dropping = false;        // Whether we're currently in a drop sequence
unsigned long lastDropTime = 0;     // Last time a drop was performed
unsigned long lastCheckTime = 0;    // Last time IR sensor was checked
const unsigned long debounceDelay = 500; // 500ms debounce

// Function to initialize the IR sensor and servo
void initIRDropper() {
  // Initialize the IR sensor pin
  pinMode(IR_SENSOR_PIN, INPUT);
  
  // Initialize the servo
  servos[0].initServo(CUBE_SERVO_PIN, 0, SERVO_REST_POS);
  
  // Reset state variables
  objectDetected = false;
  dropping = false;
  
  Serial.println("IR Dropper system initialized on pin " + String(IR_SENSOR_PIN));
}

// Function to check the IR sensor with debounce
void checkIRSensor() {
  // Only check every debounceDelay ms
  if (millis() - lastCheckTime < debounceDelay) {
    return;
  }
  
  lastCheckTime = millis();
  
  // Read the digital state of the IR sensor pin
  int irStatus = digitalRead(IR_SENSOR_PIN);
  
  // If object detected in loading area and we're not currently dropping
  if (irStatus == HIGH && !objectDetected && !dropping) {
    objectDetected = true;
    // Notify Pi that the load is present and ready
    Serial.println("LOAD:DETECTED");
  }
  // If no object detected and we previously detected one (e.g., after unloading)
  else if (irStatus == LOW && objectDetected && !dropping) {
    objectDetected = false;
    Serial.println("LOAD:REMOVED");
  }
}

// Function to perform the unloading/dropping sequence
// Perform a single drop attempt and return whether the IR confirmed unload
bool performSingleDropAttempt() {
  dropping = true;
  Serial.println("DROP:ATTEMPTING");

  // Move to drop position and wait briefly
  servos[0].setTargetPosition(SERVO_DROP_POS);
  delay(800);

  // Move back to rest
  servos[0].setTargetPosition(SERVO_REST_POS);
  delay(500);

  // Allow a short window for the IR sensor to update
  delay(200);

  // If IR no longer detects the object, the unload succeeded
  bool success = !objectDetected;
  Serial.println(success ? "DROP:SUCCESS" : "DROP:FAILED");

  dropping = false;
  lastDropTime = millis();
  return success;
}

// Perform up to two attempts to unload; return true if unload confirmed
bool performTwoAttemptUnload() {
  // First attempt
  if (performSingleDropAttempt()) return true;
  // Second attempt
  if (performSingleDropAttempt()) return true;
  return false;
}

// Backwards-compatible wrapper called by main command processor
void performDrop() {
  bool success = performTwoAttemptUnload();
  if (success) {
    Serial.println("DROP:COMPLETE");
  } else {
    Serial.println("DROP:FAILED");
  }
}

// Color support removed; no-op placeholder removed

// Function to get the cube status (simplified)
String getCubeStatus() {
  String status = "STATUS:";
  status += objectDetected ? "DETECTED" : "NONE";
  return status;
}

#endif
