#ifdef USE_SERVOS

// Global variables
SweepServo servos[N_SERVOS];
int servoPins[N_SERVOS] = {CUBE_SERVO_PIN}; 
int servoInitPosition[N_SERVOS] = {SERVO_REST_POS};
int stepDelay[N_SERVOS] = {0};

// Function to initialize the servo
void initServo() {
	// Initialize the servo
	servos[0].initServo(CUBE_SERVO_PIN, 0, SERVO_REST_POS);
  
	Serial.println("Servo initialized on pin " + String(CUBE_SERVO_PIN));
}

// Simple function to move the servo to a position
void moveServo(int position) {
	// Move to specified position
	servos[0].setTargetPosition(position);
	Serial.println("Servo moved to position: " + String(position));
}

#endif


