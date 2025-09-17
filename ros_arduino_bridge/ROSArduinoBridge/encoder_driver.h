

// Encoder pin assignments for IBT-2 driver setup
#define LEFT_ENC_PIN_A 2   // Motor A encoder pin 1
#define LEFT_ENC_PIN_B 3   // Motor A encoder pin 2
#define RIGHT_ENC_PIN_A 18 // Motor B encoder pin 1
#define RIGHT_ENC_PIN_B 19 // Motor B encoder pin 2

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();
