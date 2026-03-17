#define LEFT_ENC_PIN_A 2
#define LEFT_ENC_PIN_B 3
#define RIGHT_ENC_PIN_A 18
#define RIGHT_ENC_PIN_B 19

long readEncoder(int i);
void resetEncoder(int i);
void resetEncoders();

// 990 encoder counts per revolution
