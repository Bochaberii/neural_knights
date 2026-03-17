/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

// #ifdef L298_MOTOR_DRIVER
//   #define RIGHT_MOTOR_BACKWARD 5
//   #define LEFT_MOTOR_BACKWARD  6
//   #define RIGHT_MOTOR_FORWARD  9
//   #define LEFT_MOTOR_FORWARD   10
//   #define RIGHT_MOTOR_ENABLE 12
//   #define LEFT_MOTOR_ENABLE 13
// #endif
#ifdef IBT2_MOTOR_DRIVER
  // Motor A (Left)
  #define LEFT_MOTOR_RPWM 5
  #define LEFT_MOTOR_LPWM 6
  #define LEFT_MOTOR_REN  22
  #define LEFT_MOTOR_LEN  23
  // Motor B (Right)
  #define RIGHT_MOTOR_RPWM 9
  #define RIGHT_MOTOR_LPWM 10
  #define RIGHT_MOTOR_REN  24
  #define RIGHT_MOTOR_LEN  25
#endif

void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);
