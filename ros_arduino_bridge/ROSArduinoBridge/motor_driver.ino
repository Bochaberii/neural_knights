#include "motor_driver.h"

#ifdef USE_BASE

#ifdef POLOLU_VNH5019
/* Include the Pololu library */
#include "DualVNH5019MotorShield.h"

/* Create the motor driver object */
DualVNH5019MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController()
{
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd)
{
  if (i == LEFT)
    drive.setM1Speed(spd);
  else
    drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined POLOLU_MC33926
/* Include the Pololu library */
#include "DualMC33926MotorShield.h"

/* Create the motor driver object */
DualMC33926MotorShield drive;

/* Wrap the motor driver initialization */
void initMotorController()
{
  drive.init();
}

/* Wrap the drive motor set speed function */
void setMotorSpeed(int i, int spd)
{
  if (i == LEFT)
    drive.setM1Speed(spd);
  else
    drive.setM2Speed(spd);
}

// A convenience function for setting both motor speeds
void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#elif defined IBT2_MOTOR_DRIVER
void initMotorController() {
  // Left motor
  pinMode(LEFT_MOTOR_RPWM, OUTPUT);
  pinMode(LEFT_MOTOR_LPWM, OUTPUT);
  pinMode(LEFT_MOTOR_REN, OUTPUT);
  pinMode(LEFT_MOTOR_LEN, OUTPUT);

  // Right motor
  pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_LPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_REN, OUTPUT);
  pinMode(RIGHT_MOTOR_LEN, OUTPUT);

  // Enable both IBT-2 drivers
  digitalWrite(LEFT_MOTOR_REN, HIGH);
  digitalWrite(LEFT_MOTOR_LEN, HIGH);
  digitalWrite(RIGHT_MOTOR_REN, HIGH);
  digitalWrite(RIGHT_MOTOR_LEN, HIGH);
  
  // Initialize with motors stopped
  analogWrite(LEFT_MOTOR_RPWM, 0);
  analogWrite(LEFT_MOTOR_LPWM, 0);
  analogWrite(RIGHT_MOTOR_RPWM, 0);
  analogWrite(RIGHT_MOTOR_LPWM, 0);
  
  Serial.println("IBT-2 motor drivers initialized");
}

void setMotorSpeed(int motor, int spd) {
  spd = constrain(spd, -255, 255); // keep within PWM range

  if (motor == LEFT) {
    if (spd > 0) {
      analogWrite(LEFT_MOTOR_RPWM, spd);   // forward
      analogWrite(LEFT_MOTOR_LPWM, 0);
    } else if (spd < 0) {
      analogWrite(LEFT_MOTOR_RPWM, 0);
      analogWrite(LEFT_MOTOR_LPWM, -spd);  // backward
    } else {
      analogWrite(LEFT_MOTOR_RPWM, 0);
      analogWrite(LEFT_MOTOR_LPWM, 0);     // stop (coast)
    }
  } else if (motor == RIGHT) {
    if (spd > 0) {
      analogWrite(RIGHT_MOTOR_RPWM, spd);  // forward
      analogWrite(RIGHT_MOTOR_LPWM, 0);
    } else if (spd < 0) {
      analogWrite(RIGHT_MOTOR_RPWM, 0);
      analogWrite(RIGHT_MOTOR_LPWM, -spd); // backward
    } else {
      analogWrite(RIGHT_MOTOR_RPWM, 0);
      analogWrite(RIGHT_MOTOR_LPWM, 0);    // stop (coast)
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#else
#error A motor driver must be selected!
#endif

#endif