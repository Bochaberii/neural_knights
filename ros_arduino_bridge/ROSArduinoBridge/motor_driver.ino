/***************************************************************
   Motor driver definitions

   Add a "#elif defined" block to this file to include support
   for a particular motor driver.  Then add the appropriate
   #define near the top of the main ROSArduinoBridge.ino file.

   *************************************************************/

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
void initMotorController()
{
  pinMode(LEFT_MOTOR_RPWM, OUTPUT);
  pinMode(LEFT_MOTOR_LPWM, OUTPUT);
  pinMode(LEFT_MOTOR_REN, OUTPUT);
  pinMode(LEFT_MOTOR_LEN, OUTPUT);
  pinMode(RIGHT_MOTOR_RPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_LPWM, OUTPUT);
  pinMode(RIGHT_MOTOR_REN, OUTPUT);
  pinMode(RIGHT_MOTOR_LEN, OUTPUT);
  digitalWrite(LEFT_MOTOR_REN, HIGH);
  digitalWrite(LEFT_MOTOR_LEN, HIGH);
  digitalWrite(RIGHT_MOTOR_REN, HIGH);
  digitalWrite(RIGHT_MOTOR_LEN, HIGH);
}

void setMotorSpeed(int i, int spd)
{
  unsigned char reverse = 0;
  if (spd < 0)
  {
    spd = -spd;
    reverse = 1;
  }
  if (spd > 255)
    spd = 255;
  if (i == LEFT)
  {
    if (reverse == 0)
    {
      analogWrite(LEFT_MOTOR_RPWM, spd);
      analogWrite(LEFT_MOTOR_LPWM, 0);
    }
    else
    {
      analogWrite(LEFT_MOTOR_RPWM, 0);
      analogWrite(LEFT_MOTOR_LPWM, spd);
    }
  }
  else
  {
    if (reverse == 0)
    {
      analogWrite(RIGHT_MOTOR_RPWM, spd);
      analogWrite(RIGHT_MOTOR_LPWM, 0);
    }
    else
    {
      analogWrite(RIGHT_MOTOR_RPWM, 0);
      analogWrite(RIGHT_MOTOR_LPWM, spd);
    }
  }
}

void setMotorSpeeds(int leftSpeed, int rightSpeed)
{
  setMotorSpeed(LEFT, leftSpeed);
  setMotorSpeed(RIGHT, rightSpeed);
}
#else
#error A motor driver must be selected!
#endif

#endif
