/***************************************************************
   Motor driver function definitions - by James Nugen
   *************************************************************/

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "commands.h"  // For LEFT and RIGHT definitions

#ifdef IBT2_MOTOR_DRIVER
  // Motor A (Left)
  #define LEFT_MOTOR_RPWM 5    // PWM pin for left motor forward
  #define LEFT_MOTOR_LPWM 6    // PWM pin for left motor backward
  #define LEFT_MOTOR_REN  22   // Enable pin for left motor forward
  #define LEFT_MOTOR_LEN  23   // Enable pin for left motor backward
  
  // Motor B (Right)
  #define RIGHT_MOTOR_RPWM 9   // PWM pin for right motor forward
  #define RIGHT_MOTOR_LPWM 10  // PWM pin for right motor backward
  #define RIGHT_MOTOR_REN  24  // Enable pin for right motor forward
  #define RIGHT_MOTOR_LEN  25  // Enable pin for right motor backward
#endif

// Function prototypes
void initMotorController();
void setMotorSpeed(int i, int spd);
void setMotorSpeeds(int leftSpeed, int rightSpeed);

#endif