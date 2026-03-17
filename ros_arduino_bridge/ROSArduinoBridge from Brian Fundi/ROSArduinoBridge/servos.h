#ifndef SERVOS_H
#define SERVOS_H

// -----------------------------
// Servo Control Class
// -----------------------------
#include <Servo.h>

#ifdef USE_SERVOS

class SweepServo
{
private:
  Servo servo;
  int pin;
  int currentPositionDegrees;
  int targetPositionDegrees;
  int minDegrees;
  int maxDegrees;
  int stepDelayMs;
  bool hasArrived;
  long lastSweepCommand;

public:
  SweepServo()
  {
    currentPositionDegrees = 0;
    targetPositionDegrees = 0;
    lastSweepCommand = 0;
    hasArrived = true;
  }

  void initServo(int servoPin, int stepDelay, int initialPosition = 90, int minPos = 0, int maxPos = 180)
  {
    pin = servoPin;
    stepDelayMs = stepDelay;
    currentPositionDegrees = initialPosition;
    targetPositionDegrees = initialPosition;
    minDegrees = minPos;
    maxDegrees = maxPos;
    hasArrived = true;

    servo.attach(pin);
    servo.write(currentPositionDegrees);
  }

  void setMinMax(int minPos, int maxPos)
  {
    minDegrees = minPos;
    maxDegrees = maxPos;
  }

  void setTargetPosition(int position)
  {
    targetPositionDegrees = constrain(position, minDegrees, maxDegrees);
    hasArrived = false;
  }

  void doSweep()
  {
    if (currentPositionDegrees != targetPositionDegrees)
    {
      int delta = millis() - lastSweepCommand;
      if (delta > stepDelayMs)
      {
        if (currentPositionDegrees < targetPositionDegrees)
        {
          currentPositionDegrees++;
        }
        else
        {
          currentPositionDegrees--;
        }

        servo.write(currentPositionDegrees);
        lastSweepCommand = millis();

        if (currentPositionDegrees == targetPositionDegrees)
        {
          hasArrived = true;
        }
      }
    }
  }

  Servo &getServo()
  {
    return servo;
  }

  int getTargetPositionDegrees()
  {
    return targetPositionDegrees;
  }

  int getCurrentPositionDegrees()
  {
    return currentPositionDegrees;
  }

  bool hasReachedTargetPosition()
  {
    return hasArrived;
  }

  int getPosition()
  {
    return currentPositionDegrees;
  }
};

// -----------------------------
// Servo Configuration
// -----------------------------
#define N_SERVOS 1

// Configuration constants
#define CUBE_SERVO_PIN 11 // Servo pin
#define SERVO_REST_POS 90 // Servo resting position

// Global variables
extern SweepServo servos[N_SERVOS];
extern int servoPins[N_SERVOS];
extern int servoInitPosition[N_SERVOS];
extern int stepDelay[N_SERVOS];

// -----------------------------
// Function Declarations
// -----------------------------

#endif // USE_SERVOS

#endif // SERVOS_H
