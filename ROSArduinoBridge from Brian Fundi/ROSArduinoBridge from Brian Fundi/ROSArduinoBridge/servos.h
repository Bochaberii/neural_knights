#ifndef SWEEP_SERVO_H
#define SWEEP_SERVO_H

#include <Arduino.h>
#include <Servo.h>

// SweepServo class
class SweepServo {
  public:
    SweepServo() {}

    void initServo(int servoPin, int stepDelayMs, int initPosition) {
      this->servo.attach(servoPin);
      this->stepDelayMs = stepDelayMs;#ifndef SERVOS_H
#define SERVOS_H

#ifdef USE_SERVOS //cephas
#define N_SERVOS 1

// This delay in milliseconds determines the pause 
// between each one degree step the servo travels.  Increasing 
// this number will make the servo sweep more slowly.  
// Decreasing this number will make the servo sweep more quickly.
// Zero is the default number and will make the servos spin at
// full speed.  150 ms makes them spin very slowly.
//cephas
int servoPins[N_SERVOS] = {11}; // Changed to pin 11

//Step delay for each servo (ms between degree steps)
int stepDelay[N_SERVOS] = {10}; // Adjusted for smoother movement

//Initial positions for each servo (degrees)
int servoInitPosition[N_SERVOS] = {90}; // Changed to holding position of 90
//cephas
// int stepDelay [N_SERVOS] = { 15, 15 }; // ms

// //Pins
// byte servoPins [N_SERVOS] = { 44, 45};

// //Initial Position
// byte servoInitPosition [N_SERVOS] = { 0, 180 }; // [0, 180] degrees


class SweepServo
{
  public:
    SweepServo();
    void initServo(
        int servoPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    Servo& getServo(); // Changed to return by reference

  private:
    Servo servo;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    unsigned long lastSweepCommand; // Changed to unsigned long for proper time math
};

SweepServo servos [N_SERVOS];

#endif
#endif
      this->currentPositionDegrees = initPosition;
      this->targetPositionDegrees = initPosition;
      this->servo.write(initPosition);
      this->lastSweepCommand = millis();
    }

    void doSweep() {
      if (currentPositionDegrees == targetPositionDegrees) return;

      long now = millis();
      if (now - lastSweepCommand >= stepDelayMs) {
        if (currentPositionDegrees < targetPositionDegrees) {
          currentPositionDegrees++;
        } else if (currentPositionDegrees > targetPositionDegrees) {
          currentPositionDegrees--;
        }
        servo.write(currentPositionDegrees);
        lastSweepCommand = now;
      }
    }

    void setTargetPosition(int position) {
      position = constrain(position, 0, 180);
      targetPositionDegrees = position;
    }

    int getCurrentPosition() { return currentPositionDegrees; }

  private:
    Servo servo;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};

#endif
