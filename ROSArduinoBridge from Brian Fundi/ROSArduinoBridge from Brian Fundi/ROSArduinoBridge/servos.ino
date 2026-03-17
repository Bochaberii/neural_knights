/***************************************************************
   Servo Sweep - by Nathaniel Gallinger

   Sweep servos one degree step at a time with a user defined
   delay in between steps.  Supports changing direction 
   mid-sweep.  Important for applications such as robotic arms
   where the stock servo speed is too fast for the strength
   of your system.

 *************************************************************/

#ifdef USE_SERVOS


// Constructor
SweepServo::SweepServo()
{
  this->currentPositionDegrees = 0;
  this->targetPositionDegrees = 0;
  this->lastSweepCommand = 0;
}


// Init
void SweepServo::initServo(
    int servoPin,
    int stepDelayMs,
    int initPosition)
{
  this->servo.attach(servoPin);
  this->stepDelayMs = stepDelayMs;
  // Add position constraint
  this->currentPositionDegrees = constrain(initPosition, 0, 180);
  this->targetPositionDegrees = constrain(initPosition, 0, 180);
  this->lastSweepCommand = millis();
}


// Perform Sweep
void SweepServo::doSweep()
{
  // Fixed time calculation to handle millis() overflow
  unsigned long now = millis();
  
  // Check if time for a step
  if (now - this->lastSweepCommand >= (unsigned long)this->stepDelayMs) {
    // Check step direction
    if (this->targetPositionDegrees > this->currentPositionDegrees) {
      this->currentPositionDegrees++;
      this->servo.write(this->currentPositionDegrees);
    }
    else if (this->targetPositionDegrees < this->currentPositionDegrees) {
      this->currentPositionDegrees--;
      this->servo.write(this->currentPositionDegrees);
    }
    // if target == current position, do nothing

    // reset timer
    this->lastSweepCommand = now;
  }
}


// Set a new target position
void SweepServo::setTargetPosition(int position)
{
  // Add position constraint
  this->targetPositionDegrees = constrain(position, 0, 180);
}


// Accessor for servo object - now returns a reference
Servo& SweepServo::getServo()
{
  return this->servo;
}


#endif
