
#include <Arduino.h>
#include "pins.h"
#include "packets.h"

// Default robot speed
int defaultSpeed = 180;

// Encoder counters (must be volatile since used in ISRs)
volatile long pulsesA = 0;
volatile long pulsesB = 0;

// Motor specs (adjust to match your encoder)
const int PPR = 20;               // pulses per revolution of encoder
const int GEAR_RATIO = 30;        // gearbox ratio
const int CPR = PPR * GEAR_RATIO; // counts per shaft revolution

// --- Function prototypes ---
void setMotor(int RPWM, int LPWM, int speed);
void stopAll();
void encoderA_ISR();
void encoderB_ISR();
void reportFeedback();
void sendtoSPI(const MotorDrivePacket &packet); // simple implementation below

void setup()
{
  Serial.begin(9600);

  // Motor pins
  pinMode(RPWM_A, OUTPUT);
  pinMode(LPWM_A, OUTPUT);
  pinMode(REN_A, OUTPUT);
  pinMode(LEN_A, OUTPUT);

  pinMode(RPWM_B, OUTPUT);
  pinMode(LPWM_B, OUTPUT);
  pinMode(REN_B, OUTPUT);
  pinMode(LEN_B, OUTPUT);

  // Enable drivers (if you wired EN pins to Arduino)
  digitalWrite(REN_A, HIGH);
  digitalWrite(LEN_A, HIGH);
  digitalWrite(REN_B, HIGH);
  digitalWrite(LEN_B, HIGH);

  // Encoder pins
  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);

  // Attach interrupts safely (check if the pin supports external interrupts)
  if (digitalPinToInterrupt(ENC_A1) != NOT_AN_INTERRUPT)
  {
    attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, RISING);
  }
  else
  {
    Serial.println("Warning: ENC_A1 is not attachable as interrupt on this board.");
  }

  if (digitalPinToInterrupt(ENC_B1) != NOT_AN_INTERRUPT)
  {
    attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, RISING);
  }
  else
  {
    Serial.println("Warning: ENC_B1 is not attachable as interrupt on this board.");
  }

  stopAll();
  Serial.println("IBT-2 Robot Control with RPM Feedback Ready.");
  Serial.println("Commands: A <speed>, B <speed>, F, BK, L, R, S, V <speed>, FB");
}

void loop()
{
  if (Serial.available())
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0)
      return;

    int sp = line.indexOf(' ');
    String cmd = (sp == -1) ? line : line.substring(0, sp);
    String rest = (sp == -1) ? "" : line.substring(sp + 1);
    cmd.trim();
    rest.trim();

    if (cmd.equalsIgnoreCase("A"))
    {
      int val = rest.toInt();
      setMotor(RPWM_A, LPWM_A, val);
      Serial.print("Motor A set to ");
      Serial.println(val);
    }
    else if (cmd.equalsIgnoreCase("B"))
    {
      int val = rest.toInt();
      setMotor(RPWM_B, LPWM_B, val);
      Serial.print("Motor B set to ");
      Serial.println(val);
    }
    else if (cmd.equalsIgnoreCase("F") || cmd.equalsIgnoreCase("FORWARD"))
    {
      setMotor(RPWM_A, LPWM_A, defaultSpeed);
      setMotor(RPWM_B, LPWM_B, defaultSpeed);
      Serial.println("Robot: FORWARD");
    }
    else if (cmd.equalsIgnoreCase("BK") || cmd.equalsIgnoreCase("BACK") || cmd.equalsIgnoreCase("BACKWARD"))
    {
      setMotor(RPWM_A, LPWM_A, -defaultSpeed);
      setMotor(RPWM_B, LPWM_B, -defaultSpeed);
      Serial.println("Robot: BACKWARD");
    }
    else if (cmd.equalsIgnoreCase("L") || cmd.equalsIgnoreCase("LEFT"))
    {
      setMotor(RPWM_A, LPWM_A, -defaultSpeed);
      setMotor(RPWM_B, LPWM_B, defaultSpeed);
      Serial.println("Robot: LEFT");
    }
    else if (cmd.equalsIgnoreCase("R") || cmd.equalsIgnoreCase("RIGHT"))
    {
      setMotor(RPWM_A, LPWM_A, defaultSpeed);
      setMotor(RPWM_B, LPWM_B, -defaultSpeed);
      Serial.println("Robot: RIGHT");
    }
    else if (cmd.equalsIgnoreCase("S") || cmd.equalsIgnoreCase("STOP"))
    {
      stopAll();
      Serial.println("Robot: STOP");
    }
    else if (cmd.equalsIgnoreCase("V"))
    {
      if (rest.length() > 0)
      {
        defaultSpeed = constrain(rest.toInt(), 0, 255);
        Serial.print("Default speed set to ");
        Serial.println(defaultSpeed);
      }
      else
      {
        Serial.print("Default speed is ");
        Serial.println(defaultSpeed);
      }
    }
    else if (cmd.equalsIgnoreCase("FB") || cmd.equalsIgnoreCase("FEEDBACK"))
    {
      reportFeedback();
    }
    else
    {
      Serial.print("Unknown command: ");
      Serial.println(line);
    }
  }
}

// --- Feedback ---
void reportFeedback()
{
  noInterrupts();
  long countA = pulsesA;
  long countB = pulsesB;
  pulsesA = 0;
  pulsesB = 0;
  interrupts();

  float rpmA = 0;
  float rpmB = 0;
  float distanceA = 0;
  float distanceB = 0;
  float wheel_circumference = 0.20f; // meters (example: 20cm wheel, adjust as needed)

  if (CPR > 0)
  {
    rpmA = (countA * 60.0) / CPR; // RPM Motor A
    rpmB = (countB * 60.0) / CPR; // RPM Motor B
    distanceA = (countA / (float)CPR) * wheel_circumference;
    distanceB = (countB / (float)CPR) * wheel_circumference;
  }

  // Print feedback in CSV format for easy parsing: RPM_A,RPM_B,Speed,Current_A,Current_B,Distance_A,Distance_B
  Serial.print("FEEDBACK,");
  Serial.print(rpmA, 2); Serial.print(",");
  Serial.print(rpmB, 2); Serial.print(",");
  Serial.print(defaultSpeed); Serial.print(",");
  Serial.print(0.0, 2); Serial.print(","); // Current A (stub)
  Serial.print(0.0, 2); Serial.print(","); // Current B (stub)
  Serial.print(distanceA, 4); Serial.print(",");
  Serial.println(distanceB, 4);

  // Optionally, fill and send a feedback packet for SPI or other use
  MotorDrivePacket pkt;
  pkt.rpm = (int)rpmA;
  pkt.distance = distanceA;
  pkt.speed = defaultSpeed;
  pkt.current = 0.0;
  sendtoSPI(pkt);
}

// --- Motor Control Functions ---
void setMotor(int RPWM, int LPWM, int speed)
{
  speed = constrain(speed, -255, 255);
  if (speed > 0)
  {
    analogWrite(RPWM, speed);
    analogWrite(LPWM, 0);
  }
  else if (speed < 0)
  {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, -speed);
  }
  else
  {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, 0);
  }
}

void stopAll()
{
  setMotor(RPWM_A, LPWM_A, 0);
  setMotor(RPWM_B, LPWM_B, 0);
}

// --- Encoder ISRs ---
void encoderA_ISR()
{
  pulsesA++;
}
void encoderB_ISR()
{
  pulsesB++;
}

// --- Simple sendtoSPI implementation (placeholder) ---
void sendtoSPI(const MotorDrivePacket &packet)
{
  // Placeholder: you can replace this with actual SPI.transfer code
  Serial.print("sendtoSPI: rpm=");
  Serial.print(packet.rpm);
  Serial.print(" speed=");
  Serial.print(packet.speed);
  Serial.print(" current=");
  Serial.println(packet.current);
}
