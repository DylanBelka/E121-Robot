#include <SendOnlySoftwareSerial.h>
#include <ArduinoInit.h>

/// motor 1 is the one on the right
#define MOTOR_1_SPEED 50
#define MOTOR_2_SPEED 54

#define rightMotor 1
#define leftMotor 2

#define sideWhite 0
#define sideBlack 1

#define oneFoot 1400 // 1 foot ~ 1400ms of movement
#define oneInch (oneFoot / 12)
#define oneDegree 3

const int fsmLEDPin = 8;
const int fsmSensorPin = 3;

void forward()
{
  motors(rightMotor, 'a', MOTOR_1_SPEED);
  motors(leftMotor, 'b', MOTOR_2_SPEED);
}

void backward()
{
  motors(rightMotor, 'b', MOTOR_1_SPEED);
  motors(leftMotor, 'a', MOTOR_2_SPEED);
}

void turnLeft()
{
  motors(rightMotor, 'a', MOTOR_1_SPEED);
  motors(leftMotor, 'a', MOTOR_2_SPEED);
}

void turnRight()
{
  motors(rightMotor, 'b', MOTOR_1_SPEED);
  motors(leftMotor, 'b', MOTOR_2_SPEED);
}

void halt()
{
  motors('b', 'o', 0);
}

/*
 *  Detects current side and returns 1 for black and 0 for white
 *  returns -1 if the returned light value is not within range and prints val
 */
int detectCurrentSide()
{
  const int sideWhiteLow = 3000;  // lower bound of white side values
  const int sideWhiteHigh = 5000; // upper bound
  const int sideBlackLow = 15000;
  const int sideBlackHigh = 18000;

  outputHigh(fsmLEDPin);
  pause(100);
  unsigned int fsmReading = readADC(fsmSensorPin);
  if (fsmReading > sideWhiteLow && fsmReading < sideWhiteHigh)
  {
    return sideWhite;
  }
  else if (fsmReading > sideBlackLow && fsmReading < sideBlackHigh)
  {
    return sideBlack;
  }
  else
  {
    Serial.print("SIDE UNKNOWN\nReading: ");
    Serial.println(fsmReading);
    Serial.println();
    return -1;
  }
}

void setup() {
  configArduino();
  attachInterrupt(0, interrupt0, LOW);
  attachInterrupt(1, interrupt1, LOW);
  motors(1, 'o', 0);
  motors(2, 'o', 0);
}

void interrupt0()
{
  handleInterrupt();
}

void interrupt1()
{
  handleInterrupt();
}

void handleInterrupt()
{
  const unsigned int rightBumper = 2;
  const unsigned int leftBumper = 3;
  
  pause(5);
  if (readInput(leftBumper) == 1 && readInput(rightBumper) == 1)
  {
    return;
  }

  motors('b', 'o', 0);
  pause(100);

  while(readInput(2) == 0 || readInput(3) == 0) // while bumper is hit
  {
    backward();
    pause(oneInch * 2); // move back 2 inches

    // now rotate depending on bumper hit ~20-30degrees
    if (readInput(leftBumper) == 0)
    {
      turnRight();
      pause(oneDegree * 20);
      Serial.println("left bumper hit");
    }
    else if (readInput(rightBumper) == 0)
    {
      turnLeft();
      pause(oneDegree * 20);
      Serial.println("right bumper hit");
    }
  }
}

void loop()
{
  int currentSide = detectCurrentSide();
  if (currentSide == sideBlack)
  {
    Serial.println("on black side");
  }
  else if (currentSide == sideWhite)
  {
    Serial.println("on white side");
  }

  turnLeft();
  pause(oneDegree * 180);
  halt();
  pause(1000);
}

