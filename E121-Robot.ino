#include <SendOnlySoftwareSerial.h>
#include <ArduinoInit.h>

#define DEBUG

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
  unsigned int leftBumperStat;
  unsigned int rightBumperStat;
  
  pause(5);
  if ((leftBumperStat = readInput(leftBumper)) == 1 && (rightBumperStat = readInput(rightBumper)) == 1)
  {
    return;
  }

  motors('b', 'o', 0);
  pause(100);

  // while bumper is hit, store left+right bumper status upon first check
  // this guarentees that the robot will rotate in the proper direction AFTER backing up
  while((leftBumperStat = readInput(leftBumper)) == 0 || (rightBumperStat = readInput(rightBumper)) == 0)
  {
    backward();
    pause(oneInch * 4); // move back 4 inches

    // now rotate depending on bumper hit 45 degrees
    if (leftBumperStat == 0)
    {
      turnRight();
      pause(oneDegree * 45);
      Serial.println("left bumper hit");
    }
    else if (rightBumperStat == 0)
    {
      turnLeft();
      pause(oneDegree * 45);
      Serial.println("right bumper hit");
    }
#ifdef DEBUG
    else // not sure what this status would mean if the interrupt triggered
    {
      Serial.print("handleInterrupt() branches of rotation fell through. leftBumperStat = ");
      Serial.print(leftBumperStat);
      Serial.print(" rightBumperStat = ");
      Serial.println(rightBumperStat);
    }
#endif // DEBUG
  }
}

#ifdef DEBUG
void testSensor(byte sensor)
{
  unsigned int sensorVal = readADC(sensor);
  Serial.print("sensor #");
  Serial.print(sensor);
  Serial.print(" ");
  Serial.println(sensorVal);
}
#endif // DEBUG

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

  testSensor(1);
  testSensor(2);
  testSensor(4);
  testSensor(0);

  Serial.println("\n");

  pause(1000);
}

