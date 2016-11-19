#include <SendOnlySoftwareSerial.h>
#include <ArduinoInit.h>

#define DEBUG

/***** CONSTANTS ******/

/// motor 1 is the one on the right
const int rightMotorSpeed = 50;
const int leftMotorSpeed = 54;

const unsigned int rightMotor = 1;
const unsigned int leftMotor = 2;

const unsigned int sideWhite = 0;
const unsigned int sideBlack = 1;

// values approximated by moving/rotating and measuring approx how far the robot moved/rotated in 1 second
const unsigned int oneFoot = 1400; // 1 foot ~ 1400ms of movement
const unsigned int oneInch = (oneFoot / 12);
const unsigned int twoDegrees = 7; // two degrees = 7 milliseconds (one degree ~ 3.5 millis)

const unsigned int fsmLEDPin = 8;
const unsigned int fsmSensorPin = 3;
const unsigned int navLightSensor = 0;
const unsigned int leftLightSensor = 1;
const unsigned int centerLightSensor = 2;
const unsigned int rightLightSensor = 4;

/***** GLOBAL VARIABLES *****/
static unsigned int startingSide;
static unsigned int bestNavLightSensorReading;

/*
 *  Detects current side and returns 1 for black and 0 for white
 *  returns -1 if the returned light value is not within range and prints val
 */
unsigned int detectCurrentSide()
{
  const int sideWhiteLow = 3000;  // lower bound of white side values
  const int sideWhiteHigh = 5000; // upper bound
  const int sideBlackLow = 15000;
  const int sideBlackHigh = 18000;

  outputHigh(fsmLEDPin);
  pause(100);
  unsigned int fsmReading = readADC(fsmSensorPin);
  if (fsmReading > sideWhiteLow && fsmReading < sideWhiteHigh) // white side?
  {
    return sideWhite;
  }
  else if (fsmReading > sideBlackLow && fsmReading < sideBlackHigh) // black side?
  {
    return sideBlack;
  }
  else
  {
    Serial.print("SIDE UNKNOWN\nReading: ");
    outputHigh(10); // turn on red led
    Serial.println(fsmReading);
    Serial.println();
    return -1;
  }
}

void rotateToNavLight()
{
  unsigned int navLightSensorReading = readNavLightSensor();
  bestNavLightSensorReading = navLightSensorReading;
  unsigned int timeRotated = 0; // time rotated so far
  while (timeRotated < 3000) // rotate for 3 seconds
  {
    turnLeft();
    pause(20);
    timeRotated += 20;
    navLightSensorReading = readNavLightSensor();
    if (navLightSensorReading < bestNavLightSensorReading) // this reading is brighter than previous brightest
    {
      bestNavLightSensorReading = navLightSensorReading;
    }
  }

  // to prevent the robot from rotating infinitely if it cant find a close enough brightness
  // we increase the range after 40 tries
  int attempts = 0;
  int range = 50;
  while (!isInRange(navLightSensorReading, range, bestNavLightSensorReading)) // rotate until within +-10 of brighest value
  {
    turnLeft();
    pause(20);
    navLightSensorReading = readNavLightSensor();
    attempts++;
    if (attempts == 40)
    {
      range += 300;
      attempts = 0;
    }
  }
}

unsigned int readNavLightSensor()
{
  return readADC(navLightSensor);  
}

void setup() 
{
  configArduino();
  attachInterrupt(0, interrupt0, LOW);
  attachInterrupt(1, interrupt1, LOW);
  motors(1, 'o', 0);
  motors(2, 'o', 0);
  startingSide = detectCurrentSide(); // what is our home side?
  rotateToNavLight();
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
  unsigned int currentSide = detectCurrentSide();
  while (startingSide == currentSide)
  {
    forward();
    pause(oneInch * 4);
  }

  // we are now on the enemy's side
  // move toward target light
  // figure out best way to move using 3 target light sensors
  unsigned int leftTargetLightSensorReading = readADC(leftLightSensor);
  unsigned int rightTargetLightSensorReading = readADC(rightLightSensor);
  unsigned int centerTargetLightSensorReading = readADC(centerLightSensor);

#ifdef DEBUG
  Serial.print("center reading = ");
  Serial.print(centerTargetLightSensorReading);
  Serial.print("   left reading = ");
  Serial.print(leftTargetLightSensorReading);
  Serial.print("   right reading = ");
  Serial.println(rightTargetLightSensorReading);
#endif // DEBUG

  // determine which is brightest and move towards it
  if (leftTargetLightSensorReading < rightTargetLightSensorReading && leftTargetLightSensorReading < centerTargetLightSensorReading) // left is brightest, turn left
  {
#ifdef DEBUG
    Serial.println("left is brightest");
#endif // DEBUG
    turnLeft();
    pause(twoDegrees * 10);
  }
  else if (rightTargetLightSensorReading < leftTargetLightSensorReading && rightTargetLightSensorReading < centerTargetLightSensorReading) // right is brightest, turn right
  {
#ifdef DEBUG
    Serial.println("right is brightest");
#endif // DEBUG
    turnRight();
    pause(twoDegrees * 10);
  }
  else // otherwise move forward
  {
#ifdef DEBUG
    Serial.println("center is brightest");
#endif // DEBUG
    forward();
    pause(oneInch * 4);
  }

#ifdef DEBUG
  Serial.println("\n");
#endif // DEBUG
}

/***** SUBROUTINES *****/

/* Returns true of num is in [center - R, center + R] */
// num = num to check
// R = range
// center = center
bool isInRange(int num, int R, int center) 
{
  return ((num > center && num < center + R) || (num < center && num > center - R));
}

/***** MOVEMENT SUBROUTINES *****/

void forward()
{
  motors(rightMotor, 'a', rightMotorSpeed); // one of the motors is "backwards" so opposite directions are needed
  motors(leftMotor, 'b', leftMotorSpeed);
}

void backward()
{
  motors(rightMotor, 'b', rightMotorSpeed);
  motors(leftMotor, 'a', leftMotorSpeed);
}

void turnLeft()
{
  motors(rightMotor, 'a', rightMotorSpeed);
  motors(leftMotor, 'a', leftMotorSpeed);
}

void turnRight()
{
  motors(rightMotor, 'b', rightMotorSpeed);
  motors(leftMotor, 'b', leftMotorSpeed);
}

void halt()
{
  motors('b', 'o', 0);
}

/***** INTERRUPT SUBROUTINES *****/

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
    pause(oneInch * 2); // move back 2 inches

    // now rotate depending on bumper hit 10 degrees
    if (leftBumperStat == 0)
    {
      turnRight();
      pause(twoDegrees * 5);
      Serial.println("left bumper hit");
    }
    else if (rightBumperStat == 0)
    {
      turnLeft();
      pause(twoDegrees * 5);
      Serial.println("right bumper hit");
    }
  }
}

