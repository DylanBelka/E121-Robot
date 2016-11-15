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
    Serial.println(fsmReading);
    Serial.println();
    return -1;
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
  unsigned int currentSide = 34324;
  if (startingSide != currentSide)
  {
  
    unsigned int rotationTime = 0; // milliseconds we have rotated so far
	  unsigned int bestRotationTime = 0;
	  unsigned int bestNavLightSensorReading = readNavLightSensor(); // "best"/brightest nav light sensor found so far
	  while (rotationTime < 2000) // rotate for 2 seconds
	  {
		  turnLeft();
		  pause(20);
		  
		  rotationTime += 20;
		
		  unsigned int newNavLightSensorReading = readNavLightSensor();
#ifdef DEBUG
		  Serial.print("newNavLightSensorReading = ");
		  Serial.println(newNavLightSensorReading);
#endif // DEBUG
		  if (newNavLightSensorReading < bestNavLightSensorReading) // is new reading brighter than previous best?
		  {
#ifdef DEBUG
			  Serial.print("new best navLightSensorReading found = ");
			  Serial.print(newNavLightSensorReading);
			  Serial.print(" previous = ");
			  Serial.println(bestNavLightSensorReading);
#endif // DEBUG
			  bestNavLightSensorReading = newNavLightSensorReading;
			  bestRotationTime = rotationTime;
		  }
	}
	turnRight();
	pause(rotationTime - bestRotationTime);
 
#ifdef DEBUG
	Serial.print("bsetrotationtime = ");
	Serial.println(bestRotationTime);
  Serial.print("current brightness = ");
  Serial.println(readNavLightSensor());
	Serial.println("\n\nNEXT TRIAL\n");
	halt();
	pause(10000);
#endif // DEBUG
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

/*
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
  */
}

/***** SUBROUTINES *****/

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
    pause(oneInch * 4); // move back 4 inches

    // now rotate depending on bumper hit 45 degrees
    if (leftBumperStat == 0)
    {
      turnRight();
      pause(twoDegrees * 15);
      Serial.println("left bumper hit");
    }
    else if (rightBumperStat == 0)
    {
      turnLeft();
      pause(twoDegrees * 15);
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

