void loop()
{
	unsigned int degreesRotated = 0;
	unsigned int bestNavLightSensorReadingAngle = 0;	// angle to rotate back to once the "best"/brightest
														// angle has been found
	unsigned int bestNavLightSensorReading = readNavLightSensor(); // "best"/brightest nav light sensor found so far
	while (degreesRotated < 360)
	{
		turnLeft();
		pause(twoDegrees);
		halt();
		pause(5);
		degreesRotated += 2;
		
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
			bestNavLightSensorReadingAngle = degreesRotated;
		}
	}
	turnLeft();
	pause(bestNavLightSensorReadingAngle / 2 * twoDegrees);
#ifdef DEBUG
	Serial.print("bestNavLightSensorReadingAngle = ");
	Serial.println(bestNavLightSensorReadingAngle);
	Serial.print("degreesRotated = ");
	Serial.println(degreesRotated);
	Serial.println("\n\nNEXT TRIAL\n");
	halt();
	pause(5000);
#endif // DEBUG
}