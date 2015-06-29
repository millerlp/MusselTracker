#include "MusselTrackerlib.h" // https://github.com/millerlp/MusselTrackerlib


// Define a HallSensor object (from MusselTrackerv2.h library)
// to access the hall-effect sensor functions available in that
// library. 
HallSensor sensor;
// The HallSensor functions will require an argument to 
// specify which sensor your want, HALL1 or HALL2, so you 
// must always specify that in the hall effect sensor
// function calls. 



int myVal1, myVal2;

void setup() {
  // put your setup code here, to run once:
	
	sensor.begin(HALL1); // initialize hall1 sensor pins
	sensor.begin(HALL2); // initialize hall2 sensor pins
	
	Serial.begin(57600);
	Serial.println("Hello");
}

void loop() {
  // put your main code here, to run repeatedly:
	myVal1 = sensor.readHall(HALL1);
	myVal2 = sensor.readHall(HALL2);
	Serial.print(myVal1);
	Serial.print("\t");
	Serial.println(myVal2);
	delay(100);
}
