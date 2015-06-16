/*
	A routine to check the functionality of LSM303D accelerometer
	and Allegro A139x hall effect sensor breakout boards for the
	Mussel Tracker v2 project. 
	This sketch is loaded onto an Arduino Pro Mini 3.3V board,
	which is hooked up to two mounting boards for the
	accel and hall breakouts. 
	
	It queries the I2C line looking for LSM303D, and flashed a
	green led if it finds one. It flashes a red led if it can't
	find one.
	
	It queries Analog 0 after manipulating digital pin 9, which 
	is hooked up to the hall effect sensor Sleep line. If it gets
	a non-zero reading on Analog 0, it assumes that the hall 
	sensor works. There is a 10k pull-down resistor on A0 to
	keep it at zero when there is not hall effect sensor attached.
	If it finds a reasonable value, a green led goes on steady. If
	it can't find a hall sensor, a red led flashes. 
	
	There are separate red and green indicator LEDs for the 
	two different sensors. 


*/

#include <Wire.h>
#include "LSM303.h"

#define AREDLED 5 
#define AGRNLED 4
#define HREDLED 3
#define HGRNLED 2

#define HALLSLEEP 9

// Define LSM303 accelerometer/magnetometer objects
LSM303 accelcompass1;

// Variable to keep track of LSM303 status
bool failFlag = false;
// Keep track of Hall effect failure
bool failFlag2 = false;

int hallVal = 0;


void setup() {
  
  
	// Set up LED pins
	pinMode(AREDLED, OUTPUT);
	digitalWrite(AREDLED, LOW); 
	pinMode(AGRNLED, OUTPUT);
	digitalWrite(AGRNLED, LOW);
	pinMode(HREDLED, OUTPUT);
	digitalWrite(HREDLED, LOW);
	pinMode(HGRNLED, OUTPUT);
	digitalWrite(HGRNLED, LOW);
	
	pinMode(HALLSLEEP, OUTPUT);
	digitalWrite(HALLSLEEP, LOW); // shut hall off
	
	Serial.begin(57600);
	Serial.println("Hello");
	
	Wire.begin();
	Serial.println("Wire begun");
  //***************************************
// Initialize the accelerometer/compass
	// For the LSM that has its sa0 pin pulled high (ACCEL1),
	// use the arguments accelcompass1.init(LSM303::device_D, LSM303::sa0_high)
	// to tell the initialization function to look for the correct device
	if(!accelcompass1.init()){
		Serial.print("Failed to find accelerometer");
		failFlag = true;
	}
	accelcompass1.enableDefault();

	//****************************************
	// Check Hall effect sensor
	digitalWrite(HALLSLEEP, HIGH);
	delay(1);
	hallVal = analogRead(0);
	digitalWrite(HALLSLEEP, LOW);
	if (hallVal == 0 | hallVal == 1023){
		// If the value being read is bogus,
		// set failFlag2 true
		failFlag2 = true;
	}

  
}

void loop() {
  // put your main code here, to run repeatedly:
  
	// If the failFlag is true, flash the LED rapidly
	// to indicate failure
	if (failFlag){
		digitalWrite(AREDLED, !digitalRead(AREDLED));
	} else {
		// Success, turn green led on for accelerometer
		digitalWrite(AGRNLED, HIGH); 
		// Take accelerometer readings
		accelcompass1.read();
		Serial.print(accelcompass1.a.x);
		Serial.print("\t");
		Serial.print(accelcompass1.a.y);
		Serial.print("\t");
		Serial.println(accelcompass1.a.z);
	}
	
	if (failFlag2){
		// Toggle HREDLED if hall effect sensor didn't read
		digitalWrite(HREDLED, !digitalRead(HREDLED));
	} else {
		// Success, turn green led on for hall sensor
		digitalWrite(HGRNLED, HIGH);
		
		// Take some more readings for the serial monitor
		digitalWrite(HALLSLEEP, HIGH);
		delay(1);
		hallVal = analogRead(0);
		digitalWrite(HALLSLEEP, LOW);
		Serial.print("Hall: ");
		Serial.println(hallVal);
	}
	
	delay(100);
}
