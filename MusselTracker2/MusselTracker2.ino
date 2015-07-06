/* MusselTrackerv2.ino
	Copyright Luke Miller 2015
	
  Code to run a MusselTracker v2 board with attached pairs of
  MAX31855 thermocouple sensors, Allegro A1393 hall effect
  sensors, and LSM303D accelerometer/magnetometers. 
  
  
  Error codes:
  Red flashes quickly (10Hz): real time clock not set
  Red + Green alternate rapidly: SD card not found

*/

#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h> // For reading the serial number stored in EEPROM
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include "Adafruit_MAX31855.h" // https://github.com/adafruit/Adafruit-MAX31855-library
#include "LSM303.h" // https://github.com/pololu/lsm303-arduino
#include "MusselTrackerlib.h"	// https://github.com/millerlp/MusselTrackerlib


#define ERRLED 5		// Red error LED pin
#define GREENLED 6		// Green LED pin
#define BUTTON1 2 		// BUTTON1 on INT0, pin PD2
// Comment out the following line to remove parts of the
// test code from functioning. 
#define ECHO_TO_SERIAL // For testing serial output over FTDI adapter


// ***** TYPE DEFINITIONS *****
typedef enum STATE
{
  STATE_DATA, // collecting data normally
  STATE_ENTER_CALIB, // user wants to calibrate
  STATE_CALIB1, // user chooses to calibrate accel 1
  STATE_CALIB2, // user chooses to calibrate accel 2
  STATE_CALIB_WAIT, // waiting for user to signal mussel is positioned
  STATE_CALIB_ACTIVE, // taking calibration data, button press ends this
  STATE_CLOSE_FILE, // close data file, start new file
} mainState_t;

typedef enum DEBOUNCE_STATE
{
  DEBOUNCE_STATE_IDLE,
  DEBOUNCE_STATE_CHECK,
  DEBOUNCE_STATE_TIME
} debounceState_t;


// main state machine variable, this takes on the various
// values defined for the STATE typedef above. 
mainState_t mainState;

// debounce state machine variable, this takes on the various
// values defined for the DEBOUNCE_STATE typedef above.
debounceState_t debounceState;

// Create real time clock object
RTC_DS3231 rtc;
DateTime newtime; // used to track time in main loop
DateTime oldtime; // used to track time in main loop
byte oldday; 		// used to keep track of midnight transition
#define SAMPLETIME 1 // seconds between samples

#define SAMPLES_PER_SECOND 4 // number of samples taken per second (4, 2, or 1)

// Create sd objects
SdFat sd;
SdFile logfile;  // for sd card, this is the file object to be written to
SdFile calibfile; // for sd card, this is the calibration file to write
const byte chipSelect = 10; // define the Chip Select pin for SD card

// Declare initial name for output files written to SD card
// The newer versions of SdFat library support long filenames
char filename[] = "YYYYMMDD_HHMM_00_SN00.csv";
// Define initial name of calibration file for accelerometers
char filenameCalib[] = "CAL0_YYYYMMDD_HHMM_00_SN00.csv";
// Placeholder serialNumber
char serialNumber[]="SN00";
// Define a flag to show whether the serialNumber value is real or just zeros
bool serialValid = false;

// Declare data arrays
uint32_t unixtimeArray[SAMPLES_PER_SECOND]; // store unixtime values temporarily
byte fracSecArray[SAMPLES_PER_SECOND]; // store fracSec values temporarily
int accelcompass1Array[SAMPLES_PER_SECOND][6]; // store accel/compass1 values
int accelcompass2Array[SAMPLES_PER_SECOND][6]; // store accel/compass2 values




// Define MAX31855 objects, need 2 of them for the two separate chips
#define CS_MAX1 8 // Chip Select for MAX31855 #1
#define CS_MAX2 9 // Chip Select for MAX31855 #2
Adafruit_MAX31855 thermocouple1(CS_MAX1);
Adafruit_MAX31855 thermocouple2(CS_MAX2);
double temp1 = 0; // hold output from MAX31855 #1
double temp2 = 0; // hold output from MAX31855 #2

// Define LSM303 accelerometer/magnetometer objects
LSM303 accelcompass1;
LSM303 accelcompass2;

//*************
// Define a HallSensor object (from MusselTrackerlib.h library)
// to access the hall effect sensor functions available in that
// library. 
HallSensor hallsensor;
// Variables to hold hall effect readings
int hallVal1 = 0;
int hallVal2 = 0;
// The HallSensor functions will require an argument to 
// specify which sensor your want, HALL1 or HALL2, so you 
// must always specify that in the hall effect sensor
// function calls. 


byte loopCount = 0; // counter to keep track of data sampling loops
byte fracSec = 0; // counter to keep track of fractional seconds
DateTime buttonTime; // hold the time since the button was pressed
DateTime chooseTime; // hold the time stamp when a waiting period starts
DateTime calibEnterTime; // hold the time stamp when calibration mode is entered
long buttonTime1; // hold the initial button press millis() value
byte debounceTime = 20; // milliseconds to wait for debounce
volatile bool buttonFlag = false; // Flag to mark when button was pressed
byte mediumPressTime = 2; // seconds to hold button1 to register a medium press
byte longPressTime = 5; // seconds to hold button1 to register a long press
byte pressCount = 0; // counter for number of button presses

void setup() {
	// Set button1 as an input
	pinMode(BUTTON1, INPUT_PULLUP);
	// Register an interrupt on INT0, attached to button1
	// which will call buttonFunc when the button is pressed.
	attachInterrupt(0, buttonFunc, LOW);
	// Set up the LEDs as output
	pinMode(ERRLED,OUTPUT);
	digitalWrite(ERRLED, LOW);
	pinMode(GREENLED,OUTPUT);
	digitalWrite(GREENLED, LOW);
	// Set the SPI bus chip select pins as outputs
	// for the two MAX31855 thermocouple chips
	pinMode(CS_MAX1, OUTPUT);
	digitalWrite(CS_MAX1, HIGH);
	pinMode(CS_MAX2, OUTPUT);
	digitalWrite(CS_MAX2, HIGH);


	// Grab the serial number from the EEPROM memory
	// The character array serialNumber was defined in the preamble
	EEPROM.get(0, serialNumber);
	if (serialNumber[0] == 'S') {
		serialValid = true; // set flag
#ifdef ECHO_TO_SERIAL		
		Serial.print(F("Read serial number: "));
		Serial.println(serialNumber);
#endif		
	} else {
#ifdef ECHO_TO_SERIAL	
		Serial.print(F("No valid serial number: "));
		Serial.println(serialNumber);
#endif		
	}


	// Initialize the Allegro A1393 hall effect sensors
	// There is only one hallsensor object, but you feed it
	// HALL1 or HALL2 as an argument to specify which channel
	// you want to activate and read. 
	hallsensor.begin(HALL1);
	hallsensor.begin(HALL2);


#ifdef ECHO_TO_SERIAL  
	Serial.begin(57600);
	Serial.println("Hello");
#endif

	// Initialize the real time clock DS3231M
	Wire.begin();
	rtc.begin();
	newtime = rtc.now();
	printTimeSerial(rtc.now());
	Serial.println();
	if (newtime.year() < 2015 | newtime.year() > 2035) {
		// There is an error with the clock, halt everything
		while(1){
		// Flash the error led to notify the user
		// This permanently halts execution, no data will be collected
			digitalWrite(ERRLED, !digitalRead(ERRLED));
			delay(100);
		}	
	}

//***************************************
	// Initialize the accelerometer/compass
	// For the LSM that has its sa0 pin pulled high (ACCEL1),
	// use the arguments accelcompass1.init(LSM303::device_D, LSM303::sa0_high)
	// to tell the initialization function to look for the correct device
	// When a device type and sa0 pin state are defined, the init() function 
	// will always return true.
	accelcompass1.init(LSM303::device_D, LSM303::sa0_high);
	accelcompass1.enableDefault();
	// Use sa0_low argument for the other accelerometer that has its address 
	// pin set to ground. 
	accelcompass2.init(LSM303::device_D, LSM303::sa0_low);
	accelcompass2.enableDefault();
	
	// You must set a timeout to handle cases where the sensor dies
	// or otherwise can't be found on the I2C bus, which would hang
	// the whole program without a timeout.
	accelcompass1.setTimeout(20); 
	accelcompass2.setTimeout(20);

//*************************************************************
// SD card setup and read (assumes Serial output is functional already)
	pinMode(chipSelect, OUTPUT);  // set chip select pin for SD card to output
	// Initialize the SD card object
	// Try SPI_FULL_SPEED, or SPI_HALF_SPEED if full speed produces
	// errors on a breadboard setup. 
	if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
	// If the above statement returns FALSE after trying to 
	// initialize the card, enter into this section and
	// hold in an infinite loop.
		while(1){ // infinite loop due to SD card initialization error
                        digitalWrite(ERRLED, HIGH);
                        delay(100);
                        digitalWrite(ERRLED, LOW);
                        digitalWrite(GREENLED, HIGH);
                        delay(100);
                        digitalWrite(GREENLED, LOW);
		}
	}
	newtime = rtc.now(); // grab the current time
	initFileName(newtime); // generate a file name
	
#ifdef ECHO_TO_SERIAL		
		Serial.print("Writing to ");
		Serial.println(filename);
#endif
	// Start 32.768kHz clock signal on TIMER2. 
	// Supply the current time value as the argument, returns 
	// an updated time
	newtime = startTIMER2(rtc.now());
	
	oldtime = newtime; // store the current time value
	oldday = oldtime.day(); // store the current day value
	
	mainState = STATE_DATA; // Start the main loop in data-taking state
}

void loop() {
	// Always start the loop by checking the time
	newtime = rtc.now(); // Grab the current time

	// Begin loop by checking the debounceState to 
	// handle any button presses
 	switch (debounceState) {
		// debounceState should normally start off as 
		// DEBOUNCE_STATE_IDLE until button1 is pressed,
		// which causes the state to be set to 
		// DEBOUNCE_STATE_CHECK
		case DEBOUNCE_STATE_IDLE:
			// Do nothing in this case
		break;
		
		case DEBOUNCE_STATE_CHECK:
			// If the debounce state has been set to 
			// DEBOUNCE_STATE_CHECK by the buttonFunc interrupt,
			// check if the button is still pressed
			if (digitalRead(BUTTON1) == LOW) {
				if (millis() > buttonTime1 + debounceTime) {
					// If the button has been held long enough to 
					// be a legit button press, switch to 
					// DEBOUNCE_STATE_TIME to keep track of how long 
					// the button is held
					debounceState = DEBOUNCE_STATE_TIME;
					buttonTime = rtc.now();
				} else {
					// If button is still pressed, but the debounce 
					// time hasn't elapsed, remain in this state
					debounceState = DEBOUNCE_STATE_CHECK;
				}
			} else {
				// If button1 is high again when we hit this
				// case in DEBOUNCE_STATE_CHECK, it was a false trigger
				// Reset the debounceState
				debounceState = DEBOUNCE_STATE_IDLE;
				buttonFlag = false;
				// Restart the button1 interrupt
				attachInterrupt(0, buttonFunc, LOW);
			}
		break; // end of case DEBOUNCE_STATE_CHECK
		
		case DEBOUNCE_STATE_TIME:
			if (digitalRead(BUTTON1) == HIGH) {
				// If the user released the button, now check how
				// long the button was depressed. This will determine
				// which state the user wants to enter. 
				
				
				DateTime checkTime = rtc.now(); // get the time
				
				if (checkTime.unixtime() < (buttonTime.unixtime() + mediumPressTime)) {
					Serial.println(F("Short press registered"));
					// User held button briefly, treat as a normal
					// button press, which will be handled differently
					// depending on which mainState the program is in.
					buttonFlag = true;
					
				} else if (checkTime.unixtime() > (buttonTime.unixtime() + mediumPressTime) &
					checkTime.unixtime() < (buttonTime.unixtime() + longPressTime)) {
					// User held button1 long enough to enter calibration
					// mode, but not long enough to enter close file mode
					mainState = STATE_ENTER_CALIB;
					
					// Flash both LEDs 5 times to let user know we've entered
					// calibration mode
					for (byte i = 0; i < 5; i++){
						digitalWrite(ERRLED, HIGH);
						digitalWrite(GREENLED, HIGH);
						delay(100);
						digitalWrite(ERRLED, LOW);
						digitalWrite(GREENLED, LOW);
						delay(100);
					}
					
					// Start a timer for entering Calib mode, to be used to give
					// the user time to enter 1 button press or 2 to choose 
					// which unit to calibrate
					chooseTime = rtc.now();
	
				} else if (checkTime.unixtime() > (buttonTime.unixtime() + longPressTime)){
					// User held button1 long enough to enter close file mode
					mainState = STATE_CLOSE_FILE;
				}
				
				// Now that the button press has been handled, return
				// to DEBOUNCE_STATE_IDLE and await the next button press
				debounceState = DEBOUNCE_STATE_IDLE;
				// Restart the button1 interrupt now that the button
				// has been released
				attachInterrupt(0, buttonFunc, LOW);
			} else {
				// If button is still low (depressed), remain in 
				// this DEBOUNCE_STATE_TIME
				debounceState = DEBOUNCE_STATE_TIME;
			}
			
		break; // end of case DEBOUNCE_STATE_TIME
		
	
	} // end switch(debounceState)
	 
	
	//----------------------------------------------------------
	switch (mainState) {
	
		case STATE_DATA:
			// Check to see if the current seconds value
			// is equal to oldtime.second(). If so, we
			// are still in the same second. If not,
			// the fracSec value should be reset to 0
			// and oldtime updated to equal newtime.
			if (oldtime.second() != newtime.second()) {
				fracSec = 0; // reset fracSec
				oldtime = newtime; // update oldtime
				loopCount = 0; // reset loopCount				
			}
			
			// Save current time to unixtimeArray
			unixtimeArray[loopCount] = newtime.unixtime();
			fracSecArray[loopCount] = fracSec;
			// Take accelerometer readings
			accelcompass1.read();
			accelcompass2.read();
			// If no timeout occurred, copy the data to the array
			if (!accelcompass1.timeoutOccurred()){
				accelcompass1Array[loopCount][0] = accelcompass1.a.x;
				accelcompass1Array[loopCount][1] = accelcompass1.a.y;
				accelcompass1Array[loopCount][2] = accelcompass1.a.z;
				accelcompass1Array[loopCount][3] = accelcompass1.m.x;
				accelcompass1Array[loopCount][4] = accelcompass1.m.y;
				accelcompass1Array[loopCount][5] = accelcompass1.m.z;
			} else {
				// If a timeout occurred, write NANs to the array
				for (byte j = 0; j < 6; j++){
					accelcompass1Array[loopCount][j] = NAN;
				}
			}
			
			if (!accelcompass2.timeoutOccurred()){
				// If no timeout occurred, copy the data to the array
				accelcompass2Array[loopCount][0] = accelcompass2.a.x;
				accelcompass2Array[loopCount][1] = accelcompass2.a.y;
				accelcompass2Array[loopCount][2] = accelcompass2.a.z;
				accelcompass2Array[loopCount][3] = accelcompass2.m.x;
				accelcompass2Array[loopCount][4] = accelcompass2.m.y;
				accelcompass2Array[loopCount][5] = accelcompass2.m.z;
			} else {
				for (byte j = 0; j < 6; j++){
					// If a timeout occurred, write NANs to the array
					accelcompass2Array[loopCount][j] = NAN;
				}
			}

			if (fracSec == 0) {
				temp1 = thermocouple1.readCelsius();
				temp2 = thermocouple2.readCelsius();
				// Take hall effect readings
				hallVal1 = hallsensor.readHall(HALL1);
				hallVal2 = hallsensor.readHall(HALL2);
			}
			
			// Now if loopCount is equal to the value in SAMPLES_PER_SECOND
			// (minus 1 for zero-based counting), then write out the contents
			// of the sample data arrays to the SD card. This should write data
			// every second
			if (loopCount >= (SAMPLES_PER_SECOND - 1)) {
				// Check to see if a new day has started. If so, open a new file
				// with the initFileName() function
				if (oldtime.day() != oldday) {
					// Close existing file
					logfile.close();
					// Generate a new output filename based on the new date
					initFileName(oldtime);
					// Update oldday value to match the new day
					oldday = oldtime.day();
				}
				// Call the writeToSD function to output the data array contents
				// to the SD card
				writeToSD();
	#ifdef ECHO_TO_SERIAL
			printTimeSerial(oldtime);
			Serial.print(F("\tTemp1: "));
			Serial.print(temp1);
			Serial.print(F("C, Hall1: "));
			Serial.print(hallVal1);
			Serial.print(F("\tAccel1:\t"));
			for (byte j = 0; j < 6; j++) {
				Serial.print(accelcompass1Array[0][j]);
				Serial.print(F("\t"));
			}
			Serial.print(F("Temp2: "));
			Serial.print(temp2);
			Serial.print(F("C, Hall2: "));
			Serial.print(hallVal2);
			Serial.print(F("\tAccel2:\t"));
			for (byte j = 0; j < 6; j++) {
				Serial.print(accelcompass2Array[0][j]);
				Serial.print(F("\t"));
			}
			delay(5);
			Serial.println();
			delay(10);
	#endif				
			} // end of if (loopCount >= (SAMPLES_PER_SECOND - 1))
			
			// Increment loopCount after writing all the sample data to
			// the arrays
			loopCount++; 
				
			// Increment the fractional seconds count
	#if SAMPLES_PER_SECOND == 4
			fracSec = fracSec + 25;
	#endif

	#if SAMPLES_PER_SECOND == 2
			fracSec = fracSec + 50;
	#endif
			goToSleep(); // function in MusselTrackerlib.h	
			// After awaking, this case should end and the main loop
			// should start again. 
			mainState = STATE_DATA;
		break; // end of case STATE_DATA
		//------------------------------------------------------------------
		case STATE_ENTER_CALIB:
		
			// Read a time value
			calibEnterTime = rtc.now();
	
			if (calibEnterTime.unixtime() < (chooseTime.unixtime() + longPressTime)){
				// If longPressTime has not elapsed, add any new button presses to the
				// the pressCount value
				if (buttonFlag) {
					pressCount++;
					// If the user pressed the button, reset chooseTime to
					// give them extra time to press the button again. 
					chooseTime = calibEnterTime;
					buttonFlag = false; // reset buttonFlag
					Serial.print("Button press ");
					Serial.println(pressCount);
					if (pressCount > 2) {
						pressCount = 0;
					}	

					// Flash the green led 1 or 2 times to show current count
					if (pressCount > 0) {
						for (byte i = 0; i < pressCount; i++){
							digitalWrite(GREENLED, HIGH);
							delay(250);
							digitalWrite(GREENLED, LOW);
							delay(250);
						}
					} else if (pressCount == 0) {
						// Flash red led to show that we haven't 
						// got a useful choice
						digitalWrite(ERRLED, HIGH);
						delay(250);
						digitalWrite(ERRLED, LOW);
						delay(250);
					}
				}
				mainState = STATE_ENTER_CALIB; // remain in this state
			} else if (calibEnterTime.unixtime() >= (chooseTime.unixtime() + longPressTime)){
				switch (pressCount) {
					case 0:
						// if the user didn't press the button again, return
						// to normal data taking mode
						mainState = STATE_DATA; 
						Serial.println("returning to data state");
						digitalWrite(ERRLED, HIGH);
						digitalWrite(GREENLED, HIGH);
						delay(500);
						digitalWrite(ERRLED, LOW);
						digitalWrite(GREENLED, LOW);
						
					break;
					case 1:
						// If the user pressed one time, we'll calibrate accel1
						mainState = STATE_CALIB_WAIT;
						pressCount = 1;
						Serial.println("Calib accel 1");
						digitalWrite(ERRLED, HIGH);
						digitalWrite(GREENLED, HIGH);
						delay(500);
						digitalWrite(ERRLED, LOW);
						digitalWrite(GREENLED, LOW);
						delay(500);
						for (byte i = 0; i < pressCount; i++){
							digitalWrite(GREENLED, HIGH);
							delay(250);
							digitalWrite(GREENLED, LOW);
							delay(250);
						}
					break;
					case 2:
						// If the user pressed two times, we'll calibrate accel2
						mainState = STATE_CALIB_WAIT;
						pressCount = 2;
						Serial.println("Calib accel 2");
						digitalWrite(ERRLED, HIGH);
						digitalWrite(GREENLED, HIGH);
						delay(500);
						digitalWrite(ERRLED, LOW);
						digitalWrite(GREENLED, LOW);
						delay(500);
						for (byte i = 0; i < pressCount; i++){
							digitalWrite(GREENLED, HIGH);
							delay(250);
							digitalWrite(GREENLED, LOW);
							delay(250);
						}						
					break;
				} 
			}
			// Need to let the user decide which unit to calibrate here
			// keep count of button presses, within the time set by longPressTime

			
			

		break; // end of STATE_CALIB_ENTER
		
 		case STATE_CALIB_WAIT:
			// Now we wait for the user to get situated and press the 
			// button one more time to begin taking data
			// The green led will pulse on and off at 1Hz while waiting
			if (newtime.second() != calibEnterTime.second() ) {
				calibEnterTime = newtime; // reset calibEnterTime
				digitalWrite(GREENLED, !digitalRead(GREENLED));
			}
			
			// If the user finally presses the button, enter the active
			// calibration state
			if (buttonFlag) {
				mainState = STATE_CALIB_ACTIVE;
				buttonFlag = false; // reset buttonFlag
				// Create a data output file
				// initCalibFile(newtime);
			}
			
		break;
		
		case STATE_CALIB_ACTIVE:
			// Create a calibration file and write accelerometer/compass data
			// to it at a higher speed. 
			
			// TODO: reopen calibfile, write data to it at high speed
			
			
			// The user can press the button1 again to end calibration mode
 			if (buttonFlag) {
				buttonFlag = false;
				calibfile.close();
				initFileName(newtime); // open a new data file
				mainState = STATE_DATA; // return to STATE_DATA
				
			} 
			
		break; 
		
		//------------------------------------------------------------------
 		case STATE_CLOSE_FILE:
			logfile.close(); // make sure the data file is closed and saved.
			
			// Briefly flash the green led to show that program is in
			// shutdown mode
			for (byte i = 0; i < 5; i++){
				digitalWrite(ERRLED, HIGH);
				delay(100);
				digitalWrite(ERRLED, LOW);
				delay(100);
			}
			initFileName(rtc.now()); // open a new output file
#ifdef ECHO_TO_SERIAL
			Serial.print(F("Writing to "));
			printTimeSerial(newtime);
			Serial.println();
#endif		
			mainState = STATE_DATA; // return to normal data collection
		break; // end of case STATE_FILE_CLOSE
 
	} // End of switch statement


} // end of main loop


//-----------------------------------------------------------------------------
// This Interrupt Service Routine (ISR) is called every time the
// TIMER2_OVF_vect goes high (==1), which happens when TIMER2
// overflows. The ISR doesn't care if the AVR is awake or
// in SLEEP_MODE_PWR_SAVE, it will still roll over and run this
// routine. If the AVR is in SLEEP_MODE_PWR_SAVE, the TIMER2
// interrupt will also reawaken it. This is used for the goToSleep() function
ISR(TIMER2_OVF_vect) {
	// nothing needs to happen here, this interrupt firing should 
	// just awaken the AVR
}

//------------------------------------------------------------------------------
// buttonFunc
void buttonFunc(void){
	detachInterrupt(0); // turn off the interrupt
	buttonTime1 = millis(); // grab the current elapsed time
	// debounceState = DEBOUNCE_STATE_CHECK; // switch to new debounce state

	// Execution will now return to wherever it was interrupted, and this
	// interrupt will still be disabled. 
}

//------------------------------------------------------------------------------
// initFileName - a function to create a filename for the SD card based
// on the 4-digit year, month, day, hour, minutes and a 2-digit counter. 
// The character array 'filename' was defined as a global array 
// at the top of the sketch in the form "YYYYMMDD_HHMM_00_SN00.csv"
void initFileName(DateTime time1) {
	
	char buf[5];
	// integer to ascii function itoa(), supplied with numeric year value,
	// a buffer to hold output, and the base for the conversion (base 10 here)
	itoa(time1.year(), buf, 10);
	// copy the ascii year into the filename array
	for (byte i = 0; i < 4; i++){
		filename[i] = buf[i];
	}
	// Insert the month value
	if (time1.month() < 10) {
		filename[4] = '0';
		filename[5] = time1.month() + '0';
	} else if (time1.month() >= 10) {
		filename[4] = (time1.month() / 10) + '0';
		filename[5] = (time1.month() % 10) + '0';
	}
	// Insert the day value
	if (time1.day() < 10) {
		filename[6] = '0';
		filename[7] = time1.day() + '0';
	} else if (time1.day() >= 10) {
		filename[6] = (time1.day() / 10) + '0';
		filename[7] = (time1.day() % 10) + '0';
	}
	// Insert an underscore between date and time
	filename[8] = '_';
	// Insert the hour
	if (time1.hour() < 10) {
		filename[9] = '0';
		filename[10] = time1.hour() + '0';
	} else if (time1.hour() >= 10) {
		filename[9] = (time1.hour() / 10) + '0';
		filename[10] = (time1.hour() % 10) + '0';
	}
	// Insert minutes
		if (time1.minute() < 10) {
		filename[11] = '0';
		filename[12] = time1.minute() + '0';
	} else if (time1.minute() >= 10) {
		filename[11] = (time1.minute() / 10) + '0';
		filename[12] = (time1.minute() % 10) + '0';
	}
	// Insert another underscore after time
	filename[13] = '_';
	
	//"YYYYMMDD_HHMM_00_SN00.csv"
	
	// If there is a valid serialnumber, insert it into 
	// the file name in positions 17-20. 
	if (serialValid) {
		byte serCount = 0;
		for (byte i = 17; i < 21; i++){
			filename[i] = serialNumber[serCount];
			serCount++;
		}
	}
	
		
	// Next change the counter on the end of the filename
	// (digits 14+15) to increment count for files generated on
	// the same day. This shouldn't come into play
	// during a normal data run, but can be useful when 
	// troubleshooting.
	for (uint8_t i = 0; i < 100; i++) {
		filename[14] = i / 10 + '0';
		filename[15] = i % 10 + '0';
		
		if (!sd.exists(filename)) {
			// when sd.exists() returns false, this block
			// of code will be executed to open the file
			if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
				// If there is an error opening the file, notify the
				// user. Otherwise, the file is open and ready for writing
				// Turn both indicator LEDs on to indicate a failure
				// to create the log file
				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led 
				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led 
				delay(5);
			}
			break; // Break out of the for loop when the
			// statement if(!logfile.exists())
			// is finally false (i.e. you found a new file name to use).
		} // end of if(!sd.exists())
	} // end of file-naming for loop
	
	// Serial.println(filename);
	
	// Write 1st header line to SD file based on mission info
	if (serialValid) {
		logfile.print(serialNumber);
	}
	// Print a bunch of commas in the first row to mark the 
	// additional data columns
	for (byte i = 0; i < 18; i++){
		logfile.print(F(","));
	}
	
	logfile.println(); // move to 2nd row of file
	// write a 2nd header line to the SD file
	logfile.println(F("POSIXt,DateTime,fractional.Second,a1.x,a1.y,a1.z,m1.x,m1.y,m1.z,Temp1,Hall1,a2.x,a2.y,a2.z,m2.x,m2.y,m2.z,Temp2,Hall2"));
	// Update the file's creation date, modify date, and access date.
	logfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.close(); // force the data to be written to the file by closing it

} // end of initFileName function


//------------------------------------------------------------------------------
// initCalibFile - a function to create a filename for the SD card based
// The character array 'filenameCalib' was defined as a global array 
// at the top of the sketch in the form filenameCalib[] = "CAL0_YYYYMMDD_HHMM_00_SN00.csv";
/* void initCalibFile(DateTime time1) {
	
	if (pressCount == 1) {
		filenameCalib[3] = "1";
	} else if (pressCount == 2) {
		filenameCalib[3] = "2";
	}
	
	char buf[5];
	// integer to ascii function itoa(), supplied with numeric year value,
	// a buffer to hold output, and the base for the conversion (base 10 here)
	itoa(time1.year(), buf, 10);
	// copy the ascii year into the filenameCalib array
	byte count = 0;
	for (byte i = 5; i < 9; i++){
		filenameCalib[i] = buf[count];
		count++;
	}
	// Insert the month value
	if (time1.month() < 10) {
		filenameCalib[9] = '0';
		filenameCalib[10] = time1.month() + '0';
	} else if (time1.month() >= 10) {
		filenameCalib[9] = (time1.month() / 10) + '0';
		filenameCalib[10] = (time1.month() % 10) + '0';
	}
	// Insert the day value
	if (time1.day() < 10) {
		filenameCalib[11] = '0';
		filenameCalib[12] = time1.day() + '0';
	} else if (time1.day() >= 10) {
		filenameCalib[11] = (time1.day() / 10) + '0';
		filenameCalib[12] = (time1.day() % 10) + '0';
	}
	// Insert an underscore between date and time
	filenameCalib[13] = '_';
	// Insert the hour
	if (time1.hour() < 10) {
		filenameCalib[14] = '0';
		filenameCalib[15] = time1.hour() + '0';
	} else if (time1.hour() >= 10) {
		filenameCalib[14] = (time1.hour() / 10) + '0';
		filenameCalib[15] = (time1.hour() % 10) + '0';
	}
	// Insert minutes
		if (time1.minute() < 10) {
		filenameCalib[16] = '0';
		filenameCalib[17] = time1.minute() + '0';
	} else if (time1.minute() >= 10) {
		filenameCalib[16] = (time1.minute() / 10) + '0';
		filenameCalib[17] = (time1.minute() % 10) + '0';
	}
	// Insert another underscore after time
	filenameCalib[18] = '_';
	
	//"YYYYMMDD_HHMM_00_SN00.csv"
	
	// If there is a valid serialnumber, insert it into 
	// the file name in positions 17-20. 
	if (serialValid) {
		byte serCount = 0;
		for (byte i = 22; i < 26; i++){
			filenameCalib[i] = serialNumber[serCount];
			serCount++;
		}
	}
	
		
	// Next change the counter on the end of the filenameCalib
	// (digits 14+15) to increment count for files generated on
	// the same day. This shouldn't come into play
	// during a normal data run, but can be useful when 
	// troubleshooting.
	for (uint8_t i = 0; i < 100; i++) {
		filenameCalib[19] = i / 10 + '0';
		filenameCalib[20] = i % 10 + '0';
		
		if (!sd.exists(filenameCalib)) {
			// when sd.exists() returns false, this block
			// of code will be executed to open the file
			if (!calibfile.open(filenameCalib, O_RDWR | O_CREAT | O_AT_END)) {
				// If there is an error opening the file, notify the
				// user. Otherwise, the file is open and ready for writing
				// Turn both indicator LEDs on to indicate a failure
				// to create the log file
				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led 
				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led 
				delay(5);
			}
			break; // Break out of the for loop when the
			// statement if(!logfile.exists())
			// is finally false (i.e. you found a new file name to use).
		} // end of if(!sd.exists())
	} // end of file-naming for loop
	
	
	// Write 1st header line to SD file based on mission info
	if (serialValid) {
		calibfile.print(serialNumber);
	}
	calibfile.print(F(","));
	// Write starting time in the next column
	printTimeToSD(calibfile, rtc.now());
	calibfile.print(F(","));
	// Write accelerometer number in next column
	if (pressCount == 1) {
		calibfile.print("Accel 1");
	} else if (pressCount == 2) {
		calibfile.print("Accel 2");
	}
	calibfile.print(F(","));
	// Print a bunch of commas in the first row to mark the 
	// additional data columns
	for (byte i = 0; i < 3; i++){
		calibfile.print(F(","));
	}
	
	calibfile.println(); // move to 2nd row of file
	// write a 2nd header line to the SD file
	calibfile.println(F("millis,a.x,a.y,a.z,m.x,m.y,m.z"));
	// Update the file's creation date, modify date, and access date.
	calibfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	calibfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	calibfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	calibfile.close(); // force the data to be written to the file by closing it

} // end of initFileName function */



//--------------------------------------------------------------
// writeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void writeToSD (void) {

	// Flash the green LED every 30 seconds to show data is being collected
	if (newtime.second() % 10 == 0) {
		digitalWrite(GREENLED,HIGH);
	}
	// Reopen logfile. If opening fails, notify the user
	if (!logfile.isOpen()) {
		if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
			digitalWrite(ERRLED, HIGH); // turn on error LED
		}
	}

	// Step through each element of the sample data arrays
	// and write them to the SD card
	for (byte i = 0; i < SAMPLES_PER_SECOND; i++) {
		// Write the unixtime
		logfile.print(unixtimeArray[i], DEC); // POSIX time value
		logfile.print(F(","));
		printTimeToSD(logfile, unixtimeArray[i]); // human-readable date
		logfile.print(F(","));
		logfile.print(fracSecArray[i], DEC); // fractional seconds value (milliseconds)
		logfile.print(F(","));
		// Print accel/compass readings for mussel #1
		for (byte j = 0; j < 6; j++){
			logfile.print(accelcompass1Array[i][j]);
			logfile.print(F(","));
		}
		// Print thermocouple 1 value. The same value will get repeated
		// for the multiple entries per second, since we'll only read it
		// once per second. 
		logfile.print(temp1); 
		logfile.print(F(","));
		// Print hall effect 1 sensor raw analog value. The same value 
		// will get repeated for the multiple entries per second since we'll
		// only read it once per second.
		logfile.print(hallVal1); // hall effect sensor 1
		logfile.print(F(","));
		// Print accel/compass readings for mussel #2 
		for (byte j = 0; j < 6; j++){
			logfile.print(accelcompass2Array[i][j]);
			logfile.print(F(","));
		}
		// Print thermocouple 1 value. The same value will get repeated
		// for the multiple entries per second, since we'll only read it
		// once per second.
		logfile.print(temp2); // thermocouple 2
		logfile.print(F(","));
		// Print hall effect 2 sensor raw analog value. The same value 
		// will get repeated for the multiple entries per second since we'll
		// only read it once per second.
		logfile.println(hallVal2); // hall effect sensor 2
	}
	// logfile.close(); // force the buffer to empty
	  DateTime t1 = DateTime(unixtimeArray[0]);
	  // If the seconds value is 30, update the file modified timestamp
	  if (t1.second() % 30 == 0){
	    logfile.timestamp(T_WRITE, t1.year(),t1.month(),t1.day(),t1.hour(),t1.minute(),t1.second());
	  }
	
	// Turn the green LED off if it is on.
	if (digitalRead(GREENLED)) {
		digitalWrite(GREENLED,LOW);
	}
}


//--------------------------------------------------------------
// startTIMER2 function
// Starts the 32.768kHz clock signal being fed into XTAL1 to drive the
// quarter-second interrupts used during data-collecting periods. 
// Supply a current DateTime time value. 
// This function returns a DateTime value that can be used to show the 
// current time when returning from this function. 
DateTime startTIMER2(DateTime currTime){
	TIMSK2 = 0; // stop timer 2 interrupts

	rtc.enable32kHz(true);
	ASSR = _BV(EXCLK); // Set EXCLK external clock bit in ASSR register
	// The EXCLK bit should only be set if you're trying to feed the
	// 32.768 clock signal from the Chronodot into XTAL1. 

	ASSR = ASSR | _BV(AS2); // Set the AS2 bit, using | (OR) to avoid
	// clobbering the EXCLK bit that might already be set. This tells 
	// TIMER2 to take its clock signal from XTAL1/2
	TCCR2A = 0; //override arduino settings, ensure WGM mode 0 (normal mode)
	
	// Set up TCCR2B register (Timer Counter Control Register 2 B) to use the 
	// desired prescaler on the external 32.768kHz clock signal. Depending on 
	// which bits you set high among CS22, CS21, and CS20, different 
	// prescalers will be used. See Table 18-9 on page 158 of the AVR 328P 
	// datasheet.
	//  TCCR2B = 0;  // No clock source (Timer/Counter2 stopped)
	// no prescaler -- TCNT2 will overflow once every 0.007813 seconds (128Hz)
	//  TCCR2B = _BV(CS20) ; 
	// prescaler clk/8 -- TCNT2 will overflow once every 0.0625 seconds (16Hz)
	//  TCCR2B = _BV(CS21) ; 
#if SAMPLES_PER_SECOND == 4
	// prescaler clk/32 -- TCNT2 will overflow once every 0.25 seconds
	TCCR2B = _BV(CS21) | _BV(CS20); 
#endif

#if SAMPLES_PER_SECOND == 2
	TCCR2B = _BV(CS22) ; // prescaler clk/64 -- TCNT2 will overflow once every 0.5 seconds
#endif

#if SAMPLES_PER_SECOND == 1
    TCCR2B = _BV(CS22) | _BV(CS20); // prescaler clk/128 -- TCNT2 will overflow once every 1 seconds
#endif

	// Pause briefly to let the RTC roll over a new second
	DateTime starttime = currTime;
	// Cycle in a while loop until the RTC's seconds value updates
	while (starttime.second() == currTime.second()) {
		delay(1);
		currTime = rtc.now(); // check time again
	}

	TCNT2 = 0; // start the timer at zero
	// wait for the registers to be updated
	while (ASSR & (_BV(TCN2UB) | _BV(TCR2AUB) | _BV(TCR2BUB))) {} 
	TIFR2 = _BV(OCF2B) | _BV(OCF2A) | _BV(TOV2); // clear the interrupt flags
	TIMSK2 = _BV(TOIE2); // enable the TIMER2 interrupt on overflow
	// TIMER2 will now create an interrupt every time it rolls over,
	// which should be every 0.25, 0.5 or 1 seconds (depending on value 
	// of SAMPLES_PER_SECOND) regardless of whether the AVR is awake or asleep.
	return currTime;
}

