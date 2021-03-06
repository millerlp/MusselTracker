/* MusselTracker2_screen_demo.ino
	Copyright Luke Miller 2018

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
	***********************************************************************
	
  Code to run a MusselTracker v2 board with attached pairs of
  MAX31855 thermocouple sensors, one Allegro A1393 hall effect
  sensors and a I2C OLED display
  screen. By default this program will output the sensor measurements
  to the screen for the temperatures and Hall effect 1 sensor. 

  
	Board error codes:

	
	Sensor error codes:
		At the top of every minute, the green LED will flash quickly 5
		times. This lets the user know that error codes will follow 
		during the following 6 seconds, one signal per second. 
		As each second ticks by (1-6), the red error LED will light once 
		if a sensor error code is set for that sensor. The ordering of the 6 potential flashes is:
			1. Accelerometer/magnetometer 1
			2. Accelerometer/magnetometer 2
			3. Thermocouple 1
			4. Thermocouple 2
			5. Hall effect sensor 1
			6. Hall effect sensor 2
		You will need to count off the seconds in your head after you see 
		the 5 quick green flashes to determine which error is being 
		signaled. If there are no errors, the red error LED will not light 
		during this time. 

*/

//#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>	// built in library, for I2C communications
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <SPI.h>	// built in library, for SPI communications
#include <EEPROM.h> // built in library, for reading the serial number stored in EEPROM
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include "Adafruit_MAX31855.h" // https://github.com/adafruit/Adafruit-MAX31855-library
#include "LSM303.h" // https://github.com/pololu/lsm303-arduino
#include "MusselTrackerlib.h"	// https://github.com/millerlp/MusselTrackerlib

//******************************
// Data collection rate, enter a value here of 4, 2, or 1 (samples per second)
#define SAMPLES_PER_SECOND 4 // number of samples taken per second (4, 2, or 1)
//******************************

#define ERRLED 5		// Red error LED pin
#define GREENLED 6		// Green LED pin
#define BUTTON1 2 		// BUTTON1 on INT0, pin PD2

// Comment out the following line to remove parts of the
// test code from functioning. 
#define ECHO_TO_SERIAL 0// For testing serial output over FTDI adapter

// Interval to flash green LED during normal data collection
// For every 10 seconds, enter 10, for every 30 seconds, enter 30
#define PULSE_INTERVAL 10

// ***** TYPE DEFINITIONS *****
typedef enum STATE
{
  STATE_DEMO, // collecting data normally
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
volatile debounceState_t debounceState;

//*************
// Create real time clock object
RTC_DS3231 rtc;
char buf[20]; // declare a string buffer to hold the time result

//*************
// Create sd card objects
//SdFat sd;
//SdFile logfile;  // for sd card, this is the file object to be written to
//SdFile calibfile; // for sd card, this is the calibration file to write
//const byte chipSelect = 10; // define the Chip Select pin for SD card

//************
// Define MAX31855 objects, need 2 of them for the two separate chips
#define CS_MAX1 8 // Chip Select for MAX31855 #1
#define CS_MAX2 9 // Chip Select for MAX31855 #2
Adafruit_MAX31855 thermocouple1(CS_MAX1);
Adafruit_MAX31855 thermocouple2(CS_MAX2);
double temp1 = 0; // hold output from MAX31855 #1
double temp2 = 0; // hold output from MAX31855 #2
double prevtemp1 = 0;
double prevtemp2 = 0;

//*************
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
int prevHall1 = 0;
// The HallSensor functions will require an argument to 
// specify which sensor your want, HALL1 or HALL2, so you 
// must always specify that in the hall effect sensor
// function calls. 

//******************************************
// OLED screen setup
// 0X3C+SA0 - 0x3C or 0x3D for OLED screen on I2C bus
#define I2C_ADDRESS1 0x3C   // Typical default address
//#define I2C_ADDRESS2 0x3D // Alternate address, after moving resistor on OLED
SSD1306AsciiWire oled1; // create OLED display object, using I2C Wire

//*********************************************
// Declare data arrays
uint32_t unixtimeArray[SAMPLES_PER_SECOND]; // store unixtime values temporarily
byte fracSecArray[SAMPLES_PER_SECOND]; // store fracSec values temporarily
int accelcompass1Array[SAMPLES_PER_SECOND][6]; // store accel/compass1 values
int accelcompass2Array[SAMPLES_PER_SECOND][6]; // store accel/compass2 values

int accMagCheck[4]; // store a value from each accel/mag sensor to compare to new values
unsigned int accMagCheckCount[4]; // count up number of matches
unsigned int maxRepeats = 120; // maximum number of accel/mag repeated values allowed before throwing error flag

// Declare initial name for output files written to SD card
//char filename[] = "YYYYMMDD_HHMM_00_SN00.csv";
// Define initial name of calibration file for accelerometers
//char filenameCalib[] = "CAL0_YYYYMMDD_HHMM_00_SN00.csv";
// Placeholder serialNumber
char serialNumber[] = "SN00";
// Define a flag to show whether the serialNumber value is real or just zeros
bool serialValid = false;
byte mcusr = 0; 	// variable to hold MCU status register codes
byte loopCount = 0; // counter to keep track of data sampling loops
byte fracSec = 0; // counter to keep track of fractional seconds
DateTime newtime; // used to track time in main loop
DateTime oldtime; // used to track time in main loop
byte oldday; 		// used to keep track of midnight transition
DateTime buttonTime; // hold the time since the button was pressed
DateTime chooseTime; // hold the time stamp when a waiting period starts
DateTime calibEnterTime; // hold the time stamp when calibration mode is entered
volatile unsigned long buttonTime1; // hold the initial button press millis() value
byte debounceTime = 20; // milliseconds to wait for debounce
volatile bool buttonFlag = false; // Flag to mark when button was pressed
byte mediumPressTime = 2; // seconds to hold button1 to register a medium press
byte longPressTime = 5; // seconds to hold button1 to register a long press
byte pressCount = 0; // counter for number of button presses
unsigned long prevMillis;	// counter for faster operations
unsigned long newMillis;	// counter for faster operations
// Flags to mark when sensors go bad
bool tc1fail = false;
bool tc2fail = false;
bool hall1fail = false;
bool hall2fail = false;
bool accel1fail = false;
bool accel2fail = false;
// Define two temperature limits for the thermocouples, values outside
// this range will trigger the error notification
float TClowerlimit = 0.0;
float TCupperlimit = 60.0;
int aaFilterBW;		// store accelerometer anti-alias filter bandwidth
byte accFullScale; 	// store accelerometer full scale range +/- g
float accSens; 	// store accelerometer sensitivity (mg/LSB)
int accDataRate;	// store accelerometer sample rate (not the same as datalogger sample rate)
byte magFullScale;	// store magnetometer full scale range (gauss)
float magSens;		// store magnetometer sensitivity (mgauss/LSB)
float magDataRate;	// store magnetometer sample rate (not the same as datalogger sample rate)
//---------------- setup loop ------------------------------------------------
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

	pinMode(4, OUTPUT);
	pinMode(3, OUTPUT);
	
	// Flash green LED to show that we just booted up
	for (byte i = 0; i < 3; i++){
		digitalWrite(GREENLED, HIGH);
		delay(100);
		digitalWrite(GREENLED, LOW);
		delay(100);
	}	
	
	// Brownout detection process.
	mcusr = MCUSR; // Grab the contents of the MCUSR
	MCUSR = 0; // Reset MCUSR to 0 so it is ready for the next go-around.
	// Call the checkMCUSR function to check reason for this restart.
	// If only a brownout is detected, this function will put the board
	// into a permanent sleep that can only be reset with the reset 
	// button or a complete power-down.
	checkMCUSR(mcusr, ERRLED);  // In the MusselTrackerlib library
								
	Serial.begin(57600);
#ifdef ECHO_TO_SERIAL  
	Serial.println(F("Hello"));
	Serial.print(F("MCUSR contents: "));
	printBits(mcusr);
	Serial.println();
	
#endif
	
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
		serialValid = false;
#endif		
	}

	// Initialize the Allegro A1393 hall effect sensors
	// There is only one hallsensor object, but you feed it
	// HALL1 or HALL2 as an argument to specify which channel
	// you want to activate and read. 
	hallsensor.begin(HALL1);
	hallsensor.begin(HALL2);

	// Initialize the real time clock DS3231M
	Wire.begin();	// Start the I2C library with default options
	rtc.begin();	// Start the rtc object with default options
	newtime = rtc.now(); // read a time from the real time clock
#ifdef ECHO_TO_SERIAL  
	printTimeSerial(rtc.now()); // print time to serial monitor
	Serial.println();
#endif

//***************************************
	// Initialize the accelerometer/compass
	// For the LSM303D that has its sa0 pin pulled high (ACCEL1),
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
	
	// Now set both accelerometers to sample based on the settings
	// in the function accelNormalMode()
	accelNormalMode(accelcompass1);
	accelNormalMode(accelcompass2);
	delay(5);
	// Read back the settings so they can be written to file headers
	getaccelSettings(accelcompass1);
	getaccelSettings(accelcompass2);
//*************************************************************

  //----------------------------------
  // Start up the oled displays
  oled1.begin(&Adafruit128x64, I2C_ADDRESS1);
  oled1.set400kHz();  
  oled1.setFont(Adafruit5x7);    
  oled1.clear(); 
  oled1.home();
  oled1.set2X();

  newtime = rtc.now(); // read a time from the real time clock
  newtime.toString(buf, 20); 
  // Now extract the time by making another character pointer that
  // is advanced 10 places into buf to skip over the date. 
  char *timebuf = buf + 10;
  for (int i = 0; i<11; i++){
    oled1.print(buf[i]);
  }
  oled1.println();
  oled1.println(timebuf);
  delay(1000);
  // Set up display screen to show values
  oled1.home();
  oled1.clear();
  oled1.set2X();
  oled1.println(F("T1: "));
  oled1.println(F("T2:"));
  oled1.println(F("Hall: "));



	// Start 32.768kHz clock signal on TIMER2. 
	// Supply the current time value as the argument, returns 
	// an updated time
	newtime = startTIMER2(rtc.now());
	
	oldtime = newtime; // store the current time value
	oldday = oldtime.day(); // store the current day value
	
	mainState = STATE_DEMO; // Start the main loop in data-taking state
}

//**************************************************************************
//**************************************************************************
// Welcome to the main loop
void loop() {
//	delay(2); // trying this to quell bogus sd writes
	// Always start the loop by checking the time
	newtime = rtc.now(); // Grab the current time
	
	
	//-------------------------------------------------------------
	// Begin loop by checking the debounceState to 
	// handle any button presses
 	switch (debounceState) {
		// debounceState should normally start off as 
		// DEBOUNCE_STATE_IDLE until button1 is pressed,
		// which causes the state to be set to 
		// DEBOUNCE_STATE_CHECK
		//************************************
		case DEBOUNCE_STATE_IDLE:
			// Do nothing in this case
		break;
		//************************************
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
		//*************************************
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
//					mainState = STATE_ENTER_CALIB;
          mainState = STATE_DEMO;
					
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
//					mainState = STATE_CLOSE_FILE;
         mainState = STATE_DEMO;
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
		//*****************************************************
		case STATE_DEMO:
			// bitSet(PIND, 4); // toggle on, for monitoring on o-scope
		
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
			
			// If it is the start of a new minute, flash the 
			// green led each time through the loop. This is
			// used to help the user look for error codes that
			// flash at seconds 1,2,3,4,5, and 6. 
			if (newtime.second() == 0) {
				digitalWrite(GREENLED, HIGH);
				delay(5);
				digitalWrite(GREENLED, LOW);
			}
			
			// Save current time to unixtimeArray
			unixtimeArray[loopCount] = newtime.unixtime();
			fracSecArray[loopCount] = fracSec;
			// Take accelerometer readings
//			accelcompass1.read();
//			accelcompass2.read();
//			// If no timeout occurred, copy the data to the array
//			if (!accelcompass1.timeoutOccurred()){
//				accelcompass1Array[loopCount][0] = accelcompass1.a.x;
//				accelcompass1Array[loopCount][1] = accelcompass1.a.y;
//				accelcompass1Array[loopCount][2] = accelcompass1.a.z;
//				accelcompass1Array[loopCount][3] = accelcompass1.m.x;
//				accelcompass1Array[loopCount][4] = accelcompass1.m.y;
//				accelcompass1Array[loopCount][5] = accelcompass1.m.z;
//                                accel1fail = false;
//                                // Now check and see if this new reading is the exact
//                                // same as the previous reading(s). This only checks 
//                                // the x-axis of the accel + mag to save time/space
//                                if (accelcompass1.a.x == accMagCheck[0]) {
//                                  if (accMagCheckCount[0] <= maxRepeats){
//                                    accMagCheckCount[0]++; // increment the repeat counter
//                                  }
//                                } else {
//                                  // If there is not a match, update the accMagCheck array
//                                  // and reset the count for that channel to 0
//                                  accMagCheck[0] = accelcompass1.a.x;
//                                  accMagCheckCount[0] = 0;
//                                }
//                                // Check and see if the mag x-axis reading is the exact
//                                // same as the previous reading(s).
//                                if (accelcompass1.m.x == accMagCheck[1]) {
//                                  if (accMagCheckCount[1] <= maxRepeats){
//                                    accMagCheckCount[1]++; // increment the repeat counter
//                                  }
//                                } else {
//                                  // If there is not a match, update the accMagCheck array
//                                  // and reset the count for that channel to 0
//                                  accMagCheck[1] = accelcompass1.m.x;
//                                  accMagCheckCount[1] = 0;
//                                }
//                                // Now check if either the accel or mag channel has been
//                                // repeating the same exact value for longer than the
//                                // maxRepeats amount. If so, set the fail flag to true.
//                                if (accMagCheckCount[0] >= maxRepeats | accMagCheckCount[1] >= maxRepeats){
//                                   accel1fail = true; 
//                                }
//
//			} else {
//				// If a timeout occurred, write zeros to the array
//				for (byte j = 0; j < 6; j++){
//					accelcompass1Array[loopCount][j] = -9999;
//				}
//                                accel1fail = true;
//			}
//			
//			if (!accelcompass2.timeoutOccurred()){
//				// If no timeout occurred, copy the data to the array
//				accelcompass2Array[loopCount][0] = accelcompass2.a.x;
//				accelcompass2Array[loopCount][1] = accelcompass2.a.y;
//				accelcompass2Array[loopCount][2] = accelcompass2.a.z;
//				accelcompass2Array[loopCount][3] = accelcompass2.m.x;
//				accelcompass2Array[loopCount][4] = accelcompass2.m.y;
//				accelcompass2Array[loopCount][5] = accelcompass2.m.z;
//                                accel2fail = false;
//                                // Now check and see if this new reading is the exact
//                                // same as the previous reading(s). This only checks 
//                                // the x-axis of the accel + mag to save time/space
//                                if (accelcompass2.a.x == accMagCheck[2]) {
//                                  if (accMagCheckCount[2] <= maxRepeats){
//                                    accMagCheckCount[2]++; // increment the repeat counter
//                                  }
//                                } else {
//                                  // If there is not a match, update the accMagCheck array
//                                  // and reset the count for that channel to 0
//                                  accMagCheck[2] = accelcompass2.a.x;
//                                  accMagCheckCount[2] = 0;                                  
//                                }
//                                // Check and see if the mag x-axis reading is the exact
//                                // same as the previous reading(s).
//                                if (accelcompass2.m.x == accMagCheck[3]) {
//                                  if (accMagCheckCount[3] <= maxRepeats){
//                                    accMagCheckCount[3]++; // increment the repeat counter
//                                  }
//                                } else {
//                                  // If there is not a match, update the accMagCheck array
//                                  // and reset the count for that channel to 0
//                                  accMagCheck[3] = accelcompass2.m.x;
//                                  accMagCheckCount[3] = 0;
//                                }
//                                // Now check if either the accel or mag channel has been
//                                // repeating the same exact value for longer than the
//                                // maxRepeats amount. If so, set the fail flag to true.
//                                if (accMagCheckCount[2] >= maxRepeats | accMagCheckCount[3] >= maxRepeats){
//                                   accel2fail = true; 
//                                }
//			} else {
//				for (byte j = 0; j < 6; j++){
//					// If a timeout occurred, write zeros to the array
//					accelcompass2Array[loopCount][j] = -9999;
//				}
//                                accel2fail = true;
//			}

//			if (fracSec == 0 | fracSec == 50) {
				// We only read the thermocouples and hall effect sensors
				// twice per second regardless of the main sampling rate,
				// since these won't change fast enough to warrant reading
				// them (and waking them) more than once per second.
				temp1 = thermocouple1.readCelsius();
				temp2 = thermocouple2.readCelsius();
				// Sanity check the thermocouple values
				if (temp1 < TClowerlimit | isnan(temp1) | temp1 > TCupperlimit){
				   tc1fail = true; 
				} else { tc1fail = false;}
				if (temp2 < TClowerlimit | isnan(temp2) | temp2 > TCupperlimit) {
				   tc2fail = true; 
				} else { tc2fail = false;}
				// Take hall effect readings
				hallVal1 = hallsensor.readHall(HALL1);
				hallVal2 = hallsensor.readHall(HALL2);
				// Sanity check the hall effect values
				// The pull-down resistors should keep the value
				// at roughly zero if the hall sensor is not
				// sending a signal. Normal functioning values
				// should be near 512, varying with magnet proximity
				if (hallVal1 < 2) {
				   hall1fail = true; 
				} else { hall1fail = false;}
				if (hallVal2 < 2) {
				   hall2fail = true;
				} else { hall2fail = false;}
				if (hallVal1 < 0 | hallVal1 > 1023){
					hallVal1 = -9999; // overwrite bad data with -9999
				}
				if (hallVal2 < 0 | hallVal1 > 1023) {
					hallVal2 = -9999; // overwrite bad data with -9999
				}

        // Update the OLED screen with temperatures and Hall Sensor 1 data

        oled1.home();
        oled1.set2X();
        if (temp1 != prevtemp1 & fracSec == 0){
          oled1.clear(72,128,oled1.row(),(oled1.row()+1));
          oled1.println(temp1,2);
        } else {
          oled1.println(); // don't update this line
        }
        if (temp2 != prevtemp2 & fracSec == 0){
          oled1.clear(72,128,oled1.row(),(oled1.row()+1));
          oled1.println(temp2,2);
        } else {
          oled1.println(); // don't update this line
        }
        if (hallVal1 != prevHall1){
          oled1.clear(72,128,oled1.row(),(oled1.row()+1));
          oled1.println(hallVal1);
        } else {
          oled1.println(); // don't update this line
        }
        if (fracSec == 0){
          // Update time at bottom of screen
           newtime.toString(buf, 20); 
          // Now extract the time by making another character pointer that
          // is advanced 10 places into buf to skip over the date. 
          char *timebuf = buf + 10;
          oled1.println(timebuf);
        }
 
        // Update previous temperature and hall values
        prevtemp1 = temp1;
        prevtemp2 = temp2;
        prevHall1 = hallVal1; // update prevHall1
       
	
//			} 	// end of if (fracSec == 0 | fracSec == 50)

		
			// Now if loopCount is equal to the value in SAMPLES_PER_SECOND
			// (minus 1 for zero-based counting), then write out the contents
			// of the sample data arrays to the SD card. This should write data
			// every second.
			if (loopCount == (SAMPLES_PER_SECOND - 1)) {

        

				
#ifdef ECHO_TO_SERIAL
				// If ECHO_TO_SERIAL is defined at the start of the 
				// program, then this section will send updates of the
				// sensor values once per second.
				printTimeSerial(oldtime);
				Serial.print(F(" Temp1: "));
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
				delay(10);
				Serial.println();
				delay(5);

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
			// bitSet(PIND, 4); // toggle off, for monitoring on o-scope
			// delay(1);
			// bitSet(PIND, 3); // toggle on, for monitoring on o-scope
			goToSleep(); // function in MusselTrackerlib.h	
			// bitSet(PIND, 3); // toggle off, for monitoring on o-scope
			// After waking, this case should end and the main loop
			// should start again. 
			mainState = STATE_DEMO;
		break; // end of case STATE_DEMO
		
		//*****************************************************
	
	} // End of switch (mainState) statement
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

//--------------- buttonFunc --------------------------------------------------
// buttonFunc
void buttonFunc(void){
	detachInterrupt(0); // Turn off the interrupt
	buttonTime1 = millis(); // Grab the current elapsed time
	debounceState = DEBOUNCE_STATE_CHECK; // Switch to new debounce state
	// Execution will now return to wherever it was interrupted, and this
	// interrupt will still be disabled. 
}


//----------- enableCalibMode ----------------------------------
// enableCalibMode function
// Requires the LSM303 library
void enableCalibMode(LSM303& accelcompass){
	// Uses the writeReg function from the LSM303 library to 
	// write to the various registers on a LSM303D and change
	// the device's settings. 
	
	// CTRL0: leave set to 0b0000 0000 to disable FIFO buffer
	accelcompass.writeReg(LSM303::CTRL0, 0x00);
	// CTRL1: set accel data rate to 400Hz
	// 0b1000 0111 = 0x87 AODR = 400Hz, BDU = 0 (continuous), xyz axes enabled
	// 0b0101 0111 = 0x57 AODR = 50Hz, BDU = 0 (continuous), xyz axes enabled
	accelcompass.writeReg(LSM303::CTRL1, 0x87);
	// CTRL2: leave accel full-scale range at +/- 4g
	// 0b0000 1000 = 0x08, ABW = 773Hz anti alias bw, AFS = +/- 4g
	// 0b1000 1000 = 0x88, ABW = 362Hz antialias bw, AFS = +/- 4g
	// 0b1100 1000 = 0xC8, ABW = 50Hz antialias bw, AFS = +/- 4g
	accelcompass.writeReg(LSM303::CTRL2, 0xC8);
	// CTRL6: set magnetic scale to +/- x gauss
        // 0b0000 0000 = 0x00 MFS = +/- 2gauss
	// 0b0010 0000 = 0x20 MFS = +/- 4gauss
	// 0b0100 0000 = 0x40 MFS = +/- 8gauss
	// 0b0110 0000 = 0x60 MFS = +/- 12gauss
	accelcompass.writeReg(LSM303::CTRL6, 0x40);
	// CTRL7: set magnetic data low-power mode to 0 so that changes
	// to CTRL5 have an effect
	// 0b0000 0000 = 0x00, AHPM = normal mode with xyz reset, AFDS = filter bypass, MPL = 0, MD = continuous mode
	// 0b0010 0000 = 0x20, AHPM = normal mode with xyz reset, AFDS = filter on, MPL = 0, MD = continuous
	// 0b1000 0000 = 0x80, AHPM = normal mode no reset, AFDS = filter bypass, MPL = 0, MD = continuous mode
	// 0b1010 0000 = 0xA0, AHPM = normal mode no reset, AFDS = filter on, MPL = 0, MD = continuous
	accelcompass.writeReg(LSM303::CTRL7, 0x00);
	// CTRL5: set magnetometer data rate to 100Hz, high resolution mode
	// 0b0111 0100 = 0x74
	accelcompass.writeReg(LSM303::CTRL5, 0x74);
}

//----------- accelNormalMode -------------------------------
void accelNormalMode(LSM303& accelcompass){
	// Set the LSM303D accel/magnetometer to normal data-collection
	// settings.
	// CTRL0: leave set to 0b0000 0000 to disable FIFO buffer
	accelcompass.writeReg(LSM303::CTRL0, 0x00);
	// CTRL1: set accel data rate to 50Hz
	// 0b1000 0111 = 0x87 AODR = 400Hz, BDU = 0 (continuous), xyz axes enabled
	// 0b0101 0111 = 0x57 AODR = 50Hz, BDU = 0 (continuous), xyz axes enabled
	accelcompass.writeReg(LSM303::CTRL1, 0x57);	
	// CTRL2: Set accel anti-alias filter bandwidth, leave accel full-scale range at +/- 4g
	// 0b0000 1000 = 0x08, ABW = 773Hz anti alias bw, AFS = +/- 4g
	// 0b1000 1000 = 0x88, ABW = 362Hz antialias bw, AFS = +/- 4g
	// 0b1100 1000 = 0xC8, ABW = 50Hz antialias bw, AFS = +/- 4g
	accelcompass.writeReg(LSM303::CTRL2, 0xC8);
	// CTRL7: set magnetic data low-power mode to 0 so that changes
	// to CTRL5 have an effect
	// 0b0000 0000 = 0x00, AHPM = normal mode with xyz reset, AFDS = filter bypass, MPL = 0, MD = continuous mode
	// 0b0010 0000 = 0x20, AHPM = normal mode with xyz reset, AFDS = filter on, MPL = 0, MD = continuous
	// 0b1000 0000 = 0x80, AHPM = normal mode no reset, AFDS = filter bypass, MPL = 0, MD = continuous mode
	// 0b1010 0000 = 0xA0, AHPM = normal mode no reset, AFDS = filter on, MPL = 0, MD = continuous
	accelcompass.writeReg(LSM303::CTRL7, 0x00);
	// CTRL6: set magnetic scale to +/- x gauss
	// 0b0000 0000 = 0x00 MFS = +/- 2gauss
	// 0b0010 0000 = 0x20 MFS = +/- 4gauss
	// 0b0100 0000 = 0x40 MFS = +/- 8gauss
	// 0b0110 0000 = 0x60 MFS = +/- 12gauss
	accelcompass.writeReg(LSM303::CTRL6, 0x40);
	// CTRL5: set magnetometer data rate to 6.25Hz, high resolution mode
	// 0b0110 0100 = 0x64
	accelcompass.writeReg(LSM303::CTRL5, 0x64);
}


// ---------- getaccelSettings ----------------------
// A function to read the CTRL registers of the LSM303D and assign 
// values to a set of global variables to keep track of sample rates
// and settings being used. These values could be written to an output
// file
// Values written are:
//	aaFilterBW = accelerometer anti-alias filter bandwidth (low pass filter)
//	accFullScale = accelerometer full scale range, units of g
// 	accSens = accelerometer nominal sensitivity, milli-g per least significant bit
//	accDataRate = accelerometer data rate, Hz
//	magFullScale = magnetometer full scale range, units of gauss
// 	magSens = magnetometer nominal sensitivity, milli-gauss per least significant bit
//	magDataRate = magnetometer data rate, Hz
void getaccelSettings(LSM303& accelcompass){
	// Read CTRL2 register
	byte ctrl = accelcompass.readReg(LSM303::CTRL2);
	byte tempVar = ctrl & 0xC0; // Test 1st to bits of ctrl2 value
	switch (tempVar){
		case 0x00:
			aaFilterBW = 773; // Hz
		break;
		case 0x40:
			aaFilterBW = 194; // Hz
		break;
		case 0x80:
			aaFilterBW = 362; // Hz
		break;
		case 0xC0:
			aaFilterBW = 50; // Hz
		break;
	}
	// Determine the accelerometer full-scale range (and sensitivity)
	tempVar = ctrl & 0x38; // test bits 5,4,3 of ctrl2 value
	switch (tempVar){
		case 0x00:
			accFullScale = 2; // +/- 2g
			accSens = 0.61; 	// sensitivity = 0.061 mg/LSB
		break;
		case 0x08:
			accFullScale = 4; // +/- 4g
			accSens = 0.122; 	// sensitivity = 0.122 mg/LSB
		break;
		case 0x10:
			accFullScale = 6; // +/- 6g
			accSens = 0.183; 	// sensitivity = 0.183 mg/LSB
		break;
		case 0x18:
			accFullScale = 8; // +/- 8g
			accSens = 0.244; 	// sensitivity = 0.244 mg/LSB
		break;
		case 0x20:
			accFullScale = 16; // +/- 16g
			accSens = 0.732; 	// sensitivity = 0.732 mg/LSB
		break;
	}	// end of switch(tempVar) for CTRL2
	ctrl = accelcompass.readReg(LSM303::CTRL1);
	tempVar = ctrl & 0xF0;
	switch (tempVar){
		case 0x00:
			accDataRate = 0;		// Hz (power-down mode)
		break;
		case 0x10:
			accDataRate = 3.125; 	// Hz
		break;
		case 0x20:
			accDataRate = 6.25; 	// Hz
		break;
		case 0x30:
			accDataRate = 12.5;		// Hz
		break;
		case 0x40:
			accDataRate = 25;		// Hz
		break;
		case 0x50:
			accDataRate = 50;		// Hz
		break;
		case 0x60:
			accDataRate = 100;		// Hz
		break;
		case 0x70:
			accDataRate = 200;		// Hz
		break;
		case 0x80:
			accDataRate = 400;		// Hz
		break;
		case 0x90:
			accDataRate = 800;		// Hz
		break;
		case 0xA0:
			accDataRate = 1600;		// Hz
		break;
		
	} // end of switch(tempVar) for CTRL1
	
	// Read magnetometer full scale range from CTRL6
	ctrl = accelcompass.readReg(LSM303::CTRL6);
	tempVar = ctrl & 0x60;
	switch (tempVar){
		case 0x00:
			magFullScale = 2;		// +/- 2 gauss
			magSens = 0.080;		// sensitivity, mgauss/LSB
		break;
		case 0x20:
			magFullScale = 4;		// +/- 4 gauss
			magSens = 0.160;		// sensitivity, mgauss/LSB
		break;
		case 0x40:
			magFullScale = 8;		// +/- 8 gauss
			magSens = 0.320;		// sensitivity, mgauss/LSB
		break;
		case 0x60:
			magFullScale = 12;		// +/- 12 gauss
			magSens = 0.479;		// sensitivity, mgauss/LSB
		break;
	} // end of switch (tempVar) for CTRL6
	
	ctrl = accelcompass.readReg(LSM303::CTRL5);
	tempVar = ctrl & 0x1C; // test the MODR bits 2:0
	switch (tempVar){
		case 0x00:
			magDataRate = 3.125;	// Hz
		break;
		case 0x04:
			magDataRate = 6.25;		// Hz
		break;
		case 0x08:
			magDataRate = 12.5;		// Hz
		break;
		case 0x0C:
			magDataRate = 25;		// Hz
		break;
		case 0x10:
			magDataRate = 50;		// Hz
		break;
		case 0x14:
			magDataRate = 100;		// Hz
		break;
	} // end of switch (tempVar) for CTRL5
}


//---------- startTIMER2 ----------------------------------------------------
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

