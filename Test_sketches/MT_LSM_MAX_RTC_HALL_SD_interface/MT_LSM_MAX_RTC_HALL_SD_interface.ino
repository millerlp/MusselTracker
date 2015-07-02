/* MMT_LSM_MAX_RTC_HALL_SD_interface.ino
  Code to test what portions of a newly assembled 
  Mussel Tracker v2 board are functional.
  
  This version attempts to read the temperature
  from two MAX31855 chips, from LSM303D accelerometers
  and Allegro A1391 hall effect sensors, 
  and save the data to a SD card.
  
  Error codes:
  Red flashes quickly (10Hz): real time clock not set
  Red + Green alternate rapidly: SD card not found

*/

#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include "Adafruit_MAX31855.h" // https://github.com/adafruit/Adafruit-MAX31855-library
#include "LSM303.h" // https://github.com/pololu/lsm303-arduino
#include "MusselTrackerlib.h"	// https://github.com/millerlp/MusselTrackerlib

#define ERRLED 5		// red error LED pin
#define GREENLED 6		// green LED pin

// Comment out the following line to remove parts of the
// test code from functioning. 
#define ECHO_TO_SERIAL // For testing serial output over FTDI adapter


// Create real time clock object
RTC_DS3231 rtc;
DateTime newtime; // used to track time in main loop
DateTime oldtime; // used to track time in main loop
#define SAMPLETIME 1 // seconds between samples
#define SAMPLES_PER_SECOND 1 // number of samples per second

// Create sd objects
SdFat sd;
SdFile logfile;  // for sd card, this is the file object to be written to
SdFile setfile; // for sd card, this is the settings file to read from
const byte chipSelect = 10; // define the Chip Select pin for SD card

// Declare initial name for output files written to SD card
// The newer versions of SdFat library support long filenames
char filename[] = "YYYYMMDD_HHMM_00.CSV";
// Define name of settings file that may appear on SD card
char setfilename[] = "settings.txt";  
// Define an array to hold the 17-digit serial number that may be in eeprom
char serialnumber[17];

// Define character array to hold missionInfo from settings.txt
#define arraylen 50 // maximum number of characters in missionInfo array
char missionInfo[arraylen] = "Default mission information for csv file header"; 
// Define a flag to show whether the serialnumber value is value or just zeros
bool serialValid = 0;

// Declare data arrays
uint32_t unixtimeArray[SAMPLES_PER_SECOND]; // store unixtime values temporarily
float temp1Array[SAMPLES_PER_SECOND]; // store pressure readings temporarily
float temp2Array[SAMPLES_PER_SECOND]; // store temperature readings temporarily



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
// Define a HallSensor object (from MusselTrackerv2.h library)
// to access the hall effect sensor functions available in that
// library. 
HallSensor sensor;
// Variables to hold hall effect readings
int hallVal1 = 0;
int hallVal2 = 0;
// The HallSensor functions will require an argument to 
// specify which sensor your want, HALL1 or HALL2, so you 
// must always specify that in the hall effect sensor
// function calls. 



void setup() {

	// Initialize the Allegro A1391 hall effect sensors
	sensor.begin(HALL1);
	sensor.begin(HALL2);


#ifdef ECHO_TO_SERIAL  
  Serial.begin(57600);
  Serial.println("Hello");
#endif
  
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
		digitalWrite(ERRLED, !digitalRead(ERRLED));
		delay(100);
	}
	
  }

  delay(10);

  
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
	newtime = rtc.now();
	initFileName(newtime);
	


#ifdef ECHO_TO_SERIAL		
		Serial.print("Writing to ");
		Serial.println(filename);
#endif

	
	oldtime = rtc.now();
}

void loop() {
  // put your main code here, to run repeatedly:
	newtime = rtc.now(); // Grab the current time
	// Serial.println(newtime.unixtime());
	// printTime(newtime); Serial.println();
	if (newtime.unixtime() >= oldtime.unixtime() + 1) {
		// Serial.println("Arrived in the if statement");
		oldtime = newtime; // update oldtime
		
		
		unixtimeArray[0] = oldtime.unixtime();
		// Take temperature readings
		temp1Array[0] = thermocouple1.readCelsius();
		temp2Array[0] = thermocouple2.readCelsius();
		// Take accelerometer readings
		accelcompass1.read();
		if (accelcompass1.timeoutOccurred()){
			Serial.print("\tNo accel1\t");
		}
		delay(5);
		accelcompass2.read();
		if (accelcompass1.timeoutOccurred()){
			Serial.print("\tNo accel2\t");
		}
		// Take hall effect readings
		hallVal1 = sensor.readHall(HALL1);
		hallVal2 = sensor.readHall(HALL2);
		
		printTimeSerial(oldtime);
		Serial.print("\t");
		Serial.print("Temps: ");
		Serial.print(temp1Array[0]);
		Serial.print("C, ");
		Serial.print(temp2Array[0]);
		Serial.print("C ");
		Serial.print(hallVal1);
		Serial.print(F("\t"));
		Serial.print(hallVal2);
		Serial.print(F("\t"));
		Serial.print(accelcompass1.a.x);
		Serial.print(F("\t"));
		Serial.print(accelcompass1.a.y);
		Serial.print(F("\t"));
		Serial.print(accelcompass1.a.z);
		Serial.print(F("\t"));
		Serial.print(accelcompass1.m.x);
		Serial.print(F("\t"));
		Serial.print(accelcompass1.m.y);
		Serial.print(F("\t"));
		Serial.print(accelcompass1.m.z);
		Serial.print(F("\t"));
		Serial.print(accelcompass2.a.x);
		Serial.print(F("\t"));
		Serial.print(accelcompass2.a.y);
		Serial.print(F("\t"));
		Serial.print(accelcompass2.a.z);
		Serial.print(F("\t"));
		Serial.print(accelcompass2.m.x);
		Serial.print(F("\t"));
		Serial.print(accelcompass2.m.y);
		Serial.print(F("\t"));
		Serial.print(accelcompass2.m.z);
		Serial.println();
		
		writeToSD();

	}

} // end of main loop


//------------------------------------------------------------------------------
// initFileName - a function to create a filename for the SD card based
// on the 4-digit year, month, day, hour, minutes and a 2-digit counter. 
// The character array 'filename' was defined as a 20-character global array 
// at the top of the sketch.
void initFileName(DateTime time1) {
	// Serial.println("Start of initFileName"); delay(10);
	
	char buf[5];
	// integer to ascii function itoa(), supplied with numeric year value,
	// a buffer to hold output, and the base for the conversion (base 10 here)
	itoa(time1.year(), buf, 10);
	// copy the ascii year into the filename array
	for (byte i = 0; i <= 4; i++){
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
	
	// Serial.print("partial filename: ");
	// Serial.print(filename);
		
	// Next change the counter on the end of the filename
	// (digits 14+15) to increment count for files generated on
	// the same day. This shouldn't come into play
	// during a normal data run, but can be useful when 
	// troubleshooting.
	for (uint8_t i = 0; i < 100; i++) {
		filename[14] = i / 10 + '0';
		filename[15] = i % 10 + '0';
		filename[16] = '.';
		filename[17] = 'c';
		filename[18] = 's';
		filename[19] = 'v';
		
		
		
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
		logfile.print(serialnumber);
		logfile.print(" ");
	}
	logfile.print(missionInfo);
	logfile.print(F(","));
	// logfile.print(F("startMinute"));
	logfile.print(F(","));
	// logfile.print(startMinute);
	logfile.print(F(","));
	// logfile.print(F("minutes per hour"));
	logfile.print(F(","));
	// logfile.println(dataDuration);
	logfile.println(); // move to 2nd row of file
	// write a 2nd header line to the SD file
	logfile.println(F("POSIXt,DateTime,a1.x,a1.y,a1.z,m1.x,m1.y,m1.z,Temp1,Temp2,Hall1,Hall2"));
	// Update the file's creation date, modify date, and access date.
	logfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.close();

} // end of initFileName function


//--------------------------------------------------------------
// writeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void writeToSD (void) {

//	bitSet(PIND, 7); // Toggle Arduino pin D7 for oscilloscope monitoring
	digitalWrite(GREENLED,HIGH);
	// Reopen logfile. If opening fails, notify the user
	if (!logfile.isOpen()) {
		if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
			digitalWrite(ERRLED, HIGH); // turn on error LED
		}
	}

	// Step through each element of the sample data arrays
	// and write them to the SD card
	for (int i = 0; i < SAMPLES_PER_SECOND; i++) {
		// Write the unixtime
		logfile.print(unixtimeArray[i], DEC);
		logfile.print(F(","));
		printTimeToSD(logfile, unixtimeArray[i]);
		logfile.print(F(","));
		logfile.print(accelcompass1.a.x);
		logfile.print(F(","));
		logfile.print(accelcompass1.a.y);
		logfile.print(F(","));
		logfile.print(accelcompass1.a.z);
		logfile.print(F(","));
		logfile.print(accelcompass1.m.x);
		logfile.print(F(","));
		logfile.print(accelcompass1.m.y);
		logfile.print(F(","));
		logfile.print(accelcompass1.m.z);
		logfile.print(F(","));
		// logfile.print(fracSecArray[i], DEC);
		// logfile.print(F(","));
		
		logfile.print(temp1Array[i]);
		logfile.print(F(","));
		logfile.print(temp2Array[i]);
		logfile.print(F(","));
		logfile.print(hallVal1);
		logfile.print(F(","));
		logfile.println(hallVal2);

		// Begin by converting the floating point value of pressure to
		// a string, truncating at 2 digits of precision
		// dtostrf(pressureArray[i], precision+3, precision, pressBuffer);
		// // Then print the value to the logfile. 
		// logfile.print(pressBuffer);
		// logfile.print(F(","));
		
		// Write out temperature in Celsius
		// Begin by converting the floating point value of temperature to
		// a string, truncating at 2 digits of precision
		// dtostrf(tempCArray[i], precision+3, precision, tempBuffer);
		// logfile.println(tempBuffer);
	}
	// logfile.close(); // force the buffer to empty
	  DateTime t1 = DateTime(unixtimeArray[0]);
	  // If the seconds value is 30, update the file modified timestamp
	  if (t1.second() % 30 == 0){
	    logfile.timestamp(T_WRITE, t1.year(),t1.month(),t1.day(),t1.hour(),t1.minute(),t1.second());
	  }
	  digitalWrite(GREENLED, LOW);
}


