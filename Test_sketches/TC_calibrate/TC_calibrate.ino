/* TC_calibrate.ino

	Use this sketch with the Mussel Tracker v2 boards
	to record thermocouple temperatures to the SD 
	card. 

	Error codes:
	Red led flashes quickly: real time clock not set
	Red + Green alternate quickly: SD card not found
	Red + Green flash together at 1Hz: could not initialize file
	Green LED on constantly: SD card failed during data collection
	

*/

#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>
#include <SPI.h>
#include <EEPROM.h>
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include "Adafruit_MAX31855.h" // https://github.com/adafruit/Adafruit-MAX31855-library
#include "MusselTrackerlib.h" // https://github.com/millerlp/MusselTrackerlib

#define ERRLED 5		// red error LED pin
#define GREENLED 6		// green LED pin

// Comment out the following line to remove parts of the
// test code from functioning. 
#define ECHO_TO_SERIAL // For testing serial output over FTDI adapter

// Placeholder serialNumber
char serialNumber[]="SN00";


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
char filename[] = "SN00_CALIB_YYYYMMDD_HHMM_00.CSV";

// Define an array to hold the 4-digit serial number that may be in eeprom
char serialnumber[5]; // the extra space is for a \0 null terminator character
// Define a flag to show whether the serialNumber value is value or just zeros
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

void setup() {
#ifdef ECHO_TO_SERIAL  
	Serial.begin(57600);
	Serial.println("Hello");
#endif
	// Setup led pins
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
	delay(10);
	printTimeSerial(rtc.now()); // function in MusselTrackerlib
	Serial.println();
	if (newtime.year() < 2015 | newtime.year() > 2035) {
		// There is an error with the clock, halt everything
		while(1){
			// Flash the error led to notify the user
			digitalWrite(ERRLED, !digitalRead(ERRLED));
			delay(100);
		}
	}
	

	
	// Grab the serial number from the EEPROM memory
	// The character array serialNumber was defined in the preamble
	EEPROM.get(0, serialNumber);
	if (serialNumber[0] == 'S') {
		serialValid = true;
		Serial.print(F("Read serial number: "));
		Serial.println(serialNumber);
	} else {
		Serial.print(F("No valid serial number: "));
		Serial.println(serialNumber);
	}

	
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
	initCalibFileName(newtime);

#ifdef ECHO_TO_SERIAL		
	Serial.print(F("Writing to "));
	Serial.println(filename);
#endif	
	
	oldtime = rtc.now();
}

void loop() {
	newtime = rtc.now(); // Grab the current time
	if (newtime.unixtime() >= oldtime.unixtime() + 1) {
		oldtime = newtime; // update oldtime
		
		// Store the unixtime in the array
		unixtimeArray[0] = oldtime.unixtime();
		// Take temperature readings
		temp1Array[0] = thermocouple1.readCelsius();
		temp2Array[0] = thermocouple2.readCelsius();
#ifdef ECHO_TO_SERIAL		
		printTimeSerial(oldtime); // from MusselTrackerlib
		Serial.print("\t");
		Serial.print("Temps: ");
		Serial.print(temp1Array[0]);
		Serial.print("C, ");
		Serial.print(temp2Array[0]);
		Serial.println("C ");
#endif		
		
		writeToSD();
		
	
	} // end of newtime >= oldtime if statement

}

//--------------------------------------------------------------
// writeToSD function. This formats the available data in the
// data arrays and writes them to the SD card file in a
// comma-separated value format.
void writeToSD (void) {

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
		printTimeToSD(logfile, unixtimeArray[i]); // function from MusselTrackerlib
		logfile.print(F(","));
		logfile.print(temp1Array[i]);
		logfile.print(F(","));
		logfile.println(temp2Array[i]);
	}
	// logfile.close(); // force the buffer to empty
	  DateTime t1 = DateTime(unixtimeArray[0]);
	  // If the seconds value is 30, update the file modified timestamp
	  if (t1.second() % 30 == 0){
	    logfile.timestamp(T_WRITE, t1.year(),t1.month(),t1.day(),t1.hour(),t1.minute(),t1.second());
	  }
	  digitalWrite(GREENLED, LOW);
}


//------------------------------------------------------------------------------
// initFileName - a function to create a filename for the SD card based
// on the 4-digit year, month, day, hour, minutes, serial number, and a 2-digit counter. 
// The variables 'filename' and 'serialNumber' are global character arrays defined
// in the preamble of the program.
// filename[] = "SN00_CALIB_YYYYMMDD_HHMM_00.CSV";

void initCalibFileName(DateTime time1) {
	 // Write the serialNumber value into the first positions
	for (byte i = 0; i <= 3; i++){
		filename[i] = serialNumber[i];
	}
	
	char buf[5];
	// integer to ascii function itoa(), supplied with numeric year value,
	// a buffer to hold output, and the base for the conversion (base 10 here)
	itoa(time1.year(), buf, 10);
	
	byte counter = 0;
	// copy the ascii year into the filename array
	for (byte i = 11; i <= 14; i++){
		filename[i] = buf[counter];
		counter++;
	}
	// Insert the month value
	if (time1.month() < 10) {
		filename[15] = '0';
		filename[16] = time1.month() + '0';
	} else if (time1.month() >= 10) {
		filename[15] = (time1.month() / 10) + '0';
		filename[16] = (time1.month() % 10) + '0';
	}
	// Insert the day value
	if (time1.day() < 10) {
		filename[17] = '0';
		filename[18] = time1.day() + '0';
	} else if (time1.day() >= 10) {
		filename[17] = (time1.day() / 10) + '0';
		filename[18] = (time1.day() % 10) + '0';
	}
	// Insert an underscore between date and time
	filename[19] = '_';
	// Insert the hour
	if (time1.hour() < 10) {
		filename[20] = '0';
		filename[21] = time1.hour() + '0';
	} else if (time1.hour() >= 10) {
		filename[20] = (time1.hour() / 10) + '0';
		filename[21] = (time1.hour() % 10) + '0';
	}
	// Insert minutes
		if (time1.minute() < 10) {
		filename[22] = '0';
		filename[23] = time1.minute() + '0';
	} else if (time1.minute() >= 10) {
		filename[22] = (time1.minute() / 10) + '0';
		filename[23] = (time1.minute() % 10) + '0';
	}
	// Insert another underscore after time
	filename[24] = '_';

		
	// Next change the counter on the end of the filename
	// (digits 19+20) to increment count for files generated on
	// the same day. This shouldn't come into play
	// during a normal data run, but can be useful when 
	// troubleshooting.
	for (uint8_t i = 0; i < 100; i++) {
		filename[25] = i / 10 + '0';
		filename[26] = i % 10 + '0';
		filename[27] = '.';
		filename[28] = 'c';
		filename[29] = 's';
		filename[30] = 'v';
		
		if (!sd.exists(filename)) {
			// when sd.exists() returns false, this block
			// of code will be executed to open the file
			if (!logfile.open(filename, O_RDWR | O_CREAT | O_AT_END)) {
				// If there is an error opening the file, notify the
				// user. Otherwise, the file is open and ready for writing
				// Turn both indicator LEDs on to indicate a failure
				// to create the log file
				while (1) {
				digitalWrite(ERRLED, !digitalRead(ERRLED)); // Toggle error led 
				digitalWrite(GREENLED, !digitalRead(GREENLED)); // Toggle indicator led 
				delay(1000);
				}
			}
			break; // Break out of the for loop when the
			// statement if(!logfile.exists())
			// is finally false (i.e. you found a new file name to use).
		} // end of if(!sd.exists())
	} // end of file-naming for loop
	
	// Serial.println(filename);
	
	// Write 1st header line to SD file based on mission info
	logfile.print(F("Board serial number:,"));
	if (serialValid) {
		// Print the serialNumber if it was valid
		logfile.print(serialNumber);
	}
	logfile.print(F(","));
	logfile.print(F(","));
	logfile.println(); // move to 2nd row of file
	// write a 2nd header line to the SD file
	logfile.println(F("POSIXt,DateTime,Temp1,Temp2"));
	// Update the file's creation date, modify date, and access date.
	logfile.timestamp(T_CREATE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_WRITE, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.timestamp(T_ACCESS, time1.year(), time1.month(), time1.day(), 
			time1.hour(), time1.minute(), time1.second());
	logfile.close();

} // end of initFileName function

