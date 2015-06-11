/* MusselTrackerTesting
  Code to test what portions of a newly assembled 
  Mussel Tracker v2 board are functional

*/

#include "SdFat.h" // https://github.com/greiman/SdFat
#include <Wire.h>
#include <SPI.h>
#include "RTClib.h" // https://github.com/millerlp/RTClib
#include <Adafruit_MAX31855.h> // https://github.com/adafruit/Adafruit-MAX31855-library


#define ERRLED 5
#define GREENLED 6

// Comment out any of the following 4 lines to remove parts of the
// test code from functioning. 
#define Serialtest // For testing serial output over FTDI adapter
#define RTCtest // For testing DS3231 function (requires Serial to be working)
#define SDtest	// For testing SD card slot function, requires Serial to be working
#define MAXtest	// For testing MAX31855 thermocouple chips, requires Serial to work

// Create real time clock object
RTC_DS3231 RTC;

// Create sd objects
SdFat sd;
SdFile logfile;  // for sd card, this is the file object to be written to
SdFile setfile; // for sd card, this is the settings file to read from
const byte chipSelect = 10; // define the Chip Select pin for SD card
// Serial streams
ArduinoOutStream cout(Serial);

byte thermocoupleSOPin = 12; // MISO
byte CS_MAX1 = 8; // Chip Select for MAX31855 #1
byte CS_MAX2 = 9; // Chip Select for MAX31855 #2
byte thermocoupleCLKPin = 13; // SCK pin for SPI

// Define MAX31855 objects, need 2 of them for the two separate chips
Adafruit_MAX31855 thermocouple1(CS_MAX1);
// Adafruit_MAX31855 thermocouple2(CS_MAX2);
double temp1 = 0;
double temp2 = 0;

void setup() {
	// SPI.begin();
	// SPI.setDataMode(SPI_MODE0);

  // put your setup code here, to run once:
#ifdef Serialtest  
  Serial.begin(57600);
  Serial.println("Hello");
#endif
  
  pinMode(ERRLED,OUTPUT);
  digitalWrite(ERRLED, LOW);
  pinMode(GREENLED,OUTPUT);
  digitalWrite(GREENLED, LOW);

#ifdef RTCtest 
  Wire.begin();
  RTC.begin();
  RTC.adjust(DateTime(__DATE__, __TIME__));
#endif


	// Set the chip select pins as outputs
	pinMode(CS_MAX1, OUTPUT);
	digitalWrite(CS_MAX1, HIGH);
	pinMode(CS_MAX2, OUTPUT);
	digitalWrite(CS_MAX2, HIGH);


//*************************************************************
// SD card setup and read (assumes Serial output is functional already)
#ifdef SDtest
	pinMode(chipSelect, OUTPUT);  // set chip select pin for SD card to output
	// Initialize the SD card object
	// Try SPI_FULL_SPEED, or SPI_HALF_SPEED if full speed produces
	// errors on a breadboard setup. 
	if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    if (sd.card()->errorCode()) {
      cout << F(
             "\nSD initialization failed.\n"
             "Do not reformat the card!\n"
             "Is the card correctly inserted?\n"
             "Is chipSelect set to the correct value?\n"
             "Does another SPI device need to be disabled?\n"
             "Is there a wiring/soldering problem?\n");
      cout << F("\nerrorCode: ") << hex << showbase;
      cout << int(sd.card()->errorCode());
      cout << F(", errorData: ") << int(sd.card()->errorData());
      cout << dec << noshowbase << endl;
      return;
    }
    cout << F("\nCard successfully initialized.\n");
    if (sd.vol()->fatType() == 0) {
      cout << F("Can't find a valid FAT16/FAT32 partition.\n");
      reformatMsg();
      return;
    }
    if (!sd.vwd()->isOpen()) {
      cout << F("Can't open root directory.\n");
      return;
    }
    cout << F("Can't determine error type\n");
    return;
  }
  cout << F("\nCard successfully initialized.\n");
  cout << endl;

  uint32_t size = sd.card()->cardSize();
  if (size == 0) {
    cout << F("Can't determine the card size.\n");
    cardOrSpeed();
    return;
  }
  uint32_t sizeMB = 0.000512 * size + 0.5;
  cout << F("Card size: ") << sizeMB;
  cout << F(" MB (MB = 1,000,000 bytes)\n");
  cout << endl;
  cout << F("Volume is FAT") << int(sd.vol()->fatType());
  cout << F(", Cluster size (bytes): ") << 512L * sd.vol()->blocksPerCluster();
  cout << endl << endl;

  cout << F("Files found (date time size name):\n");
  sd.ls(LS_R | LS_DATE | LS_SIZE);

  if ((sizeMB > 1100 && sd.vol()->blocksPerCluster() < 64)
      || (sizeMB < 2200 && sd.vol()->fatType() == 32)) {
    cout << F("\nThis card should be reformatted for best performance.\n");
    cout << F("Use a cluster size of 32 KB for cards larger than 1 GB.\n");
    cout << F("Only cards larger than 2 GB should be formatted FAT32.\n");
    reformatMsg();
    return;
  }
#endif 



}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(ERRLED,HIGH);
  delay(500);
  digitalWrite(GREENLED,HIGH);
  delay(250);
  digitalWrite(ERRLED, LOW);
  digitalWrite(GREENLED,LOW);
  delay(250);

#ifdef Serialtest
  Serial.println(millis());
#endif

#ifdef RTCtest  
  printTime(RTC.now());
  Serial.println();
#endif

#ifdef MAXtest
	temp1 = thermocouple1.readInternal();
	// temp2 = thermocouple2.readInternal();
	Serial.print("Internal temps: ");
	Serial.print(temp1);
	Serial.print("\t");
	Serial.println(temp2);
#endif


} // end of main loop




//------------------------------------------------
//------------------------------------------------
// printTime function takes a DateTime object from
// the real time clock and prints the date and time 
// to the serial monitor. 
void printTime(DateTime now){
	Serial.print(now.year(), DEC);
    Serial.print('-');
    Serial.print(now.month(), DEC);
    Serial.print('-');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
	if (now.minute() < 10) {
		Serial.print("0");
	}
    Serial.print(now.minute(), DEC);
    Serial.print(':');
	if (now.second() < 10) {
		Serial.print("0");
	}
    Serial.print(now.second(), DEC);
	// You may want to print a newline character
	// after calling this function i.e. Serial.println();
}

void cardOrSpeed() {
  cout << F("Try another SD card or reduce the SPI bus speed.\n");
  cout << F("Edit spiSpeed in this program to change it.\n");
}

void reformatMsg() {
  cout << F("Try reformatting the card.  For best results use\n");
  cout << F("the SdFormatter program in SdFat/examples or download\n");
  cout << F("and use SDFormatter from www.sdcard.org/downloads.\n");
}