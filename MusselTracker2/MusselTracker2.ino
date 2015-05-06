/* 
	MusselTracker2.ino
	Based on code created by 
	Henry Jordan, Lenny Turcios, and Andrew Petersen
	Loyola Marymount University Engineering + Biology programs
	
	Modified by Luke Miller 2015
*/
 
 
// Includes 
#include <Wire.h>
#include <Adafruit_LSM303.h> // https://github.com/adafruit/Adafruit_LSM303
#include <SD.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>


// Define objects and buffers
Adafruit_LSM303 lsm;
  
char outputString [100];
char fileName [13];
char timeString [16];
char currentTime[16];

char timeFile[13] = "THETIME1.TXT";
char magFile[13] = "MAGNETLG.TXT";
char errFile[13] = "ERRORLOG.TXT";

int hall;         // Hall Effect voltage
int Vcc = 0;      //source voltage
int Vo = 0;       //voltage across thermistor

int second;
int minute;
int hour;
int day;
int month;
int year;


//pin definitions
const int pVcc = A0;  //Pin for Vcc
const int pVo = A1;   //Pin for Vo
const int hallPin = A2;
const int rtcInterrupt = 2;
const int heartbeatLED = 8;
const int errorLED = 9;
const int chipSelect = 10;

// Magic Numbers
const int rtcBytes = 7;
const int delayLength = 450; // delay in millisec


// Initialize the RTC
void startTime() {

  // This block puts 0x00 in the control register, starting the RTC
  // It also configures the RTC to output a 1Hz SQW
  // The date should be set before this code runs
  Wire.beginTransmission(0x68);                              
  Wire.write(byte(0x0E));           
  Wire.write(byte(0x00));
  Wire.endTransmission();     
  
}


// Get the time from the RTC
void getTime() {
  
  // Set RTC pointer back to 0
  Wire.beginTransmission(0x68);                              
  Wire.write(byte(0x00));
  Wire.endTransmission(); 

  // get 7 bytes from RTC (the ones that contain current time info)
  Wire.requestFrom(0x68, rtcBytes);
  
  // Read the bytes
  int i = 0;  
  while(Wire.available() && i < rtcBytes) {    // slave may send less than requested 
    currentTime[i++] = Wire.read(); 
  }
  
  // calculate time (hour, day, month are global for use with file names)
  second = 10 * (currentTime[0] >> 4) + (currentTime[0] & 0b00001111); 
  minute = 10 * (currentTime[1] >> 4) + (currentTime[1] & 0b00001111);
  hour = 10 * (currentTime[2] >> 4) + (currentTime[2] & 0b00001111);
  day = 10 * (currentTime[4] >> 4) + (currentTime[4] & 0b00001111);
  month = 10 * (currentTime[5] >> 4) + (currentTime[5] & 0b00001111);
  year = 10 * (currentTime[6] >> 4) + (currentTime[6] & 0b00001111);
  
  // print time to timeString
  sprintf(timeString, "%2.2d/%2.2d/%2.2d %2.2d:%2.2d:%2.2d", 
     year, month, day, hour, minute, second);
  
}

// Check bounds and set error LED
void displayError() {
  
  if (Vo > 1000 || Vo < 25) {
    digitalWrite(errorLED, !digitalRead(errorLED));
    
  }
  
   if (hall > 900 || hall < 130) {
    digitalWrite(errorLED, HIGH);
  }
  
}  

void collectData() {
 
   // heartbeat LED flash
  digitalWrite(heartbeatLED, HIGH); 
  
  // Read in analog pins (Hall Effect and Thermistor)
  hall = analogRead(hallPin);
  Vcc = analogRead(pVcc);            
  Vo = analogRead(pVo);            
  
  //  Read in IMU data (comment for testing without cables)
  lsm.read(); 
  
  // Read in current time       
  getTime();       
  
  // Check bounds and set errorLED
  displayError();


  // Build final output string
  sprintf(outputString, "%s\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d", 
     timeString,
     (int)lsm.accelData.x, (int)lsm.accelData.y, (int)lsm.accelData.z, 
    (int)lsm.magData.x, (int)lsm.magData.y, (int)lsm.magData.z, Vcc, Vo,  hall);

  // Print the output to Serial 
  Serial.println(outputString);

  // Set up file name and print data to file
  sprintf(fileName,"MT%2.2d%2.2d%2.2d.txt", month, day, hour);
  
  
  File dataFile = SD.open(fileName, O_CREAT | O_APPEND | O_WRITE);
  
  if (dataFile) {
    dataFile.println(outputString);
    
  }  
  // if the file isn't open, pop up an error:
  else {
    digitalWrite(errorLED, HIGH);
    Serial.println("error opening datalog!!!!.txt");
  }
  
  dataFile.close();
  
  delay(5);
  
  // heartbeat/delay (needs to switch to sleep)           
  digitalWrite(heartbeatLED, LOW);
}

// Wake up from sleep here
void pin2Interrupt(void)
{
  /* This will bring us back from sleep. */
  
  /* We detach the interrupt to stop it from 
   * continuously firing while the interrupt pin
   * is low.
   */
  detachInterrupt(0);
}


// Use this to go to sleep
void enterSleep(void)
{
  
  /* Setup pin2 as an interrupt and attach handler. */
  attachInterrupt(0, pin2Interrupt, LOW);
  delay(10);
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  
  sleep_enable();
  
  sleep_mode();
  
  /* The program will continue from here. */
  
  /* First thing to do is disable sleep. */
  sleep_disable(); 
}


// the setup routine runs once when you press reset
void setup() {
  
  // Heartbeat LED
  pinMode(heartbeatLED, OUTPUT);
  pinMode(errorLED, OUTPUT);
  pinMode(rtcInterrupt, INPUT);
  
  digitalWrite(heartbeatLED, HIGH);
  digitalWrite(errorLED, HIGH);
  
  // initialize serial communication 
  Serial.begin(57600);
  
  // Start communications with RTC/LSM
  Wire.begin(); 
  delay(50); // wait a bit 
   
  // Start IMU (this code will never ever run, stupid LSM library)
  if (!lsm.begin()) {
    Serial.println("Oops ... unable to initialize the LSM303. Check your wiring!");
    digitalWrite(errorLED, LOW);
    while (1);
  }
  
  // Start the RTC
  startTime();     
    
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    digitalWrite(errorLED, LOW);
    return;
  }
  
  Serial.println("Initialization Success");
  
  
  // Startup procedure block
  // Plug in battery, Upload time via the SD file, head to ocean
  // Attach sensors (no magnet), Press Reset #1, wait for red light to dim
  // Attach Magnet, Press Reset, good to go
  // On Failure to read SD, just go to main loop (mainly for debugging)
  
  if (SD.exists(errFile)) {
    
     if (!SD.exists(magFile)) {  


        while(1) {
          
          hall = analogRead(hallPin);
          if ( hall < 485 || hall > 560 ) {
            digitalWrite(errorLED, HIGH);
          } else {
            digitalWrite(errorLED, LOW);
          }
          
                    // Read in analog pins (Hall Effect and Thermistor)
          hall = analogRead(hallPin);
          Vcc = analogRead(pVcc);            
          Vo = analogRead(pVo);            
          
          //  Read in IMU data (comment for testing without cables)
          lsm.read(); 
          
          // Read in current time       
          getTime();       
          
       
        
          // Build final output string
          sprintf(outputString, "%s\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d\t%5d", 
             timeString,
             (int)lsm.accelData.x, (int)lsm.accelData.y, (int)lsm.accelData.z, 
            (int)lsm.magData.x, (int)lsm.magData.y, (int)lsm.magData.z, Vcc, Vo,  hall);
            
            File dataFile = SD.open(magFile, O_CREAT | O_APPEND | O_WRITE);
      
           if (dataFile) {
              dataFile.println(outputString);
            }  
            // if the file isn't open, pop up an error:
           else {
              digitalWrite(errorLED, LOW);
              Serial.println("Error opening Magnet log!");
           }
            
           dataFile.close();   
                  
          delay(5);
          
        }
      
     }
    
    
    
     getTime();
     
     File dataFile = SD.open(errFile, O_CREAT | O_APPEND | O_WRITE);
      
     if (dataFile) {
        dataFile.print("Reset pressed at ");
        dataFile.println(timeString);
      }  
      // if the file isn't open, pop up an error:
     else {
        digitalWrite(errorLED, LOW);
        Serial.println("Error opening Errorlog! How ironic");
     }
      
     dataFile.close();
     
  
  } else {  
    if (SD.exists(timeFile)) {
        
       File dataFile = SD.open(timeFile);
        
       if (dataFile) {
          year = dataFile.parseInt();
          month = dataFile.parseInt();
          day = dataFile.parseInt();
          hour = dataFile.parseInt();
          minute = dataFile.parseInt();
          second = dataFile.parseInt();
       }  
        // if the file isn't open, pop up an error:
       else {
          digitalWrite(errorLED, LOW);
          Serial.println("error opening time file");
       }
        
       dataFile.close();
        
        
        
        sprintf(timeString, "%2.2d/%2.2d %2.2d:%2.2d:%2.2d", 
            month, day, hour, minute, second);
            
        Serial.println(timeString);
        
        delay(10);
        
        Wire.beginTransmission(0x68);                              
        Wire.write(byte(0x00));
  
        Wire.write(byte( (second / 10) << 4 | (second % 10) ));
        Wire.write(byte( (minute / 10) << 4 | (minute % 10) ));
        Wire.write(byte( (hour / 10) << 4 | (hour % 10) ));
        Wire.write(byte( 0x00 ));
        Wire.write(byte( (day / 10) << 4 | (day % 10) ));
        Wire.write(byte( (month / 10) << 4 | (month % 10) ));
        Wire.write(byte( (year / 10) << 4 | (year % 10) ));
                                
        Wire.endTransmission(); 
        
        delay(10);
        
       dataFile = SD.open(errFile, O_CREAT | O_APPEND | O_WRITE);
        
       if (dataFile) {
          dataFile.print("Unit started at ");
          dataFile.println(timeString);
        }  
        // if the file isn't open, pop up an error:
       else {
          digitalWrite(errorLED, LOW);
          Serial.println("Error opening Errorlog! How ironic");
       }
        
       dataFile.close();
      
    } else {
        Serial.println("No Start Time detected, halting");
        while(1);
    }
    
  }

  
   digitalWrite(errorLED, !digitalRead(errorLED));
  
}

// the loop routine runs over and over again forever:
void loop() {
  
  collectData();
  
  while( !digitalRead(2)) {
      delay(1);
  }
  
  collectData();
  Serial.print("Entering sleep  ");
  enterSleep();
  Serial.println("Waking");
  // Use this if you don't want to sleep
  //while( digitalRead(2)) {
  //    delay(1);
  //}
  
}
