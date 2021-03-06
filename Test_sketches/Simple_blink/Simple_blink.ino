/*
	Tested and functional on MusselTracker board v2
	uploaded via FTDI usb serial chip after burning a
	copy of ATmegaBOOT_168_atmega328_pro_8MHz.hex to the
	chip using the profile laid out in my boards.txt entry
	"Mussel Tracker v2 (8 MHz internal clock)"
	
	LPM 2015-05-05

*/

const int heartbeatLED = 6;
const int errorLED = 5;

void setup(void){
	Serial.begin(57600);
	Serial.println("Hello world");
	pinMode(heartbeatLED, OUTPUT);
	pinMode(errorLED, OUTPUT);
	digitalWrite(heartbeatLED, HIGH);
}

void loop(void) {
	digitalWrite(heartbeatLED, !digitalRead(heartbeatLED));
	digitalWrite(errorLED, !digitalRead(errorLED));
	delay(1000);
}
