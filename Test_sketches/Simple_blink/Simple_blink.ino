/*
	Tested and functional on MusselTracker board v1
	uploaded via FTDI usb serial chip after burning a
	copy of ATmegaBOOT_168_atmega328_pro_8MHz.hex to the
	chip using the profile laid out in my boards.txt entry
	"Mussel Tracker2 (328P @ 8 MHz external, 3.3V ATmegaBOOT)"
	
	LPM 2015-05-05

*/

const int heartbeatLED = 8;
const int errorLED = 9;

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