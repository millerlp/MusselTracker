/*
	For the Mussel Tracker v2 RevB boards, 
	Hall effect board 1 sleep line is Analog 2 (PC2), 
	analog input line is Analog 6 (ADC6).
	Hall effect board 2 sleep line is Analog 3 (PC3),
	analog input line is Analog 7 (ADC7).
	
	Pull sleep lines LOW to put the chip to sleep

*/

long elapsed;
int myDelay = 500;
int read1; // hold A-D value
int hall1; // hold A-D value for hall effect sensor 1
int hall2; // hold A-D value for hall effect sensor 2


void setup() {
	pinMode(A2, OUTPUT); // sleep line for hall effect 1
	digitalWrite(A2, LOW);
	pinMode(A3, OUTPUT); // sleep line for hall effect 2
	digitalWrite(A3, LOW);
	pinMode(A6, INPUT);
	pinMode(A7, INPUT);
  
	Serial.begin(57600);
	Serial.println("Hello");
	
	elapsed = millis(); // initialize the elapsed variable


}

void loop() {
  // put your main code here, to run repeatedly:
	if (millis() > elapsed + myDelay){
		elapsed = millis();
		digitalWrite(A2, HIGH); // wake Hall Effect sensor 1
		delay(1); // give 1ms delay to wake
		analogRead(A6); // throw-away first reading
		hall1 = 0; // always reset the hall1 value to zero before A-D reads
		for (byte i = 0; i < 4; i++) {
			read1 = analogRead(A6);
			Serial.print(read1);
			Serial.print("\t");
			hall1 = hall1 + read1;
			delay(5); // short delay
		}
		// hall1 should now hold the sum of 4 analog reads.
		// Do a 2-position right shift to divide that value
		// by 4 to get the average.
		hall1 = hall1 >> 2; // right-shift 2 bits
		Serial.println(hall1); // read Hall Effect sensor 1
		digitalWrite(A2, LOW); // turn hall effect back off
	}
}
