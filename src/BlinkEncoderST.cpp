#include <Arduino.h>
// #include "wiring_private.h"
#include <EEPROM.h>
#include <GyverEncoder.h>
#include <U8g2lib.h>
#include <Wire.h>


enum TMode {
	modeGuage,
	modeGraph,
						
	modeOscil
};
TMode mode = modeGuage;




constexpr byte buttonMode = PB5;
constexpr byte pinEnc1 = PB3;
constexpr byte pinEnc2 = PB4;

constexpr byte pinAnNumber = PA6;
constexpr byte pinAnBrightness = PA7;
constexpr byte pinAnPhoto = PA3;

constexpr byte pinEnablePhoto = PA2;
constexpr byte pinEnableDebug = PB12;

byte pinAnToRead = pinAnPhoto;


bool buttonState = false;
unsigned long lastTimeRead = 0;
unsigned long lastTimeBlink = 0;
unsigned long lastTimeOscill = 0;
unsigned long delayBlink;
unsigned long delayBlinkOn;
unsigned long delayBlinkOff;

unsigned long delayMainLoop = 1;
unsigned long delayOscill = 50;
bool ledState;
byte lastADCChannel = 0;
bool enableWriteCom;

constexpr byte GuageStartX 	= 4;
constexpr byte GuageEndX 		= 128 - 4;																																						 

Encoder enc1(pinEnc1, pinEnc2, buttonMode);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display

void printMode();

void isrCLK() {
	enc1.tick();  
}

void isrDT() {
	enc1.tick();  
}

void setup() {
	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	//	pinMode(buttonMode, INPUT_PULLUP);
	pinMode(pinEnablePhoto, INPUT_PULLUP);
	pinMode(pinEnableDebug, INPUT_PULLUP);

	pinMode(pinAnNumber, INPUT_ANALOG);
	pinMode(pinAnBrightness, INPUT_ANALOG);  
	pinMode(pinAnPhoto, INPUT_ANALOG);



	Serial.begin(2000000);
	enc1.setType(TYPE2);

	mode = static_cast<TMode>( (byte)EEPROM.read(0));
	

	u8g2.begin();

	u8g2.clearBuffer();
	u8g2.setFont(u8g2_font_6x10_tf);
	u8g2.setCursor(0, 10);
	printMode();
	u8g2.sendBuffer();

// отработка энкодера в прерывании
	attachInterrupt(digitalPinToInterrupt(pinEnc1), isrCLK, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pinEnc2), isrDT, CHANGE);
	
}



int Number, LastNumber, LastBrightness;

void loop() {
	int Brightness;

	enc1.tick();

//  bool enableWriteComChanged = false;
//  if (enc1.isHold())
//  {
//    enableWriteCom = !enableWriteCom;
//    enableWriteComChanged = true;
//  }

	unsigned long curTime = micros();
	if (curTime - lastTimeRead >= 50000) {
		lastTimeRead = curTime;


		if (enc1.isClick() ) {
			//mode++;
			mode = static_cast<TMode>( (mode) + 1);
			if (mode > modeOscil) mode = modeGuage;

			EEPROM.update(0, mode);
			
			u8g2.clearBuffer();
			u8g2.setFont(u8g2_font_6x10_tf);
			u8g2.setCursor(0, 10);
			printMode();
			u8g2.sendBuffer();
			
			digitalWrite(LED_BUILTIN, HIGH);
		}


		enableWriteCom = !digitalRead(pinEnableDebug);
		LastNumber = Number;
														

		if (enc1.isLeft()) {
			Number++;
//      Serial.print(F("      <--enc1"));
		}
		if (enc1.isRight()) {
			Number--;
//      Serial.print(F("         enc1-->"));
		}
		
		if (digitalRead(pinEnablePhoto))
			pinAnToRead = (pinAnPhoto);
		else
			pinAnToRead = (pinAnBrightness);

			
		if (mode != modeOscil) {
			Number = constrain(Number, 1, 10);
			

			{
				Brightness = analogRead(pinAnToRead);
				lastADCChannel = pinAnToRead;
			}
			

			constexpr int16_t MAX_ADC_VALUE = bit(12) - 1;
			constexpr int16_t ADC_THRESHOLD = MAX_ADC_VALUE * 0.03;


			
			bool BrightnessChanged = abs(Brightness - LastBrightness) > ADC_THRESHOLD;

			if (mode == modeGuage){
				u8g2.clearBuffer();
				u8g2.setFont(u8g2_font_6x10_tf);

				u8g2.setCursor(0, 10);
																
				printMode();
																	 

				u8g2.setCursor(0, 25);
				u8g2.print(F("Number="));
				u8g2.setCursor(50, 25);
				u8g2.print(Number);

				u8g2.setCursor(70, 25);
				u8g2.print(F("ADC="));
				u8g2.setCursor(100, 25);
				u8g2.print(Brightness);

				constexpr byte mul = MAX_ADC_VALUE / 127;

				byte GuageValueX =  Brightness / mul; 

 				u8g2.setCursor(0, 40);
				u8g2.print(F("Guage="));
				u8g2.setCursor(50, 40);
				u8g2.print(GuageValueX);

  			u8g2.drawFrame(100, 40, 3, 3);
	  		u8g2.drawBox(110, 40, 3, 3);			 

  			u8g2.drawFrame(0, 50, 128, 15);
	  		u8g2.drawBox(0, 50, GuageValueX, 15);

     

				u8g2.sendBuffer();	
							
				LastBrightness = Brightness;
			}

		}
	
		if (mode == modeOscil)
		if ((Number != LastNumber) ) {
//      enableWriteComChanged = false;
			Number = constrain(Number, 1, 50);
			delayOscill = Number;
			
			u8g2.clearBuffer();
			u8g2.setFont(u8g2_font_6x10_tf);

			u8g2.setCursor(0, 10);


			printMode();

			u8g2.setCursor(0, 25);
			u8g2.print(F("Period="));
			u8g2.setCursor(50, 25);
			u8g2.print(delayOscill);
				
			if (enableWriteCom){
				u8g2.setCursor(0, 55);
				u8g2.print(F("Transmit"));  
			}    
														 

			u8g2.sendBuffer();        
		}

	}

	if (mode == modeOscil) {
		curTime = micros();

		word value = 0;

		if (curTime - lastTimeOscill >= delayOscill) {
			lastTimeOscill = curTime;

			{
				value = analogRead(pinAnToRead);
				lastADCChannel = pinAnToRead;
			}


			if (enableWriteCom) {
				//				Serial.println(value);

				// ADC bits: 0 0 0 0  0 0 1 2 , 3 4 5 6  7 8 9 10
				byte hbyte = (value >> 7) | bit(7); // in high byte: (sign of hbyte = 1) 0 0 0  0 1 2 3  
				byte lbyte = value & 0b01111111;    // in low  byte: (sign of lbyte = 0) 4 5 6  7 8 9 10
				Serial.write(hbyte);
				Serial.write(lbyte);

//        byte hbyte = (value >> 4);
//        Serial.write(hbyte);        

				//				if (value > 1000) Serial.write("1");
				//				if (value > 100) Serial.write("2");
				//				if (value > 10) Serial.write("3");
				//				Serial.write("4");
				//				Serial.println();
			}
		}
	}

}

void printMode() {
	switch (mode) {
		case modeGuage:
			u8g2.print(F("mode Guage")); break;
		case modeGraph:
			u8g2.print(F("mode Graph")); break;
		case modeOscil:
			u8g2.print(F("mode Oscil")); break;
	}
}



