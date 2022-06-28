#include <Arduino.h>
// #include "wiring_private.h"
#include <EEPROM.h>
#include <GyverEncoder.h>
#include <U8g2lib.h>
#include <Wire.h>

enum class TMode {
	Guage,
	Graph,
					
	Oscil
};
TMode mode = TMode::Guage;


constexpr byte pinEncButton = PB5;
constexpr byte pinEnc1 = PB3;
constexpr byte pinEnc2 = PB4;

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

constexpr int16_t MAX_ADC_VALUE = bit(12) - 1;

Encoder enc1(pinEnc1, pinEnc2, pinEncButton);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C display(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
//U8G2_SSD1306_128X64_NONAME_F_SW_I2C display(U8G2_R0, /* clock=*/ SCL, /* data=*/ SDA, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display

constexpr byte display_width 	= 128;
constexpr byte display_height 	= 64;	
	const byte yOffset = 16;

byte ADC_buffer[display_width];
byte ADC_buffer_start = 0;

void printMode();
void printFPS();

void isrCLK() {
	enc1.tick();  
}

void isrDT() {
	enc1.tick();  
}

void setup() {
	// initialize digital pin LED_BUILTIN as an output.
	pinMode(LED_BUILTIN, OUTPUT);
	//	pinMode(pinEncButton, INPUT_PULLUP);
	pinMode(pinEnablePhoto, INPUT_PULLUP);
	pinMode(pinEnableDebug, INPUT_PULLUP);

	pinMode(pinAnBrightness, INPUT_ANALOG);  
	pinMode(pinAnPhoto, INPUT_ANALOG);

	Serial.begin(1000000);
	enc1.setType(TYPE2);

	mode = static_cast<TMode>( (byte)EEPROM.read(0));

	display.begin();

	display.clearBuffer();
	display.setFont(u8g2_font_6x10_tf);
	display.setCursor(0, 10);
	printMode();
	display.sendBuffer();

// отработка энкодера в прерывании
	attachInterrupt(digitalPinToInterrupt(pinEnc1), isrCLK, CHANGE);
	attachInterrupt(digitalPinToInterrupt(pinEnc2), isrDT, CHANGE);
//	Timer1.setPeriod(1000);
//	Timer1.attachInterrupt();
}

void drawGrid() 
{
	for (byte y = yOffset; y <= display_height; y += 12) 
	{
		if (y == display_height)
			y = display_height - 1;
		
		// display.drawHVLine(0, y, display_width, 0);
		for (byte x = 0; x < display_width; x+=2)
			display.drawPixel(x, y);
		
	}

	for (byte x = 0; x <= display_width; x += 16) 
	{
		if (x == display_width)
			x = display_width - 1;

		// display.drawHVLine(x, yOffset, display_height, 1);
		for (byte y = yOffset; y < display_height; y+=2)
			display.drawPixel(x, y);
	}
}

void drawGraph() 
{
	byte prevX = 0, prevY = 0;
	bool first = true;
	for (byte x = 0; x < display_width; x++) 
	{
		byte pos = (x + ADC_buffer_start) % display_width;

		byte value = ADC_buffer[pos];
		byte y = display_height - value - 1;

		if (first) 
		{
			prevX = x;
			prevY = y;
			first = false;
		}
		display.drawLine(prevX, prevY, x, y);
		prevX = x;
		prevY = y;
	}

}


int Number, LastNumber, lastAnalogValue;
unsigned long lastFpsTime = 0;
uint16_t fps = 0, frames = 0;

void loop() 
{
	int analogValue;

	enc1.tick();

//  bool enableWriteComChanged = false;
//  if (enc1.isHold())
//  {
//    enableWriteCom = !enableWriteCom;
//    enableWriteComChanged = true;
//  }

	unsigned long curTime = micros();
	if (curTime - lastTimeRead >= 50000) 
	{
		lastTimeRead = curTime;


		if (enc1.isClick() ) 
		{
			//mode++;
			mode = static_cast<TMode>( static_cast<int>(mode) + 1);
			if (mode > TMode::Oscil) mode = TMode::Guage;

			EEPROM.update(0, static_cast<int>(mode));
			
			display.clearBuffer();
			printMode();
			display.sendBuffer();
			
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
			
		if (mode != TMode::Oscil) 
		{
			Number = constrain(Number, 1, 10);
			
			{
				analogValue = analogRead(pinAnToRead);
				lastADCChannel = pinAnToRead;
			};

			constexpr unsigned long graph_height = display_height - yOffset - 1;
			ADC_buffer[ADC_buffer_start] = (analogValue * graph_height) / MAX_ADC_VALUE;
			ADC_buffer_start++;
			if (ADC_buffer_start >= display_width)
				ADC_buffer_start = 0;


			if (mode == TMode::Guage)
			{
				display.clearBuffer();

				printMode();
				printFPS();

				display.setCursor(0, 25);
				display.print(F("Number="));
				display.setCursor(50, 25);
				display.print(Number);

				display.setCursor(70, 25);
				display.print(F("ADC="));
				display.setCursor(100, 25);
				display.print(analogValue);

				byte GuageValueX = (analogValue * 126ul) / MAX_ADC_VALUE; 

				display.drawFrame(0, 48, display_width, 15);
				display.drawBox(1, 48, GuageValueX, 15);



				display.sendBuffer();

							
				lastAnalogValue = analogValue;
			}

			if (mode == TMode::Graph)
			{
				display.clearBuffer();

				printMode();
				printFPS();
		
				drawGrid();
				drawGraph(); 

				display.sendBuffer();
			}	

			frames++;
			unsigned long curTime = micros();
			if (curTime - lastFpsTime >= 1000000) 
			{
				fps = frames;
				frames = 0;
				lastFpsTime = curTime;
			}


		}
	
		if (mode == TMode::Oscil)
		if ((Number != LastNumber) ) 
		{
//      enableWriteComChanged = false;
			Number = constrain(Number, 1, 50);
			delayOscill = Number;
			
			display.clearBuffer();

			printMode();

			display.setCursor(0, 25);
			display.print(F("Period="));
			display.setCursor(50, 25);
			display.print(delayOscill);
				
			if (enableWriteCom)
			{
				display.setCursor(0, 55);
				display.print(F("Transmit"));  
			}    
														 

			display.sendBuffer();        
		}

	}

	if (mode == TMode::Oscil) 
	{
		curTime = micros();

		word value = 0;

		if (curTime - lastTimeOscill >= delayOscill) 
		{
			lastTimeOscill = curTime;

			{
				value = analogRead(pinAnToRead);
				lastADCChannel = pinAnToRead;
			}


			if (enableWriteCom) 
			{
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

void printMode() 
{
	display.setFont(u8g2_font_6x10_tf);
	display.setCursor(0, 10);

	switch (mode) 
	{
		case TMode::Guage:
			display.print(F("Guage")); break;
		case TMode::Graph:
			display.print(F("Graph")); break;
		case TMode::Oscil:
			display.print(F("Oscil to PC5")); break;
	}
}

void printFPS() 
{
	display.setCursor(85, 10);
	display.print(F("fps="));
	byte x = 106;
	if (fps < 100) x += 6;
	display.setCursor(x, 10);
	display.print(fps);
}
