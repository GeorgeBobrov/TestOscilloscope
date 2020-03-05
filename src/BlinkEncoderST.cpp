#include <Arduino.h>
#include "wiring_private.h"
#include <EEPROM.h>
#include <GyverEncoder.h>
#include <U8g2lib.h>
#include <Wire.h>

enum TMode {
  mode7seg,
  modeLedPWM10,
  modeLedPWM100,
  modeSound,
  modeOscil
};
TMode mode = mode7seg;




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

word notes[] = {523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988,};

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



  Serial.begin(500000);
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




int Number;

void loop() {
  int Brightness;

  enc1.tick();

  unsigned long curTime = micros();
  if (curTime - lastTimeRead >= delayMainLoop) {
    lastTimeRead = curTime;


    if (enc1.isClick()) {
      //mode++;
      mode = static_cast<TMode>( (mode) + 1);
      if (mode > modeOscil) mode = mode7seg;

      EEPROM.update(0, mode);

      delayMainLoop = 50000;


      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0, 10);
      printMode();
      u8g2.sendBuffer();
      
    }


    enableWriteCom = !digitalRead(pinEnableDebug);

    if (mode != modeOscil) {


      if (enc1.isLeft()) {
        Number++;
//        Serial.print(F("      <--enc1"));
      }
      if (enc1.isRight()) {
        Number--;
//        Serial.print(F("         enc1-->"));
      }

      if (Number < 0)
        Number = 0;

      if (mode == modeSound)
      {
        Number = constrain(Number, 0, 11);

        word note = notes[Number];

        delayBlink = (1000000UL / note);

        delayBlinkOn = delayBlink / 2;
        delayBlinkOff = delayBlinkOn;
      }
      else
      {
        Number = constrain(Number, 1, 19);
      }

      if (digitalRead(pinEnablePhoto))
        pinAnToRead = (pinAnPhoto);
      else
        pinAnToRead = (pinAnBrightness);


      {
        Brightness = analogRead(pinAnToRead);
        lastADCChannel = pinAnToRead;
      }

constexpr int16_t MAX_ADC_VALUE = bit(12) - 1;
 

      if ((Number != 0) && (mode != modeSound)) {
        if (mode == modeLedPWM10)
          delayBlink = (1000000UL / Number);
        if (mode == modeLedPWM100)
          delayBlink = (100000UL / Number);

        delayBlinkOn = delayBlink * Brightness / 1023;
        delayBlinkOff = delayBlink - delayBlinkOn;
      }

      if (enableWriteCom) {
        if ((mode != modeOscil))
        {
          Serial.print(F("Number="));
          Serial.print(Number);

          Serial.print(F("\tBrightness="));
          Serial.print(Brightness);

          Serial.print(F("\tdelayBlink="));
          Serial.print(delayBlink);

          Serial.print(F("\tOn="));
          Serial.print(delayBlinkOn);

          Serial.print(F("\tOff="));
          Serial.println(delayBlinkOff);
        }
      }



      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_6x10_tf);

      u8g2.setCursor(0, 10);


      printMode();

      u8g2.setCursor(0, 25);
      u8g2.print(F("Number="));
      u8g2.setCursor(50, 25);
      u8g2.print(Number);

      u8g2.setCursor(0, 40);
      u8g2.print(F("Brightness="));
      u8g2.setCursor(80, 40);
      u8g2.print(Brightness);

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

        //				if (value > 1000) Serial.write("1");
        //				if (value > 100) Serial.write("2");
        //				if (value > 10) Serial.write("3");
        //				Serial.write("4");
        //				Serial.println();
      }
    }
  }

  if (((mode == modeLedPWM10) || (mode == modeLedPWM100) || (mode == modeSound))) {

    if (ledState &&  (curTime - lastTimeBlink > delayBlinkOn)) {
      lastTimeBlink = curTime;
      digitalWrite(LED_BUILTIN, LOW);
      ledState = LOW;
    }

    if (!ledState &&  (curTime - lastTimeBlink > delayBlinkOff)) {
      lastTimeBlink = curTime;
      digitalWrite(LED_BUILTIN, HIGH);
      ledState = HIGH;
    }
  }


}

void printMode() {
  switch (mode) {
    case mode7seg:
      u8g2.print(F("mode 7seg")); break;
    case modeLedPWM10:
      u8g2.print(F("mode Led PWM 10")); break;
    case modeLedPWM100:
      u8g2.print(F("mode Led PWM 100")); break;
    case modeOscil:
      u8g2.print(F("mode Oscil")); break;
  }
}



