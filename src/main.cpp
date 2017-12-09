#include <Arduino.h>
#include <Wire.h>
#include <INA219.h>
#include <U8x8lib.h>
#include <SPI.h>
// #include "SdFat.h"
// SdFat SD;

#define OLED_RESET 4
Adafruit_INA219 ina219;
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8;

const uint8_t buttonPin = 2;     // the number of the pushbutton pin
const uint8_t ledPin =  13;      // the number of the LED pin
const uint16_t LONG_KEY_PRESS_INTERVAL = 1000; //1 sec is considered long keypress

unsigned long previousMillis = 0;
const unsigned int interval = 100;
const uint8_t chipSelect = 10;
const uint8_t StrBufSize8 = 9;
unsigned long currentMillis = 0;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power = 0;
float energy = 0;
uint8_t ovf = 0;
uint8_t cnvr = 0;

uint8_t state_btn_pressed = 0;
uint32_t time_btn_pressed = 0;

void displaydata() {
  char _float_buf[9];
  // char _str_buf8[StrBufSize8];
  // memset(_str_buf8, '\0', StrBufSize8);

  // u8x8.setFont(u8x8_font_pressstart2p_f);
  u8x8.setFont(u8x8_font_artossans8_r);
  // lines 0-1 Voltmeter
  u8x8.setCursor(0, 0);
  u8x8.print(F("V:"));
  dtostrf(loadvoltage, 7, ina219.getVbusDigitsAfterPoint(), _float_buf);  /* 8 is mininum width, 2 is precision */
  // snprintf(_str_buf8, StrBufSize8, "=%4.2f V=", 15.33L);
  u8x8.draw2x2String(2, 0, _float_buf);

  // lines 2-4 Ampermeter
  u8x8.setCursor(0, 2);
  u8x8.print(F("mA:"));
  if (ovf) {
    u8x8.setInverseFont(1);
  }
  dtostrf(current_mA, 8, ina219.getCurrentDigitsAfterPoint(), _float_buf);  /* 8 is mininum width, 2 is precision */
  u8x8.draw2x2String(0, 3, _float_buf);
  u8x8.setInverseFont(0);

  // line 5
  u8x8.setCursor(0, 5);
  u8x8.clearLine(5);
  u8x8.print(ina219.getCurrent_raw());

  // // line 5 bits
  // u8x8.setCursor(0, 5);
  // u8x8.clearLine(5);
  // u8x8.print(F("OVF: "));
  // u8x8.print(ovf);
  // u8x8.print(F("  CNVR: "));
  // u8x8.print(cnvr);

  // line 6, power
  u8x8.setCursor(0, 6);
  u8x8.clearLine(6);
  // u8x8.print(F("P: "));
  u8x8.print(power);
  u8x8.print(F(" mW"));

  // line 7, energy
  u8x8.setCursor(0, 7);
  // u8x8.print(F("E: "));
  u8x8.print(energy);
  u8x8.print(F(" mWh"));
}

#ifdef DEBUG
void debugdata() {
  Serial.print(F("Timestamp:     ")); Serial.print(currentMillis); Serial.println(F(" ms"));
  Serial.print(F("Bus Voltage:   ")); Serial.print(busvoltage); Serial.println(F(" V"));
  Serial.print(F("Shunt Voltage: ")); Serial.print(shuntvoltage); Serial.println(F(" mV"));
  Serial.print(F("Load Voltage:  ")); Serial.print(loadvoltage); Serial.println(F(" V"));
  Serial.print(F("Current:       ")); Serial.print(current_mA); Serial.println(F(" mA"));
  Serial.print(F("Power:         ")); Serial.print(power); Serial.println(F(" mW"));
  Serial.print(F("Energy burned: ")); Serial.print(energy); Serial.println(F(" mWh"));
  Serial.print(F("OVF:           ")); Serial.println(ovf);
  Serial.print(F("CNVR:          ")); Serial.println(cnvr);
  Serial.println("");
}
#endif

void ina219values() {
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power = ina219.getPower_mW();
  loadvoltage = busvoltage;
  energy += power / 3600;  //TODO wtf
  ovf = ina219.getOVF();
  cnvr = ina219.getCNVR();
}

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  Serial.println(F("Start debugging output ..."));
  #endif

  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  // SD.begin(chipSelect);
  u8x8.begin();
  ina219.begin();
}

void loop() {
  currentMillis = millis();

  // read the state of the pushbutton value:
  uint8_t buttonState = digitalRead(buttonPin);

  if (state_btn_pressed) { //key was being pressed in the last cycle
    if (buttonState == HIGH) {
      // the user keeps pressing the button
      // do smth special
    } else {
      // we have the key was pressed down and just released
      state_btn_pressed = 0;
      uint16_t btn_pressed_interval = currentMillis - time_btn_pressed;
      // time_btn_released = currentMillis;
      if (btn_pressed_interval < LONG_KEY_PRESS_INTERVAL) {
        // it was short key press
        #ifdef DEBUG
        Serial.println(F("BUTTON HAS BEEN SHORT PRESSED"));
        #endif
      } else {
        // it was long key press
        #ifdef DEBUG
        Serial.println(F("BUTTON HAS BEEN LONG PRESSED"));
        #endif
      }
    }
  } else { // no keys was pressed in the last cycle
    if (buttonState == HIGH) {
      state_btn_pressed = 1;
      time_btn_pressed = currentMillis;
    }
  }

  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  } else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    ina219values();
    displaydata();
    #ifdef DEBUG
    debugdata();
    delay(200);
    #endif
  }
}
