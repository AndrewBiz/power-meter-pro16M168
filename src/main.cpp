#include <Arduino.h>
#include <Wire.h>
#include <INA219.h>
#include <U8x8lib.h>
#include <SPI.h>
#include "PinButton.h"

#define OLED_RESET 4

Adafruit_INA219 ina219;
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8;

const uint8_t modeButtonPin = 2;
PinButton modeButton(modeButtonPin);

unsigned long previousMillis = 0;
const unsigned int interval = 200;
unsigned long currentMillis = 0;
float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float loadvoltage = 0;
float power = 0;
float energy = 0;
uint8_t ovf = 0;
uint8_t cnvr = 0;

void displaydata() {
  char _float_buf[9];
  // u8x8.setFont(u8x8_font_pressstart2p_f);
  u8x8.setFont(u8x8_font_artossans8_r);
  // lines 0-1 Voltmeter
  u8x8.setCursor(0, 0);
  u8x8.print(F("V:"));
  dtostrf(loadvoltage, 7, ina219.getVbusDigitsAfterPoint(), _float_buf);  /* 8 is mininum width, 2 is precision */
  u8x8.draw2x2String(2, 0, _float_buf);

  // lines 2-4 Ampermeter
  u8x8.setCursor(0, 2);
  u8x8.print(F("mA ("));
  u8x8.print(ina219.getRange());
  u8x8.print(F("):"));
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
  u8x8.begin();
  ina219.begin();
}

void loop() {
  currentMillis = millis();
  modeButton.update();

  if (modeButton.isSingleClick()) {
    ina219.setNextRange();
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
