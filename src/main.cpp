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

const uint8_t LEDPin = 13;

enum {VA_METER, TO_POWER_METER, POWER_METER, TO_VA_METER};
uint8_t mode = VA_METER; //The current mode

unsigned long timeNow = 0;
unsigned long timePrevious = 0;
const unsigned int interval = 200;

float shuntvoltage = 0;
float busvoltage = 0;
float current_mA = 0;
float power = 0;
float energy = 0;
uint8_t ovf = 0;
uint8_t cnvr = 0;

/*****************************************************************************
   Reading INA219 values
*****************************************************************************/
void read_ina219() {
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power = ina219.getPower_mW();
  energy += power * (timeNow - timePrevious) / 3600000;
  ovf = ina219.getOVF();
  cnvr = ina219.getCNVR();
}

/*****************************************************************************
   Debugging values to Serial
*****************************************************************************/
#ifdef DEBUG
void debugdata() {
  Serial.print(F("Timestamp:     ")); Serial.print(timeNow); Serial.println(F(" ms"));
  Serial.print(F("Bus Voltage:   ")); Serial.print(busvoltage); Serial.println(F(" V"));
  Serial.print(F("Shunt Voltage: ")); Serial.print(shuntvoltage); Serial.println(F(" mV"));
  Serial.print(F("Current:       ")); Serial.print(current_mA); Serial.println(F(" mA"));
  Serial.print(F("Power:         ")); Serial.print(power); Serial.println(F(" mW"));
  Serial.print(F("Energy burned: ")); Serial.print(energy); Serial.println(F(" mWh"));
  Serial.print(F("OVF:           ")); Serial.println(ovf);
  Serial.print(F("CNVR:          ")); Serial.println(cnvr);
  Serial.println("");
}
#endif

/*****************************************************************************
   Display in diff modes
*****************************************************************************/
void clear_display() {
  u8x8.clear();
}

void displaydata_VA_METER() {
  u8x8.setFont(u8x8_font_artossans8_r);
  // u8x8.setFont(u8x8_font_pxplustandynewtv_f);

  // Voltmeter
  u8x8.setCursor(0, 0);
  u8x8.print(F("V:"));
  char _float_buf1[9];
  dtostrf(busvoltage, 7, ina219.getVbusDigitsAfterPoint(), _float_buf1);  /* 8 is mininum width, 2 is precision */
  u8x8.draw2x2String(2, 0, _float_buf1);

  // Ampermeter
  u8x8.setCursor(0, 2);
  u8x8.print(F("mA:"));
  if (ovf) {
    u8x8.setInverseFont(1);
  }
  dtostrf(current_mA, 8, ina219.getCurrentDigitsAfterPoint(), _float_buf1);  /* 8 is mininum width, 2 is precision */
  u8x8.draw2x2String(0, 3, _float_buf1);
  u8x8.setInverseFont(0);

  // Scale info
  u8x8.setCursor(0, 6);
  u8x8.print(F("scale:"));
  char _float_buf2[6];
  dtostrf(ina219.getCurrentOverflow(), 5, 3, _float_buf2);
  char _float_buf3[5];
  dtostrf(ina219.getCurrentLSB(), 4, 2, _float_buf3);
  u8x8.setCursor(0, 7);
  u8x8.print(F(" "));
  u8x8.print(_float_buf2);
  u8x8.print(F("A  "));
  u8x8.print(_float_buf3);
  u8x8.print(F("mA"));
}

void displaydata_POWER_METER() {
  // u8x8.setFont(u8x8_font_pressstart2p_f);
  u8x8.setFont(u8x8_font_artossans8_r);
  // Voltmeter
  u8x8.setCursor(0, 0);
  u8x8.print(F("Vbus:"));
  char _float_buf8[9];
  dtostrf(busvoltage, 8, 3, _float_buf8);
  u8x8.print(_float_buf8);
  u8x8.print(F(" V"));

  u8x8.setCursor(0, 1);
  u8x8.print(F("Vsh:"));
  dtostrf(shuntvoltage, 8, 2, _float_buf8);
  u8x8.print(_float_buf8);
  u8x8.print(F(" mV"));

  // Ampermeter
  u8x8.setCursor(0, 3);
  u8x8.print(F("Ish:"));
  if (ovf) {
    u8x8.setInverseFont(1);
  }
  dtostrf(current_mA, 8, ina219.getCurrentDigitsAfterPoint(), _float_buf8);
  u8x8.print(_float_buf8);
  u8x8.print(F(" mA"));
  u8x8.setInverseFont(0);

  // Scale info
  u8x8.setCursor(0, 4);
  u8x8.print(F("("));
  char _float_buf5[6];
  dtostrf(ina219.getCurrentOverflow(), 5, 3, _float_buf5);
  char _float_buf4[5];
  dtostrf(ina219.getCurrentLSB(), 4, 2, _float_buf4);
  u8x8.print(_float_buf5);
  u8x8.print(F("A, "));
  u8x8.print(_float_buf4);
  u8x8.print(F("mA)"));

  // power
  u8x8.setCursor(0, 6);
  u8x8.print(F("Pwr:"));
  dtostrf(power, 8, 1, _float_buf8);
  u8x8.print(_float_buf8);
  u8x8.print(F(" mW"));

  // energy
  u8x8.setCursor(0, 7);
  u8x8.print(F("Enr:"));
  dtostrf(energy, 8, 1, _float_buf8);
  u8x8.print(_float_buf8);
  u8x8.print(F(" mWh"));
}

void displaydata_RAW_MODE() {
  // u8x8.setFont(u8x8_font_pressstart2p_f);
  u8x8.setFont(u8x8_font_artossans8_r);
  // Voltmeter
  u8x8.setCursor(0, 0);
  u8x8.print(F("Vbus:"));
  char _float_buf8[9];
  dtostrf(busvoltage, 8, 3, _float_buf8);
  u8x8.print(_float_buf8);
  u8x8.print(F(" V"));

  u8x8.setCursor(0, 1);
  u8x8.print(F("Vsh:"));
  dtostrf(shuntvoltage, 8, 2, _float_buf8);
  u8x8.print(_float_buf8);
  u8x8.print(F(" mV"));

  // lines 2-4 Ampermeter
  u8x8.setCursor(0, 2);
  u8x8.print(F("Ish:"));
  if (ovf) {
    u8x8.setInverseFont(1);
  }
  dtostrf(current_mA, 8, ina219.getCurrentDigitsAfterPoint(), _float_buf8);
  u8x8.print(_float_buf8);
  u8x8.print(F(" mA"));
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

/*****************************************************************************
   init devices
*****************************************************************************/
void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    Serial.println(F("Start debugging output ..."));
  #endif
  pinMode(LED_BUILTIN, OUTPUT);
  u8x8.begin();
  ina219.begin();
}

/*****************************************************************************
   main loop
*****************************************************************************/
void loop() {
  timeNow = millis();
  modeButton.update();

  switch (mode) {

    case VA_METER: // main working mode: big V and A readings
      if (modeButton.isSingleClick()) {
        ina219.setNextScale();
      } else if (modeButton.isLongClick()) {
        digitalWrite(LED_BUILTIN, HIGH);
        mode = TO_POWER_METER;
      }
      if (timeNow - timePrevious >= interval) {
        read_ina219();
        displaydata_VA_METER();
        timePrevious = timeNow;
      }
      break;

    case TO_POWER_METER:  //transition to the next mode
      if (modeButton.isReleased()) {
        digitalWrite(LED_BUILTIN, LOW);
        clear_display();
        mode = POWER_METER;
      }
      break;

    case POWER_METER: // power meter mode
      if (modeButton.isSingleClick()) {
        ina219.setNextScale();
      } else if (modeButton.isLongClick()) {
        digitalWrite(LED_BUILTIN, HIGH);
        mode = TO_VA_METER;
      }
      if (timeNow - timePrevious >= interval) {
        read_ina219();
        displaydata_POWER_METER();
        timePrevious = timeNow;
      }
      break;

    case TO_VA_METER:  //transition to the next mode
      if (modeButton.isReleased()) {
        digitalWrite(LED_BUILTIN, LOW);
        clear_display();
        mode = VA_METER;
      }
      break;

    default:
      break;
  }

  #ifdef DEBUG
    debugdata();
    delay(200);
  #endif
}
