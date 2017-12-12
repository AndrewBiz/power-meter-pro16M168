/**************************************************************************/
/*!
    @file     Adafruit_INA219.cpp
    @author   K.Townsend (Adafruit Industries)
	@license  BSD (see license.txt)

	Driver for the INA219 current sensor

	This is a library for the Adafruit INA219 breakout
	----> https://www.adafruit.com/products/???

	Adafruit invests time and resources providing this open source code,
	please support Adafruit and open-source hardware by purchasing
	products from Adafruit!

	@section  HISTORY

    v1.0 - First release
    v2.0 modified by ANB
*/
/**************************************************************************/
#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#include "INA219.h"

const uint8_t NumberOfRangeSets = 4;

// calibration values arrays
// calculated for Rshunt = 0.1 Ohm
// Calculation Example:
// 1) Take desired CurrentLSB, say 0.00002 A (0.02 mA) // means 0.02mA per 1 bit of current_register value
// 2) calibration_value = trunc(4096/(100000*CurrentLSB*Rshunt)) = 20480
// 3) max_current = CurrentLSB * 32767 = 0.655A
// 4) Vshunt_max = 0.08V (using INA219_CONFIG_GAIN_2_80MV)
// 5) max_possible_current = Vshunt_max / Rshunt = 0.8A
// 6) Max_Current_Before_Overflow = min(max_current, max_possible_current) = 0.655A

const float ina219_range_set_f[NumberOfRangeSets][2] PROGMEM =
{
  // {0.007, 0.229}, // CurrentLSB (mA), Max_Current_Before_Overflow (A)
  {0.01, 0.327}, // CurrentLSB (mA), Max_Current_Before_Overflow (A)
  {0.02, 0.655}, // CurrentLSB (mA), Max_Current_Before_Overflow (A)
  {0.04, 1.311}, // CurrentLSB (mA), Max_Current_Before_Overflow (A)
  {0.1, 3.2}  // CurrentLSB (mA), Max_Current_Before_Overflow (A)
};

const uint16_t ina219_range_set_i[NumberOfRangeSets][3] PROGMEM =
{
  // {58514, 3, INA219_CONFIG_GAIN_1_40MV}, // calibration_value, currentDigitsAfterPoint, config_gain
  {40960, 2, INA219_CONFIG_GAIN_1_40MV}, // calibration_value, currentDigitsAfterPoint, config_gain (0-400mA)
  {20480, 2, INA219_CONFIG_GAIN_2_80MV}, // calibration_value, currentDigitsAfterPoint, config_gain (0-800mA)
  {10240, 2, INA219_CONFIG_GAIN_4_160MV}, // calibration_value, currentDigitsAfterPoint, config_gain (0-1,6A)
  {4096, 1, INA219_CONFIG_GAIN_8_320MV}  // calibration_value, currentDigitsAfterPoint, config_gain (0-3,2A)
};

/*!
    @brief  Instantiates a new INA219 class
*/
/**************************************************************************/
Adafruit_INA219::Adafruit_INA219(uint8_t addr, uint8_t range) {
  _i2caddr = addr;
  _range = range;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (defaults to 32V and 2A for calibration values)
*/
/**************************************************************************/
void Adafruit_INA219::begin(uint8_t addr, uint8_t range) {
  _i2caddr = addr;
  _range = range;
  begin();
}

void Adafruit_INA219::begin(void) {
  Wire.begin();
  _setConfig();
  _setCalibration();
}

/**************************************************************************/
/*!
    @brief  Sets config register
*/
/**************************************************************************/
void Adafruit_INA219::_setConfig() {
  _configValue =  INA219_CONFIG_BVOLTAGERANGE_32V |
                  pgm_read_word(&(ina219_range_set_i[_range][2])) |
                  // INA219_CONFIG_GAIN_8_320MV |
                  //
                  INA219_CONFIG_BADCRES_12BIT |
                  //
                  // INA219_CONFIG_SADCRES_12BIT_1S_532US |
                  // INA219_CONFIG_SADCRES_12BIT_64S_34MS |
                  INA219_CONFIG_SADCRES_12BIT_16S_8510US |
                  // INA219_CONFIG_SADCRES_12BIT_128S_69MS |
                  //
                  INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  _wireWriteRegister(INA219_REG_CONFIG, _configValue);
}

/**************************************************************************/
/*!
    @brief  Sets calibration values
*/
/**************************************************************************/
void Adafruit_INA219::_setCalibration() {
  _calValue = pgm_read_word(&(ina219_range_set_i[_range][0]));
  _currentLSB = pgm_read_float(&(ina219_range_set_f[_range][0])); // mA
  _currentDigitsAfterPoint = pgm_read_word(&(ina219_range_set_i[_range][1]));
  _currentOverflow = pgm_read_float(&(ina219_range_set_f[_range][1])); // A
  _powerLSB = 20 * _currentLSB; //mWatts

  // Set Calibration register to 'Cal' calculated above
  _wireWriteRegister(INA219_REG_CALIBRATION, _calValue);
}

/**************************************************************************/
/*!
    @brief  Sends a single command byte over I2C
*/
/**************************************************************************/
void Adafruit_INA219::_wireWriteRegister (uint8_t reg, uint16_t value)
{
  Wire.beginTransmission(_i2caddr);
  #if ARDUINO >= 100
    Wire.write(reg);                       // Register
    Wire.write((value >> 8) & 0xFF);       // Upper 8-bits
    Wire.write(value & 0xFF);              // Lower 8-bits
  #else
    Wire.send(reg);                        // Register
    Wire.send(value >> 8);                 // Upper 8-bits
    Wire.send(value & 0xFF);               // Lower 8-bits
  #endif
  Wire.endTransmission();
}

/**************************************************************************/
/*!
    @brief  Reads a 16 bit values over I2C
*/
/**************************************************************************/
void Adafruit_INA219::_wireReadRegister(uint8_t reg, uint16_t *value)
{

  Wire.beginTransmission(_i2caddr);
  #if ARDUINO >= 100
    Wire.write(reg);                       // Register
  #else
    Wire.send(reg);                        // Register
  #endif
  Wire.endTransmission();

  // delay(1); // Max 12-bit conversion time is 586us per sample

  Wire.requestFrom(_i2caddr, (uint8_t)2);
  #if ARDUINO >= 100
    // Shift values to create properly formed integer
    *value = ((Wire.read() << 8) | Wire.read());
  #else
    // Shift values to create properly formed integer
    *value = ((Wire.receive() << 8) | Wire.receive());
  #endif
}

/**************************************************************************/
/*!
    @brief  Sets the next Metering range
*/
/**************************************************************************/
void Adafruit_INA219::setNextRange() {
  _range++;
  if (_range > NumberOfRangeSets - 1) {
    _range = 0;
  }
  _setConfig();
  _setCalibration();
}

/**************************************************************************/
/*!
    @brief  Gets the Range
*/
/**************************************************************************/
uint8_t Adafruit_INA219::getRange() {
  return _range;
}

/**************************************************************************/
/*!
    @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Adafruit_INA219::getBusVoltage_raw() {
  uint16_t value;
  uint16_t i = 0;
  do {
    _wireReadRegister(INA219_REG_BUSVOLTAGE, &value);
    i++;
  } while(~value & (1 << 1)); //try until CNVR is set to 1

  _OVF = (1 << 0) & value; // read 0st bit
  // _CNVR = ((1 << 1) & value) >> 1;  // read 1st bit
  _CNVR = i; //number of read attempts to get updated values
  return (int16_t)(value >> 3);
}

/**************************************************************************/
/*!
    @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Adafruit_INA219::getShuntVoltage_raw() {
  uint16_t value;
  _wireReadRegister(INA219_REG_SHUNTVOLTAGE, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*!
    @brief  Gets the raw current value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Adafruit_INA219::getCurrent_raw() {
  uint16_t value;
  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
  _wireWriteRegister(INA219_REG_CALIBRATION, _calValue);

  // Now we can safely read the CURRENT register!
  _wireReadRegister(INA219_REG_CURRENT, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*!
    @brief  Gets the raw power value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Adafruit_INA219::getPower_raw() {
  uint16_t value;
  _wireReadRegister(INA219_REG_POWER, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*!
    @brief  Gets the OVF bit value
*/
/**************************************************************************/
int8_t Adafruit_INA219::getOVF() {
  return _OVF;
}

/**************************************************************************/
/*!
    @brief  Gets the CNVR bit value
*/
/**************************************************************************/
int8_t Adafruit_INA219::getCNVR() {
  return _CNVR;
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in mV
*/
/**************************************************************************/
float Adafruit_INA219::getShuntVoltage_mV() {
  int16_t value = getShuntVoltage_raw();
  return (float)value * _vshuntLSB;
}

/**************************************************************************/
/*!
    @brief  Gets the shunt voltage in volts
*/
/**************************************************************************/
float Adafruit_INA219::getBusVoltage_V() {
  int16_t value = getBusVoltage_raw();
  return (float)value * _vbusLSB;
}

/**************************************************************************/
/*!
    @brief  Gets the digits after point value
*/
/**************************************************************************/
uint16_t Adafruit_INA219::getVbusDigitsAfterPoint() {
  return _vbusDigitsAfterPoint;
}

/**************************************************************************/
/*!
    @brief  Gets the current value in mA, taking into account the
            config settings and current LSB
*/
/**************************************************************************/
float Adafruit_INA219::getCurrent_mA() {
  int16_t value = getCurrent_raw();
  return (float)value * _currentLSB;
}

/**************************************************************************/
/*!
    @brief  Gets the digits after point value
*/
/**************************************************************************/
uint16_t Adafruit_INA219::getCurrentDigitsAfterPoint() {
  return _currentDigitsAfterPoint;
}

/**************************************************************************/
/*!
    @brief  Gets the power value in mW, taking into account the
            config settings and LSB
*/
/**************************************************************************/
float Adafruit_INA219::getPower_mW() {
  int16_t value = getPower_raw();
  return (float)value * _powerLSB;
}
