#include "Arduino.h"
#include <SPI.h>
#include "BME280.h"


/*
  TODO:
  * Remove getTemperature dependency, find a smart way to compute t_fine for calibration
  * use of Config register
  * read in block calibration data to increase performance
  * read in block measurement data to increase performance
  * SPI 3 Wire
  * i2c (I prefer SPI, it allows multiple sensor and greater speed (10Mhz), further  a i2c library is available on github)
  * BMP280 compatibility
*/

BME280::BME280(uint8_t cs_pin ):_cs_pin(cs_pin){
  _spi_settings = SPISettings(BME280_DEF_SPEED, MSBFIRST, SPI_MODE0);
}

BME280::BME280(uint8_t cs_pin, uint32_t spi_speed):_cs_pin(cs_pin){
  _spi_settings = SPISettings(spi_speed, MSBFIRST, SPI_MODE0);
}

void BME280::setup(){
  pinMode (_cs_pin, OUTPUT);
  SPI.begin();
  _mode = Mode::Sleep;
  _humidity_oversampling = HumidityOversampling::Skipped;
  _temperature_oversampling = TemperatureOversampling::Skipped;
  _pressure_oversampling = PressureOversampling::Skipped;
    
  dig_T1 = (uint16_t)read2RegisterLM(BME280_TEMPERATURE_CALIB_DIG_T1_LSB_REG);
  dig_T2 = (int16_t) read2RegisterLM(BME280_TEMPERATURE_CALIB_DIG_T2_LSB_REG);
  dig_T3 = (int16_t) read2RegisterLM(BME280_TEMPERATURE_CALIB_DIG_T3_LSB_REG);

  dig_P1 = (uint16_t)read2RegisterLM(BME280_PRESSURE_CALIB_DIG_P1_LSB_REG);
  dig_P2 = (int16_t) read2RegisterLM(BME280_PRESSURE_CALIB_DIG_P2_LSB_REG);
  dig_P3 = (int16_t) read2RegisterLM(BME280_PRESSURE_CALIB_DIG_P3_LSB_REG);
  dig_P4 = (int16_t) read2RegisterLM(BME280_PRESSURE_CALIB_DIG_P4_LSB_REG);
  dig_P5 = (int16_t) read2RegisterLM(BME280_PRESSURE_CALIB_DIG_P5_LSB_REG);
  dig_P6 = (int16_t) read2RegisterLM(BME280_PRESSURE_CALIB_DIG_P6_LSB_REG);
  dig_P7 = (int16_t) read2RegisterLM(BME280_PRESSURE_CALIB_DIG_P7_LSB_REG);
  dig_P8 = (int16_t) read2RegisterLM(BME280_PRESSURE_CALIB_DIG_P8_LSB_REG);
  dig_P9 = (int16_t) read2RegisterLM(BME280_PRESSURE_CALIB_DIG_P9_LSB_REG);
  

  dig_H1 = (uint8_t) readRegister(BME280_HUMIDITY_CALIB_DIG_H1_REG);
  dig_H2 = (int16_t) read2RegisterLM(BME280_HUMIDITY_CALIB_DIG_H2_LSB_REG);
  dig_H3 = (uint8_t) readRegister(BME280_HUMIDITY_CALIB_DIG_H3_REG);
  uint8_t dig_H4_MSB = (uint8_t)  readRegister(BME280_HUMIDITY_CALIB_DIG_H4_MSB_REG);
  uint8_t dig_H4_H5 = (uint8_t) readRegister(BME280_HUMIDITY_CALIB_DIG_H4_LSB_REG);
  uint8_t dig_H5_MSB = (uint8_t)  readRegister(BME280_HUMIDITY_CALIB_DIG_H5_MSB_REG);
  dig_H4 = (int16_t) (dig_H4_MSB << 4) | (dig_H4_H5 & BME280_MASK_DIG_H4);
  dig_H5 = (int16_t) ( dig_H5_MSB << 4 ) | (dig_H4_H5 >>4) ;
  dig_H6 = (int8_t)  readRegister(BME280_HUMIDITY_CALIB_DIG_H6_REG);
  

  
  Serial.print("H1 ");
  Serial.println(dig_H1);
    Serial.print("H2 ");

    Serial.println(dig_H2);
  Serial.print("H3 ");

  Serial.println(dig_H3);
  Serial.print("H4 ");

  Serial.println(dig_H4);
  Serial.print("H5 ");

  Serial.println(dig_H5);
    Serial.print("H6 ");

  Serial.println(dig_H6);

}

void BME280::setHumiditySampling(HumidityOversampling::HumidityOversampling_t oversampling) {
  if (_humidity_oversampling == oversampling) 
      return;
  _humidity_oversampling = oversampling;
  updateCtrlHumReg();
  updateCtrlMeasReg();
}

void BME280::setTemperatureSampling(TemperatureOversampling::TemperatureOversampling_t oversampling) {
  if (_temperature_oversampling == oversampling) 
      return;
  _temperature_oversampling = oversampling;
  updateCtrlMeasReg();
}

void BME280::setPressureSampling(PressureOversampling::PressureOversampling_t oversampling) {
  if (_pressure_oversampling == oversampling) 
      return;
  _pressure_oversampling = oversampling;
  updateCtrlMeasReg();
}
    
void BME280::setMode(Mode::Mode_t mode) {
  if (_mode == mode) 
      return;
  _mode = mode;
  updateCtrlMeasReg();
}

void BME280::updateCtrlHumReg() {
   digitalWrite (_cs_pin, LOW);
  SPI.beginTransaction(_spi_settings);
  // Writing to ctrl_hum register
  SPI.transfer(BME280_CTRL_HUMIDITY_REG & BME280_REG_WRITE_MASK);
  SPI.transfer(static_cast<uint8_t>(_humidity_oversampling));
  // Writing to ctrl_meas register
  SPI.endTransaction();
  digitalWrite (_cs_pin, HIGH);
}

void BME280::updateCtrlMeasReg() {
   digitalWrite (_cs_pin, LOW);
  SPI.beginTransaction(_spi_settings);
  // Writing to ctrl_meas register
  SPI.transfer(BME280_CTRL_MEAS_REG & BME280_REG_WRITE_MASK);
  // Enabling 1xTemp and normal mode (cyclic measurement)
  byte cmd = B0;
  cmd = (static_cast<uint8_t>(_temperature_oversampling) << 5) |
        (static_cast<uint8_t>(_pressure_oversampling) << 2) |
        static_cast<uint8_t>(_mode);
  SPI.transfer(cmd);
  SPI.endTransaction();
  digitalWrite (_cs_pin, HIGH);
}

double BME280::getTemperature() {
  if (_temperature_oversampling == TemperatureOversampling::Skipped)
      return -1;
  if(_mode == Mode::Forced)
      updateCtrlMeasReg();
  int32_t val = read3Registers(BME280_TEMPERATURE_MSB_REG);
  double temp = BME280_compensate_T_double(val);
  return temp;
  
}

double BME280::getPressure() {
  if (_pressure_oversampling == PressureOversampling::Skipped)
      return -1;
  if(_mode == Mode::Forced)
      updateCtrlMeasReg();
  int32_t val2 = read3Registers(BME280_PRESSURE_MSB_REG);
  double pressure = BME280_compensate_P_double(val2);
  return pressure;
}

double BME280::getHumidity() {
  if (_humidity_oversampling == HumidityOversampling::Skipped)
      return -1;
  if(_mode == Mode::Forced)
      updateCtrlMeasReg();
  int32_t val2 = read2Register(BME280_HUMIDITY_MSB_REG);
  double pressure = bme280_compensate_H_double(val2);
  return pressure;
}
    

//Read from or write to register from the SCP1000:
int32_t BME280::read3Registers(byte start_reg) {
  digitalWrite (_cs_pin, LOW);
  SPI.beginTransaction(_spi_settings);
  uint8_t inByte = 0;           // incoming byte from the SPI
  uint32_t result = 0;   // result to return
  // Sending read command for msb register
  SPI.transfer(start_reg  | BME280_REG_READ_CMD);
  
  inByte = SPI.transfer(0x00);
  result = result | inByte;
  
  // Reading lsb byte
  inByte = SPI.transfer(0x00);
    // Shifting left to make room for the lsb byte
  result = result << 8;
  
  // Combine with result
  result = result | inByte;
  
  // Reading xlsb byte
  inByte = SPI.transfer(0x00);
  // Shifting right 0ed bits.
    // Shifting left to make room for the xlsb half byte
  result = result << 4;
  inByte = inByte >> 4;
  // Combine with result
  result = result | inByte;
  SPI.endTransaction();
  digitalWrite (_cs_pin, HIGH);
  return (result);
}


uint16_t BME280::read2Register(byte reg) {
  digitalWrite (_cs_pin, LOW);
  SPI.beginTransaction(_spi_settings);

  uint8_t lsb = 0;           // incoming byte from the SPI
  uint8_t msb = 0;           // incoming byte from the SPI
  uint16_t result = 0;   // result to return
  /*Serial.print(" ");
  Serial.print(reg + 1, HEX);
  Serial.println(" \t");
  Serial.print("RAW ");
  */
  SPI.transfer(reg   | BME280_REG_READ_CMD);
  msb = SPI.transfer(0x00);
  /*Serial.print(reg, HEX);
  Serial.print(lsb, BIN);*/
  lsb = SPI.transfer(0x00);
  /*Serial.print(reg + 1, HEX);
  Serial.print(msb, BIN);
  Serial.print(" ");*/
  result = (msb << 8) | lsb;
  //Serial.println(result,BIN);
  SPI.endTransaction();
  digitalWrite (_cs_pin, HIGH);

  return ((msb << 8) | lsb);
}


uint16_t BME280::read2RegisterLM(byte reg) {
  digitalWrite (_cs_pin, LOW);
  SPI.beginTransaction(_spi_settings);

  uint8_t lsb = 0;           // incoming byte from the SPI
  uint8_t msb = 0;           // incoming byte from the SPI
  uint16_t result = 0;   // result to return
  SPI.transfer(reg | BME280_REG_READ_CMD);
  lsb = SPI.transfer(0x00);
  msb = SPI.transfer(0x00);
  result = (msb << 8) | lsb;
  SPI.endTransaction();
  digitalWrite (_cs_pin, HIGH);

  return ((msb << 8) | lsb);
}



//Read from or write to register from the SCP1000:
uint8_t BME280::readRegister(byte reg) {
  digitalWrite (_cs_pin, LOW);
  SPI.beginTransaction(_spi_settings);

  uint8_t read_byte = 0;           // incoming byte from the SPI
  SPI.transfer(reg  | BME280_REG_READ_CMD);
  read_byte = SPI.transfer(0x00);
  SPI.endTransaction();
  digitalWrite (_cs_pin, HIGH);

  return read_byte;
}

double BME280::BME280_compensate_T_double(int32_t adc_T) {
  double var1, var2, T;
  var1 = (((double)adc_T) / 16384.0 - ((double)dig_T1) / 1024.0) * ((double)dig_T2);
  var2 = ((((double)adc_T) / 131072.0 - ((double)dig_T1) / 8192.0) * (((double)adc_T) / 131072.0 - ((double) dig_T1) / 8192.0)) * ((double)dig_T3);
  t_fine = (int32_t)(var1 + var2);
  T = (var1 + var2) / 5120.0f;
  return T;
}

// Returns pressure in Pa as double. Output value of “96386.2” equals 96386.2 Pa = 963.862 hPa
double BME280::BME280_compensate_P_double(int32_t adc_P)
{
  double var1, var2, p;
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)dig_P4) * 65536.0);
  var1 = (((double)dig_P3) * var1 * var1 / 524288.0 + ((double)dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)dig_P1);
  if (var1 == 0.0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576.0 - (double)adc_P;
  p = (p - (var2 / 4096.0)) * 6250.0 / var1;
  var1 = ((double)dig_P9) * p * p / 2147483648.0;
  var2 = p * ((double)dig_P8) / 32768.0;
  p = p + (var1 + var2 + ((double)dig_P7)) / 16.0;
  return p;
}

int32_t BME280::BME280_compensate_T_int32(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) *
          ((int32_t)dig_T3)) >> 14;
  t_fine = var1 + var2;
  T = (t_fine * 5 + 128) >> 8;
  return T;
}

// Returns humidity in %rH as as double. Output value of “46.332” represents 46.332 %rH
double BME280::bme280_compensate_H_double(int32_t adc_H)
{
  double var_H;
  var_H = (((double)t_fine) - 76800.0);
  var_H = (adc_H - (((double)dig_H4) * 64.0 + ((double)dig_H5) / 16384.0 * var_H)) *
          (((double)dig_H2) / 65536.0 * (1.0 + ((double)dig_H6) / 67108864.0 * var_H *
                                         (1.0 + ((double)dig_H3) / 67108864.0 * var_H)));
  var_H = var_H * (1.0 - ((double)dig_H1) * var_H / 524288.0);
  if (var_H > 100.0)
    var_H = 100.0;
  else if (var_H < 0.0)
    var_H = 0.0;
  return var_H;
}
