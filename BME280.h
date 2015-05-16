/*
  BME280.h - read temp,pressure and humidity from bme280 in SPI 4W
  Created by Mirco Chinelli 
  
  Warning: To avoid erroneous reads, getTemperature should be called immediatly before 
*/




#ifndef BME280_h
#define BME280_h

#include "Arduino.h"
#include <SPI.h>
#include "BME280_defs.h"

#define BME280_DEF_SPEED 10000000


#define BME280_OVERSAMP_SKIPPED          (0x00)
#define BME280_OVERSAMP_1X               (0x01)
#define BME280_OVERSAMP_2X               (0x02)
#define BME280_OVERSAMP_4X               (0x03)
#define BME280_OVERSAMP_8X               (0x04)
#define BME280_OVERSAMP_16X              (0x05)



struct HumidityOversampling {
    enum HumidityOversampling_t{
        Skipped = BME280_OVERSAMP_SKIPPED,
        X1 = BME280_OVERSAMP_1X,
        X2 = BME280_OVERSAMP_2X,
        X4 = BME280_OVERSAMP_4X,
        X8 = BME280_OVERSAMP_8X,
        X16 = BME280_OVERSAMP_16X
    };
};

struct TemperatureOversampling {
  enum TemperatureOversampling_t{
      Skipped = BME280_OVERSAMP_SKIPPED,
      X1 = BME280_OVERSAMP_1X,
      X2 = BME280_OVERSAMP_2X,
      X4 = BME280_OVERSAMP_4X,
      X8 = BME280_OVERSAMP_8X,
      X16 = BME280_OVERSAMP_16X,
  };
};

struct PressureOversampling {
  enum PressureOversampling_t{
    Skipped = BME280_OVERSAMP_SKIPPED,
    X1 = BME280_OVERSAMP_1X,
    X2 = BME280_OVERSAMP_2X,
    X4 = BME280_OVERSAMP_4X,
    X8 = BME280_OVERSAMP_8X,
    X16 = BME280_OVERSAMP_16X,
 };
};

struct Mode {
  enum Mode_t{
      Sleep = BME280_SLEEP_MODE,
      Forced = BME280_FORCED_MODE,
      Normal = BME280_NORMAL_MODE,
 };
};

class BME280
{
  public:
    BME280(uint8_t cs_pin);
    BME280(uint8_t cs_pin, uint32_t spi_speed);
    void setup();

    void setMode(Mode::Mode_t );
    void setHumiditySampling(HumidityOversampling::HumidityOversampling_t );
    void setTemperatureSampling(TemperatureOversampling::TemperatureOversampling_t );
    void setPressureSampling(PressureOversampling::PressureOversampling_t );
    
    double getTemperature();
    double getPressure();
    double getHumidity();
    
  private:
    Mode::Mode_t _mode = Mode::Sleep;
    HumidityOversampling::HumidityOversampling_t _humidity_oversampling = HumidityOversampling::Skipped;
    TemperatureOversampling::TemperatureOversampling_t _temperature_oversampling = TemperatureOversampling::Skipped;
    PressureOversampling::PressureOversampling_t _pressure_oversampling = PressureOversampling::Skipped;
    
    
    // SPI
    uint8_t _cs_pin;
    SPISettings _spi_settings;
    
    // Calibration
    uint16_t dig_T1 = 0;
    int16_t dig_T2 = 0, dig_T3 = 0; 
    uint16_t dig_P1 = 0;
    int16_t dig_P2 = 0, dig_P3 = 0, dig_P4 = 0, dig_P5 = 0, dig_P6 = 0, dig_P7 = 0, dig_P8 = 0, dig_P9 = 0;
    uint8_t dig_H1,dig_H3;
    int16_t dig_H2,dig_H4,dig_H5;
    int8_t dig_H6;
    int32_t t_fine;
    
    // Internal functions
    void updateCtrlHumReg();
    void updateCtrlMeasReg();
    double bme280_compensate_H_double(int32_t adc_H);
    int32_t BME280_compensate_T_int32(int32_t adc_T);
    double BME280_compensate_P_double(int32_t adc_P);
    double BME280_compensate_T_double(int32_t adc_T);
    uint8_t readRegister(byte reg);
    uint16_t read2RegisterLM(byte reg);
    uint16_t read2Register(byte reg);
    int32_t read3Registers(byte reg);
};

#endif

