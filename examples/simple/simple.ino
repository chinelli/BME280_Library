#include <SPI.h>
#include "BME280.h"

#define BME280_PIN 9
BME280 sensor = BME280(BME280_PIN);

double temperature;
double pressure;
double humidity;


void setup() {
    SPI.begin();
    Serial.begin(9600);
    Serial.println("BEGIN");
    sensor.setup();
    sensor.setTemperatureSampling(TemperatureOversampling::X1);
    sensor.setHumiditySampling(HumidityOversampling::X1);
    sensor.setPressureSampling(PressureOversampling::X1);
    sensor.setMode(Mode::Normal);
    delay(1000);
}



void loop() {

    temperature = sensor.getTemperature();
    pressure = sensor.getPressure();
    humidity = sensor.getHumidity();

    Serial.print("TEMPERATURE;");
    Serial.print(temperature);
    Serial.print(";");

    Serial.print("PRESSURE:");
    Serial.print(pressure);
    Serial.print(";");

    Serial.print("HUMIDITY:");
    Serial.print(humidity);
    Serial.println(";");


    delay(500);
}




