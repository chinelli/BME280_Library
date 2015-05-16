# Bosch BME280 Arduino/Teensy Library
An Arduino/Teensy library to communicate with the environmental Bosch sensor BME280.

* This library is in the early stages of development.
* It is tested on Teensy3.1.
* It should work on any Arduino with SPI transactions (at least it compiles).
* Since temperature is used for compensation of humidity and pressure it should be read just before other values to receive consistent measurements.
* **Most important: Feedbacks and pull request are welcome!**

##### TODO
1. use config register
* Remove getTemperature dependency, find a smart way to compute t_fine for compensation
* Read in burst measurement data to increase performance and consistency
* More documentation / examples
* Read in burst calibration data to increase performance
* SPI 3 Wire
* i2c: I prefer SPI, it allows multiple sensor and greater speed (10Mhz), further an i2c library is available on github.
* BMP280 compatibility

##### Changelog
* [0.1] - Initial version
