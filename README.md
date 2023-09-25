<a href="https://www.tindie.com/products/onehorse/vl53l1-long-range-proximity-sensor"><img src="vl53l1.jpg" width=500></a>

This library derives from the 
[Stm32Duino VL53L1X Arduino Library](https://github.com/stm32duino/VL53L1X), q.v. for details.

I made the following modifications:

* Made the library header-only

* Replaced ```#define``` with typed static constants

* Added support for non-Arduino platforms via subclassing

Using the [Pesky Products Vl53L1 sensor](https://www.tindie.com/products/onehorse/vl53l1-long-range-proximity-sensor)
I have tested this library with the following MCUs:

* [Teensy 4.0](https://www.pjrc.com/store/teensy40.html)

* [Dragonfly STM32L4](https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board)
  
* [TinyPICO ESP32](https://www.tinypico.com/)

* [Adafruit Feather STM32F405 Express](https://www.adafruit.com/product/4382)

* [RaspberryPi 3](https://www.raspberrypi.com/products/raspberry-pi-3-model-b/)
