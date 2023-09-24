<a href="https://www.tindie.com/products/onehorse/vl53l1-long-range-proximity-sensor"><img src="vl53l1.jpg" width=500></a>

This library derives from the 
[Stm32Duino VL53L1X Arduino Library](https://github.com/stm32duino/VL53L1X), q.v. for details.

I made the following modifications:

* Made the library header-only

* Replaced ```#define``` with typed static constants

* Added support for non-Arduino platforms via subclassing

I have tested this library with the following hardware:

* Teensy 4.0 + [Pesky Products Vl53L1 sensor](https://www.tindie.com/products/onehorse/vl53l1-long-range-proximity-sensor)

