/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>

#include "vl53l1x.hpp"

static VL53L1X sensor = VL53L1X(&Wire, -1, -1);

void setup(void)
{
  Wire.begin();

  Serial.begin(115200);

  sensor.begin();

  Serial.println("Sensor online!");
}

void loop(void)
{
  sensor.startRanging(); 

  while (true) {
      uint8_t dataReady = 0;
      sensor.checkForDataReady(&dataReady);
      if (dataReady) {
          break;
      }
  }

  uint16_t distance = 0;
  sensor.getDistance(&distance);

  sensor.clearInterrupt();

  sensor.stopRanging();

  Serial.print("Distance(mm): ");
  Serial.println(distance);
}
