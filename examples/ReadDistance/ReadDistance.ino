/*
  Reading distance from the laser based VL53L1X
*/

#include <Wire.h>

#include "vl53l1x_i2c.hpp"

static VL53L1X sensor = VL53L1X(&Wire);

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
