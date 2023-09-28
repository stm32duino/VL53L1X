/*
  Reading distance from the laser based VL53L1X
*/

#include <Wire.h>

#include "vl53l1x.hpp"

static VL53L1X sensor;

void setup(void)
{
  Wire.begin();

  Serial.begin(115200);

  sensor.begin(0x29);

  sensor.stopRanging();
  sensor.setDistanceMode(VL53L1X::DISTANCE_MODE_MEDIUM);
  sensor.setTimingBudgetInMs(25);

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

        delay(1);
    }

    uint16_t distance = 0;
    sensor.getDistance(&distance);

    Serial.print("Distance(mm): ");
    Serial.println(distance);

    sensor.stopRanging();
    sensor.startRanging();
}
