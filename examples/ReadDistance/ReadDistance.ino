#include <Wire.h>

#include "vl53l1x_arduino.hpp"

static VL53L1X_Arduino ranger;

void setup(void)
{
    int8_t status = VL53L1X::ERROR_NONE;

    Wire.begin();

    Serial.begin(115200);

    status |= ranger.init();

    status |= ranger.SetDistanceMode(VL53L1X::DISTANCEMODE_MEDIUM);

    status |= ranger.SetMeasurementTimingBudgetMicroSeconds(25000);

    if (status) {
        Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
        while (true)
            ;
    }
}

void loop(void)
{
    ranger.startRanging();

    while (true) {

        bool dataReady = false;

        ranger.checkForDataReady(&dataReady);

        if (dataReady) {
            break;
        }

        delay(1);
    }

    uint16_t distance = 0;
    ranger.getDistance(&distance);

    ranger.stopRanging();

    Serial.printf("%d mm\n", distance);
}
