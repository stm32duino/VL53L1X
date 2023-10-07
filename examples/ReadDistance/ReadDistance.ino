#include <Wire.h>

#include "vl53l1x_arduino.hpp"

static VL53L1X_Arduino ranger;

void setup(void)
{
    int8_t status = VL53L1X::ERROR_NONE;

    Wire.begin();

    Serial.begin(115200);

    status |= ranger.begin();

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
    uint16_t distance = 0;

    auto status = ranger.readDistance(&distance);

    if (status) {
        Serial.print("Error reading from sensor: ");
        Serial.println(status);
    }

    else{ 
        Serial.print(distance);
        Serial.println(" mm");
    }
}
