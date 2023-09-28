#include <stdio.h>

#include <vl53l1x_i2cdev.hpp>

int main(int argc, char ** argv)
{
    static VL53L1X sensor;

    sensor.begin();

    while (true) {

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

        printf("Distance(mm): %d\n", distance);
    }

    return 0;
}
