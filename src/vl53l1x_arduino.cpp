#include "vl53l1x.hpp"

#include <Wire.h>

VL53L1X_Abstract::error_t VL53L1X_Abstract::write_bytes(const uint16_t rgstr,
        const uint8_t count, const uint8_t *data)
{
    Wire.beginTransmission(_i2c_address);

    uint8_t buffer[2] = {};
    buffer[0] = rgstr >> 8;
    buffer[1] = rgstr & 0xFF;
    Wire.write(buffer, 2);
    for (uint16_t i = 0; i < count; i++) {
        Wire.write(data[i]);
    }
    Wire.endTransmission(true);
    return 0;
}

VL53L1X_Abstract::error_t VL53L1X_Abstract::read_bytes(const uint16_t rgstr, 
        const uint8_t count, uint8_t *data)
{
    int status = 0;

    //Loop until the port is transmitted correctly
    uint8_t maxAttempts = 5;
    for (uint8_t x = 0; x < maxAttempts; x++)
    {
        Wire.beginTransmission(_i2c_address);
        uint8_t buffer[2] = {};
        buffer[0] = rgstr >> 8;
        buffer[1] = rgstr & 0xFF;
        Wire.write(buffer, 2);
        status = Wire.endTransmission(false);

        if (status == 0) {
            break;
        }

        //Fix for some STM32 boards: reinitialize th i2c bus with the default parameters
#ifdef ARDUINO_ARCH_STM32
        if (status)
        {
            Wire.end();
            Wire.begin();
        }
#endif
        //End of fix
    }

    Wire.requestFrom(_i2c_address, (byte)count);

    int i = 0;
    while (Wire.available())
    {
        data[i] = Wire.read();
        i++;
    }

    return 0;
}
