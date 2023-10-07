#pragma once

#include "vl53l1x.hpp"

#include <Wire.h>

class VL53L1X_Arduino : public VL53L1X
{

    public:

        VL53L1X_Arduino(TwoWire * twoWire = &Wire) 
            : VL53L1X((void *)twoWire), _twoWire(twoWire)
        {
        }

    private:

        TwoWire * _twoWire;
};

VL53L1X::error_t VL53L1X::read_bytes(void * device, const uint16_t rgstr, 
        const uint8_t count, uint8_t *data)
{
    TwoWire * twoWire = (TwoWire *)device;

    int status = 0;

    //Loop until the port is transmitted correctly
    uint8_t maxAttempts = 5;
    for (uint8_t x = 0; x < maxAttempts; x++)
    {
        twoWire->beginTransmission(_i2c_address);
        uint8_t buffer[2] = {};
        buffer[0] = rgstr >> 8;
        buffer[1] = rgstr & 0xFF;
        twoWire->write(buffer, 2);
        status = twoWire->endTransmission(false);

        if (status == 0) {
            break;
        }

        //Fix for some STM32 boards: reinitialize th i2c bus with the default parameters
#ifdef ARDUINO_ARCH_STM32
        if (status)
        {
            twoWire->end();
            twoWire->begin();
        }
#endif
        //End of fix
    }

    twoWire->requestFrom(_i2c_address, (byte)count);

    int i = 0;
    while (twoWire->available())
    {
        data[i] = twoWire->read();
        i++;
    }

    return 0;
}

VL53L1X::error_t VL53L1X::write_bytes(void * device, const uint16_t rgstr, 
        const uint8_t count, const uint8_t *data)
{
    TwoWire * twoWire = (TwoWire *)device;

    twoWire->beginTransmission(_i2c_address);

    uint8_t buffer[2] = {};
    buffer[0] = rgstr >> 8;
    buffer[1] = rgstr & 0xFF;
    twoWire->write(buffer, 2);
    for (uint16_t i = 0; i < count; i++) {
        twoWire->write(data[i]);
    }
    twoWire->endTransmission(true);
    return 0;
}


