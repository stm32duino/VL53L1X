#pragma once

/*
   Arduino support for VL53L1X

   Copyright (c) 2023, Simon D. Levy
   All Rights Reserved

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

   3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software
   without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
   FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
   DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
   SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
   OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


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

    return ERROR_NONE;
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

    return ERROR_NONE;
}

void VL53L1X::delay_msec(const uint32_t msec)
{
    delay(msec);
}
