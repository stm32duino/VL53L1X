#pragma once

/*
   Linux I2Cdev support for VL53L1X

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

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/time.h>

extern "C" {
#include <i2c/smbus.h>
}

#include "vl53l1x.hpp"

class VL53L1X_Linux : public VL53L1X
{
    private:

        int8_t _fd;

    public:

        error_t begin(const uint8_t bus=1)
        {
            // Attempt to open /dev/i2c-<BUS>
            char fname[32] = {};
            sprintf(fname,"/dev/i2c-%d", bus);
            _fd = open(fname, O_RDWR);
            if (_fd < 0) {
                fprintf(stderr, "Unable to open %s\n", fname);
                exit(1);
            }

            return VL53L1X::begin((void *)&_fd);
        }
};

// Adapted from 
//  https://github.com/mjbogusz/vl53l1x-linux/blob/master/src/I2CBus.cpp

VL53L1X::error_t VL53L1X::read_bytes(void * device, const uint16_t rgstr, 
        const uint8_t count, uint8_t * data)
{
    const int8_t fd = *((int8_t *)device);

    uint8_t writeBuffer[2] = {};
    writeBuffer[0] = (rgstr >> 8) & 0xFF;
    writeBuffer[1] = rgstr & 0xFF;
    i2c_msg registerSelectMsgs[1] = {{
        .addr = _i2c_address,
            .flags = 0,
            .len = 2,
            .buf = writeBuffer
    }};
    i2c_rdwr_ioctl_data registerSelectMsgSet = {
        .msgs = registerSelectMsgs,
        .nmsgs = 1,
    };
    i2c_msg registerReadMsgs[1] = {{
        .addr = _i2c_address,
            .flags = I2C_M_RD,
            .len = count,
            .buf = data,
    }};
    i2c_rdwr_ioctl_data registerReadMsgSet = {
        .msgs = registerReadMsgs,
        .nmsgs = 1,
    };

    ioctl(fd, I2C_RDWR, &registerSelectMsgSet);
    ioctl(fd, I2C_RDWR, &registerReadMsgSet);

    return 0;
}


VL53L1X::error_t VL53L1X::write_bytes(void * device, const uint16_t rgstr, 
        const uint8_t count, const uint8_t * data)
{
    const int8_t fd = *((int8_t *)device);

    uint8_t writeBuffer[count + 2];
    writeBuffer[0] = (rgstr >> 8) & 0xFF;
    writeBuffer[1] = rgstr & 0xFF;
    for (int i = 0; i < count; ++i) {
        writeBuffer[i + 2] = data[i];
    }

    i2c_msg msgs[1] = {{
        .addr = _i2c_address,
            .flags = 0,
            .len = static_cast<uint16_t>(count + 2),
            .buf = writeBuffer,
    }};
    i2c_rdwr_ioctl_data msgset = {
        .msgs = msgs,
        .nmsgs = 1,
    };

    ioctl(fd, I2C_RDWR, &msgset);

    return 0;
}

void VL53L1X::delay_msec(const uint32_t msec)
{
    usleep(1000 * msec);
}
