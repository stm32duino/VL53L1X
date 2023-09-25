/**
 ******************************************************************************
 * @brief Header-only file for the I^2C-based VL53L1X driver class
 ******************************************************************************
 */

#pragma once

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

class VL53L1X : public VL53L1X_Abstract
{
    private:

        int8_t _fd;

    public:

        void begin(const uint8_t bus=1)
        {
            // Attempt to open /dev/i2c-<BUS>
            char fname[32] = {};
            sprintf(fname,"/dev/i2c-%d", bus);
            _fd = open(fname, O_RDWR);
            if (_fd < 0) {
                fprintf(stderr, "Unable to open %s\n", fname);
                exit(1);
            }
        }

    protected:

        // Adapted from 
        //  https://github.com/mjbogusz/vl53l1x-linux/blob/master/src/I2CBus.cpp

        virtual error_t i2c_write(const uint16_t rgstr, const uint8_t * data,
                const uint16_t nbytes) override 
        {
            uint8_t writeBuffer[nbytes + 2];
            writeBuffer[0] = (rgstr >> 8) & 0xFF;
            writeBuffer[1] = rgstr & 0xFF;
            for (int i = 0; i < nbytes; ++i) {
                writeBuffer[i + 2] = data[i];
            }

            i2c_msg msgs[1] = {{
                .addr = _i2c_addr,
                    .flags = 0,
                    .len = static_cast<uint16_t>(nbytes + 2),
                    .buf = writeBuffer,
            }};
            i2c_rdwr_ioctl_data msgset = {
                .msgs = msgs,
                .nmsgs = 1,
            };

            ioctl(_fd, I2C_RDWR, &msgset);

            return 0;
        }

        virtual error_t i2c_read(const uint16_t rgstr, uint8_t * data, 
                const uint16_t nbytes) override 
        {
            uint8_t writeBuffer[2] = {};
            writeBuffer[0] = (rgstr >> 8) & 0xFF;
            writeBuffer[1] = rgstr & 0xFF;
            i2c_msg registerSelectMsgs[1] = {{
                .addr = _i2c_addr,
                    .flags = 0,
                    .len = 2,
                    .buf = writeBuffer
            }};
            i2c_rdwr_ioctl_data registerSelectMsgSet = {
                .msgs = registerSelectMsgs,
                .nmsgs = 1,
            };
            i2c_msg registerReadMsgs[1] = {{
                .addr = _i2c_addr,
                    .flags = I2C_M_RD,
                    .len = nbytes,
                    .buf = data,
            }};
            i2c_rdwr_ioctl_data registerReadMsgSet = {
                .msgs = registerReadMsgs,
                .nmsgs = 1,
            };

            ioctl(_fd, I2C_RDWR, &registerSelectMsgSet);
            ioctl(_fd, I2C_RDWR, &registerReadMsgSet);

            return 0;
        }

        virtual void wait_ms(const int32_t wait_ms) override
        {
            usleep(1000 * wait_ms);
        }

}; // class VL53L1X
