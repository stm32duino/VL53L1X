/**
 ******************************************************************************
 * @brief Header-only file for the I^2C-based VL53L1X driver class
 ******************************************************************************
 */

#pragma once

#include <linux/i2c-dev.h>
extern "C" {
#include <i2c/smbus.h>
}
#include <sys/ioctl.h>
#include <string.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>

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

            // Attempt to make this device an I2C slave
            if (ioctl(_fd, I2C_SLAVE, address) < 0) {
                fprintf(stderr, "ioctl failed on %s\n", fname);
                exit(1);
            }        
        }

    protected:

        virtual error_t i2c_write(const uint16_t addr, const uint16_t rgstr,
                const uint8_t * data, const uint16_t nbytes) override 
        {
            return 0;
        }

        virtual error_t i2c_read(const uint16_t addr, const uint16_t rgstr,
                uint8_t * data, const uint16_t nbytes) override 
        {
            int status = 0;

            return status;
        }

        virtual void wait_ms(const int32_t wait_ms) override
        {
        }

}; // class VL53L1X
