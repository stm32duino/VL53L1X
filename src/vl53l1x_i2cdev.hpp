/**
 ******************************************************************************
 * @brief Header-only file for the I^2C-based VL53L1X driver class
 ******************************************************************************
 */

#pragma once

#include "vl53l1x.hpp"

class VL53L1X : public VL53L1X_Abstract
{
    private:


    public:

        VL53L1X() 
        {
        }

    protected:

        virtual error_t i2c_write(const uint16_t addr, const uint16_t rgstr,
                uint8_t *buff, const uint16_t nbytes) override 
        {
            return 0;
        }

        virtual error_t i2c_read(const uint16_t addr, const uint16_t rgstr, uint8_t *buff,
                const uint16_t nbytes) override 
        {
            int status = 0;

            return status;
        }

        virtual void wait_ms(const int32_t wait_ms) override
        {
        }

}; // class VL53L1X
