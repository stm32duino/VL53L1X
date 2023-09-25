/**
 ******************************************************************************
 * @brief Header-only file for the I^2C-based VL53L1X driver class
 ******************************************************************************
 */

#pragma once

#include "vl53l1x.hpp"

#include <Wire.h>

class VL53L1X : public VL53L1X_Abstract
{
    private:

        TwoWire * _twoWire;

    public:

        /** Constructor
         * @param[in] &i2c device I2C to be used for communication
         * @param[in] &pin_gpio1 pin Mbed InterruptIn PinName to be used as 
                      component GPIO_1 INT
         * @param[in] DevAddr device address, 0x52 by default
         */
        VL53L1X(TwoWire * twoWire, const uint8_t i2c_addr = 0x29) 
            : VL53L1X_Abstract(i2c_addr)
        {
            _twoWire = twoWire;
        }

    private:

        void beginTransmission(const uint8_t addr, const uint8_t rgstr)
        {
            _twoWire->beginTransmission(addr);

            const uint8_t buffer[2] = {
                (uint8_t)(rgstr >> 8), 
                (uint8_t)(rgstr & 0xFF)
            };

            _twoWire->write(buffer, 2);
        }

    protected:

        virtual error_t i2c_write(const uint16_t addr, const uint16_t rgstr,
                const uint8_t * data, const uint16_t nbytes) override 
        {
            beginTransmission(addr, rgstr);

            for (uint16_t i = 0; i < nbytes; i++)
                _twoWire->write(data[i]);

            _twoWire->endTransmission(true);

            return 0;

        }

        virtual error_t i2c_read(const uint16_t addr, const uint16_t rgstr,
                uint8_t * data, const uint16_t nbytes) override 
        {
            int status = 0;

            //Loop until the port is transmitted correctly
            uint8_t maxAttempts = 5;
            for (uint8_t x = 0; x < maxAttempts; x++) {

                beginTransmission(addr, rgstr);

                status = _twoWire->endTransmission(false);

                if (status == 0)
                    break;

                //Fix for some STM32 boards
                //Reinitialize th i2c bus with the default parameters
                if (status) {
#ifdef ARDUINO_ARCH_STM32
                    _twoWire->end();
                    _twoWire->begin();
#endif
                }
                //End of fix
            }

            _twoWire->requestFrom(addr, nbytes);

            int i = 0;
            while (_twoWire->available()) {
                data[i] = _twoWire->read();
                i++;
            }

            return 0;
        }

        virtual void wait_ms(const int32_t wait_ms) override
        {
            delay(wait_ms);
        }

}; // class VL53L1X
