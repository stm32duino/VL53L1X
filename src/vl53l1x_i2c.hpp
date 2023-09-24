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
        VL53L1X(TwoWire * twoWire) 
        {
            _twoWire = twoWire;
        }

    protected:

        virtual error_t i2c_write(uint16_t addr, uint16_t rgstr, uint8_t *buff,
                uint16_t nbytes) override 
        {
            _twoWire->beginTransmission((addr >> 1) & 0x7F);
            uint8_t buffer[2];
            buffer[0] = rgstr >> 8;
            buffer[1] = rgstr & 0xFF;
            _twoWire->write(buffer, 2);
            for (uint16_t i = 0; i < nbytes; i++)
                _twoWire->write(buff[i]);

            _twoWire->endTransmission(true);
            return 0;
        }

        virtual error_t i2c_read(uint16_t addr, uint16_t rgstr, uint8_t *buff,
                uint16_t nbytes) override 
        {
            int status = 0;

            //Loop until the port is transmitted correctly
            uint8_t maxAttempts = 5;
            for (uint8_t x = 0; x < maxAttempts; x++)
            {
                _twoWire->beginTransmission((addr >> 1) & 0x7F);
                uint8_t buffer[2];
                buffer[0] = rgstr >> 8;
                buffer[1] = rgstr & 0xFF;
                _twoWire->write(buffer, 2);
                status = _twoWire->endTransmission(false);

                if (status == 0)
                    break;

                //Fix for some STM32 boards
                //Reinitialize th i2c bus with the default parameters
#ifdef ARDUINO_ARCH_STM32
                if (status)
                {
                    _twoWire->end();
                    _twoWire->begin();
                }
#endif
                //End of fix
            }

            _twoWire->requestFrom((addr >> 1) & 0x7F, nbytes);

            int i = 0;
            while (_twoWire->available())
            {
                buff[i] = _twoWire->read();
                i++;
            }

            return 0;
        }

        virtual error_t wait_ms(int32_t wait_ms) override
        {
            delay(wait_ms);
            return ERROR_NONE;
        }

}; // class VL53L1X
