/**
 ******************************************************************************
 * @brief Header-only file for the VL53L1X driver class, abstracting the 
          I^2C calls
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2018 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#pragma once

class VL53L1X_Abstract
{
    public:

        /**
         * @brief defines SW Version
         */
        typedef struct {
            uint8_t major; /*!< major number */
            uint8_t minor; /*!< minor number */
            uint8_t build; /*!< build number */
            uint32_t revision; /*!< revision number */
        } version_t;

        typedef int8_t error_t;

        enum {

            ERROR_NONE = 0,
            ERROR_CALIBRATION_WARNING = - 1,
            /*!< Warning invalid calibration data may be in used
              \a InitData(,
              \a GetOffsetCalibrationData
              \a SetOffsetCalibrationData */
            ERROR_MIN_CLIPPED = - 2,
            /*!< Warning parameter passed was clipped to min before to be applied */

            ERROR_UNDEFINED = - 3,
            /*!< Unqualified error */
            ERROR_INVALID_PARAMS = - 4,
            /*!< Parameter passed is invalid or out of range */
            ERROR_NOT_SUPPORTED = - 5,
            /*!< Function is not supported in current mode or configuration */
            ERROR_RANGE_ERROR = - 6,
            /*!< Device report a ranging error interrupt status */
            ERROR_TIME_OUT = - 7,
            /*!< Aborted due to time out */
            ERROR_MODE_NOT_SUPPORTED = - 8,
            /*!< Asked mode is not supported by the device */
            ERROR_BUFFER_TOO_SMALL = - 9,
            /*!< ... */
            ERROR_COMMS_BUFFER_TOO_SMALL = - 10,
            /*!< Supplied buffer is larger than I2C supports */
            ERROR_GPIO_NOT_EXISTING = - 11,
            /*!< User tried to setup a non-existing GPIO pin */
            ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED = - 12,
            /*!< unsupported GPIO functionality */
            ERROR_CONTROL_INTERFACE = - 13,
            /*!< error reported from IO functions */
            ERROR_INVALID_COMMAND = - 14,
            /*!< The command is not allowed in the current device state
             * (power down, */
            ERROR_DIVISION_BY_ZERO = - 15,
            /*!< In the function a division by zero occurs */
            ERROR_REF_SPAD_INIT = - 16,
            /*!< Error during reference SPAD initialization */
            ERROR_GPH_SYNC_CHECK_FAIL = - 17,
            /*!< GPH sync interrupt check fail - API out of sync with device*/
            ERROR_STREAM_COUNT_CHECK_FAIL = - 18,
            /*!< Stream count check fail - API out of sync with device */
            ERROR_GPH_ID_CHECK_FAIL = - 19,
            /*!< GPH ID check fail - API out of sync with device */
            ERROR_ZONE_STREAM_COUNT_CHECK_FAIL = - 20,
            /*!< Zone dynamic config stream count check failed - API out of sync */
            ERROR_ZONE_GPH_ID_CHECK_FAIL = - 21,
            /*!< Zone dynamic config GPH ID check failed - API out of sync */

            ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL = - 22,
            /*!< Thrown when run_xtalk_extraction fn has 0 succesful samples
             * when using the full array to sample the xtalk. In this case there is
             * not enough information to generate new Xtalk parm info. The function
             * will exit and leave the current xtalk parameters unaltered */
            ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL = - 23,
            /*!< Thrown when run_xtalk_extraction fn has found that the
             * avg sigma estimate of the full array xtalk sample is > than the
             * maximal limit allowed. In this case the xtalk sample is too noisy for
             * measurement. The function will exit and leave the current xtalk parameters
             * unaltered. */


            ERROR_OFFSET_CAL_NO_SAMPLE_FAIL = - 24,
            /*!< Thrown if there one of stages has no valid offset calibration
             * samples. A fatal error calibration not valid */
            ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL = - 25,
            /*!< Thrown if there one of stages has zero effective SPADS
             * Traps the case when MM1 SPADs is zero.
             * A fatal error calibration not valid */
            ERROR_ZONE_CAL_NO_SAMPLE_FAIL = - 26,
            /*!< Thrown if then some of the zones have no valid samples
             * A fatal error calibration not valid */

            ERROR_TUNING_PARM_KEY_MISMATCH = - 27,
            /*!< Thrown if the tuning file key table version does not match with
             * expected value. The driver expects the key table version to match
             * the compiled default version number in the define
             * #TUNINGPARM_KEY_TABLE_VERSION_DEFAULT
             * */

            WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS = - 28,
            /*!< Thrown if there are less than 5 good SPADs are available. */
            WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH = - 29,
            /*!< Thrown if the final reference rate is greater than
              the upper reference rate limit - default is 40 Mcps.
              Implies a minimum Q3 (x10, SPAD (5, selected */
            WARNING_REF_SPAD_CHAR_RATE_TOO_LOW = - 30,
            /*!< Thrown if the final reference rate is less than
              the lower reference rate limit - default is 10 Mcps.
              Implies maximum Q1 (x1, SPADs selected */


            WARNING_OFFSET_CAL_MISSING_SAMPLES = - 31,
            /*!< Thrown if there is less than the requested number of
             * valid samples. */
            WARNING_OFFSET_CAL_SIGMA_TOO_HIGH = - 32,
            /*!< Thrown if the offset calibration range sigma estimate is greater
             * than 8.0 mm. This is the recommended min value to yield a stable
             * offset measurement */
            WARNING_OFFSET_CAL_RATE_TOO_HIGH = - 33,
            /*!< Thrown when run_offset_calibration(, peak rate is greater
              than that 50.0Mcps. This is the recommended max rate to avoid
              pile-up influencing the offset measurement */
            WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW = - 34,
            /*!< Thrown when run_offset_calibration(, when one of stages
              range has less that 5.0 effective SPADS. This is the recommended
              min value to yield a stable offset */


            WARNING_ZONE_CAL_MISSING_SAMPLES = - 35,
            /*!< Thrown if one of more of the zones have less than
              the requested number of valid samples */
            WARNING_ZONE_CAL_SIGMA_TOO_HIGH = - 36,
            /*!< Thrown if one or more zones have sigma estimate value greater
             * than 8.0 mm. This is the recommended min value to yield a stable
             * offset measurement */
            WARNING_ZONE_CAL_RATE_TOO_HIGH = - 37,
            /*!< Thrown if one of more zones have peak rate higher than
              that 50.0Mcps. This is the recommended max rate to avoid
              pile-up influencing the offset measurement */


            WARNING_XTALK_MISSING_SAMPLES = - 38,
            /*!< Thrown to notify that some of the xtalk samples did not yield
             * valid ranging pulse data while attempting to measure
             * the xtalk signal in vl53l1_run_xtalk_extract(,. This can signify any of
             * the zones are missing samples, for further debug information the
             * xtalk_results struct should be referred to. This warning is for
             * notification only, the xtalk pulse and shape have still been generated
             */
            WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT = - 39,
            /*!< Thrown to notify that some of teh xtalk samples used for gradient
             * generation did not yield valid ranging pulse data while attempting to
             * measure the xtalk signal in vl53l1_run_xtalk_extract(,. This can signify
             * that any one of the zones 0-3 yielded no successful samples. The
             * xtalk_results struct should be referred to for further debug info.
             * This warning is for notification only, the xtalk pulse and shape
             * have still been generated.
             */
            WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT = - 40,
            /*!< Thrown to notify that some of the xtalk samples used for gradient
             * generation did not pass the sigma limit check while attempting to
             * measure the xtalk signal in vl53l1_run_xtalk_extract(,. This can signify
             * that any one of the zones 0-3 yielded an avg sigma_mm value > the limit.
             * The xtalk_results struct should be referred to for further debug info.
             * This warning is for notification only, the xtalk pulse and shape
             * have still been generated.
             */

            ERROR_NOT_IMPLEMENTED = - 41,
            /*!< Tells requested functionality has not been implemented yet or
             * not compatible with the device */
            ERROR_PLATFORM_SPECIFIC_START = - 60,
            /*!< Tells the starting code for platform */
        };

    private:

        typedef const uint16_t reg_t;

        static reg_t SOFT_RESET                                                  = 0x0000;
        static reg_t I2C_SLAVE__DEVICE_ADDRESS                                   = 0x0001;
        static reg_t VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND                       = 0x0008;
        static reg_t ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS              = 0x0016;
        static reg_t ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS          = 0x0018;
        static reg_t ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS          = 0x001A;
        static reg_t ALGO__PART_TO_PART_RANGE_OFFSET_MM                          = 0x001E;
        static reg_t MM_CONFIG__INNER_OFFSET_MM                                  = 0x0020;
        static reg_t MM_CONFIG__OUTER_OFFSET_MM                                  = 0x0022;
        static reg_t GPIO_HV_MUX__CTRL                                           = 0x0030;
        static reg_t GPIO__TIO_HV_STATUS                                         = 0x0031;
        static reg_t SYSTEM__INTERRUPT_CONFIG_GPIO                               = 0x0046;
        static reg_t PHASECAL_CONFIG__TIMEOUT_MACROP                             = 0x004B;
        static reg_t RANGE_CONFIG__TIMEOUT_MACROP_A_HI                           = 0x005E;
        static reg_t RANGE_CONFIG__VCSEL_PERIOD_A                                = 0x0060;
        static reg_t RANGE_CONFIG__TIMEOUT_MACROP_B_HI                           = 0x0061;
        static reg_t RANGE_CONFIG__TIMEOUT_MACROP_B_LO                           = 0x0062;
        static reg_t RANGE_CONFIG__VCSEL_PERIOD_B                                = 0x0063;
        static reg_t RANGE_CONFIG__SIGMA_THRESH                                  = 0x0064;
        static reg_t RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS                 = 0x0066;
        static reg_t RANGE_CONFIG__VALID_PHASE_HIGH                              = 0x0069;
        static reg_t SYSTEM__INTERMEASUREMENT_PERIOD                             = 0x006C;
        static reg_t SYSTEM__THRESH_HIGH                                         = 0x0072;
        static reg_t SYSTEM__THRESH_LOW                                          = 0x0074;
        static reg_t SD_CONFIG__WOI_SD0                                          = 0x0078;
        static reg_t SD_CONFIG__INITIAL_PHASE_SD0                                = 0x007A;
        static reg_t ROI_CONFIG__USER_ROI_CENTRE_SPAD                            = 0x007F;
        static reg_t ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE               = 0x0080;
        static reg_t SYSTEM__SEQUENCE_CONFIG                                     = 0x0081;
        static reg_t SYSTEM__GROUPED_PARAMETER_HOLD                              = 0x0082;
        static reg_t SYSTEM__INTERRUPT_CLEAR                                     = 0x0086;
        static reg_t SYSTEM__MODE_START                                          = 0x0087;
        static reg_t RESULT__RANGE_STATUS                                        = 0x0089;
        static reg_t RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0                      = 0x008C;
        static reg_t RESULT__AMBIENT_COUNT_RATE_MCPS_SD                          = 0x0090;
        static reg_t RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0              = 0x0096;
        static reg_t RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0 = 0x0098;
        static reg_t RESULT__OSC_CALIBRATE_VAL                                   = 0x00DE;
        static reg_t FIRMWARE__SYSTEM_STATUS                                     = 0x00E5;
        static reg_t IDENTIFICATION__MODEL_ID                                    = 0x010F;
        static reg_t ROI_CONFIG__MODE_ROI_CENTRE_SPAD                            = 0x013E;

        static const uint8_t DEFAULT_DEVICE_ADDRESS = 0x52;

        enum {
            IMPLEMENTATION_VER_MAJOR = 1,
            IMPLEMENTATION_VER_MINOR = 0,
            IMPLEMENTATION_VER_SUB = 1,
            IMPLEMENTATION_VER_REVISION = 0
        };

        uint8_t DEFAULT_CONFIGURATION[91] = {

            // 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else
            // don't touch
            0x00, 

            // 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull
            // up at AVDD)
            0x01, 

            // 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1
            // (pull up at AVDD) //
            0x01, 

            // 0x30 : set bit 4 to 0 for active high interrupt and 1 for active
            // low (bits 3:0 must be 0x1), use SetInterruptPolarity()
            0x01, 

            // 0x31 : bit 1 = interrupt depending on the polarity, use
            // checkForDataReady()
            0x02, 

            0x00, // 0x32 : not user-modifiable
            0x02, // 0x33 : not user-modifiable
            0x08, // 0x34 : not user-modifiable
            0x00, // 0x35 : not user-modifiable
            0x08, // 0x36 : not user-modifiable
            0x10, // 0x37 : not user-modifiable
            0x01, // 0x38 : not user-modifiable
            0x01, // 0x39 : not user-modifiable
            0x00, // 0x3a : not user-modifiable
            0x00, // 0x3b : not user-modifiable
            0x00, // 0x3c : not user-modifiable
            0x00, // 0x3d : not user-modifiable
            0xff, // 0x3e : not user-modifiable
            0x00, // 0x3f : not user-modifiable
            0x0F, // 0x40 : not user-modifiable
            0x00, // 0x41 : not user-modifiable
            0x00, // 0x42 : not user-modifiable
            0x00, // 0x43 : not user-modifiable
            0x00, // 0x44 : not user-modifiable
            0x00, // 0x45 : not user-modifiable

            // 0x46 : interrupt configuration 0->level low detection, 1-> level
            // high, 2-> Out of window, 3->In window, 0x20-> New sample ready ,
            // TBC
            0x20, 

            0x0b, // 0x47 : not user-modifiable
            0x00, // 0x48 : not user-modifiable
            0x00, // 0x49 : not user-modifiable
            0x02, // 0x4a : not user-modifiable
            0x0a, // 0x4b : not user-modifiable
            0x21, // 0x4c : not user-modifiable
            0x00, // 0x4d : not user-modifiable
            0x00, // 0x4e : not user-modifiable
            0x05, // 0x4f : not user-modifiable
            0x00, // 0x50 : not user-modifiable
            0x00, // 0x51 : not user-modifiable
            0x00, // 0x52 : not user-modifiable
            0x00, // 0x53 : not user-modifiable
            0xc8, // 0x54 : not user-modifiable
            0x00, // 0x55 : not user-modifiable
            0x00, // 0x56 : not user-modifiable
            0x38, // 0x57 : not user-modifiable
            0xff, // 0x58 : not user-modifiable
            0x01, // 0x59 : not user-modifiable
            0x00, // 0x5a : not user-modifiable
            0x08, // 0x5b : not user-modifiable
            0x00, // 0x5c : not user-modifiable
            0x00, // 0x5d : not user-modifiable
            0x01, // 0x5e : not user-modifiable
            0xdb, // 0x5f : not user-modifiable
            0x0f, // 0x60 : not user-modifiable
            0x01, // 0x61 : not user-modifiable
            0xf1, // 0x62 : not user-modifiable
            0x0d, // 0x63 : not user-modifiable

            // 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use
            // SetSigmaThreshold(), default value 90 mm //
            0x01, 

            // 0x65 : Sigma threshold LSB //
            0x68, 

            // 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use
            // SetSignalThreshold()
            0x00, 

            // 0x67 : Min count Rate LSB //
            0x80, 

            0x08, // 0x68 : not user-modifiable
            0xb8, // 0x69 : not user-modifiable
            0x00, // 0x6a : not user-modifiable
            0x00, // 0x6b : not user-modifiable


            // 0x6c : Intermeasurement period MSB, 32 bits register, use
            // SetIntermeasurementInMs()
            0x00, 
            0x00, // 0x6d : Intermeasurement period
            0x0f, // 0x6e : Intermeasurement period
            0x89, // 0x6f : Intermeasurement period LSB
            0x00, // 0x70 : not user-modifiable
            0x00, // 0x71 : not user-modifiable

            // 0x72 : distance threshold high MSB (in mm, MSB+LSB), use
            // SetD:tanceThreshold()
            0x00, 

            0x00, // 0x73 : distance threshold high LSB 

            // 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use
            // SetDistanceThreshold() 
            0x00, 

            0x00, // 0x75 : distance threshold low LSB

            0x00, // 0x76 : not user-modifiable
            0x01, // 0x77 : not user-modifiable
            0x0f, // 0x78 : not user-modifiable
            0x0d, // 0x79 : not user-modifiable
            0x0e, // 0x7a : not user-modifiable
            0x0e, // 0x7b : not user-modifiable
            0x00, // 0x7c : not user-modifiable
            0x00, // 0x7d : not user-modifiable
            0x02, // 0x7e : not user-modifiable

            0xc7, // 0x7f : ROI center, use SetROI() //

            0xff, // 0x80 : XY ROI (X=Width, Y=Height), use SetROI()

            0x9B, // 0x81 : not user-modifiable
            0x00, // 0x82 : not user-modifiable
            0x00, // 0x83 : not user-modifiable
            0x00, // 0x84 : not user-modifiable
            0x01, // 0x85 : not user-modifiable

            0x00, // 0x86 : clear interrupt, use clearInterrupt()

            // 0x87 : start ranging, use startRanging() or stopRanging(), If
            // you want an automatic start after init() call, put 0x40 in
            // location 0x87
            0x00 
        };

    public:

        /** Constructor
         * @param[in] &i2c device I2C to be used for communication
         * @param[in] &pin_gpio1 pin Mbed InterruptIn PinName to be used as component 
                      GPIO_1 INT
         * @param[in] DevAddr device address, 0x52 by default
         */
        VL53L1X_Abstract(void)
        {
            _i2c_addr = DEFAULT_DEVICE_ADDRESS;
        }

        /** Destructor
         */
        virtual ~VL53L1X_Abstract()
        {
        }

        /**
         * @brief This function returns the SW driver version
         */
        void getSWVersion(version_t *pVersion)
        {
            pVersion->major = IMPLEMENTATION_VER_MAJOR;
            pVersion->minor = IMPLEMENTATION_VER_MINOR;
            pVersion->build = IMPLEMENTATION_VER_SUB;
            pVersion->revision = IMPLEMENTATION_VER_REVISION;
        }

        /**
         * @brief This function sets the sensor I2C address used in case
         * multiple devices application, default address 0x52
         */
        error_t setI2CAddress(const uint8_t new_i2c_addr)
        {
            error_t status = WrByte(I2C_SLAVE__DEVICE_ADDRESS, new_i2c_addr >> 1);

            _i2c_addr = new_i2c_addr;

            return status;
        }

        /**
         * @brief This function loads the 135 bytes default values to initialize 
         the sensor.
         * @param dev Device address
         * @return 0:success, != 0:failed
         */
        error_t begin()
        {
            error_t status = 0;
            uint8_t Addr = 0x00, dataReady = 0, timeout = 0;

            for (Addr = 0x2D; Addr <= 0x87; Addr++) {
                status = WrByte(Addr, DEFAULT_CONFIGURATION[Addr - 0x2D]);
            }
            status = startRanging();

            //We need to wait at least the default intermeasurement period of
            //103ms before dataready will occur But if a unit has already been
            //powered and polling, it may happen much faster
            while (dataReady == 0) {
                status = checkForDataReady(&dataReady);
                if (timeout++ > 150)
                    return ERROR_TIME_OUT;
                wait_ms(1);
            }
            status = clearInterrupt();
            status = stopRanging();

            // two bounds VHV
            status = WrByte(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); 

            // start VHV from the previous temperature
            status = WrByte(0x0B, 0); 

            return status;
        }

        /**
         * @brief This function clears the interrupt, to be called after a
         * ranging data reading to arm the interrupt for the next data ready
         * event.
         */
        error_t clearInterrupt()
        {

            return WrByte(SYSTEM__INTERRUPT_CLEAR, 0x01);
        }

        /**
         * @brief This function programs the interrupt polarity
         * 1=active high (default), 0=active low
         */
        error_t setInterruptPolarity(uint8_t NewPolarity)
        {
            uint8_t Temp = 0;
            auto status = RdByte(GPIO_HV_MUX__CTRL, &Temp);
            Temp = Temp & 0xEF;
            status |= WrByte(GPIO_HV_MUX__CTRL, Temp | (!(NewPolarity & 1)) << 4);
            return status;
        }

        /**
         * @brief This function returns the current interrupt polarity\n
         * 1=active high (default), 0=active low
         */
        error_t getInterruptPolarity(uint8_t *pInterruptPolarity)
        {
            uint8_t Temp = 0;
            auto status = RdByte(GPIO_HV_MUX__CTRL, &Temp);
            Temp = Temp & 0x10;
            *pInterruptPolarity = !(Temp >> 4);
            return status;
        }

        /**
         * @brief This function starts the ranging distance operation
         * The ranging operation is continuous. The clear interrupt has to be
         * done after each get data to allow the interrupt to raise when the
         * next data is ready

         * 1=active high (default), 0=active low, use SetInterruptPolarity() to
         * change the interrupt polarity if required.
         */
        error_t startRanging()
        {
            // clear interrupt trigger
            auto status = WrByte(SYSTEM__INTERRUPT_CLEAR, 0x01); 

            // Enable VL53L1X
            status |= WrByte(SYSTEM__MODE_START, 0x40); 

            return status;
        }

        /**
         * @brief This function starts a one-shot ranging distance operation\n
         */
        error_t startOneshotRanging()
        {
            // clear interrupt trigger
            auto status = WrByte(SYSTEM__INTERRUPT_CLEAR, 0x01); 
            
            // Enable VL53L1X one-shot ranging
            status |= WrByte(SYSTEM__MODE_START, 0x10); 
            return status;
        }

        /**
         * @brief This function stops the ranging.
         */
        error_t stopRanging()
        {
            return WrByte(SYSTEM__MODE_START, 0x00); // Disable VL53L1X
        }

        /**
         * @brief This function checks if the new ranging data is available by
         * polling the dedicated register.
         * @param : isDataReady==0 -> not ready; isDataReady==1 -> ready
         */
        error_t checkForDataReady(uint8_t *isDataReady)
        {
            uint8_t IntPol = 0;
            auto status = getInterruptPolarity(&IntPol);

            uint8_t Temp = 0;
            status |= RdByte(GPIO__TIO_HV_STATUS, &Temp);

            /* Read in the register to check if a new value is available */
            if (status == 0) {
                *isDataReady = (Temp & 1) == IntPol ? 1 : 0;
            }

            return status;
        }

        /**
         * @brief This function programs the timing budget in ms.
         * Predefined values = 15, 20, 33, 50, 100(default), 200, 500.
         */
        error_t setTimingBudgetInMs(uint16_t TimingBudgetInMs)
        {
            uint16_t DM = 0;

            auto status = getDistanceMode(&DM);

            if (DM == 0) {
                return 1;
            }

            if (DM == 1)
            { // Short DistanceMode
                switch (TimingBudgetInMs)
                {
                    case 15: /* only available in short distance mode */
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x01D);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0027);
                        break;

                    case 20:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0051);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
                        break;

                    case 33:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x00D6);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x006E);
                        break;

                    case 50:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x1AE);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x01E8);
                        break;

                    case 100:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x02E1);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0388);
                        break;

                    case 200:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x03E1);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x0496);
                        break;

                    case 500:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, 0x0591);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI, 0x05C1);
                        break;

                    default:
                        status = 1;
                        break;
                }
            }
            else
            {
                switch (TimingBudgetInMs)
                {
                    case 20:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                                0x001E);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                                0x0022);
                        break;
                    case 33:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                                0x0060);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                                0x006E);
                        break;
                    case 50:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                                0x00AD);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                                0x00C6);
                        break;
                    case 100:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                                0x01CC);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                                0x01EA);
                        break;
                    case 200:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                                0x02D9);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                                0x02F8);
                        break;
                    case 500:
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI,
                                0x048F);
                        WrWord(RANGE_CONFIG__TIMEOUT_MACROP_B_HI,
                                0x04A4);
                        break;
                    default:
                        status = 1;
                        break;
                }
            }
            return status;
        }


        /**
         * @brief This function returns the current timing budget in ms.
         */
        error_t getTimingBudgetInMs(uint16_t *pTimingBudget)
        {
            uint16_t Temp = 0;
            auto status = RdWord(RANGE_CONFIG__TIMEOUT_MACROP_A_HI, &Temp);

            switch (Temp)
            {
                case 0x001D:
                    *pTimingBudget = 15;
                    break;

                case 0x0051:
                case 0x001E:
                    *pTimingBudget = 20;
                    break;

                case 0x00D6:
                case 0x0060:
                    *pTimingBudget = 33;
                    break;

                case 0x1AE:
                case 0x00AD:
                    *pTimingBudget = 50;
                    break;

                case 0x02E1:
                case 0x01CC:
                    *pTimingBudget = 100;
                    break;

                case 0x03E1:
                case 0x02D9:
                    *pTimingBudget = 200;
                    break;

                case 0x0591:
                case 0x048F:
                    *pTimingBudget = 500;
                    break;

                default:
                    *pTimingBudget = 0;
                    break;
            }

            return status;
        }


        /**
         * @brief This function programs the distance mode (1=short, 2=long(default)).
         * Short mode max distance is limited to 1.3 m but better ambient immunity.\n
         * Long mode can range up to 4 m in the dark with 200 ms timing budget.
         */
        error_t setDistanceMode(uint16_t DM)
        {
            uint16_t TB = 0;

            auto status = getTimingBudgetInMs(&TB);

            switch (DM)
            {
                case 1:
                    status |= WrByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x14);
                    status |= WrByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
                    status |= WrByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
                    status |= WrByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
                    status |= WrWord(SD_CONFIG__WOI_SD0, 0x0705);
                    status |= WrWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0606);
                    break;

                case 2:
                    status |= WrByte(PHASECAL_CONFIG__TIMEOUT_MACROP, 0x0A);
                    status |= WrByte(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
                    status |= WrByte(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
                    status |= WrByte(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
                    status |= WrWord(SD_CONFIG__WOI_SD0, 0x0F0D);
                    status |= WrWord(SD_CONFIG__INITIAL_PHASE_SD0, 0x0E0E);
                    break;
                default:
                    break;
            }

            return status | setTimingBudgetInMs(TB);
        }


        /**
         * @brief This function returns the current distance mode (1=short, 2=long).
         */
        error_t getDistanceMode(uint16_t *DM)
        {
            uint8_t TempDM = 0;

            auto status = RdByte(PHASECAL_CONFIG__TIMEOUT_MACROP, &TempDM);

            if (TempDM == 0x14) {
                *DM = 1;
            }

            if (TempDM == 0x0A) {
                *DM = 2;
            }

            return status;
        }


        /**
         * @brief This function programs the Intermeasurement period in ms\n
         * Intermeasurement period must be >/= timing budget. This condition is
         * not checked by the API, the customer has the duty to check the
         * condition. Default = 100 ms
         */
        error_t setInterMeasurementInMs(uint16_t InterMeasMs)
        {
            uint16_t ClockPLL = 0;

            auto status = RdWord(RESULT__OSC_CALIBRATE_VAL, &ClockPLL);

            ClockPLL = ClockPLL & 0x3FF;

            WrDWord(SYSTEM__INTERMEASUREMENT_PERIOD,
                    (uint32_t)(ClockPLL * InterMeasMs * 1.075));

            return status;
        }


        /**
         * @brief This function returns the Intermeasurement period in ms.
         */
        error_t getInterMeasurementInMs(uint16_t *pIM)
        {
            uint16_t ClockPLL = 0;
            uint32_t tmp = 0;

            auto status = RdDWord(SYSTEM__INTERMEASUREMENT_PERIOD, &tmp);
            *pIM = (uint16_t)tmp;
            status = RdWord(RESULT__OSC_CALIBRATE_VAL, &ClockPLL);
            ClockPLL = ClockPLL & 0x3FF;
            *pIM = (uint16_t)(*pIM / (ClockPLL * 1.065));
            return status;
        }


        /**
         * @brief This function returns the boot state of the device (1:booted,
         * 0:not booted)
         */
        error_t bootState(uint8_t *state)
        {
            uint8_t tmp = 0;

            auto status = RdByte(FIRMWARE__SYSTEM_STATUS, &tmp);
            *state = tmp;
            return status;
        }


        /**
         * @brief This function returns the sensor id, sensor Id must be 0xEEAC
         */
        error_t getSensorId(uint16_t *sensorId)
        {
            uint16_t tmp = 0;

            auto status = RdWord(IDENTIFICATION__MODEL_ID, &tmp);
            *sensorId = tmp;
            return status;
        }


        /**
         * @brief This function returns the distance measured by the sensor in mm
         */
        error_t getDistance(uint16_t *distance)
        {
            uint16_t tmp = 0;

            auto status = (RdWord(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &tmp));
            *distance = tmp;
            return status;
        }


        /**
         * @brief This function returns the returned signal per SPAD in kcps/SPAD.
         * With kcps stands for Kilo Count Per Second
         */
        error_t getSignalPerSpad(uint16_t *signalRate)
        {
            uint16_t SpNb = 1, signal = 0;

            auto status = RdWord(
                    RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &signal);
            status = RdWord(
                    RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
            *signalRate = (uint16_t)(2000.0 * signal / SpNb);
            return status;
        }


        /**
         * @brief This function returns the ambient per SPAD in kcps/SPAD
         */
        error_t getAmbientPerSpad(uint16_t *ambPerSp)
        {
            uint16_t AmbientRate = 0, SpNb = 1;

            auto status = RdWord(RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &AmbientRate);
            status |= RdWord(RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &SpNb);
            *ambPerSp = (uint16_t)(2000.0 * AmbientRate / SpNb);
            return status;
        }


        /**
         * @brief This function returns the returned signal in kcps.
         */
        error_t getSignalRate(uint16_t *signal)
        {
            uint16_t tmp = 0;

            auto status = RdWord(
                    RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0, &tmp);
            *signal = tmp * 8;
            return status;
        }


        /**
         * @brief This function returns the current number of enabled SPADs
         */
        error_t getSpadNb(uint16_t *spNb)
        {
            uint16_t tmp = 0;

            auto status = RdWord(
                    RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0, &tmp);
            *spNb = tmp >> 8;
            return status;
        }


        /**
         * @brief This function returns the ambient rate in kcps
         */
        error_t getAmbientRate(uint16_t *ambRate)
        {
            uint16_t tmp = 0;

            auto status = RdWord(RESULT__AMBIENT_COUNT_RATE_MCPS_SD, &tmp);
            *ambRate = tmp * 8;
            return status;
        }


        /**
         * @brief This function returns the ranging status error \n
         * (0:no error, 1:sigma failed, 2:signal failed, ..., 7:wrap-around)
         */
        error_t getRangeStatus(uint8_t *rangeStatus)
        {
            uint8_t RgSt = 0;

            auto status = RdByte(RESULT__RANGE_STATUS, &RgSt);

            RgSt = RgSt & 0x1F;

            switch (RgSt)
            {
                case 9:
                    RgSt = 0;
                    break;
                case 6:
                    RgSt = 1;
                    break;
                case 4:
                    RgSt = 2;
                    break;
                case 8:
                    RgSt = 3;
                    break;
                case 5:
                    RgSt = 4;
                    break;
                case 3:
                    RgSt = 5;
                    break;
                case 19:
                    RgSt = 6;
                    break;
                case 7:
                    RgSt = 7;
                    break;
                case 12:
                    RgSt = 9;
                    break;
                case 18:
                    RgSt = 10;
                    break;
                case 22:
                    RgSt = 11;
                    break;
                case 23:
                    RgSt = 12;
                    break;
                case 13:
                    RgSt = 13;
                    break;
                default:
                    RgSt = 255;
                    break;
            }
            *rangeStatus = RgSt;
            return status;
        }


        /**
         * @brief This function programs the offset correction in mm
         * @param OffsetValue:the offset correction value to program in mm
         */
        error_t setOffset(int16_t OffsetValue)
        {
            int16_t Temp = 0;

            Temp = (OffsetValue * 4);

            auto status = WrWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, (uint16_t)Temp);
            status |= WrWord(MM_CONFIG__INNER_OFFSET_MM, 0x0);
            status |= WrWord(MM_CONFIG__OUTER_OFFSET_MM, 0x0);

            return status;
        }


        /**
         * @brief This function returns the programmed offset correction value in mm
         */
        error_t getOffset(int16_t *offset)
        {
            uint16_t Temp = 0;

            auto status = RdWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, &Temp);
            Temp = Temp << 3;
            Temp = Temp >> 5;
            *offset = (int16_t)(Temp);
            return status;
        }


        /**
         * @brief This function programs the xtalk correction value in cps
         * (Count Per Second).
         * This is the number of photons reflected back from the cover glass in cps.
         */
        error_t setXtalk(uint16_t XtalkValue)
        {
            /* XTalkValue in count per second to avoid float type */
            auto status = WrWord(ALGO__CROSSTALK_COMPENSATION_X_PLANE_GRADIENT_KCPS,
                    0x0000);

            status |= WrWord(ALGO__CROSSTALK_COMPENSATION_Y_PLANE_GRADIENT_KCPS,
                    0x0000);

            // * << 9 (7.9 format) and /1000 to convert cps to kpcs
            status |= WrWord(ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS,
                    (XtalkValue << 9) / 1000); 

            return status;
        }


        /**
         * @brief This function returns the current programmed xtalk correction
         * value in cps
         */
        error_t getXtalk(uint16_t *xtalk)
        {
            uint16_t tmp = 0;

            auto status = RdWord(ALGO__CROSSTALK_COMPENSATION_PLANE_OFFSET_KCPS, &tmp);


            // * 1000 to convert kcps to cps and >> 9 (7.9 format)
            *xtalk = (tmp * 1000) >> 9; 
            return status;
        }

        /**
         * @brief This function programs the threshold detection mode\n
         * Example:\n
         * SetDistanceThreshold(dev,100,300,0,1): Below 100 \n
         * SetDistanceThreshold(dev,100,300,1,1): Above 300 \n
         * SetDistanceThreshold(dev,100,300,2,1): Out of window \n
         * SetDistanceThreshold(dev,100,300,3,1): In window \n
         * @param dev : device address
         * @param ThreshLow(in mm) : the threshold under which one the device
         * raises an interrupt if Window = 0
         * @param ThreshHigh(in mm) : the threshold above which one the
         * device raises an interrupt if Window = 1
         * @param Window detection mode : 0=below, 1=above, 2=out, 3=in
         * @param IntOnNoTarget = 1 (No longer used - just use 1)
         */
        error_t setDistanceThreshold(uint16_t ThreshLow,
                uint16_t ThreshHigh, uint8_t Window,
                uint8_t IntOnNoTarget)
    {
        uint8_t Temp = 0;

        auto status = RdByte(SYSTEM__INTERRUPT_CONFIG_GPIO, &Temp);
        Temp = Temp & 0x47;
        if (IntOnNoTarget == 0) {
            status |= WrByte(SYSTEM__INTERRUPT_CONFIG_GPIO,
                    (Temp | (Window & 0x07)));
        }
        else {
            status |= WrByte(SYSTEM__INTERRUPT_CONFIG_GPIO,
                    ((Temp | (Window & 0x07)) | 0x40));
        }
        status |= WrWord(SYSTEM__THRESH_HIGH, ThreshHigh);
        status |= WrWord(SYSTEM__THRESH_LOW, ThreshLow);

        return status;
    }


        /**
         * @brief This function returns the window detection mode (0=below;
         * 1=above; 2=out; 3=in)
         */
        error_t getDistanceThresholdWindow(uint16_t *window)
        {
            uint8_t tmp = 0;
            auto status = RdByte(SYSTEM__INTERRUPT_CONFIG_GPIO, &tmp);
            *window = (uint16_t)(tmp & 0x7);
            return status;
        }


        /**
         * @brief This function returns the low threshold in mm
         */
        error_t getDistanceThresholdLow(uint16_t *low)
        {
            uint16_t tmp = 0;

            auto status = RdWord(SYSTEM__THRESH_LOW, &tmp);
            *low = tmp;
            return status;
        }


        /**
         * @brief This function returns the high threshold in mm
         */
        error_t getDistanceThresholdHigh(uint16_t *high)
        {
            uint16_t tmp = 0;

            auto status = RdWord(SYSTEM__THRESH_HIGH, &tmp);
            *high = tmp;
            return status;
        }


        /**
         * @brief This function programs the ROI (Region of Interest)\n
         * The ROI position is centered, only the ROI size can be reprogrammed.\n
         * The smallest acceptable ROI size = 4\n
         * @param X:ROI Width; Y=ROI Height
         */
        error_t setROI(uint8_t X, uint8_t Y, uint8_t opticalCenter)
        {

            if (X > 16)
                X = 16;
            if (Y > 16)
                Y = 16;
            if (X > 10 || Y > 10)
            {
                opticalCenter = 199;
            }
            auto status = WrByte(ROI_CONFIG__USER_ROI_CENTRE_SPAD, opticalCenter);
            status |= WrByte(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE,
                    (Y - 1) << 4 | (X - 1));

            return status;
        }

        /**
         *@brief This function returns width X and height Y
         */
        error_t getROI_XY(uint16_t *ROI_X, uint16_t *ROI_Y)
        {
            uint8_t tmp = 0;

            auto status = RdByte(ROI_CONFIG__USER_ROI_REQUESTED_GLOBAL_XY_SIZE, &tmp);

            *ROI_X = ((uint16_t)tmp & 0x0F) + 1;
            *ROI_Y = (((uint16_t)tmp & 0xF0) >> 4) + 1;

            return status;
        }


        /**
         * @brief This function programs a new signal threshold in kcps
         * (default=1024 kcps\n
         */
        error_t setSignalThreshold(uint16_t Signal)
        {
            return WrWord(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, Signal >> 3);
        }


        /**
         * @brief This function returns the current signal threshold in kcps
         */
        error_t getSignalThreshold(uint16_t *signal)
        {
            uint16_t tmp = 0;

            auto status = RdWord(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, &tmp);
            *signal = tmp << 3;
            return status;
        }


        /**
         * @brief This function programs a new sigma threshold in mm (default=15 mm)
         */
        error_t setSigmaThreshold(uint16_t Sigma)
        {
            if (Sigma > (0xFFFF >> 2)) {
                return 1;
            }

            /* 16 bits register 14.2 format */
            return WrWord(RANGE_CONFIG__SIGMA_THRESH, Sigma << 2);
        }


        /**
         * @brief This function returns the current sigma threshold in mm
         */
        error_t getSigmaThreshold(uint16_t *sigma)
        {
            uint16_t tmp = 0;

            auto status = RdWord(RANGE_CONFIG__SIGMA_THRESH, &tmp);
            *sigma = tmp >> 2;
            return status;
        }


        /**
         * @brief This function performs the temperature calibration.
         * It is recommended to call this function any time the temperature
         * might have changed by more than 8 deg C without sensor ranging
         * activity for an extended period.
         */
        error_t startTemperatureUpdate()
        {
            uint8_t tmp = 0;


            // full VHV 
            auto status = WrByte(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x81); 
            status |= WrByte(0x0B, 0x92);
            status |= startRanging();
            while (tmp == 0)
            {
                status |= checkForDataReady(&tmp);
            }
            tmp = 0;
            status |= clearInterrupt();
            status |= stopRanging();

            // two bounds VHV
            status |= WrByte(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09); 

            status |= WrByte(0x0B, 0); // start VHV from the previous temperature */

            return status;
        }


        /* calibration.h functions */

        /**
         * @brief This function performs the offset calibration.\n
         * The function returns the offset value found and programs the offset
         * compensation into the device.
         * @param TargetDistInMm target distance in mm, ST recommended 100 mm
         * Target reflectance = grey17%
         * @return 0:success, !=0: failed
         * @return offset pointer contains the offset found in mm
         */
        int8_t calibrateOffset(uint16_t TargetDistInMm, int16_t *offset)
        {
            uint8_t i = 0, tmp = 0;
            int16_t AverageDistance = 0;
            uint16_t distance = 0;

            auto status = WrWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
            status |= WrWord(MM_CONFIG__INNER_OFFSET_MM, 0x0);
            status |= WrWord(MM_CONFIG__OUTER_OFFSET_MM, 0x0);
            status |= startRanging();
            for (i = 0; i < 50; i++)
            {
                while (tmp == 0)
                {
                    status |= checkForDataReady(&tmp);
                }
                tmp = 0;
                status |= getDistance(&distance);
                status |= clearInterrupt();
                AverageDistance = AverageDistance + distance;
            }
            status |= stopRanging();
            AverageDistance = AverageDistance / 50;
            *offset = TargetDistInMm - AverageDistance;
            status |= WrWord(ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset * 4);
            return status;
        }


        /**
         * @brief This function performs the xtalk calibration.
         * The function returns the xtalk value found and programs the xtalk
         * compensation to the device
         * @param TargetDistInMm target distance in mm\n
         * The target distance : the distance where the sensor start to "under
         * range"\n due to the influence of the photons reflected back from the
         * cover glass becoming strong\n
         * It's also called inflection point
         * Target reflectance = grey 17%
         * @return 0: success, !=0: failed
         * @return xtalk pointer contains the xtalk value found in cps (number
         * of photons in count per second)
         */
        int8_t calibrateXtalk(uint16_t TargetDistInMm, uint16_t *xtalk)
        {
            uint8_t i = 0, tmp = 0;
            float AverageSignalRate = 0;
            float AverageDistance = 0;
            float AverageSpadNb = 0;
            uint16_t distance = 0, spadNum;
            uint16_t sr = 0;
            error_t status = 0;

            status = WrWord(0x0016, 0);
            status = startRanging();
            for (i = 0; i < 50; i++)
            {
                while (tmp == 0)
                {
                    status = checkForDataReady(&tmp);
                }
                tmp = 0;
                status = getSignalRate(&sr);
                status = getDistance(&distance);
                status = clearInterrupt();
                AverageDistance = AverageDistance + distance;
                status = getSpadNb(&spadNum);
                AverageSpadNb = AverageSpadNb + spadNum;
                AverageSignalRate =
                    AverageSignalRate + sr;
            }
            status = stopRanging();
            AverageDistance = AverageDistance / 50;
            AverageSpadNb = AverageSpadNb / 50;
            AverageSignalRate = AverageSignalRate / 50;
            /* Calculate Xtalk value */
            *xtalk = (uint16_t)(512 * (AverageSignalRate * (1 - (AverageDistance / TargetDistInMm))) / AverageSpadNb);
            status = WrWord(0x0016, *xtalk);
            return status;
        }

        uint8_t _i2c_addr;

    protected:

        virtual error_t i2c_write( const uint16_t addr, const uint16_t rgstr,
                uint8_t *buff, const uint16_t nbytes) = 0;

        virtual error_t i2c_read(const uint16_t addr, const uint16_t rgstr,
                uint8_t *buff, const uint16_t nbytes) = 0;

        virtual void wait_ms(const int32_t wait_ms) = 0;

    private:

        // Write and read functions from I2C

        error_t WriteMulti(uint16_t index, uint8_t *pdata, uint32_t count)
        {
            return i2c_write(_i2c_addr, index, pdata, (uint16_t)count);
        }

        error_t ReadMulti(uint16_t index, uint8_t *pdata, uint32_t count)
        {
            return i2c_read(_i2c_addr, index, pdata, (uint16_t)count);
        }

        error_t WrByte(uint16_t index, uint8_t data)
        {
            return i2c_write(_i2c_addr, index, &data, 1);
        }

        error_t WrWord(uint16_t index, uint16_t data)
        {
            uint8_t buffer[2] = {};

            buffer[0] = data >> 8;
            buffer[1] = data & 0x00FF;
            return i2c_write(_i2c_addr, index, (uint8_t *)buffer, 2);
        }

        error_t WrDWord(uint16_t index, uint32_t data)
        {
            uint8_t buffer[4] = {};

            buffer[0] = (data >> 24) & 0xFF;
            buffer[1] = (data >> 16) & 0xFF;
            buffer[2] = (data >> 8) & 0xFF;
            buffer[3] = (data >> 0) & 0xFF;
            return i2c_write(_i2c_addr, index, (uint8_t *)buffer, 4);
        }

        error_t RdByte(uint16_t index, uint8_t *data)
        {
            return i2c_read(_i2c_addr, index, data, 1) ? -1 : 0;
        }

        error_t RdWord(uint16_t index, uint16_t *data)
        {
            uint8_t buffer[2] = {};

            auto status = i2c_read(_i2c_addr, index, buffer, 2);

            if (!status) {
                *data = (buffer[0] << 8) + buffer[1];
            }
            return status;
        }

        error_t RdDWord(uint16_t index, uint32_t *data)
        {
            uint8_t buffer[4] = {};

            auto status = i2c_read(_i2c_addr, index, buffer, 4);
            if (!status) {
                *data = (buffer[0] << 24) + (buffer[1] << 16U) + (buffer[2] << 8) + 
                    buffer[3]; 
            }

            return status;
        }

        error_t UpdateByte(uint16_t index, uint8_t AndData, uint8_t OrData)
        {
            uint8_t buffer = 0;

            /* read data direct onto buffer */
            auto status = i2c_read(_i2c_addr, index, &buffer, 1);
            if (!status) {
                buffer = (buffer & AndData) | OrData;
                status = i2c_write(_i2c_addr, index, &buffer, (uint16_t)1);
            }
            return status;
        }

        error_t getTickCount(
                uint32_t *ptick_count_ms)
        {

            /* Returns current tick count in [ms] */

            error_t status = ERROR_NONE;

            //*ptick_count_ms = timeGetTime();
            *ptick_count_ms = 0;

            return status;
        }

        error_t waitValueMaskEx(
                uint32_t timeout_ms,
                uint16_t index,
                uint8_t value,
                uint8_t mask,
                uint32_t poll_delay_ms)
        {

            /*
             * Platform implementation of WaitValueMaskEx V2WReg script command
             *
             * WaitValueMaskEx(
             * duration_ms,
             * index,
             * value,
             * mask,
             * poll_delay_ms);
             */

            int8_t status = ERROR_NONE;
            uint32_t start_time_ms = 0;
            uint32_t current_time_ms = 0;
            uint32_t polling_time_ms = 0;
            uint8_t byte_value = 0;
            uint8_t found = 0;

            /* calculate time limit in absolute time */

            getTickCount(&start_time_ms);

            /* remember current trace functions and temporarily disable
             * function logging
             */

            /* wait until value is found, timeout reached on error occurred */

            while ((status == ERROR_NONE) &&
                    (polling_time_ms < timeout_ms) &&
                    (found == 0))
            {

                if (status == ERROR_NONE) {
                    status = RdByte(index, &byte_value);
                }

                if ((byte_value & mask) == value) {
                    found = 1;
                }

                if (status == ERROR_NONE && found == 0 && poll_delay_ms > 0) {
                    wait_ms(poll_delay_ms);
                }

                /* Update polling time (Compare difference rather than absolute to
                   negate 32bit wrap around issue) */
                getTickCount(&current_time_ms);
                polling_time_ms = current_time_ms - start_time_ms;
            }

            if (found == 0 && status == ERROR_NONE)
                status = ERROR_TIME_OUT;

            return status;
        }

}; // class VL53L1X
