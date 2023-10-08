#pragma once

/*
   VL53L1X class

   Copyright (c) 2017, STMicroelectronics
   Copyright (c) 2023 Simon D. Levy

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

#include <stdint.h>

#include "i2c_helpers.h"

class VL53L1X {

    public:

        enum {
            DISTANCEMODE_SHORT = 1,
            DISTANCEMODE_MEDIUM,
            DISTANCEMODE_LONG, 
        };

        enum {
            ERROR_NONE                              =  0,
            ERROR_CALIBRATION_WARNING               = -1,
            ERROR_MIN_CLIPPED                       = -2,
            ERROR_UNDEFINED                         = -3,
            ERROR_INVALID_PARAMS                    = -4,
            ERROR_NOT_SUPPORTED                     = -5,
            ERROR_RANGE_ERROR                       = -6,
            ERROR_TIME_OUT                          = -7,
            ERROR_MODE_NOT_SUPPORTED                = -8,
            ERROR_BUFFER_TOO_SMALL                  = -9,
            ERROR_COMMS_BUFFER_TOO_SMALL            = -10,
            ERROR_GPIO_NOT_EXISTING                 = -11,
            ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED  = -12,
            ERROR_CONTROL_INTERFACE                 = -13,
            ERROR_INVALID_COMMAND                   = -14,
            ERROR_DIVISION_BY_ZERO                  = -15,
            ERROR_REF_SPAD_INIT                     = -16,
            ERROR_GPH_SYNC_CHECK_FAIL               = -17,
            ERROR_STREAM_COUNT_CHECK_FAIL           = -18,
            ERROR_GPH_ID_CHECK_FAIL                 = -19,
            ERROR_ZONE_STREAM_COUNT_CHECK_FAIL      = -20,
            ERROR_ZONE_GPH_ID_CHECK_FAIL            = -21,
            ERROR_XTALK_EXTRACTION_NO_SAMPLE_FAIL   = -22,
            ERROR_XTALK_EXTRACTION_SIGMA_LIMIT_FAIL = -23,
            ERROR_OFFSET_CAL_NO_SAMPLE_FAIL         = -24,
            ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL  = -25,
            ERROR_ZONE_CAL_NO_SAMPLE_FAIL           = -26,
            ERROR_TUNING_PARAM_KEY_MISMATCH          = -27,
            WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS  = -28,
            WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH     = -29,
            WARNING_REF_SPAD_CHAR_RATE_TOO_LOW      = -30,
            WARNING_OFFSET_CAL_MISSING_SAMPLES      = -31,
            WARNING_OFFSET_CAL_SIGMA_TOO_HIGH       = -32,
            WARNING_OFFSET_CAL_RATE_TOO_HIGH        = -33,
            WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW   = -34,
            WARNING_ZONE_CAL_MISSING_SAMPLES        = -35,
            WARNING_ZONE_CAL_SIGMA_TOO_HIGH         = -36,
            WARNING_ZONE_CAL_RATE_TOO_HIGH          = -37,
            WARNING_XTALK_MISSING_SAMPLES           = -38,
            WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT   = -39,
            WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT  = -40,
            ERROR_NOT_IMPLEMENTED                   = -41,
            ERROR_PLATFORM_SPECIFIC_START           = -60,
            ERROR_DEVICE_FIRMWARE_TOO_OLD           = -80,
            ERROR_DEVICE_FIRMWARE_TOO_NEW           = -85,
            ERROR_UNIT_TEST_FAIL                    = -90,
            ERROR_FILE_READ_FAIL                    = -95,
            ERROR_FILE_WRITE_FAIL                   = -96,
        };

        typedef int8_t error_t;

        typedef uint8_t distanceMode_t;

        error_t begin(
                const void * device=NULL, 
                const uint8_t addr=0x29,
                const distanceMode_t distanceMode=DISTANCEMODE_MEDIUM,
                const uint32_t timingBudgetMsec=25)
        {
            _device = (void *)device;

            _i2c_address = 0x29;

            auto status = write_byte(RGSTR_I2C_ADDRESS, addr);

            _i2c_address = addr;

            init_ll_driver_state(DEVICESTATE_UNKNOWN);

            _wait_method       = WAIT_METHOD_BLOCKING;
            _preset_mode       = DEVICEPRESETMODE_STANDARD_RANGING;
            _measurement_mode  = DEVICEMEASUREMENTMODE_STOP;

            _offset_calibration_mode =
                OFFSETCALIBRATIONMODE_MM1_MM2_STANDARD;
            _offset_correction_mode  =
                OFFSETCORRECTIONMODE_MM1_MM2_OFFSETS;

            _phasecal_config_timeout_us  =  1000;
            _mm_config_timeout_us        =  2000;
            _range_config_timeout_us     = 13000;
            _inter_measurement_period_ms =   100;
            _dss_config_target_total_rate_mcps = 0x0A00;
            _debug_mode                  =  0x00;

            _gain_cal.standard_ranging_gain_factor =
                TUNINGPARAM_LITE_RANGING_GAIN_FACTOR_DEFAULT;

            if (status == ERROR_NONE)
                status = get_static_nvm_managed(&(_stat_nvm)); 

            if (status == ERROR_NONE)
                status = get_customer_nvm_managed(&(_customer));

            if (status == ERROR_NONE) {
                status = get_nvm_copy_data(&(_nvm_copy_data));

                if (status == ERROR_NONE)
                    copy_rtn_good_spads_to_buffer( &(_nvm_copy_data),
                            &(_rtn_good_spads[0])); 
            }

            if (status == ERROR_NONE)
                status = RdWord(RGSTR_RESULT_OSC_CALIBRATE_VAL,
                        &(_dbg_results.result_osc_calibrate_val));

            if (_stat_nvm.osc_measured_fast_osc_frequency < 0x1000) {
                _stat_nvm.osc_measured_fast_osc_frequency = 0xBCCC;
            }

            if (status == ERROR_NONE) {
                get_mode_mitigation_roi(&(_mm_roi));
            }

            if (_optical_centre.x_centre == 0 && _optical_centre.y_centre == 0) {
                _optical_centre.x_centre = _mm_roi.x_centre << 4;
                _optical_centre.y_centre = _mm_roi.y_centre << 4;
            }

            init_refspadchar_config_struct( &(_refspadchar));

            status = init_ssc_config_struct( &(_ssc_cfg));

            init_xtalk_config_struct();

            status = init_offset_cal_config_struct( &(_offsetcal_cfg));

            status = init_tuning_parm_storage_struct(&_tuning_params);

            set_vhv_loopbound(TUNINGPARAM_VHV_LOOPBOUND_DEFAULT);

            if (status == ERROR_NONE)
                status = set_preset_mode(
                        _preset_mode,
                        _dss_config_target_total_rate_mcps,
                        _phasecal_config_timeout_us,
                        _mm_config_timeout_us,
                        _range_config_timeout_us,
                        _inter_measurement_period_ms);

            low_power_auto_data_init();

            if (status == ERROR_NONE) {

                _current_parameters.presetMode = PRESETMODE_LOWPOWER_AUTONOMOUS;
            }

            for (uint8_t i=0; i<CHECKENABLE_NUMBER_OF_CHECKS; i++) {
                if (status == ERROR_NONE)
                    status = set_limit_check_enable(i, 1);
                else
                    break;
            }

            if (status == ERROR_NONE) {
                status = set_limit_check_value(
                        CHECKENABLE_SIGMA_FINAL_RANGE,
                        (fixed_point_1616_t)(18 * 65536));
            }

            if (status == ERROR_NONE) {
                status = set_limit_check_value(
                        CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                        (fixed_point_1616_t)(25 * 65536 / 100));
            }

            uint8_t measurement_mode  = DEVICEMEASUREMENTMODE_BACKTOBACK;

            _measurement_mode = measurement_mode;

            _current_parameters.new_distance_mode = DISTANCEMODE_LONG;

            _current_parameters.internal_distance_mode = DISTANCEMODE_LONG;

            _current_parameters.distance_mode = DISTANCEMODE_LONG;

            distanceMode_t distance_mode = DISTANCEMODE_LONG;

            preset_mode_t presetMode = PRESETMODE_LOWPOWER_AUTONOMOUS;

            status = helper_set_preset_mode(presetMode, distance_mode, 1000);

            if (status == ERROR_NONE) {

                _current_parameters.internal_distance_mode = distance_mode;

                _current_parameters.new_distance_mode = distance_mode;

                if ((presetMode == PRESETMODE_LITE_RANGING) ||
                        (presetMode == PRESETMODE_AUTONOMOUS) ||
                        (presetMode == PRESETMODE_LOWPOWER_AUTONOMOUS))
                    status = set_measurement_timing_budget_usec(41000);
                else
                    status = set_measurement_timing_budget_usec(33333);
            }

            if (status == ERROR_NONE) {
                status = set_inter_measurement_period_ms(1000);
            }

            for (uint16_t rgstr = 0x002D; rgstr <= 0x0087; rgstr++) {
                status |= write_byte(rgstr, DEFAULT_CONFIGURATION[rgstr - 0x002D]);
            }

            status |= write_byte(RGSTR_SYSTEM_INTERRUPT_CLEAR, 0x01); 
            status |= write_byte(RGSTR_SYSTEM_MODE_START, 0x40); 
            status |= write_byte(RGSTR_SYSTEM_INTERRUPT_CLEAR, 0x01);
            status |= write_byte(RGSTR_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09); 

            status |= write_byte(0x0B, 0);											

            status |= set_measurement_timing_budget_usec(timingBudgetMsec * 1000);

            status |= set_distance_mode(distanceMode);

            return status; 

        }

        error_t readDistance(uint16_t * distance)
        {
            start_ranging();

            while (true) {

                auto dataReady = false;

                check_for_data_ready(&dataReady);

                if (dataReady) {
                    break;
                }

                delay_msec(1);
            }

            get_distance(distance);

            stop_ranging();

            return ERROR_NONE;
        }

    private:

        // Constants ---------------------------------------------------------

        static const uint8_t STATIC_NVM_MANAGED_I2C_SIZE_BYTES           = 11;
        static const uint8_t CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES         = 23;
        static const uint8_t STATIC_CONFIG_I2C_SIZE_BYTES                = 32;
        static const uint8_t GENERAL_CONFIG_I2C_SIZE_BYTES               = 22;
        static const uint8_t TIMING_CONFIG_I2C_SIZE_BYTES                = 23;
        static const uint8_t DYNAMIC_CONFIG_I2C_SIZE_BYTES               = 18;
        static const uint8_t SYSTEM_CONTROL_I2C_SIZE_BYTES               =  5;
        static const uint8_t SYSTEM_RESULTS_I2C_SIZE_BYTES               = 44;
        static const uint8_t CORE_RESULTS_I2C_SIZE_BYTES                 = 33;
        static const uint8_t DEBUG_RESULTS_I2C_SIZE_BYTES                = 56;
        static const uint8_t NVM_COPY_DATA_I2C_SIZE_BYTES                = 49;
        static const uint8_t PREV_SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES   = 44;
        static const uint8_t PREV_SHADOW_CORE_RESULTS_I2C_SIZE_BYTES     = 33;
        static const uint8_t PATCH_DEBUG_I2C_SIZE_BYTES                  =  2;
        static const uint8_t GPH_GENERAL_CONFIG_I2C_SIZE_BYTES           =  5;
        static const uint8_t GPH_STATIC_CONFIG_I2C_SIZE_BYTES            =  6;
        static const uint8_t GPH_TIMING_CONFIG_I2C_SIZE_BYTES            = 16;
        static const uint8_t FW_INTERNAL_I2C_SIZE_BYTES                  =  2;
        static const uint8_t PATCH_RESULTS_I2C_SIZE_BYTES                = 90;
        static const uint8_t SHADOW_SYSTEM_RESULTS_I2C_SIZE_BYTES        = 82;
        static const uint8_t SHADOW_CORE_RESULTS_I2C_SIZE_BYTES          = 33;

        static const uint8_t DEVICESSCARRAY_RTN = 0x00;
        static const uint16_t AMBIENT_WINDOW_VCSEL_PERIODS  = 256;
        static const uint16_t RANGING_WINDOW_VCSEL_PERIODS  = 2048;
        static const uint8_t DEVICEDSSMODE_REQUESTED_EFFFECTIVE_SPADS = 2;
        static const uint8_t DEVICEDSSMODE_TARGET_RATE = 1;
        static const uint8_t OFFSETCORRECTIONMODE_MM1_MM2_OFFSETS    =  1;
        static const uint8_t DEVICEGPIOMODE_OUTPUT_RANGE_AND_ERROR_INTERRUPTS       =  0x01;
        static const uint8_t DEVICESEQUENCECONFIG_RANGE            = 7;
        static const uint8_t DEVICEINTERRUPTPOLARITY_ACTIVE_LOW               =  0x10;
        static const uint8_t OFFSETCALIBRATIONMODE_MM1_MM2_STANDARD =  1;
        static const uint8_t DEVICESTATE_UNKNOWN               = 98;
        static const uint8_t WAIT_METHOD_BLOCKING               =  0;
        static const uint16_t TUNINGPARAM_PUBLIC_PAGE_BASE_ADDRESS  = 0x8000;
        static const uint16_t LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING = 2100;
        static const uint32_t FDA_MAX_TIMING_BUDGET_US = 550000;
        static const uint16_t LOWPOWER_AUTO_VHV_LOOP_DURATION_US = 245;
        static const uint16_t LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING = 1448;
        static const uint16_t MAX_I2C_XFER_SIZE = 256;

        static const uint16_t MACRO_PERIOD_VCSEL_PERIODS = (
                AMBIENT_WINDOW_VCSEL_PERIODS + RANGING_WINDOW_VCSEL_PERIODS);

        static const uint16_t TUNINGPARAM_LITE_PHASECAL_CONFIG_TIMEOUT_US = TUNINGPARAM_PUBLIC_PAGE_BASE_ADDRESS + 48;

        static const uint8_t DEVICEMEASUREMENTMODE_STOP_MASK   = 0x0F;
        static const uint8_t GROUPEDPARAMETERHOLD_ID_MASK      = 0x02;
        static const uint8_t INTERRUPT_CONFIG_NEW_SAMPLE_READY = 0x20;
        static const uint8_t CLEAR_RANGE_INT                   = 0x01;

        static const uint16_t TUNINGPARAM_VERSION_DEFAULT =  32771;
        static const uint16_t TUNINGPARAM_KEY_TABLE_VERSION_DEFAULT =  32769;
        static const uint16_t TUNINGPARAM_LLD_VERSION_DEFAULT =  32833;
        static const uint16_t TUNINGPARAM_LITE_CAL_REPEAT_RATE_DEFAULT =  0;
        static const uint16_t TUNINGPARAM_LITE_RANGING_GAIN_FACTOR_DEFAULT =  2011;
        static const uint16_t TUNINGPARAM_LITE_MIN_CLIP_MM_DEFAULT =  0;
        static const uint16_t TUNINGPARAM_LITE_LONG_SIGMA_THRESH_MM_DEFAULT =  60;
        static const uint16_t TUNINGPARAM_LITE_MED_SIGMA_THRESH_MM_DEFAULT =  60;
        static const uint16_t TUNINGPARAM_LITE_SHORT_SIGMA_THRESH_MM_DEFAULT =  360;
        static const uint16_t TUNINGPARAM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS_DEFAULT =  128;
        static const uint16_t TUNINGPARAM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS_DEFAULT =  128;
        static const uint16_t TUNINGPARAM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS_DEFAULT =  192;
        static const uint16_t TUNINGPARAM_LITE_XTALK_MARGIN_KCPS_DEFAULT = 0;
        static const uint16_t TUNINGPARAM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS_DEFAULT =  2560;
        static const uint16_t TUNINGPARAM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS_DEFAULT =  1280;
        static const uint16_t TUNINGPARAM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS_DEFAULT =  5120;
        static const uint16_t TUNINGPARAM_OFFSET_CAL_DSS_RATE_MCPS_DEFAULT =  2560;
        static const uint16_t TUNINGPARAM_SPADMAP_RATE_LIMIT_MCPS_DEFAULT =  12;
        static const uint16_t TUNINGPARAM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT =  2560;
        static const uint16_t TUNINGPARAM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT =  2560;

        static const uint8_t TUNINGPARAM_CONSISTENCY_LITE_PHASE_TOLERANCE_DEFAULT =  2;
        static const uint8_t TUNINGPARAM_PHASECAL_TARGET_DEFAULT =  33;
        static const uint8_t TUNINGPARAM_LITE_SIGMA_EST_PULSE_WIDTH_DEFAULT =  8;
        static const uint8_t TUNINGPARAM_LITE_SIGMA_EST_AMB_WIDTH_NS_DEFAULT =  16;
        static const uint8_t TUNINGPARAM_LITE_SIGMA_REF_MM_DEFAULT =  1;
        static const uint8_t TUNINGPARAM_LITE_RIT_MULT_DEFAULT =  64;
        static const uint8_t TUNINGPARAM_LITE_SEED_CONFIG_DEFAULT =  2;
        static const uint8_t TUNINGPARAM_LITE_QUANTIFIER_DEFAULT =  2;
        static const uint8_t TUNINGPARAM_LITE_FIRST_ORDER_SELECT_DEFAULT =  0;
        static const uint8_t TUNINGPARAM_INITIAL_PHASE_RTN_LITE_LONG_RANGE_DEFAULT =  14;
        static const uint8_t TUNINGPARAM_INITIAL_PHASE_RTN_LITE_MED_RANGE_DEFAULT =  10;
        static const uint8_t TUNINGPARAM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE_DEFAULT =  6;
        static const uint8_t TUNINGPARAM_INITIAL_PHASE_REF_LITE_LONG_RANGE_DEFAULT =  14;
        static const uint8_t TUNINGPARAM_INITIAL_PHASE_REF_LITE_MED_RANGE_DEFAULT =  10;
        static const uint8_t TUNINGPARAM_INITIAL_PHASE_REF_LITE_SHORT_RANGE_DEFAULT =  6;
        static const uint8_t TUNINGPARAM_TIMED_SEED_CONFIG_DEFAULT =  1;
        static const uint8_t TUNINGPARAM_VHV_LOOPBOUND_DEFAULT =  32;
        static const uint8_t TUNINGPARAM_REFSPADCHAR_DEVICE_TEST_MODE_DEFAULT =  8;
        static const uint8_t TUNINGPARAM_REFSPADCHAR_VCSEL_PERIOD_DEFAULT =  11;
        static const uint8_t TUNINGPARAM_OFFSET_CAL_PRE_SAMPLES_DEFAULT =  8;
        static const uint8_t TUNINGPARAM_OFFSET_CAL_MM1_SAMPLES_DEFAULT =  40;
        static const uint8_t TUNINGPARAM_OFFSET_CAL_MM2_SAMPLES_DEFAULT =  9;
        static const uint8_t TUNINGPARAM_SPADMAP_VCSEL_PERIOD_DEFAULT =  18;
        static const uint8_t TUNINGPARAM_SPADMAP_VCSEL_START_DEFAULT =  15;
        static const uint8_t TUNINGPARAM_LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT =  3;

        static const uint32_t TUNINGPARAM_REFSPADCHAR_PHASECAL_TIMEOUT_US_DEFAULT =  1000;
        static const uint32_t TUNINGPARAM_OFFSET_CAL_PHASECAL_TIMEOUT_US_DEFAULT =  1000;
        static const uint32_t TUNINGPARAM_OFFSET_CAL_MM_TIMEOUT_US_DEFAULT =  13000;
        static const uint32_t TUNINGPARAM_OFFSET_CAL_RANGE_TIMEOUT_US_DEFAULT =  13000;
        static const uint32_t TUNINGPARAM_TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT =  1000;
        static const uint32_t TUNINGPARAM_LITE_MM_CONFIG_TIMEOUT_US_DEFAULT =  2000;
        static const uint32_t TUNINGPARAM_TIMED_MM_CONFIG_TIMEOUT_US_DEFAULT =  2000;
        static const uint32_t TUNINGPARAM_LITE_RANGE_CONFIG_TIMEOUT_US_DEFAULT =  63000;
        static const uint32_t TUNINGPARAM_TIMED_RANGE_CONFIG_TIMEOUT_US_DEFAULT =  13000;
        static const uint32_t TUNINGPARAM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT =  1;
        static const uint32_t TUNINGPARAM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US_DEFAULT =  8000;

        const uint8_t DEFAULT_CONFIGURATION[91] = { 0x00, 0x01, 0x01,
            0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01, 0x00,
            0x00, 0x00, 0x00, 0xff, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x20, 0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21, 0x00, 0x00, 0x05, 0x00,
            0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38, 0xff, 0x01, 0x00, 0x08,
            0x00, 0x00, 0x01, 0xdb, 0x0f, 0x01, 0xf1, 0x0d, 0x01, 0x68, 0x00,
            0x80, 0x08, 0xb8, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x89, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00,
            0x00, 0x02, 0xc7, 0xff, 0x9B, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00
        };

        enum {
            DEVICESTATE_POWERDOWN              ,
            DEVICESTATE_HW_STANDBY             ,
            DEVICESTATE_FW_COLDBOOT            ,
            DEVICESTATE_SW_STANDBY             ,
            DEVICESTATE_RANGING_DSS_AUTO       ,
            DEVICESTATE_RANGING_DSS_MANUAL     ,
            DEVICESTATE_RANGING_WAIT_GPH_SYNC  ,
            DEVICESTATE_RANGING_GATHER_DATA    ,
            DEVICESTATE_RANGING_OUTPUT_DATA    ,
        };

        enum {

            CHECKENABLE_SIGMA_FINAL_RANGE,
            CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        };

        enum {
            SEQUENCESTEP_MM1  = 5,
            SEQUENCESTEP_MM2  = 6,
        };

        enum {
            DEVICEREADOUTMODE_SINGLE_SD     = 0x00 << 2,
            DEVICEREADOUTMODE_DUAL_SD       = 0x01 << 2,
            DEVICEREADOUTMODE_SPLIT_READOUT = 0x02 << 2,
            DEVICEREADOUTMODE_SPLIT_MANUAL  = 0x03 << 2,
        };

        enum {
            SEQUENCE_VHV_EN				= 0x01,
            SEQUENCE_PHASECAL_EN        = 0x02,
            SEQUENCE_REFERENCE_PHASE_EN = 0x04,
            SEQUENCE_DSS1_EN            = 0x08,
            SEQUENCE_DSS2_EN            = 0x10,
            SEQUENCE_MM1_EN             = 0x20,
            SEQUENCE_MM2_EN             = 0x40,
            SEQUENCE_RANGE_EN           = 0x80,
        };

        enum {
            PRESETMODE_AUTONOMOUS          =  3,
            PRESETMODE_LITE_RANGING        =  4,
            PRESETMODE_LOWPOWER_AUTONOMOUS =  8,
        };

        enum {
            DEVICESCHEDULERMODE_PSEUDO_SOLO,
            DEVICESCHEDULERMODE_STREAMING,
            DEVICESCHEDULERMODE_HISTOGRAM,
        };

        enum {
            DEVICEPRESETMODE_NONE                         =  0,
            DEVICEPRESETMODE_STANDARD_RANGING             =  1,
            DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE =  2,
            DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE  =  3,
            DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL     =  4,
            DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL     =  5,
            DEVICEPRESETMODE_TIMED_RANGING                =  6,
            DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE    =  7,
            DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE     =  8,
            DEVICEPRESETMODE_OLT                          = 17,
            DEVICEPRESETMODE_SINGLESHOT_RANGING           = 18,
            DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE	  = 36,
            DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE	  = 37,
            DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE	  = 38,
        };

        enum {
            DEVICEMEASUREMENTMODE_STOP       =  0x00,
            DEVICEMEASUREMENTMODE_SINGLESHOT =  0x10,
            DEVICEMEASUREMENTMODE_BACKTOBACK =  0x20,
            DEVICEMEASUREMENTMODE_TIMED      =  0x40,
            DEVICEMEASUREMENTMODE_ABORT      =  0x80,
        };

        enum {
            RGSTR_I2C_ADDRESS                                    = 0x0001,
            RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX                  = 0x0002,
            RGSTR_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND           = 0x0008,
            RGSTR_GLOBAL_CONFIG_SPAD_ENABLES_REF_0               = 0x000D,
            RGSTR_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS              = 0x0024,
            RGSTR_GPIO_HV_MUX_CTRL                               = 0x0030,
            RGSTR_GPIO_TIO_HV_STATUS                             = 0x0031,
            RGSTR_GPH_CONFIG_STREAM_COUNT_UPDATE_VALUE           = 0x0044,
            RGSTR_SYSTEM_GROUPED_PARAMETER_HOLD_0                = 0x0071,
            RGSTR_POWER_MANAGEMENT_GO1_POWER_FORCE               = 0x0083,
            RGSTR_FIRMWARE_ENABLE                                = 0x0085,
            RGSTR_SYSTEM_INTERRUPT_CLEAR                         = 0x0086,
            RGSTR_SYSTEM_MODE_START                              = 0x0087,
            RGSTR_RESULT_OSC_CALIBRATE_VAL                       = 0x00DE,
            RGSTR_MM_CONFIG_TIMEOUT_MACROP_A_HI                  = 0x005A,
            RGSTR_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0  = 0x0096,
            RGSTR_IDENTIFICATION_MODEL_ID                        = 0x010F,
        };

        // Typedefs ----------------------------------------------------------

        typedef uint8_t sequence_step_id_t;
        typedef uint32_t fixed_point_1616_t;
        typedef uint8_t device_state_t;
        typedef uint8_t device_preset_mode_t;
        typedef uint8_t device_measurement_mode_t;
        typedef uint8_t offset_calibration_mode_t;
        typedef uint8_t offset_correction_mode_t;
        typedef uint8_t device_sequence_config_t;
        typedef uint8_t device_ssc_array_t;
        typedef uint8_t preset_mode_t;

        typedef struct {
            uint8_t   i2c_slave_device_address;
            uint8_t   ana_config_vhv_ref_sel_vddpix;
            uint8_t   ana_config_vhv_ref_sel_vquench;
            uint8_t   ana_config_reg_avdd1v2_sel;
            uint8_t   ana_config_fast_osc_trim;
            uint16_t  osc_measured_fast_osc_frequency;
            uint8_t   vhv_config_timeout_macrop_loop_bound;
            uint8_t   vhv_config_count_thresh;
            uint8_t   vhv_config_offset;
            uint8_t   vhv_config_init;
        } static_nvm_managed_t;

        typedef struct {
            uint8_t   global_config_spad_enables_ref_0;
            uint8_t   global_config_spad_enables_ref_1;
            uint8_t   global_config_spad_enables_ref_2;
            uint8_t   global_config_spad_enables_ref_3;
            uint8_t   global_config_spad_enables_ref_4;
            uint8_t   global_config_spad_enables_ref_5;
            uint8_t   global_config_ref_en_start_select;
            uint8_t   ref_spad_man_num_requested_ref_spads;
            uint8_t   ref_spad_man_ref_location;
            uint16_t  algo_crosstalk_compensation_plane_offset_kcps;
            int16_t   algo_crosstalk_compensation_x_plane_gradient_kcps;
            int16_t   algo_crosstalk_compensation_y_plane_gradient_kcps;
            uint16_t  ref_spad_char_total_rate_target_mcps;
            int16_t   algo_part_to_part_range_offset_mm;
            int16_t   mm_config_inner_offset_mm;
            int16_t   mm_config_outer_offset_mm;
        } customer_nvm_managed_t;

        typedef struct {
            uint16_t  dss_config_target_total_rate_mcps;
            uint8_t   debug_ctrl;
            uint8_t   test_mode_ctrl;
            uint8_t   clk_gating_ctrl;
            uint8_t   nvm_bist_ctrl;
            uint8_t   nvm_bist_num_nvm_words;
            uint8_t   nvm_bist_start_address;
            uint8_t   host_if_status;
            uint8_t   pad_i2c_hv_config;
            uint8_t   pad_i2c_hv_extsup_config;
            uint8_t   gpio_hv_pad_ctrl;
            uint8_t   gpio_hv_mux_ctrl;
            uint8_t   gpio_tio_hv_status;
            uint8_t   gpio_fio_hv_status;
            uint8_t   ana_config_spad_sel_pswidth;
            uint8_t   ana_config_vcsel_pulse_width_offset;
            uint8_t   ana_config_fast_osc_config_ctrl;
            uint8_t   sigma_estimator_effective_pulse_width_ns;
            uint8_t   sigma_estimator_effective_ambient_width_ns;
            uint8_t   sigma_estimator_sigma_ref_mm;
            uint8_t   algo_crosstalk_compensation_valid_height_mm;
            uint8_t   spare_host_config_static_config_spare_0;
            uint8_t   spare_host_config_static_config_spare_1;
            uint16_t  algo_range_ignore_threshold_mcps;
            uint8_t   algo_range_ignore_valid_height_mm;
            uint8_t   algo_range_min_clip;
            uint8_t   algo_consistency_check_tolerance;
            uint8_t   spare_host_config_static_config_spare_2;
            uint8_t   sd_config_reset_stages_msb;
            uint8_t   sd_config_reset_stages_lsb;
        } static_config_t;

        typedef struct {
            uint8_t   gph_config_stream_count_update_value;
            uint8_t   global_config_stream_divider;
            uint8_t   system_interrupt_config_gpio;
            uint8_t   cal_config_vcsel_start;
            uint16_t  cal_config_repeat_rate;
            uint8_t   global_config_vcsel_width;
            uint8_t   phasecal_config_timeout_macrop;
            uint8_t   phasecal_config_target;
            uint8_t   phasecal_config_override;
            uint8_t   dss_config_roi_mode_control;
            uint16_t  system_thresh_rate_high;
            uint16_t  system_thresh_rate_low;
            uint16_t  dss_config_manual_effective_spads_select;
            uint8_t   dss_config_manual_block_select;
            uint8_t   dss_config_aperture_attenuation;
            uint8_t   dss_config_max_spads_limit;
            uint8_t   dss_config_min_spads_limit;
        } general_config_t;

        typedef struct {
            uint8_t   mm_config_timeout_macrop_a_hi;
            uint8_t   mm_config_timeout_macrop_a_lo;
            uint8_t   mm_config_timeout_macrop_b_hi;
            uint8_t   mm_config_timeout_macrop_b_lo;
            uint8_t   range_config_timeout_macrop_a_hi;
            uint8_t   range_config_timeout_macrop_a_lo;
            uint8_t   range_config_vcsel_period_a;
            uint8_t   range_config_timeout_macrop_b_hi;
            uint8_t   range_config_timeout_macrop_b_lo;
            uint8_t   range_config_vcsel_period_b;
            uint16_t  range_config_sigma_thresh;
            uint16_t  range_config_min_count_rate_rtn_limit_mcps;
            uint8_t   range_config_valid_phase_low;
            uint8_t   range_config_valid_phase_high;
            uint32_t  system_intermeasurement_period;
            uint8_t   system_fractional_enable;
        } timing_config_t;

        typedef struct {
            uint8_t   system_grouped_parameter_hold_0;
            uint16_t  system_thresh_high;
            uint16_t  system_thresh_low;
            uint8_t   system_enable_xtalk_per_quadrant;
            uint8_t   system_seed_config;
            uint8_t   sd_config_woi_sd0;
            uint8_t   sd_config_woi_sd1;
            uint8_t   sd_config_initial_phase_sd0;
            uint8_t   sd_config_initial_phase_sd1;
            uint8_t   system_grouped_parameter_hold_1;
            uint8_t   sd_config_first_order_select;
            uint8_t   sd_config_quantifier;
            uint8_t   roi_config_user_roi_centre_spad;
            uint8_t   roi_config_user_roi_requested_global_xy_size;
            uint8_t   system_sequence_config;
            uint8_t   system_grouped_parameter_hold;
        } dynamic_config_t;

        typedef struct {
            uint8_t   power_management_go1_power_force;
            uint8_t   system_stream_count_ctrl;
            uint8_t   firmware_enable;
            uint8_t   system_interrupt_clear;
            uint8_t   system_mode_start;
        } system_control_t;

        typedef struct {
            uint16_t  phasecal_result_reference_phase;
            uint8_t   phasecal_result_vcsel_start;
            uint8_t   ref_spad_char_result_num_actual_ref_spads;
            uint8_t   ref_spad_char_result_ref_location;
            uint8_t   vhv_result_coldboot_status;
            uint8_t   vhv_result_search_result;
            uint8_t   vhv_result_latest_setting;
            uint16_t  result_osc_calibrate_val;
            uint8_t   ana_config_powerdown_go1;
            uint8_t   ana_config_ref_bg_ctrl;
            uint8_t   ana_config_regdvdd1v2_ctrl;
            uint8_t   ana_config_osc_slow_ctrl;
            uint8_t   test_mode_status;
            uint8_t   firmware_system_status;
            uint8_t   firmware_mode_status;
            uint8_t   firmware_secondary_mode_status;
            uint16_t  firmware_cal_repeat_rate_counter;
            uint16_t  gph_system_thresh_high;
            uint16_t  gph_system_thresh_low;
            uint8_t   gph_system_enable_xtalk_per_quadrant;
            uint8_t   gph_spare_0;
            uint8_t   gph_sd_config_woi_sd0;
            uint8_t   gph_sd_config_woi_sd1;
            uint8_t   gph_sd_config_initial_phase_sd0;
            uint8_t   gph_sd_config_initial_phase_sd1;
            uint8_t   gph_sd_config_first_order_select;
            uint8_t   gph_sd_config_quantifier;
            uint8_t   gph_roi_config_user_roi_centre_spad;
            uint8_t   gph_roi_config_user_roi_requested_global_xy_size;
            uint8_t   gph_system_sequence_config;
            uint8_t   gph_gph_id;
            uint8_t   system_interrupt_set;
            uint8_t   interrupt_manager_enables;
            uint8_t   interrupt_manager_clear;
            uint8_t   interrupt_manager_status;
            uint8_t   mcu_to_host_bank_wr_access_en;
            uint8_t   power_management_go1_reset_status;
            uint8_t   pad_startup_mode_value_ro;
            uint8_t   pad_startup_mode_value_ctrl;
            uint32_t  pll_period_us;
            uint32_t  interrupt_scheduler_data_out;
            uint8_t   nvm_bist_complete;
            uint8_t   nvm_bist_status;
        } debug_results_t;

        typedef struct {
            uint8_t   identification_model_id;
            uint8_t   identification_module_type;
            uint8_t   identification_revision_id;
            uint16_t  identification_module_id;
            uint8_t   ana_config_fast_osc_trim_max;
            uint8_t   ana_config_fast_osc_freq_set;
            uint8_t   ana_config_vcsel_trim;
            uint8_t   ana_config_vcsel_selion;
            uint8_t   ana_config_vcsel_selion_max;
            uint8_t   protected_laser_safety_lock_bit;
            uint8_t   laser_safety_key;
            uint8_t   laser_safety_key_ro;
            uint8_t   laser_safety_clip;
            uint8_t   laser_safety_mult;
            uint8_t   global_config_spad_enables_rtn_0;
            uint8_t   global_config_spad_enables_rtn_1;
            uint8_t   global_config_spad_enables_rtn_2;
            uint8_t   global_config_spad_enables_rtn_3;
            uint8_t   global_config_spad_enables_rtn_4;
            uint8_t   global_config_spad_enables_rtn_5;
            uint8_t   global_config_spad_enables_rtn_6;
            uint8_t   global_config_spad_enables_rtn_7;
            uint8_t   global_config_spad_enables_rtn_8;
            uint8_t   global_config_spad_enables_rtn_9;
            uint8_t   global_config_spad_enables_rtn_10;
            uint8_t   global_config_spad_enables_rtn_11;
            uint8_t   global_config_spad_enables_rtn_12;
            uint8_t   global_config_spad_enables_rtn_13;
            uint8_t   global_config_spad_enables_rtn_14;
            uint8_t   global_config_spad_enables_rtn_15;
            uint8_t   global_config_spad_enables_rtn_16;
            uint8_t   global_config_spad_enables_rtn_17;
            uint8_t   global_config_spad_enables_rtn_18;
            uint8_t   global_config_spad_enables_rtn_19;
            uint8_t   global_config_spad_enables_rtn_20;
            uint8_t   global_config_spad_enables_rtn_21;
            uint8_t   global_config_spad_enables_rtn_22;
            uint8_t   global_config_spad_enables_rtn_23;
            uint8_t   global_config_spad_enables_rtn_24;
            uint8_t   global_config_spad_enables_rtn_25;
            uint8_t   global_config_spad_enables_rtn_26;
            uint8_t   global_config_spad_enables_rtn_27;
            uint8_t   global_config_spad_enables_rtn_28;
            uint8_t   global_config_spad_enables_rtn_29;
            uint8_t   global_config_spad_enables_rtn_30;
            uint8_t   global_config_spad_enables_rtn_31;
            uint8_t   roi_config_mode_roi_centre_spad;
            uint8_t   roi_config_mode_roi_xy_size;
        } nvm_copy_data_t;

        typedef struct {
            uint8_t    device_test_mode;     
            uint8_t    vcsel_period;         
            uint32_t   timeout_us;           
            uint16_t   target_count_rate_mcps;
            uint16_t   min_count_rate_limit_mcps;
            uint16_t   max_count_rate_limit_mcps;
        } refspadchar_config_t;

        typedef struct {
            device_ssc_array_t  array_select;
            uint8_t    vcsel_period;
            uint8_t    vcsel_start;
            uint8_t    vcsel_width;
            uint32_t   timeout_us;
            uint16_t   rate_limit_mcps;
        } ssc_config_t;

        typedef struct {
            uint32_t  algo_crosstalk_compensation_plane_offset_kcps;
            int16_t   algo_crosstalk_compensation_x_plane_gradient_kcps;
            int16_t   algo_crosstalk_compensation_y_plane_gradient_kcps;
            uint32_t  nvm_default_crosstalk_compensation_plane_offset_kcps;
            int16_t   nvm_default_crosstalk_compensation_x_plane_gradient_kcps;
            int16_t   nvm_default_crosstalk_compensation_y_plane_gradient_kcps;
            uint8_t   global_crosstalk_compensation_enable;
            int16_t   lite_mode_crosstalk_margin_kcps;
            uint8_t   crosstalk_range_ignore_threshold_mult;
            uint16_t  crosstalk_range_ignore_threshold_rate_mcps;
        } xtalk_config_t;

        typedef struct {
            uint16_t  tp_tuning_parm_version;
            uint16_t  tp_tuning_parm_key_table_version;
            uint16_t  tp_tuning_parm_lld_version;
            uint8_t   tp_init_phase_rtn_lite_long;
            uint8_t   tp_init_phase_rtn_lite_med;
            uint8_t   tp_init_phase_rtn_lite_short;
            uint8_t   tp_init_phase_ref_lite_long;
            uint8_t   tp_init_phase_ref_lite_med;
            uint8_t   tp_init_phase_ref_lite_short;
            uint8_t   tp_consistency_lite_phase_tolerance;
            uint8_t   tp_phasecal_target;
            uint16_t  tp_cal_repeat_rate;
            uint8_t   tp_lite_min_clip;
            uint16_t  tp_lite_long_sigma_thresh_mm;
            uint16_t  tp_lite_med_sigma_thresh_mm;
            uint16_t  tp_lite_short_sigma_thresh_mm;
            uint16_t  tp_lite_long_min_count_rate_rtn_mcps;
            uint16_t  tp_lite_med_min_count_rate_rtn_mcps;
            uint16_t  tp_lite_short_min_count_rate_rtn_mcps;
            uint8_t   tp_lite_sigma_est_pulse_width_ns;
            uint8_t   tp_lite_sigma_est_amb_width_ns;
            uint8_t   tp_lite_sigma_ref_mm;
            uint8_t   tp_lite_seed_cfg;
            uint8_t   tp_timed_seed_cfg;
            uint8_t   tp_lite_quantifier;
            uint8_t   tp_lite_first_order_select;
            uint16_t  tp_dss_target_lite_mcps;
            uint16_t  tp_dss_target_timed_mcps;
            uint32_t  tp_phasecal_timeout_lite_us;
            uint32_t  tp_phasecal_timeout_timed_us;
            uint32_t  tp_mm_timeout_lite_us;
            uint32_t  tp_mm_timeout_timed_us;
            uint32_t  tp_mm_timeout_lpa_us;
            uint32_t  tp_range_timeout_lite_us;
            uint32_t  tp_range_timeout_timed_us;
            uint32_t  tp_range_timeout_lpa_us;
        } tuning_parm_storage_t;

        typedef struct {
            uint8_t   x_centre;  
            uint8_t   y_centre;  
        } optical_centre_t;

        typedef struct {
            uint8_t   x_centre;   
            uint8_t   y_centre;   
            uint8_t   width;      
            uint8_t   height;     
        } user_zone_t;

        typedef struct {
            uint8_t		vhv_loop_bound;
            uint8_t		is_low_power_auto_mode;
            uint8_t		low_power_auto_range_count;
            uint8_t		saved_interrupt_config;
            uint8_t		saved_vhv_init;
            uint8_t		saved_vhv_timeout;
            uint8_t		first_run_phasecal_result;
            uint32_t	dss_total_rate_per_spad_mcps;
            uint16_t	dss_required_spads;
        } low_power_auto_data_t;

        typedef struct {
            device_state_t     cfg_device_state;
            device_state_t     rd_device_state;
            uint8_t                stream_count;
            uint8_t                device_status;
        } range_results_t;

        typedef struct {
            uint8_t    preset_mode;
            uint8_t    dss_config_roi_mode_control;
            uint16_t   dss_config_manual_effective_spads_select;
            uint8_t    no_of_samples;
            uint32_t   effective_spads;
            uint32_t   peak_rate_mcps;
            uint32_t   sigma_mm;
            int32_t    median_range_mm;
            int32_t    range_mm_offset;
        } offset_range_data_t;

        static const uint8_t MAX_OFFSET_RANGE_RESULTS = 3;
        typedef struct {
            int16_t      cal_distance_mm;
            error_t cal_status;
            uint8_t      cal_report;
            uint8_t      max_results;
            uint8_t      active_results;
            offset_range_data_t data[MAX_OFFSET_RANGE_RESULTS];
        } offset_range_results_t;

        typedef struct {
            uint16_t  result_mm_inner_actual_effective_spads;
            uint16_t  result_mm_outer_actual_effective_spads;
            uint16_t  result_mm_inner_peak_signal_count_rtn_mcps;
            uint16_t  result_mm_outer_peak_signal_count_rtn_mcps;
        } additional_offset_cal_data_t;

        static const uint8_t NVM_PEAK_RATE_MAP_SAMPLES = 25;
        typedef struct {
            int16_t     cal_distance_mm;
            uint16_t    max_samples;
            uint16_t    width;
            uint16_t    height;
            uint16_t    peak_rate_mcps[NVM_PEAK_RATE_MAP_SAMPLES];
        } cal_peak_rate_map_t;

        typedef struct {

            uint16_t   standard_ranging_gain_factor;

        } gain_calibration_data_t;

        typedef struct {
            device_state_t   cfg_device_state;
            uint8_t   cfg_stream_count;
            uint8_t   cfg_gph_id;
            uint8_t   cfg_timing_status;
            device_state_t   rd_device_state;
            uint8_t   rd_stream_count;
            uint8_t   rd_gph_id;
            uint8_t   rd_timing_status;
        } ll_driver_state_t;

        typedef struct {
            uint16_t  dss_config_target_total_rate_mcps;
            uint32_t  phasecal_config_timeout_us;
            uint32_t  range_config_timeout_us;
            uint32_t  mm_config_timeout_us;
            uint8_t   pre_num_of_samples;
            uint8_t   mm1_num_of_samples;
            uint8_t   mm2_num_of_samples;
        } offsetcal_config_t;

        static const uint8_t CHECKENABLE_NUMBER_OF_CHECKS = 2;
        typedef struct {
            preset_mode_t presetMode;
            distanceMode_t distance_mode;
            distanceMode_t internal_distance_mode;
            distanceMode_t new_distance_mode;
            uint32_t measurement_timing_budget_usec;
            uint8_t limit_checks_enable[CHECKENABLE_NUMBER_OF_CHECKS];
            uint8_t limit_checks_status[CHECKENABLE_NUMBER_OF_CHECKS];
            fixed_point_1616_t limit_checks_value[CHECKENABLE_NUMBER_OF_CHECKS];
            fixed_point_1616_t limit_checks_current[CHECKENABLE_NUMBER_OF_CHECKS];
        } device_parameters_t;

        // Static methods -----------------------------------------------------

        static void i2c_encode_uint16_t(
                const uint16_t ip_value,
                const uint16_t count,
                uint8_t *pbuffer)
        {
            auto data =  ip_value;

            for (uint16_t i = 0; i < count ; i++) {
                pbuffer[count-i-1] = (uint8_t)(data & 0x00FF);
                data = data >> 8;
            }
        }

        static uint16_t i2c_decode_uint16_t(uint16_t count, uint8_t *pbuffer)
        {
            uint16_t value = 0x00;

            while (count-- > 0) {
                value = (value << 8) | (uint16_t)*pbuffer++;
            }

            return value;
        }

        static void i2c_encode_int16_t(
                const int16_t ip_value,
                const uint16_t count,
                uint8_t *pbuffer)
        {
            auto data =  ip_value;

            for (uint16_t i = 0; i < count ; i++) {
                pbuffer[count-i-1] = (uint8_t)(data & 0x00FF);
                data = data >> 8;
            }
        }

        static int16_t i2c_decode_int16_t(uint16_t count, uint8_t *pbuffer)
        {
            int16_t value = 0;

            if (*pbuffer >= 0x80) {
                value = 0xFFFF;
            }

            while (count-- > 0) {
                value = (value << 8) | (int16_t)*pbuffer++;
            }

            return value;
        }

        static void i2c_encode_uint32_t(
                const uint32_t ip_value,
                const uint16_t count,
                uint8_t *pbuffer)
        {

            auto data = ip_value;

            for (uint16_t i = 0; i < count ; i++) {
                pbuffer[count-i-1] = (uint8_t)(data & 0x00FF);
                data = data >> 8;
            }
        }

        static void decode_row_col(
                const uint8_t  spad_number,
                uint8_t *prow,
                uint8_t *pcol)
        {

            if (spad_number > 127) {
                *prow = 8 + ((255-spad_number) & 0x07);
                *pcol = (spad_number-128) >> 3;
            } else {
                *prow = spad_number & 0x07;
                *pcol = (127-spad_number) >> 3;
            }
        }

        static void copy_rtn_good_spads_to_buffer(
                const nvm_copy_data_t  *pdata,
                uint8_t  *pbuffer)
        {
            pbuffer[ 0] = pdata->global_config_spad_enables_rtn_0;
            pbuffer[ 1] = pdata->global_config_spad_enables_rtn_1;
            pbuffer[ 2] = pdata->global_config_spad_enables_rtn_2;
            pbuffer[ 3] = pdata->global_config_spad_enables_rtn_3;
            pbuffer[ 4] = pdata->global_config_spad_enables_rtn_4;
            pbuffer[ 5] = pdata->global_config_spad_enables_rtn_5;
            pbuffer[ 6] = pdata->global_config_spad_enables_rtn_6;
            pbuffer[ 7] = pdata->global_config_spad_enables_rtn_7;
            pbuffer[ 8] = pdata->global_config_spad_enables_rtn_8;
            pbuffer[ 9] = pdata->global_config_spad_enables_rtn_9;
            pbuffer[10] = pdata->global_config_spad_enables_rtn_10;
            pbuffer[11] = pdata->global_config_spad_enables_rtn_11;
            pbuffer[12] = pdata->global_config_spad_enables_rtn_12;
            pbuffer[13] = pdata->global_config_spad_enables_rtn_13;
            pbuffer[14] = pdata->global_config_spad_enables_rtn_14;
            pbuffer[15] = pdata->global_config_spad_enables_rtn_15;
            pbuffer[16] = pdata->global_config_spad_enables_rtn_16;
            pbuffer[17] = pdata->global_config_spad_enables_rtn_17;
            pbuffer[18] = pdata->global_config_spad_enables_rtn_18;
            pbuffer[19] = pdata->global_config_spad_enables_rtn_19;
            pbuffer[20] = pdata->global_config_spad_enables_rtn_20;
            pbuffer[21] = pdata->global_config_spad_enables_rtn_21;
            pbuffer[22] = pdata->global_config_spad_enables_rtn_22;
            pbuffer[23] = pdata->global_config_spad_enables_rtn_23;
            pbuffer[24] = pdata->global_config_spad_enables_rtn_24;
            pbuffer[25] = pdata->global_config_spad_enables_rtn_25;
            pbuffer[26] = pdata->global_config_spad_enables_rtn_26;
            pbuffer[27] = pdata->global_config_spad_enables_rtn_27;
            pbuffer[28] = pdata->global_config_spad_enables_rtn_28;
            pbuffer[29] = pdata->global_config_spad_enables_rtn_29;
            pbuffer[30] = pdata->global_config_spad_enables_rtn_30;
            pbuffer[31] = pdata->global_config_spad_enables_rtn_31;
        }

        static uint32_t decode_timeout(uint16_t encoded_timeout)
        {
            uint32_t timeout_macro_clks = ((uint32_t) (encoded_timeout & 0x00FF)
                    << (uint32_t) ((encoded_timeout & 0xFF00) >> 8)) + 1;

            return timeout_macro_clks;
        }

        static uint32_t calc_decoded_timeout_us(
                const uint16_t timeout_encoded,
                const uint32_t macro_period_us)
        {
            auto timeout_mclks = decode_timeout(timeout_encoded);

            return calc_timeout_us(timeout_mclks, macro_period_us);
        }

        static uint16_t encode_timeout(const uint32_t timeout_mclks)
        {
            uint16_t encoded_timeout = 0;

            if (timeout_mclks > 0) {

                uint32_t ls_byte = timeout_mclks - 1;
                uint16_t ms_byte = 0;

                while ((ls_byte & 0xFFFFFF00) > 0) {
                    ls_byte = ls_byte >> 1;
                    ms_byte++;
                }

                encoded_timeout = (ms_byte << 8)
                    + (uint16_t) (ls_byte & 0x000000FF);
            }

            return encoded_timeout;
        }

        static uint32_t calc_timeout_mclks(const uint32_t timeout_us,
                const uint32_t macro_period_us)
        {
            return ((timeout_us << 12) + 
                    (macro_period_us>>1)) / macro_period_us;
        }

        static uint16_t calc_encoded_timeout(const uint32_t timeout_us,
                const uint32_t macro_period_us)
        {
            auto timeout_mclks = 
                calc_timeout_mclks(timeout_us, macro_period_us);

            return encode_timeout(timeout_mclks);
        }

        static uint32_t calc_timeout_us(
                uint32_t timeout_mclks,
                uint32_t macro_period_us)
        {

            uint32_t timeout_us     = 0;
            uint64_t tmp            = 0;

            tmp  = (uint64_t)timeout_mclks * (uint64_t)macro_period_us;
            tmp += 0x00800;
            tmp  = tmp >> 12;

            timeout_us = (uint32_t)tmp;

            return timeout_us;
        }

        static uint16_t calc_range_ignore_threshold(
                const uint32_t central_rate,
                const int16_t  x_gradient,
                const int16_t  y_gradient,
                const uint8_t  rate_mult)
        {
            int32_t central_rate_int = ((int32_t)central_rate * (1 << 4)) / (1000);

            int16_t x_gradient_int = x_gradient < 0 ?  -x_gradient : 0;

            int16_t y_gradient_int = y_gradient < 0 ?  -y_gradient : 0;

            int32_t range_ignore_thresh_int = 
                (8 * x_gradient_int * 4) + (8 * y_gradient_int * 4);

            range_ignore_thresh_int = range_ignore_thresh_int / 1000;

            range_ignore_thresh_int = range_ignore_thresh_int + central_rate_int;

            range_ignore_thresh_int = (int32_t)rate_mult * range_ignore_thresh_int;

            range_ignore_thresh_int = (range_ignore_thresh_int + (1<<4)) / (1<<5);

            return 
                range_ignore_thresh_int > 0xFFFF ?
                0xFFFF :
                (uint16_t)range_ignore_thresh_int;
        }

        static uint8_t encode_row_col(const uint8_t  row, const uint8_t  col)
        {
            return row > 7 ?  
                128 + (col << 3) + (15-row) : 
                ((15-col) << 3) + row;
        }

        static uint8_t encode_zone_size(const uint8_t  width,
                const uint8_t height)
        {
            return (height << 4) + width;

        }

        static void decode_zone_size(const uint8_t  encoded_xy_size, 
                uint8_t  *pwidth, uint8_t  *pheight)
        {
            *pheight = encoded_xy_size >> 4;
            *pwidth  = encoded_xy_size & 0x0F;
        }

        // Instance variables --------------------------------------------------

        uint8_t                      _i2c_address;
        uint8_t                      _wait_method;
        device_preset_mode_t         _preset_mode;
        device_measurement_mode_t    _measurement_mode;
        offset_calibration_mode_t    _offset_calibration_mode;
        offset_correction_mode_t     _offset_correction_mode;
        uint32_t                     _phasecal_config_timeout_us;
        uint32_t                     _mm_config_timeout_us;
        uint32_t                     _range_config_timeout_us;
        uint32_t                     _inter_measurement_period_ms;
        uint16_t                     _dss_config_target_total_rate_mcps;
        uint32_t                     _fw_ready_poll_duration_ms;
        uint8_t                      _fw_ready;
        uint8_t                      _debug_mode;
        ll_driver_state_t            _ll_state;
        customer_nvm_managed_t       _customer;
        cal_peak_rate_map_t          _cal_peak_rate_map;
        additional_offset_cal_data_t _add_off_cal_data;
        gain_calibration_data_t      _gain_cal;
        user_zone_t                  _mm_roi;
        optical_centre_t             _optical_centre;
        tuning_parm_storage_t        _tuning_params;
        uint8_t                      _rtn_good_spads[32];
        refspadchar_config_t         _refspadchar;
        ssc_config_t                 _ssc_cfg;
        xtalk_config_t               _xtalk_cfg;
        offsetcal_config_t           _offsetcal_cfg;
        static_nvm_managed_t         _stat_nvm;
        static_config_t              _stat_cfg;
        general_config_t             _gen_cfg;
        timing_config_t              _tim_cfg;
        dynamic_config_t             _dyn_cfg;
        system_control_t             _sys_ctrl;
        nvm_copy_data_t              _nvm_copy_data;
        offset_range_results_t       _offset_results;
        debug_results_t              _dbg_results;
        low_power_auto_data_t		 _low_power_auto_data;
        device_parameters_t          _current_parameters;

        // Platform-specific

        void * _device;

        // Instance methods ---------------------------------------------------

        error_t read_word(const uint16_t rgstr, uint16_t *data)
        {
            uint8_t buffer[2] = {};

            auto status = read_bytes(_device, _i2c_address, rgstr, 2, buffer);

            if (!status) {
                *data = (buffer[0] << 8) + buffer[1];
            }

            return status;
        }

        error_t write_word(const uint16_t rgstr, const uint16_t data)
        {
            uint8_t buffer[2] = {};
            buffer[0] = data >> 8;
            buffer[1] = data & 0x00FF;
            return write_bytes(_device, _i2c_address, rgstr, 2, 
                    (uint8_t *)buffer);
        }

        error_t read_byte(const uint16_t rgstr, uint8_t *data)
        {
            return read_bytes(_device, _i2c_address, rgstr, 1, data);
        }

        error_t write_byte(const uint16_t rgstr, const uint8_t data)
        {
            return write_bytes(_device, _i2c_address, rgstr, 1, &data);
        }

        error_t i2c_decode_static_nvm_managed(const uint16_t buf_size,
                uint8_t  *pbuffer, static_nvm_managed_t  *pdata)
        {
            if (STATIC_NVM_MANAGED_I2C_SIZE_BYTES > buf_size) {
                return ERROR_COMMS_BUFFER_TOO_SMALL;
            }

            pdata->i2c_slave_device_address =
                (*(pbuffer +   0)) & 0x7F;
            pdata->ana_config_vhv_ref_sel_vddpix =
                (*(pbuffer +   1)) & 0xF;
            pdata->ana_config_vhv_ref_sel_vquench =
                (*(pbuffer +   2)) & 0x7F;
            pdata->ana_config_reg_avdd1v2_sel =
                (*(pbuffer +   3)) & 0x3;
            pdata->ana_config_fast_osc_trim =
                (*(pbuffer +   4)) & 0x7F;
            pdata->osc_measured_fast_osc_frequency =
                (i2c_decode_uint16_t(2, pbuffer +   5));
            pdata->vhv_config_timeout_macrop_loop_bound =
                (*(pbuffer +   7));
            pdata->vhv_config_count_thresh =
                (*(pbuffer +   8));
            pdata->vhv_config_offset =
                (*(pbuffer +   9)) & 0x3F;
            pdata->vhv_config_init =
                (*(pbuffer +  10));

            return ERROR_NONE;
        }

        error_t get_static_nvm_managed(static_nvm_managed_t  *pdata)
        {
            uint8_t comms_buffer[STATIC_NVM_MANAGED_I2C_SIZE_BYTES] = {};

            auto status = read_bytes(_device, _i2c_address, RGSTR_I2C_ADDRESS, 
                    STATIC_NVM_MANAGED_I2C_SIZE_BYTES, comms_buffer);

            if (status == ERROR_NONE) {
                status = i2c_decode_static_nvm_managed(
                        STATIC_NVM_MANAGED_I2C_SIZE_BYTES,
                        comms_buffer,
                        pdata);
            }

            return status;
        }

        error_t i2c_encode_customer_nvm_managed(const uint16_t buf_size, 
                uint8_t *pbuffer)
        {
            if (CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES > buf_size) {
                return ERROR_COMMS_BUFFER_TOO_SMALL;
            }

            *(pbuffer +   0) =
                _customer.global_config_spad_enables_ref_0;
            *(pbuffer +   1) =
                _customer.global_config_spad_enables_ref_1;
            *(pbuffer +   2) =
                _customer.global_config_spad_enables_ref_2;
            *(pbuffer +   3) =
                _customer.global_config_spad_enables_ref_3;
            *(pbuffer +   4) =
                _customer.global_config_spad_enables_ref_4;
            *(pbuffer +   5) =
                _customer.global_config_spad_enables_ref_5 & 0xF;
            *(pbuffer +   6) =
                _customer.global_config_ref_en_start_select;
            *(pbuffer +   7) =
                _customer.ref_spad_man_num_requested_ref_spads & 0x3F;
            *(pbuffer +   8) =
                _customer.ref_spad_man_ref_location & 0x3;
            i2c_encode_uint16_t(
                    _customer.algo_crosstalk_compensation_plane_offset_kcps,
                    2,
                    pbuffer +   9);
            i2c_encode_int16_t(
                    _customer.algo_crosstalk_compensation_x_plane_gradient_kcps,
                    2,
                    pbuffer +  11);
            i2c_encode_int16_t(
                    _customer.algo_crosstalk_compensation_y_plane_gradient_kcps,
                    2,
                    pbuffer +  13);
            i2c_encode_uint16_t(
                    _customer.ref_spad_char_total_rate_target_mcps,
                    2,
                    pbuffer +  15);
            i2c_encode_int16_t(
                    _customer.algo_part_to_part_range_offset_mm & 0x1FFF,
                    2,
                    pbuffer +  17);
            i2c_encode_int16_t(
                    _customer.mm_config_inner_offset_mm,
                    2,
                    pbuffer +  19);
            i2c_encode_int16_t(
                    _customer.mm_config_outer_offset_mm,
                    2,
                    pbuffer +  21);

            return ERROR_NONE;
        }

        error_t i2c_decode_customer_nvm_managed(const uint16_t buf_size,
                uint8_t  *pbuffer, customer_nvm_managed_t *pdata)
        {

            if (CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES > buf_size) {
                return ERROR_COMMS_BUFFER_TOO_SMALL;
            }

            pdata->global_config_spad_enables_ref_0 =
                (*(pbuffer +   0));
            pdata->global_config_spad_enables_ref_1 =
                (*(pbuffer +   1));
            pdata->global_config_spad_enables_ref_2 =
                (*(pbuffer +   2));
            pdata->global_config_spad_enables_ref_3 =
                (*(pbuffer +   3));
            pdata->global_config_spad_enables_ref_4 =
                (*(pbuffer +   4));
            pdata->global_config_spad_enables_ref_5 =
                (*(pbuffer +   5)) & 0xF;
            pdata->global_config_ref_en_start_select =
                (*(pbuffer +   6));
            pdata->ref_spad_man_num_requested_ref_spads =
                (*(pbuffer +   7)) & 0x3F;
            pdata->ref_spad_man_ref_location =
                (*(pbuffer +   8)) & 0x3;
            pdata->algo_crosstalk_compensation_plane_offset_kcps =
                (i2c_decode_uint16_t(2, pbuffer +   9));
            pdata->algo_crosstalk_compensation_x_plane_gradient_kcps =
                (i2c_decode_int16_t(2, pbuffer +  11));
            pdata->algo_crosstalk_compensation_y_plane_gradient_kcps =
                (i2c_decode_int16_t(2, pbuffer +  13));
            pdata->ref_spad_char_total_rate_target_mcps =
                (i2c_decode_uint16_t(2, pbuffer +  15));
            pdata->algo_part_to_part_range_offset_mm =
                (i2c_decode_int16_t(2, pbuffer +  17)) & 0x1FFF;
            pdata->mm_config_inner_offset_mm =
                (i2c_decode_int16_t(2, pbuffer +  19));
            pdata->mm_config_outer_offset_mm =
                (i2c_decode_int16_t(2, pbuffer +  21));

            return ERROR_NONE;
        }

        error_t get_customer_nvm_managed(
                customer_nvm_managed_t  *pdata)
        {
            error_t status = ERROR_NONE;

            uint8_t comms_buffer[CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES] = {};

            if (status == ERROR_NONE) 
                status = read_bytes(_device, _i2c_address,
                        RGSTR_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                        CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES, comms_buffer);

            if (status == ERROR_NONE)
                status = i2c_decode_customer_nvm_managed(
                        CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES,
                        comms_buffer,
                        pdata);

            return status;
        }

        error_t i2c_encode_static_config(const uint16_t buf_size, 
                uint8_t  *pbuffer)
        {

            if (STATIC_CONFIG_I2C_SIZE_BYTES > buf_size) {
                return ERROR_COMMS_BUFFER_TOO_SMALL;
            }

            i2c_encode_uint16_t(
                    _stat_cfg.dss_config_target_total_rate_mcps,
                    2,
                    pbuffer +   0);
            *(pbuffer +   2) =
                _stat_cfg.debug_ctrl & 0x1;
            *(pbuffer +   3) =
                _stat_cfg.test_mode_ctrl & 0xF;
            *(pbuffer +   4) =
                _stat_cfg.clk_gating_ctrl & 0xF;
            *(pbuffer +   5) =
                _stat_cfg.nvm_bist_ctrl & 0x1F;
            *(pbuffer +   6) =
                _stat_cfg.nvm_bist_num_nvm_words & 0x7F;
            *(pbuffer +   7) =
                _stat_cfg.nvm_bist_start_address & 0x7F;
            *(pbuffer +   8) =
                _stat_cfg.host_if_status & 0x1;
            *(pbuffer +   9) =
                _stat_cfg.pad_i2c_hv_config;
            *(pbuffer +  10) =
                _stat_cfg.pad_i2c_hv_extsup_config & 0x1;
            *(pbuffer +  11) =
                _stat_cfg.gpio_hv_pad_ctrl & 0x3;
            *(pbuffer +  12) =
                _stat_cfg.gpio_hv_mux_ctrl & 0x1F;
            *(pbuffer +  13) =
                _stat_cfg.gpio_tio_hv_status & 0x3;
            *(pbuffer +  14) =
                _stat_cfg.gpio_fio_hv_status & 0x3;
            *(pbuffer +  15) =
                _stat_cfg.ana_config_spad_sel_pswidth & 0x7;
            *(pbuffer +  16) =
                _stat_cfg.ana_config_vcsel_pulse_width_offset & 0x1F;
            *(pbuffer +  17) =
                _stat_cfg.ana_config_fast_osc_config_ctrl & 0x1;
            *(pbuffer +  18) =
                _stat_cfg.sigma_estimator_effective_pulse_width_ns;
            *(pbuffer +  19) =
                _stat_cfg.sigma_estimator_effective_ambient_width_ns;
            *(pbuffer +  20) =
                _stat_cfg.sigma_estimator_sigma_ref_mm;
            *(pbuffer +  21) =
                _stat_cfg.algo_crosstalk_compensation_valid_height_mm;
            *(pbuffer +  22) =
                _stat_cfg.spare_host_config_static_config_spare_0;
            *(pbuffer +  23) =
                _stat_cfg.spare_host_config_static_config_spare_1;
            i2c_encode_uint16_t(
                    _stat_cfg.algo_range_ignore_threshold_mcps,
                    2,
                    pbuffer +  24);
            *(pbuffer +  26) =
                _stat_cfg.algo_range_ignore_valid_height_mm;
            *(pbuffer +  27) =
                _stat_cfg.algo_range_min_clip;
            *(pbuffer +  28) =
                _stat_cfg.algo_consistency_check_tolerance & 0xF;
            *(pbuffer +  29) =
                _stat_cfg.spare_host_config_static_config_spare_2;
            *(pbuffer +  30) =
                _stat_cfg.sd_config_reset_stages_msb & 0xF;
            *(pbuffer +  31) =
                _stat_cfg.sd_config_reset_stages_lsb;

            return ERROR_NONE;
        }

        error_t i2c_encode_general_config(const uint16_t buf_size, 
                uint8_t  *pbuffer)
        {
            if (GENERAL_CONFIG_I2C_SIZE_BYTES > buf_size) {
                return ERROR_COMMS_BUFFER_TOO_SMALL;
            }

            *(pbuffer +   0) =
                _gen_cfg.gph_config_stream_count_update_value;
            *(pbuffer +   1) =
                _gen_cfg.global_config_stream_divider;
            *(pbuffer +   2) =
                _gen_cfg.system_interrupt_config_gpio;
            *(pbuffer +   3) =
                _gen_cfg.cal_config_vcsel_start & 0x7F;
            i2c_encode_uint16_t(
                    _gen_cfg.cal_config_repeat_rate & 0xFFF,
                    2,
                    pbuffer +   4);
            *(pbuffer +   6) =
                _gen_cfg.global_config_vcsel_width & 0x7F;
            *(pbuffer +   7) =
                _gen_cfg.phasecal_config_timeout_macrop;
            *(pbuffer +   8) =
                _gen_cfg.phasecal_config_target;
            *(pbuffer +   9) =
                _gen_cfg.phasecal_config_override & 0x1;
            *(pbuffer +  11) =
                _gen_cfg.dss_config_roi_mode_control & 0x7;
            i2c_encode_uint16_t(
                    _gen_cfg.system_thresh_rate_high,
                    2,
                    pbuffer +  12);
            i2c_encode_uint16_t(
                    _gen_cfg.system_thresh_rate_low,
                    2,
                    pbuffer +  14);
            i2c_encode_uint16_t(
                    _gen_cfg.dss_config_manual_effective_spads_select,
                    2,
                    pbuffer +  16);
            *(pbuffer +  18) =
                _gen_cfg.dss_config_manual_block_select;
            *(pbuffer +  19) =
                _gen_cfg.dss_config_aperture_attenuation;
            *(pbuffer +  20) =
                _gen_cfg.dss_config_max_spads_limit;
            *(pbuffer +  21) =
                _gen_cfg.dss_config_min_spads_limit;

            return ERROR_NONE;
        }

        error_t i2c_encode_timing_config(const uint16_t buf_size, 
                uint8_t  *pbuffer)
        {
            if (TIMING_CONFIG_I2C_SIZE_BYTES > buf_size) {
                return ERROR_COMMS_BUFFER_TOO_SMALL;
            }

            *(pbuffer +   0) =
                _tim_cfg.mm_config_timeout_macrop_a_hi & 0xF;
            *(pbuffer +   1) =
                _tim_cfg.mm_config_timeout_macrop_a_lo;
            *(pbuffer +   2) =
                _tim_cfg.mm_config_timeout_macrop_b_hi & 0xF;
            *(pbuffer +   3) =
                _tim_cfg.mm_config_timeout_macrop_b_lo;
            *(pbuffer +   4) =
                _tim_cfg.range_config_timeout_macrop_a_hi & 0xF;
            *(pbuffer +   5) =
                _tim_cfg.range_config_timeout_macrop_a_lo;
            *(pbuffer +   6) =
                _tim_cfg.range_config_vcsel_period_a & 0x3F;
            *(pbuffer +   7) =
                _tim_cfg.range_config_timeout_macrop_b_hi & 0xF;
            *(pbuffer +   8) =
                _tim_cfg.range_config_timeout_macrop_b_lo;
            *(pbuffer +   9) =
                _tim_cfg.range_config_vcsel_period_b & 0x3F;
            i2c_encode_uint16_t(
                    _tim_cfg.range_config_sigma_thresh,
                    2,
                    pbuffer +  10);
            i2c_encode_uint16_t(
                    _tim_cfg.range_config_min_count_rate_rtn_limit_mcps,
                    2,
                    pbuffer +  12);
            *(pbuffer +  14) =
                _tim_cfg.range_config_valid_phase_low;
            *(pbuffer +  15) =
                _tim_cfg.range_config_valid_phase_high;
            i2c_encode_uint32_t(
                    _tim_cfg.system_intermeasurement_period,
                    4,
                    pbuffer +  18);
            *(pbuffer +  22) =
                _tim_cfg.system_fractional_enable & 0x1;

            return ERROR_NONE;
        }

        error_t i2c_encode_dynamic_config(const uint16_t buf_size, 
                uint8_t  *pbuffer)
        {
            if (DYNAMIC_CONFIG_I2C_SIZE_BYTES > buf_size) {
                return ERROR_COMMS_BUFFER_TOO_SMALL;
            }

            *(pbuffer +   0) =
                _dyn_cfg.system_grouped_parameter_hold_0 & 0x3;
            i2c_encode_uint16_t(
                    _dyn_cfg.system_thresh_high,
                    2,
                    pbuffer +   1);
            i2c_encode_uint16_t(
                    _dyn_cfg.system_thresh_low,
                    2,
                    pbuffer +   3);
            *(pbuffer +   5) =
                _dyn_cfg.system_enable_xtalk_per_quadrant & 0x1;
            *(pbuffer +   6) =
                _dyn_cfg.system_seed_config & 0x7;
            *(pbuffer +   7) =
                _dyn_cfg.sd_config_woi_sd0;
            *(pbuffer +   8) =
                _dyn_cfg.sd_config_woi_sd1;
            *(pbuffer +   9) =
                _dyn_cfg.sd_config_initial_phase_sd0 & 0x7F;
            *(pbuffer +  10) =
                _dyn_cfg.sd_config_initial_phase_sd1 & 0x7F;
            *(pbuffer +  11) =
                _dyn_cfg.system_grouped_parameter_hold_1 & 0x3;
            *(pbuffer +  12) =
                _dyn_cfg.sd_config_first_order_select & 0x3;
            *(pbuffer +  13) =
                _dyn_cfg.sd_config_quantifier & 0xF;
            *(pbuffer +  14) =
                _dyn_cfg.roi_config_user_roi_centre_spad;
            *(pbuffer +  15) =
                _dyn_cfg.roi_config_user_roi_requested_global_xy_size;
            *(pbuffer +  16) =
                _dyn_cfg.system_sequence_config;
            *(pbuffer +  17) =
                _dyn_cfg.system_grouped_parameter_hold & 0x3;

            return ERROR_NONE;
        }

        error_t i2c_decode_nvm_copy_data(const uint16_t buf_size, 
                uint8_t *pbuffer, nvm_copy_data_t *pdata)
        {
            if (NVM_COPY_DATA_I2C_SIZE_BYTES > buf_size) {
                return ERROR_COMMS_BUFFER_TOO_SMALL;
            }

            pdata->identification_model_id =
                (*(pbuffer +   0));
            pdata->identification_module_type =
                (*(pbuffer +   1));
            pdata->identification_revision_id =
                (*(pbuffer +   2));
            pdata->identification_module_id =
                (i2c_decode_uint16_t(2, pbuffer +   3));
            pdata->ana_config_fast_osc_trim_max =
                (*(pbuffer +   5)) & 0x7F;
            pdata->ana_config_fast_osc_freq_set =
                (*(pbuffer +   6)) & 0x7;
            pdata->ana_config_vcsel_trim =
                (*(pbuffer +   7)) & 0x7;
            pdata->ana_config_vcsel_selion =
                (*(pbuffer +   8)) & 0x3F;
            pdata->ana_config_vcsel_selion_max =
                (*(pbuffer +   9)) & 0x3F;
            pdata->protected_laser_safety_lock_bit =
                (*(pbuffer +  10)) & 0x1;
            pdata->laser_safety_key =
                (*(pbuffer +  11)) & 0x7F;
            pdata->laser_safety_key_ro =
                (*(pbuffer +  12)) & 0x1;
            pdata->laser_safety_clip =
                (*(pbuffer +  13)) & 0x3F;
            pdata->laser_safety_mult =
                (*(pbuffer +  14)) & 0x3F;
            pdata->global_config_spad_enables_rtn_0 =
                (*(pbuffer +  15));
            pdata->global_config_spad_enables_rtn_1 =
                (*(pbuffer +  16));
            pdata->global_config_spad_enables_rtn_2 =
                (*(pbuffer +  17));
            pdata->global_config_spad_enables_rtn_3 =
                (*(pbuffer +  18));
            pdata->global_config_spad_enables_rtn_4 =
                (*(pbuffer +  19));
            pdata->global_config_spad_enables_rtn_5 =
                (*(pbuffer +  20));
            pdata->global_config_spad_enables_rtn_6 =
                (*(pbuffer +  21));
            pdata->global_config_spad_enables_rtn_7 =
                (*(pbuffer +  22));
            pdata->global_config_spad_enables_rtn_8 =
                (*(pbuffer +  23));
            pdata->global_config_spad_enables_rtn_9 =
                (*(pbuffer +  24));
            pdata->global_config_spad_enables_rtn_10 =
                (*(pbuffer +  25));
            pdata->global_config_spad_enables_rtn_11 =
                (*(pbuffer +  26));
            pdata->global_config_spad_enables_rtn_12 =
                (*(pbuffer +  27));
            pdata->global_config_spad_enables_rtn_13 =
                (*(pbuffer +  28));
            pdata->global_config_spad_enables_rtn_14 =
                (*(pbuffer +  29));
            pdata->global_config_spad_enables_rtn_15 =
                (*(pbuffer +  30));
            pdata->global_config_spad_enables_rtn_16 =
                (*(pbuffer +  31));
            pdata->global_config_spad_enables_rtn_17 =
                (*(pbuffer +  32));
            pdata->global_config_spad_enables_rtn_18 =
                (*(pbuffer +  33));
            pdata->global_config_spad_enables_rtn_19 =
                (*(pbuffer +  34));
            pdata->global_config_spad_enables_rtn_20 =
                (*(pbuffer +  35));
            pdata->global_config_spad_enables_rtn_21 =
                (*(pbuffer +  36));
            pdata->global_config_spad_enables_rtn_22 =
                (*(pbuffer +  37));
            pdata->global_config_spad_enables_rtn_23 =
                (*(pbuffer +  38));
            pdata->global_config_spad_enables_rtn_24 =
                (*(pbuffer +  39));
            pdata->global_config_spad_enables_rtn_25 =
                (*(pbuffer +  40));
            pdata->global_config_spad_enables_rtn_26 =
                (*(pbuffer +  41));
            pdata->global_config_spad_enables_rtn_27 =
                (*(pbuffer +  42));
            pdata->global_config_spad_enables_rtn_28 =
                (*(pbuffer +  43));
            pdata->global_config_spad_enables_rtn_29 =
                (*(pbuffer +  44));
            pdata->global_config_spad_enables_rtn_30 =
                (*(pbuffer +  45));
            pdata->global_config_spad_enables_rtn_31 =
                (*(pbuffer +  46));
            pdata->roi_config_mode_roi_centre_spad =
                (*(pbuffer +  47));
            pdata->roi_config_mode_roi_xy_size =
                (*(pbuffer +  48));

            return ERROR_NONE;
        }

        error_t get_nvm_copy_data(nvm_copy_data_t * pdata)
        {
            uint8_t comms_buffer[NVM_COPY_DATA_I2C_SIZE_BYTES] = {};

            auto status = read_bytes(_device, _i2c_address, 
                    RGSTR_IDENTIFICATION_MODEL_ID, 
                    NVM_COPY_DATA_I2C_SIZE_BYTES, comms_buffer);

            if (status == ERROR_NONE) {
                status = i2c_decode_nvm_copy_data(NVM_COPY_DATA_I2C_SIZE_BYTES,
                        comms_buffer, pdata);
            }

            return status;
        }

        error_t i2c_encode_static_nvm_managed(uint16_t buf_size, uint8_t *pbuffer)
        {
            error_t status = ERROR_NONE;

            if (STATIC_NVM_MANAGED_I2C_SIZE_BYTES > buf_size)
                return ERROR_COMMS_BUFFER_TOO_SMALL;

            *(pbuffer +   0) =
                _stat_nvm.i2c_slave_device_address & 0x7F;
            *(pbuffer +   1) =
                _stat_nvm.ana_config_vhv_ref_sel_vddpix & 0xF;
            *(pbuffer +   2) =
                _stat_nvm.ana_config_vhv_ref_sel_vquench & 0x7F;
            *(pbuffer +   3) =
                _stat_nvm.ana_config_reg_avdd1v2_sel & 0x3;
            *(pbuffer +   4) =
                _stat_nvm.ana_config_fast_osc_trim & 0x7F;
            i2c_encode_uint16_t(
                    _stat_nvm.osc_measured_fast_osc_frequency,
                    2,
                    pbuffer +   5);
            *(pbuffer +   7) =
                _stat_nvm.vhv_config_timeout_macrop_loop_bound;
            *(pbuffer +   8) =
                _stat_nvm.vhv_config_count_thresh;
            *(pbuffer +   9) =
                _stat_nvm.vhv_config_offset & 0x3F;
            *(pbuffer +  10) =
                _stat_nvm.vhv_config_init;

            return status;
        }

        uint32_t calc_macro_period_us(const uint16_t fast_osc_frequency,
                const uint8_t vcsel_period)
        {
            uint32_t pll_period_us = (0x01 << 30) / fast_osc_frequency;

            uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

            uint32_t macro_period_us =
                (uint32_t)MACRO_PERIOD_VCSEL_PERIODS *
                pll_period_us;

            macro_period_us = macro_period_us >> 6;
            macro_period_us = macro_period_us * (uint32_t)vcsel_period_pclks;
            macro_period_us = macro_period_us >> 6;

            return macro_period_us;
        }

        error_t calc_timeout_register_values(
                const uint32_t phasecal_config_timeout_us,
                const uint32_t mm_config_timeout_us,
                const uint32_t range_config_timeout_us,
                const uint16_t fast_osc_frequency)
        {

            error_t status = ERROR_NONE;

            uint32_t macro_period_us    = 0;
            uint32_t timeout_mclks      = 0;
            uint16_t timeout_encoded    = 0;

            if (fast_osc_frequency == 0) {
                status = ERROR_DIVISION_BY_ZERO;
            } else {

                macro_period_us =
                    calc_macro_period_us(
                            fast_osc_frequency,
                            _tim_cfg.range_config_vcsel_period_a);

                timeout_mclks =
                    calc_timeout_mclks(
                            phasecal_config_timeout_us,
                            macro_period_us);

                if (timeout_mclks > 0xFF)
                    timeout_mclks = 0xFF;

                _gen_cfg.phasecal_config_timeout_macrop =
                    (uint8_t)timeout_mclks;

                timeout_encoded =
                    calc_encoded_timeout(
                            mm_config_timeout_us,
                            macro_period_us);

                _tim_cfg.mm_config_timeout_macrop_a_hi =
                    (uint8_t)((timeout_encoded & 0xFF00) >> 8);
                _tim_cfg.mm_config_timeout_macrop_a_lo =
                    (uint8_t) (timeout_encoded & 0x00FF);

                timeout_encoded =
                    calc_encoded_timeout(
                            range_config_timeout_us,
                            macro_period_us);

                _tim_cfg.range_config_timeout_macrop_a_hi =
                    (uint8_t)((timeout_encoded & 0xFF00) >> 8);
                _tim_cfg.range_config_timeout_macrop_a_lo =
                    (uint8_t) (timeout_encoded & 0x00FF);

                macro_period_us =
                    calc_macro_period_us(
                            fast_osc_frequency,
                            _tim_cfg.range_config_vcsel_period_b);

                timeout_encoded =
                    calc_encoded_timeout(
                            mm_config_timeout_us,
                            macro_period_us);

                _tim_cfg.mm_config_timeout_macrop_b_hi =
                    (uint8_t)((timeout_encoded & 0xFF00) >> 8);
                _tim_cfg.mm_config_timeout_macrop_b_lo =
                    (uint8_t) (timeout_encoded & 0x00FF);

                timeout_encoded = calc_encoded_timeout(
                        range_config_timeout_us,
                        macro_period_us);

                _tim_cfg.range_config_timeout_macrop_b_hi =
                    (uint8_t)((timeout_encoded & 0xFF00) >> 8);
                _tim_cfg.range_config_timeout_macrop_b_lo =
                    (uint8_t) (timeout_encoded & 0x00FF);
            }

            return status;

        }

        void config_low_power_auto_mode(void)
        {
            _low_power_auto_data.is_low_power_auto_mode = 1;

            _low_power_auto_data.low_power_auto_range_count = 0;

            _dyn_cfg.system_sequence_config = 
                SEQUENCE_VHV_EN | 
                SEQUENCE_PHASECAL_EN | 
                SEQUENCE_DSS1_EN | 
                SEQUENCE_RANGE_EN;

            _gen_cfg.dss_config_manual_effective_spads_select = 200 << 8;
            _gen_cfg.dss_config_roi_mode_control =
                DEVICEDSSMODE_REQUESTED_EFFFECTIVE_SPADS;
        }

        error_t get_timeouts_us(
                uint32_t *pphasecal_config_timeout_us,
                uint32_t *pmm_config_timeout_us,
                uint32_t *prange_config_timeout_us)
        {

            error_t  status = ERROR_NONE;

            uint32_t  macro_period_us = 0;
            uint16_t  timeout_encoded = 0;

            if (_stat_nvm.osc_measured_fast_osc_frequency == 0)
                status = ERROR_DIVISION_BY_ZERO;

            if (status == ERROR_NONE) {

                macro_period_us =
                    calc_macro_period_us(
                            _stat_nvm.osc_measured_fast_osc_frequency,
                            _tim_cfg.range_config_vcsel_period_a);

                *pphasecal_config_timeout_us =
                    calc_timeout_us(
                            (uint32_t)_gen_cfg.phasecal_config_timeout_macrop,
                            macro_period_us);

                timeout_encoded =
                    (uint16_t)_tim_cfg.mm_config_timeout_macrop_a_hi;
                timeout_encoded = (timeout_encoded << 8) +
                    (uint16_t)_tim_cfg.mm_config_timeout_macrop_a_lo;

                *pmm_config_timeout_us =
                    calc_decoded_timeout_us(
                            timeout_encoded,
                            macro_period_us);

                timeout_encoded =
                    (uint16_t)_tim_cfg.range_config_timeout_macrop_a_hi;
                timeout_encoded = (timeout_encoded << 8) +
                    (uint16_t)_tim_cfg.range_config_timeout_macrop_a_lo;

                *prange_config_timeout_us =
                    calc_decoded_timeout_us(
                            timeout_encoded,
                            macro_period_us);

                _phasecal_config_timeout_us = *pphasecal_config_timeout_us;
                _mm_config_timeout_us       = *pmm_config_timeout_us;
                _range_config_timeout_us    = *prange_config_timeout_us;
            }

            return status;
        }

        error_t get_sequence_config_bit(const device_sequence_config_t bit_id,
                uint8_t *pvalue)
        {
            error_t  status = ERROR_NONE;

            uint8_t  bit_mask        = 0x01;

            if (bit_id <= DEVICESEQUENCECONFIG_RANGE) {

                if (bit_id > 0) {
                    bit_mask  = 0x01 << bit_id;
                }

                *pvalue = _dyn_cfg.system_sequence_config & bit_mask;

                if (bit_id > 0) {
                    *pvalue  = *pvalue >> bit_id;
                }

            } else {
                status = ERROR_INVALID_PARAMS;
            }

            return status;
        }

        void init_refspadchar_config_struct(refspadchar_config_t   *pdata)
        {
            pdata->device_test_mode =
                TUNINGPARAM_REFSPADCHAR_DEVICE_TEST_MODE_DEFAULT;
            pdata->vcsel_period              =
                TUNINGPARAM_REFSPADCHAR_VCSEL_PERIOD_DEFAULT;
            pdata->timeout_us                =
                TUNINGPARAM_REFSPADCHAR_PHASECAL_TIMEOUT_US_DEFAULT;
            pdata->target_count_rate_mcps    =
                TUNINGPARAM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS_DEFAULT;
            pdata->min_count_rate_limit_mcps =
                TUNINGPARAM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS_DEFAULT;
            pdata->max_count_rate_limit_mcps =
                TUNINGPARAM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS_DEFAULT;
        }

        error_t init_ssc_config_struct(ssc_config_t  *pdata)
        {
            error_t  status = ERROR_NONE;

            pdata->array_select = DEVICESSCARRAY_RTN;

            pdata->vcsel_period =
                TUNINGPARAM_SPADMAP_VCSEL_PERIOD_DEFAULT;

            pdata->vcsel_start  =
                TUNINGPARAM_SPADMAP_VCSEL_START_DEFAULT;

            pdata->vcsel_width  = 0x02;

            pdata->timeout_us   = 36000;

            pdata->rate_limit_mcps =
                TUNINGPARAM_SPADMAP_RATE_LIMIT_MCPS_DEFAULT;

            return status;
        }

        void init_xtalk_config_struct(void)
        {
            _xtalk_cfg.algo_crosstalk_compensation_plane_offset_kcps      =
                _customer.algo_crosstalk_compensation_plane_offset_kcps;
            _xtalk_cfg.algo_crosstalk_compensation_x_plane_gradient_kcps  =
                _customer.algo_crosstalk_compensation_x_plane_gradient_kcps;
            _xtalk_cfg.algo_crosstalk_compensation_y_plane_gradient_kcps  =
                _customer.algo_crosstalk_compensation_y_plane_gradient_kcps;

            _xtalk_cfg.nvm_default_crosstalk_compensation_plane_offset_kcps      =
                (uint32_t)_customer.algo_crosstalk_compensation_plane_offset_kcps;
            _xtalk_cfg.nvm_default_crosstalk_compensation_x_plane_gradient_kcps  =
                _customer.algo_crosstalk_compensation_x_plane_gradient_kcps;
            _xtalk_cfg.nvm_default_crosstalk_compensation_y_plane_gradient_kcps  =
                _customer.algo_crosstalk_compensation_y_plane_gradient_kcps;

            _xtalk_cfg.lite_mode_crosstalk_margin_kcps                     =
                TUNINGPARAM_LITE_XTALK_MARGIN_KCPS_DEFAULT;

            _xtalk_cfg.crosstalk_range_ignore_threshold_mult =
                TUNINGPARAM_LITE_RIT_MULT_DEFAULT;

            if ((_xtalk_cfg.algo_crosstalk_compensation_plane_offset_kcps == 0x00)
                    && (_xtalk_cfg.algo_crosstalk_compensation_x_plane_gradient_kcps == 
                        0x00)
                    && (_xtalk_cfg.algo_crosstalk_compensation_y_plane_gradient_kcps == 
                        0x00))
                _xtalk_cfg.global_crosstalk_compensation_enable = 0x00;
            else
                _xtalk_cfg.global_crosstalk_compensation_enable = 0x01;

            if (_xtalk_cfg.global_crosstalk_compensation_enable == 0x01) {
                _xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps =
                    calc_range_ignore_threshold(
                            _xtalk_cfg.algo_crosstalk_compensation_plane_offset_kcps,
                            _xtalk_cfg.algo_crosstalk_compensation_x_plane_gradient_kcps,
                            _xtalk_cfg.algo_crosstalk_compensation_y_plane_gradient_kcps,
                            _xtalk_cfg.crosstalk_range_ignore_threshold_mult);
            } else {
                _xtalk_cfg.crosstalk_range_ignore_threshold_rate_mcps = 0;
            }
        }

        error_t init_offset_cal_config_struct(offsetcal_config_t * pdata)
        {

            error_t  status = ERROR_NONE;

            pdata->dss_config_target_total_rate_mcps          =
                TUNINGPARAM_OFFSET_CAL_DSS_RATE_MCPS_DEFAULT;

            pdata->phasecal_config_timeout_us                  =
                TUNINGPARAM_OFFSET_CAL_PHASECAL_TIMEOUT_US_DEFAULT;

            pdata->range_config_timeout_us                     =
                TUNINGPARAM_OFFSET_CAL_RANGE_TIMEOUT_US_DEFAULT;

            pdata->mm_config_timeout_us                        =
                TUNINGPARAM_OFFSET_CAL_MM_TIMEOUT_US_DEFAULT;

            pdata->pre_num_of_samples                          =
                TUNINGPARAM_OFFSET_CAL_PRE_SAMPLES_DEFAULT;
            pdata->mm1_num_of_samples                          =
                TUNINGPARAM_OFFSET_CAL_MM1_SAMPLES_DEFAULT;
            pdata->mm2_num_of_samples                          =
                TUNINGPARAM_OFFSET_CAL_MM2_SAMPLES_DEFAULT;

            return status;
        }

        error_t init_tuning_parm_storage_struct(tuning_parm_storage_t * pdata)
        {
            error_t  status = ERROR_NONE;

            pdata->tp_tuning_parm_version              =
                TUNINGPARAM_VERSION_DEFAULT;
            pdata->tp_tuning_parm_key_table_version    =
                TUNINGPARAM_KEY_TABLE_VERSION_DEFAULT;
            pdata->tp_tuning_parm_lld_version          =
                TUNINGPARAM_LLD_VERSION_DEFAULT;
            pdata->tp_init_phase_rtn_lite_long         =
                TUNINGPARAM_INITIAL_PHASE_RTN_LITE_LONG_RANGE_DEFAULT;
            pdata->tp_init_phase_rtn_lite_med          =
                TUNINGPARAM_INITIAL_PHASE_RTN_LITE_MED_RANGE_DEFAULT;
            pdata->tp_init_phase_rtn_lite_short        =
                TUNINGPARAM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE_DEFAULT;
            pdata->tp_init_phase_ref_lite_long         =
                TUNINGPARAM_INITIAL_PHASE_REF_LITE_LONG_RANGE_DEFAULT;
            pdata->tp_init_phase_ref_lite_med          =
                TUNINGPARAM_INITIAL_PHASE_REF_LITE_MED_RANGE_DEFAULT;
            pdata->tp_init_phase_ref_lite_short        =
                TUNINGPARAM_INITIAL_PHASE_REF_LITE_SHORT_RANGE_DEFAULT;
            pdata->tp_consistency_lite_phase_tolerance =
                TUNINGPARAM_CONSISTENCY_LITE_PHASE_TOLERANCE_DEFAULT;
            pdata->tp_phasecal_target                  =
                TUNINGPARAM_PHASECAL_TARGET_DEFAULT;
            pdata->tp_cal_repeat_rate                  =
                TUNINGPARAM_LITE_CAL_REPEAT_RATE_DEFAULT;
            pdata->tp_lite_min_clip                    =
                TUNINGPARAM_LITE_MIN_CLIP_MM_DEFAULT;
            pdata->tp_lite_long_sigma_thresh_mm        =
                TUNINGPARAM_LITE_LONG_SIGMA_THRESH_MM_DEFAULT;
            pdata->tp_lite_med_sigma_thresh_mm         =
                TUNINGPARAM_LITE_MED_SIGMA_THRESH_MM_DEFAULT;
            pdata->tp_lite_short_sigma_thresh_mm       =
                TUNINGPARAM_LITE_SHORT_SIGMA_THRESH_MM_DEFAULT;
            pdata->tp_lite_long_min_count_rate_rtn_mcps  =
                TUNINGPARAM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
            pdata->tp_lite_med_min_count_rate_rtn_mcps   =
                TUNINGPARAM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
            pdata->tp_lite_short_min_count_rate_rtn_mcps =
                TUNINGPARAM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
            pdata->tp_lite_sigma_est_pulse_width_ns      =
                TUNINGPARAM_LITE_SIGMA_EST_PULSE_WIDTH_DEFAULT;
            pdata->tp_lite_sigma_est_amb_width_ns        =
                TUNINGPARAM_LITE_SIGMA_EST_AMB_WIDTH_NS_DEFAULT;
            pdata->tp_lite_sigma_ref_mm                  =
                TUNINGPARAM_LITE_SIGMA_REF_MM_DEFAULT;
            pdata->tp_lite_seed_cfg                      =
                TUNINGPARAM_LITE_SEED_CONFIG_DEFAULT;
            pdata->tp_timed_seed_cfg                     =
                TUNINGPARAM_TIMED_SEED_CONFIG_DEFAULT;
            pdata->tp_lite_quantifier                    =
                TUNINGPARAM_LITE_QUANTIFIER_DEFAULT;
            pdata->tp_lite_first_order_select            =
                TUNINGPARAM_LITE_FIRST_ORDER_SELECT_DEFAULT;

            pdata->tp_dss_target_lite_mcps               =
                TUNINGPARAM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
            pdata->tp_dss_target_timed_mcps              =
                TUNINGPARAM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
            pdata->tp_phasecal_timeout_lite_us           =
                TUNINGPARAM_LITE_PHASECAL_CONFIG_TIMEOUT_US;
            pdata->tp_phasecal_timeout_timed_us          =
                TUNINGPARAM_TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
            pdata->tp_mm_timeout_lite_us                 =
                TUNINGPARAM_LITE_MM_CONFIG_TIMEOUT_US_DEFAULT;
            pdata->tp_mm_timeout_timed_us                =
                TUNINGPARAM_TIMED_MM_CONFIG_TIMEOUT_US_DEFAULT;
            pdata->tp_range_timeout_lite_us              =
                TUNINGPARAM_LITE_RANGE_CONFIG_TIMEOUT_US_DEFAULT;
            pdata->tp_range_timeout_timed_us             =
                TUNINGPARAM_TIMED_RANGE_CONFIG_TIMEOUT_US_DEFAULT;

            pdata->tp_mm_timeout_lpa_us =
                TUNINGPARAM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT;
            pdata->tp_range_timeout_lpa_us =
                TUNINGPARAM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US_DEFAULT;

            return status;
        }

        error_t set_inter_measurement_period_ms(
                const uint32_t inter_measurement_period_ms)
        {
            error_t  status = ERROR_NONE;

            if (_dbg_results.result_osc_calibrate_val == 0)
                status = ERROR_DIVISION_BY_ZERO;

            if (status == ERROR_NONE) {
                _inter_measurement_period_ms = inter_measurement_period_ms;
                _tim_cfg.system_intermeasurement_period =
                    inter_measurement_period_ms * 
                    (uint32_t)_dbg_results.result_osc_calibrate_val;
            }

            return status;
        }

        error_t get_preset_mode_timing_cfg(
                device_preset_mode_t     device_preset_mode,
                uint16_t *pdss_config_target_total_rate_mcps,
                uint32_t *pphasecal_config_timeout_us,
                uint32_t *pmm_config_timeout_us,
                uint32_t *prange_config_timeout_us)
        {
            error_t  status = ERROR_NONE;

            switch (device_preset_mode) {

                case DEVICEPRESETMODE_STANDARD_RANGING:
                case DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE:
                case DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE:
                case DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL:
                case DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL:
                case DEVICEPRESETMODE_OLT:
                    *pdss_config_target_total_rate_mcps =
                        _tuning_params.tp_dss_target_lite_mcps;
                    *pphasecal_config_timeout_us =
                        _tuning_params.tp_phasecal_timeout_lite_us;
                    *pmm_config_timeout_us =
                        _tuning_params.tp_mm_timeout_lite_us;
                    *prange_config_timeout_us =
                        _tuning_params.tp_range_timeout_lite_us;
                    break;

                case DEVICEPRESETMODE_TIMED_RANGING:
                case DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE:
                case DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE:
                case DEVICEPRESETMODE_SINGLESHOT_RANGING:
                    *pdss_config_target_total_rate_mcps =
                        _tuning_params.tp_dss_target_timed_mcps;
                    *pphasecal_config_timeout_us =
                        _tuning_params.tp_phasecal_timeout_timed_us;
                    *pmm_config_timeout_us =
                        _tuning_params.tp_mm_timeout_timed_us;
                    *prange_config_timeout_us =
                        _tuning_params.tp_range_timeout_timed_us;
                    break;

                case DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE:
                case DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE:
                case DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE:
                    *pdss_config_target_total_rate_mcps =
                        _tuning_params.tp_dss_target_timed_mcps;
                    *pphasecal_config_timeout_us =
                        _tuning_params.tp_phasecal_timeout_timed_us;
                    *pmm_config_timeout_us =
                        _tuning_params.tp_mm_timeout_lpa_us;
                    *prange_config_timeout_us =
                        _tuning_params.tp_range_timeout_lpa_us;
                    break;

                default:
                    status = ERROR_INVALID_PARAMS;
                    break;

            }

            return status;
        }

        error_t set_timeouts_us(
                const uint32_t phasecal_config_timeout_us,
                const uint32_t mm_config_timeout_us,
                const uint32_t range_config_timeout_us)
        {
            if (_stat_nvm.osc_measured_fast_osc_frequency == 0) {
                return ERROR_DIVISION_BY_ZERO;
            }

            _phasecal_config_timeout_us = phasecal_config_timeout_us;
            _mm_config_timeout_us       = mm_config_timeout_us;
            _range_config_timeout_us    = range_config_timeout_us;

            return calc_timeout_register_values(phasecal_config_timeout_us,
                    mm_config_timeout_us, range_config_timeout_us,
                    _stat_nvm.osc_measured_fast_osc_frequency);
        }

        error_t preset_mode_standard_ranging(void)
        {
            error_t  status = ERROR_NONE;

            _stat_cfg.dss_config_target_total_rate_mcps               = 0x0A00;
            _stat_cfg.debug_ctrl                                      = 0x00;
            _stat_cfg.test_mode_ctrl                                  = 0x00;
            _stat_cfg.clk_gating_ctrl                                 = 0x00;
            _stat_cfg.nvm_bist_ctrl                                   = 0x00;
            _stat_cfg.nvm_bist_num_nvm_words                          = 0x00;
            _stat_cfg.nvm_bist_start_address                          = 0x00;
            _stat_cfg.host_if_status                                  = 0x00;
            _stat_cfg.pad_i2c_hv_config                               = 0x00;
            _stat_cfg.pad_i2c_hv_extsup_config                        = 0x00;

            _stat_cfg.gpio_hv_pad_ctrl                                = 0x00;

            _stat_cfg.gpio_hv_mux_ctrl  = 
                DEVICEINTERRUPTPOLARITY_ACTIVE_LOW |
                DEVICEGPIOMODE_OUTPUT_RANGE_AND_ERROR_INTERRUPTS;

            _stat_cfg.gpio_tio_hv_status                              = 0x02;
            _stat_cfg.gpio_fio_hv_status                              = 0x00;
            _stat_cfg.ana_config_spad_sel_pswidth                     = 0x02;
            _stat_cfg.ana_config_vcsel_pulse_width_offset             = 0x08;
            _stat_cfg.ana_config_fast_osc_config_ctrl                = 0x00;

            _stat_cfg.sigma_estimator_effective_pulse_width_ns        =
                _tuning_params.tp_lite_sigma_est_pulse_width_ns;
            _stat_cfg.sigma_estimator_effective_ambient_width_ns      =
                _tuning_params.tp_lite_sigma_est_amb_width_ns;
            _stat_cfg.sigma_estimator_sigma_ref_mm                    =
                _tuning_params.tp_lite_sigma_ref_mm;

            _stat_cfg.algo_crosstalk_compensation_valid_height_mm     = 0x01;
            _stat_cfg.spare_host_config_static_config_spare_0         = 0x00;
            _stat_cfg.spare_host_config_static_config_spare_1         = 0x00;

            _stat_cfg.algo_range_ignore_threshold_mcps                = 0x0000;

            _stat_cfg.algo_range_ignore_valid_height_mm               = 0xff;
            _stat_cfg.algo_range_min_clip                             =
                _tuning_params.tp_lite_min_clip;

            _stat_cfg.algo_consistency_check_tolerance               =
                _tuning_params.tp_consistency_lite_phase_tolerance;
            _stat_cfg.spare_host_config_static_config_spare_2         = 0x00;
            _stat_cfg.sd_config_reset_stages_msb                      = 0x00;
            _stat_cfg.sd_config_reset_stages_lsb                      = 0x00;

            _gen_cfg.gph_config_stream_count_update_value           = 0x00;
            _gen_cfg.global_config_stream_divider                   = 0x00;
            _gen_cfg.system_interrupt_config_gpio =
                INTERRUPT_CONFIG_NEW_SAMPLE_READY;
            _gen_cfg.cal_config_vcsel_start                         = 0x0B;

            _gen_cfg.cal_config_repeat_rate                         =
                _tuning_params.tp_cal_repeat_rate;
            _gen_cfg.global_config_vcsel_width                      = 0x02;

            _gen_cfg.phasecal_config_timeout_macrop                 = 0x0D;

            _gen_cfg.phasecal_config_target                         =
                _tuning_params.tp_phasecal_target;
            _gen_cfg.phasecal_config_override                       = 0x00;
            _gen_cfg.dss_config_roi_mode_control =
                DEVICEDSSMODE_TARGET_RATE;

            _gen_cfg.system_thresh_rate_high                        = 0x0000;
            _gen_cfg.system_thresh_rate_low                         = 0x0000;

            _gen_cfg.dss_config_manual_effective_spads_select       = 0x8C00;
            _gen_cfg.dss_config_manual_block_select                 = 0x00;

            _gen_cfg.dss_config_aperture_attenuation                = 0x38;
            _gen_cfg.dss_config_max_spads_limit                     = 0xFF;
            _gen_cfg.dss_config_min_spads_limit                     = 0x01;

            _tim_cfg.mm_config_timeout_macrop_a_hi                   = 0x00;
            _tim_cfg.mm_config_timeout_macrop_a_lo                   = 0x1a;
            _tim_cfg.mm_config_timeout_macrop_b_hi                   = 0x00;
            _tim_cfg.mm_config_timeout_macrop_b_lo                   = 0x20;

            _tim_cfg.range_config_timeout_macrop_a_hi                = 0x01;
            _tim_cfg.range_config_timeout_macrop_a_lo                = 0xCC;

            _tim_cfg.range_config_vcsel_period_a                     = 0x0B;

            _tim_cfg.range_config_timeout_macrop_b_hi                = 0x01;
            _tim_cfg.range_config_timeout_macrop_b_lo                = 0xF5;

            _tim_cfg.range_config_vcsel_period_b                     = 0x09;

            _tim_cfg.range_config_sigma_thresh                       =
                _tuning_params.tp_lite_med_sigma_thresh_mm;

            _tim_cfg.range_config_min_count_rate_rtn_limit_mcps      =
                _tuning_params.tp_lite_med_min_count_rate_rtn_mcps;

            _tim_cfg.range_config_valid_phase_low                    = 0x08;
            _tim_cfg.range_config_valid_phase_high                   = 0x78;
            _tim_cfg.system_intermeasurement_period                  = 0x00000000;
            _tim_cfg.system_fractional_enable                        = 0x00;

            _dyn_cfg.system_grouped_parameter_hold_0                 = 0x01;

            _dyn_cfg.system_thresh_high                              = 0x0000;
            _dyn_cfg.system_thresh_low                               = 0x0000;
            _dyn_cfg.system_enable_xtalk_per_quadrant                = 0x00;
            _dyn_cfg.system_seed_config =
                _tuning_params.tp_lite_seed_cfg;

            _dyn_cfg.sd_config_woi_sd0                               = 0x0B;

            _dyn_cfg.sd_config_woi_sd1                               = 0x09;

            _dyn_cfg.sd_config_initial_phase_sd0                     =
                _tuning_params.tp_init_phase_rtn_lite_med;
            _dyn_cfg.sd_config_initial_phase_sd1                     =
                _tuning_params.tp_init_phase_ref_lite_med;;

            _dyn_cfg.system_grouped_parameter_hold_1                 = 0x01;

            _dyn_cfg.sd_config_first_order_select =
                _tuning_params.tp_lite_first_order_select;
            _dyn_cfg.sd_config_quantifier         =
                _tuning_params.tp_lite_quantifier;

            _dyn_cfg.roi_config_user_roi_centre_spad              = 0xC7;

            _dyn_cfg.roi_config_user_roi_requested_global_xy_size = 0xFF;

            _dyn_cfg.system_sequence_config = 
                SEQUENCE_VHV_EN | 
                SEQUENCE_PHASECAL_EN | 
                SEQUENCE_DSS1_EN | 
                SEQUENCE_DSS2_EN | 
                SEQUENCE_MM2_EN | 
                SEQUENCE_RANGE_EN;

            _dyn_cfg.system_grouped_parameter_hold = 0x02;

            _sys_ctrl.system_stream_count_ctrl = 0x00;
            _sys_ctrl.firmware_enable = 0x01;
            _sys_ctrl.system_interrupt_clear = CLEAR_RANGE_INT;

            _sys_ctrl.system_mode_start = 
                DEVICESCHEDULERMODE_STREAMING | 
                DEVICEREADOUTMODE_SINGLE_SD | 
                DEVICEMEASUREMENTMODE_BACKTOBACK;

            return status;
        }

        error_t preset_mode_standard_ranging_short_range(void)
        {
            error_t  status = ERROR_NONE;

            status = preset_mode_standard_ranging();

            if (status == ERROR_NONE) {

                _tim_cfg.range_config_vcsel_period_a                = 0x07;
                _tim_cfg.range_config_vcsel_period_b                = 0x05;
                _tim_cfg.range_config_sigma_thresh                  =
                    _tuning_params.tp_lite_short_sigma_thresh_mm;
                _tim_cfg.range_config_min_count_rate_rtn_limit_mcps =
                    _tuning_params.tp_lite_short_min_count_rate_rtn_mcps;
                _tim_cfg.range_config_valid_phase_low               = 0x08;
                _tim_cfg.range_config_valid_phase_high              = 0x38;

                _dyn_cfg.sd_config_woi_sd0                         = 0x07;
                _dyn_cfg.sd_config_woi_sd1                         = 0x05;
                _dyn_cfg.sd_config_initial_phase_sd0               =
                    _tuning_params.tp_init_phase_rtn_lite_short;
                _dyn_cfg.sd_config_initial_phase_sd1               =
                    _tuning_params.tp_init_phase_ref_lite_short;
            }

            return status;
        }

        error_t preset_mode_standard_ranging_long_range(void)
        {
            error_t  status = preset_mode_standard_ranging();

            if (status == ERROR_NONE) {

                _tim_cfg.range_config_vcsel_period_a                = 0x0F;
                _tim_cfg.range_config_vcsel_period_b                = 0x0D;
                _tim_cfg.range_config_sigma_thresh                  =
                    _tuning_params.tp_lite_long_sigma_thresh_mm;
                _tim_cfg.range_config_min_count_rate_rtn_limit_mcps =
                    _tuning_params.tp_lite_long_min_count_rate_rtn_mcps;
                _tim_cfg.range_config_valid_phase_low               = 0x08;
                _tim_cfg.range_config_valid_phase_high              = 0xB8;

                _dyn_cfg.sd_config_woi_sd0                         = 0x0F;
                _dyn_cfg.sd_config_woi_sd1                         = 0x0D;
                _dyn_cfg.sd_config_initial_phase_sd0               =
                    _tuning_params.tp_init_phase_rtn_lite_long;
                _dyn_cfg.sd_config_initial_phase_sd1               =
                    _tuning_params.tp_init_phase_ref_lite_long;
            }

            return status;
        }

        error_t preset_mode_standard_ranging_mm1_cal(void)
        {
            error_t  status =  preset_mode_standard_ranging();

            if (status == ERROR_NONE) {

                _gen_cfg.dss_config_roi_mode_control =
                    DEVICEDSSMODE_REQUESTED_EFFFECTIVE_SPADS;

                _dyn_cfg.system_sequence_config  = 
                    SEQUENCE_VHV_EN | 
                    SEQUENCE_PHASECAL_EN |
                    SEQUENCE_DSS1_EN | 
                    SEQUENCE_DSS2_EN | 
                    SEQUENCE_MM1_EN;
            }

            return status;
        }

        error_t preset_mode_standard_ranging_mm2_cal(void)
        {
            error_t  status = ERROR_NONE;

            status = preset_mode_standard_ranging();

            if (status == ERROR_NONE) {

                _gen_cfg.dss_config_roi_mode_control =
                    DEVICEDSSMODE_REQUESTED_EFFFECTIVE_SPADS;

                _dyn_cfg.system_sequence_config  = 
                    SEQUENCE_VHV_EN | 
                    SEQUENCE_PHASECAL_EN | 
                    SEQUENCE_DSS1_EN | 
                    SEQUENCE_DSS2_EN | 
                    SEQUENCE_MM2_EN;
            }

            return status;
        }

        error_t preset_mode_timed_ranging(void)
        {
            error_t  status = ERROR_NONE;

            status = preset_mode_standard_ranging();

            if (status == ERROR_NONE) {

                _dyn_cfg.system_grouped_parameter_hold = 0x00;

                _tim_cfg.range_config_timeout_macrop_a_hi                = 0x00;
                _tim_cfg.range_config_timeout_macrop_a_lo                = 0xB1;

                _tim_cfg.range_config_timeout_macrop_b_hi                = 0x00;
                _tim_cfg.range_config_timeout_macrop_b_lo                = 0xD4;

                _tim_cfg.system_intermeasurement_period = 0x00000600;
                _dyn_cfg.system_seed_config =
                    _tuning_params.tp_timed_seed_cfg;

                _sys_ctrl.system_mode_start =
                    DEVICESCHEDULERMODE_PSEUDO_SOLO | 
                    DEVICEREADOUTMODE_SINGLE_SD     | 
                    DEVICEMEASUREMENTMODE_TIMED;
            }

            return status;
        }

        error_t preset_mode_timed_ranging_short_range(void)
        {
            error_t  status = ERROR_NONE;

            status = preset_mode_standard_ranging_short_range();

            if (status == ERROR_NONE) {

                _dyn_cfg.system_grouped_parameter_hold = 0x00;

                _tim_cfg.range_config_timeout_macrop_a_hi                = 0x01;
                _tim_cfg.range_config_timeout_macrop_a_lo                = 0x84;

                _tim_cfg.range_config_timeout_macrop_b_hi                = 0x01;
                _tim_cfg.range_config_timeout_macrop_b_lo                = 0xB1;

                _tim_cfg.system_intermeasurement_period = 0x00000600;
                _dyn_cfg.system_seed_config =
                    _tuning_params.tp_timed_seed_cfg;

                _sys_ctrl.system_mode_start =
                    DEVICESCHEDULERMODE_PSEUDO_SOLO | 
                    DEVICEREADOUTMODE_SINGLE_SD     | 
                    DEVICEMEASUREMENTMODE_TIMED;
            }

            return status;
        }

        error_t preset_mode_timed_ranging_long_range(void)
        {
            error_t  status = preset_mode_standard_ranging_long_range();

            if (status == ERROR_NONE) {

                _dyn_cfg.system_grouped_parameter_hold = 0x00;

                _tim_cfg.range_config_timeout_macrop_a_hi                = 0x00;
                _tim_cfg.range_config_timeout_macrop_a_lo                = 0x97;

                _tim_cfg.range_config_timeout_macrop_b_hi                = 0x00;
                _tim_cfg.range_config_timeout_macrop_b_lo                = 0xB1;

                _tim_cfg.system_intermeasurement_period = 0x00000600;
                _dyn_cfg.system_seed_config =
                    _tuning_params.tp_timed_seed_cfg;

                _sys_ctrl.system_mode_start =
                    DEVICESCHEDULERMODE_PSEUDO_SOLO | 
                    DEVICEREADOUTMODE_SINGLE_SD     | 
                    DEVICEMEASUREMENTMODE_TIMED;
            }

            return status;
        }

        error_t preset_mode_low_power_auto_ranging(void)
        {
            error_t  status = preset_mode_timed_ranging();

            if (status == ERROR_NONE) {
                config_low_power_auto_mode();
            }

            return status;
        }

        error_t preset_mode_low_power_auto_short_ranging(void)
        {

            error_t status = preset_mode_timed_ranging_short_range();

            if (status == ERROR_NONE) {
                config_low_power_auto_mode();
            }

            return status;
        }

        error_t preset_mode_low_power_auto_long_ranging(void)
        {

            error_t  status = preset_mode_timed_ranging_long_range();

            if (status == ERROR_NONE) {
                config_low_power_auto_mode();
            }

            return status;
        }

        error_t preset_mode_singleshot_ranging(void)
        {
            error_t  status = ERROR_NONE;

            status = preset_mode_standard_ranging();

            if (status == ERROR_NONE) {

                _dyn_cfg.system_grouped_parameter_hold = 0x00;

                _tim_cfg.range_config_timeout_macrop_a_hi                = 0x00;
                _tim_cfg.range_config_timeout_macrop_a_lo                = 0xB1;

                _tim_cfg.range_config_timeout_macrop_b_hi                = 0x00;
                _tim_cfg.range_config_timeout_macrop_b_lo                = 0xD4;

                _dyn_cfg.system_seed_config = _tuning_params.tp_timed_seed_cfg;

                _sys_ctrl.system_mode_start = 
                    DEVICESCHEDULERMODE_PSEUDO_SOLO | 
                    DEVICEREADOUTMODE_SINGLE_SD     | 
                    DEVICEMEASUREMENTMODE_SINGLESHOT;
            }

            return status;
        }

        error_t preset_mode_olt(void)
        {

            error_t  status = preset_mode_standard_ranging();

            if (status == ERROR_NONE) {

                _sys_ctrl.system_stream_count_ctrl  = 0x01;
            }

            return status;
        }

        void init_ll_driver_state(const device_state_t device_state)
        {
            ll_driver_state_t *driver_state = &(_ll_state);

            driver_state->cfg_device_state  = device_state;
            driver_state->cfg_stream_count  = 0;
            driver_state->cfg_gph_id        = GROUPEDPARAMETERHOLD_ID_MASK;
            driver_state->cfg_timing_status = 0;

            driver_state->rd_device_state   = device_state;
            driver_state->rd_stream_count   = 0;
            driver_state->rd_gph_id         = GROUPEDPARAMETERHOLD_ID_MASK;
            driver_state->rd_timing_status  = 0;
        }

        error_t set_preset_mode(
                const device_preset_mode_t  device_preset_mode,
                const uint16_t dss_config_target_total_rate_mcps,
                const uint32_t phasecal_config_timeout_us,
                const uint32_t mm_config_timeout_us,
                const uint32_t range_config_timeout_us,
                const uint32_t inter_measurement_period_ms)
        {
            error_t  status = ERROR_NONE;

            _preset_mode                 = device_preset_mode;
            _mm_config_timeout_us        = mm_config_timeout_us;
            _range_config_timeout_us     = range_config_timeout_us;
            _inter_measurement_period_ms = inter_measurement_period_ms;

            init_ll_driver_state(DEVICESTATE_SW_STANDBY);

            switch (device_preset_mode) {

                case DEVICEPRESETMODE_STANDARD_RANGING:
                    status = preset_mode_standard_ranging();
                    break;

                case DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE:
                    status = preset_mode_standard_ranging_short_range();
                    break;

                case DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE:
                    status = preset_mode_standard_ranging_long_range();
                    break;

                case DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL:
                    status = preset_mode_standard_ranging_mm1_cal();
                    break;

                case DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL:
                    status = preset_mode_standard_ranging_mm2_cal();
                    break;

                case DEVICEPRESETMODE_TIMED_RANGING:
                    status = preset_mode_timed_ranging();
                    break;

                case DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE:
                    status = preset_mode_timed_ranging_short_range();
                    break;

                case DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE:
                    status = preset_mode_timed_ranging_long_range();
                    break;

                case DEVICEPRESETMODE_OLT:
                    status = preset_mode_olt();
                    break;

                case DEVICEPRESETMODE_SINGLESHOT_RANGING:
                    status = preset_mode_singleshot_ranging();
                    break;

                case DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE:
                    status = preset_mode_low_power_auto_short_ranging();
                    break;

                case DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE:
                    status = preset_mode_low_power_auto_ranging();
                    break;

                case DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE:
                    status = preset_mode_low_power_auto_long_ranging();
                    break;

                default:
                    status = ERROR_INVALID_PARAMS;
                    break;
            }

            if (status == ERROR_NONE) {

                _stat_cfg.dss_config_target_total_rate_mcps =
                    dss_config_target_total_rate_mcps;
                _dss_config_target_total_rate_mcps    =
                    dss_config_target_total_rate_mcps;

            }

            if (status == ERROR_NONE) {
                status = set_timeouts_us(
                        phasecal_config_timeout_us,
                        mm_config_timeout_us,
                        range_config_timeout_us);
            }

            if (status == ERROR_NONE) {
                status = set_inter_measurement_period_ms(inter_measurement_period_ms);
            }

            return status;
        }

        error_t compute_device_preset_mode(
                const preset_mode_t presetMode,
                const distanceMode_t distance_mode,
                device_preset_mode_t *pdevice_presetMode)
        {
            error_t status = ERROR_NONE;

            device_preset_mode_t LightModes[3] = {
                DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE,
                DEVICEPRESETMODE_STANDARD_RANGING,
                DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE};

            device_preset_mode_t TimedModes[3] = {
                DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE,
                DEVICEPRESETMODE_TIMED_RANGING,
                DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE};

            device_preset_mode_t LowPowerTimedModes[3] = {
                DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE,
                DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE,
                DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE};

            *pdevice_presetMode = DEVICEPRESETMODE_STANDARD_RANGING;

            uint8_t DistIdx = 0;
            switch (distance_mode) {
                case DISTANCEMODE_SHORT:
                    DistIdx = 0;
                    break;
                case DISTANCEMODE_MEDIUM:
                    DistIdx = 1;
                    break;
                default:
                    DistIdx = 2;
            }

            switch (presetMode) {
                case PRESETMODE_LITE_RANGING:
                    *pdevice_presetMode = LightModes[DistIdx];
                    break;

                case PRESETMODE_AUTONOMOUS:
                    *pdevice_presetMode = TimedModes[DistIdx];
                    break;

                case PRESETMODE_LOWPOWER_AUTONOMOUS:
                    *pdevice_presetMode = LowPowerTimedModes[DistIdx];
                    break;

                default:
                    status = ERROR_MODE_NOT_SUPPORTED;
            }

            return status;
        }

        error_t helper_set_preset_mode(
                const preset_mode_t presetMode,
                const distanceMode_t distance_mode,
                const uint32_t inter_measurement_period_ms)
        {
            error_t status = ERROR_NONE;

            auto measurement_mode = 
                (presetMode == PRESETMODE_AUTONOMOUS ||
                 presetMode == PRESETMODE_LOWPOWER_AUTONOMOUS) ?
                DEVICEMEASUREMENTMODE_TIMED :
                DEVICEMEASUREMENTMODE_BACKTOBACK;

            device_preset_mode_t  device_preset_mode = {};
            status = compute_device_preset_mode(presetMode, distance_mode,
                    &device_preset_mode);

            uint16_t dss_config_target_total_rate_mcps = 0;
            uint32_t phasecal_config_timeout_us= 0;
            uint32_t mm_config_timeout_us= 0;
            uint32_t lld_range_config_timeout_us= 0;

            if (status == ERROR_NONE) {
                status =  get_preset_mode_timing_cfg(
                        device_preset_mode,
                        &dss_config_target_total_rate_mcps,
                        &phasecal_config_timeout_us,
                        &mm_config_timeout_us,
                        &lld_range_config_timeout_us);
            }

            if (status == ERROR_NONE) {
                status = set_preset_mode(
                        device_preset_mode,
                        dss_config_target_total_rate_mcps,
                        phasecal_config_timeout_us,
                        mm_config_timeout_us,
                        lld_range_config_timeout_us,
                        inter_measurement_period_ms);
            }

            if (status == ERROR_NONE) {
                _measurement_mode = measurement_mode;
            }

            if (status == ERROR_NONE) {
                _current_parameters.presetMode = presetMode;
            }

            return status;
        }

        error_t GetSequenceStepEnable(
                const sequence_step_id_t sequence_step_id_t, 
                uint8_t *pSequenceStepEnabled)
        {
            error_t status = ERROR_NONE;

            status = get_sequence_config_bit(
                    (device_sequence_config_t)sequence_step_id_t,
                    pSequenceStepEnabled);

            return status;
        }

        error_t RdWord(const uint16_t index, uint16_t *pdata)
        {
            static uint16_t r16data = 0;

            error_t status = read_bytes(_device, _i2c_address, index, 2, 
                    (uint8_t *)&r16data);

            *pdata = r16data;

            return status;
        }

        error_t i2c_encode_system_control(
                const uint16_t buf_size, 
                uint8_t *buffer)
        {
            if (SYSTEM_CONTROL_I2C_SIZE_BYTES > buf_size) {
                return ERROR_COMMS_BUFFER_TOO_SMALL;
            }

            buffer[0] = _sys_ctrl.power_management_go1_power_force & 0x1;
            buffer[1] = _sys_ctrl.system_stream_count_ctrl & 0x1;
            buffer[2] = _sys_ctrl.firmware_enable & 0x1;
            buffer[3] = _sys_ctrl.system_interrupt_clear & 0x3;
            buffer[4] = _sys_ctrl.system_mode_start;

            return ERROR_NONE;
        }

        error_t set_limit_value(
                const uint16_t LimitCheckId, 
                const fixed_point_1616_t value)
        {
            error_t status = ERROR_NONE;

            uint16_t tmpuint16 = 0;

            switch (LimitCheckId) {
                case CHECKENABLE_SIGMA_FINAL_RANGE:
                    tmpuint16 = (uint16_t)((value>>14)&0xFFFF);
                    _tim_cfg.range_config_sigma_thresh = tmpuint16;
                    break;
                case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
                    tmpuint16 = (uint16_t)((value>>9)&0xFFFF);
                    _tim_cfg.range_config_min_count_rate_rtn_limit_mcps = 
                        tmpuint16;
                    break;
                default:
                    status = ERROR_INVALID_PARAMS;
            }

            return status;
        }

        error_t set_limit_check_value(
                const uint16_t LimitCheckId, 
                const fixed_point_1616_t LimitCheckValue)
        {
            error_t status = ERROR_NONE;
            uint8_t limit_checks_enable;

            if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
                status = ERROR_INVALID_PARAMS;
            } 
            else {

                limit_checks_enable = _current_parameters.limit_checks_enable[LimitCheckId];

                if (limit_checks_enable == 0) {

                    _current_parameters.limit_checks_value[LimitCheckId] = 
                        LimitCheckValue;

                } else {

                    status = set_limit_value(LimitCheckId, LimitCheckValue);

                    if (status == ERROR_NONE) {

                        _current_parameters.limit_checks_value[LimitCheckId] = 
                            LimitCheckValue; 
                    }
                }
            }

            return status;
        }

        error_t set_limit_check_enable(
                const uint16_t LimitCheckId,
                const uint8_t LimitCheckEnable)
        {
            error_t status = ERROR_NONE;
            fixed_point_1616_t TempFix1616 = 0;

            if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
                status = ERROR_INVALID_PARAMS;
            } else {

                if (LimitCheckEnable == 0) {
                    TempFix1616 = 0;
                }
                else {
                    TempFix1616 = _current_parameters.limit_checks_value[LimitCheckId];
                }

                status = set_limit_value(LimitCheckId, TempFix1616); }

            if (status == ERROR_NONE) {

                _current_parameters.limit_checks_enable[LimitCheckId] = 
                    LimitCheckEnable == 0 ? 0 : 1;

            }

            return status;
        }

        void low_power_auto_data_init(void)
        {
            _low_power_auto_data.vhv_loop_bound =
                TUNINGPARAM_LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT;
            _low_power_auto_data.is_low_power_auto_mode = 0;
            _low_power_auto_data.low_power_auto_range_count = 0;
            _low_power_auto_data.saved_interrupt_config = 0;
            _low_power_auto_data.saved_vhv_init = 0;
            _low_power_auto_data.saved_vhv_timeout = 0;
            _low_power_auto_data.first_run_phasecal_result = 0;
            _low_power_auto_data.dss_total_rate_per_spad_mcps = 0;
            _low_power_auto_data.dss_required_spads = 0;
        }

        void set_vhv_loopbound(const uint8_t vhv_loopbound) 
        {
            _stat_nvm.vhv_config_timeout_macrop_loop_bound =
                (_stat_nvm.vhv_config_timeout_macrop_loop_bound & 0x03) +
                (vhv_loopbound * 4);
        }

        void get_user_zone(user_zone_t * puser_zone)
        {
            decode_row_col(
                    _dyn_cfg.roi_config_user_roi_centre_spad,
                    &(puser_zone->y_centre),
                    &(puser_zone->x_centre));

            decode_zone_size(
                    _dyn_cfg.roi_config_user_roi_requested_global_xy_size,
                    &(puser_zone->width),
                    &(puser_zone->height));
        }

        void set_user_zone(const user_zone_t * puser_zone)
        {
            _dyn_cfg.roi_config_user_roi_centre_spad =
                encode_row_col( puser_zone->y_centre, puser_zone->x_centre);

            _dyn_cfg.roi_config_user_roi_requested_global_xy_size =
                encode_zone_size( puser_zone->width, puser_zone->height);
        }

        void get_mode_mitigation_roi(user_zone_t * pmm_roi)
        {
            uint8_t  x       = 0;
            uint8_t  y       = 0;
            uint8_t  xy_size = 0;

            decode_row_col( _nvm_copy_data.roi_config_mode_roi_centre_spad, &y,
                    &x);

            pmm_roi->x_centre = x;
            pmm_roi->y_centre = y;

            xy_size = _nvm_copy_data.roi_config_mode_roi_xy_size;

            pmm_roi->height = xy_size >> 4;
            pmm_roi->width  = xy_size & 0x0F;
        }

        error_t stop_ranging(void)
        {
            return 
                write_byte(RGSTR_FIRMWARE_ENABLE, 0x01) |
                write_byte(RGSTR_SYSTEM_INTERRUPT_CLEAR, 0x03) |
                write_byte(RGSTR_SYSTEM_MODE_START, 0x00); 
        }

        error_t start_ranging()
        {
            error_t status = ERROR_NONE;

            uint8_t buffer[MAX_I2C_XFER_SIZE] = {};

            _sys_ctrl.system_mode_start =
                (_sys_ctrl.system_mode_start &
                 DEVICEMEASUREMENTMODE_STOP_MASK) |
                _measurement_mode;

            auto i2c_buffer_size_bytes = (
                    RGSTR_POWER_MANAGEMENT_GO1_POWER_FORCE +
                    SYSTEM_CONTROL_I2C_SIZE_BYTES) - 
                RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX;

            uint8_t i2c_buffer_offset_bytes = 0;

            status |= i2c_encode_static_nvm_managed(STATIC_NVM_MANAGED_I2C_SIZE_BYTES,
                    &buffer[i2c_buffer_offset_bytes]);

            i2c_buffer_offset_bytes = RGSTR_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 -
                RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX;

            status |= i2c_encode_customer_nvm_managed(
                    CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES,
                    &buffer[i2c_buffer_offset_bytes]);

            i2c_buffer_offset_bytes = 
                RGSTR_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS - 
                RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX;

            status |= i2c_encode_static_config(
                    STATIC_CONFIG_I2C_SIZE_BYTES,
                    &buffer[i2c_buffer_offset_bytes]);

            i2c_buffer_offset_bytes =
                RGSTR_GPH_CONFIG_STREAM_COUNT_UPDATE_VALUE - 
                RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX;

            status |= i2c_encode_general_config(
                    GENERAL_CONFIG_I2C_SIZE_BYTES,
                    &buffer[i2c_buffer_offset_bytes]);

            i2c_buffer_offset_bytes = 
                RGSTR_MM_CONFIG_TIMEOUT_MACROP_A_HI - 
                RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX;

            status |= i2c_encode_timing_config(
                    TIMING_CONFIG_I2C_SIZE_BYTES,
                    &buffer[i2c_buffer_offset_bytes]);

            i2c_buffer_offset_bytes = 
                RGSTR_SYSTEM_GROUPED_PARAMETER_HOLD_0 - 
                RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX;

            status |= i2c_encode_dynamic_config(
                    DYNAMIC_CONFIG_I2C_SIZE_BYTES,
                    &buffer[i2c_buffer_offset_bytes]);

            i2c_buffer_offset_bytes =
                RGSTR_POWER_MANAGEMENT_GO1_POWER_FORCE - 
                RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX;

            status |= i2c_encode_system_control(SYSTEM_CONTROL_I2C_SIZE_BYTES,
                    &buffer[i2c_buffer_offset_bytes]);

            status |= write_bytes(_device, _i2c_address,
                    RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX, 
                    i2c_buffer_size_bytes, buffer);

            return status;
        }

        error_t get_distance(uint16_t * distance)
        {
            return read_word(RGSTR_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, 
                    distance);
        }

        error_t set_measurement_timing_budget_usec(const uint32_t usec)
        {
            if (usec > 10000000) {
                return ERROR_INVALID_PARAMS;
            }

            uint8_t Mm1Enabled = 0;
            auto status = GetSequenceStepEnable(SEQUENCESTEP_MM1, &Mm1Enabled);

            uint8_t Mm2Enabled = 0;
            if (status == ERROR_NONE) {
                status = GetSequenceStepEnable(SEQUENCESTEP_MM2, &Mm2Enabled);
            }

            uint32_t TimingBudget = 0;
            uint32_t MmTimeoutUs = 0;
            uint32_t PhaseCalTimeoutUs = 0;
            if (status == ERROR_NONE)
                status = get_timeouts_us( &PhaseCalTimeoutUs, &MmTimeoutUs,
                        &TimingBudget);

            uint32_t TimingGuard = 0;

            if (status == ERROR_NONE) {

                auto presetMode = _current_parameters.presetMode;

                auto FDAMaxTimingBudgetUs = FDA_MAX_TIMING_BUDGET_US;

                TimingGuard = 0;
                uint32_t divisor = 1;
                switch (presetMode) {
                    case PRESETMODE_LITE_RANGING:
                        if ((Mm1Enabled == 1) || (Mm2Enabled == 1))
                            TimingGuard = 5000;
                        else
                            TimingGuard = 1000;
                        break;

                    case PRESETMODE_AUTONOMOUS:
                        FDAMaxTimingBudgetUs *= 2;
                        if ((Mm1Enabled == 1) || (Mm2Enabled == 1))
                            TimingGuard = 26600;
                        else
                            TimingGuard = 21600;
                        divisor = 2;
                        break;

                    case PRESETMODE_LOWPOWER_AUTONOMOUS:
                        {
                            FDAMaxTimingBudgetUs *= 2;
                            auto vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US;
                            auto vhv_loops = _low_power_auto_data.vhv_loop_bound;
                            if (vhv_loops > 0) {
                                vhv += vhv_loops *
                                    LOWPOWER_AUTO_VHV_LOOP_DURATION_US;
                            }
                            TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
                                LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING + vhv;
                            divisor = 2;
                        }
                        break;

                    default:

                        status = ERROR_MODE_NOT_SUPPORTED;
                }

                if (usec <= TimingGuard)
                    status = ERROR_INVALID_PARAMS;
                else {
                    TimingBudget = (usec - TimingGuard);
                }

                if (status == ERROR_NONE) {
                    if (TimingBudget > FDAMaxTimingBudgetUs)
                        status = ERROR_INVALID_PARAMS;
                    else {
                        TimingBudget /= divisor;
                        status = set_timeouts_us(
                                PhaseCalTimeoutUs,
                                MmTimeoutUs,
                                TimingBudget);
                    }

                    if (status == ERROR_NONE) {
                        _range_config_timeout_us = TimingBudget;
                    }
                }
            }
            if (status == ERROR_NONE) {
                _current_parameters.measurement_timing_budget_usec = usec;
            }

            return status;
        }

        error_t set_distance_mode(const distanceMode_t distance_mode)
        {
            auto presetMode = _current_parameters.presetMode;

            if ((distance_mode != DISTANCEMODE_SHORT) &&
                    (distance_mode != DISTANCEMODE_MEDIUM) &&
                    (distance_mode != DISTANCEMODE_LONG))
                return ERROR_INVALID_PARAMS;

            auto internal_distance_mode = 
                (distance_mode == DISTANCEMODE_SHORT ||
                 distance_mode == DISTANCEMODE_MEDIUM) ?
                distance_mode :
                DISTANCEMODE_LONG;

            user_zone_t user_zone = {};
            get_user_zone(&user_zone);

            auto inter_measurement_period_ms = _inter_measurement_period_ms;

            uint32_t TimingBudget = 0;
            uint32_t MmTimeoutUs = 0;
            uint32_t PhaseCalTimeoutUs = 0;
            auto status = get_timeouts_us(&PhaseCalTimeoutUs, &MmTimeoutUs, 
                    &TimingBudget);

            if (status == ERROR_NONE) {
                status = helper_set_preset_mode(presetMode,
                        internal_distance_mode, inter_measurement_period_ms);
            }

            if (status == ERROR_NONE) {
                _current_parameters.internal_distance_mode = internal_distance_mode;
                _current_parameters.new_distance_mode = internal_distance_mode;
                _current_parameters.distance_mode = distance_mode;
            }

            if (status == ERROR_NONE) {
                status = set_timeouts_us(PhaseCalTimeoutUs,
                        MmTimeoutUs, TimingBudget);

                if (status == ERROR_NONE)
                    _range_config_timeout_us = TimingBudget;
            }

            if (status == ERROR_NONE) {
                set_user_zone(&user_zone);
            }

            return status;
        }

        error_t check_for_data_ready(bool *isDataReady)
        {
            uint8_t tmp = 0;
            auto status = read_byte(RGSTR_GPIO_HV_MUX_CTRL, &tmp);

            tmp = tmp & 0x10;
            auto interruptPolarity = !(tmp >> 4);

            uint8_t hvstatus = 0;
            status |= read_byte(RGSTR_GPIO_TIO_HV_STATUS, &hvstatus);

            if (status == ERROR_NONE) {
                *isDataReady = (hvstatus & 1) == interruptPolarity;
            }

            return status;
        }

}; 
