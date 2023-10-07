#pragma once

/*
   Copyright (c) 2017, STMicroelectronics
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

#include <stdint.h>

class VL53L1X_Abstract {

    public:

        typedef int8_t error_t;

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
            ERROR_OFFSET_CAL_NO_SAMPLE_FAIL           = -24,
            ERROR_OFFSET_CAL_NO_SPADS_ENABLED_FAIL    = -25,
            ERROR_ZONE_CAL_NO_SAMPLE_FAIL             = -26,
            ERROR_TUNING_PARM_KEY_MISMATCH             = -27,
            WARNING_REF_SPAD_CHAR_NOT_ENOUGH_SPADS   = -28,
            WARNING_REF_SPAD_CHAR_RATE_TOO_HIGH      = -29,
            WARNING_REF_SPAD_CHAR_RATE_TOO_LOW       = -30,
            WARNING_OFFSET_CAL_MISSING_SAMPLES       = -31,
            WARNING_OFFSET_CAL_SIGMA_TOO_HIGH        = -32,
            WARNING_OFFSET_CAL_RATE_TOO_HIGH         = -33,
            WARNING_OFFSET_CAL_SPAD_COUNT_TOO_LOW    = -34,
            WARNING_ZONE_CAL_MISSING_SAMPLES       = -35,
            WARNING_ZONE_CAL_SIGMA_TOO_HIGH        = -36,
            WARNING_ZONE_CAL_RATE_TOO_HIGH         = -37,
            WARNING_XTALK_MISSING_SAMPLES             = -38,
            WARNING_XTALK_NO_SAMPLES_FOR_GRADIENT     = -39,
            WARNING_XTALK_SIGMA_LIMIT_FOR_GRADIENT    = -40,
            ERROR_NOT_IMPLEMENTED                   = -41,
            ERROR_PLATFORM_SPECIFIC_START           = -60,
            ERROR_DEVICE_FIRMWARE_TOO_OLD           = -80,
            ERROR_DEVICE_FIRMWARE_TOO_NEW           = -85,
            ERROR_UNIT_TEST_FAIL                    = -90,
            ERROR_FILE_READ_FAIL                    = -95,
            ERROR_FILE_WRITE_FAIL                   = -96,
        };

        enum {
            DISTANCEMODE_SHORT = 1,
            DISTANCEMODE_MEDIUM,
            DISTANCEMODE_LONG, 
        };

        typedef uint8_t DistanceModes;

        error_t init(const uint8_t addr=0x29)
        {
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
                TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT;

            if (status == ERROR_NONE)
                status = get_static_nvm_managed(&(_stat_nvm)); 

            if (status == ERROR_NONE)
                status = get_customer_nvm_managed(&(_customer));

            if (status == ERROR_NONE) {
                status = get_nvm_copy_data(&(_nvm_copy_data));

                if (status == ERROR_NONE)
                    copy_rtn_good_spads_to_buffer( &(_nvm_copy_data),
                            &(_rtn_good_spads[0])); }

                if (status == ERROR_NONE)
                    status = RdWord(RGSTR_RESULT_OSC_CALIBRATE_VAL,
                            &(_dbg_results.result_osc_calibrate_val));

                if (_stat_nvm.osc_measured_fast_osc_frequency < 0x1000) {
                    _stat_nvm.osc_measured_fast_osc_frequency = 0xBCCC;
                }

                if (status == ERROR_NONE)
                    status = get_mode_mitigation_roi(&(_mm_roi));

                if (_optical_centre.x_centre == 0 && _optical_centre.y_centre == 0) {
                    _optical_centre.x_centre = _mm_roi.x_centre << 4;
                    _optical_centre.y_centre = _mm_roi.y_centre << 4;
                }

                status = init_refspadchar_config_struct( &(_refspadchar));

                status = init_ssc_config_struct( &(_ssc_cfg));

                status = init_xtalk_config_struct( &(_customer), &(_xtalk_cfg));

                status = init_offset_cal_config_struct( &(_offsetcal_cfg));

                status = init_tuning_parm_storage_struct(&_tuning_parms);

                status = set_vhv_loopbound(TUNINGPARM_VHV_LOOPBOUND_DEFAULT);

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

                    CurrentParameters.PresetMode = PRESETMODE_LOWPOWER_AUTONOMOUS;
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
                            (FixedPoint1616_t)(18 * 65536));
                }

                if (status == ERROR_NONE) {
                    status = set_limit_check_value(
                            CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                            (FixedPoint1616_t)(25 * 65536 / 100));
                }

                uint8_t measurement_mode  = DEVICEMEASUREMENTMODE_BACKTOBACK;

                _measurement_mode = measurement_mode;

                CurrentParameters.NewDistanceMode = DISTANCEMODE_LONG;

                CurrentParameters.InternalDistanceMode = DISTANCEMODE_LONG;

                CurrentParameters.DistanceMode = DISTANCEMODE_LONG;

                DistanceModes DistanceMode = DISTANCEMODE_LONG;

                PresetModes PresetMode = PRESETMODE_LOWPOWER_AUTONOMOUS;

                status = helper_set_preset_mode(PresetMode, DistanceMode, 1000);

                if (status == ERROR_NONE) {

                    CurrentParameters.InternalDistanceMode = DistanceMode;

                    CurrentParameters.NewDistanceMode = DistanceMode;

                    if ((PresetMode == PRESETMODE_LITE_RANGING) ||
                            (PresetMode == PRESETMODE_AUTONOMOUS) ||
                            (PresetMode == PRESETMODE_LOWPOWER_AUTONOMOUS))
                        status = SetMeasurementTimingBudgetMicroSeconds(41000);
                    else
                        status = SetMeasurementTimingBudgetMicroSeconds(33333);
                }

                if (status == ERROR_NONE) {
                    status = SetInterMeasurementPeriodMilliSeconds(1000);
                }

                for (uint16_t rgstr = 0x002D; rgstr <= 0x0087; rgstr++) {
                    status |= write_byte(rgstr, DEFAULT_CONFIGURATION[rgstr - 0x002D]);
                }

                status |= write_byte(RGSTR_SYSTEM_INTERRUPT_CLEAR, 0x01); 

                status |= write_byte(RGSTR_SYSTEM_MODE_START, 0x40); 

                status |= write_byte(RGSTR_SYSTEM_INTERRUPT_CLEAR, 0x01);

                status |= stopRanging();

                status |= write_byte(RGSTR_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09); 

                status |= write_byte(0x0B, 0);											

                return status; 

        } // init

        error_t stopRanging(void)
        {
            return 
                write_byte(RGSTR_FIRMWARE_ENABLE, 0x01) |
                write_byte(RGSTR_SYSTEM_INTERRUPT_CLEAR, 0x03) |
                write_byte(RGSTR_SYSTEM_MODE_START, 0x00); 
        }

        error_t startRanging()
        {
            error_t status = ERROR_NONE;

            uint8_t buffer[MAX_I2C_XFER_SIZE] = {};

            _sys_ctrl.system_mode_start =
                (_sys_ctrl.system_mode_start &
                 DEVICEMEASUREMENTMODE_STOP_MASK) |
                _measurement_mode;

            auto i2c_buffer_size_bytes = (RGSTR_POWER_MANAGEMENT_GO1_POWER_FORCE +
                    SYSTEM_CONTROL_I2C_SIZE_BYTES) - RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX;

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

            status |= write_bytes(RGSTR_ANA_CONFIG_VHV_REF_SEL_VDDPIX, 
                    i2c_buffer_size_bytes, buffer);

            return status;
        }

        error_t getDistance(uint16_t * distance)
        {
            return read_word(RGSTR_RESULT_FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, 
                    distance);
        }

        error_t SetMeasurementTimingBudgetMicroSeconds(
                uint32_t MeasurementTimingBudgetMicroSeconds)
        {
            error_t Status = ERROR_NONE;
            uint8_t Mm1Enabled;
            uint8_t Mm2Enabled;
            uint32_t TimingGuard;
            uint32_t divisor;
            uint32_t TimingBudget;
            uint32_t MmTimeoutUs;
            PresetModes PresetMode;
            uint32_t PhaseCalTimeoutUs;
            uint32_t vhv;
            int32_t vhv_loops;
            uint32_t FDAMaxTimingBudgetUs = FDA_MAX_TIMING_BUDGET_US;

            /* Timing budget is limited to 10 seconds */
            if (MeasurementTimingBudgetMicroSeconds > 10000000)
                Status = ERROR_INVALID_PARAMS;

            if (Status == ERROR_NONE) {
                Status = GetSequenceStepEnable(
                        SEQUENCESTEP_MM1, &Mm1Enabled);
            }

            if (Status == ERROR_NONE) {
                Status = GetSequenceStepEnable(
                        SEQUENCESTEP_MM2, &Mm2Enabled);
            }

            if (Status == ERROR_NONE)
                Status = get_timeouts_us(
                        &PhaseCalTimeoutUs,
                        &MmTimeoutUs,
                        &TimingBudget);

            if (Status == ERROR_NONE) {

                PresetMode = CurrentParameters.PresetMode;

                TimingGuard = 0;
                divisor = 1;
                switch (PresetMode) {
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
                        FDAMaxTimingBudgetUs *= 2;
                        vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US;
                        vhv_loops = _low_power_auto_data.vhv_loop_bound;
                        if (vhv_loops > 0) {
                            vhv += vhv_loops *
                                LOWPOWER_AUTO_VHV_LOOP_DURATION_US;
                        }
                        TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
                            LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING +
                            vhv;
                        divisor = 2;
                        break;

                    default:
                        /* Unsupported mode */
                        Status = ERROR_MODE_NOT_SUPPORTED;
                }

                if (MeasurementTimingBudgetMicroSeconds <= TimingGuard)
                    Status = ERROR_INVALID_PARAMS;
                else {
                    TimingBudget = (MeasurementTimingBudgetMicroSeconds
                            - TimingGuard);
                }

                if (Status == ERROR_NONE) {
                    if (TimingBudget > FDAMaxTimingBudgetUs)
                        Status = ERROR_INVALID_PARAMS;
                    else {
                        TimingBudget /= divisor;
                        Status = set_timeouts_us(
                                PhaseCalTimeoutUs,
                                MmTimeoutUs,
                                TimingBudget);
                    }

                    if (Status == ERROR_NONE) {
                        _range_config_timeout_us = TimingBudget;
                    }
                }
            }
            if (Status == ERROR_NONE) {
                CurrentParameters.MeasurementTimingBudgetMicroSeconds =
                    MeasurementTimingBudgetMicroSeconds;
            }

            return Status;
        }

        error_t SetDistanceMode(DistanceModes DistanceMode)
        {
            error_t Status = ERROR_NONE;
            PresetModes PresetMode;
            DistanceModes InternalDistanceMode;
            uint32_t inter_measurement_period_ms;
            uint32_t TimingBudget;
            uint32_t MmTimeoutUs;
            uint32_t PhaseCalTimeoutUs;
            user_zone_t user_zone;

            PresetMode = CurrentParameters.PresetMode;

            if ((DistanceMode != DISTANCEMODE_SHORT) &&
                    (DistanceMode != DISTANCEMODE_MEDIUM) &&
                    (DistanceMode != DISTANCEMODE_LONG))
                return ERROR_INVALID_PARAMS;

            if (Status == ERROR_NONE) {
                if ((DistanceMode == DISTANCEMODE_SHORT) ||
                        (DistanceMode == DISTANCEMODE_MEDIUM))
                    InternalDistanceMode = DistanceMode;
                else /* (DistanceMode == DISTANCEMODE_LONG) */
                    InternalDistanceMode = DISTANCEMODE_LONG;
            }

            if (Status == ERROR_NONE) {
                Status = get_user_zone(&user_zone);
            }

            inter_measurement_period_ms =  _inter_measurement_period_ms;

            if (Status == ERROR_NONE)
                Status = get_timeouts_us(&PhaseCalTimeoutUs,
                        &MmTimeoutUs, &TimingBudget);

            if (Status == ERROR_NONE) {
                Status = helper_set_preset_mode(
                        PresetMode,
                        InternalDistanceMode,
                        inter_measurement_period_ms);
            }

            if (Status == ERROR_NONE) {
                CurrentParameters.InternalDistanceMode = InternalDistanceMode;
                CurrentParameters.NewDistanceMode = InternalDistanceMode;
                CurrentParameters.DistanceMode = DistanceMode;
            }

            if (Status == ERROR_NONE) {
                Status = set_timeouts_us(PhaseCalTimeoutUs,
                        MmTimeoutUs, TimingBudget);

                if (Status == ERROR_NONE)
                    _range_config_timeout_us = TimingBudget;
            }

            if (Status == ERROR_NONE)
                Status = set_user_zone(&user_zone);

            return Status;
        }

        error_t checkForDataReady(bool *isDataReady)
        {
            uint8_t tmp = 0;
            auto status = read_byte(RGSTR_GPIO_HV_MUX_CTRL, &tmp);

            tmp = tmp & 0x10;
            auto interruptPolarity = !(tmp >> 4);

            uint8_t hvStatus = 0;
            status |= read_byte(RGSTR_GPIO_TIO_HV_STATUS, &hvStatus);

            if (status == ERROR_NONE) {
                *isDataReady = (hvStatus & 1) == interruptPolarity;
            }

            return status;
        }

    protected:

        uint8_t _i2c_address;

    private:

        enum {

            CHECKENABLE_SIGMA_FINAL_RANGE,
            CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
        };

        enum {
            SEQUENCESTEP_MM1  = 5,
            SEQUENCESTEP_MM2  = 6,
        };

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
        static const uint16_t TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS  = 0x8000;
        static const uint16_t LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING = 2100;
        static const uint32_t FDA_MAX_TIMING_BUDGET_US = 550000;
        static const uint16_t LOWPOWER_AUTO_VHV_LOOP_DURATION_US = 245;
        static const uint16_t LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING = 1448;
        static const uint16_t MAX_I2C_XFER_SIZE = 256;

        static const uint16_t MACRO_PERIOD_VCSEL_PERIODS = (
                AMBIENT_WINDOW_VCSEL_PERIODS + RANGING_WINDOW_VCSEL_PERIODS);

        static const uint16_t TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US = TUNINGPARM_PUBLIC_PAGE_BASE_ADDRESS + 48;

        static const uint8_t DEVICEMEASUREMENTMODE_STOP_MASK   = 0x0F;
        static const uint8_t GROUPEDPARAMETERHOLD_ID_MASK      = 0x02;
        static const uint8_t INTERRUPT_CONFIG_NEW_SAMPLE_READY = 0x20;
        static const uint8_t CLEAR_RANGE_INT                   = 0x01;

        static const uint16_t TUNINGPARM_VERSION_DEFAULT =  32771;
        static const uint16_t TUNINGPARM_KEY_TABLE_VERSION_DEFAULT =  32769;
        static const uint16_t TUNINGPARM_LLD_VERSION_DEFAULT =  32833;
        static const uint16_t TUNINGPARM_LITE_CAL_REPEAT_RATE_DEFAULT =  0;
        static const uint16_t TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT =  2011;
        static const uint16_t TUNINGPARM_LITE_MIN_CLIP_MM_DEFAULT =  0;
        static const uint16_t TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM_DEFAULT =  60;
        static const uint16_t TUNINGPARM_LITE_MED_SIGMA_THRESH_MM_DEFAULT =  60;
        static const uint16_t TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM_DEFAULT =  360;
        static const uint16_t TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS_DEFAULT =  128;
        static const uint16_t TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS_DEFAULT =  128;
        static const uint16_t TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS_DEFAULT =  192;
        static const uint16_t TUNINGPARM_LITE_XTALK_MARGIN_KCPS_DEFAULT = 0;
        static const uint16_t TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS_DEFAULT =  2560;
        static const uint16_t TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS_DEFAULT =  1280;
        static const uint16_t TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS_DEFAULT =  5120;
        static const uint16_t TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS_DEFAULT =  2560;
        static const uint16_t TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS_DEFAULT =  12;
        static const uint16_t TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT =  2560;
        static const uint16_t TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT =  2560;

        static const uint8_t TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE_DEFAULT =  2;
        static const uint8_t TUNINGPARM_PHASECAL_TARGET_DEFAULT =  33;
        static const uint8_t TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH_DEFAULT =  8;
        static const uint8_t TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS_DEFAULT =  16;
        static const uint8_t TUNINGPARM_LITE_SIGMA_REF_MM_DEFAULT =  1;
        static const uint8_t TUNINGPARM_LITE_RIT_MULT_DEFAULT =  64;
        static const uint8_t TUNINGPARM_LITE_SEED_CONFIG_DEFAULT =  2;
        static const uint8_t TUNINGPARM_LITE_QUANTIFIER_DEFAULT =  2;
        static const uint8_t TUNINGPARM_LITE_FIRST_ORDER_SELECT_DEFAULT =  0;
        static const uint8_t TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE_DEFAULT =  14;
        static const uint8_t TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE_DEFAULT =  10;
        static const uint8_t TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE_DEFAULT =  6;
        static const uint8_t TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE_DEFAULT =  14;
        static const uint8_t TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE_DEFAULT =  10;
        static const uint8_t TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE_DEFAULT =  6;
        static const uint8_t TUNINGPARM_TIMED_SEED_CONFIG_DEFAULT =  1;
        static const uint8_t TUNINGPARM_VHV_LOOPBOUND_DEFAULT =  32;
        static const uint8_t TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE_DEFAULT =  8;
        static const uint8_t TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD_DEFAULT =  11;
        static const uint8_t TUNINGPARM_OFFSET_CAL_PRE_SAMPLES_DEFAULT =  8;
        static const uint8_t TUNINGPARM_OFFSET_CAL_MM1_SAMPLES_DEFAULT =  40;
        static const uint8_t TUNINGPARM_OFFSET_CAL_MM2_SAMPLES_DEFAULT =  9;
        static const uint8_t TUNINGPARM_SPADMAP_VCSEL_PERIOD_DEFAULT =  18;
        static const uint8_t TUNINGPARM_SPADMAP_VCSEL_START_DEFAULT =  15;
        static const uint8_t TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT =  3;

        static const uint32_t TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US_DEFAULT =  1000;
        static const uint32_t TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US_DEFAULT =  1000;
        static const uint32_t TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US_DEFAULT =  13000;
        static const uint32_t TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US_DEFAULT =  13000;
        static const uint32_t TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT =  1000;
        static const uint32_t TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US_DEFAULT =  2000;
        static const uint32_t TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US_DEFAULT =  2000;
        static const uint32_t TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US_DEFAULT =  63000;
        static const uint32_t TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US_DEFAULT =  13000;
        static const uint32_t TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT =  1;
        static const uint32_t TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US_DEFAULT =  8000;

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

        typedef uint32_t FixedPoint1616_t;

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

        typedef uint8_t WaitMethod;
        typedef uint8_t DeviceState;
        typedef uint8_t DevicePresetModes;
        typedef uint8_t DeviceMeasurementModes;
        typedef uint8_t OffsetCalibrationMode;
        typedef uint8_t OffsetCorrectionMode;
        typedef uint8_t DeviceSequenceConfig;
        typedef uint8_t DeviceInterruptPolarity;
        typedef uint8_t DeviceGpioMode;
        typedef uint8_t DeviceError;
        typedef uint8_t DeviceReportStatus;
        typedef uint8_t DeviceDssMode;
        typedef uint8_t DeviceConfigLevel;
        typedef uint8_t DeviceResultsLevel;
        typedef uint8_t DeviceTestMode;
        typedef uint8_t DeviceSscArray;
        typedef uint8_t GPIO_Interrupt_Mode;
        typedef uint16_t TuningParms;

        typedef struct {

            uint8_t    device_test_mode;     
            uint8_t    vcsel_period;         
            uint32_t   timeout_us;           
            uint16_t   target_count_rate_mcps;
            uint16_t   min_count_rate_limit_mcps;
            uint16_t   max_count_rate_limit_mcps;

        } refspadchar_config_t;

        typedef struct {

            DeviceSscArray  array_select;
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

            DeviceState     cfg_device_state;
            DeviceState     rd_device_state;
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

            DeviceState   cfg_device_state;
            uint8_t   cfg_stream_count;
            uint8_t   cfg_gph_id;
            uint8_t   cfg_timing_status;
            DeviceState   rd_device_state;
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

        static const uint8_t RTN_SPAD_BUFFER_SIZE = 32;

        uint8_t   _wait_method;
        DevicePresetModes        _preset_mode;
        DeviceMeasurementModes   _measurement_mode;
        OffsetCalibrationMode    _offset_calibration_mode;
        OffsetCorrectionMode     _offset_correction_mode;
        uint32_t  _phasecal_config_timeout_us;
        uint32_t  _mm_config_timeout_us;
        uint32_t  _range_config_timeout_us;
        uint32_t  _inter_measurement_period_ms;
        uint16_t  _dss_config_target_total_rate_mcps;
        uint32_t  _fw_ready_poll_duration_ms;
        uint8_t   _fw_ready;
        uint8_t   _debug_mode;
        ll_driver_state_t            _ll_state;
        customer_nvm_managed_t       _customer;
        cal_peak_rate_map_t          _cal_peak_rate_map;
        additional_offset_cal_data_t _add_off_cal_data;
        gain_calibration_data_t      _gain_cal;
        user_zone_t                  _mm_roi;
        optical_centre_t             _optical_centre;
        tuning_parm_storage_t        _tuning_parms;
        uint8_t _rtn_good_spads[RTN_SPAD_BUFFER_SIZE];
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
        low_power_auto_data_t		_low_power_auto_data;

        typedef uint8_t PresetModes;
        typedef uint8_t XtalkCalibrationModes;
        typedef uint8_t OffsetCalibrationModes;
        typedef uint8_t ThresholdMode;

        static const uint8_t CHECKENABLE_NUMBER_OF_CHECKS = 2;

        typedef struct {

            PresetModes PresetMode;
            DistanceModes DistanceMode;
            DistanceModes InternalDistanceMode;
            DistanceModes NewDistanceMode;
            uint32_t MeasurementTimingBudgetMicroSeconds;
            uint8_t LimitChecksEnable[CHECKENABLE_NUMBER_OF_CHECKS];
            uint8_t LimitChecksStatus[CHECKENABLE_NUMBER_OF_CHECKS];
            FixedPoint1616_t LimitChecksValue[CHECKENABLE_NUMBER_OF_CHECKS];
            FixedPoint1616_t LimitChecksCurrent[CHECKENABLE_NUMBER_OF_CHECKS];

        } DeviceParameters_t;

        typedef uint8_t State;

        DeviceParameters_t CurrentParameters;

        error_t read_word(const uint16_t rgstr, uint16_t *data)
        {
            uint8_t buffer[2] = {};

            auto status = read_bytes(rgstr, 2, buffer);

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
            return write_bytes(rgstr, 2, (uint8_t *)buffer);
        }

        error_t read_byte(const uint16_t rgstr, uint8_t *data)
        {
            return read_bytes(rgstr, 1, data);
        }

        error_t write_byte(const uint16_t rgstr, const uint8_t data)
        {
            return write_bytes(rgstr, 1, &data);
        }

        error_t write_bytes(const uint16_t rgstr, const uint8_t count, 
                const uint8_t *data);

        error_t read_bytes(const uint16_t rgstr, const uint8_t count, 
                uint8_t *data);

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

        typedef uint8_t SequenceStepId;

        error_t update_ll_driver_cfg_state(void);

        static uint16_t FIXEDPOINT1616TOFIXEDPOINT142(FixedPoint1616_t Value) 
        { 
            return (uint16_t)((Value>>14)&0xFFFF);
        }

        static uint16_t FIXEDPOINT1616TOFIXEDPOINT97(FixedPoint1616_t Value) 
        {
            return (uint16_t)((Value>>9)&0xFFFF);
        }

        static void i2c_encode_uint16_t(
                uint16_t    ip_value,
                uint16_t    count,
                uint8_t    *pbuffer)
        {
            uint16_t   i    = 0;
            uint16_t   data = 0;

            data =  ip_value;

            for (i = 0; i < count ; i++) {
                pbuffer[count-i-1] = (uint8_t)(data & 0x00FF);
                data = data >> 8;
            }
        }

        static uint16_t i2c_decode_uint16_t(
                uint16_t    count,
                uint8_t    *pbuffer)
        {
            uint16_t   value = 0x00;

            while (count-- > 0) {
                value = (value << 8) | (uint16_t)*pbuffer++;
            }

            return value;
        }

        static void i2c_encode_int16_t(
                int16_t     ip_value,
                uint16_t    count,
                uint8_t    *pbuffer)
        {
            /*
             * Encodes a int16_t register value into an I2C write buffer
             * MS byte first order (as per I2C register map.
             */

            uint16_t   i    = 0;
            int16_t    data = 0;

            data =  ip_value;

            for (i = 0; i < count ; i++) {
                pbuffer[count-i-1] = (uint8_t)(data & 0x00FF);
                data = data >> 8;
            }
        }

        static int16_t i2c_decode_int16_t(
                uint16_t    count,
                uint8_t    *pbuffer)
        {
            /*
             * Decodes a int16_t from the input I2C read buffer
             * (MS byte first order)
             */

            int16_t    value = 0x00;

            /* implement sign extension */
            if (*pbuffer >= 0x80) {
                value = 0xFFFF;
            }

            while (count-- > 0) {
                value = (value << 8) | (int16_t)*pbuffer++;
            }

            return value;
        }

        static void i2c_encode_uint32_t(
                uint32_t    ip_value,
                uint16_t    count,
                uint8_t    *pbuffer)
        {
            /*
             * Encodes a uint32_t register value into an I2C write buffer
             * MS byte first order (as per I2C register map.
             */

            uint16_t   i    = 0;
            uint32_t   data = 0;

            data =  ip_value;

            for (i = 0; i < count ; i++) {
                pbuffer[count-i-1] = (uint8_t)(data & 0x00FF);
                data = data >> 8;
            }
        }

        error_t i2c_decode_static_nvm_managed(
                uint16_t                   buf_size,
                uint8_t                   *pbuffer,
                static_nvm_managed_t  *pdata)
        {
            /**
             * Decodes data structure static_nvm_managed_t from the input I2C read buffer
             * Buffer must be at least 11 bytes
             */

            error_t status = ERROR_NONE;

            if (STATIC_NVM_MANAGED_I2C_SIZE_BYTES > buf_size)
                return ERROR_COMMS_BUFFER_TOO_SMALL;

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


            return status;
        }

        error_t get_static_nvm_managed(
                static_nvm_managed_t  *pdata)
        {
            /**
             * Reads and de-serialises the contents of static_nvm_managed_t
             * data structure from the device
             */

            error_t status = ERROR_NONE;
            uint8_t comms_buffer[STATIC_NVM_MANAGED_I2C_SIZE_BYTES];

            if (status == ERROR_NONE) /*lint !e774 always true*/
                status = read_bytes(
                        RGSTR_I2C_ADDRESS,
                        STATIC_NVM_MANAGED_I2C_SIZE_BYTES,
                        comms_buffer);

            if (status == ERROR_NONE)
                status = i2c_decode_static_nvm_managed(
                        STATIC_NVM_MANAGED_I2C_SIZE_BYTES,
                        comms_buffer,
                        pdata);

            return status;
        }

        error_t i2c_encode_customer_nvm_managed(
                uint16_t buf_size, uint8_t *pbuffer)
        {
            /**
             * Encodes data structure customer_nvm_managed_t into a I2C write buffer
             * Buffer must be at least 23 bytes
             */

            error_t status = ERROR_NONE;

            if (CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES > buf_size)
                return ERROR_COMMS_BUFFER_TOO_SMALL;

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


            return status;
        }

        error_t i2c_decode_customer_nvm_managed(
                uint16_t                   buf_size,
                uint8_t                   *pbuffer,
                customer_nvm_managed_t  *pdata)
        {
            /**
             * Decodes data structure customer_nvm_managed_t from the input I2C read buffer
             * Buffer must be at least 23 bytes
             */

            error_t status = ERROR_NONE;


            if (CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES > buf_size)
                return ERROR_COMMS_BUFFER_TOO_SMALL;

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

            return status;
        }

        error_t get_customer_nvm_managed(
                customer_nvm_managed_t  *pdata)
        {
            /**
             * Reads and de-serialises the contents of customer_nvm_managed_t
             * data structure from the device
             */

            error_t status = ERROR_NONE;
            uint8_t comms_buffer[CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES];


            if (status == ERROR_NONE) /*lint !e774 always true*/
                status = read_bytes(
                        RGSTR_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                        CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES,
                        comms_buffer);

            if (status == ERROR_NONE)
                status = i2c_decode_customer_nvm_managed(
                        CUSTOMER_NVM_MANAGED_I2C_SIZE_BYTES,
                        comms_buffer,
                        pdata);


            return status;
        }

        error_t i2c_encode_static_config(uint16_t buf_size, uint8_t  *pbuffer)
        {
            /**
             * Encodes data structure static_config_t into a I2C write buffer
             * Buffer must be at least 32 bytes
             */

            error_t status = ERROR_NONE;


            if (STATIC_CONFIG_I2C_SIZE_BYTES > buf_size)
                return ERROR_COMMS_BUFFER_TOO_SMALL;

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

            return status;
        }


        error_t i2c_encode_general_config(uint16_t buf_size, uint8_t  *pbuffer)
        {
            error_t status = ERROR_NONE;


            if (GENERAL_CONFIG_I2C_SIZE_BYTES > buf_size)
                return ERROR_COMMS_BUFFER_TOO_SMALL;

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


            return status;
        }


        error_t i2c_encode_timing_config(uint16_t buf_size, uint8_t  *pbuffer)
        {
            error_t status = ERROR_NONE;

            if (TIMING_CONFIG_I2C_SIZE_BYTES > buf_size)
                return ERROR_COMMS_BUFFER_TOO_SMALL;

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


            return status;
        }

        error_t i2c_encode_dynamic_config(uint16_t buf_size, uint8_t  *pbuffer)
        {
            error_t status = ERROR_NONE;

            if (DYNAMIC_CONFIG_I2C_SIZE_BYTES > buf_size)
                return ERROR_COMMS_BUFFER_TOO_SMALL;

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

            return status;
        }

        error_t i2c_decode_nvm_copy_data(
                uint16_t                   buf_size,
                uint8_t                   *pbuffer,
                nvm_copy_data_t    *pdata)
        {
            /**
             * Decodes data structure nvm_copy_data_t from the input I2C read buffer
             * Buffer must be at least 49 bytes
             */

            error_t status = ERROR_NONE;


            if (NVM_COPY_DATA_I2C_SIZE_BYTES > buf_size)
                return ERROR_COMMS_BUFFER_TOO_SMALL;

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


            return status;
        }

        error_t get_nvm_copy_data(
                nvm_copy_data_t    *pdata)
        {
            /**
             * Reads and de-serialises the contents of nvm_copy_data_t
             * data structure from the device
             */

            error_t status = ERROR_NONE;
            uint8_t comms_buffer[NVM_COPY_DATA_I2C_SIZE_BYTES];


            if (status == ERROR_NONE) /*lint !e774 always true*/
                status = read_bytes(
                        RGSTR_IDENTIFICATION_MODEL_ID,
                        NVM_COPY_DATA_I2C_SIZE_BYTES,
                        comms_buffer);

            if (status == ERROR_NONE)
                status = i2c_decode_nvm_copy_data(
                        NVM_COPY_DATA_I2C_SIZE_BYTES,
                        comms_buffer,
                        pdata);


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

        uint32_t calc_pll_period_us(
                uint16_t  fast_osc_frequency)
        {
            /*  Calculates PLL frequency using NVM fast_osc_frequency
             *  Fast osc frequency fixed point format = unsigned 4.12
             *
             *  PLL period fixed point format = unsigned 0.24
             *  Min input fast osc frequency  = 1 MHz
             *  PLL Multiplier = 64 (fixed)
             *  Min PLL freq = 64.0MHz
             *  -> max PLL period = 1/ 64
             *  ->  only the 18 LS bits are used
             *
             *  2^30 = (2^24) (1.0us) * 4096 (2^12) / 64 (PLL Multiplier)
             */

            uint32_t  pll_period_us        = 0;


            pll_period_us = (0x01 << 30) / fast_osc_frequency;

            return pll_period_us;
        }

        uint8_t decode_vcsel_period(uint8_t vcsel_period_reg)
        {
            /*
             * Converts the encoded VCSEL period register value into
             * the real period in PLL clocks
             */

            uint8_t vcsel_period_pclks = 0;

            vcsel_period_pclks = (vcsel_period_reg + 1) << 1;

            return vcsel_period_pclks;
        }

        static void decode_row_col(
                uint8_t  spad_number,
                uint8_t  *prow,
                uint8_t  *pcol)
        {

            /**
             *  Decodes the array (row,col) location from
             *  the input SPAD number
             */

            if (spad_number > 127) {
                *prow = 8 + ((255-spad_number) & 0x07);
                *pcol = (spad_number-128) >> 3;
            } else {
                *prow = spad_number & 0x07;
                *pcol = (127-spad_number) >> 3;
            }
        }

        static void copy_rtn_good_spads_to_buffer(
                nvm_copy_data_t  *pdata,
                uint8_t                 *pbuffer)
        {
            /*
             * Convenience function to copy return SPAD enables to buffer
             */

            *(pbuffer +  0) = pdata->global_config_spad_enables_rtn_0;
            *(pbuffer +  1) = pdata->global_config_spad_enables_rtn_1;
            *(pbuffer +  2) = pdata->global_config_spad_enables_rtn_2;
            *(pbuffer +  3) = pdata->global_config_spad_enables_rtn_3;
            *(pbuffer +  4) = pdata->global_config_spad_enables_rtn_4;
            *(pbuffer +  5) = pdata->global_config_spad_enables_rtn_5;
            *(pbuffer +  6) = pdata->global_config_spad_enables_rtn_6;
            *(pbuffer +  7) = pdata->global_config_spad_enables_rtn_7;
            *(pbuffer +  8) = pdata->global_config_spad_enables_rtn_8;
            *(pbuffer +  9) = pdata->global_config_spad_enables_rtn_9;
            *(pbuffer + 10) = pdata->global_config_spad_enables_rtn_10;
            *(pbuffer + 11) = pdata->global_config_spad_enables_rtn_11;
            *(pbuffer + 12) = pdata->global_config_spad_enables_rtn_12;
            *(pbuffer + 13) = pdata->global_config_spad_enables_rtn_13;
            *(pbuffer + 14) = pdata->global_config_spad_enables_rtn_14;
            *(pbuffer + 15) = pdata->global_config_spad_enables_rtn_15;
            *(pbuffer + 16) = pdata->global_config_spad_enables_rtn_16;
            *(pbuffer + 17) = pdata->global_config_spad_enables_rtn_17;
            *(pbuffer + 18) = pdata->global_config_spad_enables_rtn_18;
            *(pbuffer + 19) = pdata->global_config_spad_enables_rtn_19;
            *(pbuffer + 20) = pdata->global_config_spad_enables_rtn_20;
            *(pbuffer + 21) = pdata->global_config_spad_enables_rtn_21;
            *(pbuffer + 22) = pdata->global_config_spad_enables_rtn_22;
            *(pbuffer + 23) = pdata->global_config_spad_enables_rtn_23;
            *(pbuffer + 24) = pdata->global_config_spad_enables_rtn_24;
            *(pbuffer + 25) = pdata->global_config_spad_enables_rtn_25;
            *(pbuffer + 26) = pdata->global_config_spad_enables_rtn_26;
            *(pbuffer + 27) = pdata->global_config_spad_enables_rtn_27;
            *(pbuffer + 28) = pdata->global_config_spad_enables_rtn_28;
            *(pbuffer + 29) = pdata->global_config_spad_enables_rtn_29;
            *(pbuffer + 30) = pdata->global_config_spad_enables_rtn_30;
            *(pbuffer + 31) = pdata->global_config_spad_enables_rtn_31;
        }

        uint32_t calc_macro_period_us(
                uint16_t  fast_osc_frequency,
                uint8_t   vcsel_period)
        {
            /* Calculates macro period in [us] from the input fast oscillator
             * frequency and VCSEL period
             *
             * Macro period fixed point format = unsigned 12.12
             * Maximum supported macro period  = 4095.9999 us
             */

            uint32_t  pll_period_us        = 0;
            uint8_t   vcsel_period_pclks   = 0;
            uint32_t  macro_period_us      = 0;


            /*  Calculate PLL period in [us] from the  fast_osc_frequency
             *  Fast osc frequency fixed point format = unsigned 4.12
             */

            pll_period_us = calc_pll_period_us(fast_osc_frequency);

            /*  VCSEL period
             *  - the real VCSEL period in PLL clocks = 2*(VCSEL_PERIOD+1)
             */

            vcsel_period_pclks = decode_vcsel_period(vcsel_period);

            /*  Macro period
             *  - PLL period [us]      = 0.24 format
             *      - for 1.0 MHz fast oscillator freq
             *      - max PLL period = 1/64 (6-bits)
             *      - i.e only the lower 18-bits of PLL Period value are used
             *  - Macro period [vclks] = 2304 (12-bits)
             *
             *  Max bits (24 - 6) + 12 = 30-bits usage
             *
             *  Downshift by 6 before multiplying by the VCSEL Period
             */

            macro_period_us =
                (uint32_t)MACRO_PERIOD_VCSEL_PERIODS *
                pll_period_us;
            macro_period_us = macro_period_us >> 6;

            macro_period_us = macro_period_us * (uint32_t)vcsel_period_pclks;
            macro_period_us = macro_period_us >> 6;

            return macro_period_us;
        }


        static uint32_t calc_timeout_us(
                uint32_t timeout_mclks,
                uint32_t macro_period_us)
        {
            /*  Calculates the  timeout in [us] based on the input
             *  encoded timeout and the macro period in [us]
             *
             *  Max timeout supported is 1000000 us (1 sec) -> 20-bits
             *  Max timeout in 20.12 format = 32-bits
             *
             *  Macro period [us] = 12.12 format
             */

            uint32_t timeout_us     = 0;
            uint64_t tmp            = 0;


            tmp  = (uint64_t)timeout_mclks * (uint64_t)macro_period_us;
            tmp += 0x00800;
            tmp  = tmp >> 12;

            timeout_us = (uint32_t)tmp;

            return timeout_us;
        }

        static uint32_t decode_timeout(uint16_t encoded_timeout)
        {
            /*
             * Decode 16-bit timeout register value
             * format (LSByte * 2^MSByte) + 1
             */

            uint32_t timeout_macro_clks = 0;

            timeout_macro_clks = ((uint32_t) (encoded_timeout & 0x00FF)
                    << (uint32_t) ((encoded_timeout & 0xFF00) >> 8)) + 1;

            return timeout_macro_clks;
        }
        static uint32_t calc_decoded_timeout_us(
                uint16_t timeout_encoded,
                uint32_t macro_period_us)
        {
            /*  Calculates the  timeout in [us] based on the input
             *  encoded timeout and the macro period in [us]
             *
             *  Max timeout supported is 1000000 us (1 sec) -> 20-bits
             *  Max timeout in 20.12 format = 32-bits
             *
             *  Macro period [us] = 12.12 format
             */

            uint32_t timeout_mclks  = 0;
            uint32_t timeout_us     = 0;

            timeout_mclks =
                decode_timeout(timeout_encoded);

            timeout_us    =
                calc_timeout_us(timeout_mclks, macro_period_us);

            return timeout_us;
        }

        static uint16_t encode_timeout(uint32_t timeout_mclks)
        {
            /*
             * Encode timeout in macro periods in (LSByte * 2^MSByte) + 1 format
             */

            uint16_t encoded_timeout = 0;
            uint32_t ls_byte = 0;
            uint16_t ms_byte = 0;

            if (timeout_mclks > 0) {
                ls_byte = timeout_mclks - 1;

                while ((ls_byte & 0xFFFFFF00) > 0) {
                    ls_byte = ls_byte >> 1;
                    ms_byte++;
                }

                encoded_timeout = (ms_byte << 8)
                    + (uint16_t) (ls_byte & 0x000000FF);
            }

            return encoded_timeout;
        }

        static uint32_t calc_timeout_mclks(
                uint32_t timeout_us,
                uint32_t macro_period_us)
        {
            /*  Calculates the timeout value in macro periods based on the input
             *  timeout period in milliseconds and the macro period in [us]
             *
             *  Max timeout supported is 1000000 us (1 sec) -> 20-bits
             *  Max timeout in 20.12 format = 32-bits
             *
             *  Macro period [us] = 12.12 format
             */

            uint32_t timeout_mclks   = 0;


            timeout_mclks   =
                ((timeout_us << 12) + (macro_period_us>>1)) /
                macro_period_us;


            return timeout_mclks;
        }

        static uint16_t calc_encoded_timeout(
                uint32_t timeout_us,
                uint32_t macro_period_us)
        {
            /*  Calculates the encoded timeout register value based on the input
             *  timeout period in milliseconds and the macro period in [us]
             *
             *  Max timeout supported is 1000000 us (1 sec) -> 20-bits
             *  Max timeout in 20.12 format = 32-bits
             *
             *  Macro period [us] = 12.12 format
             */

            uint32_t timeout_mclks   = 0;
            uint16_t timeout_encoded = 0;


            timeout_mclks   =
                calc_timeout_mclks(timeout_us, macro_period_us);

            timeout_encoded =
                encode_timeout(timeout_mclks);

            return timeout_encoded;
        }



        error_t calc_timeout_register_values(
                uint32_t                 phasecal_config_timeout_us,
                uint32_t                 mm_config_timeout_us,
                uint32_t                 range_config_timeout_us,
                uint16_t                 fast_osc_frequency,
                general_config_t *pgeneral,
                timing_config_t  *ptiming)
        {
            /*
             * Converts the input MM and range timeouts in [us]
             * into the appropriate register values
             *
             * Must also be run after the VCSEL period settings are changed
             */

            error_t status = ERROR_NONE;

            uint32_t macro_period_us    = 0;
            uint32_t timeout_mclks      = 0;
            uint16_t timeout_encoded    = 0;


            if (fast_osc_frequency == 0) {
                status = ERROR_DIVISION_BY_ZERO;
            } else {
                /* Update Macro Period for Range A VCSEL Period */
                macro_period_us =
                    calc_macro_period_us(
                            fast_osc_frequency,
                            ptiming->range_config_vcsel_period_a);

                /*  Update Phase timeout - uses Timing A */
                timeout_mclks =
                    calc_timeout_mclks(
                            phasecal_config_timeout_us,
                            macro_period_us);

                /* clip as the phase cal timeout register is only 8-bits */
                if (timeout_mclks > 0xFF)
                    timeout_mclks = 0xFF;

                pgeneral->phasecal_config_timeout_macrop =
                    (uint8_t)timeout_mclks;

                /*  Update MM Timing A timeout */
                timeout_encoded =
                    calc_encoded_timeout(
                            mm_config_timeout_us,
                            macro_period_us);

                ptiming->mm_config_timeout_macrop_a_hi =
                    (uint8_t)((timeout_encoded & 0xFF00) >> 8);
                ptiming->mm_config_timeout_macrop_a_lo =
                    (uint8_t) (timeout_encoded & 0x00FF);

                /* Update Range Timing A timeout */
                timeout_encoded =
                    calc_encoded_timeout(
                            range_config_timeout_us,
                            macro_period_us);

                ptiming->range_config_timeout_macrop_a_hi =
                    (uint8_t)((timeout_encoded & 0xFF00) >> 8);
                ptiming->range_config_timeout_macrop_a_lo =
                    (uint8_t) (timeout_encoded & 0x00FF);

                /* Update Macro Period for Range B VCSEL Period */
                macro_period_us =
                    calc_macro_period_us(
                            fast_osc_frequency,
                            ptiming->range_config_vcsel_period_b);

                /* Update MM Timing B timeout */
                timeout_encoded =
                    calc_encoded_timeout(
                            mm_config_timeout_us,
                            macro_period_us);

                ptiming->mm_config_timeout_macrop_b_hi =
                    (uint8_t)((timeout_encoded & 0xFF00) >> 8);
                ptiming->mm_config_timeout_macrop_b_lo =
                    (uint8_t) (timeout_encoded & 0x00FF);

                /* Update Range Timing B timeout */
                timeout_encoded = calc_encoded_timeout(
                        range_config_timeout_us,
                        macro_period_us);

                ptiming->range_config_timeout_macrop_b_hi =
                    (uint8_t)((timeout_encoded & 0xFF00) >> 8);
                ptiming->range_config_timeout_macrop_b_lo =
                    (uint8_t) (timeout_encoded & 0x00FF);
            }


            return status;

        }

        static uint16_t calc_range_ignore_threshold(
                uint32_t central_rate,
                int16_t  x_gradient,
                int16_t  y_gradient,
                uint8_t  rate_mult)
        {
            /* Calculates Range Ignore Threshold rate per spad
             * in Mcps - 3.13 format
             *
             * Calculates worst case xtalk rate per spad in array corner
             * based on input central xtalk and x and y gradients
             *
             * Worst case rate = central rate + (8*(magnitude(xgrad)) +
             * (8*(magnitude(ygrad)))
             *
             * Range ignore threshold rate is then multiplied by user input
             * rate_mult (in 3.5 fractional format)
             *
             */

            int32_t    range_ignore_thresh_int  = 0;
            uint16_t   range_ignore_thresh_kcps = 0;
            int32_t    central_rate_int         = 0;
            int16_t    x_gradient_int           = 0;
            int16_t    y_gradient_int           = 0;


            /* Shift central_rate to .13 fractional for simple addition */

            central_rate_int = ((int32_t)central_rate * (1 << 4)) / (1000);

            if (x_gradient < 0) {
                x_gradient_int = x_gradient * -1;
            }

            if (y_gradient < 0) {
                y_gradient_int = y_gradient * -1;
            }

            /* Calculate full rate per spad - worst case from measured xtalk */
            /* Generated here from .11 fractional kcps */
            /* Additional factor of 4 applied to bring fractional precision to .13 */

            range_ignore_thresh_int = (8 * x_gradient_int * 4) + (8 * y_gradient_int * 4);

            /* Convert Kcps to Mcps */

            range_ignore_thresh_int = range_ignore_thresh_int / 1000;

            /* Combine with Central Rate - Mcps .13 format*/

            range_ignore_thresh_int = range_ignore_thresh_int + central_rate_int;

            /* Mult by user input */

            range_ignore_thresh_int = (int32_t)rate_mult * range_ignore_thresh_int;

            range_ignore_thresh_int = (range_ignore_thresh_int + (1<<4)) / (1<<5);

            /* Finally clip and output in correct format */

            if (range_ignore_thresh_int > 0xFFFF) {
                range_ignore_thresh_kcps = 0xFFFF;
            } else {
                range_ignore_thresh_kcps = (uint16_t)range_ignore_thresh_int;
            }

            return range_ignore_thresh_kcps;
        }

        static void encode_row_col(
                uint8_t  row,
                uint8_t  col,
                uint8_t *pspad_number)
        {
            /**
             *  Encodes the input array(row,col) location as SPAD number.
             */

            if (row > 7) {
                *pspad_number = 128 + (col << 3) + (15-row);
            } else {
                *pspad_number = ((15-col) << 3) + row;
            }
        }

        static void encode_zone_size(
                uint8_t  width,
                uint8_t  height,
                uint8_t *pencoded_xy_size)
        {
            /* merge x and y sizes
             *
             * Important: the sense of the device width and height is swapped
             * versus the API sense
             *
             * MS Nibble = height
             * LS Nibble = width
             */

            *pencoded_xy_size = (height << 4) + width;

        }
        error_t config_low_power_auto_mode(
                general_config_t   *pgeneral,
                dynamic_config_t   *pdynamic,
                low_power_auto_data_t *plpadata
                )
        {

            /*
             * Initializes configs for when low power auto presets are selected
             */

            /* don't really use this here */
            error_t  status = ERROR_NONE;


            /* set low power auto mode */
            plpadata->is_low_power_auto_mode = 1;

            /* set low power range count to 0 */
            plpadata->low_power_auto_range_count = 0;

            /* Turn off MM1/MM2 and DSS2 */
            pdynamic->system_sequence_config = 
                SEQUENCE_VHV_EN | 
                SEQUENCE_PHASECAL_EN | 
                SEQUENCE_DSS1_EN | 
                SEQUENCE_RANGE_EN;

            /* Set DSS to manual/expected SPADs */
            pgeneral->dss_config_manual_effective_spads_select = 200 << 8;
            pgeneral->dss_config_roi_mode_control =
                DEVICEDSSMODE_REQUESTED_EFFFECTIVE_SPADS;

            return status;
        }

        error_t get_timeouts_us(
                uint32_t            *pphasecal_config_timeout_us,
                uint32_t            *pmm_config_timeout_us,
                uint32_t			*prange_config_timeout_us)
        {
            /**
             * Convenience function for getting the MM and range
             * timeouts
             */

            error_t  status = ERROR_NONE;

            uint32_t  macro_period_us = 0;
            uint16_t  timeout_encoded = 0;

            if (_stat_nvm.osc_measured_fast_osc_frequency == 0)
                status = ERROR_DIVISION_BY_ZERO;

            if (status == ERROR_NONE) {

                /* Update Macro Period for Range A VCSEL Period */
                macro_period_us =
                    calc_macro_period_us(
                            _stat_nvm.osc_measured_fast_osc_frequency,
                            _tim_cfg.range_config_vcsel_period_a);

                /*  Get Phase Cal Timing A timeout */

                *pphasecal_config_timeout_us =
                    calc_timeout_us(
                            (uint32_t)_gen_cfg.phasecal_config_timeout_macrop,
                            macro_period_us);

                /*  Get MM Timing A timeout */

                timeout_encoded =
                    (uint16_t)_tim_cfg.mm_config_timeout_macrop_a_hi;
                timeout_encoded = (timeout_encoded << 8) +
                    (uint16_t)_tim_cfg.mm_config_timeout_macrop_a_lo;

                *pmm_config_timeout_us =
                    calc_decoded_timeout_us(
                            timeout_encoded,
                            macro_period_us);

                /* Get Range Timing A timeout */

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

        error_t get_sequence_config_bit(
                DeviceSequenceConfig   bit_id,
                uint8_t                      *pvalue)
        {
            /**
             * Convenience function for getting sequence
             * config enable bits
             */

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

        error_t init_refspadchar_config_struct(
                refspadchar_config_t   *pdata)
        {
            /*
             * Initializes Ref SPAD Char data structures preset mode
             */

            error_t  status = ERROR_NONE;

            /* Reference SPAD Char Configuration
             *
             * vcsel_period              = 0x0B   - 24 clock VCSEL period
             * timeout_us                = 1000   - Set 1000us phase cal timeout
             * target_count_rate_mcps    = 0x0A00 - 9.7 -> 20.0 Mcps
             * min_count_rate_limit_mcps = 0x0500 - 9.7 -> 10.0 Mcps
             * max_count_rate_limit_mcps = 0x1400 - 9.7 -> 40.0 Mcps
             */

            pdata->device_test_mode =
                TUNINGPARM_REFSPADCHAR_DEVICE_TEST_MODE_DEFAULT;
            pdata->vcsel_period              =
                TUNINGPARM_REFSPADCHAR_VCSEL_PERIOD_DEFAULT;
            pdata->timeout_us                =
                TUNINGPARM_REFSPADCHAR_PHASECAL_TIMEOUT_US_DEFAULT;
            pdata->target_count_rate_mcps    =
                TUNINGPARM_REFSPADCHAR_TARGET_COUNT_RATE_MCPS_DEFAULT;
            pdata->min_count_rate_limit_mcps =
                TUNINGPARM_REFSPADCHAR_MIN_COUNTRATE_LIMIT_MCPS_DEFAULT;
            pdata->max_count_rate_limit_mcps =
                TUNINGPARM_REFSPADCHAR_MAX_COUNTRATE_LIMIT_MCPS_DEFAULT;

            return status;
        }

        error_t init_ssc_config_struct(
                ssc_config_t   *pdata)
        {
            /*
             * Initializes SPAD Self Check (SSC) data structure
             */

            error_t  status = ERROR_NONE;

            /* SPAD Select Check Configuration */

            /* 0 - store RTN count rates
             * 1 - store REF count rates
             */
            pdata->array_select = DEVICESSCARRAY_RTN;

            /* VCSEL period register value  0x12 (18) -> 38 VCSEL clocks */
            pdata->vcsel_period =
                TUNINGPARM_SPADMAP_VCSEL_PERIOD_DEFAULT;

            /* VCSEL pulse start */
            pdata->vcsel_start  =
                TUNINGPARM_SPADMAP_VCSEL_START_DEFAULT;

            /* VCSEL pulse width */
            pdata->vcsel_width  = 0x02;

            /* SSC timeout [us] */
            pdata->timeout_us   = 36000;

            /* SSC rate limit [Mcps]
             * - 9.7 for VCSEL ON
             * - 1.15 for VCSEL OFF
             */
            pdata->rate_limit_mcps =
                TUNINGPARM_SPADMAP_RATE_LIMIT_MCPS_DEFAULT;


            return status;
        }

        error_t init_xtalk_config_struct(
                customer_nvm_managed_t *pnvm,
                xtalk_config_t   *pdata)
        {
            /*
             * Initializes Xtalk Config structure
             */

            error_t  status = ERROR_NONE;


            /* Store xtalk data into golden copy */

            pdata->algo_crosstalk_compensation_plane_offset_kcps      =
                pnvm->algo_crosstalk_compensation_plane_offset_kcps;
            pdata->algo_crosstalk_compensation_x_plane_gradient_kcps  =
                pnvm->algo_crosstalk_compensation_x_plane_gradient_kcps;
            pdata->algo_crosstalk_compensation_y_plane_gradient_kcps  =
                pnvm->algo_crosstalk_compensation_y_plane_gradient_kcps;

            /* Store NVM defaults for later use */

            pdata->nvm_default_crosstalk_compensation_plane_offset_kcps      =
                (uint32_t)pnvm->algo_crosstalk_compensation_plane_offset_kcps;
            pdata->nvm_default_crosstalk_compensation_x_plane_gradient_kcps  =
                pnvm->algo_crosstalk_compensation_x_plane_gradient_kcps;
            pdata->nvm_default_crosstalk_compensation_y_plane_gradient_kcps  =
                pnvm->algo_crosstalk_compensation_y_plane_gradient_kcps;

            pdata->lite_mode_crosstalk_margin_kcps                     =
                TUNINGPARM_LITE_XTALK_MARGIN_KCPS_DEFAULT;

            /* Default for Range Ignore Threshold Mult = 2.0 */

            pdata->crosstalk_range_ignore_threshold_mult =
                TUNINGPARM_LITE_RIT_MULT_DEFAULT;

            if ((pdata->algo_crosstalk_compensation_plane_offset_kcps == 0x00)
                    && (pdata->algo_crosstalk_compensation_x_plane_gradient_kcps == 0x00)
                    && (pdata->algo_crosstalk_compensation_y_plane_gradient_kcps == 0x00))
                pdata->global_crosstalk_compensation_enable = 0x00;
            else
                pdata->global_crosstalk_compensation_enable = 0x01;

            if ((status == ERROR_NONE) &&
                    (pdata->global_crosstalk_compensation_enable == 0x01)) {
                pdata->crosstalk_range_ignore_threshold_rate_mcps =
                    calc_range_ignore_threshold(
                            pdata->algo_crosstalk_compensation_plane_offset_kcps,
                            pdata->algo_crosstalk_compensation_x_plane_gradient_kcps,
                            pdata->algo_crosstalk_compensation_y_plane_gradient_kcps,
                            pdata->crosstalk_range_ignore_threshold_mult);
            } else {
                pdata->crosstalk_range_ignore_threshold_rate_mcps = 0;
            }


            return status;
        }

        error_t init_offset_cal_config_struct(
                offsetcal_config_t   *pdata)
        {
            /*
             * Initializes Offset Calibration Config structure
             * - for use with run_offset_calibration()
             */

            error_t  status = ERROR_NONE;


            /* Preset Timeout and DSS defaults */

            pdata->dss_config_target_total_rate_mcps          =
                TUNINGPARM_OFFSET_CAL_DSS_RATE_MCPS_DEFAULT;
            /* 20.0 Mcps */
            pdata->phasecal_config_timeout_us                  =
                TUNINGPARM_OFFSET_CAL_PHASECAL_TIMEOUT_US_DEFAULT;
            /* 1000 us */
            pdata->range_config_timeout_us                     =
                TUNINGPARM_OFFSET_CAL_RANGE_TIMEOUT_US_DEFAULT;
            /* 13000 us */
            pdata->mm_config_timeout_us                        =
                TUNINGPARM_OFFSET_CAL_MM_TIMEOUT_US_DEFAULT;
            /* 13000 us - Added as part of Patch_AddedOffsetCalMMTuningParm_11791 */

            /* Init number of averaged samples */

            pdata->pre_num_of_samples                          =
                TUNINGPARM_OFFSET_CAL_PRE_SAMPLES_DEFAULT;
            pdata->mm1_num_of_samples                          =
                TUNINGPARM_OFFSET_CAL_MM1_SAMPLES_DEFAULT;
            pdata->mm2_num_of_samples                          =
                TUNINGPARM_OFFSET_CAL_MM2_SAMPLES_DEFAULT;


            return status;
        }

        error_t init_tuning_parm_storage_struct(
                tuning_parm_storage_t   *pdata)
        {
            /*
             * Initializes  Tuning Param storage structure
             */

            error_t  status = ERROR_NONE;


            /* Default configuration
             *
             * - Custom overwrite possible from vl53l1_set_tuning_parms()
             * - via tuning file input
             */

            pdata->tp_tuning_parm_version              =
                TUNINGPARM_VERSION_DEFAULT;
            pdata->tp_tuning_parm_key_table_version    =
                TUNINGPARM_KEY_TABLE_VERSION_DEFAULT;
            pdata->tp_tuning_parm_lld_version          =
                TUNINGPARM_LLD_VERSION_DEFAULT;
            pdata->tp_init_phase_rtn_lite_long         =
                TUNINGPARM_INITIAL_PHASE_RTN_LITE_LONG_RANGE_DEFAULT;
            pdata->tp_init_phase_rtn_lite_med          =
                TUNINGPARM_INITIAL_PHASE_RTN_LITE_MED_RANGE_DEFAULT;
            pdata->tp_init_phase_rtn_lite_short        =
                TUNINGPARM_INITIAL_PHASE_RTN_LITE_SHORT_RANGE_DEFAULT;
            pdata->tp_init_phase_ref_lite_long         =
                TUNINGPARM_INITIAL_PHASE_REF_LITE_LONG_RANGE_DEFAULT;
            pdata->tp_init_phase_ref_lite_med          =
                TUNINGPARM_INITIAL_PHASE_REF_LITE_MED_RANGE_DEFAULT;
            pdata->tp_init_phase_ref_lite_short        =
                TUNINGPARM_INITIAL_PHASE_REF_LITE_SHORT_RANGE_DEFAULT;
            pdata->tp_consistency_lite_phase_tolerance =
                TUNINGPARM_CONSISTENCY_LITE_PHASE_TOLERANCE_DEFAULT;
            pdata->tp_phasecal_target                  =
                TUNINGPARM_PHASECAL_TARGET_DEFAULT;
            pdata->tp_cal_repeat_rate                  =
                TUNINGPARM_LITE_CAL_REPEAT_RATE_DEFAULT;
            pdata->tp_lite_min_clip                    =
                TUNINGPARM_LITE_MIN_CLIP_MM_DEFAULT;
            pdata->tp_lite_long_sigma_thresh_mm        =
                TUNINGPARM_LITE_LONG_SIGMA_THRESH_MM_DEFAULT;
            pdata->tp_lite_med_sigma_thresh_mm         =
                TUNINGPARM_LITE_MED_SIGMA_THRESH_MM_DEFAULT;
            pdata->tp_lite_short_sigma_thresh_mm       =
                TUNINGPARM_LITE_SHORT_SIGMA_THRESH_MM_DEFAULT;
            pdata->tp_lite_long_min_count_rate_rtn_mcps  =
                TUNINGPARM_LITE_LONG_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
            pdata->tp_lite_med_min_count_rate_rtn_mcps   =
                TUNINGPARM_LITE_MED_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
            pdata->tp_lite_short_min_count_rate_rtn_mcps =
                TUNINGPARM_LITE_SHORT_MIN_COUNT_RATE_RTN_MCPS_DEFAULT;
            pdata->tp_lite_sigma_est_pulse_width_ns      =
                TUNINGPARM_LITE_SIGMA_EST_PULSE_WIDTH_DEFAULT;
            pdata->tp_lite_sigma_est_amb_width_ns        =
                TUNINGPARM_LITE_SIGMA_EST_AMB_WIDTH_NS_DEFAULT;
            pdata->tp_lite_sigma_ref_mm                  =
                TUNINGPARM_LITE_SIGMA_REF_MM_DEFAULT;
            pdata->tp_lite_seed_cfg                      =
                TUNINGPARM_LITE_SEED_CONFIG_DEFAULT;
            pdata->tp_timed_seed_cfg                     =
                TUNINGPARM_TIMED_SEED_CONFIG_DEFAULT;
            pdata->tp_lite_quantifier                    =
                TUNINGPARM_LITE_QUANTIFIER_DEFAULT;
            pdata->tp_lite_first_order_select            =
                TUNINGPARM_LITE_FIRST_ORDER_SELECT_DEFAULT;

            /* Preset Mode Configurations */
            /* - New parms added as part of Patch_TuningParmPresetModeAddition_11839 */

            pdata->tp_dss_target_lite_mcps               =
                TUNINGPARM_LITE_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
            pdata->tp_dss_target_timed_mcps              =
                TUNINGPARM_TIMED_DSS_CONFIG_TARGET_TOTAL_RATE_MCPS_DEFAULT;
            pdata->tp_phasecal_timeout_lite_us           =
                TUNINGPARM_LITE_PHASECAL_CONFIG_TIMEOUT_US;
            pdata->tp_phasecal_timeout_timed_us          =
                TUNINGPARM_TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT;
            pdata->tp_mm_timeout_lite_us                 =
                TUNINGPARM_LITE_MM_CONFIG_TIMEOUT_US_DEFAULT;
            pdata->tp_mm_timeout_timed_us                =
                TUNINGPARM_TIMED_MM_CONFIG_TIMEOUT_US_DEFAULT;
            pdata->tp_range_timeout_lite_us              =
                TUNINGPARM_LITE_RANGE_CONFIG_TIMEOUT_US_DEFAULT;
            pdata->tp_range_timeout_timed_us             =
                TUNINGPARM_TIMED_RANGE_CONFIG_TIMEOUT_US_DEFAULT;

            /* Added for Patch_LowPowerAutoMode */

            pdata->tp_mm_timeout_lpa_us =
                TUNINGPARM_LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT;
            pdata->tp_range_timeout_lpa_us =
                TUNINGPARM_LOWPOWERAUTO_RANGE_CONFIG_TIMEOUT_US_DEFAULT;

            return status;
        }

        error_t set_inter_measurement_period_ms(uint32_t inter_measurement_period_ms)
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
                DevicePresetModes     device_preset_mode,
                uint16_t                    *pdss_config_target_total_rate_mcps,
                uint32_t                    *pphasecal_config_timeout_us,
                uint32_t                    *pmm_config_timeout_us,
                uint32_t                    *prange_config_timeout_us)
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
                        _tuning_parms.tp_dss_target_lite_mcps;
                    *pphasecal_config_timeout_us =
                        _tuning_parms.tp_phasecal_timeout_lite_us;
                    *pmm_config_timeout_us =
                        _tuning_parms.tp_mm_timeout_lite_us;
                    *prange_config_timeout_us =
                        _tuning_parms.tp_range_timeout_lite_us;
                    break;

                case DEVICEPRESETMODE_TIMED_RANGING:
                case DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE:
                case DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE:
                case DEVICEPRESETMODE_SINGLESHOT_RANGING:
                    *pdss_config_target_total_rate_mcps =
                        _tuning_parms.tp_dss_target_timed_mcps;
                    *pphasecal_config_timeout_us =
                        _tuning_parms.tp_phasecal_timeout_timed_us;
                    *pmm_config_timeout_us =
                        _tuning_parms.tp_mm_timeout_timed_us;
                    *prange_config_timeout_us =
                        _tuning_parms.tp_range_timeout_timed_us;
                    break;

                case DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE:
                case DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE:
                case DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE:
                    *pdss_config_target_total_rate_mcps =
                        _tuning_parms.tp_dss_target_timed_mcps;
                    *pphasecal_config_timeout_us =
                        _tuning_parms.tp_phasecal_timeout_timed_us;
                    *pmm_config_timeout_us =
                        _tuning_parms.tp_mm_timeout_lpa_us;
                    *prange_config_timeout_us =
                        _tuning_parms.tp_range_timeout_lpa_us;
                    break;

                default:
                    status = ERROR_INVALID_PARAMS;
                    break;

            }

            return status;
        }

        error_t set_timeouts_us(
                uint32_t            phasecal_config_timeout_us,
                uint32_t            mm_config_timeout_us,
                uint32_t            range_config_timeout_us)
        {
            /**
             * Convenience function for setting the MM and range
             * timeouts
             */

            error_t  status = ERROR_NONE;

            if (_stat_nvm.osc_measured_fast_osc_frequency == 0)
                status = ERROR_DIVISION_BY_ZERO;

            if (status == ERROR_NONE) {

                _phasecal_config_timeout_us = phasecal_config_timeout_us;
                _mm_config_timeout_us       = mm_config_timeout_us;
                _range_config_timeout_us    = range_config_timeout_us;

                status =
                    calc_timeout_register_values(
                            phasecal_config_timeout_us,
                            mm_config_timeout_us,
                            range_config_timeout_us,
                            _stat_nvm.osc_measured_fast_osc_frequency,
                            &(_gen_cfg),
                            &(_tim_cfg));
            }


            return status;
        }

        error_t preset_mode_standard_ranging(
                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms)
        {
            /*
             * Initializes static and dynamic data structures fordevice preset mode
             * DEVICEPRESETMODE_STANDARD_RANGING
             *
             *  - streaming
             *  - single sigma delta
             *  - back to back
             *
             *  PLEASE NOTE THE SETTINGS BELOW AT PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Static Configuration */

            /* dss_config_target_total_rate_mcps = 20.0 Mcps 9.7 fp */
            pstatic->dss_config_target_total_rate_mcps               = 0x0A00;
            pstatic->debug_ctrl                                      = 0x00;
            pstatic->test_mode_ctrl                                  = 0x00;
            pstatic->clk_gating_ctrl                                 = 0x00;
            pstatic->nvm_bist_ctrl                                   = 0x00;
            pstatic->nvm_bist_num_nvm_words                          = 0x00;
            pstatic->nvm_bist_start_address                          = 0x00;
            pstatic->host_if_status                                  = 0x00;
            pstatic->pad_i2c_hv_config                               = 0x00;
            pstatic->pad_i2c_hv_extsup_config                        = 0x00;

            /*
             *  0 - gpio_extsup_hv
             *  1 - gpio_vmodeint_hv
             */
            pstatic->gpio_hv_pad_ctrl                                = 0x00;

            /*
             * Set interrupt active low
             *
             *  3:0 - gpio_mux_select_hv
             *    4 - gpio_mux_active_high_hv
             */
            pstatic->gpio_hv_mux_ctrl  = 
                DEVICEINTERRUPTPOLARITY_ACTIVE_LOW |
                DEVICEGPIOMODE_OUTPUT_RANGE_AND_ERROR_INTERRUPTS;

            pstatic->gpio_tio_hv_status                              = 0x02;
            pstatic->gpio_fio_hv_status                              = 0x00;
            pstatic->ana_config_spad_sel_pswidth                     = 0x02;
            pstatic->ana_config_vcsel_pulse_width_offset             = 0x08;
            pstatic->ana_config_fast_osc_config_ctrl                = 0x00;

            pstatic->sigma_estimator_effective_pulse_width_ns        =
                ptuning_parms->tp_lite_sigma_est_pulse_width_ns;
            pstatic->sigma_estimator_effective_ambient_width_ns      =
                ptuning_parms->tp_lite_sigma_est_amb_width_ns;
            pstatic->sigma_estimator_sigma_ref_mm                    =
                ptuning_parms->tp_lite_sigma_ref_mm;
            /* Minimum allowable value of 1 - 0 disables the feature */
            pstatic->algo_crosstalk_compensation_valid_height_mm     = 0x01;
            pstatic->spare_host_config_static_config_spare_0         = 0x00;
            pstatic->spare_host_config_static_config_spare_1         = 0x00;

            pstatic->algo_range_ignore_threshold_mcps                = 0x0000;

            /* set RIT distance to 20 mm */
            pstatic->algo_range_ignore_valid_height_mm               = 0xff;
            pstatic->algo_range_min_clip                             =
                ptuning_parms->tp_lite_min_clip;
            /*
             * Phase consistency check limit - format 1.3 fp
             * 0x02 -> 0.25
             * 0x08 -> 1.00
             */
            pstatic->algo_consistency_check_tolerance               =
                ptuning_parms->tp_consistency_lite_phase_tolerance;
            pstatic->spare_host_config_static_config_spare_2         = 0x00;
            pstatic->sd_config_reset_stages_msb                      = 0x00;
            pstatic->sd_config_reset_stages_lsb                      = 0x00;

            pgeneral->gph_config_stream_count_update_value           = 0x00;
            pgeneral->global_config_stream_divider                   = 0x00;
            pgeneral->system_interrupt_config_gpio =
                INTERRUPT_CONFIG_NEW_SAMPLE_READY;
            pgeneral->cal_config_vcsel_start                         = 0x0B;

            /*
             * Set VHV / Phase Cal repeat rate to 1 every
             * 60 * 60 ranges (once every minute @ 60Hz)
             * 0 - disables
             * 12-bit value -> 4095 max
             */
            pgeneral->cal_config_repeat_rate                         =
                ptuning_parms->tp_cal_repeat_rate;
            pgeneral->global_config_vcsel_width                      = 0x02;
            /* 13 macro periods gives a timeout of 1ms */
            pgeneral->phasecal_config_timeout_macrop                 = 0x0D;
            /* Phase cal target phase 2.0625 - 4.4 fp -> 0x21*/
            pgeneral->phasecal_config_target                         =
                ptuning_parms->tp_phasecal_target;
            pgeneral->phasecal_config_override                       = 0x00;
            pgeneral->dss_config_roi_mode_control =
                DEVICEDSSMODE_TARGET_RATE;
            /* format for threshold high and low is 9.7 fp */
            pgeneral->system_thresh_rate_high                        = 0x0000;
            pgeneral->system_thresh_rate_low                         = 0x0000;
            /* The format for manual effective spads is 8.8 -> 0x8C00 = 140.00 */
            pgeneral->dss_config_manual_effective_spads_select       = 0x8C00;
            pgeneral->dss_config_manual_block_select                 = 0x00;

            /*
             * Aperture attenuation value - format 0.8
             *
             * Nominal:  5x   -> 0.200000 * 256 = 51 = 0x33
             * Measured: 4.6x -> 0.217391 * 256 = 56 = 0x38
             */
            pgeneral->dss_config_aperture_attenuation                = 0x38;
            pgeneral->dss_config_max_spads_limit                     = 0xFF;
            pgeneral->dss_config_min_spads_limit                     = 0x01;

            /* Timing Configuration */

            /* Default timing of 2ms */
            ptiming->mm_config_timeout_macrop_a_hi                   = 0x00;
            ptiming->mm_config_timeout_macrop_a_lo                   = 0x1a;
            ptiming->mm_config_timeout_macrop_b_hi                   = 0x00;
            ptiming->mm_config_timeout_macrop_b_lo                   = 0x20;
            /* Setup for 30ms default */
            ptiming->range_config_timeout_macrop_a_hi                = 0x01;
            ptiming->range_config_timeout_macrop_a_lo                = 0xCC;
            /* register value 11 gives a 24 VCSEL period */
            ptiming->range_config_vcsel_period_a                     = 0x0B;
            /* Setup for 30ms default */
            ptiming->range_config_timeout_macrop_b_hi                = 0x01;
            ptiming->range_config_timeout_macrop_b_lo                = 0xF5;
            /* register value  09 gives a 20 VCSEL period */
            ptiming->range_config_vcsel_period_b                     = 0x09;
            /*
             * Sigma thresh register - format 14.2
             *
             * 0x003C -> 15.0 mm
             * 0x0050 -> 20.0 mm
             */
            ptiming->range_config_sigma_thresh                       =
                ptuning_parms->tp_lite_med_sigma_thresh_mm;
            /*
             *  Rate Limit - format 9.7fp
             *  0x0020 -> 0.250 Mcps
             *  0x0080 -> 1.000 Mcps
             */
            ptiming->range_config_min_count_rate_rtn_limit_mcps      =
                ptuning_parms->tp_lite_med_min_count_rate_rtn_mcps;

            /* Phase limit register formats = 5.3
             * low   = 0x08 ->  1.0
             * high  = 0x78 -> 15.0 -> 3.0m
             */
            ptiming->range_config_valid_phase_low                    = 0x08;
            ptiming->range_config_valid_phase_high                   = 0x78;
            ptiming->system_intermeasurement_period                  = 0x00000000;
            ptiming->system_fractional_enable                        = 0x00;

            /* Dynamic Configuration */

            pdynamic->system_grouped_parameter_hold_0                 = 0x01;

            pdynamic->system_thresh_high                              = 0x0000;
            pdynamic->system_thresh_low                               = 0x0000;
            pdynamic->system_enable_xtalk_per_quadrant                = 0x00;
            pdynamic->system_seed_config =
                ptuning_parms->tp_lite_seed_cfg;

            /* Timing A */
            pdynamic->sd_config_woi_sd0                               = 0x0B;
            /* Timing B */
            pdynamic->sd_config_woi_sd1                               = 0x09;

            pdynamic->sd_config_initial_phase_sd0                     =
                ptuning_parms->tp_init_phase_rtn_lite_med;
            pdynamic->sd_config_initial_phase_sd1                     =
                ptuning_parms->tp_init_phase_ref_lite_med;;

            pdynamic->system_grouped_parameter_hold_1                 = 0x01;

            /*
             *  Quantifier settings
             *
             *  sd_config_first_order_select
             *     bit 0 - return sigma delta
             *     bit 1 - reference sigma delta
             *
             *  sd_config_first_order_select = 0x03 (1st order)
             *
             *      sd_config_quantifier options
             *        0
             *        1 ->   64
             *        2 ->  128
             *        3 ->  256
             *
             *  sd_config_first_order_select = 0x00 (2nd order)
             *
             *      sd_config_quantifier options
             *        0
             *        1  ->  256
             *        2  -> 1024
             *        3  -> 4095
             *
             *  Setting below 2nd order, Quantifier = 1024
             */

            pdynamic->sd_config_first_order_select =
                ptuning_parms->tp_lite_first_order_select;
            pdynamic->sd_config_quantifier         =
                ptuning_parms->tp_lite_quantifier;

            /* Below defaults will be overwritten by zone_cfg
             * Spad no = 199 (0xC7)
             * Spad no =  63 (0x3F)
             */
            pdynamic->roi_config_user_roi_centre_spad              = 0xC7;
            /* 16x16 ROI */
            pdynamic->roi_config_user_roi_requested_global_xy_size = 0xFF;

            pdynamic->system_sequence_config = 
                SEQUENCE_VHV_EN | 
                SEQUENCE_PHASECAL_EN | 
                SEQUENCE_DSS1_EN | 
                SEQUENCE_DSS2_EN | 
                SEQUENCE_MM2_EN | 
                SEQUENCE_RANGE_EN;

            pdynamic->system_grouped_parameter_hold                   = 0x02;

            /* System control */

            psystem->system_stream_count_ctrl                         = 0x00;
            psystem->firmware_enable                                  = 0x01;
            psystem->system_interrupt_clear                           = CLEAR_RANGE_INT;

            psystem->system_mode_start = 
                DEVICESCHEDULERMODE_STREAMING | 
                DEVICEREADOUTMODE_SINGLE_SD | 
                DEVICEMEASUREMENTMODE_BACKTOBACK;

            return status;
        }

        error_t preset_mode_standard_ranging_short_range(
                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms)
        {
            /*
             * Initializes static and dynamic data structures for
             * device preset mode
             *
             * DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE
             * (up to 1.4 metres)
             *
             * PLEASE NOTE THE SETTINGS BELOW AT PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration followed by
             * overrides for the  short range configuration
             */

            status = preset_mode_standard_ranging(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now override standard ranging specific registers */

            if (status == ERROR_NONE) {

                /* Timing Configuration
                 *
                 * vcsel_period_a    = 7 -> 16 period
                 * vcsel_period_b    = 5 -> 12 period
                 * sigma_thresh                  = 0x003C -> 14.2fp -> 15.0 mm
                 * min_count_rate_rtn_limit_mcps = 0x0080 ->  9.7fp ->  1.0 Mcps
                 * valid_phase_low               = 0x08 -> 5.3fp -> 1.0
                 * valid_phase_high              = 0x38 -> 5.3fp -> 7.0 -> 1.4m
                 */

                ptiming->range_config_vcsel_period_a                = 0x07;
                ptiming->range_config_vcsel_period_b                = 0x05;
                ptiming->range_config_sigma_thresh                  =
                    ptuning_parms->tp_lite_short_sigma_thresh_mm;
                ptiming->range_config_min_count_rate_rtn_limit_mcps =
                    ptuning_parms->tp_lite_short_min_count_rate_rtn_mcps;
                ptiming->range_config_valid_phase_low               = 0x08;
                ptiming->range_config_valid_phase_high              = 0x38;

                /* Dynamic Configuration
                 * SD0 -> Timing A
                 * SD1 -> Timing B
                 */

                pdynamic->sd_config_woi_sd0                         = 0x07;
                pdynamic->sd_config_woi_sd1                         = 0x05;
                pdynamic->sd_config_initial_phase_sd0               =
                    ptuning_parms->tp_init_phase_rtn_lite_short;
                pdynamic->sd_config_initial_phase_sd1               =
                    ptuning_parms->tp_init_phase_ref_lite_short;
            }


            return status;
        }

        error_t preset_mode_standard_ranging_long_range(
                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms)
        {
            /*
             * Initializes static and dynamic data structures for
             * device preset mode
             *
             * DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE
             * (up to 4.8 metres)
             *
             *  PLEASE NOTE THE SETTINGS BELOW AT PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration with
             * overrides for long range configuration
             */

            status = preset_mode_standard_ranging(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now override standard ranging specific registers */

            if (status == ERROR_NONE) {

                /* Timing Configuration
                 *
                 * vcsel_period_a    = 15 -> 32 period
                 * vcsel_period_b    = 13 -> 28 period
                 * sigma_thresh                  = 0x003C -> 14.2fp -> 15.0 mm
                 * min_count_rate_rtn_limit_mcps = 0x0080 ->  9.7fp ->  1.0 Mcps
                 * valid_phase_low               = 0x08 -> 5.3fp ->  1.0
                 * valid_phase_high              = 0xB8 -> 5.3fp -> 23.0 -> 4.6m
                 */

                ptiming->range_config_vcsel_period_a                = 0x0F;
                ptiming->range_config_vcsel_period_b                = 0x0D;
                ptiming->range_config_sigma_thresh                  =
                    ptuning_parms->tp_lite_long_sigma_thresh_mm;
                ptiming->range_config_min_count_rate_rtn_limit_mcps =
                    ptuning_parms->tp_lite_long_min_count_rate_rtn_mcps;
                ptiming->range_config_valid_phase_low               = 0x08;
                ptiming->range_config_valid_phase_high              = 0xB8;

                /* Dynamic Configuration
                 * SD0 -> Timing A
                 * SD1 -> Timing B
                 */

                pdynamic->sd_config_woi_sd0                         = 0x0F;
                pdynamic->sd_config_woi_sd1                         = 0x0D;
                pdynamic->sd_config_initial_phase_sd0               =
                    ptuning_parms->tp_init_phase_rtn_lite_long;
                pdynamic->sd_config_initial_phase_sd1               =
                    ptuning_parms->tp_init_phase_ref_lite_long;
            }


            return status;
        }

        error_t preset_mode_standard_ranging_mm1_cal(
                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms)
        {
            /*
             * Initializes static and dynamic data structures for
             * device preset mode
             *
             * DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL
             *
             * PLEASE NOTE THE SETTINGS BELOW AT PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration with
             * overrides for long range configuration
             */

            status = preset_mode_standard_ranging(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now override standard ranging specific registers */

            if (status == ERROR_NONE) {

                pgeneral->dss_config_roi_mode_control =
                    DEVICEDSSMODE_REQUESTED_EFFFECTIVE_SPADS;

                pdynamic->system_sequence_config  = 
                    SEQUENCE_VHV_EN | 
                    SEQUENCE_PHASECAL_EN |
                    SEQUENCE_DSS1_EN | 
                    SEQUENCE_DSS2_EN | 
                    SEQUENCE_MM1_EN;
            }

            return status;
        }

        error_t preset_mode_standard_ranging_mm2_cal(
                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms)
        {
            /*
             * Initializes static and dynamic data structures for
             * device preset mode
             *
             * DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL
             *
             * PLEASE NOTE THE SETTINGS BELOW AT PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration with
             * overrides for long range configuration
             */

            status = preset_mode_standard_ranging(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now override standard ranging specific registers */

            if (status == ERROR_NONE) {

                pgeneral->dss_config_roi_mode_control =
                    DEVICEDSSMODE_REQUESTED_EFFFECTIVE_SPADS;

                pdynamic->system_sequence_config  = 
                    SEQUENCE_VHV_EN | 
                    SEQUENCE_PHASECAL_EN | 
                    SEQUENCE_DSS1_EN | 
                    SEQUENCE_DSS2_EN | 
                    SEQUENCE_MM2_EN;
            }

            return status;
        }

        error_t preset_mode_timed_ranging(

                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms)
        {
            /*
             * Initializes static and dynamic data structures for
             * device preset mode
             *
             * DEVICEPRESETMODE_TIMED_RANGING
             *
             *  - pseudo-solo
             *  - single sigma delta
             *  - timed
             *
             *  PLEASE NOTE THE SETTINGS BELOW AT PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration */

            status = preset_mode_standard_ranging(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now override standard ranging specific registers */

            if (status == ERROR_NONE) {

                /* Dynamic Configuration */

                /* Disable GPH  */
                pdynamic->system_grouped_parameter_hold = 0x00;

                /* Re-Configure timing budget default for 13ms */
                ptiming->range_config_timeout_macrop_a_hi                = 0x00;
                ptiming->range_config_timeout_macrop_a_lo                = 0xB1;
                /* Setup for 13ms default */
                ptiming->range_config_timeout_macrop_b_hi                = 0x00;
                ptiming->range_config_timeout_macrop_b_lo                = 0xD4;

                /* Timing Configuration */

                ptiming->system_intermeasurement_period = 0x00000600;
                pdynamic->system_seed_config =
                    ptuning_parms->tp_timed_seed_cfg;

                /* System control */

                /* Configure Timed/Psuedo-solo mode */
                psystem->system_mode_start =
                    DEVICESCHEDULERMODE_PSEUDO_SOLO | 
                    DEVICEREADOUTMODE_SINGLE_SD     | 
                    DEVICEMEASUREMENTMODE_TIMED;
            }


            return status;
        }

        error_t preset_mode_timed_ranging_short_range(

                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms)
        {
            /*
             * Initializes static and dynamic data structures for
             * device preset mode
             *
             * DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE
             *
             *  - pseudo-solo
             *  - single sigma delta
             *  - timed
             *
             *  PLEASE NOTE THE SETTINGS BELOW AT PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration */

            status = preset_mode_standard_ranging_short_range(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now override standard ranging specific registers */

            if (status == ERROR_NONE) {

                /* Dynamic Configuration */

                /* Disable GPH  */
                pdynamic->system_grouped_parameter_hold = 0x00;

                /* Timing Configuration */

                /* Re-Configure timing budget default for 13ms */
                ptiming->range_config_timeout_macrop_a_hi                = 0x01;
                ptiming->range_config_timeout_macrop_a_lo                = 0x84;
                /* Setup for 13ms default */
                ptiming->range_config_timeout_macrop_b_hi                = 0x01;
                ptiming->range_config_timeout_macrop_b_lo                = 0xB1;

                ptiming->system_intermeasurement_period = 0x00000600;
                pdynamic->system_seed_config =
                    ptuning_parms->tp_timed_seed_cfg;

                /* System control */

                /* Configure Timed/Psuedo-solo mode */
                psystem->system_mode_start =
                    DEVICESCHEDULERMODE_PSEUDO_SOLO | 
                    DEVICEREADOUTMODE_SINGLE_SD     | 
                    DEVICEMEASUREMENTMODE_TIMED;
            }


            return status;
        }

        error_t preset_mode_timed_ranging_long_range(

                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms)
        {
            /*
             * Initializes static and dynamic data structures for
             * device preset mode
             *
             * DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE
             *
             *  - pseudo-solo
             *  - single sigma delta
             *  - timed
             *
             *  PLEASE NOTE THE SETTINGS BELOW AT PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration */

            status = preset_mode_standard_ranging_long_range(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now override standard ranging specific registers */

            if (status == ERROR_NONE) {

                /* Dynamic Configuration */

                /* Disable GPH  */
                pdynamic->system_grouped_parameter_hold = 0x00;

                /* Timing Configuration */

                /* Re-Configure timing budget default for 13ms */
                ptiming->range_config_timeout_macrop_a_hi                = 0x00;
                ptiming->range_config_timeout_macrop_a_lo                = 0x97;
                /* Setup for 13ms default */
                ptiming->range_config_timeout_macrop_b_hi                = 0x00;
                ptiming->range_config_timeout_macrop_b_lo                = 0xB1;

                ptiming->system_intermeasurement_period = 0x00000600;
                pdynamic->system_seed_config =
                    ptuning_parms->tp_timed_seed_cfg;

                /* System control */

                /* Configure Timed/Psuedo-solo mode */
                psystem->system_mode_start =
                    DEVICESCHEDULERMODE_PSEUDO_SOLO | 
                    DEVICEREADOUTMODE_SINGLE_SD     | 
                    DEVICEMEASUREMENTMODE_TIMED;
            }


            return status;
        }

        error_t preset_mode_low_power_auto_ranging(

                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms,
                low_power_auto_data_t *plpadata)
        {
            /*
             * Initializes static and dynamic data structures for
             * device preset mode
             *
             * DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE
             *
             *  - pseudo-solo
             *  - single sigma delta
             *  - timed
             *  - special low power auto mode for Presence application
             *
             *  PLEASE NOTE THE SETTINGS BELOW ARE PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration */

            status = preset_mode_timed_ranging(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now setup the low power auto mode */

            if (status == ERROR_NONE) {
                status = config_low_power_auto_mode(
                        pgeneral,
                        pdynamic,
                        plpadata
                        );
            }


            return status;
        }

        error_t preset_mode_low_power_auto_short_ranging(

                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms,
                low_power_auto_data_t *plpadata)
        {
            /*
             * Initializes static and dynamic data structures for
             * device preset mode
             *
             * DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE
             *
             *  - pseudo-solo
             *  - single sigma delta
             *  - timed
             *  - special low power auto mode for Presence application
             *
             *  PLEASE NOTE THE SETTINGS BELOW ARE PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration */

            status = preset_mode_timed_ranging_short_range(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now setup the low power auto mode */

            if (status == ERROR_NONE) {
                status = config_low_power_auto_mode(
                        pgeneral,
                        pdynamic,
                        plpadata
                        );
            }


            return status;
        }

        error_t preset_mode_low_power_auto_long_ranging(

                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms,
                low_power_auto_data_t *plpadata)
        {
            /*
             * Initializes static and dynamic data structures for
             * device preset mode
             *
             * DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE
             *
             *  - pseudo-solo
             *  - single sigma delta
             *  - timed
             *  - special low power auto mode for Presence application
             *
             *  PLEASE NOTE THE SETTINGS BELOW ARE PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration */

            status = preset_mode_timed_ranging_long_range(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now setup the low power auto mode */

            if (status == ERROR_NONE) {
                status = config_low_power_auto_mode(
                        pgeneral,
                        pdynamic,
                        plpadata
                        );
            }


            return status;
        }

        /* End Patch_LowPowerAutoMode */

        error_t preset_mode_singleshot_ranging(

                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms)
        {
            /*
             * Initializes static and dynamic data structures for device preset mode
             * DEVICEPRESETMODE_TIMED_RANGING
             *
             *  - pseudo-solo
             *  - single sigma delta
             *  - timed
             *
             *  PLEASE NOTE THE SETTINGS BELOW AT PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration */

            status = preset_mode_standard_ranging(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now override standard ranging specific registers */

            if (status == ERROR_NONE) {

                /* Dynamic Configuration */

                /* Disable GPH  */
                pdynamic->system_grouped_parameter_hold = 0x00;

                /* Timing Configuration */

                /* Re-Configure timing budget default for 13ms */
                ptiming->range_config_timeout_macrop_a_hi                = 0x00;
                ptiming->range_config_timeout_macrop_a_lo                = 0xB1;
                /* Setup for 13ms default */
                ptiming->range_config_timeout_macrop_b_hi                = 0x00;
                ptiming->range_config_timeout_macrop_b_lo                = 0xD4;

                pdynamic->system_seed_config =
                    ptuning_parms->tp_timed_seed_cfg;

                /* System control */

                /* Configure Timed/Psuedo-solo mode */
                psystem->system_mode_start = 
                    DEVICESCHEDULERMODE_PSEUDO_SOLO | 
                    DEVICEREADOUTMODE_SINGLE_SD     | 
                    DEVICEMEASUREMENTMODE_SINGLESHOT;
            }


            return status;
        }

        error_t preset_mode_olt(
                static_config_t    *pstatic,
                general_config_t   *pgeneral,
                timing_config_t    *ptiming,
                dynamic_config_t   *pdynamic,
                system_control_t   *psystem,
                tuning_parm_storage_t *ptuning_parms)
        {
            /**
             * Initializes static and dynamic data structures for device preset mode
             * DEVICEPRESETMODE_OLT
             *
             *  PLEASE NOTE THE SETTINGS BELOW AT PROVISIONAL AND WILL CHANGE!
             */

            error_t  status = ERROR_NONE;


            /* Call standard ranging configuration */

            status = preset_mode_standard_ranging(
                    pstatic,
                    pgeneral,
                    ptiming,
                    pdynamic,
                    psystem,
                    ptuning_parms);

            /* now override OLT specific registers */

            if (status == ERROR_NONE) {

                /* Disables requirement for host handshake */
                psystem->system_stream_count_ctrl  = 0x01;
            }

            return status;
        }

        void init_ll_driver_state(DeviceState device_state)
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
                DevicePresetModes     device_preset_mode,
                uint16_t                     dss_config_target_total_rate_mcps,
                uint32_t                     phasecal_config_timeout_us,
                uint32_t                     mm_config_timeout_us,
                uint32_t                     range_config_timeout_us,
                uint32_t                     inter_measurement_period_ms)
        {
            error_t  status = ERROR_NONE;

            static_config_t        *pstatic       = &(_stat_cfg);
            general_config_t       *pgeneral      = &(_gen_cfg);
            timing_config_t        *ptiming       = &(_tim_cfg);
            dynamic_config_t       *pdynamic      = &(_dyn_cfg);
            system_control_t       *psystem       = &(_sys_ctrl);
            tuning_parm_storage_t  *ptuning_parms = &(_tuning_parms);
            low_power_auto_data_t  *plpadata      =
                &(_low_power_auto_data);

            _preset_mode                 = device_preset_mode;
            _mm_config_timeout_us        = mm_config_timeout_us;
            _range_config_timeout_us     = range_config_timeout_us;
            _inter_measurement_period_ms = inter_measurement_period_ms;

            init_ll_driver_state(DEVICESTATE_SW_STANDBY);

            /* apply selected preset */

            switch (device_preset_mode) {

                case DEVICEPRESETMODE_STANDARD_RANGING:
                    status = preset_mode_standard_ranging(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms);
                    break;

                case DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE:
                    status = preset_mode_standard_ranging_short_range(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms);
                    break;

                case DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE:
                    status = preset_mode_standard_ranging_long_range(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms);
                    break;

                case DEVICEPRESETMODE_STANDARD_RANGING_MM1_CAL:
                    status = preset_mode_standard_ranging_mm1_cal(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms);
                    break;

                case DEVICEPRESETMODE_STANDARD_RANGING_MM2_CAL:
                    status = preset_mode_standard_ranging_mm2_cal(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms);
                    break;

                case DEVICEPRESETMODE_TIMED_RANGING:
                    status = preset_mode_timed_ranging(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms);
                    break;

                case DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE:
                    status = preset_mode_timed_ranging_short_range(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms);
                    break;

                case DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE:
                    status = preset_mode_timed_ranging_long_range(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms);
                    break;

                case DEVICEPRESETMODE_OLT:
                    status = preset_mode_olt(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms);
                    break;

                case DEVICEPRESETMODE_SINGLESHOT_RANGING:
                    status = preset_mode_singleshot_ranging(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms);
                    break;

                case DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE:
                    status = preset_mode_low_power_auto_short_ranging(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms,
                            plpadata);
                    break;

                case DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE:
                    status = preset_mode_low_power_auto_ranging(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms,
                            plpadata);
                    break;

                case DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE:
                    status = preset_mode_low_power_auto_long_ranging(
                            pstatic,
                            pgeneral,
                            ptiming,
                            pdynamic,
                            psystem,
                            ptuning_parms,
                            plpadata);
                    break;

                default:
                    status = ERROR_INVALID_PARAMS;
                    break;

            }

            /* update DSS target */

            if (status == ERROR_NONE) {

                pstatic->dss_config_target_total_rate_mcps =
                    dss_config_target_total_rate_mcps;
                _dss_config_target_total_rate_mcps    =
                    dss_config_target_total_rate_mcps;

            }

            /*
             * Update the register timeout values based on input
             * real time values and preset mode VCSEL periods
             */

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

        error_t ComputeDevicePresetMode(
                PresetModes PresetMode,
                DistanceModes DistanceMode,
                DevicePresetModes *pDevicePresetMode)
        {
            error_t Status = ERROR_NONE;

            uint8_t DistIdx;
            DevicePresetModes LightModes[3] = {
                DEVICEPRESETMODE_STANDARD_RANGING_SHORT_RANGE,
                DEVICEPRESETMODE_STANDARD_RANGING,
                DEVICEPRESETMODE_STANDARD_RANGING_LONG_RANGE};

            DevicePresetModes TimedModes[3] = {
                DEVICEPRESETMODE_TIMED_RANGING_SHORT_RANGE,
                DEVICEPRESETMODE_TIMED_RANGING,
                DEVICEPRESETMODE_TIMED_RANGING_LONG_RANGE};

            DevicePresetModes LowPowerTimedModes[3] = {
                DEVICEPRESETMODE_LOWPOWERAUTO_SHORT_RANGE,
                DEVICEPRESETMODE_LOWPOWERAUTO_MEDIUM_RANGE,
                DEVICEPRESETMODE_LOWPOWERAUTO_LONG_RANGE};

            *pDevicePresetMode = DEVICEPRESETMODE_STANDARD_RANGING;

            switch (DistanceMode) {
                case DISTANCEMODE_SHORT:
                    DistIdx = 0;
                    break;
                case DISTANCEMODE_MEDIUM:
                    DistIdx = 1;
                    break;
                default:
                    DistIdx = 2;
            }

            switch (PresetMode) {
                case PRESETMODE_LITE_RANGING:
                    *pDevicePresetMode = LightModes[DistIdx];
                    break;

                case PRESETMODE_AUTONOMOUS:
                    *pDevicePresetMode = TimedModes[DistIdx];
                    break;

                case PRESETMODE_LOWPOWER_AUTONOMOUS:
                    *pDevicePresetMode = LowPowerTimedModes[DistIdx];
                    break;

                default:
                    /* Unsupported mode */
                    Status = ERROR_MODE_NOT_SUPPORTED;
            }

            return Status;
        }

        error_t helper_set_preset_mode(
                PresetModes PresetMode,
                DistanceModes DistanceMode,
                uint32_t inter_measurement_period_ms)
        {
            error_t Status = ERROR_NONE;
            DevicePresetModes   device_preset_mode;
            uint8_t measurement_mode;
            uint16_t dss_config_target_total_rate_mcps;
            uint32_t phasecal_config_timeout_us;
            uint32_t mm_config_timeout_us;
            uint32_t lld_range_config_timeout_us;

            if ((PresetMode == PRESETMODE_AUTONOMOUS) ||
                    (PresetMode == PRESETMODE_LOWPOWER_AUTONOMOUS))
                measurement_mode  = DEVICEMEASUREMENTMODE_TIMED;
            else
                measurement_mode  = DEVICEMEASUREMENTMODE_BACKTOBACK;

            Status = ComputeDevicePresetMode(PresetMode, DistanceMode,
                    &device_preset_mode);

            if (Status == ERROR_NONE)
                Status =  get_preset_mode_timing_cfg(
                        device_preset_mode,
                        &dss_config_target_total_rate_mcps,
                        &phasecal_config_timeout_us,
                        &mm_config_timeout_us,
                        &lld_range_config_timeout_us);

            if (Status == ERROR_NONE)
                Status = set_preset_mode(
                        device_preset_mode,
                        dss_config_target_total_rate_mcps,
                        phasecal_config_timeout_us,
                        mm_config_timeout_us,
                        lld_range_config_timeout_us,
                        inter_measurement_period_ms);

            if (Status == ERROR_NONE) {
                _measurement_mode = measurement_mode;
            }

            if (Status == ERROR_NONE) {
                CurrentParameters.PresetMode = PresetMode;
            }

            return Status;
        }

        error_t SetInterMeasurementPeriodMilliSeconds(
                uint32_t InterMeasurementPeriodMilliSeconds)
        {
            error_t Status = ERROR_NONE;

            Status = set_inter_measurement_period_ms(InterMeasurementPeriodMilliSeconds);

            return Status;
        }

        error_t GetSequenceStepEnable(
                SequenceStepId SequenceStepId, uint8_t *pSequenceStepEnabled)
        {
            error_t Status = ERROR_NONE;

            Status = get_sequence_config_bit(
                    (DeviceSequenceConfig)SequenceStepId,
                    pSequenceStepEnabled);

            return Status;
        }


        error_t RdWord(uint16_t index, uint16_t *pdata)
        {
            static uint16_t r16data = 0;

            error_t status = read_bytes(index, 2, (uint8_t *)&r16data);

            *pdata = r16data;

            return status;
        }

        error_t i2c_encode_system_control(uint16_t buf_size, uint8_t *pbuffer)
        {
            if (SYSTEM_CONTROL_I2C_SIZE_BYTES > buf_size) {
                return ERROR_COMMS_BUFFER_TOO_SMALL;
            }

            //*(pbuffer + 0) = _sys_ctrl.power_management_go1_power_force & 0x1;
            //*(pbuffer + 1) = _sys_ctrl.system_stream_count_ctrl & 0x1;
            *(pbuffer + 2) = _sys_ctrl.firmware_enable & 0x1;
            *(pbuffer + 3) = _sys_ctrl.system_interrupt_clear & 0x3;
            *(pbuffer + 4) = _sys_ctrl.system_mode_start;

            return ERROR_NONE;
        }

        error_t set_lite_min_count_rate(uint16_t lite_mincountrate)
        {
            error_t  status = ERROR_NONE;

            _tim_cfg.range_config_min_count_rate_rtn_limit_mcps =
                lite_mincountrate;

            return status;

        }

        error_t set_lite_sigma_threshold(uint16_t lite_sigma)
        {
            error_t  status = ERROR_NONE;

            _tim_cfg.range_config_sigma_thresh = lite_sigma;

            return status;

        }

        error_t set_limit_value(uint16_t LimitCheckId, FixedPoint1616_t value)
        {
            error_t Status = ERROR_NONE;
            uint16_t tmpuint16; /* temporary variable */

            switch (LimitCheckId) {
                case CHECKENABLE_SIGMA_FINAL_RANGE:
                    tmpuint16 = FIXEDPOINT1616TOFIXEDPOINT142(value);
                    set_lite_sigma_threshold(tmpuint16);
                    break;
                case CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
                    tmpuint16 = FIXEDPOINT1616TOFIXEDPOINT97(value);
                    set_lite_min_count_rate(tmpuint16);
                    break;
                default:
                    Status = ERROR_INVALID_PARAMS;
            }

            return Status;
        }

        error_t set_limit_check_value(

                uint16_t LimitCheckId, 
                FixedPoint1616_t LimitCheckValue)
        {
            error_t Status = ERROR_NONE;
            uint8_t LimitChecksEnable;

            if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
                Status = ERROR_INVALID_PARAMS;
            } else {

                LimitChecksEnable = CurrentParameters.LimitChecksEnable[LimitCheckId];

                if (LimitChecksEnable == 0) {

                    CurrentParameters.LimitChecksValue[LimitCheckId] = LimitCheckValue;

                } else {

                    Status = set_limit_value(LimitCheckId, LimitCheckValue);

                    if (Status == ERROR_NONE) {

                        CurrentParameters.LimitChecksValue[LimitCheckId] = LimitCheckValue; 
                    }
                }
            }

            return Status;
        }

        error_t set_limit_check_enable( uint16_t LimitCheckId,
                uint8_t LimitCheckEnable)
        {
            error_t Status = ERROR_NONE;
            FixedPoint1616_t TempFix1616 = 0;

            if (LimitCheckId >= CHECKENABLE_NUMBER_OF_CHECKS) {
                Status = ERROR_INVALID_PARAMS;
            } else {
                /* TempFix1616 contains either 0 or the limit value */
                if (LimitCheckEnable == 0) {
                    TempFix1616 = 0;
                }
                else {
                    TempFix1616 = CurrentParameters.LimitChecksValue[LimitCheckId];
                }

                Status = set_limit_value(LimitCheckId, TempFix1616); }

            if (Status == ERROR_NONE) {

                CurrentParameters.LimitChecksEnable[LimitCheckId] = 
                    LimitCheckEnable == 0 ? 0 : 1;

            }

            return Status;
        }

        error_t low_power_auto_data_init(void)
        {
            error_t  status = ERROR_NONE;

            _low_power_auto_data.vhv_loop_bound =
                TUNINGPARM_LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT;
            _low_power_auto_data.is_low_power_auto_mode = 0;
            _low_power_auto_data.low_power_auto_range_count = 0;
            _low_power_auto_data.saved_interrupt_config = 0;
            _low_power_auto_data.saved_vhv_init = 0;
            _low_power_auto_data.saved_vhv_timeout = 0;
            _low_power_auto_data.first_run_phasecal_result = 0;
            _low_power_auto_data.dss_total_rate_per_spad_mcps = 0;
            _low_power_auto_data.dss_required_spads = 0;

            return status;
        }

        error_t set_vhv_loopbound(uint8_t vhv_loopbound) 
        {
            error_t  status = ERROR_NONE;

            _stat_nvm.vhv_config_timeout_macrop_loop_bound =
                (_stat_nvm.vhv_config_timeout_macrop_loop_bound & 0x03) +
                (vhv_loopbound * 4);

            return status;
        }

        static void decode_zone_size(
                uint8_t  encoded_xy_size,
                uint8_t  *pwidth,
                uint8_t  *pheight)
        {
            *pheight = encoded_xy_size >> 4;
            *pwidth  = encoded_xy_size & 0x0F;

        }

        error_t get_user_zone(user_zone_t * puser_zone)
        {
            error_t  status = ERROR_NONE;

            decode_row_col(
                    _dyn_cfg.roi_config_user_roi_centre_spad,
                    &(puser_zone->y_centre),
                    &(puser_zone->x_centre));

            decode_zone_size(
                    _dyn_cfg.roi_config_user_roi_requested_global_xy_size,
                    &(puser_zone->width),
                    &(puser_zone->height));

            return status;
        }

        error_t set_user_zone(
                user_zone_t     *puser_zone)
        {
            /**
             * Convenience function for setting the user ROI
             */

            error_t  status = ERROR_NONE;

            /* convert (row,col) location into a SPAD number */
            encode_row_col(
                    puser_zone->y_centre,
                    puser_zone->x_centre,
                    &(_dyn_cfg.roi_config_user_roi_centre_spad));

            /* merge x and y sizes */
            encode_zone_size(
                    puser_zone->width,
                    puser_zone->height,
                    &(_dyn_cfg.roi_config_user_roi_requested_global_xy_size));

            /* need to add checks to ensure ROI is within array */


            return status;
        }

        error_t get_mode_mitigation_roi(
                user_zone_t     *pmm_roi)
        {
            /**
             * Convenience function for getting the mode mitigation ROI
             */

            error_t  status = ERROR_NONE;

            uint8_t  x       = 0;
            uint8_t  y       = 0;
            uint8_t  xy_size = 0;


            decode_row_col(
                    _nvm_copy_data.roi_config_mode_roi_centre_spad,
                    &y,
                    &x);

            pmm_roi->x_centre = x;
            pmm_roi->y_centre = y;

            xy_size = _nvm_copy_data.roi_config_mode_roi_xy_size;

            pmm_roi->height = xy_size >> 4;
            pmm_roi->width  = xy_size & 0x0F;


            return status;
        }

}; // class VL53L1X_Abstract
