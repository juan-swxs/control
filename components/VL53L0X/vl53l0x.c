// VL53L0X control — ported to the ESP-IDF "new" I2C master driver (driver/i2c_master.h)
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
// Based on https://github.com/pololu/vl53l0x-arduino
static const char __attribute__((unused)) TAG[] = "ranger";

#include "vl53l0x.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>

#ifdef	CONFIG_VL53L0X_DEBUG
#define VL53L0X_LOG   ESP_LOGI        // Set to allow I2C logging
#else
#define VL53L0X_LOG(tag,...) do {} while (0)
#endif

#define I2C_TIMEOUT_MS 1000    // Bus-level timeout for a single transaction
#define I2C_MAX_MULTI  32      // Max bytes for writeMulti/readMulti (SPAD map needs 6)

enum
{
   SYSRANGE_START = 0x00,

   SYSTEM_THRESH_HIGH = 0x0C,
   SYSTEM_THRESH_LOW = 0x0E,

   SYSTEM_SEQUENCE_CONFIG = 0x01,
   SYSTEM_RANGE_CONFIG = 0x09,
   SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

   SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

   GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

   SYSTEM_INTERRUPT_CLEAR = 0x0B,

   RESULT_INTERRUPT_STATUS = 0x13,
   RESULT_RANGE_STATUS = 0x14,

   RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
   RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
   RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
   RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
   RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

   ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

   MSRC_CONFIG_CONTROL = 0x60,

   PRE_RANGE_CONFIG_MIN_SNR = 0x27,
   PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
   PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
   PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

   FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
   FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
   FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
   FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

   PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
   PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

   PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
   PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
   PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

   SYSTEM_HISTOGRAM_BIN = 0x81,
   HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
   HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

   FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
   FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
   FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
   CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

   MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

   I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

   SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
   IDENTIFICATION_MODEL_ID = 0xC0,
   IDENTIFICATION_REVISION_ID = 0xC2,

   OSC_CALIBRATE_VAL = 0xF8,

   GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

   GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
   DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
   DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
   POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

   VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

   ALGO_PHASECAL_LIM = 0x30,
   ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
};

struct vl53l0x_s
{
   i2c_master_bus_handle_t bus;
   i2c_master_dev_handle_t dev;
   uint8_t owns_bus:1;          // did we create `bus`? if so, delete it in vl53l0x_end()
   uint8_t address;
   int8_t xshut;
   uint16_t io_timeout;
   uint8_t io_2v8:1;
   uint8_t did_timeout:1;
   uint8_t i2c_fail:1;
   uint8_t stop_variable;
   uint32_t measurement_timing_budget_us;
};

typedef struct
{
   uint8_t tcc:1;
   uint8_t msrc:1;
   uint8_t dss:1;
   uint8_t pre_range:1;
   uint8_t final_range:1;
} SequenceStepEnables;

typedef struct
{
   uint16_t pre_range_vcsel_period_pclks,
     final_range_vcsel_period_pclks;
   uint16_t msrc_dss_tcc_mclks,
     pre_range_mclks,
     final_range_mclks;
   uint32_t msrc_dss_tcc_us,
     pre_range_us,
     final_range_us;
}
SequenceStepTimeouts;

#define millis() (esp_timer_get_time()/1000LL)

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (v->io_timeout > 0 && ((uint16_t)(millis() - timeout_start_ms)) > v->io_timeout)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Bail out of the calling function with an error string if `x` (an esp_err_t
// expression) is not ESP_OK. Marks the handle as having had an I2C failure and logs it.
#define CHK(x) do { esp_err_t _err = (x); if (_err != ESP_OK) { v->i2c_fail = 1; ESP_LOGE (TAG, "%s:%d I2C error: %s", __func__, __LINE__, esp_err_to_name (_err)); return "I2C error"; } } while (0)

// ---------------------------------------------------------------------------
// Low level register access
// ---------------------------------------------------------------------------

esp_err_t
vl53l0x_writeReg8Bit (vl53l0x_t * v, uint8_t reg, uint8_t val)
{
   uint8_t buf[2] = { reg, val };
   esp_err_t err = i2c_master_transmit (v->dev, buf, sizeof (buf), I2C_TIMEOUT_MS);
   if (err)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "W %02X=%02X %s", reg, val, esp_err_to_name (err));
   return err;
}

esp_err_t
vl53l0x_writeReg16Bit (vl53l0x_t * v, uint8_t reg, uint16_t val)
{
   uint8_t buf[3] = { reg, val >> 8, val };
   esp_err_t err = i2c_master_transmit (v->dev, buf, sizeof (buf), I2C_TIMEOUT_MS);
   if (err)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "W %02X=%04X %s", reg, val, esp_err_to_name (err));
   return err;
}

esp_err_t
vl53l0x_writeReg32Bit (vl53l0x_t * v, uint8_t reg, uint32_t val)
{
   uint8_t buf[5] = { reg, val >> 24, val >> 16, val >> 8, val };
   esp_err_t err = i2c_master_transmit (v->dev, buf, sizeof (buf), I2C_TIMEOUT_MS);
   if (err)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "W %02X=%08X %s", reg, val, esp_err_to_name (err));
   return err;
}

esp_err_t
vl53l0x_readReg8Bit (vl53l0x_t * v, uint8_t reg, uint8_t * value)
{
   esp_err_t err = i2c_master_transmit_receive (v->dev, &reg, 1, value, 1, I2C_TIMEOUT_MS);
   if (err)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "R %02X=%02X %s", reg, err ? 0 : *value, esp_err_to_name (err));
   return err;
}

esp_err_t
vl53l0x_readReg16Bit (vl53l0x_t * v, uint8_t reg, uint16_t * value)
{
   uint8_t buf[2] = { };
   esp_err_t err = i2c_master_transmit_receive (v->dev, &reg, 1, buf, sizeof (buf), I2C_TIMEOUT_MS);
   if (err)
      v->i2c_fail = 1;
   else
      *value = (buf[0] << 8) + buf[1];
   VL53L0X_LOG (TAG, "R %02X=%04X %s", reg, err ? 0 : *value, esp_err_to_name (err));
   return err;
}

esp_err_t
vl53l0x_readReg32Bit (vl53l0x_t * v, uint8_t reg, uint32_t * value)
{
   uint8_t buf[4] = { };
   esp_err_t err = i2c_master_transmit_receive (v->dev, &reg, 1, buf, sizeof (buf), I2C_TIMEOUT_MS);
   if (err)
      v->i2c_fail = 1;
   else
      *value = ((uint32_t) buf[0] << 24) + ((uint32_t) buf[1] << 16) + (buf[2] << 8) + buf[3];
   VL53L0X_LOG (TAG, "R %02X=%08X %s", reg, err ? 0 : *value, esp_err_to_name (err));
   return err;
}

esp_err_t
vl53l0x_readMulti (vl53l0x_t * v, uint8_t reg, uint8_t * dst, size_t count)
{
   if (!count)
      return ESP_OK;
   esp_err_t err = i2c_master_transmit_receive (v->dev, &reg, 1, dst, count, I2C_TIMEOUT_MS);
   if (err)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "R %02X (%d) %s", reg, (int) count, esp_err_to_name (err));
   return err;
}

esp_err_t
vl53l0x_writeMulti (vl53l0x_t * v, uint8_t reg, uint8_t const *src, size_t count)
{
   if (count > I2C_MAX_MULTI)
      return ESP_ERR_INVALID_SIZE;
   uint8_t buf[I2C_MAX_MULTI + 1];
   buf[0] = reg;
   memcpy (buf + 1, src, count);
   esp_err_t err = i2c_master_transmit (v->dev, buf, count + 1, I2C_TIMEOUT_MS);
   if (err)
      v->i2c_fail = 1;
   VL53L0X_LOG (TAG, "W %02X (%d) %s", reg, (int) count, esp_err_to_name (err));
   return err;
}

// ---------------------------------------------------------------------------
// Internal helpers (timing budget maths etc — unchanged from the reference
// implementation, just adapted to the esp_err_t-returning accessors above)
// ---------------------------------------------------------------------------

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value — based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
static uint16_t
decodeTimeout (uint16_t reg_val)
{
   // format: "(LSByte * 2^MSByte) + 1"
   return (uint16_t) ((reg_val & 0x00FF) << (uint16_t) ((reg_val & 0xFF00) >> 8)) + 1;
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
static uint32_t
timeoutMclksToMicroseconds (uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
   uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);
   return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t
timeoutMicrosecondsToMclks (uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
   uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);
   return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
static uint16_t
encodeTimeout (uint16_t timeout_mclks)
{
   // format: "(LSByte * 2^MSByte) + 1"
   uint32_t ls_byte = 0;
   uint16_t ms_byte = 0;

   if (timeout_mclks > 0)
   {
      ls_byte = timeout_mclks - 1;
      while ((ls_byte & 0xFFFFFF00) > 0)
      {
         ls_byte >>= 1;
         ms_byte++;
      }
      return (ms_byte << 8) | (ls_byte & 0xFF);
   }
   return 0;
}

// based on VL53L0X_perform_single_ref_calibration()
static const char *
performSingleRefCalibration (vl53l0x_t * v, uint8_t vhv_init_byte)
{
   uint16_t timeout_start_ms;
   uint8_t status;

   CHK (vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x01 | vhv_init_byte));      // VL53L0X_REG_SYSRANGE_MODE_START_STOP
   startTimeout ();
   for (;;)
   {
      CHK (vl53l0x_readReg8Bit (v, RESULT_INTERRUPT_STATUS, &status));
      if (status & 0x07)
         break;
      if (checkTimeoutExpired ())
      {
         v->did_timeout = 1;
         return "CAL timeout";
      }
   }
   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CLEAR, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x00));
   return NULL;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
const char *
vl53l0x_getVcselPulsePeriod (vl53l0x_t * v, vl53l0x_vcselPeriodType type, uint8_t * period_pclks)
{
   uint8_t reg;
   if (type == VcselPeriodPreRange)
      CHK (vl53l0x_readReg8Bit (v, PRE_RANGE_CONFIG_VCSEL_PERIOD, &reg));
   else if (type == VcselPeriodFinalRange)
      CHK (vl53l0x_readReg8Bit (v, FINAL_RANGE_CONFIG_VCSEL_PERIOD, &reg));
   else
      return "Invalid type";
   *period_pclks = decodeVcselPeriod (reg);
   return NULL;
}

// Get sequence step enables — based on VL53L0X_GetSequenceStepEnables()
static const char *
getSequenceStepEnables (vl53l0x_t * v, SequenceStepEnables * enables)
{
   uint8_t sequence_config;
   CHK (vl53l0x_readReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, &sequence_config));
   enables->tcc = (sequence_config >> 4) & 0x1;
   enables->dss = (sequence_config >> 3) & 0x1;
   enables->msrc = (sequence_config >> 2) & 0x1;
   enables->pre_range = (sequence_config >> 6) & 0x1;
   enables->final_range = (sequence_config >> 7) & 0x1;
   return NULL;
}

// Get sequence step timeouts — based on get_sequence_step_timeout(), but gets all
// timeouts instead of just the requested one, and also stores intermediate values
static const char *
getSequenceStepTimeouts (vl53l0x_t * v, SequenceStepEnables const *enables, SequenceStepTimeouts * timeouts)
{
   const char *err;
   uint8_t reg8;
   uint16_t reg16;

   if ((err = vl53l0x_getVcselPulsePeriod (v, VcselPeriodPreRange, &reg8)))
      return err;
   timeouts->pre_range_vcsel_period_pclks = reg8;

   CHK (vl53l0x_readReg8Bit (v, MSRC_CONFIG_TIMEOUT_MACROP, &reg8));
   timeouts->msrc_dss_tcc_mclks = reg8 + 1;
   timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds (timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

   CHK (vl53l0x_readReg16Bit (v, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &reg16));
   timeouts->pre_range_mclks = decodeTimeout (reg16);
   timeouts->pre_range_us = timeoutMclksToMicroseconds (timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

   if ((err = vl53l0x_getVcselPulsePeriod (v, VcselPeriodFinalRange, &reg8)))
      return err;
   timeouts->final_range_vcsel_period_pclks = reg8;

   CHK (vl53l0x_readReg16Bit (v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &reg16));
   timeouts->final_range_mclks = decodeTimeout (reg16);

   if (enables->pre_range)
      timeouts->final_range_mclks -= timeouts->pre_range_mclks;

   timeouts->final_range_us = timeoutMclksToMicroseconds (timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
   return NULL;
}

// Set the return signal rate limit check value in units of MCPS (mega counts per
// second). "This represents the amplitude of the signal reflected from the target and
// detected by the device"; setting this limit presumably determines the minimum
// measurement necessary for the sensor to report a valid reading. Setting a lower limit
// increases the potential range of the sensor but also seems to increase the likelihood
// of getting an inaccurate reading because of unwanted reflections from objects other
// than the intended target. Defaults to 0.25 MCPS as initialized by the ST API.
const char *
vl53l0x_setSignalRateLimit (vl53l0x_t * v, float limit_Mcps)
{
   if (limit_Mcps < 0 || limit_Mcps > 511.99)
      return "Bad rate";
   // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
   CHK (vl53l0x_writeReg16Bit (v, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t) (limit_Mcps * (1 << 7))));
   return NULL;
}

const char *
vl53l0x_getSignalRateLimit (vl53l0x_t * v, float *limit_Mcps)
{
   uint16_t reg;
   CHK (vl53l0x_readReg16Bit (v, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &reg));
   *limit_Mcps = reg / (float) (1 << 7);
   return NULL;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(), but only gets reference SPAD count and type
static const char *
getSpadInfo (vl53l0x_t * v, uint8_t * count, int *type_is_aperture)
{
   uint16_t timeout_start_ms;
   uint8_t tmp;

   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x00));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x06));
   CHK (vl53l0x_readReg8Bit (v, 0x83, &tmp));
   CHK (vl53l0x_writeReg8Bit (v, 0x83, tmp | 0x04));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x07));
   CHK (vl53l0x_writeReg8Bit (v, 0x81, 0x01));

   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x01));

   CHK (vl53l0x_writeReg8Bit (v, 0x94, 0x6b));
   CHK (vl53l0x_writeReg8Bit (v, 0x83, 0x00));
   startTimeout ();
   for (;;)
   {
      CHK (vl53l0x_readReg8Bit (v, 0x83, &tmp));
      if (tmp != 0x00)
         break;
      if (checkTimeoutExpired ())
      {
         v->did_timeout = 1;
         return "SPAD timeout";
      }
   }
   CHK (vl53l0x_writeReg8Bit (v, 0x83, 0x01));
   CHK (vl53l0x_readReg8Bit (v, 0x92, &tmp));

   *count = tmp & 0x7f;
   *type_is_aperture = (tmp >> 7) & 0x01;

   CHK (vl53l0x_writeReg8Bit (v, 0x81, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x06));
   CHK (vl53l0x_readReg8Bit (v, 0x83, &tmp));
   CHK (vl53l0x_writeReg8Bit (v, 0x83, tmp & ~0x04));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x01));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x00));

   return NULL;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
const char *
vl53l0x_getMeasurementTimingBudget (vl53l0x_t * v, uint32_t *budget_us)
{
   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;
   const char *err;

   uint16_t const StartOverhead = 1910; // note that this is different than the value in set_
   uint16_t const EndOverhead = 960;
   uint16_t const MsrcOverhead = 660;
   uint16_t const TccOverhead = 590;
   uint16_t const DssOverhead = 690;
   uint16_t const PreRangeOverhead = 660;
   uint16_t const FinalRangeOverhead = 550;

   // "Start and end overhead times always present"
   uint32_t budget = StartOverhead + EndOverhead;

   if ((err = getSequenceStepEnables (v, &enables)))
      return err;
   if ((err = getSequenceStepTimeouts (v, &enables, &timeouts)))
      return err;

   if (enables.tcc)
      budget += (timeouts.msrc_dss_tcc_us + TccOverhead);

   if (enables.dss)
      budget += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
   else if (enables.msrc)
      budget += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

   if (enables.pre_range)
      budget += (timeouts.pre_range_us + PreRangeOverhead);

   if (enables.final_range)
      budget += (timeouts.final_range_us + FinalRangeOverhead);

   v->measurement_timing_budget_us = budget;    // store for internal reuse
   *budget_us = budget;
   return NULL;
}

// Set the measurement timing budget in microseconds, which is the time allowed for one
// measurement; the ST API and this library take care of splitting the timing budget
// among the sub-steps in the ranging sequence. A longer timing budget allows for more
// accurate measurements. Increasing the budget by a factor of N decreases the range
// measurement standard deviation by a factor of sqrt(N). Defaults to about 33
// milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
const char *
vl53l0x_setMeasurementTimingBudget (vl53l0x_t * v, uint32_t budget_us)
{
   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;
   const char *err;

   uint16_t const StartOverhead = 1320; // note that this is different than the value in get_
   uint16_t const EndOverhead = 960;
   uint16_t const MsrcOverhead = 660;
   uint16_t const TccOverhead = 590;
   uint16_t const DssOverhead = 690;
   uint16_t const PreRangeOverhead = 660;
   uint16_t const FinalRangeOverhead = 550;

   uint32_t const MinTimingBudget = 20000;

   if (budget_us < MinTimingBudget)
      return "Low budget";

   uint32_t used_budget_us = StartOverhead + EndOverhead;

   if ((err = getSequenceStepEnables (v, &enables)))
      return err;
   if ((err = getSequenceStepTimeouts (v, &enables, &timeouts)))
      return err;

   if (enables.tcc)
      used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

   if (enables.dss)
      used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
   else if (enables.msrc)
      used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

   if (enables.pre_range)
      used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);

   if (enables.final_range)
   {
      used_budget_us += FinalRangeOverhead;

      // "Note that the final range timeout is determined by the timing budget and the
      // sum of all other timeouts within the sequence. If there is no room for the
      // final range timeout, then an error will be set. Otherwise the remaining time
      // will be applied to the final range."
      if (used_budget_us > budget_us)
         return "High budget";  // "Requested timeout too big."

      uint32_t final_range_timeout_us = budget_us - used_budget_us;

      // set_sequence_step_timeout() begin (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)
      // "For the final range timeout, the pre-range timeout must be added. To do this
      // both final and pre-range timeouts must be expressed in macro periods MClks
      // because they have different vcsel periods."
      uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks (final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

      if (enables.pre_range)
         final_range_timeout_mclks += timeouts.pre_range_mclks;

      CHK (vl53l0x_writeReg16Bit (v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout (final_range_timeout_mclks)));
      // set_sequence_step_timeout() end

      v->measurement_timing_budget_us = budget_us; // store for internal reuse
   }
   return NULL;
}

// ---------------------------------------------------------------------------
// Setup / teardown
// ---------------------------------------------------------------------------

vl53l0x_t *
vl53l0x_config (i2c_master_bus_handle_t bus, int8_t port, int8_t scl, int8_t sda, int8_t xshut, uint8_t address, uint8_t io_2v8)
{
   if (xshut >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO (xshut))
      return NULL;

   bool owns_bus = false;
   if (!bus)
   {                            // Create and own our own bus
      if (port < 0 || scl < 0 || sda < 0 || scl == sda)
         return NULL;
      if (!GPIO_IS_VALID_OUTPUT_GPIO (scl) || !GPIO_IS_VALID_OUTPUT_GPIO (sda))
         return NULL;
      i2c_master_bus_config_t bus_config = {
         .i2c_port = port,
         .sda_io_num = sda,
         .scl_io_num = scl,
         .clk_source = I2C_CLK_SRC_DEFAULT,
         .glitch_ignore_cnt = 7,
         .flags.enable_internal_pullup = true,
      };
      if (i2c_new_master_bus (&bus_config, &bus) != ESP_OK)
         return NULL;
      owns_bus = true;
   }

   i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = address,
      .scl_speed_hz = 100000,
   };
   i2c_master_dev_handle_t dev;
   if (i2c_master_bus_add_device (bus, &dev_config, &dev) != ESP_OK)
   {
      if (owns_bus)
         i2c_del_master_bus (bus);
      return NULL;
   }

   if (xshut >= 0)
   {
      gpio_reset_pin (xshut);
      gpio_set_level (xshut, 0);        // Off
      gpio_set_drive_capability (xshut, GPIO_DRIVE_CAP_3);
      gpio_set_direction (xshut, GPIO_MODE_OUTPUT);
   }

   vl53l0x_t *v = malloc (sizeof (*v));
   if (!v)
   {
      i2c_master_bus_rm_device (dev);
      if (owns_bus)
         i2c_del_master_bus (bus);
      return NULL;
   }
   memset (v, 0, sizeof (*v));
   v->bus = bus;
   v->dev = dev;
   v->owns_bus = owns_bus;
   v->xshut = xshut;
   v->io_2v8 = io_2v8;
   v->address = address;
   v->io_timeout = 100;
   return v;
}

void
vl53l0x_end (vl53l0x_t * v)
{
   if (!v)
      return;
   i2c_master_bus_rm_device (v->dev);
   if (v->owns_bus)
      i2c_del_master_bus (v->bus);
   free (v);
}

// Initialize sensor using sequence based on VL53L0X_DataInit(), VL53L0X_StaticInit(),
// and VL53L0X_PerformRefCalibration(). This function does not perform reference SPAD
// calibration (VL53L0X_PerformRefSpadManagement()), since the API user manual says that
// it is performed by ST on the bare modules; it seems like that should work well enough
// unless a cover glass is added.
const char *
vl53l0x_init (vl53l0x_t * v)
{
   const char *err;
   uint8_t tmp;

   if (v->xshut >= 0)
   {                            // XSHUT or power control
      gpio_set_level (v->xshut, 0);     // Off
      usleep (100000);
      gpio_set_level (v->xshut, 1);     // On
      usleep (10000);           // Plenty of time to boot (data sheet says 1.2ms)
   }

   // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
   if (v->io_2v8)
   {
      CHK (vl53l0x_readReg8Bit (v, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &tmp));
      CHK (vl53l0x_writeReg8Bit (v, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, tmp | 0x01));   // set bit 0
   }
   // "Set I2C standard mode"
   CHK (vl53l0x_writeReg8Bit (v, 0x88, 0x00));

   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x00));
   CHK (vl53l0x_readReg8Bit (v, 0x91, &v->stop_variable));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x00));

   // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
   CHK (vl53l0x_readReg8Bit (v, MSRC_CONFIG_CONTROL, &tmp));
   CHK (vl53l0x_writeReg8Bit (v, MSRC_CONFIG_CONTROL, tmp | 0x12));

   // set final range signal rate limit to 0.25 MCPS (million counts per second)
   if ((err = vl53l0x_setSignalRateLimit (v, 0.25)))
      return err;

   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0xFF));

   // VL53L0X_DataInit() end
   // VL53L0X_StaticInit() begin

   uint8_t spad_count;
   int spad_type_is_aperture;
   if ((err = getSpadInfo (v, &spad_count, &spad_type_is_aperture)))
      return err;

   // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in the
   // API, but the same data seems to be more easily readable from
   // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
   uint8_t ref_spad_map[6];
   CHK (vl53l0x_readMulti (v, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6));

   // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4));

   uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;       // 12 is the first aperture spad
   uint8_t spads_enabled = 0;

   for (uint8_t i = 0; i < 48; i++)
   {
      if (i < first_spad_to_enable || spads_enabled == spad_count)
      {
         // This bit is lower than the first one that should be enabled, or
         // (reference_spad_count) bits have already been enabled, so zero this bit
         ref_spad_map[i / 8] &= ~(1 << (i % 8));
      } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
         spads_enabled++;
   }

   CHK (vl53l0x_writeMulti (v, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6));
   // -- VL53L0X_set_reference_spads() end

   // -- VL53L0X_load_tuning_settings() begin — DefaultTuningSettings from vl53l0x_tuning.h
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x00));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x09, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x10, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x11, 0x00));

   CHK (vl53l0x_writeReg8Bit (v, 0x24, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x25, 0xFF));
   CHK (vl53l0x_writeReg8Bit (v, 0x75, 0x00));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x4E, 0x2C));
   CHK (vl53l0x_writeReg8Bit (v, 0x48, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x30, 0x20));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x30, 0x09));
   CHK (vl53l0x_writeReg8Bit (v, 0x54, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x31, 0x04));
   CHK (vl53l0x_writeReg8Bit (v, 0x32, 0x03));
   CHK (vl53l0x_writeReg8Bit (v, 0x40, 0x83));
   CHK (vl53l0x_writeReg8Bit (v, 0x46, 0x25));
   CHK (vl53l0x_writeReg8Bit (v, 0x60, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x27, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x50, 0x06));
   CHK (vl53l0x_writeReg8Bit (v, 0x51, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x52, 0x96));
   CHK (vl53l0x_writeReg8Bit (v, 0x56, 0x08));
   CHK (vl53l0x_writeReg8Bit (v, 0x57, 0x30));
   CHK (vl53l0x_writeReg8Bit (v, 0x61, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x62, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x64, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x65, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x66, 0xA0));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x22, 0x32));
   CHK (vl53l0x_writeReg8Bit (v, 0x47, 0x14));
   CHK (vl53l0x_writeReg8Bit (v, 0x49, 0xFF));
   CHK (vl53l0x_writeReg8Bit (v, 0x4A, 0x00));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x7A, 0x0A));
   CHK (vl53l0x_writeReg8Bit (v, 0x7B, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x78, 0x21));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x23, 0x34));
   CHK (vl53l0x_writeReg8Bit (v, 0x42, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x44, 0xFF));
   CHK (vl53l0x_writeReg8Bit (v, 0x45, 0x26));
   CHK (vl53l0x_writeReg8Bit (v, 0x46, 0x05));
   CHK (vl53l0x_writeReg8Bit (v, 0x40, 0x40));
   CHK (vl53l0x_writeReg8Bit (v, 0x0E, 0x06));
   CHK (vl53l0x_writeReg8Bit (v, 0x20, 0x1A));
   CHK (vl53l0x_writeReg8Bit (v, 0x43, 0x40));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x34, 0x03));
   CHK (vl53l0x_writeReg8Bit (v, 0x35, 0x44));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x31, 0x04));
   CHK (vl53l0x_writeReg8Bit (v, 0x4B, 0x09));
   CHK (vl53l0x_writeReg8Bit (v, 0x4C, 0x05));
   CHK (vl53l0x_writeReg8Bit (v, 0x4D, 0x04));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x44, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x45, 0x20));
   CHK (vl53l0x_writeReg8Bit (v, 0x47, 0x08));
   CHK (vl53l0x_writeReg8Bit (v, 0x48, 0x28));
   CHK (vl53l0x_writeReg8Bit (v, 0x67, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x70, 0x04));
   CHK (vl53l0x_writeReg8Bit (v, 0x71, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x72, 0xFE));
   CHK (vl53l0x_writeReg8Bit (v, 0x76, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x77, 0x00));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x0D, 0x01));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x01, 0xF8));

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x8E, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x00));
   // -- VL53L0X_load_tuning_settings() end

   // "Set interrupt config to new sample ready" -- VL53L0X_SetGpioConfig() begin
   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04));
   CHK (vl53l0x_readReg8Bit (v, GPIO_HV_MUX_ACTIVE_HIGH, &tmp));
   CHK (vl53l0x_writeReg8Bit (v, GPIO_HV_MUX_ACTIVE_HIGH, tmp & ~0x10));       // active low
   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CLEAR, 0x01));
   // -- VL53L0X_SetGpioConfig() end

   if ((err = vl53l0x_getMeasurementTimingBudget (v, &v->measurement_timing_budget_us)))
      return err;

   // "Disable MSRC and TCC by default" (MSRC = Minimum Signal Rate Check, TCC = Target
   // CentreCheck) -- VL53L0X_SetSequenceStepEnable() begin
   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0xE8));
   // -- VL53L0X_SetSequenceStepEnable() end

   // "Recalculate timing budget"
   if ((err = vl53l0x_setMeasurementTimingBudget (v, v->measurement_timing_budget_us)))
      return err;

   // VL53L0X_StaticInit() end
   // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

   // -- VL53L0X_perform_vhv_calibration() begin
   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0x01));
   if ((err = performSingleRefCalibration (v, 0x40)))
      return err;
   // -- VL53L0X_perform_vhv_calibration() end

   // -- VL53L0X_perform_phase_calibration() begin
   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0x02));
   if ((err = performSingleRefCalibration (v, 0x00)))
      return err;
   // -- VL53L0X_perform_phase_calibration() end

   // "restore the previous Sequence Config"
   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0xE8));
   // VL53L0X_PerformRefCalibration() end

   return NULL;
}

// ---------------------------------------------------------------------------
// Address / VCSEL period / ranging
// ---------------------------------------------------------------------------

const char *
vl53l0x_setAddress (vl53l0x_t * v, uint8_t new_addr)
{
   CHK (vl53l0x_writeReg8Bit (v, I2C_SLAVE_DEVICE_ADDRESS, new_addr & 0x7F));

   // The device itself now answers on the new address, so we must rebind our
   // i2c_master_dev_handle_t to match, or every subsequent transaction will target
   // the (now stale) old address.
   i2c_master_dev_handle_t new_dev;
   i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = new_addr & 0x7F,
      .scl_speed_hz = 100000,
   };
   if (i2c_master_bus_add_device (v->bus, &dev_config, &new_dev) != ESP_OK)
      return "Address change failed";
   i2c_master_bus_rm_device (v->dev);
   v->dev = new_dev;
   v->address = new_addr;
   return NULL;
}

uint8_t
vl53l0x_getAddress (vl53l0x_t * v)
{
   return v->address;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the given
// period type (pre-range or final range) to the given value in PCLKs. Longer periods
// seem to increase the potential range of the sensor. Valid values (even numbers only):
//  pre:   12 to 18 (initialized default: 14)
//  final:  8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
const char *
vl53l0x_setVcselPulsePeriod (vl53l0x_t * v, vl53l0x_vcselPeriodType type, uint8_t period_pclks)
{
   uint8_t vcsel_period_reg = encodeVcselPeriod (period_pclks);
   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;
   const char *err;

   if ((err = getSequenceStepEnables (v, &enables)))
      return err;
   if ((err = getSequenceStepTimeouts (v, &enables, &timeouts)))
      return err;

   // "Apply specific settings for the requested clock period" / "Re-calculate and
   // apply timeouts, in macro periods"
   //
   // "When the VCSEL period for the pre or final range is changed, the corresponding
   // timeout must be read from the device using the current VCSEL period, then the new
   // VCSEL period can be applied. The timeout then must be written back to the device
   // using the new VCSEL period. For the MSRC timeout, the same applies - this timeout
   // being dependant on the pre-range vcsel period."

   if (type == VcselPeriodPreRange)
   {
      // "Set phase check limits"
      switch (period_pclks)
      {
      case 12:
         CHK (vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18));
         break;
      case 14:
         CHK (vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30));
         break;
      case 16:
         CHK (vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40));
         break;
      case 18:
         CHK (vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50));
         break;
      default:
         return "Invalid period";
      }
      CHK (vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));

      // apply new VCSEL period
      CHK (vl53l0x_writeReg8Bit (v, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg));

      // update timeouts -- set_sequence_step_timeout() begin (SequenceStepId == PRE_RANGE)
      uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks (timeouts.pre_range_us, period_pclks);
      CHK (vl53l0x_writeReg16Bit (v, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout (new_pre_range_timeout_mclks)));
      // set_sequence_step_timeout() end

      // set_sequence_step_timeout() begin (SequenceStepId == MSRC)
      uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks (timeouts.msrc_dss_tcc_us, period_pclks);
      CHK (vl53l0x_writeReg8Bit (v, MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1)));
      // set_sequence_step_timeout() end
   } else if (type == VcselPeriodFinalRange)
   {
      switch (period_pclks)
      {
      case 8:
         CHK (vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10));
         CHK (vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
         CHK (vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02));
         CHK (vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C));
         CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
         CHK (vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x30));
         CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
         break;
      case 10:
         CHK (vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28));
         CHK (vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
         CHK (vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03));
         CHK (vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09));
         CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
         CHK (vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x20));
         CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
         break;
      case 12:
         CHK (vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38));
         CHK (vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
         CHK (vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03));
         CHK (vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08));
         CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
         CHK (vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x20));
         CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
         break;
      case 14:
         CHK (vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48));
         CHK (vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
         CHK (vl53l0x_writeReg8Bit (v, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03));
         CHK (vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07));
         CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
         CHK (vl53l0x_writeReg8Bit (v, ALGO_PHASECAL_LIM, 0x20));
         CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
         break;
      default:
         return "Invalid period";
      }

      // apply new VCSEL period
      CHK (vl53l0x_writeReg8Bit (v, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg));

      // update timeouts -- set_sequence_step_timeout() begin (SequenceStepId == FINAL_RANGE)
      // "For the final range timeout, the pre-range timeout must be added. To do this
      // both final and pre-range timeouts must be expressed in macro periods MClks
      // because they have different vcsel periods."
      uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks (timeouts.final_range_us, period_pclks);
      if (enables.pre_range)
         new_final_range_timeout_mclks += timeouts.pre_range_mclks;
      CHK (vl53l0x_writeReg16Bit (v, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout (new_final_range_timeout_mclks)));
      // set_sequence_step_timeout end
   } else
      return "Invalid type";

   // "Finally, the timing budget must be re-applied"
   if ((err = vl53l0x_setMeasurementTimingBudget (v, v->measurement_timing_budget_us)))
      return err;

   // "Perform the phase calibration. This is needed after changing on vcsel period."
   // VL53L0X_perform_phase_calibration() begin
   uint8_t sequence_config;
   CHK (vl53l0x_readReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, &sequence_config));
   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, 0x02));
   if ((err = performSingleRefCalibration (v, 0x0)))
      return err;
   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_SEQUENCE_CONFIG, sequence_config));
   // VL53L0X_perform_phase_calibration() end

   return NULL;
}

// Start continuous ranging measurements. If period_ms is 0, continuous back-to-back
// mode is used (the sensor takes measurements as often as possible); otherwise,
// continuous timed mode is used, with the given inter-measurement period in
// milliseconds determining how often the sensor takes a measurement.
// based on VL53L0X_StartMeasurement()
const char *
vl53l0x_startContinuous (vl53l0x_t * v, uint32_t period_ms)
{
   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x91, v->stop_variable));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x00));

   if (period_ms != 0)
   {                            // continuous timed mode
      // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin
      uint16_t osc_calibrate_val;
      CHK (vl53l0x_readReg16Bit (v, OSC_CALIBRATE_VAL, &osc_calibrate_val));
      if (osc_calibrate_val != 0)
         period_ms *= osc_calibrate_val;
      CHK (vl53l0x_writeReg32Bit (v, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms));
      // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
      CHK (vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x04));    // VL53L0X_REG_SYSRANGE_MODE_TIMED
   } else
      CHK (vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x02));    // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK

   return NULL;
}

// Stop continuous measurements — based on VL53L0X_StopMeasurement()
const char *
vl53l0x_stopContinuous (vl53l0x_t * v)
{
   CHK (vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x01));       // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x91, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   return NULL;
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a single-shot
// range measurement)
const char *
vl53l0x_readRangeContinuousMillimeters (vl53l0x_t * v, uint16_t * range_mm)
{
   uint16_t timeout_start_ms;
   uint8_t status;

   startTimeout ();
   for (;;)
   {
      CHK (vl53l0x_readReg8Bit (v, RESULT_INTERRUPT_STATUS, &status));
      if (status & 0x07)
         break;
      if (checkTimeoutExpired ())
      {
         v->did_timeout = 1;
         return "Range timeout";
      }
   }
   // assumptions: Linearity Corrective Gain is 1000 (default); fractional ranging is
   // not enabled
   CHK (vl53l0x_readReg16Bit (v, RESULT_RANGE_STATUS + 10, range_mm));
   CHK (vl53l0x_writeReg8Bit (v, SYSTEM_INTERRUPT_CLEAR, 0x01));
   return NULL;
}

// Performs a single-shot range measurement and returns the reading in millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
const char *
vl53l0x_readRangeSingleMillimeters (vl53l0x_t * v, uint16_t * range_mm)
{
   uint16_t timeout_start_ms;
   uint8_t status;

   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x91, v->stop_variable));
   CHK (vl53l0x_writeReg8Bit (v, 0x00, 0x01));
   CHK (vl53l0x_writeReg8Bit (v, 0xFF, 0x00));
   CHK (vl53l0x_writeReg8Bit (v, 0x80, 0x00));

   CHK (vl53l0x_writeReg8Bit (v, SYSRANGE_START, 0x01));

   // "Wait until start bit has been cleared"
   startTimeout ();
   for (;;)
   {
      CHK (vl53l0x_readReg8Bit (v, SYSRANGE_START, &status));
      if (!(status & 0x01))
         break;
      if (checkTimeoutExpired ())
      {
         v->did_timeout = 1;
         return "Range timeout";
      }
   }

   return vl53l0x_readRangeContinuousMillimeters (v, range_mm);
}

// ---------------------------------------------------------------------------
// Misc
// ---------------------------------------------------------------------------

void
vl53l0x_setTimeout (vl53l0x_t * v, uint16_t new_timeout)
{
   v->io_timeout = new_timeout;
}

uint16_t
vl53l0x_getTimeout (vl53l0x_t * v)
{
   return v->io_timeout;
}

int
vl53l0x_timeoutOccurred (vl53l0x_t * v)
{
   int tmp = v->did_timeout;
   v->did_timeout = 0;
   return tmp;
}

int
vl53l0x_i2cFail (vl53l0x_t * v)
{
   int tmp = v->i2c_fail;
   v->i2c_fail = 0;
   return tmp;
}