// VL53L0X control — ported to the ESP-IDF "new" I2C master driver (driver/i2c_master.h)
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
// Based on https://github.com/pololu/vl53l0x-arduino
//
// Changes vs the legacy `driver/i2c.h` version:
//  - Uses i2c_master_bus_handle_t / i2c_master_dev_handle_t instead of the deprecated
//    legacy I2C driver (which is removed in newer ESP-IDF releases).
//  - Can either create+own its own I2C bus, or attach to a bus you already created and
//    share with other devices (recommended when you have several sensors on one bus,
//    e.g. several VL53L0X with XSHUT-assigned addresses).
//  - Every register access can fail (bus errors, NACK, timeout, device unplugged...).
//    The low level accessors now return esp_err_t instead of silently returning 0/garbage
//    on failure, and every higher-level function that used to "return a value" now
//    returns a `const char *` error string (NULL = success) and writes the result via an
//    out-parameter, so callers can actually tell a real reading apart from a failed one.

#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

typedef struct vl53l0x_s vl53l0x_t;

typedef enum
{ VcselPeriodPreRange, VcselPeriodFinalRange } vl53l0x_vcselPeriodType;

// ---- Setup / teardown -----------------------------------------------------

// Create the sensor handle and bring the device up on I2C.
//
// `bus`: pass an existing i2c_master_bus_handle_t to attach this sensor as one more
//        device on a bus you already created (e.g. shared with other VL53L0X or other
//        peripherals) — `port`/`scl`/`sda` are ignored in that case.
//        Pass NULL to have the library create and own a new bus on `port`/`scl`/`sda`;
//        it will be deleted automatically by vl53l0x_end().
// `xshut`: GPIO driving the sensor's XSHUT pin, or -1 if not connected (useful to bring
//          sensors up one at a time so each can be given a unique address).
// `address`: 7-bit I2C address to use (0x29 is the power-on default).
// `io_2v8`: set for 2V8 I/O mode (most breakout boards), clear for native 1V8.
//
// Returns NULL if the bus/device could not be set up (bad pins, OOM, etc). This does
// NOT talk to the sensor yet — call vl53l0x_init() next.
vl53l0x_t *vl53l0x_config (i2c_master_bus_handle_t bus, int8_t port, int8_t scl, int8_t sda, int8_t xshut, uint8_t address, uint8_t io_2v8);

// Bring the sensor out of reset (if xshut is wired) and run the ST init/calibration
// sequence. Returns NULL on success, or a short description of what failed.
const char *vl53l0x_init (vl53l0x_t * v);

// Release the I2C device (and the bus too, if this handle created it) and free `v`.
void vl53l0x_end (vl53l0x_t * v);

// ---- Address / low level register access -----------------------------------

// Change the device's I2C address (e.g. after bringing sensors up one at a time via
// XSHUT). Returns NULL on success and updates the handle to use the new address, or an
// error string on I2C failure (in which case the handle still uses the old address).
const char *vl53l0x_setAddress (vl53l0x_t * v, uint8_t new_addr);
uint8_t vl53l0x_getAddress (vl53l0x_t * v);

esp_err_t vl53l0x_writeReg8Bit (vl53l0x_t * v, uint8_t reg, uint8_t value);
esp_err_t vl53l0x_writeReg16Bit (vl53l0x_t * v, uint8_t reg, uint16_t value);
esp_err_t vl53l0x_writeReg32Bit (vl53l0x_t * v, uint8_t reg, uint32_t value);
esp_err_t vl53l0x_readReg8Bit (vl53l0x_t * v, uint8_t reg, uint8_t * value);
esp_err_t vl53l0x_readReg16Bit (vl53l0x_t * v, uint8_t reg, uint16_t * value);
esp_err_t vl53l0x_readReg32Bit (vl53l0x_t * v, uint8_t reg, uint32_t * value);

// count is capped at 32 bytes (see VL53L0X_I2C_MAX_MULTI in vl53l0x.c) — plenty for
// everything this sensor needs (SPAD map is 6 bytes).
esp_err_t vl53l0x_writeMulti (vl53l0x_t * v, uint8_t reg, uint8_t const *src, size_t count);
esp_err_t vl53l0x_readMulti (vl53l0x_t * v, uint8_t reg, uint8_t * dst, size_t count);

// ---- Tuning -----------------------------------------------------------------

const char *vl53l0x_setSignalRateLimit (vl53l0x_t * v, float limit_Mcps);
const char *vl53l0x_getSignalRateLimit (vl53l0x_t * v, float *limit_Mcps);

const char *vl53l0x_setMeasurementTimingBudget (vl53l0x_t * v, uint32_t budget_us);
const char *vl53l0x_getMeasurementTimingBudget (vl53l0x_t * v, uint32_t *budget_us);

const char *vl53l0x_setVcselPulsePeriod (vl53l0x_t * v, vl53l0x_vcselPeriodType type, uint8_t period_pclks);
const char *vl53l0x_getVcselPulsePeriod (vl53l0x_t * v, vl53l0x_vcselPeriodType type, uint8_t *period_pclks);

// ---- Ranging ------------------------------------------------------------------

const char *vl53l0x_startContinuous (vl53l0x_t * v, uint32_t period_ms);
const char *vl53l0x_stopContinuous (vl53l0x_t * v);
// range_mm receives the reading; the return value tells you whether it's trustworthy.
const char *vl53l0x_readRangeContinuousMillimeters (vl53l0x_t * v, uint16_t *range_mm);
const char *vl53l0x_readRangeSingleMillimeters (vl53l0x_t * v, uint16_t *range_mm);

// ---- Misc ------------------------------------------------------------------

// Timeout (ms) for the internal poll loops (ranging wait, calibration wait, SPAD wait).
// 0 disables the timeout (waits forever). Does NOT affect the I2C bus timeout itself.
void vl53l0x_setTimeout (vl53l0x_t * v, uint16_t timeout_ms);
uint16_t vl53l0x_getTimeout (vl53l0x_t * v);
// Returns nonzero if a poll timeout happened since the last call, and clears the flag.
int vl53l0x_timeoutOccurred (vl53l0x_t * v);
// Returns nonzero if an I2C bus error happened since the last call, and clears the flag.
int vl53l0x_i2cFail (vl53l0x_t * v);

#endif