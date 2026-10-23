/**
 * @file mcp4725.h
 * @brief Driver para el DAC MCP4725 (12 bits, I2C) con el driver nuevo de ESP-IDF (i2c_master).
 *
 * Sin dependencias externas: solo esp_driver_i2c y FreeRTOS.
 * Todas las funciones son thread-safe por dispositivo (mutex interno).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MCP4725_ADDR_A0_LOW   0x60  /**< Chip desnudo con A0 = GND (módulos Adafruit: 0x62) */
#define MCP4725_ADDR_A0_HIGH  0x61  /**< Chip desnudo con A0 = VCC (módulos Adafruit: 0x63) */
#define MCP4725_MAX_RAW       4095

/** Modo de operación / power-down (resistencia de la salida a GND). */
typedef enum {
    MCP4725_PD_NORMAL = 0,
    MCP4725_PD_1K     = 1,
    MCP4725_PD_100K   = 2,
    MCP4725_PD_500K   = 3,
} mcp4725_pd_t;

/** Configuración del dispositivo. */
typedef struct {
    uint8_t  address;       /**< Dirección I2C de 7 bits */
    uint32_t scl_speed_hz;  /**< 100000, 400000 (o hasta 3.4 MHz según tu bus) */
    float    vref;          /**< Vdd real del MCP4725 (es su referencia) para las funciones en voltios */
    int      timeout_ms;    /**< Timeout por transacción I2C (-1 = infinito) */
} mcp4725_config_t;

#define MCP4725_CONFIG_DEFAULT() { \
    .address      = MCP4725_ADDR_A0_LOW, \
    .scl_speed_hz = 400000, \
    .vref         = 3.3f, \
    .timeout_ms   = 100, \
}

/** Contenido de los registros leídos del chip. */
typedef struct {
    uint16_t     dac_raw;       /**< Valor actual del registro DAC */
    mcp4725_pd_t dac_pd;        /**< Modo actual */
    uint16_t     eeprom_raw;    /**< Valor guardado en EEPROM (se carga al encender) */
    mcp4725_pd_t eeprom_pd;     /**< Modo guardado en EEPROM */
    bool         eeprom_ready;  /**< false mientras hay una escritura de EEPROM en curso */
} mcp4725_status_t;

typedef struct mcp4725_dev *mcp4725_handle_t;

/**
 * @brief Agrega el MCP4725 a un bus creado con i2c_new_master_bus() y verifica que responda.
 * @return ESP_OK, ESP_ERR_NOT_FOUND si no hay ACK, ESP_ERR_NO_MEM, ESP_ERR_INVALID_ARG
 */
esp_err_t mcp4725_create(i2c_master_bus_handle_t bus, const mcp4725_config_t *cfg,
                         mcp4725_handle_t *out);

/** @brief Quita el dispositivo del bus y libera memoria (no toca el bus). */
esp_err_t mcp4725_delete(mcp4725_handle_t dev);

/** @brief Cambia Vdd/referencia usada por las funciones en voltios (útil si la mides). */
esp_err_t mcp4725_set_vref(mcp4725_handle_t dev, float vref);

/**
 * @brief Escribe el registro DAC en modo rápido (2 bytes, modo normal). No toca la EEPROM.
 * @return ESP_ERR_INVALID_ARG si raw > 4095
 */
esp_err_t mcp4725_set_raw(mcp4725_handle_t dev, uint16_t raw);

/** @brief Igual que set_raw pero en voltios (0..vref, redondeo al LSB más cercano). */
esp_err_t mcp4725_set_voltage(mcp4725_handle_t dev, float volts);

/** @brief Lee el registro DAC. */
esp_err_t mcp4725_get_raw(mcp4725_handle_t dev, uint16_t *raw);

/** @brief Lee el registro DAC y lo convierte a voltios con vref. */
esp_err_t mcp4725_get_voltage(mcp4725_handle_t dev, float *volts);

/** @brief Lee DAC, EEPROM, modos y bit de EEPROM lista (5 bytes en una transacción). */
esp_err_t mcp4725_get_status(mcp4725_handle_t dev, mcp4725_status_t *status);

/**
 * @brief Cambia el modo de operación conservando el valor del DAC.
 *        Con MCP4725_PD_NORMAL sale del power-down.
 */
esp_err_t mcp4725_set_power_mode(mcp4725_handle_t dev, mcp4725_pd_t mode);

/**
 * @brief Escribe DAC + EEPROM (valor y modo de arranque).
 * @param wait true: espera (hasta ~100 ms) a que termine la escritura de EEPROM
 * @note  La EEPROM tarda ~25-50 ms y tiene ciclos de escritura limitados: no usar en bucles.
 */
esp_err_t mcp4725_write_eeprom(mcp4725_handle_t dev, uint16_t raw, mcp4725_pd_t mode, bool wait);

/** @brief Espera a que termine una escritura de EEPROM. ESP_ERR_TIMEOUT si no termina. */
esp_err_t mcp4725_wait_eeprom_ready(mcp4725_handle_t dev, int timeout_ms);

/**
 * @brief General Call Reset (0x00 + 0x06): afecta a TODOS los MCP4725 del bus
 *        (y a cualquier otro dispositivo que atienda general call). Recarga la EEPROM al DAC.
 */
esp_err_t mcp4725_general_call_reset(mcp4725_handle_t dev);

/** @brief General Call Wake-Up (0x00 + 0x09): saca de power-down a todos los MCP4725 del bus. */
esp_err_t mcp4725_general_call_wakeup(mcp4725_handle_t dev);

#ifdef __cplusplus
}
#endif