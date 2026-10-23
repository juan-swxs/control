#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "mcp4725.h"

static const char *TAG = "mcp4725";

#define GENERAL_CALL_ADDR    0x00
#define GENERAL_CALL_RESET   0x06
#define GENERAL_CALL_WAKEUP  0x09
#define CMD_WRITE_DAC_EEPROM 0x60   /* C2 C1 C0 = 0 1 1 */
#define EEPROM_POLL_MS       5

struct mcp4725_dev {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t i2c;
    SemaphoreHandle_t       lock;
    uint32_t                scl_speed_hz;
    float                   vref;
    int                     timeout_ms;
};

#define CHECK_DEV(d) ESP_RETURN_ON_FALSE((d), ESP_ERR_INVALID_ARG, TAG, "handle nulo")

static inline void lock(mcp4725_handle_t d)   { xSemaphoreTake(d->lock, portMAX_DELAY); }
static inline void unlock(mcp4725_handle_t d) { xSemaphoreGive(d->lock); }

/* ---- helpers internos (sin lock) ---- */

static esp_err_t read_regs(mcp4725_handle_t d, mcp4725_status_t *st)
{
    uint8_t b[5];
    esp_err_t err = i2c_master_receive(d->i2c, b, sizeof(b), d->timeout_ms);
    if (err != ESP_OK) {
        return err;
    }
    st->eeprom_ready = (b[0] & 0x80) != 0;
    st->dac_pd       = (mcp4725_pd_t)((b[0] >> 1) & 0x03);
    st->dac_raw      = ((uint16_t)b[1] << 4) | (b[2] >> 4);
    st->eeprom_pd    = (mcp4725_pd_t)((b[3] >> 5) & 0x03);
    st->eeprom_raw   = ((uint16_t)(b[3] & 0x0F) << 8) | b[4];
    return ESP_OK;
}

static esp_err_t fast_write(mcp4725_handle_t d, uint16_t raw, mcp4725_pd_t pd)
{
    /* Fast mode: [0 0 PD1 PD0 D11..D8] [D7..D0] */
    uint8_t buf[2] = {
        (uint8_t)(((pd & 0x03) << 4) | ((raw >> 8) & 0x0F)),
        (uint8_t)(raw & 0xFF),
    };
    return i2c_master_transmit(d->i2c, buf, sizeof(buf), d->timeout_ms);
}

static esp_err_t wait_ready(mcp4725_handle_t d, int timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t limit = pdMS_TO_TICKS(timeout_ms);
    mcp4725_status_t st;
    do {
        esp_err_t err = read_regs(d, &st);
        if (err != ESP_OK) {
            return err;
        }
        if (st.eeprom_ready) {
            return ESP_OK;
        }
        vTaskDelay(pdMS_TO_TICKS(EEPROM_POLL_MS) ? pdMS_TO_TICKS(EEPROM_POLL_MS) : 1);
    } while ((xTaskGetTickCount() - start) < limit);
    return ESP_ERR_TIMEOUT;
}

static esp_err_t general_call(mcp4725_handle_t d, uint8_t cmd)
{
    i2c_device_config_t cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = GENERAL_CALL_ADDR,
        .scl_speed_hz    = d->scl_speed_hz,
    };
    i2c_master_dev_handle_t gc;
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(d->bus, &cfg, &gc), TAG, "general call: add_device");
    esp_err_t err = i2c_master_transmit(gc, &cmd, 1, d->timeout_ms);
    i2c_master_bus_rm_device(gc);
    return err;
}

/* ---- API ---- */

esp_err_t mcp4725_create(i2c_master_bus_handle_t bus, const mcp4725_config_t *cfg,
                         mcp4725_handle_t *out)
{
    ESP_RETURN_ON_FALSE(bus && cfg && out, ESP_ERR_INVALID_ARG, TAG, "argumento nulo");
    ESP_RETURN_ON_FALSE(cfg->vref > 0.0f, ESP_ERR_INVALID_ARG, TAG, "vref debe ser > 0");

    esp_err_t err = i2c_master_probe(bus, cfg->address, cfg->timeout_ms);
    ESP_RETURN_ON_ERROR(err, TAG, "sin respuesta en 0x%02X", cfg->address);

    mcp4725_handle_t d = calloc(1, sizeof(*d));
    ESP_RETURN_ON_FALSE(d, ESP_ERR_NO_MEM, TAG, "sin memoria");
    d->lock = xSemaphoreCreateMutex();
    if (!d->lock) {
        free(d);
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->address,
        .scl_speed_hz    = cfg->scl_speed_hz,
    };
    err = i2c_master_bus_add_device(bus, &dev_cfg, &d->i2c);
    if (err != ESP_OK) {
        vSemaphoreDelete(d->lock);
        free(d);
        return err;
    }
    d->bus = bus;
    d->scl_speed_hz = cfg->scl_speed_hz;
    d->vref = cfg->vref;
    d->timeout_ms = cfg->timeout_ms;
    *out = d;
    return ESP_OK;
}

esp_err_t mcp4725_delete(mcp4725_handle_t d)
{
    CHECK_DEV(d);
    lock(d);
    esp_err_t err = i2c_master_bus_rm_device(d->i2c);
    unlock(d);
    vSemaphoreDelete(d->lock);
    free(d);
    return err;
}

esp_err_t mcp4725_set_vref(mcp4725_handle_t d, float vref)
{
    CHECK_DEV(d);
    ESP_RETURN_ON_FALSE(vref > 0.0f, ESP_ERR_INVALID_ARG, TAG, "vref debe ser > 0");
    lock(d);
    d->vref = vref;
    unlock(d);
    return ESP_OK;
}

esp_err_t mcp4725_set_raw(mcp4725_handle_t d, uint16_t raw)
{
    CHECK_DEV(d);
    ESP_RETURN_ON_FALSE(raw <= MCP4725_MAX_RAW, ESP_ERR_INVALID_ARG, TAG, "raw fuera de rango");
    lock(d);
    esp_err_t err = fast_write(d, raw, MCP4725_PD_NORMAL);
    unlock(d);
    return err;
}

esp_err_t mcp4725_set_voltage(mcp4725_handle_t d, float volts)
{
    CHECK_DEV(d);
    lock(d);
    float vref = d->vref;
    unlock(d);
    ESP_RETURN_ON_FALSE(volts >= 0.0f && volts <= vref, ESP_ERR_INVALID_ARG, TAG,
                        "voltaje fuera de rango 0..%.3f V", (double)vref);
    /* Datasheet: Vout = Vdd * D / 4096 */
    float raw_f = (volts / vref) * 4096.0f + 0.5f;
    if (raw_f > MCP4725_MAX_RAW) {
        raw_f = MCP4725_MAX_RAW;
    }
    return mcp4725_set_raw(d, (uint16_t)raw_f);
}

esp_err_t mcp4725_get_status(mcp4725_handle_t d, mcp4725_status_t *st)
{
    CHECK_DEV(d);
    ESP_RETURN_ON_FALSE(st, ESP_ERR_INVALID_ARG, TAG, "status nulo");
    lock(d);
    esp_err_t err = read_regs(d, st);
    unlock(d);
    return err;
}

esp_err_t mcp4725_get_raw(mcp4725_handle_t d, uint16_t *raw)
{
    ESP_RETURN_ON_FALSE(raw, ESP_ERR_INVALID_ARG, TAG, "raw nulo");
    mcp4725_status_t st;
    ESP_RETURN_ON_ERROR(mcp4725_get_status(d, &st), TAG, "lectura");
    *raw = st.dac_raw;
    return ESP_OK;
}

esp_err_t mcp4725_get_voltage(mcp4725_handle_t d, float *volts)
{
    ESP_RETURN_ON_FALSE(volts, ESP_ERR_INVALID_ARG, TAG, "volts nulo");
    uint16_t raw;
    ESP_RETURN_ON_ERROR(mcp4725_get_raw(d, &raw), TAG, "lectura");
    lock(d);
    float vref = d->vref;
    unlock(d);
    *volts = (float)raw * vref / 4096.0f;
    return ESP_OK;
}

esp_err_t mcp4725_set_power_mode(mcp4725_handle_t d, mcp4725_pd_t mode)
{
    CHECK_DEV(d);
    ESP_RETURN_ON_FALSE((unsigned)mode <= MCP4725_PD_500K, ESP_ERR_INVALID_ARG, TAG, "modo inválido");
    mcp4725_status_t st;
    lock(d);
    /* El modo rápido también carga los 12 bits de datos: se relee el DAC para conservarlo. */
    esp_err_t err = read_regs(d, &st);
    if (err == ESP_OK) {
        err = fast_write(d, st.dac_raw, mode);
    }
    unlock(d);
    return err;
}

esp_err_t mcp4725_write_eeprom(mcp4725_handle_t d, uint16_t raw, mcp4725_pd_t mode, bool wait)
{
    CHECK_DEV(d);
    ESP_RETURN_ON_FALSE(raw <= MCP4725_MAX_RAW, ESP_ERR_INVALID_ARG, TAG, "raw fuera de rango");
    ESP_RETURN_ON_FALSE((unsigned)mode <= MCP4725_PD_500K, ESP_ERR_INVALID_ARG, TAG, "modo inválido");
    /* [0 1 1 x x PD1 PD0 x] [D11..D4] [D3..D0 0 0 0 0] */
    uint8_t buf[3] = {
        (uint8_t)(CMD_WRITE_DAC_EEPROM | ((mode & 0x03) << 1)),
        (uint8_t)(raw >> 4),
        (uint8_t)((raw & 0x0F) << 4),
    };
    lock(d);
    esp_err_t err = i2c_master_transmit(d->i2c, buf, sizeof(buf), d->timeout_ms);
    if (err == ESP_OK && wait) {
        err = wait_ready(d, 100);
    }
    unlock(d);
    return err;
}

esp_err_t mcp4725_wait_eeprom_ready(mcp4725_handle_t d, int timeout_ms)
{
    CHECK_DEV(d);
    lock(d);
    esp_err_t err = wait_ready(d, timeout_ms);
    unlock(d);
    return err;
}

esp_err_t mcp4725_general_call_reset(mcp4725_handle_t d)
{
    CHECK_DEV(d);
    lock(d);
    esp_err_t err = general_call(d, GENERAL_CALL_RESET);
    unlock(d);
    return err;
}

esp_err_t mcp4725_general_call_wakeup(mcp4725_handle_t d)
{
    CHECK_DEV(d);
    lock(d);
    esp_err_t err = general_call(d, GENERAL_CALL_WAKEUP);
    unlock(d);
    return err;
}