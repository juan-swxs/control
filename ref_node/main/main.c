/*
 * ============================================================================
 *      LAB 4 · ESP32 DE REFERENCIA (MCP4725 #3 -> Vr) — ESP-IDF >= 5.3
 *
 *  Recibe por ESP-NOW el voltaje de referencia que pide el ESP32 principal
 *  y lo saca en su MCP4725 (I2C0, SDA=21, SCL=22, dirección 0x60).
 *  Después de cada escritura relee el registro del DAC y le responde al
 *  principal con lo que de verdad quedó en la salida.
 *
 *  Con el push-pull (control digital) el principal manda el código del DAC
 *  cada 10 ms. 2048 = 1.65 V = motor parado: es el valor de arranque, el de
 *  la EEPROM y al que vuelve si pasan FAILSAFE_MS sin mensajes.
 *  Si el MCP4725 no responde, reintenta encontrarlo cada 2 s.
 * ============================================================================
 */

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_check.h"
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "mcp4725.h"
#include "lab_link.h"

static const char *TAG = "REF_NODE";

#define I2C_SDA_GPIO 21
#define I2C_SCL_GPIO 22
#define MCP_VDD_DEFAULT 3.3f
#define MCP_RETRY_MS 2000
#define DAC_MID_RAW 2048    /* motor parado */
#define FAILSAFE_MS 200     /* sin mensajes -> DAC_MID_RAW */
#define STATUS_MS 250       /* cada cuánto se le responde al principal */

static const uint8_t BROADCAST[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static i2c_master_bus_handle_t s_bus = NULL;
static mcp4725_handle_t s_mcp = NULL;
static float s_vdd = MCP_VDD_DEFAULT;

/* ESP32 principal con el que quedó emparejado (el primero que habló) */
static uint8_t s_master[ESP_NOW_ETH_ALEN];
static bool s_paired = false;

typedef struct
{
    lab_link_msg_t msg;
    uint8_t src[ESP_NOW_ETH_ALEN];
} rx_item_t;

static QueueHandle_t s_rx_queue = NULL;

/* ============================================================================
 * MCP4725 #3
 * ==========================================================================*/

static esp_err_t mcp_init(void)
{
    if (s_bus == NULL)
    {
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = I2C_SDA_GPIO,
            .scl_io_num = I2C_SCL_GPIO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "Error creando bus I2C");
    }

    uint8_t addr = 0;
    for (uint8_t a = 0x60; a <= 0x67 && addr == 0; a++)
    {
        if (i2c_master_probe(s_bus, a, 50) == ESP_OK)
            addr = a;
    }
    if (addr == 0)
        return ESP_ERR_NOT_FOUND;

    mcp4725_config_t cfg = MCP4725_CONFIG_DEFAULT();
    cfg.address = addr;
    cfg.vref = s_vdd;
    cfg.timeout_ms = 20; /* tick de 10 ms: menos de 10 ms valdría 0 ticks */
    cfg.scl_speed_hz = 100000;
    ESP_RETURN_ON_ERROR(mcp4725_create(s_bus, &cfg, &s_mcp), TAG, "Error creando MCP4725");
    ESP_RETURN_ON_ERROR(mcp4725_set_raw(s_mcp, DAC_MID_RAW), TAG, "Error escribiendo MCP4725");

    /* Al encenderse el MCP4725 saca su EEPROM hasta que arranca el firmware:
     * 2048 = motor parado. Solo se escribe si hace falta (ciclos limitados). */
    mcp4725_status_t st;
    if (mcp4725_get_status(s_mcp, &st) != ESP_OK)
        ESP_LOGE(TAG, "MCP4725 #3: no se pudo leer la EEPROM");
    else if (st.eeprom_raw == DAC_MID_RAW && st.eeprom_pd == MCP4725_PD_NORMAL)
        ESP_LOGI(TAG, "MCP4725 #3: EEPROM en 2048 (motor parado al encender)");
    else
    {
        if (mcp4725_write_eeprom(s_mcp, DAC_MID_RAW, MCP4725_PD_NORMAL, true) == ESP_OK)
            ESP_LOGW(TAG, "MCP4725 #3: EEPROM tenía raw %u, ahora arranca en 2048", st.eeprom_raw);
        else
            ESP_LOGE(TAG, "MCP4725 #3: no se pudo grabar la EEPROM");
    }
    ESP_LOGI(TAG, "MCP4725 #3 -> Vr en 0x%02X (SDA=%d, SCL=%d), arranca en 2048", addr, I2C_SDA_GPIO, I2C_SCL_GPIO);
    return ESP_OK;
}

static void mcp_drop(void)
{
    if (s_mcp)
    {
        mcp4725_delete(s_mcp);
        s_mcp = NULL;
    }
}

/* Escribe Vr y relee el registro; devuelve false si el I2C falló */
static bool mcp_apply(uint16_t raw, uint16_t *raw_out, float *v_out)
{
    if (s_mcp == NULL)
        return false;
    if (raw > MCP4725_MAX_RAW)
        raw = MCP4725_MAX_RAW;

    uint16_t rb = 0;
    if (mcp4725_set_raw(s_mcp, (uint16_t)raw) != ESP_OK || mcp4725_get_raw(s_mcp, &rb) != ESP_OK)
        return false;
    *raw_out = rb;
    *v_out = rb * s_vdd / 4096.0f;
    return rb == raw;
}

/* ============================================================================
 * ESP-NOW
 * ==========================================================================*/

static void on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len != sizeof(lab_link_msg_t))
        return;
    rx_item_t item;
    memcpy(&item.msg, data, sizeof(item.msg));
    if (item.msg.magic != LAB_LINK_MAGIC || item.msg.version != LAB_LINK_VERSION || item.msg.type != LINK_SET_REF)
        return;
    memcpy(item.src, info->src_addr, ESP_NOW_ETH_ALEN);
    xQueueSend(s_rx_queue, &item, 0);
}

static esp_err_t espnow_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(err, TAG, "NVS");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "netif");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop");

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&wcfg), TAG, "wifi init");
    ESP_RETURN_ON_ERROR(esp_wifi_set_storage(WIFI_STORAGE_RAM), TAG, "wifi storage");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "wifi mode");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start");
    ESP_RETURN_ON_ERROR(esp_wifi_set_channel(LAB_LINK_CHANNEL, WIFI_SECOND_CHAN_NONE), TAG, "canal");
    ESP_RETURN_ON_ERROR(esp_wifi_set_max_tx_power(LAB_LINK_TX_POWER_QDBM), TAG, "potencia TX");

    ESP_RETURN_ON_ERROR(esp_now_init(), TAG, "esp_now init");
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(on_recv), TAG, "recv cb");

    esp_now_peer_info_t peer = {.channel = LAB_LINK_CHANNEL, .ifidx = WIFI_IF_STA, .encrypt = false};
    memcpy(peer.peer_addr, BROADCAST, ESP_NOW_ETH_ALEN);
    ESP_RETURN_ON_ERROR(esp_now_add_peer(&peer), TAG, "peer broadcast");

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "ESP-NOW listo en canal %d, MAC " MACSTR, LAB_LINK_CHANNEL, MAC2STR(mac));
    return ESP_OK;
}

static void send_status(uint32_t seq, float volts, uint16_t raw)
{
    lab_link_msg_t st = {
        .magic = LAB_LINK_MAGIC,
        .version = LAB_LINK_VERSION,
        .type = LINK_STATUS,
        .seq = seq,
        .volts = volts,
        .vdd = s_vdd,
        .raw = raw,
        .mcp_ok = s_mcp != NULL,
    };
    esp_now_send(BROADCAST, (const uint8_t *)&st, sizeof(st));
}

/* ============================================================================
 * TAREA PRINCIPAL
 * ==========================================================================*/

static void link_task(void *arg)
{
    uint32_t last_seq = 0;
    float out_v = DAC_MID_RAW * MCP_VDD_DEFAULT / 4096.0f;
    uint16_t out_raw = DAC_MID_RAW;
    int last_req = DAC_MID_RAW; /* mcp_init ya dejó el DAC en 2048 */
    int64_t last_retry_us = 0, last_rx_us = 0, last_status_us = 0;
    bool link_on = false;

    while (1)
    {
        rx_item_t item;
        bool got = xQueueReceive(s_rx_queue, &item, pdMS_TO_TICKS(50)) == pdTRUE;
        int64_t now = esp_timer_get_time();

        /* MCP4725 ausente: reintenta cada MCP_RETRY_MS */
        if (s_mcp == NULL && now - last_retry_us > MCP_RETRY_MS * 1000LL)
        {
            last_retry_us = now;
            if (mcp_init() != ESP_OK)
                ESP_LOGW(TAG, "No hay MCP4725 en SDA=%d, SCL=%d; reintento en %d s", I2C_SDA_GPIO, I2C_SCL_GPIO, MCP_RETRY_MS / 1000);
            else
                last_req = -1; /* fuerza reescribir */
        }

        int want = -1;
        if (got)
        {
            if (!s_paired)
            {
                memcpy(s_master, item.src, ESP_NOW_ETH_ALEN);
                s_paired = true;
                ESP_LOGI(TAG, "Emparejado con el ESP32 principal " MACSTR, MAC2STR(s_master));
            }
            if (memcmp(item.src, s_master, ESP_NOW_ETH_ALEN) != 0)
                continue; /* otro grupo del laboratorio */

            last_rx_us = now;
            if (!link_on)
            {
                ESP_LOGI(TAG, "Enlace con el principal activo");
                link_on = true;
            }
            if (item.msg.vdd >= 2.7f && item.msg.vdd <= 5.5f && fabsf(item.msg.vdd - s_vdd) > 1e-4f)
            {
                s_vdd = item.msg.vdd;
                if (s_mcp)
                    mcp4725_set_vref(s_mcp, s_vdd);
            }
            last_seq = item.msg.seq;
            want = item.msg.raw;
        }
        else if (link_on && now - last_rx_us > FAILSAFE_MS * 1000LL)
        {
            ESP_LOGW(TAG, "Sin mensajes del principal: DAC a %d (motor parado)", DAC_MID_RAW);
            link_on = false;
            want = DAC_MID_RAW;
        }
        else if (!link_on)
        {
            want = DAC_MID_RAW;
        }

        if (want >= 0 && want != last_req)
        {
            if (mcp_apply((uint16_t)want, &out_raw, &out_v))
                last_req = want;
            else if (s_mcp)
            {
                ESP_LOGE(TAG, "Fallo I2C escribiendo el DAC; se busca de nuevo el módulo");
                mcp_drop();
                last_req = -1;
            }
        }

        if (now - last_status_us >= STATUS_MS * 1000LL)
        {
            last_status_us = now;
            send_status(last_seq, out_v, out_raw);
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando ESP32 de referencia...");
    s_rx_queue = xQueueCreate(8, sizeof(rx_item_t));

    if (mcp_init() != ESP_OK)
        ESP_LOGW(TAG, "No hay MCP4725 en SDA=%d, SCL=%d; se reintenta", I2C_SDA_GPIO, I2C_SCL_GPIO);

    ESP_ERROR_CHECK(espnow_init());
    xTaskCreate(link_task, "link_task", 4096, NULL, 5, NULL);
}
