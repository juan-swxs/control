/*
 * ============================================================================
 *      CONTROL DIGITAL DE POSICIÓN (PIÑÓN-CREMALLERA) — ESP32, ESP-IDF >= 5.3
 *
 *  El ESP32 lee el encoder, calcula la ley de control y mueve el motor a
 *  través del push-pull (hardware/push_pull):
 *
 *      Vm = 4.7 · (DAC − 1.65 V)       DAC = 2048 -> motor parado
 *
 *  LEY DE CONTROL (cada 10 ms):
 *      e  = r − x
 *      Vm = KP·e − KD·ẋ + KI·∫e        (recortada a ±VM_MAX, anti-windup)
 *
 *  SALIDA: el mismo código del DAC se escribe en los tres MCP4725, así el
 *  push-pull funciona esté en el que esté:
 *      MCP4725 #1 (I2C0, SDA=21, SCL=22) y #2 (I2C1, SDA=19, SCL=18) aquí
 *      MCP4725 #3 en el ESP32 de referencia, por ESP-NOW cada 10 ms
 *  Las EEPROM quedan en 2048: al encender la fuente el motor no se mueve.
 *  El ESP32 de referencia vuelve a 2048 si deja de recibir mensajes.
 *
 *  Al arrancar, la posición actual es 0 mm y el lazo queda cerrado con r = 0.
 *
 *  TELEMETRÍA (ESP32 -> PC), cada 20 ms:
 *      DATA,t_ms,x_mm,v_mm_s,r_mm,vr_v,p_raw,p_v,vel_raw,vel_v,modo,prueba,
 *           vm_v,motor_raw,xh1,xh2,modo_ctrl,ley
 *      (los 12 primeros son los de la interfaz app/: vr, P y V son lo que de
 *      verdad sale por los MCP4725 #3, #1 y #2; el DAC del push-pull muestra Vm)
 *      xh1, xh2: estimación del observador (mm, mm/s); ley: 0 PID, 1 EST, 2 OBS
 *      modo_ctrl: 0 = parado, 1 = lazo cerrado, 2 = lazo abierto (OPEN)
 *  CFG,sx,sv,x_max,r_max,vdd,mcp1_ok,mcp2_ok,ref_ok,ley,k1,k2,ki,kp,kd,ki_pid,mid_raw,salida,r_min,x_min
 *  REFST,enlace,mcp_ok,raw,v_salida   (MCP4725 #3, cada 500 ms)
 *  Respuesta a comandos: ACK,<texto> o ERR,<texto>
 *
 *  COMANDOS (PC -> ESP32), una línea terminada en '\n':
 *      REF <mm>        referencia y cierra el lazo
 *      ZERO            la posición actual pasa a ser 0 mm (y r = 0)
 *      KP <V/mm> | KD <V/(mm/s)> | KI <V/(mm·s)>   ganancias
 *      LEY PID|EST|OBS ley de control: PID, estados medidos u observador
 *      SFK <K1> <K2> <Ki>  ganancias de la realimentación de estados
 *      OPEN <V>        lazo abierto: Vm fijo en el motor (identificación)
 *      STOP            Vm = 0 (DAC en el punto medio)
 *      RUN             cierra el lazo con la referencia actual
 *      MODE POS|NEG    sentido del encoder: x > 0 donde empuja Vm > 0
 *      REFV <mV>       referencia en mV (r = Vr/SX)
 *      TEST <mVp> <mVv> | AUTO   P y V fijas / vuelven al encoder
 *      SALIDA 1|2|3    DAC conectado al push-pull (los demás: P, V, Vr)
 *      SOLO n          detección: Vm solo en el DAC n, los demás a 2048
 *      MID <raw>       código del DAC en el que el motor queda parado
 *      VDD <mV>        Vdd medido de los MCP4725
 *      CFG             reenvía la configuración
 * ============================================================================
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_log.h"
#include "esp_check.h"

#include "driver/pulse_cnt.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "mcp4725.h"
#include "lab_link.h"

static const char *TAG = "CTRL_DIGITAL";

/* ============================================================================
 * PARÁMETROS
 * ==========================================================================*/

#define ENCODER_GPIO_A 34
#define ENCODER_GPIO_B 35
#define PCNT_GLITCH_FILTER_NS 1000
#define PCNT_HIGH_LIMIT 30000
#define PCNT_LOW_LIMIT -30000

#define ENCODER_PPR 11.0f    // Pulsos por revolución motor
#define GEAR_RATIO 20.0f     // Reducción 20:1
#define QUAD_MULTIPLIER 4.0f // Modo x4
#define COUNTS_PER_REV (ENCODER_PPR * GEAR_RATIO * QUAD_MULTIPLIER)
#define PINION_RADIUS_MM 9.4f
#define MM_PER_COUNT ((2.0f * (float)M_PI * PINION_RADIUS_MM) / COUNTS_PER_REV)

#define SAMPLE_PERIOD_MS 10
#define LOG_PERIOD_MS 20
#define LOG_DECIMATION_FACTOR (LOG_PERIOD_MS / SAMPLE_PERIOD_MS)
#define VEL_EMA_ALPHA 0.3f

#define MCP4725_VDD_DEFAULT 3.3f
#define DAC_MID_RAW 2048     // 1.65 V: motor parado (también va a la EEPROM)
#define PP_GAIN 4.7f         // Vm = PP_GAIN · (DAC − Vmid)
#define VM_MAX 7.5f          // recorte de Vm (el push-pull llega a ~±7.8 V)

/* Recorrido medido el 2026-11-13 (desde el centro): -6.4 a +10.5 mm. La
 * referencia se limita con margen para el sobrepico y para que al control le
 * quede voltaje (cerca de los extremos los resortes piden 5 a 7 V). */
#define R_MAX_MM 9.0f        // referencia máxima [mm]
#define R_MIN_MM -5.0f       // referencia mínima [mm]
#define X_MIN_MM -6.4f       // tope medido hacia negativo [mm]
#define X_SAFE_MM 25.0f      // si |x| pasa de aquí, el motor se para

#define KP_DEFAULT 2.0f      // V/mm
#define KD_DEFAULT 0.05f     // V/(mm/s)
#define KI_DEFAULT 4.0f      // V/(mm·s): vence el rozamiento (2 a 4 V)

/* Realimentación de estados, controlador de escalón (m = 1), diseño en
 * Documents/lab4_simulink/lab4_diseno.m con la planta identificada
 *   G(s) = 678.6/(s^2 + 37.93 s + 431.0) [mm/V],  tsd = 0.5 s, zeta = 0.8,
 *   polos -8 ± 6j y -80:   u = -K1 x - K2 xdot + Ki rho,  rho' = r - x
 * u es el voltaje efectivo; el firmware le suma la zona muerta (U0P/U0N). */
#define SF_K1 1.39848f       // V/mm
#define SF_K2 0.0855761f     // V/(mm/s)
#define SF_KI 11.7889f       // V/(mm·s)
/* Observador de orden completo, polos -80 ± 60j, Euler hacia adelante */
#define OBS_L1 122.072f
#define OBS_L2 4939.08f
#define PLANT_B 678.6027f
#define PLANT_A1 37.9278f
#define PLANT_A2 430.9857f
#define U0P 0.0f             // zona muerta del ajuste dinámico [V]
#define U0N 1.1f
#define U0_MIN_U 0.1f        // por debajo de |u| no compensa (evita cascabeleo)

/* ============================================================================
 * ESTADO COMPARTIDO ENTRE TAREAS
 * ==========================================================================*/

typedef enum
{
    DIR_MODE_POS = 0,
    DIR_MODE_NEG = 1,
} dir_mode_t;

typedef enum
{
    CTRL_STOP = 0,
    CTRL_CLOSED = 1,
    CTRL_OPEN = 2,
} ctrl_mode_t;

static volatile float s_ref_mm = 0.0f;
static volatile float s_kp = KP_DEFAULT, s_kd = KD_DEFAULT, s_ki = KI_DEFAULT;
static volatile float s_open_vm = 0.0f;
static volatile ctrl_mode_t s_ctrl = CTRL_CLOSED;

typedef enum
{
    LEY_PID = 0, // PD + I del principio
    LEY_EST = 1, // realimentación de estados con x y xdot medidos
    LEY_OBS = 2, // realimentación de estados con x y xdot del observador
} ley_t;
static volatile ley_t s_ley = LEY_EST;
static volatile float s_k1 = SF_K1, s_k2 = SF_K2, s_kie = SF_KI;
/* 2026-11-13: con el cableado actual Vm > 0 hace contar el encoder hacia negativo */
static volatile dir_mode_t s_dir_mode = DIR_MODE_NEG;
static volatile bool s_zero_request = false;
static volatile bool s_reset_integral = false;
static volatile int s_mid_raw = DAC_MID_RAW;

/* Último código del DAC calculado (lo manda link_task al ESP32 de referencia) */
static volatile uint16_t s_dac_raw = DAC_MID_RAW;

/* Señales "físicas" del montaje analógico del Lab 4. Los DAC que no mueven el
 * motor sacan lo que sacaría el circuito: #1 P = SX·x, #2 V = SV·ẋ, #3 Vr = SX·r
 * (recortadas a 0..Vdd, el MCP4725 no saca negativos). */
#define SX_V_PER_MM 0.4125f  // 3.3 V / 8 mm
#define SV_V_PER_MMS 0.1f    // V/(mm/s)
#define X_APP_MM 10.5f       // tope medido hacia positivo [mm] (escala de la interfaz)
/* DAC que va al push-pull (1, 2 o 3). 0 = el mismo Vm en los tres (solo para
 * detectar a cuál está conectado; ver comando SOLO). */
#ifndef MOTOR_DAC_DEFAULT
#define MOTOR_DAC_DEFAULT 3  // 2026-11-13: el push-pull está en el MCP4725 #3 (ESP32 de referencia)
#endif
static volatile int s_salida = MOTOR_DAC_DEFAULT;
static volatile int s_solo = 0;         // SOLO n: Vm solo en el DAC n, los demás en 2048
static volatile bool s_test_mode = false; // TEST: P y V fijas (pruebas de banco)
static volatile float s_test_p_v = 0.0f, s_test_v_v = 0.0f;

/* ============================================================================
 * ENCODER (PCNT), misma lectura que control_digital
 * ==========================================================================*/

static esp_err_t encoder_pcnt_init(pcnt_unit_handle_t *out_unit)
{
    /* accum_count: el driver acumula al llegar a los límites y get_count ya
     * devuelve el total (no hace falta sumar nada en un callback) */
    pcnt_unit_config_t unit_config = {
        .high_limit = PCNT_HIGH_LIMIT,
        .low_limit = PCNT_LOW_LIMIT,
        .flags.accum_count = true,
    };

    pcnt_unit_handle_t unit = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_unit(&unit_config, &unit), TAG, "Error creando unidad PCNT");

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = PCNT_GLITCH_FILTER_NS,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_set_glitch_filter(unit, &filter_config), TAG, "Error filtro glitch");

    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENCODER_GPIO_A,
        .level_gpio_num = ENCODER_GPIO_B,
    };
    pcnt_channel_handle_t chan_a = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(unit, &chan_a_config, &chan_a), TAG, "Error canal A");

    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENCODER_GPIO_B,
        .level_gpio_num = ENCODER_GPIO_A,
    };
    pcnt_channel_handle_t chan_b = NULL;
    ESP_RETURN_ON_ERROR(pcnt_new_channel(unit, &chan_b_config, &chan_b), TAG, "Error canal B");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE), TAG, "Error edge A");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "Error level A");

    ESP_RETURN_ON_ERROR(pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE), TAG, "Error edge B");
    ESP_RETURN_ON_ERROR(pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE), TAG, "Error level B");

    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(unit, PCNT_HIGH_LIMIT), TAG, "Error watch point alto");
    ESP_RETURN_ON_ERROR(pcnt_unit_add_watch_point(unit, PCNT_LOW_LIMIT), TAG, "Error watch point bajo");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(unit), TAG, "Error habilitando PCNT");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unit), TAG, "Error limpiando contador");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(unit), TAG, "Error iniciando PCNT");

    *out_unit = unit;
    return ESP_OK;
}


static inline int encoder_get_total_counts(pcnt_unit_handle_t unit)
{
    int c = 0;
    pcnt_unit_get_count(unit, &c);
    return c;
}

/* ============================================================================
 * MCP4725 #1 Y #2 (mismo código del DAC que el #3)
 * ==========================================================================*/

typedef struct
{
    i2c_port_num_t port;
    int sda_gpio;
    int scl_gpio;
} mcp4725_salida_cfg_t;

static const mcp4725_salida_cfg_t s_mcp_cfg[2] = {
    {.port = I2C_NUM_0, .sda_gpio = 21, .scl_gpio = 22},
    {.port = I2C_NUM_1, .sda_gpio = 19, .scl_gpio = 18},
};

static i2c_master_bus_handle_t s_i2c_bus[2] = {NULL, NULL};
static mcp4725_handle_t s_mcp[2] = {NULL, NULL};
static volatile float s_mcp_vdd = MCP4725_VDD_DEFAULT;

static uint8_t i2c_find_mcp4725(int idx)
{
    for (uint8_t a = 0x60; a <= 0x67; a++)
    {
        if (i2c_master_probe(s_i2c_bus[idx], a, 50) == ESP_OK)
            return a;
    }
    return 0;
}

/* Al encender, el MCP4725 saca lo que tiene en su EEPROM hasta que arranca el
 * firmware. Con el push-pull, 2048 (1.65 V) es motor parado. Solo se graba si
 * hace falta: la EEPROM tiene ciclos de escritura limitados. */
static void mcp_eeprom_to_mid(mcp4725_handle_t dev, int num)
{
    mcp4725_status_t st;
    if (mcp4725_get_status(dev, &st) != ESP_OK)
    {
        ESP_LOGE(TAG, "MCP4725 #%d: no se pudo leer la EEPROM", num);
        return;
    }
    if (st.eeprom_raw == DAC_MID_RAW && st.eeprom_pd == MCP4725_PD_NORMAL)
    {
        ESP_LOGI(TAG, "MCP4725 #%d: EEPROM en %d (motor parado al encender)", num, DAC_MID_RAW);
        return;
    }
    esp_err_t err = mcp4725_write_eeprom(dev, DAC_MID_RAW, MCP4725_PD_NORMAL, true);
    if (err == ESP_OK)
        ESP_LOGW(TAG, "MCP4725 #%d: EEPROM tenía raw %u, ahora arranca en %d", num, st.eeprom_raw, DAC_MID_RAW);
    else
        ESP_LOGE(TAG, "MCP4725 #%d: no se pudo grabar la EEPROM (%s)", num, esp_err_to_name(err));
}

static esp_err_t mcp4725_salida_init(int idx)
{
    const mcp4725_salida_cfg_t *sc = &s_mcp_cfg[idx];

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = sc->port,
        .sda_io_num = sc->sda_gpio,
        .scl_io_num = sc->scl_gpio,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_i2c_bus[idx]), TAG, "Error creando bus I2C %d", idx);

    uint8_t addr = i2c_find_mcp4725(idx);
    ESP_RETURN_ON_FALSE(addr != 0, ESP_ERR_NOT_FOUND, TAG, "No hay MCP4725 en el bus %d", idx);

    mcp4725_handle_t dev = NULL;
    mcp4725_config_t cfg = MCP4725_CONFIG_DEFAULT();
    cfg.address = addr;
    cfg.vref = s_mcp_vdd;
    cfg.timeout_ms = 20;        // tick de 10 ms: menos de 10 ms valdría 0 ticks
    cfg.scl_speed_hz = 100000;  // pull-ups internos: 400 kHz da NACK
    ESP_RETURN_ON_ERROR(mcp4725_create(s_i2c_bus[idx], &cfg, &dev), TAG, "Error creando MCP4725 #%d", idx + 1);
    ESP_RETURN_ON_ERROR(mcp4725_set_raw(dev, DAC_MID_RAW), TAG, "Error escribiendo MCP4725 #%d", idx + 1);
    mcp_eeprom_to_mid(dev, idx + 1);

    s_mcp[idx] = dev;
    ESP_LOGI(TAG, "MCP4725 #%d en 0x%02X (SDA=%d, SCL=%d), arranca en %d", idx + 1, addr, sc->sda_gpio, sc->scl_gpio, DAC_MID_RAW);
    return ESP_OK;
}

/* Vm deseado -> código del DAC alrededor del punto medio */
static inline uint16_t dac_raw_for_vm(float vm)
{
    long raw = s_mid_raw + lroundf(vm / PP_GAIN / s_mcp_vdd * 4096.0f);
    if (raw < 0)
        raw = 0;
    if (raw > MCP4725_MAX_RAW)
        raw = MCP4725_MAX_RAW;
    return (uint16_t)raw;
}

static inline float dac_v_for_raw(uint16_t raw)
{
    return raw * s_mcp_vdd / 4096.0f;
}

/* ============================================================================
 * ENLACE ESP-NOW CON EL ESP32 DE REFERENCIA (MCP4725 #3)
 * ==========================================================================*/

static const uint8_t BROADCAST_MAC[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

static uint32_t s_link_seq = 0;
static uint8_t s_ref_node_mac[ESP_NOW_ETH_ALEN];
static volatile bool s_ref_node_paired = false;
static volatile int64_t s_link_last_rx_us = 0;
static volatile uint8_t s_link_mcp_ok = 0;
static volatile uint16_t s_link_raw = 0;
static volatile float s_link_out_v = 0.0f;

static bool link_alive(void)
{
    return s_link_last_rx_us != 0 &&
           esp_timer_get_time() - s_link_last_rx_us < LAB_LINK_TIMEOUT_MS * 1000LL;
}

static bool ref_output_ok(void)
{
    return link_alive() && s_link_mcp_ok;
}

static void link_on_recv(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len != sizeof(lab_link_msg_t))
        return;
    lab_link_msg_t m;
    memcpy(&m, data, sizeof(m));
    if (m.magic != LAB_LINK_MAGIC || m.version != LAB_LINK_VERSION || m.type != LINK_STATUS)
        return;
    if (!s_ref_node_paired)
    {
        memcpy(s_ref_node_mac, info->src_addr, ESP_NOW_ETH_ALEN);
        s_ref_node_paired = true;
    }
    else if (memcmp(info->src_addr, s_ref_node_mac, ESP_NOW_ETH_ALEN) != 0)
    {
        return;
    }
    s_link_mcp_ok = m.mcp_ok;
    s_link_raw = m.raw;
    s_link_out_v = m.volts;
    s_link_last_rx_us = esp_timer_get_time();
}

static void link_send_dac(uint16_t raw)
{
    lab_link_msg_t m = {
        .magic = LAB_LINK_MAGIC,
        .version = LAB_LINK_VERSION,
        .type = LINK_SET_REF,
        .seq = ++s_link_seq,
        .volts = dac_v_for_raw(raw),
        .vdd = s_mcp_vdd,
        .raw = raw,
    };
    esp_now_send(BROADCAST_MAC, (const uint8_t *)&m, sizeof(m));
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
    ESP_RETURN_ON_ERROR(esp_now_register_recv_cb(link_on_recv), TAG, "recv cb");

    esp_now_peer_info_t peer = {.channel = LAB_LINK_CHANNEL, .ifidx = WIFI_IF_STA, .encrypt = false};
    memcpy(peer.peer_addr, BROADCAST_MAC, ESP_NOW_ETH_ALEN);
    ESP_RETURN_ON_ERROR(esp_now_add_peer(&peer), TAG, "peer broadcast");

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG, "ESP-NOW listo en canal %d, MAC " MACSTR, LAB_LINK_CHANNEL, MAC2STR(mac));
    return ESP_OK;
}

static void print_cfg(void);

/* Manda el código del DAC cada periodo de muestreo y vigila el enlace */
static void link_task(void *arg)
{
    bool was_ok = false;
    int64_t last_report_us = 0;
    TickType_t last_wake = xTaskGetTickCount();
    while (1)
    {
        link_send_dac(s_dac_raw);

        int64_t now = esp_timer_get_time();
        bool ok = ref_output_ok();
        if (ok != was_ok)
        {
            was_ok = ok;
            if (ok)
                ESP_LOGI(TAG, "MCP4725 #3 activo en el ESP32 de referencia " MACSTR, MAC2STR(s_ref_node_mac));
            else
                printf("ERR,Sin respuesta del ESP32 de referencia (MCP #3)\n");
            print_cfg();
        }
        if (now - last_report_us >= 500000LL)
        {
            last_report_us = now;
            printf("REFST,%d,%d,%u,%.4f\n", link_alive(), s_link_mcp_ok, s_link_raw, (double)s_link_out_v);
        }
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/* ============================================================================
 * CONSOLA SERIAL + COMANDOS
 * ==========================================================================*/

static void uart_console_init(void)
{
    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);

    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_cfg));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 2048, 0, NULL, 0));
    uart_vfs_dev_use_driver(UART_NUM_0);
}

static void print_cfg(void)
{
    /* los 9 primeros campos son los de la interfaz (app/), el resto es extra */
    printf("CFG,%.4f,%.3f,%.2f,%.2f,%.3f,%d,%d,%d,%d,%.5f,%.5f,%.4f,%.4f,%.4f,%.4f,%d,%d,%.2f,%.2f\n",
           SX_V_PER_MM, SV_V_PER_MMS, X_APP_MM, R_MAX_MM, (double)s_mcp_vdd,
           s_mcp[0] != NULL, s_mcp[1] != NULL, ref_output_ok(),
           (int)s_ley, (double)s_k1, (double)s_k2, (double)s_kie,
           (double)s_kp, (double)s_kd, (double)s_ki, s_mid_raw, s_salida,
           R_MIN_MM, X_MIN_MM);
}

static const char *cmd_arg(const char *line, size_t cmd_len)
{
    const char *arg = line + cmd_len;
    while (*arg == ' ')
        arg++;
    return arg;
}

static void handle_command(char *line)
{
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' || line[len - 1] == ' '))
        line[--len] = '\0';
    if (len == 0)
        return;

    if (strncmp(line, "REF", 3) == 0)
    {
        float r;
        if (strncmp(line, "REFV", 4) == 0) // referencia en mV, como Vr = SX·r
            r = strtof(cmd_arg(line, 4), NULL) / 1000.0f / SX_V_PER_MM;
        else
            r = strtof(cmd_arg(line, 3), NULL);
        if (r > R_MAX_MM)
            r = R_MAX_MM;
        if (r < R_MIN_MM)
            r = R_MIN_MM;
        s_ref_mm = r;
        s_ctrl = CTRL_CLOSED;
        printf("ACK,REF,%.3f,%.4f\n", (double)r, (double)(r * SX_V_PER_MM));
    }
    else if (strcmp(line, "ZERO") == 0)
    {
        s_ref_mm = 0.0f;
        s_zero_request = true;
        printf("ACK,ZERO\n");
    }
    else if (strncmp(line, "KP", 2) == 0)
    {
        s_kp = strtof(cmd_arg(line, 2), NULL);
        printf("ACK,KP,%.4f\n", (double)s_kp);
    }
    else if (strncmp(line, "KD", 2) == 0)
    {
        s_kd = strtof(cmd_arg(line, 2), NULL);
        printf("ACK,KD,%.4f\n", (double)s_kd);
    }
    else if (strncmp(line, "KI", 2) == 0)
    {
        s_ki = strtof(cmd_arg(line, 2), NULL);
        s_reset_integral = true;
        printf("ACK,KI,%.4f\n", (double)s_ki);
    }
    else if (strncmp(line, "OPEN", 4) == 0)
    {
        float vm = strtof(cmd_arg(line, 4), NULL);
        if (vm > VM_MAX)
            vm = VM_MAX;
        if (vm < -VM_MAX)
            vm = -VM_MAX;
        s_open_vm = vm;
        s_ctrl = CTRL_OPEN;
        printf("ACK,OPEN,%.3f\n", (double)vm);
    }
    else if (strncmp(line, "TEST", 4) == 0)
    {
        char *end = NULL;
        float mvp = strtof(cmd_arg(line, 4), &end);
        float mvv = strtof(end, NULL);
        s_test_p_v = mvp / 1000.0f;
        s_test_v_v = mvv / 1000.0f;
        s_test_mode = true;
        printf("ACK,TEST,%.3f,%.3f\n", (double)s_test_p_v, (double)s_test_v_v);
    }
    else if (strcmp(line, "AUTO") == 0)
    {
        s_test_mode = false;
        printf("ACK,AUTO\n");
    }
    else if (strncmp(line, "SALIDA", 6) == 0)
    {
        int n = atoi(cmd_arg(line, 6));
        if (n < 0 || n > 3)
        {
            printf("ERR,SALIDA 1, 2 o 3 (0 = Vm en los tres)\n");
            return;
        }
        s_salida = n;
        s_solo = 0;
        printf("ACK,SALIDA,%d\n", n);
        print_cfg();
    }
    else if (strncmp(line, "SOLO", 4) == 0)
    {
        int n = atoi(cmd_arg(line, 4));
        if (n < 0 || n > 3)
        {
            printf("ERR,SOLO 1, 2 o 3 (0 = desactivado)\n");
            return;
        }
        s_solo = n;
        printf("ACK,SOLO,%d\n", n);
    }
    else if (strncmp(line, "LEY", 3) == 0)
    {
        const char *arg = cmd_arg(line, 3);
        if (strcmp(arg, "PID") == 0)
            s_ley = LEY_PID;
        else if (strcmp(arg, "EST") == 0)
            s_ley = LEY_EST;
        else if (strcmp(arg, "OBS") == 0)
            s_ley = LEY_OBS;
        else
        {
            printf("ERR,Ley desconocida: %s (PID, EST u OBS)\n", arg);
            return;
        }
        s_reset_integral = true;
        printf("ACK,LEY,%s\n", arg);
    }
    else if (strncmp(line, "SFK", 3) == 0)
    {
        char *end = NULL;
        float k1 = strtof(cmd_arg(line, 3), &end);
        float k2 = strtof(end, &end);
        float ki = strtof(end, NULL);
        s_k1 = k1;
        s_k2 = k2;
        s_kie = ki;
        s_reset_integral = true;
        printf("ACK,SFK,%.5f,%.5f,%.4f\n", (double)k1, (double)k2, (double)ki);
    }
    else if (strcmp(line, "STOP") == 0)
    {
        s_ctrl = CTRL_STOP;
        printf("ACK,STOP\n");
    }
    else if (strcmp(line, "RUN") == 0)
    {
        s_reset_integral = true;
        s_ctrl = CTRL_CLOSED;
        printf("ACK,RUN\n");
    }
    else if (strncmp(line, "MODE", 4) == 0)
    {
        const char *arg = cmd_arg(line, 4);
        if (strcmp(arg, "POS") == 0 || strcmp(arg, "NEG") == 0)
        {
            s_dir_mode = (arg[0] == 'P') ? DIR_MODE_POS : DIR_MODE_NEG;
            s_ref_mm = 0.0f;
            s_zero_request = true;
            printf("ACK,MODE,%s\n", arg);
        }
        else
        {
            printf("ERR,Modo desconocido: %s\n", arg);
        }
    }
    else if (strncmp(line, "MID", 3) == 0)
    {
        int mid = atoi(cmd_arg(line, 3));
        if (mid < 1500 || mid > 2600)
        {
            printf("ERR,MID fuera de rango (1500 a 2600)\n");
            return;
        }
        s_mid_raw = mid;
        printf("ACK,MID,%d\n", mid);
    }
    else if (strncmp(line, "VDD", 3) == 0)
    {
        float vdd = strtof(cmd_arg(line, 3), NULL) / 1000.0f;
        if (vdd < 2.7f || vdd > 5.5f)
        {
            printf("ERR,VDD fuera de rango (2700 a 5500 mV)\n");
            return;
        }
        s_mcp_vdd = vdd;
        for (int i = 0; i < 2; i++)
        {
            if (s_mcp[i])
                mcp4725_set_vref(s_mcp[i], vdd);
        }
        printf("ACK,VDD,%.3f\n", (double)vdd);
        print_cfg();
    }
    else if (strcmp(line, "CFG") == 0)
    {
        print_cfg();
    }
    else
    {
        printf("ERR,Comando desconocido: %s\n", line);
    }
}

static void command_task(void *arg)
{
    char line[64];
    while (1)
    {
        if (fgets(line, sizeof(line), stdin) != NULL)
            handle_command(line);
    }
}

/* ============================================================================
 * LAZO DE CONTROL: ENCODER -> PD(+I) -> DAC -> PUSH-PULL
 * ==========================================================================*/

static void control_task(void *arg)
{
    pcnt_unit_handle_t unit = (pcnt_unit_handle_t)arg;
    const float T = SAMPLE_PERIOD_MS / 1000.0f;

    int zero_counts = encoder_get_total_counts(unit);
    ESP_LOGI(TAG, "Cero: %d cuentas = 0 mm", zero_counts);

    int prev_counts = 0;
    float v_filt = 0.0f, integ = 0.0f;
    float rho = 0.0f, xh1 = 0.0f, xh2 = 0.0f, u_obs = 0.0f;
    int log_decim = 0, fallos_i2c = 0, n_reves = 0, n_trabado = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1)
    {
        int64_t now_us = esp_timer_get_time();

        if (s_zero_request)
        {
            s_zero_request = false;
            zero_counts = encoder_get_total_counts(unit);
            prev_counts = 0;
            v_filt = 0.0f;
            integ = 0.0f;
            rho = 0.0f;
            xh1 = xh2 = 0.0f;
        }
        bool reset_obs = false;
        if (s_reset_integral)
        {
            s_reset_integral = false;
            integ = 0.0f;
            rho = 0.0f;
            reset_obs = true;
        }

        dir_mode_t modo = s_dir_mode;
        float dir = (modo == DIR_MODE_NEG) ? -1.0f : 1.0f;
        int counts = encoder_get_total_counts(unit) - zero_counts;
        float x = dir * counts * MM_PER_COUNT;
        float v_raw = dir * (counts - prev_counts) * MM_PER_COUNT / T;
        prev_counts = counts;
        v_filt += VEL_EMA_ALPHA * (v_raw - v_filt);

        ctrl_mode_t ctrl = s_ctrl;
        if (ctrl != CTRL_STOP && fabsf(x) > X_SAFE_MM)
        {
            s_ctrl = ctrl = CTRL_STOP;
            printf("ERR,|x| > %.0f mm: motor parado (STOP). Revisa MODE o la mecánica\n", X_SAFE_MM);
        }

        /* Observador (siempre corre, así se puede comparar con lo medido):
         *   xh' = (A - L C) xh + B u + L y,  Euler hacia adelante */
        if (reset_obs)
        {
            xh1 = x;
            xh2 = v_filt;
        }
        else
        {
            float d1 = -OBS_L1 * xh1 + xh2 + OBS_L1 * x;
            float d2 = (-PLANT_A2 - OBS_L2) * xh1 - PLANT_A1 * xh2 + PLANT_B * u_obs + OBS_L2 * x;
            xh1 += T * d1;
            xh2 += T * d2;
        }

        float r = s_ref_mm;
        float vm = 0.0f;
        float u_ef = 0.0f; // voltaje efectivo (sin zona muerta) que pidió la ley
        ley_t ley = s_ley;
        if (ctrl == CTRL_CLOSED && ley != LEY_PID)
        {
            float e = r - x;
            float xs = (ley == LEY_OBS) ? xh1 : x;
            float vs = (ley == LEY_OBS) ? xh2 : v_filt;
            float rho_try = rho + T * e;
            float u = -s_k1 * xs - s_k2 * vs + s_kie * rho_try;
            float comp = (u > U0_MIN_U) ? U0P : (u < -U0_MIN_U) ? -U0N : 0.0f;
            /* anti-windup: no integra si la salida ya está saturada hacia el error */
            if (fabsf(u + comp) < VM_MAX || u * e < 0.0f)
                rho = rho_try;
            u = -s_k1 * xs - s_k2 * vs + s_kie * rho;
            comp = (u > U0_MIN_U) ? U0P : (u < -U0_MIN_U) ? -U0N : 0.0f;
            vm = u + comp;
            u_ef = u;
        }
        else if (ctrl == CTRL_CLOSED)
        {
            float e = r - x;
            float vm_pd = s_kp * e - s_kd * v_filt;
            float vm_try = vm_pd + s_ki * (integ + e * T);
            /* anti-windup: solo integra si la salida no está saturada */
            if (fabsf(vm_try) < VM_MAX)
                integ += e * T;
            vm = vm_pd + s_ki * integ;
        }
        else if (ctrl == CTRL_OPEN)
        {
            vm = s_open_vm;
        }
        if (vm > VM_MAX)
            vm = VM_MAX;
        if (vm < -VM_MAX)
            vm = -VM_MAX;
        /* entrada efectiva del observador: lo que de verdad sale, sin la zona muerta */
        if (ctrl == CTRL_CLOSED && ley != LEY_PID)
            u_obs = (u_ef > U0_MIN_U) ? vm - U0P : (u_ef < -U0_MIN_U) ? vm + U0N : vm;
        else
            u_obs = (vm > 0.0f) ? fmaxf(vm - U0P, 0.0f) : (vm < 0.0f) ? fminf(vm + U0N, 0.0f) : 0.0f;

        /* Protecciones del lazo cerrado:
         *  - se aleja de la referencia empujando a fondo hacia ella durante
         *    200 ms -> el sentido está invertido (MODE o cables del motor)
         *  - empuja a fondo sin moverse durante 1.5 s -> tope o atasco */
        if (ctrl == CTRL_CLOSED)
        {
            float e = r - x;
            bool saturado = fabsf(vm) >= VM_MAX * 0.99f;
            bool al_reves = saturado && fabsf(e) > 1.0f && e * v_filt < 0.0f && fabsf(v_filt) > 5.0f;
            bool trabado = saturado && fabsf(v_filt) < 0.5f;
            n_reves = al_reves ? n_reves + 1 : 0;
            n_trabado = trabado ? n_trabado + 1 : 0;
            if (n_reves >= 200 / SAMPLE_PERIOD_MS || n_trabado >= 1500 / SAMPLE_PERIOD_MS)
            {
                s_ctrl = ctrl = CTRL_STOP;
                vm = 0.0f;
                printf("ERR,%s: motor parado (STOP)\n",
                       n_reves ? "Se aleja de la referencia, sentido invertido? prueba MODE POS/NEG"
                               : "Voltaje maximo sin moverse: tope o atasco");
                n_reves = n_trabado = 0;
            }
        }
        else
        {
            n_reves = n_trabado = 0;
        }

        uint16_t raw = dac_raw_for_vm(vm);

        /* Qué sale por cada DAC: el del push-pull lleva Vm; los demás, la señal
         * física del montaje analógico (o 2048 durante la detección con SOLO). */
        bool prueba = s_test_mode;
        float sig_v[3] = {
            prueba ? s_test_p_v : x * SX_V_PER_MM,
            prueba ? s_test_v_v : v_filt * SV_V_PER_MMS,
            r * SX_V_PER_MM,
        };
        uint16_t out[3];
        int salida = s_salida, solo = s_solo;
        for (int i = 0; i < 3; i++)
        {
            if (solo)
                out[i] = (solo == i + 1) ? raw : DAC_MID_RAW;
            else if (salida == 0 || salida == i + 1)
                out[i] = raw;
            else
            {
                long c = lroundf(sig_v[i] / s_mcp_vdd * 4096.0f);
                out[i] = (uint16_t)(c < 0 ? 0 : (c > MCP4725_MAX_RAW ? MCP4725_MAX_RAW : c));
            }
        }
        s_dac_raw = out[2];
        esp_err_t err = ESP_OK;
        for (int i = 0; i < 2; i++)
        {
            if (s_mcp[i])
            {
                esp_err_t e2 = mcp4725_set_raw(s_mcp[i], out[i]);
                if (e2 != ESP_OK)
                    err = e2;
            }
        }
        if (err != ESP_OK && ++fallos_i2c % 100 == 1)
            printf("ERR,Fallo de escritura I2C (%d veces): %s\n", fallos_i2c, esp_err_to_name(err));

        if (++log_decim >= LOG_DECIMATION_FACTOR)
        {
            log_decim = 0;
            /* 12 primeros campos: los de la interfaz (vr, P y V que salen de
             * verdad por los DAC); después: Vm, código del motor, observador,
             * modo de control y ley */
            printf("DATA,%.1f,%.3f,%.2f,%.3f,%.4f,%u,%.3f,%u,%.3f,%d,%d,%.3f,%u,%.3f,%.2f,%d,%d\n",
                   now_us / 1000.0, (double)x, (double)v_filt, (double)r,
                   (double)fminf(fmaxf(sig_v[2], 0.0f), s_mcp_vdd), /* Vr = SX·r */
                   out[0], (double)dac_v_for_raw(out[0]), out[1], (double)dac_v_for_raw(out[1]),
                   (int)modo, (int)prueba,
                   (double)vm, raw, (double)xh1, (double)xh2, (int)ctrl, (int)ley);
        }

        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/* ============================================================================
 * FUNCIÓN PRINCIPAL
 * ==========================================================================*/

void app_main(void)
{
    uart_console_init();
    ESP_LOGI(TAG, "Iniciando control digital...");

    pcnt_unit_handle_t encoder_unit = NULL;
    esp_err_t err = encoder_pcnt_init(&encoder_unit);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error inicializando PCNT: %s", esp_err_to_name(err));
        return;
    }

    for (int i = 0; i < 2; i++)
    {
        if (mcp4725_salida_init(i) != ESP_OK)
            ESP_LOGW(TAG, "MCP4725 #%d no encontrado (SDA=%d, SCL=%d)", i + 1, s_mcp_cfg[i].sda_gpio, s_mcp_cfg[i].scl_gpio);
    }

    err = espnow_init();
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Error iniciando ESP-NOW: %s", esp_err_to_name(err));

    ESP_LOGI(TAG, "mm por cuenta: %.5f  |  Vm = %.1f·(DAC − 1.65 V), DAC medio = %d", MM_PER_COUNT, PP_GAIN, DAC_MID_RAW);
    ESP_LOGI(TAG, "Realimentacion de estados: K1=%.4f K2=%.5f Ki=%.4f, observador L=[%.1f %.1f]  |  r = 0 mm",
             SF_K1, SF_K2, SF_KI, OBS_L1, OBS_L2);
    ESP_LOGI(TAG, "Comandos: REF <mm> | ZERO | LEY PID|EST|OBS | SFK k1 k2 ki | KP/KD/KI <v> | OPEN <V> | STOP | RUN | MODE POS|NEG | MID <raw> | VDD <mV> | CFG");
    print_cfg();

    xTaskCreate(control_task, "control", 4096, encoder_unit, 6, NULL);
    xTaskCreate(command_task, "command_task", 3072, NULL, 4, NULL);
    if (err == ESP_OK)
        xTaskCreate(link_task, "link_task", 3072, NULL, 5, NULL);
}
