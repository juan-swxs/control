/*
 * ============================================================================
 *      MEDICIÓN DE DESPLAZAMIENTO Y SALIDA DAC PARA PID ANALÓGICO
 *      ESP32 - ESP-IDF >= 5.0
 *
 *  La lectura del encoder, el cálculo de posición/velocidad y el DAC
 *  automático (GPIO25) mantienen la misma lógica original. Se agrega:
 *    - Segundo DAC (GPIO26) controlado por comandos seriales, con techo
 *      real de voltaje aplicado mediante DAC_VREF.
 *    - Modo de dirección seleccionable (POS/NEG) para GPIO25, porque el
 *      recorrido físico del rack es distinto en cada sentido, con VREF
 *      independiente por modo (DAC_VREF_POS / DAC_VREF_NEG).
 *
 *  SALIDA ANALÓGICA 1 (automática, PID externo) — GPIO25:
 *      Modo POS: solo mapea x_mm > 0, usando X_MAX_POS_MM y DAC_VREF_POS.
 *      Modo NEG: solo mapea x_mm < 0 (en magnitud), usando X_MAX_NEG_MM
 *                y DAC_VREF_NEG.
 *      El sentido contrario al modo activo se reporta como 0V.
 *
 *  SALIDA ANALÓGICA 2 (manual) — GPIO26:
 *      0 a DAC_VREF voltios reales, fijado por comando serial RAW/MV.
 *      DAC_VREF actúa como techo físico real (antes solo era decorativo).
 *
 *  TELEMETRÍA (ESP32 -> PC), cada 50 ms:
 *      DATA,t_ms,x_mm,vel_mm_s,dac_auto_raw,dac_auto_v,dac_man_raw,dac_man_v,modo,xmax_mm
 *      modo: 0 = positivo, 1 = negativo
 *      Los voltajes reportados son siempre sobre la escala física real
 *      del hardware (DAC_HW_FULL_SCALE), para que coincidan con lo que
 *      mide un multímetro.
 *
 *  COMANDOS (PC -> ESP32), una línea terminada en '\n':
 *      RAW <0-255>   MV <0-3300>   ZERO   MODE POS   MODE NEG
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
#include "driver/dac_oneshot.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"

#include "mcp4725.h"

/* ============================================================================
 * TAG Y PINES
 * ==========================================================================*/

static const char *TAG = "PINON_CREMALLERA_ENC";

#define ENCODER_GPIO_A 34
#define ENCODER_GPIO_B 35
#define PCNT_GLITCH_FILTER_NS 1000

/* ============================================================================
 * PARÁMETROS MECÁNICOS Y ESCALAMIENTO DAC
 * ==========================================================================*/

#define ENCODER_PPR 11.0f    // Pulsos por revolución motor
#define GEAR_RATIO 20.0f     // Reducción 20:1
#define QUAD_MULTIPLIER 4.0f // Modo x4

#define COUNTS_PER_REV (ENCODER_PPR * GEAR_RATIO * QUAD_MULTIPLIER)
#define PINION_RADIUS_MM 9.4f
#define MM_PER_COUNT ((2.0f * (float)M_PI * PINION_RADIUS_MM) / COUNTS_PER_REV)

#define X_MAX_POS_MM 13.0f // Máximo recorrido en sentido positivo
#define X_MAX_NEG_MM 13.0f // Máximo recorrido en sentido negativo

#define DAC_VREF_POS 3.3f // Voltaje máximo de salida en modo POSITIVO (GPIO25)
#define DAC_VREF_NEG 3.3f // Voltaje máximo de salida en modo NEGATIVO (GPIO25)

/* Escala física real del hardware DAC del ESP32 (fija, no la toques) */
#define DAC_HW_FULL_SCALE 3.3f

/* DAC manual (GPIO26) — techo real de voltaje, ahora sí aplicado */
#define DAC_VREF 3.3f

/* ============================================================================
 * RANGO DE HARDWARE PCNT (SIN CAMBIOS)
 * ==========================================================================*/

#define PCNT_HIGH_LIMIT 30000
#define PCNT_LOW_LIMIT -30000

/* ============================================================================
 * PARÁMETROS DE MUESTREO Y FILTRO (SIN CAMBIOS)
 * ==========================================================================*/

#define SAMPLE_PERIOD_MS 10
#define LOG_PERIOD_MS 50
#define LOG_DECIMATION_FACTOR (LOG_PERIOD_MS / SAMPLE_PERIOD_MS)

#define VEL_MA_WINDOW 8
#define VEL_EMA_ALPHA 0.6

static double s_vel_ma_buf[VEL_MA_WINDOW];
static int s_vel_ma_idx = 0;
static bool s_vel_ma_full = false;
static double s_vel_filtered = 0.0;

// Handles de los dos canales DAC
static dac_oneshot_handle_t s_dac_handle = NULL;        // GPIO25 -> automático
static dac_oneshot_handle_t s_dac_manual_handle = NULL; // GPIO26 -> manual

// Valor manual actual, escrito por command_task y leído en la telemetría
static volatile uint8_t s_dac_manual = 0;

// Bandera para recalibrar el cero, activada por el comando ZERO
static volatile bool s_zero_request = false;

/* Modo de dirección activo para el DAC automático (GPIO25) */
typedef enum
{
    DIR_MODE_POS = 0, // Mapea solo desplazamientos positivos
    DIR_MODE_NEG = 1, // Mapea solo desplazamientos negativos (default)
} dir_mode_t;

static volatile dir_mode_t s_dir_mode = DIR_MODE_NEG;

static double vel_moving_average(double new_sample)
{
    s_vel_ma_buf[s_vel_ma_idx] = new_sample;
    s_vel_ma_idx = (s_vel_ma_idx + 1) % VEL_MA_WINDOW;

    if (s_vel_ma_idx == 0)
    {
        s_vel_ma_full = true;
    }

    int n = s_vel_ma_full ? VEL_MA_WINDOW : s_vel_ma_idx;

    if (n == 0)
    {
        return new_sample;
    }

    double sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        sum += s_vel_ma_buf[i];
    }

    return sum / n;
}

/* ============================================================================
 * ACUMULADOR Y CONFIGURACIÓN PCNT (SIN CAMBIOS)
 * ==========================================================================*/

static volatile int64_t s_pulse_accum = 0;

static bool IRAM_ATTR pcnt_on_reach_callback(pcnt_unit_handle_t unit,
                                             const pcnt_watch_event_data_t *edata,
                                             void *user_ctx)
{
    s_pulse_accum += edata->watch_point_value;
    return false;
}

static esp_err_t encoder_pcnt_init(pcnt_unit_handle_t *out_unit)
{
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

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_on_reach_callback,
    };
    ESP_RETURN_ON_ERROR(pcnt_unit_register_event_callbacks(unit, &cbs, NULL), TAG, "Error registro callbacks");

    ESP_RETURN_ON_ERROR(pcnt_unit_enable(unit), TAG, "Error habilitando PCNT");
    ESP_RETURN_ON_ERROR(pcnt_unit_clear_count(unit), TAG, "Error limpiando contador");
    ESP_RETURN_ON_ERROR(pcnt_unit_start(unit), TAG, "Error iniciando PCNT");

    *out_unit = unit;
    return ESP_OK;
}

static inline int64_t encoder_get_total_counts(pcnt_unit_handle_t unit)
{
    int pulse_count = 0;
    pcnt_unit_get_count(unit, &pulse_count);
    return s_pulse_accum + pulse_count;
}

/* ============================================================================
 * INICIALIZACIÓN DE LOS DOS DAC (SIN CAMBIOS)
 * ==========================================================================*/

static esp_err_t dac_output_init(void)
{
    dac_oneshot_config_t dac_cfg = {
        .chan_id = DAC_CHAN_0, // DAC_CHAN_0 asigna la salida física al GPIO25
    };

    ESP_RETURN_ON_ERROR(dac_oneshot_new_channel(&dac_cfg, &s_dac_handle), TAG, "Error creando canal DAC");

    dac_oneshot_config_t dac_manual_cfg = {
        .chan_id = DAC_CHAN_1,
    };
    ESP_RETURN_ON_ERROR(dac_oneshot_new_channel(&dac_manual_cfg, &s_dac_manual_handle),
                        TAG, "Error creando canal DAC manual");
    ESP_RETURN_ON_ERROR(dac_oneshot_output_voltage(s_dac_manual_handle, 0),
                        TAG, "Error inicializando DAC manual en 0");

    return ESP_OK;
}

/* MCP4725 desactivado por el momento: cambiar a #if 1 para reactivarlo */
#if 0
/* ============================================================================
 * DOS BUSES I2C + DOS DAC EXTERNOS MCP4725
 *
 *  Ambos módulos tienen la misma dirección (0x60, A0 fijo a GND), por eso
 *  cada uno va en su propio controlador I2C:
 *      MCP4725 #1 -> I2C_NUM_0, SDA=GPIO21, SCL=GPIO22 -> MCP4725_1_V
 *      MCP4725 #2 -> I2C_NUM_1, SDA=GPIO19, SCL=GPIO18 -> MCP4725_2_V
 * ==========================================================================*/

#define MCP4725_VDD 3.3f // Los módulos se alimentan a 3.3V (Vdd = referencia)

typedef struct
{
    i2c_port_num_t port;
    int sda_gpio;
    int scl_gpio;
    float voltaje;
} mcp4725_salida_cfg_t;

static const mcp4725_salida_cfg_t s_mcp_cfg[2] = {
    {.port = I2C_NUM_0, .sda_gpio = 21, .scl_gpio = 22, .voltaje = 2.0f},
    {.port = I2C_NUM_1, .sda_gpio = 19, .scl_gpio = 18, .voltaje = 1.0f},
};

static i2c_master_bus_handle_t s_i2c_bus[2] = {NULL, NULL};
static mcp4725_handle_t s_mcp[2] = {NULL, NULL};

/* Recorre todas las direcciones de 7 bits de un bus y devuelve la primera
 * que responde en el rango del MCP4725 (0x60-0x67), o 0 si no hay ninguna. */
static uint8_t i2c_scan_mcp4725(int idx)
{
    uint8_t found = 0;
    int n_total = 0;
    ESP_LOGI(TAG, "Escaneando bus I2C %d (SDA=%d, SCL=%d)...",
             idx, s_mcp_cfg[idx].sda_gpio, s_mcp_cfg[idx].scl_gpio);
    for (uint8_t addr = 0x08; addr < 0x78; addr++)
    {
        if (i2c_master_probe(s_i2c_bus[idx], addr, 50) == ESP_OK)
        {
            n_total++;
            bool es_mcp = (addr >= 0x60 && addr <= 0x67);
            ESP_LOGI(TAG, "  Dispositivo en 0x%02X%s", addr, es_mcp ? " (MCP4725)" : "");
            if (es_mcp && found == 0)
            {
                found = addr;
            }
        }
    }
    ESP_LOGI(TAG, "Escaneo bus %d terminado: %d dispositivo(s)", idx, n_total);
    return found;
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

    uint8_t addr = i2c_scan_mcp4725(idx);
    ESP_RETURN_ON_FALSE(addr != 0, ESP_ERR_NOT_FOUND, TAG, "No hay MCP4725 en el bus %d", idx);

    mcp4725_config_t cfg = MCP4725_CONFIG_DEFAULT();
    cfg.address = addr;
    cfg.vref = MCP4725_VDD;
    ESP_RETURN_ON_ERROR(mcp4725_create(s_i2c_bus[idx], &cfg, &s_mcp[idx]), TAG, "Error creando MCP4725 #%d", idx + 1);
    ESP_RETURN_ON_ERROR(mcp4725_set_voltage(s_mcp[idx], sc->voltaje), TAG, "Error escribiendo MCP4725 #%d", idx + 1);

    uint16_t raw = 0;
    float v_leido = 0.0f;
    mcp4725_get_raw(s_mcp[idx], &raw);
    mcp4725_get_voltage(s_mcp[idx], &v_leido);
    ESP_LOGI(TAG, "MCP4725 #%d (bus %d, 0x%02X): pedido %.3fV -> raw %u, leído %.3fV",
             idx + 1, idx, addr, sc->voltaje, raw, v_leido);
    return ESP_OK;
}

/* Inicializa ambos módulos; un fallo en uno no impide configurar el otro */
static esp_err_t mcp4725_outputs_init(void)
{
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < 2; i++)
    {
        esp_err_t err = mcp4725_salida_init(i);
        if (err != ESP_OK)
        {
            ret = err;
        }
    }
    return ret;
}
#endif /* MCP4725 */

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
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 512, 512, 0, NULL, 0));
    uart_vfs_dev_use_driver(UART_NUM_0);
}

/* Convierte un voltaje deseado (0 a DAC_VREF) al raw real del hardware,
 * aplicando DAC_VREF como techo físico del DAC manual (GPIO26). Antes
 * DAC_VREF no se usaba aquí y por eso el pin siempre podía llegar a
 * 3.3V reales sin importar su valor. */
static inline uint8_t dac_manual_raw_for_voltage(float v_deseado)
{
    if (v_deseado < 0.0f)     v_deseado = 0.0f;
    if (v_deseado > DAC_VREF) v_deseado = DAC_VREF;   // <- techo real aplicado aquí

    float raw_f = (v_deseado / DAC_HW_FULL_SCALE) * 255.0f;
    if (raw_f > 255.0f) raw_f = 255.0f;
    return (uint8_t)lroundf(raw_f);
}

static void handle_command(char *line)
{
    size_t len = strlen(line);
    while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r' || line[len - 1] == ' '))
    {
        line[--len] = '\0';
    }
    if (len == 0)
        return;

    if (strncmp(line, "RAW", 3) == 0)
    {
        int raw_in = atoi(line + 3);
        if (raw_in < 0)   raw_in = 0;
        if (raw_in > 255) raw_in = 255;

        /* El raw pedido (0-255) se interpreta sobre la escala física real,
         * y luego se recorta a DAC_VREF si excede el techo configurado. */
        float   v_deseado = (raw_in / 255.0f) * DAC_HW_FULL_SCALE;
        uint8_t raw_c     = dac_manual_raw_for_voltage(v_deseado);

        dac_oneshot_output_voltage(s_dac_manual_handle, raw_c);
        s_dac_manual = raw_c;
    }
    else if (strncmp(line, "MV", 2) == 0)
    {
        int mv = atoi(line + 2);
        if (mv < 0)    mv = 0;
        if (mv > 3300) mv = 3300;

        float   v_deseado = mv / 1000.0f;
        uint8_t raw_c     = dac_manual_raw_for_voltage(v_deseado);

        dac_oneshot_output_voltage(s_dac_manual_handle, raw_c);
        s_dac_manual = raw_c;
    }
    else if (strcmp(line, "ZERO") == 0)
    {
        s_zero_request = true;
    }
    else if (strncmp(line, "MODE", 4) == 0)
    {
        char *arg = line + 4;
        while (*arg == ' ')
            arg++;

        if (strcmp(arg, "POS") == 0)
        {
            s_dir_mode = DIR_MODE_POS;
            ESP_LOGI(TAG, "Modo de dirección: POSITIVO (xmax=%.2f mm, vref=%.2fV)", X_MAX_POS_MM, DAC_VREF_POS);
        }
        else if (strcmp(arg, "NEG") == 0)
        {
            s_dir_mode = DIR_MODE_NEG;
            ESP_LOGI(TAG, "Modo de dirección: NEGATIVO (xmax=%.2f mm, vref=%.2fV)", X_MAX_NEG_MM, DAC_VREF_NEG);
        }
    }
}

static void command_task(void *arg)
{
    char line[64];
    while (1)
    {
        if (fgets(line, sizeof(line), stdin) != NULL)
        {
            handle_command(line);
        }
    }
}

/* ============================================================================
 * TAREA DE MUESTREO Y ACTUALIZACIÓN DEL DAC
 * ==========================================================================*/

static void position_sampling_task(void *arg)
{
    pcnt_unit_handle_t unit = (pcnt_unit_handle_t)arg;

    int64_t referencia_counts = encoder_get_total_counts(unit);
    ESP_LOGI(TAG, "Referencia calibrada: %lld cuentas = 0 mm", referencia_counts);

    float x_prev_mm = 0.0f;
    int64_t t_prev_us = esp_timer_get_time();
    int log_decim = 0;
    bool primera_muestra = true;

    while (1)
    {
        int64_t now_us = esp_timer_get_time();
        double dt_s = (now_us - t_prev_us) / 1e6;
        t_prev_us = now_us;

        /* Recalibrado del cero (disparado por comando ZERO) */
        if (s_zero_request)
        {
            s_zero_request = false;
            referencia_counts = encoder_get_total_counts(unit);
            x_prev_mm = 0.0f;
            s_vel_filtered = 0.0;
            s_vel_ma_idx = 0;
            s_vel_ma_full = false;
            primera_muestra = true;
        }

        // Cálculo de posición con signo (conserva dirección positiva y negativa)
        int64_t counts = encoder_get_total_counts(unit) - referencia_counts;
        float x_mm = (float)counts * MM_PER_COUNT;

        /* Según el modo activo, toma solo la componente de x_mm en ese
         * sentido (la contraria se reporta como 0V) y usa el máximo y el
         * VREF propios de esa dirección. */
        dir_mode_t modo_actual = s_dir_mode;
        float xmax_activo;
        float x_efectivo;
        float vref_activo;

        if (modo_actual == DIR_MODE_POS)
        {
            xmax_activo = X_MAX_POS_MM;
            vref_activo = DAC_VREF_POS;
            x_efectivo = (x_mm > 0.0f) ? x_mm : 0.0f;
        }
        else /* DIR_MODE_NEG */
        {
            xmax_activo = X_MAX_NEG_MM;
            vref_activo = DAC_VREF_NEG;
            x_efectivo = (x_mm < 0.0f) ? -x_mm : 0.0f;
        }

        float dac_norm = x_efectivo / xmax_activo; // Normalización respecto al máximo activo
        if (dac_norm > 1.0f)
            dac_norm = 1.0f;

        /* Voltaje deseado según el VREF del modo activo, convertido a raw
         * sobre la escala física real del hardware (3.3V) */
        float v_deseado = dac_norm * vref_activo;
        int dac_val_i = (int)lroundf((v_deseado / DAC_HW_FULL_SCALE) * 255.0f);
        if (dac_val_i < 0)
            dac_val_i = 0;
        if (dac_val_i > 255)
            dac_val_i = 255;
        uint8_t dac_val = (uint8_t)dac_val_i;

        // Actualizar canal DAC (GPIO25)
        dac_oneshot_output_voltage(s_dac_handle, dac_val);

        /* Cálculo de velocidad (Media móvil + EMA) — SIN CAMBIOS */
        double velocity_mm_s = 0.0;

        if (!primera_muestra && dt_s > 0)
        {
            double v_raw = (double)(x_mm - x_prev_mm) / dt_s;
            double v_ma = vel_moving_average(v_raw);

            s_vel_filtered = VEL_EMA_ALPHA * v_ma + (1.0 - VEL_EMA_ALPHA) * s_vel_filtered;
            velocity_mm_s = s_vel_filtered;
        }

        x_prev_mm = x_mm;
        primera_muestra = false;

        /* Telemetría — ambos DAC se reportan sobre la escala física real
         * del hardware (DAC_HW_FULL_SCALE), no sobre DAC_VREF, para que
         * coincidan con lo que mide un multímetro. */
        log_decim++;
        if (log_decim >= LOG_DECIMATION_FACTOR)
        {
            log_decim = 0;

            uint8_t dac_man_val = s_dac_manual;

            printf("DATA,%.3f,%.3f,%.3f,%u,%.3f,%u,%.3f,%d,%.2f\n",
                   now_us / 1000.0,
                   x_mm,
                   velocity_mm_s,
                   dac_val,     (dac_val     * DAC_HW_FULL_SCALE) / 255.0f,
                   dac_man_val, (dac_man_val * DAC_HW_FULL_SCALE) / 255.0f,
                   (int)modo_actual,
                   xmax_activo);
        }

        vTaskDelay(pdMS_TO_TICKS(SAMPLE_PERIOD_MS));
    }
}

/* ============================================================================
 * FUNCIÓN PRINCIPAL
 * ==========================================================================*/

void app_main(void)
{
    uart_console_init();

    ESP_LOGI(TAG, "Iniciando sistema...");

    pcnt_unit_handle_t encoder_unit = NULL;
    esp_err_t err = encoder_pcnt_init(&encoder_unit);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error inicializando PCNT: %s", esp_err_to_name(err));
        return;
    }

    err = dac_output_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error inicializando DAC: %s", esp_err_to_name(err));
        return;
    }

    /* MCP4725 desactivado por el momento
    err = mcp4725_outputs_init();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error inicializando MCP4725: %s", esp_err_to_name(err));
    }
    */

    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "   SISTEMA CONVERSIÓN ENCODER A DAC (PID)");
    ESP_LOGI(TAG, "==========================================");
    ESP_LOGI(TAG, "Cuentas por revolucion (x4): %.0f", COUNTS_PER_REV);
    ESP_LOGI(TAG, "Radio del pinon: %.2f mm", PINION_RADIUS_MM);
    ESP_LOGI(TAG, "mm por cuenta: %.5f", MM_PER_COUNT);
    ESP_LOGI(TAG, "DAC automatico GPIO25 -> modo NEGATIVO por defecto (xmax=%.2f mm, vref=%.2fV)", X_MAX_NEG_MM, DAC_VREF_NEG);
    ESP_LOGI(TAG, "DAC manual     GPIO26 (comando serial, 0 a %.2fV)", DAC_VREF);
    ESP_LOGI(TAG, "Comandos: RAW <0-255> | MV <0-3300> | ZERO | MODE POS | MODE NEG");
    ESP_LOGI(TAG, "==========================================");

    xTaskCreate(position_sampling_task, "position_sampling", 4096, encoder_unit, 5, NULL);
    xTaskCreate(command_task, "command_task", 3072, NULL, 5, NULL);

    ESP_LOGI(TAG, "Sistema iniciado correctamente.");
}