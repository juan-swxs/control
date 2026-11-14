/**
 * @file lab_link.h
 * @brief Protocolo ESP-NOW entre el ESP32 principal (encoder, P y V) y el
 *        ESP32 de referencia (MCP4725 #3 -> Vr).
 *
 *  Los tres MCP4725 tienen la misma dirección I2C (0x60), por eso el de la
 *  referencia va en otra placa y recibe el voltaje por radio.
 *
 *  Principal -> referencia: LINK_SET_REF cada vez que cambia r y, además,
 *                           cada LINK_RESEND_MS (por si la otra placa se reinicia).
 *  Referencia -> principal: LINK_STATUS al aplicar cada SET_REF y cada
 *                           LINK_RESEND_MS, con lo que realmente quedó en el DAC.
 *
 *  Ambos lados usan broadcast y se "emparejan" con la primera placa que les
 *  habla: así otro grupo del laboratorio con el mismo firmware no interfiere.
 */
#pragma once

#include <stdint.h>

#define LAB_LINK_MAGIC     0x4C34u  /* "L4" */
#define LAB_LINK_VERSION   1
#define LAB_LINK_CHANNEL   1        /* canal WiFi fijo en ambas placas */
/* Cada transmisión saca picos de ~250 mA del 3.3 V, y el MCP4725 usa ese
 * mismo 3.3 V como referencia: menos envíos y menos potencia = Vr más limpio.
 * Un cambio de r se manda al instante; esto es solo el repaso periódico. */
#define LAB_LINK_RESEND_MS 1000
#define LAB_LINK_TIMEOUT_MS 3500    /* sin STATUS en este tiempo = enlace caído */
#define LAB_LINK_TX_POWER_QDBM 34   /* 8.5 dBm (unidades de 0.25 dBm); las placas están cerca */

typedef enum
{
    LINK_SET_REF = 1,
    LINK_STATUS = 2,
} lab_link_type_t;

typedef struct __attribute__((packed))
{
    uint16_t magic;   /* LAB_LINK_MAGIC */
    uint8_t version;  /* LAB_LINK_VERSION */
    uint8_t type;     /* lab_link_type_t */
    uint32_t seq;     /* SET_REF: contador; STATUS: último seq aplicado */
    float volts;      /* SET_REF: Vr pedido; STATUS: Vr que sale (leído del DAC) */
    float vdd;        /* SET_REF: Vdd para la escala; STATUS: Vdd usado */
    uint16_t raw;     /* STATUS: registro del DAC leído (0-4095) */
    uint8_t mcp_ok;   /* STATUS: 1 si el MCP4725 #3 responde */
    uint8_t reserved;
} lab_link_msg_t;
