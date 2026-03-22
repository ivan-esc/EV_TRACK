#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "Globals.h"
#include "Kalman2D.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include "driver/gpio.h"
#include "math.h"

/* ---------- protocol constants ---------- */

#define FOC_UART_SOF   0xAA

/* ---------- command IDs ---------- */

typedef enum
{
    FOC_CMD_GET_STATUS          = 0x01,
    FOC_CMD_GET_BUS_VOLTAGE     = 0x02,
    FOC_CMD_GET_BUS_CURRENT     = 0x03,
    FOC_CMD_GET_SPEED           = 0x04,
    FOC_CMD_GET_POWER           = 0x05,
    FOC_CMD_GET_ALL_FAST        = 0x06,
    FOC_CMD_GET_IQ              = 0x07,
    FOC_CMD_GET_ID              = 0x08,
    FOC_CMD_GET_PROTOCOL_VER    = 0x09,
    FOC_CMD_SET_IQ_PI_GAINS     = 0x20,
    FOC_CMD_SET_ID_PI_GAINS     = 0x21
} foc_uart_cmd_t;

/* ---------- data structures ---------- */

typedef struct __attribute__((packed))
{
    uint16_t vbus_mV;
    int32_t  ibus_mA;
    int32_t  rpm;
    uint16_t throttle_raw;
    uint16_t fault_flags;
} foc_all_fast_t;

typedef struct __attribute__((packed))
{
    uint16_t fault_flags;
    uint8_t  state;
} foc_status_t;

/* ---------- API ---------- */

void foc_uart_init(void);

/* low level */
bool foc_uart_send_cmd(foc_uart_cmd_t cmd,
                        const uint8_t *payload,
                        uint8_t payload_len);

bool foc_uart_receive_reply(foc_uart_cmd_t expected_cmd,
                            uint8_t *payload,
                            uint8_t max_payload_len,
                            uint8_t *out_payload_len,
                            uint32_t timeout_ms);

/* high level helpers */
bool foc_uart_get_all_fast(foc_all_fast_t *out);
bool foc_uart_get_status(foc_status_t *out);

bool foc_uart_get_bus_voltage(uint16_t *vbus_mV);
bool foc_uart_get_bus_current(int32_t *ibus_mA);
bool foc_uart_get_speed(int32_t *rpm);

bool foc_uart_get_iq(int16_t *iq_mA);
bool foc_uart_get_id(int16_t *id_mA);

bool foc_uart_set_iq_pi_gains(int32_t kp, int32_t ki, uint8_t *result);
bool foc_uart_set_id_pi_gains(int32_t kp, int32_t ki, uint8_t *result);

/* Task */
void foc_uart_test_task(void *arg);