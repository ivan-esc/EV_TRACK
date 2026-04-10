#include "FOC_UART.h"

/* ---------- local helpers ---------- */

static uint8_t crc8_atm(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;

    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t b = 0; b < 8; b++)
        {
            if (crc & 0x80)
                crc = (crc << 1 ) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/* ---------- init ---------- */

void foc_uart_init(void)
{
    uart_config_t cfg =
    {
        .baud_rate  = FOC_DRIVER_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    uart_driver_install(FOC_DRIVER_UART_CHANNEL, 256, 256, 0, NULL, 0);
    uart_param_config(FOC_DRIVER_UART_CHANNEL, &cfg);

    uart_set_pin(FOC_DRIVER_UART_CHANNEL,
                 FOC_DRIVER_TX_GPIO,
                 FOC_DRIVER_RX_GPIO,
                 UART_PIN_NO_CHANGE,
                 UART_PIN_NO_CHANGE);

}

/* ---------- low level send ---------- */

bool foc_uart_send_cmd(foc_uart_cmd_t cmd,
                        const uint8_t *payload,
                        uint8_t payload_len)
{
    uint8_t frame[64];

    uint8_t len = 1 + payload_len;   /* CMD + payload */

    frame[0] = FOC_UART_SOF;
    frame[1] = len;
    frame[2] = (uint8_t)cmd;

    if (payload_len && payload)
        memcpy(&frame[3], payload, payload_len);

    uint8_t crc = crc8_atm(&frame[1], len + 1); /* LEN + CMD + payload */

    frame[3 + payload_len] = crc;

    if (payload_len > 60){
        return false;
    }

    int tx_len = 4 + payload_len;

    int written = uart_write_bytes(FOC_DRIVER_UART_CHANNEL,
                                    (const char *)frame,
                                    tx_len);

    return (written == tx_len);
}

/* ---------- receive one frame ---------- */

bool foc_uart_receive_reply(foc_uart_cmd_t expected_cmd,
                            uint8_t *payload,
                            uint8_t max_payload_len,
                            uint8_t *out_payload_len,
                            uint32_t timeout_ms)
{
    uint8_t b;
    uint32_t t0 = xTaskGetTickCount();

    /* search SOF */
    while (1)
    {

        if ((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS > timeout_ms){
            return false;
        }

        // Read one byte at a time
        int r = uart_read_bytes(FOC_DRIVER_UART_CHANNEL, &b, 1, pdMS_TO_TICKS(10));

        if (r == 1){
            // ESP_LOGI("FOC", "RX byte = 0x%02X", b);

            if (b == FOC_UART_SOF)
            {
                // ESP_LOGI("FOC", "REAL SOF detected");
                break;
            }
        }
    }

    uint8_t len;
    if (uart_read_bytes(FOC_DRIVER_UART_CHANNEL, &len, 1,
                        pdMS_TO_TICKS(timeout_ms)) != 1)
        return false;
        
    if (len < 1)
        return false;

    uint8_t buf[64];

    if (len > sizeof(buf))
        return false;

    if (uart_read_bytes(FOC_DRIVER_UART_CHANNEL, buf, len,
                        pdMS_TO_TICKS(timeout_ms)) != len)
        return false;

    // ESP_LOGI("FOC", "LEN=%u CMD=%u", len, buf[0]);
    // for (int i = 0; i < len; i++)
    //     ESP_LOGI("FOC", "buf[%d]=0x%02X", i, buf[i]);

    uint8_t crc_rx;
    if (uart_read_bytes(FOC_DRIVER_UART_CHANNEL, &crc_rx, 1,
                        pdMS_TO_TICKS(timeout_ms)) != 1)
        return false;


    uint8_t crc_calc;

    uint8_t tmp[65];
    tmp[0] = len;
    memcpy(&tmp[1], buf, len);

    crc_calc = crc8_atm(tmp, len + 1);

    if (crc_calc != crc_rx)
        return false;

    uint8_t cmd = buf[0];

    if (cmd != (uint8_t)expected_cmd)
        return false;

    uint8_t pl_len = len - 1;

    if (pl_len > max_payload_len)
        return false;

    if (pl_len && payload)
        memcpy(payload, &buf[1], pl_len);

    if (out_payload_len)
        *out_payload_len = pl_len;

    return true;
}


/* ---------- high level helpers ---------- */

bool foc_uart_get_all_fast(foc_all_fast_t *out)
{
    uint8_t pl[18];
    uint8_t len;

    if (!foc_uart_send_cmd(FOC_CMD_GET_ALL_FAST, NULL, 0))
        return false;

    if (!foc_uart_receive_reply(FOC_CMD_GET_ALL_FAST, pl, sizeof(pl), &len, 20))
        return false;

    if (len != 14)
        return false;

    memcpy(out, pl, 14);
    // ESP_LOGI("FOC", "FAST struct size = %d", sizeof(foc_all_fast_t));
    return true;
}


bool foc_uart_get_status(foc_status_t *out)
{
    uint8_t pl[8];
    uint8_t len;

    if (!foc_uart_send_cmd(FOC_CMD_GET_STATUS, NULL, 0))
        return false;

    if (!foc_uart_receive_reply(FOC_CMD_GET_STATUS, pl, sizeof(pl), &len, 200))
        return false;

    if (len != 3)
        return false;

    memcpy(out, pl, 3);
    return true;
}

bool foc_uart_get_bus_voltage(uint16_t *vbus_mV)
{
    uint8_t pl[4], len;

    foc_uart_send_cmd(FOC_CMD_GET_BUS_VOLTAGE, NULL, 0);
    if (!foc_uart_receive_reply(FOC_CMD_GET_BUS_VOLTAGE, pl, sizeof(pl), &len, 50))
        return false;

    if (len != 2) return false;

    memcpy(vbus_mV, pl, 2);
    return true;
}

bool foc_uart_get_bus_current(int32_t *ibus_mA)
{
    uint8_t pl[8], len;

    foc_uart_send_cmd(FOC_CMD_GET_BUS_CURRENT, NULL, 0);
    if (!foc_uart_receive_reply(FOC_CMD_GET_BUS_CURRENT, pl, sizeof(pl), &len, 50))
        return false;

    if (len != 4) return false;

    memcpy(ibus_mA, pl, 4);
    return true;
}

bool foc_uart_get_speed(int32_t *rpm)
{
    uint8_t pl[8], len;

    foc_uart_send_cmd(FOC_CMD_GET_SPEED, NULL, 0);
    if (!foc_uart_receive_reply(FOC_CMD_GET_SPEED, pl, sizeof(pl), &len, 50))
        return false;

    if (len != 4) return false;

    memcpy(rpm, pl, 4);
    return true;
}

bool foc_uart_get_iq(int16_t *iq_mA)
{
    uint8_t pl[4], len;

    foc_uart_send_cmd(FOC_CMD_GET_IQ, NULL, 0);
    if (!foc_uart_receive_reply(FOC_CMD_GET_IQ, pl, sizeof(pl), &len, 50))
        return false;

    if (len != 2) return false;

    memcpy(iq_mA, pl, 2);
    return true;
}

bool foc_uart_get_id(int16_t *id_mA)
{
    uint8_t pl[4], len;

    foc_uart_send_cmd(FOC_CMD_GET_ID, NULL, 0);
    if (!foc_uart_receive_reply(FOC_CMD_GET_ID, pl, sizeof(pl), &len, 50))
        return false;

    if (len != 2) return false;

    memcpy(id_mA, pl, 2);
    return true;
}

/* ---------- setters ---------- */

bool foc_uart_set_iq_pi_gains(int32_t kp, int32_t ki, uint8_t *result)
{
    uint8_t pl[8];
    memcpy(&pl[0], &kp, 4);
    memcpy(&pl[4], &ki, 4);

    if (!foc_uart_send_cmd(FOC_CMD_SET_IQ_PI_GAINS, pl, 8))
        return false;

    uint8_t len;
    uint8_t r;

    if (!foc_uart_receive_reply(FOC_CMD_SET_IQ_PI_GAINS, &r, 1, &len, 50))
        return false;

    if (len != 1)
        return false;

    if (result) *result = r;
    return true;
}

bool foc_uart_set_id_pi_gains(int32_t kp, int32_t ki, uint8_t *result)
{
    uint8_t pl[8];
    memcpy(&pl[0], &kp, 4);
    memcpy(&pl[4], &ki, 4);

    if (!foc_uart_send_cmd(FOC_CMD_SET_ID_PI_GAINS, pl, 8))
        return false;

    uint8_t len;
    uint8_t r;

    if (!foc_uart_receive_reply(FOC_CMD_SET_ID_PI_GAINS, &r, 1, &len, 50))
        return false;

    if (len != 1)
        return false;

    if (result) *result = r;
    return true;
}


static const char *TAG = "FOC_TEST";

void foc_uart_test_task(void *arg)
{
    foc_all_fast_t fast;
    TelemetryData *data = (TelemetryData *)arg;

    vTaskDelay(pdMS_TO_TICKS(500));

    //ESP_LOGI(TAG, "FOC UART test task started");

    while (1)
    {
        if (foc_uart_get_all_fast(&fast))
        {
            float current_A = fast.ibus_mA / 1000.0f;
            int32_t rpm     = fast.rpm;

            /* ---------- TELEMETRY UPDATE ---------- */
            xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
            data->battery_voltage = fast.vbus_mV / 1000.0f;
            data->current_amps    = current_A;
            data->rpms            = rpm;
            data->throttle_raw    = fast.throttle_raw;
            xSemaphoreGive(telemetry_mutex);

            /* ---------- KALMAN VELOCITY UPDATE ---------- */

            /* ---------- KALMAN VELOCITY UPDATE ---------- */

            kf_msg_t msg;

            bool throttle_active = (fast.throttle_raw > THROTTLE_VALID_THRESHOLD);
            bool current_active  = (fabs(current_A) > CURRENT_VALID_THRESHOLD);

            bool motor_driving = throttle_active && current_active;

            /* ===== CASE 1: MOTOR DRIVING → TRUST RPM ===== */
            if (motor_driving)
            {
                float omega = (rpm * TWO_PI) / 60.0f;
                float v = omega * WHEEL_RADIUS_M;

                float heading = kf.X[4];

                float vx = v * cosf(heading);
                float vy = v * sinf(heading);

                msg.type = KF_MEAS_VEL;
                msg.a = vx;
                msg.b = vy;

                if (kf_queue != NULL) {
                    xQueueSend(kf_queue, &msg, 0);
                }

                // ESP_LOGI("RPM_TRUST", "ON | thr=%d cur=%.2f rpm=%ld v=%.2f",
                //     fast.throttle_raw, current_A, rpm, v);
            }

            /* ===== CASE 2: MOTOR NOT DRIVING → IGNORE RPM ===== */
            else
            {
                // DO NOTHING → let IMU + GPS handle velocity

                // ESP_LOGI("RPM_TRUST", "OFF | thr=%d cur=%.2f rpm=%ld",
                //     fast.throttle_raw, current_A, rpm);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}