#include "PeriphConfig.h"
#include "bno055.h"
#include "Kalman2D.h"
#include "math.h"
#include "Globals.h"

// Pinout check - https://lastminuteengineers.com/esp32-wroom-32-pinout-reference/

bool WIFI_transmit_enable = true;
bool test_mode_enabled = false;
volatile bool SD_card_detected = false; 

/**
 * @brief Interrupt handler to detect if an SD card is present
 * @return None
 */
static void IRAM_ATTR gpio_SD_detect_handler(void* arg){
    SD_card_detected = (gpio_get_level(SD_DETECT_PIN) == 0);
}


/**
 * @brief This function configures all the used GPIOs in the system. 
 * @return None
 */
void GPIO_Config(){
    gpio_config_t io_conf = {0};

    // Test mode input with falling edge interrupt
    io_conf.intr_type = GPIO_INTR_NEGEDGE; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = (1ULL << TEST_EN_BUTTON_PIN);
    gpio_config(&io_conf);
    // Install and hook ISR service
    // gpio_install_isr_service(0);
    // gpio_isr_handler_add(TEST_EN_BUTTON_PIN, gpio_isr_handler, NULL);

    // SD detect GPIO (CD), to GND when SD is present, 3.3V if not
    io_conf.intr_type = GPIO_INTR_ANYEDGE; 
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = (1ULL << SD_DETECT_PIN);
    gpio_config(&io_conf);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(SD_DETECT_PIN, gpio_SD_detect_handler, NULL);

    // Configure GPIO12 status LED
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = (1ULL << LED_PIN);
    gpio_config(&io_conf);

    // Configure GPIO15 LoRa CS and pull it HIGH to avoid selecting it for now 
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = (1ULL << RF_SPI_CS_PIN);
    gpio_config(&io_conf);
    gpio_set_level(RF_SPI_CS_PIN, 1);
}


/**
 * @brief Configures required UART channels
 * @note  UART0 -> GPS / UART1 -> FOC Driver / UART2 -> TFT Display
 * @return None
 */
void UART_Config(){
    // Config params and buffer 
    const int buff_size = 2048;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    
    // QueueHandle_t uart0_queue;
    // QueueHandle_t uart1_queue;
    // QueueHandle_t uart2_queue;

    // Configure UART 1 (GPS)
    uart_config.baud_rate = 9600;
    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_CHANNEL, buff_size, buff_size, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_CHANNEL, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_CHANNEL, GPS_TX_GPIO, GPS_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    GPS_Init(GPS_UART_CHANNEL);

    // Configure UART 2 (FOC Driver)
    uart_config.baud_rate = 115200;
    ESP_ERROR_CHECK(uart_driver_install(FOC_DRIVER_UART_CHANNEL, buff_size, buff_size, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(FOC_DRIVER_UART_CHANNEL, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(FOC_DRIVER_UART_CHANNEL, FOC_DRIVER_TX_GPIO, FOC_DRIVER_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void bno055_task(void *arg)
{
    TelemetryData *telem = (TelemetryData *)arg;

    double acc[3];
    double gyro[3];
    double euler[3];
    int8_t temp;

    kf_msg_t msg;

    static bool kf_heading_initialized = false;

    static float heading_buffer[HEADING_WINDOW];
    static int heading_idx = 0;
    static bool heading_buffer_full = false;

    esp_err_t err = bno055_begin_i2c(OPERATION_MODE_NDOF);
    if (err != ESP_OK){
        vTaskDelete(NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(10));

    /* ----------- LOAD CALIBRATION ----------- */
    set_opmode(OPERATION_MODE_CONFIG);
    vTaskDelay(pdMS_TO_TICKS(20));

    /* -------- AXIS REMAP (MATCH YOUR MOUNTING) -------- */
    set_axis_remap(REMAP_CONFIG_P4);
    set_axis_sign(REMAP_SIGN_P4);

    //calibrate_sensor_from_saved_profile();

    set_opmode(OPERATION_MODE_NDOF);
    vTaskDelay(pdMS_TO_TICKS(50));

    for (int i = 0; i < 10; i++) {
        double dummy[3];
        get_vector(VECTOR_EULER, dummy);
        get_vector(VECTOR_MAGNETOMETER, dummy);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    while (1)
    {
        bool ok_acc = false;
        bool ok_gyro = false;
        bool ok_euler = false;

        float current;
        xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
        current = telem->current_amps;
        xSemaphoreGive(telemetry_mutex);

        if (get_vector(VECTOR_LINEARACCEL, acc) == ESP_OK)
            ok_acc = true;

        if (get_vector(VECTOR_GYROSCOPE, gyro) == ESP_OK)
            ok_gyro = true;

        if (get_vector(VECTOR_EULER, euler) == ESP_OK)
            ok_euler = true;

        temp = get_temp();

        /* ---------- ACCEL ---------- */
        if (ok_acc)
        {
            msg.type = KF_MEAS_ACCEL;
            msg.a = acc[0];
            msg.b = acc[1];
            xQueueSend(kf_queue, &msg, 0);
        }

        /* ---------- GYRO ---------- */
        if (ok_gyro)
        {
            msg.type = KF_MEAS_GYRO;
            msg.a = gyro[2] * (M_PI / 180.0f);
            xQueueSend(kf_queue, &msg, 0);
        }

        /* ---------- HEADING ---------- */
        if (ok_euler && ok_gyro)
        {
            // ESP_LOGI("IMU_DEBUG",
            //     "EULER [deg] -> Yaw=%.2f  Roll=%.2f  Pitch=%.2f | "
            //     "GYRO [deg/s] -> X=%.2f  Y=%.2f  Z=%.2f",
            //     euler[0], euler[1], euler[2],
            //     gyro[0], gyro[1], gyro[2]);
    
            float heading = euler[0] * (M_PI / 180.0f);
            float theta_kf = (M_PI/2.0f) - heading;

            // wrap
            if (theta_kf > M_PI)  theta_kf -= 2*M_PI;
            if (theta_kf < -M_PI) theta_kf += 2*M_PI;

            float gyro_z = gyro[2] * (M_PI / 180.0f);

            /* -------- BUFFER -------- */
            heading_buffer[heading_idx++] = heading;

            if (heading_idx >= HEADING_WINDOW)
            {
                heading_idx = 0;
                heading_buffer_full = true;
            }

            bool reliable = false;

            if (heading_buffer_full)
            {
                float mean_sin = 0, mean_cos = 0;

                for (int i = 0; i < HEADING_WINDOW; i++)
                {
                    mean_sin += sinf(heading_buffer[i]);
                    mean_cos += cosf(heading_buffer[i]);
                }

                mean_sin /= HEADING_WINDOW;
                mean_cos /= HEADING_WINDOW;

                float variance = 1.0f - sqrtf(mean_sin * mean_sin + mean_cos * mean_cos);

                reliable = (variance < 0.05f) &&
                           (fabs(gyro_z) < 0.5f) &&
                           (fabs(current) < CURRENT_VALID_THRESHOLD);

               //ESP_LOGI("HEADING", "var=%f rel=%d", variance, reliable);
            }

            /* -------- KF INIT -------- */
            if (!kf_heading_initialized && reliable)
            {
                kf.X[4] = theta_kf;
                kf_heading_initialized = true;

                ESP_LOGI("KF", "Heading initialized: %f rad", heading);
            }

            /* -------- KF UPDATE -------- */
            if (reliable)
            {
                msg.type = KF_MEAS_HEADING;
                msg.a = theta_kf;
                xQueueSend(kf_queue, &msg, 0);
            }

            /* -------- TELEMETRY -------- */
            xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
            telem->orient_x = euler[2]; //roll  - not fed into kalman
            telem->orient_y = euler[1];  //pitch - not fed into kalman
            telem->ambient_temp = (float)temp;
            xSemaphoreGive(telemetry_mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/** 
 * @brief Configures I2C Buses
 * 
 * @note Only one I2C interface (from 2 available) being used in this board.
 * GPIO21 - SDA 
 * GPIO22 - SCL
 * 
 * @return None
 */
void I2C_Config(){
    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ
    };


    // Configure I2C port
    ESP_ERROR_CHECK(i2c_param_config(I2C_INSTANCE, &i2c_config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_INSTANCE, i2c_config.mode, 0, 0, 0));
}


esp_err_t SPI_Config(){
    esp_err_t ret;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;        // VSPI
    host.max_freq_khz = 400;     // slow for init

    // 2) SPI bus config
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SPI_MOSI_PIN,
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    host.max_freq_khz = 400; 

    ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);

    SD_card_detected = (gpio_get_level(SD_DETECT_PIN) == 0);   // Read SD state at start

    return ret;
}


void Peripheral_Config(void){
    GPIO_Config();
    UART_Config();
    I2C_Config();
    SPI_Config();

    xTaskCreate(bno055_task, "bno055_task", 4096, &telemetry_data, 4, NULL);
}