#include "main.h"

uint8_t current_status_code = STATUS_OFFLINE;
bool isRunning = false;

// Kalman handlers 
QueueHandle_t kf_queue;

// Default telemetry data on start
TelemetryData telemetry_data = {
    .timestamp       = 0,
    .battery_voltage = 0.0f,
    .current_amps    = 0.0f,
    .latitude        = 0.0f,
    .longitude       = 0.0f,
    .accel_x         = 0.0f,
    .accel_y         = 0.0f,
    .accel_z         = 0.0f,
    .orient_x        = 0.0f,
    .orient_y        = 0.0f,
    .orient_z        = 0.0f,
    .rpms            = 0,
    .velocity_x      = 0.0f,
    .velocity_y      = 0.0f,
    .ambient_temp    = 0.0f,
    .altitude_m      = 0.0f,
    .num_sats        = 0,
    .air_speed       = 0.0f,
    .throttle_raw    = 0
};

SemaphoreHandle_t telemetry_mutex;
SemaphoreHandle_t i2c_mutex;


void app_main(void)
{
    isRunning = true;

    // Create mutex for telemetry data struct 
    telemetry_mutex = xSemaphoreCreateMutex();
    configASSERT(telemetry_mutex);
    i2c_mutex = xSemaphoreCreateMutex();
    configASSERT(i2c_mutex);
    display_mutex = xSemaphoreCreateMutex();
    configASSERT(display_mutex);
    
    // Create Kalman queue (20 pending measurements max) 
    kf_queue = xQueueCreate(20, sizeof(kf_msg_t));
    // Create CAN frame queue
    can_tx_queue = xQueueCreate(CAN_TX_QUEUE_LEN, sizeof(can_frame_t));

    // Configuration 
    NVS_Init();
    Peripheral_Config();

    // Communication 
    xTaskCreatePinnedToCore(foc_uart_test_task, "foc_uart_test_task", 4096, &telemetry_data, 6, NULL, 1);
    xTaskCreatePinnedToCore(GPS_parse_task, "gps_parse", 4096, &telemetry_data, 10, NULL, 1);
    xTaskCreatePinnedToCore(pitot_task, "pitot_task", 4096, &telemetry_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(can_tx_task, "can_tx_task", 4096, NULL, 3, NULL, 1);

    // Data acquisition 
    xTaskCreate(post_data, "post_data", 8192, &telemetry_data, 11, NULL);
    xTaskCreate(SD_manager_task, "SD_manager", 4096, &telemetry_data, 15, NULL);
    xTaskCreate(poll_status_task, "poll_status_task", 4096, NULL, 4, NULL);
    xTaskCreate(poll_message_task, "poll_message_task", 4096, NULL, 4, NULL);

    //CAN periodic frames
    xTaskCreate(map_data_send_periodic_CAN, "map_data_send_periodic_CAN", 4096, NULL, 1, NULL);
    xTaskCreate(telemetry_send_periodic_CAN, "telemetry_send_periodic_CAN", 4096, NULL, 1, NULL);
    xTaskCreate(weather_task, "weather_task", 4096, NULL, 2, NULL);

    // Kalman task 
    xTaskCreatePinnedToCore(kalman_task, "kalman_task", 4096, NULL, 10, NULL, 0);

    // Debugging
    xTaskCreate(status_LED_task, "status_LED", 2048, NULL, 13, NULL);
    // xTaskCreate(telemetry_print_task, "telemetry_print_task", 4096, NULL, 1, NULL);
}