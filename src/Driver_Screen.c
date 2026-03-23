#include "Driver_Screen.h"

uint8_t payload_base = 1;
static bool recovery_started = false;
QueueHandle_t can_tx_queue = NULL;


/* ------------------------------------------- */
/* ------------- FRAME CODING ---------------- */
/* ------------------------------------------- */

void can_queue_start_att(uint32_t id){
    if (can_tx_queue == NULL) return;

    can_frame_t frame;
    frame.identifier = id;
    frame.dlc = 4;

    frame.data[0] = display_data.lap_count;
    
    xQueueSend(can_tx_queue, &frame, 0);
}

void can_queue_reset_att(uint32_t id){
    if (can_tx_queue == NULL) return;

    can_frame_t frame;
    frame.identifier = id;
    frame.dlc = 4;
    frame.data[0] = (uint8_t)0;

    xQueueSend(can_tx_queue, &frame, 0); 
}

void can_queue_laps(uint32_t id, uint8_t laps){
    if (can_tx_queue == NULL) return;

    can_frame_t frame;
    frame.identifier = id;
    frame.dlc = 4;

    frame.data[0] = laps;

    xQueueSend(can_tx_queue, &frame, 0);
}

void can_queue_velocity(uint32_t id, float velocity_x, uint16_t rpms){
    if (can_tx_queue == NULL) return;

    static float last_vel = 0.0f;
    static uint16_t last_rpms = 0.0f;
    
    if ((fabsf(velocity_x - last_vel) < 0.01f) && rpms == last_rpms) return; 

    last_vel = velocity_x;
    last_rpms = rpms;

    can_frame_t frame;
    frame.identifier = id;
    frame.dlc = 6;

    U16toBytes(rpms, &frame.data[0]);
    FloatToBytes(velocity_x, &frame.data[2]);
    xQueueSend(can_tx_queue, &frame, 0);
}

void can_queue_watts(uint32_t id, uint16_t watts){
    if (can_tx_queue == NULL) return;

    static uint16_t last_watts = 0;
    if (watts == last_watts) return;   // exact compare is fine for integers

    last_watts = watts;

    can_frame_t frame;
    frame.identifier = id;
    frame.dlc = 2;

    U16toBytes(watts, &frame.data[0]);
    xQueueSend(can_tx_queue, &frame, 0);
}

void can_queue_watt_range(uint32_t id, uint16_t range_lower, uint16_t range_upper){
    if (can_tx_queue == NULL) return;

    can_frame_t frame;
    frame.identifier = id;
    frame.dlc = 8;

    U16toBytes(range_lower, &frame.data[0]);
    U16toBytes(range_upper, &frame.data[2]);
    xQueueSend(can_tx_queue, &frame, 0);
}

void can_queue_max_brightness(uint32_t id, uint8_t brightness){
    if (can_tx_queue == NULL) return;

    can_frame_t frame;
    frame.identifier = id;
    frame.dlc = 1;

    frame.data[0] = brightness;
    xQueueSend(can_tx_queue, &frame, 0);
   
}

void can_queue_coords(uint32_t id, double latitude, double longitude){
    if (can_tx_queue == NULL) return;

    static int32_t last_lat = 0;
    static int32_t last_lon = 0;

    int32_t lat_tx = (int32_t)(latitude * 1e7);
    int32_t lon_tx = (int32_t)(longitude * 1e7);

    if (lat_tx == last_lat && lon_tx == last_lon)
        return; 

    last_lat = lat_tx;
    last_lon = lon_tx;

    can_frame_t frame;
    frame.identifier = id;
    frame.dlc = 8;

    I32toBytes(lat_tx, &frame.data[0]);
    I32toBytes(lon_tx, &frame.data[4]);

    xQueueSend(can_tx_queue, &frame, 0);
}

void can_queue_heading(uint32_t id, float orient){
    if (can_tx_queue == NULL) return;
    static float last_orient = 0.0f;
    if (fabsf(orient - last_orient) < 5.0f) return;

    can_frame_t frame;
    frame.identifier = id;
    frame.dlc = 4;

    FloatToBytes(orient, &frame.data[0]);
    xQueueSend(can_tx_queue, &frame, 0);
}

void can_queue_map_aesthetic(uint32_t id, uint8_t map_zoom, uint8_t map_persp, uint8_t map_arrow_px){
    if (can_tx_queue == NULL) return;   

    can_frame_t frame;
    frame.identifier = id;
    frame.dlc = 3;

    frame.data[0] = map_zoom;
    frame.data[1] = map_persp;
    frame.data[2] = map_arrow_px;
    xQueueSend(can_tx_queue, &frame, 0);
}

void can_queue_custom_msg(uint32_t id, const char *msg, bool force_send)
{
    if (can_tx_queue == NULL) return;

    char buf[CUSTOM_MSG_MAX_LEN + 1];
    if (msg == NULL) {
        buf[0] = '\0';
    } else {
        // copy and ensure NUL-termination (truncate if necessary)
        strncpy(buf, msg, CUSTOM_MSG_MAX_LEN);
        buf[CUSTOM_MSG_MAX_LEN] = '\0';
    }

    static char last_msg[CUSTOM_MSG_MAX_LEN + 1] = {0};

    // If unchanged and not forced, do nothing
    if (!force_send && strcmp(buf, last_msg) == 0) {
        return;
    }

    // compute length and frames
    size_t len = strlen(buf); // 0..CUSTOM_MSG_MAX_LEN
    size_t frames = (len == 0) ? 1 : ((len + CUSTOM_CHUNK_SIZE - 1) / CUSTOM_CHUNK_SIZE);
    if (frames > CUSTOM_MAX_FRAMES) frames = CUSTOM_MAX_FRAMES; // safety (shouldn't happen)

    for (size_t part = 0; part < frames; ++part) {
        can_frame_t frame;
        frame.identifier = id;
        frame.dlc = 8;

        // first two bytes: len and part index (0-based)
        frame.data[0] = (uint8_t)len;
        frame.data[1] = (uint8_t)part;

        // zero the payload area
        memset(&frame.data[2], 0, 6);

        // compute how many bytes to copy for this chunk
        size_t offset = part * CUSTOM_CHUNK_SIZE;
        if (offset < len) {
            size_t remaining = len - offset;
            size_t copy_n = (remaining >= CUSTOM_CHUNK_SIZE) ? CUSTOM_CHUNK_SIZE : remaining;
            memcpy(&frame.data[2], &buf[offset], copy_n);
        }
        // enqueue (non-blocking). If queue full, silently drop (you can change timeout).
        xQueueSend(can_tx_queue, &frame, 0);
    }

    // update last_msg (keep trailing bytes cleared)
    memset(last_msg, 0, sizeof(last_msg));
    memcpy(last_msg, buf, len);
}

/* ------------------------------------------- */
/* ------------ QUEUE ADDER PERIODIC --------- */
/* ------------------------------------------- */

void map_data_send_periodic_CAN(void *arg){
    TickType_t last = xTaskGetTickCount();
    while(1){
        can_queue_coords(MAP_SET_COORDS,
                        telemetry_data.latitude,telemetry_data.longitude);
        can_queue_heading(MAP_SET_HEADING,
                        telemetry_data.orient_z);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(2500));
    }
}

void telemetry_send_periodic_CAN(void *arg){
    TickType_t last = xTaskGetTickCount();
    while(1){
        float vel_mag = fabsf(telemetry_data.velocity_x*telemetry_data.velocity_x + telemetry_data.velocity_y*telemetry_data.velocity_y);
        can_queue_velocity(DISP_SEND_RPM,
                        vel_mag,telemetry_data.rpms);
        can_queue_watts(DISP_SEND_POWER,
                        telemetry_data.throttle_raw);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(100));
    }
}

/* ------------------------------------------- */
/* ------------ QUEUE ADDER EVENT --- -------- */
/* ------------------------------------------- */

void brightness_send_event_CAN(uint8_t value){
    display_data.brightness = value;
    can_queue_max_brightness(DISP_MAX_BRIGHTNESS,
                            display_data.brightness);
}

void start_send_event_CAN(){
    can_queue_start_att(DISP_START_ATTEMPT);
}
void reset_send_event_CAN(){
    can_queue_reset_att(DISP_RESET_ALL);
}
void lap_count_send_event_CAN(uint8_t new_lap){
    can_queue_laps(DISP_SET_LAP_INFO, new_lap);
}


void watt_range_send_event_CAN(float value1, float value2){
    display_data.base_throttle = value1;
    display_data.max_throttle = value2;
    can_queue_watt_range(DISP_SET_BASE_POWER,
                        display_data.base_throttle,
                        display_data.max_throttle);
}

void map_aesthetic_send_event_CAN(uint8_t set_zoom,uint8_t set_perspective, uint8_t arrow_size){
    display_data.map_zoom = set_zoom;
    display_data.map_persp = set_perspective;
    display_data.map_arrowpx = arrow_size;
    can_queue_map_aesthetic(MAP_SET_VIEW,
                            display_data.map_zoom,
                            display_data.map_persp,
                            display_data.map_arrowpx);
}



void new_message_send_event_CAN(void){
    char msg_copy[CUSTOM_MSG_MAX_LEN + 1];

    xSemaphoreTake(display_mutex, portMAX_DELAY);
    memset(msg_copy, 0, sizeof(msg_copy));
    strncpy(msg_copy, display_data.custom_msg, CUSTOM_MSG_MAX_LEN);
    xSemaphoreGive(display_mutex);

    can_queue_custom_msg(DISP_MSG_CUSTOM, msg_copy, false);
}

void add_poi_send_event_CAN(uint8_t id, uint8_t color, double lat, double lon){
    /* TODO: implement */
}



/* ------------------------------------------ */
/* ---------------- CAN INIT ---------------- */
/* ------------------------------------------ */


void can_init(void)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    ESP_LOGI(TAG_CAN, "TWAI driver started");
}

/* ------------------------------------------ */
/* ---------------- CAN TX TASK ------------- */
/* ------------------------------------------ */

void can_tx_task(void *arg)
{
    can_frame_t frame;
    twai_message_t tx_msg;

    can_init();

    TickType_t last_wake = xTaskGetTickCount();
    twai_status_info_t status;

    while (1) {
        twai_get_status_info(&status);        

        if (status.state == TWAI_STATE_RUNNING) {
            recovery_started = false;

            if (xQueueReceive(can_tx_queue, &frame, pdMS_TO_TICKS(10)) == pdTRUE) {
                
                ESP_LOGI(TAG_CAN, "Sending frame ID: 0x%X", frame.identifier);
                tx_msg.identifier = frame.identifier;
                tx_msg.data_length_code = frame.dlc;
                tx_msg.flags = TWAI_MSG_FLAG_NONE;

                memcpy(tx_msg.data, frame.data, 8);

                esp_err_t err = twai_transmit(&tx_msg, pdMS_TO_TICKS(50));

                if (err != ESP_OK) {
                    ESP_LOGE(TAG_CAN, "TX failed (%s)", esp_err_to_name(err));
                }
            }
        }

        else if (status.state == TWAI_STATE_BUS_OFF) {
            twai_stop();
            twai_driver_uninstall();

            twai_general_config_t g_config =
            TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);

            twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
            twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

            ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
            ESP_ERROR_CHECK(twai_start());

            ESP_LOGI(TAG_CAN, "TWAI driver recover start");
            vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(3000));
        }
        else if (status.state == TWAI_STATE_RECOVERING) {
            // do nothing
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
    }
}
