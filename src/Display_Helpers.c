#include "Display_Helpers.h"
#include "Driver_Screen.h"

//Initialize display struct at set values
SemaphoreHandle_t display_mutex = NULL;
DisplayData display_data = {
    0, //brightness
    0, // att_flag
    0, // lap_count
    1, // gauge_unit
    175, // map_zoom
    45, // map_persp
    30, // map_arrowpx
    0.0f, // pwr_demand
    0.0f, // pwr_base
    100.0f, // pwr_max
    {0}   // msg[101]
};

void FloatToBytes(float v, uint8_t *b){
    memcpy(b, &v, sizeof(float));
}

void U32toBytes(uint32_t v, uint8_t *b){
    b[0] = (uint8_t)(v >> 24);
    b[1] = (uint8_t)(v >> 16);
    b[2] = (uint8_t)(v >> 8);
    b[3] = (uint8_t)(v);
}

void I32toBytes(int32_t v, uint8_t *b){
    U32toBytes((uint32_t)v, b);
}

void U16toBytes(uint16_t v, uint8_t *b){
    b[0] = (uint8_t)(v >> 8);   // MSB
    b[1] = (uint8_t)(v);        // LSB
}

void I16toBytes(int16_t v, uint8_t *b){
    U16toBytes((uint16_t)v, b);
}

/* =========================================================
   Tiny JSON extractors for fixed-format responses
   Expects:
   {"message":"sample message"}
   {"isRunning":true}
   ========================================================= */
bool json_extract_message(const char *json, char *out_msg, size_t out_size)
{
    if (json == NULL || out_msg == NULL || out_size == 0) {
        return false;
    }

    const char *key = "\"message\":\"";
    const char *start = strstr(json, key);
    if (start == NULL) {
        return false;
    }

    start += strlen(key);
    const char *end = strchr(start, '"');
    if (end == NULL) {
        return false;
    }

    size_t len = (size_t)(end - start);
    if (len >= out_size) {
        return false;
    }

    memcpy(out_msg, start, len);
    out_msg[len] = '\0';
    return true;
}

bool json_extract_is_running(const char *json, bool *is_running)
{
    if (json == NULL || is_running == NULL) {
        return false;
    }

    const char *key = "\"isRunning\":";
    const char *start = strstr(json, key);
    if (start == NULL) {
        return false;
    }

    start += strlen(key);

    while (*start == ' ' || *start == '\t') {
        start++;
    }

    if (strncmp(start, "true", 4) == 0) {
        *is_running = true;
        return true;
    }

    if (strncmp(start, "false", 5) == 0) {
        *is_running = false;
        return true;
    }

    return false;
}

/* =========================================================
   String helpers
   ========================================================= */
void trim_leading_spaces(char **str)
{
    if (str == NULL || *str == NULL) {
        return;
    }

    while (**str != '\0' && isspace((unsigned char)**str)) {
        (*str)++;
    }
}

static int tokenize_command(char *input, char *argv[], int max_tokens)
{
    int argc = 0;
    char *token = strtok(input, " \t\r\n");

    while (token != NULL && argc < max_tokens) {
        argv[argc++] = token;
        token = strtok(NULL, " \t\r\n");
    }

    /* If there are still tokens left, command is too long */
    if (token != NULL) {
        return -1;
    }

    return argc;
}

static bool parse_u8_arg(const char *str, uint8_t *out_value)
{
    if (str == NULL || out_value == NULL) {
        return false;
    }

    char *endptr = NULL;
    long value = strtol(str, &endptr, 10);

    if (*str == '\0' || *endptr != '\0') {
        return false;
    }

    if (value < 0 || value > 255) {
        return false;
    }

    *out_value = (uint8_t)value;
    return true;
}

static bool parse_float_arg(const char *str, float *out_value)
{
    if (str == NULL || out_value == NULL) {
        return false;
    }

    char *endptr = NULL;
    float value = strtof(str, &endptr);

    if (*str == '\0' || *endptr != '\0') {
        return false;
    }

    *out_value = value;
    return true;
}

static bool parse_double_arg(const char *str, double *out_value)
{
    if (str == NULL || out_value == NULL) {
        return false;
    }

    char *endptr = NULL;
    double value = strtod(str, &endptr);

    if (*str == '\0' || *endptr != '\0') {
        return false;
    }

    *out_value = value;
    return true;
}


/* =========================================================
   Command parser / dispatcher
   ========================================================= */
typedef enum {
    CMD_NONE = 0,
    CMD_WATT_RANGE,
    CMD_GAUGE_UNI,
    CMD_ZOOM,
    CMD_PERSPECTIVE,
    CMD_ARROW,
    CMD_DEFAULT_POIS,
    CMD_ADD_POI,
    CMD_REMOVE_POI,
    CMD_LAPTIME,
    CMD_BRIGHTNESS,
} CommandId;

/* =========================================================
   Message storage
   ========================================================= */
void store_display_message_if_new(const char *new_msg){
    bool should_send = false;

    if (new_msg == NULL) {
        return;
    }

    size_t msg_len = strlen(new_msg);
    if (msg_len > CUSTOM_MSG_MAX_LEN) {
        ESP_LOGW("MESSAGE", "Message too long, ignored: %s", new_msg);
        return;
    }

    xSemaphoreTake(display_mutex, portMAX_DELAY);

    if (strncmp(display_data.custom_msg, new_msg, CUSTOM_MSG_MAX_LEN + 1) != 0) {
        memset(display_data.custom_msg, 0, sizeof(display_data.custom_msg));
        memcpy(display_data.custom_msg, new_msg, msg_len);
        ESP_LOGI("MESSAGE", "Stored message: %s", display_data.custom_msg);
        should_send = true;
    }

    xSemaphoreGive(display_mutex);

    if (should_send) {
        new_message_send_event_CAN();
    }
}

static CommandId identify_command(const char *cmd)
{
    if (cmd == NULL) {
        return CMD_NONE;
    }
    if (strcasecmp(cmd, "/watt_range") == 0) {
        return CMD_WATT_RANGE;
    }
    if (strcasecmp(cmd, "/gauge_unit") == 0) {
        return CMD_GAUGE_UNI;
    }
    if (strcasecmp(cmd, "/zoom") == 0) {
        return CMD_ZOOM;
    }
    if (strcasecmp(cmd, "/perspective") == 0) {
        return CMD_PERSPECTIVE;
    }
    if (strcasecmp(cmd, "/arrow") == 0) {
        return CMD_ARROW;
    }
    if (strcasecmp(cmd, "/default_pois") == 0) {
        return CMD_DEFAULT_POIS;
    }
    if (strcasecmp(cmd, "/add_poi") == 0) {
        return CMD_ADD_POI;
    }
    if (strcasecmp(cmd, "/remove_poi") == 0) {
        return CMD_REMOVE_POI;
    }
    if (strcasecmp(cmd, "/laptime") == 0) {
        return CMD_LAPTIME;
    }
    if (strcasecmp(cmd, "/brightness") == 0) {
        return CMD_BRIGHTNESS;
    }
    return CMD_NONE;
}

bool execute_command_from_message(const char *message)
{
    if (message == NULL || message[0] != '/') {
        return false;
    }

    char local_copy[CUSTOM_MSG_MAX_LEN + 1];
    memset(local_copy, 0, sizeof(local_copy));
    strncpy(local_copy, message, CUSTOM_MSG_MAX_LEN);

    char *argv[MAX_CMD_TOKENS] = {0};
    int argc = tokenize_command(local_copy, argv, MAX_CMD_TOKENS);

    if (argc <= 0) {
        ESP_LOGW("CMD", "Unrecognized command: %s", message);
        return true;
    }

    CommandId cmd = identify_command(argv[0]);

    switch (cmd) {
        case CMD_BRIGHTNESS:{
            if (argc != 2) {
                ESP_LOGW("CMD", "Unrecognized command: %s", message);
                return true;
            }
            uint8_t value = 0;
            if (!parse_u8_arg(argv[1], &value)) {
                ESP_LOGW("CMD", "Unrecognized command: %s", message);
                return true;
            }
            brightness_send_event_CAN(value);
            return true;
        }

        case CMD_WATT_RANGE:{
            if (argc != 3) {
                ESP_LOGW("CMD", "Unrecognized command: %s", message);
                return true;
            }

            float value1 = 0.0f;
            float value2 = 0.0f;

            if (!parse_float_arg(argv[1], &value1) || !parse_float_arg(argv[2], &value2)) {
                ESP_LOGW("CMD", "Unrecognized command: %s", message);
                return true;
            }

            if (!(value1 < value2)) {
                ESP_LOGW("CMD", "Unrecognized command: %s", message);
                return true;
            }

            watt_range_send_event_CAN(value1, value2);
            return true;
        }

        case CMD_ADD_POI:
        {
            if (argc != 5) {
                ESP_LOGW("CMD", "Unrecognized command: %s", message);
                return true;
            }

            uint8_t id = 0;
            uint8_t color = 0;
            double lat = 0.0;
            double lon = 0.0;

            if (!parse_u8_arg(argv[1], &id) ||
                !parse_u8_arg(argv[2], &color) ||
                !parse_double_arg(argv[3], &lat) ||
                !parse_double_arg(argv[4], &lon)) {
                ESP_LOGW("CMD", "Unrecognized command: %s", message);
                return true;
            }

            if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) {
                ESP_LOGW("CMD", "Unrecognized command: %s", message);
                return true;
            }

            add_poi_send_event_CAN(id, color, lat, lon);
            return true;
        }

        case CMD_NONE:
        default:
            ESP_LOGW("CMD", "Unrecognized command: %s", message);
            return true;
    }
}

/* =========================================================
   Status parser / update
   ========================================================= */
void update_status_flag_if_changed(bool is_running)
{
    uint8_t new_flag = is_running ? 0xAA : 0x00;

    xSemaphoreTake(display_mutex, portMAX_DELAY);

    if (display_data.att_flag != new_flag) {
        uint8_t previous_flag = display_data.att_flag;
        display_data.att_flag = new_flag;

        if (previous_flag == 0x00 && new_flag == 0xAA) {
            ESP_LOGI("STATUS", "Rising edge detected: att_flag 0x00 -> 0xAA");
        } else if (previous_flag == 0xAA && new_flag == 0x00) {
            ESP_LOGI("STATUS", "Falling edge detected: att_flag 0xAA -> 0x00");
        }
    }

    xSemaphoreGive(display_mutex);
}