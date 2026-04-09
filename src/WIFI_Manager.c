#include "WIFI_Manager.h"
#include "Generic.h"
#include "Driver_Screen.h"
#include <stdlib.h>

//////// WIFI Settings //////// 
// const char *ssid = "55 HOME";
// const char *pass = "bienvenidos55";

const char *ssid = "Ivon";
const char *pass = "qwerty0987";

const bool allow_to_reconnect = true;
bool wifi_connected_flag = false;
volatile bool weather_fetch_request = false;

char ip_address[20];

//////////////////////////////// 

/**
 * @brief Initialize the Non-Volatile Storage (NVS).
 *
 * This function initializes the NVS flash partition. If the NVS
 * partition is found to be full, corrupted, or incompatible with
 * the current ESP-IDF version, it erases the partition and
 * re-initializes it.
 *
 * This ensures that NVS is always in a usable state for storing
 * persistent data such as Wi-Fi credentials, device settings,
 * and calibration values.
 */
// void NVS_Init(void){
//     esp_err_t nvs_status = nvs_flash_init();
//     if(nvs_status == ESP_ERR_NVS_NO_FREE_PAGES || nvs_status == ESP_ERR_NVS_NEW_VERSION_FOUND){
//         printf("ERROR NVS was thrown\n");
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         nvs_status = nvs_flash_init();
//         ESP_ERROR_CHECK(nvs_status);
//     }
// }

void NVS_Init(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        //ESP_LOGW("NVS", "NVS corrupted or version mismatch, erasing...");

        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else if (err != ESP_OK)
    {
        //ESP_LOGE("NVS", "NVS init failed: %d", err);
        ESP_ERROR_CHECK(err);
    }
    else
    {
        //ESP_LOGI("NVS", "NVS initialized successfully");
    }
}


void WIFI_Event_Handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch(event_id){
        case WIFI_EVENT_STA_START:
            //ESP_LOGD("WIFI", "WIFI connecting ...");
            break;

        case WIFI_EVENT_STA_CONNECTED:
            // ESP_LOGI("WIFI", "WIFI Connected");
            break;

        case WIFI_EVENT_STA_DISCONNECTED:
            // ESP_LOGW("WIFI", "WIFI disconnected");
            wifi_connected_flag = false;
            current_status_code = STATUS_OFFLINE;
            // Manage reconnection after losing signal
            if (allow_to_reconnect){
                esp_wifi_connect();
            }
            break;

        case IP_EVENT_STA_GOT_IP:
            //ESP_LOGI("WIFI", "WIFI got IP address");
            current_status_code = STATUS_ONLINE;
            wifi_connected_flag = true; 
            weather_fetch_request = true;
            break;

        default:
            // printf("An unrecognized WiFi event with id %ld occured\n", event_id);
            break;
    }
}


void WIFI_Connect(void){
    esp_netif_init();
    esp_event_loop_create_default(); 
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&wifi_initiation);

    // Register event handler for WiFi
    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, WIFI_Event_Handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, WIFI_Event_Handler, NULL);

    // WiFi configuration struct
    wifi_config_t wifi_configuration = { 
        .sta = {
            .ssid = "",
            .password= ""
        }
    };
    strcpy((char*)wifi_configuration.sta.ssid, ssid);
    strcpy((char*)wifi_configuration.sta.password, pass);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_configuration);

    // Start  configured WiFi and connect 
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_start();
    esp_wifi_connect();  // Will trigger event handler
}


void  post_data(void *pvParameter)
{
    TelemetryData *data = (TelemetryData *)pvParameter;

    // if (data == NULL) {
    //     ESP_LOGE("POST", "TelemetryData is NULL");
    //     vTaskDelete(NULL);
    // }

    WIFI_Connect();

    esp_http_client_config_t config = {
            //.url = "https://joshingly-thermotaxic-trudie.ngrok-free.dev/telemetry",
            //.host = "elyos-telemetry-exylp.ondigitalocean.app",
            .url = "https://elyos-telemetry-exylp.ondigitalocean.app/elyos-telemetry-backend/api/lectures",
            //.transport_type = HTTP_TRANSPORT_OVER_SSL,
            //.skip_cert_common_name_check = true,
            .crt_bundle_attach = esp_crt_bundle_attach,
            .keep_alive_enable = true,
            };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    bool last_wifi = false;


    while(true)
    {
        if(wifi_connected_flag){

            if (!last_wifi) {

                if (client != NULL) {
                    esp_http_client_cleanup(client);
                    client = NULL;
                }

                esp_http_client_config_t config = {
                    .url = "https://elyos-telemetry-exylp.ondigitalocean.app/elyos-telemetry-backend/api/lectures",
                    .crt_bundle_attach = esp_crt_bundle_attach,
                    .keep_alive_enable = true,
                };

                client = esp_http_client_init(&config);
            }
            
            char post_data[1024];

            // Take mutex and access telemetry data
            xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
            // Get timestamp for data
            data->timestamp = telemetry_timestamp_ms();

            // throttle
            uint16_t raw_throt = data->throttle_raw;
            uint16_t throt_scale = display_data.max_throttle - display_data.base_throttle;
            uint16_t raw = data->throttle_raw;
            uint16_t base = display_data.base_throttle;
            uint16_t max = display_data.max_throttle;

            float throttle = ((float)(raw - base) / (float)(max - base)) * 100.0f;
            float x_kmh = (data->velocity_x)*3.6f; // convert m/s to km/h
            float y_kmh = (data->velocity_y)*3.6f; // convert m/s to km/h

            snprintf(post_data, sizeof(post_data),
                "{"
                "\"timestamp\": %lu, "
                "\"voltage_battery\": %.2f, "
                "\"current\": %.2f, "
                "\"latitude\": %.6f, "
                "\"longitude\": %.6f, "
                "\"acceleration_x\": %.2f, "
                "\"acceleration_y\": %.2f, "
                "\"acceleration_z\": %.2f, "
                "\"orientation_x\": %.2f, "
                "\"orientation_y\": %.2f, "
                "\"orientation_z\": %.2f, "
                "\"rpm_motor\": %d, "
                "\"velocity_x\": %.2f, "
                "\"velocity_y\": %.2f, "
                "\"ambient_temp\": %.2f, "
                "\"altitude_m\": %.2f, "
                "\"num_sats\": %d, "
                "\"air_speed\": %.2f, "
                "\"throttle\": %.1f"
                "}",
                data->timestamp,
                data->battery_voltage,
                data->current_amps,
                data->latitude,
                data->longitude,
                data->accel_x,
                data->accel_y,
                data->accel_z,
                data->orient_x,
                data->orient_y,
                data->orient_z,
                data->rpms,
                x_kmh,
                y_kmh,
                data->ambient_temp,
                data->altitude_m,
                data->num_sats,
                fabsf(data->air_speed),
                throttle
            );

            xSemaphoreGive(telemetry_mutex);

            esp_http_client_set_method(client, HTTP_METHOD_POST);
            esp_http_client_set_post_field(client, post_data, strlen(post_data));
            esp_http_client_set_header(client, "Content-Type", "application/json");

            esp_err_t err = esp_http_client_perform(client);
            //ESP_LOGI("HTTP", "Status = %d", esp_http_client_get_status_code(client));

            // if (err == ESP_OK) {
            //     //ESP_LOGI("HTTP", "Telemetry sent");
            // } else {
            //     //ESP_LOGE("HTTP", "POST failed: %s", esp_err_to_name(err));
            // }
            if (err != ESP_OK) {
                //ESP_LOGW("HTTP", "POST failed, resetting client");

                if (client != NULL) {
                    esp_http_client_cleanup(client);
                    client = NULL;
                }

                esp_http_client_config_t config = {
                    .url = "https://elyos-telemetry-exylp.ondigitalocean.app/elyos-telemetry-backend/api/lectures",
                    .crt_bundle_attach = esp_crt_bundle_attach,
                    .keep_alive_enable = true,
                };

                client = esp_http_client_init(&config);
            }

        }
        last_wifi = wifi_connected_flag;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    esp_http_client_cleanup(client);
    vTaskDelete(NULL);
}




static esp_err_t http_event_handler_collect(esp_http_client_event_t *evt)
{
    http_resp_buf_t *resp = (http_resp_buf_t *)evt->user_data;

    if (resp == NULL) {
        return ESP_OK;
    }

    switch (evt->event_id) {
        case HTTP_EVENT_ON_DATA:
            if (evt->data != NULL && evt->data_len > 0) {
                size_t copy_len = (size_t)evt->data_len;

                if (resp->len + copy_len >= resp->max_len) {
                    copy_len = resp->max_len - resp->len - 1;
                }

                if (copy_len > 0) {
                    memcpy(resp->buffer + resp->len, evt->data, copy_len);
                    resp->len += copy_len;
                    resp->buffer[resp->len] = '\0';
                }
            }
            break;

        default:
            break;
    }

    return ESP_OK;
}

/* ========================================================= */
static void http_client_init_ctx(http_client_ctx_t *ctx, const char *url)
{
    memset(ctx, 0, sizeof(*ctx));

    ctx->resp.buffer = ctx->buffer;
    ctx->resp.max_len = sizeof(ctx->buffer);
    ctx->resp.len = 0;

    esp_http_client_config_t config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 2000,
        .event_handler = http_event_handler_collect,
        .user_data = &ctx->resp,
        .keep_alive_enable = true,  
    };

    ctx->client = esp_http_client_init(&config);
    ctx->initialized = true;
}

static esp_err_t http_client_perform_ctx(http_client_ctx_t *ctx, int *http_status)
{
    ctx->resp.len = 0;
    ctx->buffer[0] = '\0';

    esp_http_client_set_method(ctx->client, HTTP_METHOD_GET);

    esp_err_t err = esp_http_client_perform(ctx->client);

    if (err != ESP_OK) {

        // ESP_LOGW("HTTP", "GET failed, resetting client");

        if (ctx->client != NULL) {
            esp_http_client_cleanup(ctx->client);
            ctx->client = NULL;
        }

        // Recreate client
        http_client_init_ctx(ctx, STATUS_URL);  // or MESSAGE_URL depending caller

        return err;
    }

    int status_code = esp_http_client_get_status_code(ctx->client);

    if (http_status) {
        *http_status = status_code;
    }

    //ESP_LOGI("HTTP_GET", "status=%d body=%s", status_code, ctx->buffer);

    return err;
}
   

/* =========================================================
   Polling tasks
   ========================================================= */
void poll_status_task(void *pvParameter)
{
    http_client_ctx_t ctx;
    http_client_init_ctx(&ctx, STATUS_URL);

    bool parsed_is_running = false;
    bool last_wifi = false;


    while (true) {
        if (wifi_connected_flag) {

            if (!last_wifi) {
                esp_http_client_cleanup(ctx.client);
                http_client_init_ctx(&ctx, STATUS_URL);
            }

            int http_status = 0;

            esp_err_t err = http_client_perform_ctx(&ctx, &http_status);

            if (err == ESP_OK && http_status == 200) {
                bool parsed_ok = false;

                if (json_extract_is_running(ctx.buffer, &parsed_is_running)) {
                    update_status_flag_if_changed(parsed_is_running);
                    parsed_ok = true;
                }

                uint8_t parsed_lap = 0;

                if (json_extract_current_lap(ctx.buffer, &parsed_lap)) {
                    update_lap_count_if_changed(parsed_lap);
                    parsed_ok = true;
                }

                if (!parsed_ok) {
                    //ESP_LOGW("STATUS", "Invalid JSON payload: %s", ctx.buffer);
                }
            }
        }

        last_wifi = wifi_connected_flag;
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void poll_message_task(void *pvParameter)
{
    http_client_ctx_t ctx;
    http_client_init_ctx(&ctx, MESSAGE_URL);

    char received_message[CUSTOM_MSG_MAX_LEN + 1];
    bool last_wifi = false;

    while (true) {
        if (wifi_connected_flag) {

            if (!last_wifi) {
                esp_http_client_cleanup(ctx.client);
                http_client_init_ctx(&ctx, MESSAGE_URL);
            }

            int http_status = 0;

            esp_err_t err = http_client_perform_ctx(&ctx, &http_status);

            if (err == ESP_OK && http_status == 200) {

                memset(received_message, 0, sizeof(received_message));

                if (json_extract_message(ctx.buffer, received_message, sizeof(received_message))) {

                    char *msg_ptr = received_message;
                    trim_leading_spaces(&msg_ptr);

                    if (msg_ptr[0] == '/') {
                        execute_command_from_message(msg_ptr);
                    } else if (msg_ptr[0] != '\0') {
                        store_display_message_if_new(msg_ptr);
                    }
                }
            }
        }

        last_wifi = wifi_connected_flag;
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void fetch_weather_once(void)
{
    if (!wifi_connected_flag) return;

    http_client_ctx_t ctx;
    http_client_init_ctx(&ctx, WEATHER_URL);

    int http_status = 0;

    esp_err_t err = http_client_perform_ctx(&ctx, &http_status);

    if (err != ESP_OK || http_status != 200) {
        return;
    }

    float temp = 0.0f;
    int visibility = 0;
    int humidity = 0;
    int precipitation = 0;
    int weather_code = 0;
    char time_str[40] = {0};

    bool ok = true;

    ok &= json_extract_float(ctx.buffer, "\"temperature_2m\":", &temp);
    ok &= json_extract_int(ctx.buffer, "\"visibility\":", &visibility);
    ok &= json_extract_int(ctx.buffer, "\"relative_humidity_2m\":", &humidity);
    ok &= json_extract_int(ctx.buffer, "\"precipitation_probability\":", &precipitation);
    ok &= json_extract_int(ctx.buffer, "\"weather_code\":", &weather_code);
    ok &= json_extract_string(ctx.buffer, "\"time\":\"", time_str, sizeof(time_str));

    if (!ok) return;

    /* ---- Extract hour ---- */
    int hour = 0;
    if (strlen(time_str) >= 13) {
        hour = (time_str[11] - '0') * 10 + (time_str[12] - '0');
    }

    /* ---- Extract timezone offset ---- */
    int tz_offset = 0;
    const char *tz_ptr = strstr(ctx.buffer, "\"timezoneAbbreviation\":\"GMT");
    if (tz_ptr) {
        tz_ptr += strlen("\"timezoneAbbreviation\":\"GMT");
        tz_offset = atoi(tz_ptr);
    }

    int local_hour = hour + tz_offset;

    if (local_hour < 0) local_hour += 24;
    if (local_hour >= 24) local_hour -= 24;

    bool is_day = (local_hour >= 7 && local_hour <= 19);

    WeatherType weather = map_weather_code(weather_code, is_day);

    /* ---- Store safely ---- */
    xSemaphoreTake(display_mutex, portMAX_DELAY);

    display_data.ambient_temp = temp;
    display_data.visibility = visibility;
    display_data.humidity = (uint8_t)humidity;
    display_data.precipitation = (uint8_t)precipitation;
    display_data.hour = (uint8_t)local_hour;
    display_data.curr_weather = weather;

    xSemaphoreGive(display_mutex);

    /* Optional: trigger CAN update */
    // send_weather_event_CAN();  <-- if you implement later
    weather_send_event_CAN((uint8_t)precipitation,(uint8_t)humidity, (uint8_t)weather, (uint8_t)local_hour, (float)temp, (int32_t)visibility);
}

void weather_task(void *arg)
{
    while (1) {
        if (weather_fetch_request) {
            weather_fetch_request = false;
            fetch_weather_once();   // ✅ safe here
        }

        vTaskDelay(pdMS_TO_TICKS(250));
    }
}