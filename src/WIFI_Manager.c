#include "WIFI_Manager.h"
#include "Generic.h"
#include <stdlib.h>

//////// WIFI Settings //////// 
// const char *ssid = "55 HOME";
// const char *pass = "bienvenidos55";

const char *ssid = "Ivon";
const char *pass = "qwerty0987";

const bool allow_to_reconnect = true;
bool wifi_connected_flag = false;

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
        ESP_LOGW("NVS", "NVS corrupted or version mismatch, erasing...");

        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else if (err != ESP_OK)
    {
        ESP_LOGE("NVS", "NVS init failed: %d", err);
        ESP_ERROR_CHECK(err);
    }
    else
    {
        ESP_LOGI("NVS", "NVS initialized successfully");
    }
}


void WIFI_Event_Handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data){
    switch(event_id){
        case WIFI_EVENT_STA_START:
            ESP_LOGD("WIFI", "WIFI connecting ...");
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
            ESP_LOGI("WIFI", "WIFI got IP address");
            current_status_code = STATUS_ONLINE;
            wifi_connected_flag = true; 
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
            };

    esp_http_client_handle_t client = esp_http_client_init(&config);

    while(true)
    {
        if(wifi_connected_flag){
            char post_data[1024];

            // Take mutex and access telemetry data
            xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
            // Get timestamp for data
            data->timestamp = telemetry_timestamp_ms();

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
                "\"throttle_raw\": %d"
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
                data->velocity_x,
                data->velocity_y,
                data->ambient_temp,
                data->altitude_m,
                data->num_sats,
                data->air_speed,
                data->throttle_raw
            );

            xSemaphoreGive(telemetry_mutex);

            esp_http_client_set_method(client, HTTP_METHOD_POST);
            esp_http_client_set_post_field(client, post_data, strlen(post_data));
            esp_http_client_set_header(client, "Content-Type", "application/json");

            esp_err_t err = esp_http_client_perform(client);
            ESP_LOGI("HTTP", "Status = %d", esp_http_client_get_status_code(client));

            if (err == ESP_OK) {
                ESP_LOGI("HTTP", "Telemetry sent");
            } else {
                ESP_LOGE("HTTP", "POST failed: %s", esp_err_to_name(err));
            }

        }
        vTaskDelay(pdMS_TO_TICKS(500));
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

/* =========================================================
   HTTP GET helper
   ========================================================= */
static esp_err_t http_get_to_buffer(const char *url, char *rx_buffer, size_t rx_buffer_size, int *http_status)
{
    if (url == NULL || rx_buffer == NULL || rx_buffer_size == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    rx_buffer[0] = '\0';

    if (http_status != NULL) {
        *http_status = 0;
    }

    http_resp_buf_t resp = {
        .buffer = rx_buffer,
        .max_len = rx_buffer_size,
        .len = 0
    };

    esp_http_client_config_t config = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = 3000,
        .event_handler = http_event_handler_collect,
        .user_data = &resp,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == NULL) {
        return ESP_FAIL;
    }

    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_http_client_set_header(client, "Accept", "application/json");

    esp_err_t err = esp_http_client_perform(client);

    int status_code = esp_http_client_get_status_code(client);
    int content_length = esp_http_client_get_content_length(client);
    bool is_chunked = esp_http_client_is_chunked_response(client);

    if (http_status != NULL) {
        *http_status = status_code;
    }

    ESP_LOGI("HTTP_GET", "status=%d content_length=%d chunked=%d body=%s",
             status_code, content_length, is_chunked, rx_buffer);

    esp_http_client_cleanup(client);
    return err;
}


/* =========================================================
   Polling tasks
   ========================================================= */
void poll_status_task(void *pvParameter)
{
    (void)pvParameter;

    char rx_buffer[HTTP_RX_BUFFER_SIZE];
    bool parsed_is_running = false;

    while (true) {
        if (wifi_connected_flag) {
            int http_status = 0;
            esp_err_t err = http_get_to_buffer(STATUS_URL, rx_buffer, sizeof(rx_buffer), &http_status);

            if (err == ESP_OK) {
                if (http_status == 200) {
                    ESP_LOGI("STATUS", "Raw payload: %s", rx_buffer);
                    if (json_extract_is_running(rx_buffer, &parsed_is_running)) {
                        update_status_flag_if_changed(parsed_is_running);
                    } else {
                        ESP_LOGW("STATUS", "Invalid JSON payload: %s", rx_buffer);
                    }
                } else {
                    ESP_LOGW("STATUS", "HTTP status = %d", http_status);
                }
            } else {
                ESP_LOGE("STATUS", "GET failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void poll_message_task(void *pvParameter)
{
    (void)pvParameter;

    char rx_buffer[HTTP_RX_BUFFER_SIZE];
    char received_message[CUSTOM_MSG_MAX_LEN + 1];

    while (true) {
        if (wifi_connected_flag) {
            int http_status = 0;
            esp_err_t err = http_get_to_buffer(MESSAGE_URL, rx_buffer, sizeof(rx_buffer), &http_status);

            if (err == ESP_OK) {
                if (http_status == 200) {
                    memset(received_message, 0, sizeof(received_message));
                    ESP_LOGI("MESSAGE", "Raw payload: %s", rx_buffer);

                    if (json_extract_message(rx_buffer, received_message, sizeof(received_message))) {
                        char *msg_ptr = received_message;
                        trim_leading_spaces(&msg_ptr);

                        if (msg_ptr[0] == '/') {
                            (void)execute_command_from_message(msg_ptr);
                        } else if (msg_ptr[0] != '\0') {
                            store_display_message_if_new(msg_ptr);
                        }
                    } else {
                        ESP_LOGW("MESSAGE", "Invalid JSON payload: %s", rx_buffer);
                    }
                } else {
                    ESP_LOGW("MESSAGE", "HTTP status = %d", http_status);
                }
            } else {
                ESP_LOGE("MESSAGE", "GET failed: %s", esp_err_to_name(err));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}