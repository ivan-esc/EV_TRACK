#include "esp_wifi.h"
#include "esp_http_client.h"
#include "esp_netif_ip_addr.h"
#include "nvs_flash.h"
#include "lwip/err.h" //light weight ip packets error handling
#include "lwip/sys.h" //system applications for light weight ip apps
#include "esp_log.h"
#include "esp_wpa2.h"
#include "Globals.h"
#include "esp_crt_bundle.h"
#include "Display_Helpers.h"

#define STATUS_URL  "https://elyos-telemetry-exylp.ondigitalocean.app/elyos-telemetry-backend/api/record/status"
#define MESSAGE_URL "https://elyos-telemetry-exylp.ondigitalocean.app/elyos-telemetry-backend/api/record/message"
#define WEATHER_URL "https://elyos-telemetry-exylp.ondigitalocean.app/elyos-telemetry-backend/api/record/weather"

///////// WIFI SETTINGS /////////
extern const char *ssid;
extern const char *pass;
extern bool wifi_connected_flag;
extern const bool allow_to_reconnect;

extern char ip_address[20];
/////////////////////////////////

/**
 * @brief Initialize the Non-Volatile Storage (NVS) partition.
 * This function initializes the NVS flash partition required for storing 
 * key-value pairs in flash. If the initialization fails due to no free 
 * pages or because a new version of NVS is detected, it erases the NVS 
 * partition and retries initialization.
 * 
 * @note This function uses ESP-IDF NVS APIs and will abort the program 
 *       if an unrecoverable error occurs (via ESP_ERROR_CHECK).
 *
 * @return None
 */
void NVS_Init(void);


/**
 * @brief Wi-Fi event handler callback.
 * @return None 
*/
void WIFI_Event_Handler(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data);


esp_err_t _http_event_handler(esp_http_client_event_t *evt);


void post_data(void *);

/**
 * @brief Connects the ESP to a WiFi network with a given SSID and password
 */
void WIFI_Connect(void);

void poll_status_task(void *pvParameter);
void poll_message_task(void *pvParameter);
void fetch_weather_once(void);

typedef struct {
    char *buffer;
    size_t max_len;
    size_t len;
} http_resp_buf_t;

typedef struct {
    esp_http_client_handle_t client;
    http_resp_buf_t resp;
    char buffer[HTTP_RX_BUFFER_SIZE];
    bool initialized;
} http_client_ctx_t;