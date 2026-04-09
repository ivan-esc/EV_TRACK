#ifndef GLOBALS_H
#define GLOBALS_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

/////////////////////////////////////////////////////////
///////////////////// BOARD PINOUT //////////////////////
/////////////////////////////////////////////////////////

// GPIOs
#define LED_PIN                       GPIO_NUM_12
#define TEST_EN_BUTTON_PIN            GPIO_NUM_13
#define SD_DETECT_PIN                 GPIO_NUM_4

// GPS UART 
#define GPS_UART_CHANNEL              UART_NUM_1
#define GPS_RX_GPIO                   GPIO_NUM_14
#define GPS_TX_GPIO                   GPIO_NUM_15

// DRIVER UART
#define FOC_DRIVER_UART_CHANNEL       UART_NUM_0
#define FOC_DRIVER_RX_GPIO            GPIO_NUM_3
#define FOC_DRIVER_TX_GPIO            GPIO_NUM_1
#define FOC_DRIVER_BAUDRATE           115200

// I2C Bus
#define I2C_INSTANCE                  I2C_NUM_0
#define I2C_SDA                       GPIO_NUM_21
#define I2C_SCL                       GPIO_NUM_22
#define I2C_FREQ_HZ                   400000 

// SPI Bus
#define SPI_MISO_PIN                  GPIO_NUM_19
#define SPI_MOSI_PIN                  GPIO_NUM_23
#define SPI_CLK_PIN                   GPIO_NUM_18
#define SD_SPI_CS_PIN                 GPIO_NUM_26     // Micro SD
#define RF_SPI_CS_PIN                 GPIO_NUM_25
 

//////////////////////////////////////////////////////////

// PLATFORM STATUS CODES
typedef enum {
    STATUS_ONLINE,
    STATUS_OFFLINE,
    STATUS_TRANSMITTING,
    STATUS_ERROR
} STATUS_CODES;


// Control variables
extern uint8_t current_status_code;
extern bool test_mode_enabled;
extern bool GPS_fix_status; 
extern volatile bool SD_card_detected; 
extern bool isRunning;


// TELEMETRY DATA TEMPLATE
typedef struct {
    uint32_t timestamp;
    float battery_voltage;
    float current_amps;
    double latitude;
    double longitude;
    float accel_x;
    float accel_y;
    float accel_z;
    float orient_x;
    float orient_y;
    float orient_z;
    uint16_t rpms;
    float velocity_x;
    float velocity_y;
    float ambient_temp;
    float altitude_m;
    uint8_t num_sats;
    float air_speed;
    uint16_t throttle_raw;
} TelemetryData;


// TRACK VARIABLES UNUSED 
#define SPEEDWAY_TRACK

#ifdef SPEEDWAY_TRACK
    #define ORIGIN_LONGITUDE_COORD  -86.238859
    #define ORIGIN_LATITUDE_COORD    39.793157
#endif  

#define FORTALEZA_TRACK

#ifdef FORTALEZA_TRACK
    #define ORIGIN_LONGITUDE_COORD_FORT  -103.45651
    #define ORIGIN_LATITUDE_COORD_FORT    20.736650
#endif  


// Telemetry Struct to store current Data
extern TelemetryData telemetry_data;

// Mutexes
extern SemaphoreHandle_t telemetry_mutex;
extern SemaphoreHandle_t i2c_mutex;

/*-------------- DISPLAY EXTRA DATA -------------*/
/*-----------  Webpage string parsing -----------*/
#define CUSTOM_MSG_MAX_LEN 100
#define CHAR_LEN (CUSTOM_MSG_MAX_LEN+1)
#define HTTP_RX_BUFFER_SIZE     256
#define MAX_CMD_TOKENS          5
#define CUSTOM_CHUNK_SIZE 6
#define CUSTOM_MAX_FRAMES ((CUSTOM_MSG_MAX_LEN + CUSTOM_CHUNK_SIZE - 1) / CUSTOM_CHUNK_SIZE)

typedef enum{
    CLEAR_DAY,
    CLEAR_NIGHT,
    PARTLY_CLOUDY_DAY,
    PARTLY_CLOUDY_NIGHT,
    FOG_HAZE_DAY,
    FOG_HAZE_NIGHT,
    OVERCAST,
    DRIZZLE,
    RAIN_SHOWER_DAY,
    RAIN_SHOWER_NIGHT,
    RAIN,
    SNOW,
    THUNDERSTORM,
}WeatherType;

typedef struct {
    uint8_t brightness;
    uint8_t att_flag;
    uint8_t lap_count;
    uint8_t gauge_unit;
    uint8_t map_zoom;
    uint8_t map_persp;
    uint8_t map_arrowpx;
    uint16_t base_throttle;
    uint16_t max_throttle;
    uint32_t lap_set_time;
    float ambient_temp;
    int32_t visibility;
    uint8_t precipitation;
    uint8_t humidity;
    uint8_t hour;
    WeatherType curr_weather;
    char custom_msg[CHAR_LEN];
} DisplayData;
extern DisplayData display_data;
extern SemaphoreHandle_t display_mutex;

// Vehicle Info and Tuning
#define WHEEL_RADIUS_M   0.48f
#define TWO_PI           6.28318530718f
#define CURRENT_VALID_THRESHOLD   1.0f   // A
#define RPM_ZERO_THRESHOLD       5       // rpm (deadband)
#define HEADING_WINDOW 20
#define THROTTLE_VALID_THRESHOLD  650   // tune (raw units)


#endif /* GLOBALS_H */