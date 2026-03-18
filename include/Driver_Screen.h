#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "Globals.h"
#include <math.h>
#include "Display_Helpers.h"

#define TAG_CAN "CAN_TX"

#define CAN_ERROR_STATE   2
#define CAN_RUNNING_STATE 1

// CAN GPIOs
#define CAN_TX_GPIO GPIO_NUM_16
#define CAN_RX_GPIO GPIO_NUM_17

// CAN IDs
#define DISP_START_ATTEMPT 0x001
#define DISP_RESET_ALL 0x002
#define DISP_SET_LAP_INFO 0x003
#define DISP_SET_RPM_INFO 0x004
#define DISP_SEND_RPM 0x005
#define MAP_SET_TRACK 0x006
#define MAP_SET_VIEW 0x007
#define MAP_SET_COORDS 0x008
#define MAP_SET_HEADING 0x009
#define MAP_ADD_POI_1 0x010
#define MAP_ADD_POI_2 0x011
#define MAP_ADD_POI_3 0x012
#define MAP_REMOVE_POI 0x013
#define MAP_ADD_DEF_POI 0x014
#define DISP_MSG_TEMPLATE 0x015
#define DISP_MSG_CUSTOM 0x016
#define DISP_SET_BASE_POWER 0x017
#define DISP_SEND_POWER 0x018
#define DISP_MAX_BRIGHTNESS 0x019

#define CAN_TEST_ID 0x100
#define CAN_TX_QUEUE_LEN 32
extern QueueHandle_t can_tx_queue;


extern uint8_t payload_base;

void can_init(void);
void can_tx_task(void *arg);

//Periodic sent tasks
void map_data_send_periodic_CAN(void *arg);
void telemetry_send_periodic_CAN(void *arg);

//Event sent tasks
void brightness_send_event_CAN(uint8_t value);
void watt_range_send_event_CAN(float value1, float value2);
void add_poi_send_event_CAN(uint8_t id, uint8_t color, double lat, double lon);
void new_message_send_event_CAN();

//Frame structure struct
typedef struct {
    uint32_t identifier;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;