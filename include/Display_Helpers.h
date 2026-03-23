#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "Globals.h"

#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <ctype.h>
#include <stdio.h>
#include <math.h>

void FloatToBytes(float v, uint8_t *b);
void U32toBytes(uint32_t v, uint8_t *b);
void I32toBytes(int32_t v, uint8_t *b);
void U16toBytes(uint16_t v, uint8_t *b);
void I16toBytes(int16_t v, uint8_t *b);

bool json_extract_message(const char *json, char *out_msg, size_t out_size);
bool json_extract_is_running(const char *json, bool *is_running);
bool json_extract_current_lap(const char *json, uint8_t *lap);
void trim_leading_spaces(char **str);
bool execute_command_from_message(const char *message);
void update_status_flag_if_changed(bool is_running);
void update_lap_count_if_changed(uint8_t new_lap);
void store_display_message_if_new(const char *new_msg);