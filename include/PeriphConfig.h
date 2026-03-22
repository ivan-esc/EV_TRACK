#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "freertos/timers.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "math.h"
#include "BNO055.h"

#include "string.h"
#include "Globals.h"

#include "MT3333_GPS.h"
// #include "BNO055.h"
#include "Globals.h"

// CONFIGURATION FUNCTIONS 
void GPIO_Config();
void UART_Config();
void I2C_Config();
esp_err_t SPI_Config();
void Peripheral_Config();