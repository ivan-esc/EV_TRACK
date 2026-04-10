#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <stdio.h>
#include "esp_log.h"
#include "esp_err.h"
#include "Globals.h"


#ifdef __cplusplus
extern "C" {
#endif

extern sdmmc_card_t *card;

extern SemaphoreHandle_t sd_mutex;

esp_err_t mount_sdcard(void);
esp_err_t unmount_sdcard(void);
esp_err_t sd_append_line(const char *path, const char *text);
esp_err_t sd_write_csv_header(const char *path);
esp_err_t sd_append_telemetry_csv(const char *path, const TelemetryData *data);

// RTOS tasks
void SD_manager_task(void *);
// void SD_append_data_task(void *);

#ifdef __cplusplus
}
#endif