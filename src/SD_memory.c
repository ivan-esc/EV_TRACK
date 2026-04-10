#include "../include/SD_memory.h"
#include "../include/Generic.h"
#include <stdbool.h>

const char *TAG_SD = "SD_WRITE";

#define SD_TELEMETRY_SAMPLE_PERIOD_MS 100
#define SD_TELEMETRY_BATCH_SIZE       10

static char current_csv_path[64] = "";
static TelemetryData telemetry_batch[SD_TELEMETRY_BATCH_SIZE];

static bool sd_file_exists(const char *path)
{
    FILE *f = fopen(path, "r");
    if (f) {
        fclose(f);
        return true;
    }
    return false;
}

static esp_err_t sd_make_unique_csv_path(const char *base_name, char *out_path, size_t out_len)
{
    if (!base_name || !out_path || out_len == 0) {
        return ESP_FAIL;
    }

    const char mount_root[] = "/sdcard/";

    if (snprintf(out_path, out_len, "%s%s.csv", mount_root, base_name) >= (int)out_len) {
        return ESP_FAIL;
    }
    if (!sd_file_exists(out_path)) {
        return ESP_OK;
    }

    for (int idx = 1; idx < 1000; ++idx) {
        if (snprintf(out_path, out_len, "%s%s_%d.csv", mount_root, base_name, idx) >= (int)out_len) {
            return ESP_FAIL;
        }
        if (!sd_file_exists(out_path)) {
            return ESP_OK;
        }
    }

    return ESP_FAIL;
}

static int sd_write_telemetry_row(FILE *f, const TelemetryData *data)
{
    if (!f || !data) {
        return -1;
    }
    // ESP_LOGI("SD", "Tick: %lu", xTaskGetTickCount());

    return fprintf(f,
        "%lu,%.3f,%.3f,%.8lf,%.8lf,"
        "%.3f,%.3f,%.3f,"
        "%.3f,%.3f,%.3f,"
        "%u,%.3f,%.3f,%.2f,%.2f,%u,%.2f,%d\n",
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
}

static esp_err_t sd_append_telemetry_csv_batch(const char *path, const TelemetryData *data, size_t count)
{
    if (!SD_card_detected) {
        ESP_LOGE(TAG_SD, "No SD card is present");
        return ESP_FAIL;
    }

    if (!path || !data || count == 0) {
        return ESP_FAIL;
    }

    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG_SD, "Failed to open CSV file for appending: %s", path);
        return ESP_FAIL;
    }

    for (size_t i = 0; i < count; ++i) {
        if (sd_write_telemetry_row(f, &data[i]) < 0) {
            ESP_LOGE(TAG_SD, "Failed to write telemetry batch");
            fclose(f);
            return ESP_FAIL;
        }
    }

    fflush(f);
    fclose(f);

    return ESP_OK;
}

sdmmc_card_t *card;


esp_err_t mount_sdcard(void)
{
    esp_err_t ret;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI3_HOST;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_SPI_CS_PIN;
    slot_config.host_id = (spi_host_device_t)host.slot;

    // SD Mount config
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    const char mount_point[] = "/sdcard";

    // 5) Mount the SD card
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK) {
        ESP_LOGE("SD", "Failed to mount card: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI("SD", "SD card mounted");
    return ret;
}


/**
 * @brief Unmount the SD card filesystem.
 *
 * This function unmounts the FAT filesystem from the SD card that was
 * previously mounted at the "/sdcard" mount point using
 * esp_vfs_fat_sdspi_mount() or esp_vfs_fat_sdmmc_mount().
 *
 * It should be called when the SD card is physically removed or when the
 * application no longer needs access to the SD card, in order to:
 *  - Flush pending writes
 *  - Release the VFS resources
 *  - Safely detach the card from the system
 *
 * @return
 *  - ESP_OK on successful unmount
 *  - An error code if unmounting fails (e.g., invalid handle or not mounted)
 */
esp_err_t unmount_sdcard(void){
    esp_err_t ret;
    ret = esp_vfs_fat_sdcard_unmount("/sdcard", card);
    return ret;
}


/**
 * @brief Append a line of text to a file on the SD card.
 *
 * @param path Full path to file, e.g. "/sdcard/log.txt"
 * @param text Text to write (without newline)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sd_append_line(const char *path, const char *text)
{
    if(!SD_card_detected){
        ESP_LOGE(TAG_SD, "No SD card is present");
        return ESP_FAIL;
    }

    // Open file in append mode
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG_SD, "Failed to open file for appending: %s", path);
        return ESP_FAIL;
    }

    // Write text + newline
    if (fprintf(f, "%s\n", text) < 0) {
        ESP_LOGE(TAG_SD, "Failed to write to file: %s", path);
        fclose(f);
        return ESP_FAIL;
    }

    // Make sure data is actually written
    fflush(f);
    fclose(f);

    ESP_LOGI(TAG_SD, "Wrote line to: %s", path);
    return ESP_OK;
}


/**
 * @brief Task that monitors SD card insertion/removal.
 *
 * Watches the SD_card_detected flag and mounts the SD card on insertion
 * and unmounts it on removal.
 *
 * @param arg Unused.
 */

void telemetry_sample_task(void *arg)
{
    TelemetryData *src = (TelemetryData *)arg;
    TelemetryData sample;

    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        // Copy shared telemetry safely
        xSemaphoreTake(telemetry_mutex, portMAX_DELAY);
        sample = *src;
        xSemaphoreGive(telemetry_mutex);

        // Assign timestamp at sampling moment
        sample.timestamp = telemetry_timestamp_ms();

        // Send to queue (DO NOT BLOCK)
        if (xQueueSend(sd_queue, &sample, 0) != pdTRUE) {
            // Optional: drop or overwrite
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100));
    }
}


void SD_manager_task(void *arg){
    TelemetryData *telemetry_data = (TelemetryData *)arg;
    bool last_state = false;
    size_t batch_count = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        if (SD_card_detected != last_state) {
            last_state = SD_card_detected;

            if (last_state) {
                ESP_LOGI("SD", "Card inserted");
                esp_err_t ret = mount_sdcard();
                if (ret == ESP_OK) {
                    char new_csv_path[sizeof(current_csv_path)] = "";

                    if (sd_make_unique_csv_path("telem", new_csv_path, sizeof(new_csv_path)) == ESP_OK &&
                        sd_write_csv_header(new_csv_path) == ESP_OK) {
                        snprintf(current_csv_path, sizeof(current_csv_path), "%s", new_csv_path);
                        batch_count = 0;
                    } else {
                        current_csv_path[0] = '\0';
                        ESP_LOGE("SD", "Failed to initialize telemetry CSV file");
                    }
                }
            } else {
                ESP_LOGI("SD", "Card removed");
                if (batch_count > 0) {
                    ESP_LOGW("SD", "Dropping %u buffered samples after card removal", (unsigned)batch_count);
                    batch_count = 0;
                }
                unmount_sdcard();
                current_csv_path[0] = '\0';
            }
        }

        if (SD_card_detected && current_csv_path[0] != '\0') {
            TelemetryData sample;

            while (xQueueReceive(sd_queue, &sample, 0) == pdTRUE) {

                telemetry_batch[batch_count++] = sample;

                if (batch_count == SD_TELEMETRY_BATCH_SIZE) {
                    if (sd_append_telemetry_csv_batch(current_csv_path, telemetry_batch, batch_count) == ESP_OK) {
                        batch_count = 0;
                    }
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}


/**
 * @brief Create a CSV file and write the telemetry header line.
 *
 * This function creates (or overwrites) a CSV file at the given path and
 * writes the column headers corresponding to the TelemetryData fields.
 * It should be called once before appending telemetry samples.
 *
 * @param path Full path to the CSV file (e.g., "/sdcard/telemetry.csv").
 * @return
 *  - ESP_OK on success
 *  - ESP_FAIL if the file could not be created or written
 */
esp_err_t sd_write_csv_header(const char *path)
{
    if(!SD_card_detected){
        ESP_LOGE(TAG_SD, "No SD card is present");
        return ESP_FAIL;
    }

    const char *header =
        "Time,Battery Voltage,Battery Current,GPS Latitutde,GPS Longitude,"
        "G Force Lat,G Force Long,G Force Vert,"
        "orient_x,orient_y,orient_z,"
        "Engine RPM,velocity_x,velocity_y,ambient_temp,altitude_m,num_sats,air_speed,throttle_raw";

    FILE *f = fopen(path, "w");   // "w" creates/overwrites and writes header once
    if (!f) {
        ESP_LOGE("SD", "Failed to create CSV file: %s", path);
        return ESP_FAIL;
    }

    fprintf(f, "%s\n", header);
    fflush(f);
    fclose(f);

    // ESP_LOGI("SD", "CSV header written to %s", path);
    return ESP_OK;
}


/**
 * @brief Append one TelemetryData sample as a CSV row.
 *
 * Writes the fields of a TelemetryData structure to a CSV file in the same
 * order as the header created by sd_write_csv_header().
 *
 * @param path Full path to the CSV file (e.g. "/sdcard/telem.csv").
 * @param data Pointer to the TelemetryData structure to log.
 *
 * @return
 *  - ESP_OK on success
 *  - ESP_FAIL if the SD card is not present or the write fails
 */
esp_err_t sd_append_telemetry_csv(const char *path, const TelemetryData *data)
{
    return sd_append_telemetry_csv_batch(path, data, 1);
}


void SD_append_data_task(void *arg){
    TelemetryData *telem = (TelemetryData *)arg;  // cast

    while (1) {
        if (current_csv_path[0] != '\0') {
            sd_append_telemetry_csv(current_csv_path, telem);
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}