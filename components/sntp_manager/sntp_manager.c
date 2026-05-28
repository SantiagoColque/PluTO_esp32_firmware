#include "sntp_manager.h"

#include <stdlib.h>
#include <time.h>

#include "esp_log.h"
#include "esp_sntp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

#define SNTP_RETRY_COUNT 10
#define SNTP_RETRY_DELAY_MS 2000
#define SNTP_VALID_YEAR 2024

static const char *TAG = "sntp_manager";
static bool s_time_synced;

esp_err_t sntp_manager_init(void)
{
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;

    if (s_time_synced) {
        return ESP_OK;
    }

    ESP_LOGI(TAG, "Configuring SNTP server: %s", CONFIG_NTP_SERVER);
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, CONFIG_NTP_SERVER);
    esp_sntp_init();

    setenv("TZ", CONFIG_NTP_TIMEZONE, 1);
    tzset();

    time(&now);
    localtime_r(&now, &timeinfo);

    while (timeinfo.tm_year < (SNTP_VALID_YEAR - 1900) && retry < SNTP_RETRY_COUNT) {
        retry++;
        ESP_LOGI(TAG,
                 "Waiting for time synchronization... (%d/%d)",
                 retry,
                 SNTP_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(SNTP_RETRY_DELAY_MS));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (timeinfo.tm_year < (SNTP_VALID_YEAR - 1900)) {
        ESP_LOGE(TAG, "Time synchronization timed out");
        return ESP_ERR_TIMEOUT;
    }

    s_time_synced = true;
    ESP_LOGI(TAG, "Local time synchronized: %04d-%02d-%02d %02d:%02d:%02d",
             timeinfo.tm_year + 1900,
             timeinfo.tm_mon + 1,
             timeinfo.tm_mday,
             timeinfo.tm_hour,
             timeinfo.tm_min,
             timeinfo.tm_sec);

    return ESP_OK;
}