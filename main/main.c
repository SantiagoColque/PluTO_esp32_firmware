#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "mqtt_manager.h"
#include "wifi_manager.h"

static const char *TAG = "pluto_main";

static void coordinates_message_handler(const char *topic,
                                        int topic_len,
                                        const char *payload,
                                        int payload_len)
{
    ESP_LOGI(TAG,
             "Coordinates message received on %.*s: %.*s",
             topic_len,
             topic,
             payload_len,
             payload);
}

void app_main(void)
{
    char device_id[WIFI_DEVICE_ID_LEN];
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(wifi_manager_init());
    ESP_ERROR_CHECK(wifi_manager_get_device_id(device_id, sizeof(device_id)));

    ESP_LOGI(TAG, "Using user_id: %s", device_id);
    ESP_ERROR_CHECK(mqtt_manager_init(coordinates_message_handler, device_id));
}
