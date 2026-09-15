#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#include "config_store.h"
#include "mqtt_manager.h"
#include "sntp_manager.h"
#include "web_server.h"
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

/*
 * The captive DNS responder can only bind once the access point interface has
 * an address, so it follows the AP's own lifecycle rather than being started
 * alongside it. Living here keeps wifi_manager free of any dependency on
 * web_server, which would otherwise be a circular one.
 */
static void ap_lifecycle_handler(void *arg,
                                 esp_event_base_t event_base,
                                 int32_t event_id,
                                 void *event_data)
{
    if (event_id == WIFI_EVENT_AP_START) {
        web_server_set_captive(true);
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        web_server_set_captive(false);
    }
}

static esp_err_t init_nvs(void)
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    return ret;
}

void app_main(void)
{
    char device_id[WIFI_DEVICE_ID_LEN];

    ESP_ERROR_CHECK(init_nvs());
    ESP_ERROR_CHECK(config_store_init());
    ESP_ERROR_CHECK(wifi_manager_init());
    ESP_ERROR_CHECK(wifi_manager_get_device_id(device_id, sizeof(device_id)));

    ESP_LOGI(TAG, "Using device_id: %s", device_id);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &ap_lifecycle_handler,
                                                        NULL,
                                                        NULL));

    /* One server for both roles: the captive portal while the access point is
     * up, the dashboard on the LAN address once the station is connected. */
    ESP_ERROR_CHECK(web_server_start());

    if (!config_store_is_provisioned()) {
        ESP_LOGW(TAG, "Not provisioned; starting the setup portal");
        ESP_ERROR_CHECK(wifi_manager_start_provisioning());
        /* Nothing to wait for: configuration arrives over HTTP and the device
         * restarts into the station path. */
        return;
    }

    ESP_ERROR_CHECK(wifi_manager_start_sta());

    /* Only this task blocks. The WiFi manager, the HTTP server and the event
     * loop all run on their own tasks, so the dashboard stays reachable and
     * reconnection keeps going for as long as this takes. */
    ESP_ERROR_CHECK(wifi_manager_wait_connected(portMAX_DELAY));

    /* A missing clock is not worth a reboot loop over: the poller keeps trying
     * in the background. */
    if (sntp_manager_init() != ESP_OK) {
        ESP_LOGW(TAG, "Continuing without a synchronized clock");
    }

    ESP_ERROR_CHECK(mqtt_manager_init(coordinates_message_handler, device_id));
}
