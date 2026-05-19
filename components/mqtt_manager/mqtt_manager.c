#include "mqtt_manager.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "sdkconfig.h"

#ifndef CONFIG_MQTT_BROKER_USERNAME
#define CONFIG_MQTT_BROKER_USERNAME ""
#endif

#ifndef CONFIG_MQTT_BROKER_PASSWORD
#define CONFIG_MQTT_BROKER_PASSWORD ""
#endif

static const char *TAG = "mqtt_manager";

static esp_mqtt_client_handle_t s_client;
static mqtt_data_cb_t s_data_cb;
static bool s_mqtt_connected;

static void mqtt_event_handler(void *handler_args,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    (void)handler_args;
    (void)base;

    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        s_mqtt_connected = true;
        ESP_LOGI(TAG, "Connected to broker: %s", CONFIG_MQTT_BROKER_URI);
        esp_mqtt_client_subscribe(s_client, CONFIG_MQTT_TOPIC_COORDINATES, 1);
        ESP_LOGI(TAG, "Subscribed to: %s", CONFIG_MQTT_TOPIC_COORDINATES);
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected = false;
        ESP_LOGW(TAG, "Disconnected from broker");
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "Message received on %.*s", event->topic_len, event->topic);
        if (s_data_cb != NULL) {
            s_data_cb(event->topic, event->topic_len, event->data, event->data_len);
        }
        break;

    case MQTT_EVENT_ERROR:
        s_mqtt_connected = false;
        if (event->error_handle != NULL) {
            ESP_LOGE(TAG,
                     "MQTT error event: type=%d connect_return_code=%d esp_tls_last_esp_err=0x%x transport_sock_errno=%d",
                     event->error_handle->error_type,
                     event->error_handle->connect_return_code,
                     event->error_handle->esp_tls_last_esp_err,
                     event->error_handle->esp_transport_sock_errno);
        } else {
            ESP_LOGE(TAG, "MQTT error event received");
        }
        break;

    default:
        break;
    }
}

esp_err_t mqtt_manager_init(mqtt_data_cb_t data_cb)
{
    if (s_client != NULL) {
        return ESP_OK;
    }

    if (CONFIG_MQTT_BROKER_URI[0] == '\0') {
        ESP_LOGE(TAG, "CONFIG_MQTT_BROKER_URI is empty; set it with idf.py menuconfig");
        return ESP_ERR_INVALID_STATE;
    }

    if (CONFIG_MQTT_BROKER_USERNAME[0] == '\0') {
        ESP_LOGW(TAG,
                 "MQTT username is empty; if the broker requires authentication, update sdkconfig with idf.py menuconfig");
    }

    s_data_cb = data_cb;

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI,
        .credentials.username = CONFIG_MQTT_BROKER_USERNAME,
        .credentials.authentication.password = CONFIG_MQTT_BROKER_PASSWORD,
    };

    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (s_client == NULL) {
        return ESP_FAIL;
    }

    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_client,
                                                   ESP_EVENT_ANY_ID,
                                                   mqtt_event_handler,
                                                   NULL));

    return esp_mqtt_client_start(s_client);
}

bool mqtt_manager_is_connected(void)
{
    return s_mqtt_connected;
}
