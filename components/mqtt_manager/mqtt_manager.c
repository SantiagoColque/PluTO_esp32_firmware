#include "mqtt_manager.h"

#include <stdio.h>
#include <string.h>

#include "config_store.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "sdkconfig.h"
#include "wifi_manager.h"

#ifndef CONFIG_MQTT_TOPIC_SUBSCRIPTION
#define CONFIG_MQTT_TOPIC_SUBSCRIPTION "device/subscription"
#endif

#define MQTT_TOPIC_LEN 64

/* The broker publishes the last will after 1.5 keepalives without traffic, so
 * this is what bounds how long a dead board still looks online. */
#define MQTT_KEEPALIVE_SEC 15

#define STATUS_ONLINE "online"
#define STATUS_OFFLINE "offline"

static const char *TAG = "mqtt_manager";

static esp_mqtt_client_handle_t s_client;
static mqtt_data_cb_t s_data_cb;
static bool s_mqtt_connected;
static char s_device_id[WIFI_DEVICE_ID_LEN];
static char s_coordinates_topic[MQTT_TOPIC_LEN];
static char s_status_topic[MQTT_TOPIC_LEN];
static char s_state_topic[MQTT_TOPIC_LEN];

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
    {
        int registration_msg_id;
        int subscribe_msg_id;

        s_mqtt_connected = true;
        ESP_LOGI(TAG, "Connected to broker: %s", config_store_peek()->mqtt_uri);

        /* Retained, so whoever subscribes later sees it; the last will
         * replaces it with "offline" if the board drops. */
        esp_mqtt_client_publish(s_client, s_status_topic, STATUS_ONLINE, 0, 1, 1);

        registration_msg_id = esp_mqtt_client_publish(s_client,
                                                      CONFIG_MQTT_TOPIC_SUBSCRIPTION,
                                                      s_device_id,
                                                      0,
                                                      1,
                                                      0);
        ESP_LOGI(TAG,
                 "Published device registration to %s with device_id=%s (msg_id=%d)",
                 CONFIG_MQTT_TOPIC_SUBSCRIPTION,
                 s_device_id,
                 registration_msg_id);

        subscribe_msg_id = esp_mqtt_client_subscribe(s_client, s_coordinates_topic, 1);
        ESP_LOGI(TAG, "Subscribed to: %s (msg_id=%d)", s_coordinates_topic, subscribe_msg_id);
        break;
    }

    case MQTT_EVENT_DISCONNECTED:
        s_mqtt_connected = false;
        ESP_LOGW(TAG, "Disconnected from broker");
        break;

    case MQTT_EVENT_DATA:
        /* A message bigger than the client buffer arrives split over several
         * events. No valid payload comes close (147 bytes at most), so one
         * that arrives split is dropped instead of being decoded in pieces. */
        if (event->current_data_offset != 0 || event->data_len != event->total_data_len) {
            ESP_LOGW(TAG, "Dropping a fragmented message (%d bytes)", event->total_data_len);
            break;
        }
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

esp_err_t mqtt_manager_init(mqtt_data_cb_t data_cb, const char *device_id)
{
    /* Borrowed rather than copied: esp-mqtt duplicates these strings when the
     * client is created, and the cache outlives the call either way. */
    const pluto_config_t *cfg = config_store_peek();

    if (s_client != NULL) {
        return ESP_OK;
    }

    if (device_id == NULL || device_id[0] == '\0') {
        ESP_LOGE(TAG, "device_id is empty");
        return ESP_ERR_INVALID_ARG;
    }

    if (cfg->mqtt_uri[0] == '\0') {
        ESP_LOGE(TAG, "No broker URI stored; set one from the dashboard");
        return ESP_ERR_INVALID_STATE;
    }

    if (cfg->mqtt_username[0] == '\0') {
        ESP_LOGW(TAG, "MQTT username is empty; set one from the dashboard if the broker requires authentication");
    }

    s_data_cb = data_cb;
    strlcpy(s_device_id, device_id, sizeof(s_device_id));
    snprintf(s_coordinates_topic, sizeof(s_coordinates_topic), "device/%s/coordinates/polar", s_device_id);
    snprintf(s_status_topic, sizeof(s_status_topic), "device/%s/status", s_device_id);
    snprintf(s_state_topic, sizeof(s_state_topic), "device/%s/state", s_device_id);

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = cfg->mqtt_uri,
        .credentials.username = cfg->mqtt_username,
        .credentials.authentication.password = cfg->mqtt_password,
        .session.keepalive = MQTT_KEEPALIVE_SEC,
        .session.last_will = {
            .topic = s_status_topic,
            .msg = STATUS_OFFLINE,
            .msg_len = sizeof(STATUS_OFFLINE) - 1,
            .qos = 1,
            .retain = 1,
        },
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

esp_err_t mqtt_manager_publish_state(const char *json, int json_len)
{
    if (s_client == NULL || !s_mqtt_connected) {
        return ESP_ERR_INVALID_STATE;
    }
    return esp_mqtt_client_publish(s_client, s_state_topic, json, json_len, 0, 0) < 0 ? ESP_FAIL : ESP_OK;
}
