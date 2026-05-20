#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"

typedef void (*mqtt_data_cb_t)(const char *topic,
                               int topic_len,
                               const char *payload,
                               int payload_len);

esp_err_t mqtt_manager_init(mqtt_data_cb_t data_cb, const char *device_id);
bool mqtt_manager_is_connected(void);

#endif
