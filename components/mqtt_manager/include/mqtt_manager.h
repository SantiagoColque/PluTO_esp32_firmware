#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"

typedef void (*mqtt_data_cb_t)(const char *topic,
                               int topic_len,
                               const char *payload,
                               int payload_len);

/*
 * Topics:
 *   device/<device_id>/coordinates/polar  in   pointing payload
 *   device/<device_id>/status             out  online/offline, retained, last will
 *   device/<device_id>/state              out  JSON state
 */
esp_err_t mqtt_manager_init(mqtt_data_cb_t data_cb, const char *device_id);
bool mqtt_manager_is_connected(void);

/**
 * @brief Publish a state document on device/<device_id>/state.
 *
 * QoS 0, not retained. ESP_ERR_INVALID_STATE while disconnected: a stale
 * state is worthless, so nothing is queued.
 */
esp_err_t mqtt_manager_publish_state(const char *json, int json_len);

#endif
