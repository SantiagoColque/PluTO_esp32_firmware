#ifndef MQTT_MANAGER_H
#define MQTT_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"

typedef void (*mqtt_data_cb_t)(const char *topic,
                               int topic_len,
                               const char *payload,
                               int payload_len);

/*
 * Topics, as defined in PluTO's docs/contracts:
 *   device/<device_id>/coordinates/polar  in   pointing payload (coordinates-dto.md)
 *   device/<device_id>/status             out  online/offline, retained, last will (device-state.md §2)
 *   device/<device_id>/state              out  JSON state (device-state.md §3)
 */
esp_err_t mqtt_manager_init(mqtt_data_cb_t data_cb, const char *device_id);
bool mqtt_manager_is_connected(void);

/**
 * @brief Publish a state document on device/<device_id>/state, QoS 0, not retained.
 *
 * Returns ESP_ERR_INVALID_STATE while disconnected: a state is only worth
 * sending when it is current, so nothing is queued.
 */
esp_err_t mqtt_manager_publish_state(const char *json, int json_len);

#endif
