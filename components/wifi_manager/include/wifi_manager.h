#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stddef.h>
#include <stdbool.h>

#include "esp_err.h"

#define WIFI_DEVICE_ID_LEN 13

esp_err_t wifi_manager_init(void);
esp_err_t wifi_manager_get_device_id(char *buf, size_t buf_len);
bool wifi_manager_is_connected(void);

#endif
