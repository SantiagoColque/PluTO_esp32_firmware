#ifndef CONFIG_STORE_H
#define CONFIG_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define PLUTO_WIFI_SSID_LEN 33
#define PLUTO_WIFI_PASS_LEN 65
#define PLUTO_MQTT_URI_LEN 128
#define PLUTO_MQTT_USER_LEN 65
#define PLUTO_MQTT_PASS_LEN 65
#define PLUTO_NTP_SERVER_LEN 64
#define PLUTO_NTP_TZ_LEN 40
#define PLUTO_ADMIN_PASS_LEN 33

typedef struct {
    char wifi_ssid[PLUTO_WIFI_SSID_LEN];
    char wifi_password[PLUTO_WIFI_PASS_LEN];
    char mqtt_uri[PLUTO_MQTT_URI_LEN];
    char mqtt_username[PLUTO_MQTT_USER_LEN];
    char mqtt_password[PLUTO_MQTT_PASS_LEN];
    char ntp_server[PLUTO_NTP_SERVER_LEN];
    char ntp_timezone[PLUTO_NTP_TZ_LEN];
    char admin_password[PLUTO_ADMIN_PASS_LEN];
    uint8_t home_channel;
    bool provisioned;
} pluto_config_t;

esp_err_t config_store_init(void);
esp_err_t config_store_get(pluto_config_t *out);
esp_err_t config_store_set(const pluto_config_t *in);
esp_err_t config_store_validate(const pluto_config_t *in, char *err, size_t err_len);
esp_err_t config_store_set_admin_password(const char *password);
esp_err_t config_store_set_home_channel(uint8_t channel);
bool config_store_is_provisioned(void);
esp_err_t config_store_factory_reset(void);

/**
 * @brief Borrow the RAM cache instead of copying it.
 *
 * The returned pointer stays valid for the lifetime of the program, which some
 * callers need: esp_sntp_setservername() stores the pointer it is given rather
 * than copying the string, so handing it a field of a stack-allocated
 * pluto_config_t leaves a dangling pointer behind.
 *
 * Never dereference it concurrently with config_store_set().
 */
const pluto_config_t *config_store_peek(void);

#endif
