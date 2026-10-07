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

/*
 * The mount's calibration. The geographic heading the pan faces at 0 and the
 * usable range of each joint, in centidegrees, mean what the fields of the
 * same name in pan_tilt.h do. The pulses, in microseconds, are each servo's
 * two-point calibration: the pulse that puts it at 0 degrees and the one that
 * puts it at 180, as in servo_calibration_t (servo_pulse.h). Plain integers
 * rather than those types so this component depends on neither the kinematics
 * nor the servo driver; main converts them.
 *
 * Stored apart from pluto_config_t, in a namespace of its own, so that
 * re-provisioning the WiFi with config_store_factory_reset() does not lose it.
 */
typedef struct {
    int32_t az_ref_cdeg;
    int32_t pan_min_cdeg;
    int32_t pan_max_cdeg;
    int32_t tilt_min_cdeg;
    int32_t tilt_max_cdeg;
    int32_t pan_us_at_0;
    int32_t pan_us_at_180;
    int32_t tilt_us_at_0;
    int32_t tilt_us_at_180;
} pluto_calibration_t;

esp_err_t config_store_init(void);
esp_err_t config_store_get(pluto_config_t *out);
esp_err_t config_store_set(const pluto_config_t *in);
esp_err_t config_store_validate(const pluto_config_t *in, char *err, size_t err_len);
esp_err_t config_store_set_admin_password(const char *password);
esp_err_t config_store_set_home_channel(uint8_t channel);
bool config_store_is_provisioned(void);

/**
 * @brief Erase the network and dashboard configuration (pluto_config_t).
 *
 * The mount's calibration lives in another namespace and is kept.
 */
esp_err_t config_store_factory_reset(void);

/**
 * @brief Copy the mount's calibration.
 *
 * config_store_init() loads it: from NVS when it holds one, otherwise from the
 * menuconfig seed (PLUTO_CAL_* for the mount, PLUTO_SERVO_*_US_AT_* for the
 * pulses), which is then written to NVS. It is not range-checked here; check
 * it with pan_tilt_config_valid() and servo_calibration_valid() before using it.
 */
esp_err_t config_store_get_calibration(pluto_calibration_t *out);

/**
 * @brief Store the mount's calibration in NVS and in the RAM copy.
 *
 * Stores whatever it is given: validate first. The rotor copies the
 * calibration once when it starts, so a new one applies after a restart.
 */
esp_err_t config_store_set_calibration(const pluto_calibration_t *in);

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
