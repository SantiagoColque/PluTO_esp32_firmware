#include "config_store.h"

#include <stdio.h>
#include <string.h>

#include "esp_check.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define CONFIG_STORE_NAMESPACE "pluto_cfg"
#define CONFIG_STORE_VERSION 1

/* NVS keys are capped at 15 characters. */
#define KEY_VERSION "cfg_ver"
#define KEY_PROVISIONED "prov"
#define KEY_WIFI_SSID "wifi_ssid"
#define KEY_WIFI_PASS "wifi_pass"
#define KEY_MQTT_URI "mqtt_uri"
#define KEY_MQTT_USER "mqtt_user"
#define KEY_MQTT_PASS "mqtt_pass"
#define KEY_NTP_SERVER "ntp_srv"
#define KEY_NTP_TZ "ntp_tz"
#define KEY_ADMIN_PASS "admin_pass"
#define KEY_HOME_CHANNEL "home_ch"

/*
 * The mount's calibration gets a namespace of its own because
 * config_store_factory_reset() erases CONFIG_STORE_NAMESPACE whole to bring the
 * setup portal back, and moving to another WiFi network says nothing about
 * where the antenna points.
 *
 * It is still lost when init_nvs() in main.c erases the entire NVS partition,
 * which it does on ESP_ERR_NVS_NO_FREE_PAGES or ESP_ERR_NVS_NEW_VERSION_FOUND.
 * The next boot then seeds it from menuconfig again, like the rest.
 */
#define CALIBRATION_NAMESPACE "pluto_cal"

#define KEY_CAL_AZ_REF "az_ref"
#define KEY_CAL_PAN_MIN "pan_min"
#define KEY_CAL_PAN_MAX "pan_max"
#define KEY_CAL_TILT_MIN "tilt_min"
#define KEY_CAL_TILT_MAX "tilt_max"
#define KEY_CAL_PAN_US_AT_0 "pan_us0"
#define KEY_CAL_PAN_US_AT_180 "pan_us180"
#define KEY_CAL_TILT_US_AT_0 "tilt_us0"
#define KEY_CAL_TILT_US_AT_180 "tilt_us180"
#define KEY_CAL_PAN_ENC_ZERO "pan_enc_zero"
#define KEY_CAL_PAN_ENC_DIR "pan_enc_dir"
#define KEY_CAL_TILT_ENC_ZERO "tilt_enc_zero"
#define KEY_CAL_TILT_ENC_DIR "tilt_enc_dir"

static const char *TAG = "config_store";

static pluto_config_t s_cache;
/* Guarded by s_mutex, like s_cache: the dashboard writes it while main may be
 * reading it in start_rotor(). */
static pluto_calibration_t s_calibration;
static SemaphoreHandle_t s_mutex;
static bool s_initialized;

static void load_string(nvs_handle_t handle, const char *key, char *dest, size_t dest_len)
{
    size_t len = dest_len;

    if (nvs_get_str(handle, key, dest, &len) != ESP_OK) {
        /* Absent keys are normal on a fresh device; the caller has already
         * placed the default in dest. */
        return;
    }
}

static esp_err_t store_string(nvs_handle_t handle, const char *key, const char *value)
{
    return nvs_set_str(handle, key, value);
}

static void derive_default_admin_password(char *buf, size_t buf_len)
{
    uint8_t mac[6];

    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        strlcpy(buf, "pluto", buf_len);
        return;
    }

    snprintf(buf, buf_len, "pluto-%02x%02x%02x", mac[3], mac[4], mac[5]);
}

static void apply_kconfig_seed(pluto_config_t *cfg)
{
    strlcpy(cfg->wifi_ssid, CONFIG_WIFI_SSID, sizeof(cfg->wifi_ssid));
    strlcpy(cfg->wifi_password, CONFIG_WIFI_PASSWORD, sizeof(cfg->wifi_password));
    strlcpy(cfg->mqtt_uri, CONFIG_MQTT_BROKER_URI, sizeof(cfg->mqtt_uri));
    strlcpy(cfg->mqtt_username, CONFIG_MQTT_BROKER_USERNAME, sizeof(cfg->mqtt_username));
    strlcpy(cfg->mqtt_password, CONFIG_MQTT_BROKER_PASSWORD, sizeof(cfg->mqtt_password));
    strlcpy(cfg->ntp_server, CONFIG_NTP_SERVER, sizeof(cfg->ntp_server));
    strlcpy(cfg->ntp_timezone, CONFIG_NTP_TIMEZONE, sizeof(cfg->ntp_timezone));
    derive_default_admin_password(cfg->admin_password, sizeof(cfg->admin_password));
    cfg->home_channel = 0;
    cfg->provisioned = cfg->wifi_ssid[0] != '\0';
}

static esp_err_t write_all(nvs_handle_t handle, const pluto_config_t *cfg)
{
    ESP_RETURN_ON_ERROR(store_string(handle, KEY_WIFI_SSID, cfg->wifi_ssid), TAG, "wifi_ssid");
    ESP_RETURN_ON_ERROR(store_string(handle, KEY_WIFI_PASS, cfg->wifi_password), TAG, "wifi_pass");
    ESP_RETURN_ON_ERROR(store_string(handle, KEY_MQTT_URI, cfg->mqtt_uri), TAG, "mqtt_uri");
    ESP_RETURN_ON_ERROR(store_string(handle, KEY_MQTT_USER, cfg->mqtt_username), TAG, "mqtt_user");
    ESP_RETURN_ON_ERROR(store_string(handle, KEY_MQTT_PASS, cfg->mqtt_password), TAG, "mqtt_pass");
    ESP_RETURN_ON_ERROR(store_string(handle, KEY_NTP_SERVER, cfg->ntp_server), TAG, "ntp_srv");
    ESP_RETURN_ON_ERROR(store_string(handle, KEY_NTP_TZ, cfg->ntp_timezone), TAG, "ntp_tz");
    ESP_RETURN_ON_ERROR(store_string(handle, KEY_ADMIN_PASS, cfg->admin_password), TAG, "admin_pass");
    ESP_RETURN_ON_ERROR(nvs_set_u8(handle, KEY_HOME_CHANNEL, cfg->home_channel), TAG, "home_ch");
    ESP_RETURN_ON_ERROR(nvs_set_u8(handle, KEY_PROVISIONED, cfg->provisioned ? 1 : 0), TAG, "prov");
    ESP_RETURN_ON_ERROR(nvs_set_u8(handle, KEY_VERSION, CONFIG_STORE_VERSION), TAG, "cfg_ver");

    return nvs_commit(handle);
}

static void apply_calibration_seed(pluto_calibration_t *cal)
{
    cal->az_ref_cdeg = CONFIG_PLUTO_CAL_AZ_REF_CDEG;
    cal->pan_min_cdeg = CONFIG_PLUTO_CAL_PAN_MIN_CDEG;
    cal->pan_max_cdeg = CONFIG_PLUTO_CAL_PAN_MAX_CDEG;
    cal->tilt_min_cdeg = CONFIG_PLUTO_CAL_TILT_MIN_CDEG;
    cal->tilt_max_cdeg = CONFIG_PLUTO_CAL_TILT_MAX_CDEG;
    /* The pulses' seed lives in the servo component's menu, next to the
     * DS3218 version that sets their defaults, rather than in main's. */
    cal->pan_us_at_0 = CONFIG_PLUTO_SERVO_PAN_US_AT_0;
    cal->pan_us_at_180 = CONFIG_PLUTO_SERVO_PAN_US_AT_180;
    cal->tilt_us_at_0 = CONFIG_PLUTO_SERVO_TILT_US_AT_0;
    cal->tilt_us_at_180 = CONFIG_PLUTO_SERVO_TILT_US_AT_180;
    /* Measured once the encoders are mounted; until then, no offset and no reversal. */
    cal->pan_enc_zero = 0;
    cal->pan_enc_dir = 1;
    cal->tilt_enc_zero = 0;
    cal->tilt_enc_dir = 1;
}

/* One key per field, like pluto_cfg, so a field added later loads on a device
 * that never stored it (it keeps its seed), without a migration. */
static esp_err_t write_calibration(nvs_handle_t handle, const pluto_calibration_t *cal)
{
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_AZ_REF, cal->az_ref_cdeg), TAG, "az_ref");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_PAN_MIN, cal->pan_min_cdeg), TAG, "pan_min");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_PAN_MAX, cal->pan_max_cdeg), TAG, "pan_max");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_TILT_MIN, cal->tilt_min_cdeg), TAG, "tilt_min");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_TILT_MAX, cal->tilt_max_cdeg), TAG, "tilt_max");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_PAN_US_AT_0, cal->pan_us_at_0), TAG, "pan_us0");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_PAN_US_AT_180, cal->pan_us_at_180), TAG, "pan_us180");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_TILT_US_AT_0, cal->tilt_us_at_0), TAG, "tilt_us0");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_TILT_US_AT_180, cal->tilt_us_at_180), TAG, "tilt_us180");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_PAN_ENC_ZERO, cal->pan_enc_zero), TAG, "pan_enc_zero");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_PAN_ENC_DIR, cal->pan_enc_dir), TAG, "pan_enc_dir");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_TILT_ENC_ZERO, cal->tilt_enc_zero), TAG, "tilt_enc_zero");
    ESP_RETURN_ON_ERROR(nvs_set_i32(handle, KEY_CAL_TILT_ENC_DIR, cal->tilt_enc_dir), TAG, "tilt_enc_dir");

    return nvs_commit(handle);
}

static void load_i32(nvs_handle_t handle, const char *key, int32_t *dest)
{
    int32_t value;

    /* An absent key keeps the seed the caller already placed in dest. */
    if (nvs_get_i32(handle, key, &value) == ESP_OK) {
        *dest = value;
    }
}

/*
 * Fills cal from NVS, or seeds NVS when it holds no calibration yet. On any
 * error cal is left holding the menuconfig seed, so the caller always has a
 * value to hand on; it is range-checked again before the rotor uses it.
 */
static esp_err_t load_calibration(pluto_calibration_t *cal)
{
    nvs_handle_t handle;
    int32_t probe;
    bool seed_needed;
    esp_err_t err = ESP_OK;

    apply_calibration_seed(cal);

    ESP_RETURN_ON_ERROR(nvs_open(CALIBRATION_NAMESPACE, NVS_READWRITE, &handle),
                        TAG,
                        "Unable to open the calibration namespace");

    seed_needed = nvs_get_i32(handle, KEY_CAL_AZ_REF, &probe) == ESP_ERR_NVS_NOT_FOUND;

#if CONFIG_PLUTO_FORCE_KCONFIG_SEED
    if (!seed_needed) {
        ESP_LOGW(TAG, "PLUTO_FORCE_KCONFIG_SEED is on: overwriting the stored calibration");
    }
    seed_needed = true;
#endif

    if (seed_needed) {
        ESP_LOGI(TAG, "Seeding the mount calibration from menuconfig");
        err = write_calibration(handle, cal);
    } else {
        load_i32(handle, KEY_CAL_AZ_REF, &cal->az_ref_cdeg);
        load_i32(handle, KEY_CAL_PAN_MIN, &cal->pan_min_cdeg);
        load_i32(handle, KEY_CAL_PAN_MAX, &cal->pan_max_cdeg);
        load_i32(handle, KEY_CAL_TILT_MIN, &cal->tilt_min_cdeg);
        load_i32(handle, KEY_CAL_TILT_MAX, &cal->tilt_max_cdeg);
        load_i32(handle, KEY_CAL_PAN_US_AT_0, &cal->pan_us_at_0);
        load_i32(handle, KEY_CAL_PAN_US_AT_180, &cal->pan_us_at_180);
        load_i32(handle, KEY_CAL_TILT_US_AT_0, &cal->tilt_us_at_0);
        load_i32(handle, KEY_CAL_TILT_US_AT_180, &cal->tilt_us_at_180);
        load_i32(handle, KEY_CAL_PAN_ENC_ZERO, &cal->pan_enc_zero);
        load_i32(handle, KEY_CAL_PAN_ENC_DIR, &cal->pan_enc_dir);
        load_i32(handle, KEY_CAL_TILT_ENC_ZERO, &cal->tilt_enc_zero);
        load_i32(handle, KEY_CAL_TILT_ENC_DIR, &cal->tilt_enc_dir);
    }

    nvs_close(handle);

    return err;
}

esp_err_t config_store_init(void)
{
    nvs_handle_t handle;
    pluto_config_t cfg = {0};
    uint8_t provisioned = 0;
    uint8_t home_channel = 0;
    bool seed_needed;
    esp_err_t err;

    if (s_initialized) {
        return ESP_OK;
    }

    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        return ESP_ERR_NO_MEM;
    }

    ESP_RETURN_ON_ERROR(nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle),
                        TAG,
                        "Unable to open the NVS namespace");

    /* Start from the compiled defaults so every field has a sane value even if
     * NVS only holds a subset of the keys. */
    apply_kconfig_seed(&cfg);

    err = nvs_get_u8(handle, KEY_PROVISIONED, &provisioned);
    seed_needed = (err == ESP_ERR_NVS_NOT_FOUND);

#if CONFIG_PLUTO_FORCE_KCONFIG_SEED
    if (!seed_needed) {
        ESP_LOGW(TAG, "PLUTO_FORCE_KCONFIG_SEED is on: overwriting stored configuration");
    }
    seed_needed = true;
#endif

    if (seed_needed) {
        if (cfg.provisioned) {
            ESP_LOGI(TAG, "Seeding NVS from menuconfig with SSID \"%s\"", cfg.wifi_ssid);
        } else {
            ESP_LOGI(TAG, "No stored configuration and no seed SSID; starting unprovisioned");
        }

        err = write_all(handle, &cfg);
        if (err != ESP_OK) {
            nvs_close(handle);
            ESP_LOGE(TAG, "Unable to seed NVS: %s", esp_err_to_name(err));
            return err;
        }
    } else {
        cfg.provisioned = provisioned != 0;

        load_string(handle, KEY_WIFI_SSID, cfg.wifi_ssid, sizeof(cfg.wifi_ssid));
        load_string(handle, KEY_WIFI_PASS, cfg.wifi_password, sizeof(cfg.wifi_password));
        load_string(handle, KEY_MQTT_URI, cfg.mqtt_uri, sizeof(cfg.mqtt_uri));
        load_string(handle, KEY_MQTT_USER, cfg.mqtt_username, sizeof(cfg.mqtt_username));
        load_string(handle, KEY_MQTT_PASS, cfg.mqtt_password, sizeof(cfg.mqtt_password));
        load_string(handle, KEY_NTP_SERVER, cfg.ntp_server, sizeof(cfg.ntp_server));
        load_string(handle, KEY_NTP_TZ, cfg.ntp_timezone, sizeof(cfg.ntp_timezone));
        load_string(handle, KEY_ADMIN_PASS, cfg.admin_password, sizeof(cfg.admin_password));

        if (nvs_get_u8(handle, KEY_HOME_CHANNEL, &home_channel) == ESP_OK) {
            cfg.home_channel = home_channel;
        }
    }

    nvs_close(handle);

    memcpy(&s_cache, &cfg, sizeof(s_cache));

    /* Not fatal: the network configuration above is what keeps the dashboard
     * reachable to fix things, and a reboot loop over the mount would lose it. */
    err = load_calibration(&s_calibration);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Mount calibration not loaded from NVS (%s); using the menuconfig seed",
                 esp_err_to_name(err));
    }

    s_initialized = true;

    ESP_LOGI(TAG,
             "Configuration loaded: provisioned=%d ssid=\"%s\" broker=%s",
             s_cache.provisioned,
             s_cache.wifi_ssid,
             s_cache.mqtt_uri);
    ESP_LOGI(TAG, "Dashboard login: %s / %s", CONFIG_PLUTO_ADMIN_USER, s_cache.admin_password);

    return ESP_OK;
}

esp_err_t config_store_get(pluto_config_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    memcpy(out, &s_cache, sizeof(*out));
    xSemaphoreGive(s_mutex);

    return ESP_OK;
}

const pluto_config_t *config_store_peek(void)
{
    return &s_cache;
}

esp_err_t config_store_validate(const pluto_config_t *in, char *err, size_t err_len)
{
    if (in == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (in->wifi_ssid[0] == '\0') {
        snprintf(err, err_len, "SSID must not be empty");
        return ESP_ERR_INVALID_ARG;
    }

    /* WPA2 accepts an 8-63 character passphrase. An empty one means an open
     * network, which is unusual but legal. */
    if (in->wifi_password[0] != '\0' && strlen(in->wifi_password) < 8) {
        snprintf(err, err_len, "WiFi password must be empty or at least 8 characters");
        return ESP_ERR_INVALID_ARG;
    }

    if (strncmp(in->mqtt_uri, "mqtt://", 7) != 0 &&
        strncmp(in->mqtt_uri, "mqtts://", 8) != 0 &&
        strncmp(in->mqtt_uri, "ws://", 5) != 0 &&
        strncmp(in->mqtt_uri, "wss://", 6) != 0) {
        snprintf(err, err_len, "Broker URI must start with mqtt://, mqtts://, ws:// or wss://");
        return ESP_ERR_INVALID_ARG;
    }

    if (in->ntp_server[0] == '\0') {
        snprintf(err, err_len, "NTP server must not be empty");
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

esp_err_t config_store_set(const pluto_config_t *in)
{
    nvs_handle_t handle;
    esp_err_t err;

    if (in == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle), TAG, "nvs_open");

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    err = write_all(handle, in);
    if (err == ESP_OK) {
        memcpy(&s_cache, in, sizeof(s_cache));
    }
    xSemaphoreGive(s_mutex);

    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Configuration saved: ssid=\"%s\" broker=%s", in->wifi_ssid, in->mqtt_uri);
    } else {
        ESP_LOGE(TAG, "Unable to save configuration: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t config_store_set_admin_password(const char *password)
{
    pluto_config_t cfg;

    if (password == NULL || password[0] == '\0') {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_RETURN_ON_ERROR(config_store_get(&cfg), TAG, "config_store_get");
    strlcpy(cfg.admin_password, password, sizeof(cfg.admin_password));

    return config_store_set(&cfg);
}

esp_err_t config_store_set_home_channel(uint8_t channel)
{
    nvs_handle_t handle;
    esp_err_t err;

    if (!s_initialized || channel == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    /* The WiFi driver already writes to NVS on every association; skipping the
     * no-op case keeps this from adding a second write per connect. */
    if (s_cache.home_channel == channel) {
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle), TAG, "nvs_open");

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    err = nvs_set_u8(handle, KEY_HOME_CHANNEL, channel);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    if (err == ESP_OK) {
        s_cache.home_channel = channel;
    }
    xSemaphoreGive(s_mutex);

    nvs_close(handle);

    return err;
}

bool config_store_is_provisioned(void)
{
    return s_initialized && s_cache.provisioned;
}

esp_err_t config_store_get_calibration(pluto_calibration_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_calibration;
    xSemaphoreGive(s_mutex);

    return ESP_OK;
}

esp_err_t config_store_set_calibration(const pluto_calibration_t *in)
{
    nvs_handle_t handle;
    esp_err_t err;

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(nvs_open(CALIBRATION_NAMESPACE, NVS_READWRITE, &handle), TAG, "nvs_open");

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    err = write_calibration(handle, in);
    if (err == ESP_OK) {
        s_calibration = *in;
    }
    xSemaphoreGive(s_mutex);

    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Calibration saved: az_ref=%ld pan=%ld..%ld tilt=%ld..%ld cdeg, "
                      "pulses pan=%ld/%ld tilt=%ld/%ld us, encoders pan=%ld/%+ld tilt=%ld/%+ld",
                 (long)in->az_ref_cdeg,
                 (long)in->pan_min_cdeg,
                 (long)in->pan_max_cdeg,
                 (long)in->tilt_min_cdeg,
                 (long)in->tilt_max_cdeg,
                 (long)in->pan_us_at_0,
                 (long)in->pan_us_at_180,
                 (long)in->tilt_us_at_0,
                 (long)in->tilt_us_at_180,
                 (long)in->pan_enc_zero,
                 (long)in->pan_enc_dir,
                 (long)in->tilt_enc_zero,
                 (long)in->tilt_enc_dir);
    } else {
        ESP_LOGE(TAG, "Unable to save the calibration: %s", esp_err_to_name(err));
    }

    return err;
}

esp_err_t config_store_factory_reset(void)
{
    nvs_handle_t handle;
    esp_err_t err;

    ESP_RETURN_ON_ERROR(nvs_open(CONFIG_STORE_NAMESPACE, NVS_READWRITE, &handle), TAG, "nvs_open");

    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Unable to erase the configuration namespace: %s", esp_err_to_name(err));
        return err;
    }

    /* The WiFi driver keeps its own copy of the credentials in nvs.net80211.
     * Without this the device reports itself unprovisioned and still rejoins
     * the old network. */
    err = esp_wifi_restore();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "esp_wifi_restore failed: %s", esp_err_to_name(err));
    }

    ESP_LOGW(TAG, "Factory reset complete; reboot required");

    return ESP_OK;
}
