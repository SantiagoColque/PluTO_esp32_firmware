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

static const char *TAG = "config_store";

static pluto_config_t s_cache;
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
