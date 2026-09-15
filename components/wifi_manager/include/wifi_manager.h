#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#define WIFI_DEVICE_ID_LEN 13
#define WIFI_AP_SSID_LEN 33
#define WIFI_SCAN_MAX_RESULTS 20

typedef enum {
    WIFI_MGR_STATE_IDLE,           /* driver initialized, nothing started */
    WIFI_MGR_STATE_CONNECTING,     /* station trying to associate */
    WIFI_MGR_STATE_CONNECTED,      /* station has an IPv4 address */
    WIFI_MGR_STATE_PROVISIONING,   /* access point only, never provisioned */
    WIFI_MGR_STATE_AP_FALLBACK,    /* access point up, station still retrying */
} wifi_mgr_state_t;

typedef struct {
    wifi_mgr_state_t state;
    bool sta_connected;
    bool ap_active;
    char ssid[WIFI_AP_SSID_LEN];
    char ip[16];
    char ap_ssid[WIFI_AP_SSID_LEN];
    char ap_ip[16];
    int8_t rssi;
    uint8_t channel;
    uint32_t retry_count;
    uint32_t next_retry_sec;
    uint8_t ap_client_count;
    uint8_t last_disconnect_reason;
    bool auth_suspected_bad;
} wifi_mgr_status_t;

typedef struct {
    char ssid[WIFI_AP_SSID_LEN];
    int8_t rssi;
    uint8_t channel;
    bool secure;
} wifi_mgr_ap_t;

/**
 * @brief Bring up netif, the default event loop and the WiFi driver.
 *
 * Does not connect and does not block. Call exactly one of
 * wifi_manager_start_sta() or wifi_manager_start_provisioning() afterwards.
 */
esp_err_t wifi_manager_init(void);

/** Join the network stored in config_store, retrying indefinitely. */
esp_err_t wifi_manager_start_sta(void);

/** Raise the open provisioning access point. Station stays off. */
esp_err_t wifi_manager_start_provisioning(void);

/** Take the access point down and return to station-only mode. */
esp_err_t wifi_manager_stop_ap(void);

bool wifi_manager_is_connected(void);
esp_err_t wifi_manager_get_status(wifi_mgr_status_t *out);

/**
 * @brief Device id: the six bytes of the station MAC as lowercase hex.
 *
 * Reads the MAC from eFuse, so it works before the driver is started and
 * before any connection exists. The provisioning SSID needs it at that point.
 */
esp_err_t wifi_manager_get_device_id(char *buf, size_t buf_len);

esp_err_t wifi_manager_get_ap_ssid(char *buf, size_t buf_len);

/** Block the calling task until the station has an IP, or time out. */
esp_err_t wifi_manager_wait_connected(uint32_t timeout_ms);

/**
 * @brief Kick off an access point scan.
 *
 * Asynchronous on purpose: a scan takes seconds and takes the radio off
 * channel, so running it inside an HTTP handler would hold a server socket and
 * stall any client attached to the portal. Poll wifi_manager_scan_in_progress()
 * and then read wifi_manager_scan_get_results().
 */
esp_err_t wifi_manager_scan_start(void);
bool wifi_manager_scan_in_progress(void);
esp_err_t wifi_manager_scan_get_results(wifi_mgr_ap_t *out, size_t max, size_t *count, int64_t *age_ms);

#endif
