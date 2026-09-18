#ifndef DNS_HIJACK_H
#define DNS_HIJACK_H

#include "esp_err.h"

typedef struct dns_hijack_ctx *dns_hijack_handle_t;

/**
 * @brief Answer every DNS A query on the given interface with that interface's
 *        own address, so any hostname a client looks up resolves to the portal.
 *
 * @param netif_key esp_netif key of the interface to serve, e.g. "WIFI_AP_DEF".
 *                  The socket binds to that interface's address, so queries
 *                  arriving on the station side are never answered.
 *
 * @return handle, or NULL on failure.
 */
dns_hijack_handle_t dns_hijack_start(const char *netif_key);

/**
 * @brief Stop the server and wait for its task to close the socket and exit.
 */
void dns_hijack_stop(dns_hijack_handle_t handle);

#endif
