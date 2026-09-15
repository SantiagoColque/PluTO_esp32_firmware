#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdbool.h>

#include "esp_err.h"

/**
 * @brief Start the configuration web server.
 *
 * Binds to every interface, so one instance serves the captive portal at
 * 192.168.4.1 while the access point is up and the dashboard at the LAN address
 * once the station is connected, with no restart in between. Safe to call
 * before any interface has an address.
 */
esp_err_t web_server_start(void);

esp_err_t web_server_stop(void);
bool web_server_is_running(void);

/** Start or stop answering every DNS query with the portal address. */
void web_server_set_captive(bool enabled);

#endif
