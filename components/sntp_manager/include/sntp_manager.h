#ifndef SNTP_MANAGER_H
#define SNTP_MANAGER_H

#include <stdbool.h>

#include "esp_err.h"

/**
 * @brief Start the SNTP poller and wait for the clock to become valid.
 *
 * Returns ESP_ERR_TIMEOUT if the clock is still unset after the wait. That is
 * not fatal: the poller keeps running and the clock may settle later, so the
 * caller should log it and carry on rather than abort.
 */
esp_err_t sntp_manager_init(void);

bool sntp_manager_is_synced(void);

#endif
