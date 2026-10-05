# PluTO ESP32 Firmware

## What this is

The embedded half of PluTO, an automated satellite-tracking system built as an
undergraduate final project (Proyecto Integrador, FCEFyN–UNC). An ESP32 drives a
two-axis mount carrying a directional antenna; the server works out where the antenna
should point and pushes coordinates over MQTT.

Companion repo: **PluTO** (`git@github.com:joaquinpary/PluTO.git`), the Django server,
plugin orchestrator and coordinate engine. The requirements shared by both live in
`srs.md` in the parent workspace — user stories HU-01…HU-41 with their RF / RNF / RA
items. Commits and comments reference them as `HU-nn` and `RNF-nn.n`; when a constant
has a strange-looking bound, the reason is usually a specific RNF (see
`WIFI_RECONNECT_MAX_BACKOFF_SEC` and RNF-06.1).

**Toolchain: ESP-IDF, not Arduino and not PlatformIO.** Target is plain `esp32`.

## What it does today

`main/main.c` is short on purpose and reads as the whole boot story:

1. init NVS → `config_store_init()` → `wifi_manager_init()`
2. derive the device id from the station MAC (twelve lowercase hex digits)
3. start the web server — one server for both roles, before any interface has an address
4. **not provisioned** → raise the SoftAP portal and return. Configuration arrives over
   HTTP and the device reboots into the station path.
5. **provisioned** → `wifi_manager_start_sta()`, then block on
   `wifi_manager_wait_connected(portMAX_DELAY)`. Only this task blocks; the WiFi
   manager, the HTTP server and the event loop each run on their own task, so the
   dashboard stays reachable and reconnection keeps going for as long as it takes.
6. SNTP. A missing clock is a warning, not a reboot — the poller keeps trying.
7. `mqtt_manager_init()`.

Once MQTT is up, the device publishes its `device_id` to `device/subscription` and
subscribes to `device/<device_id>/coordinates/polar`. **Incoming coordinates are currently
only logged**: `coordinates_message_handler` in `main.c` is where motor control will
hook in, and it is the single biggest piece still missing.

## Components

Each is a standard IDF component under `components/`, with its public API in
`include/` and nothing else exported.

- **config_store** — the whole runtime configuration in one NVS-backed struct
  (`pluto_config_t`): WiFi, MQTT, NTP, dashboard password, home channel, provisioned
  flag. Holds a RAM cache. `config_store_peek()` lends a pointer to that cache rather
  than copying, because some IDF APIs store the pointer they are handed —
  `esp_sntp_setservername()` does — and a field of a stack-allocated struct would dangle.
- **wifi_manager** — the state machine: `IDLE → CONNECTING → CONNECTED`, plus
  `PROVISIONING` (AP only, never configured) and `AP_FALLBACK` (AP up while the station
  keeps retrying). Reconnect backoff doubles from 1 s to `WIFI_RECONNECT_MAX_BACKOFF_SEC`
  and stays there, retrying indefinitely. After `WIFI_AP_FALLBACK_SEC` without an IP a
  provisioned device raises the portal anyway, so it can be reconfigured without
  reflashing. Also owns the asynchronous AP scan — a scan takes seconds and pulls the
  radio off channel, so it must never run inside an HTTP handler.
- **web_server** — one `esp_http_server` bound to every interface, serving the captive
  portal at 192.168.4.1 while the AP is up and the dashboard on the LAN address once the
  station is connected, with no restart in between. `index.html` lives uncompressed in
  `www/` and is gzipped into the build directory by `gzip_asset.py`, so the source stays
  reviewable while the image carries about a third of the bytes.
- **dns_hijack** — answers every DNS A query on the AP interface with the portal's own
  address, which is what makes phones pop the captive-portal sheet. Started and stopped
  from `main.c` on `WIFI_EVENT_AP_START`/`AP_STOP`, not from `wifi_manager`, because the
  responder can only bind once the AP has an address — and wiring it the other way would
  make `wifi_manager` depend on `web_server` circularly.
- **sntp_manager** — starts the poller and waits for a valid clock. Returns
  `ESP_ERR_TIMEOUT` if it is still unset, which callers are meant to log and carry on from.
- **log_ring** — keeps the newest log lines of this firmware's own tags (20 by default)
  for the `/logs` page, evicting the oldest. It wraps the `vprintf` behind `ESP_LOGx`,
  so the serial port is unaffected. Needs log version 1 and RTOS timestamps, and fails
  the build otherwise. A component with a new tag must be added to the list in
  `log_ring_core.c` to show on the page. The dashboard password line is redacted,
  because the page is reachable without login on the provisioning AP.
- **mqtt_manager** — `esp-mqtt` client, registration publish and control subscription,
  with a `mqtt_data_cb_t` callback handed in by `main`.

## HTTP API

Everything is JSON. Basic Auth (`CONFIG_PLUTO_ADMIN_USER` plus the stored password) is
enforced on every request **except** those arriving on the provisioning AP — during
first-time setup there is no password to type yet.

| route | method | purpose |
|---|---|---|
| `/` | GET | the dashboard (gzipped `index.html`) |
| `/api/status` | GET | WiFi state, IP, RSSI, retry count, MQTT and clock status |
| `/api/scan` | GET | asynchronous AP scan results |
| `/api/config` | POST | WiFi / MQTT / NTP settings |
| `/api/admin-pass` | POST | change the dashboard password |
| `/api/reboot` | POST | restart, after the response reaches the browser |
| `/api/factory-reset` | POST | wipe NVS |
| `/logs` | GET | the log page (gzipped `logs.html`), linked from the Status card |
| `/api/logs` | GET | the newest `CONFIG_PLUTO_LOG_RING_LINES` lines from `log_ring`, oldest first |

The dashboard's default password is derived from the MAC: `pluto-xxxxxx`, logged at boot.

## MQTT contract

| topic | direction | payload |
|---|---|---|
| `device/subscription` | device → server | the bare `device_id`, published on every connect |
| `device/<device_id>/coordinates/polar` | server → device | binary setpoint batch, `docs/contract/coordinates-dto.md` |

`device_id` is the station MAC as twelve lowercase hex digits. The registration topic is
a compile-time constant (`CONFIG_MQTT_TOPIC_SUBSCRIPTION`) rather than a dashboard field,
because it is part of the backend contract and not site configuration.

Nothing publishes on the control topic yet — the server's chain currently ends at
`plugin/<uuid>/coordinates/polar`, and the bridge between the two is unbuilt on both sides.

The wire format **is** settled, in `docs/contract/coordinates-dto.md`: a packed
little-endian batch of az/el setpoints, each carrying the instant it must be reached, at
`19 + 8*count` bytes. Read it before touching either side; neither the decoder nor the
encoder exists yet, so that document is the only thing keeping the two repos in agreement.

Note that two topics end in `/coordinates/polar` and carry different encodings — the
`plugin/...` one is JSON, the `device/...` one is the binary DTO. That is deliberate but
it does surprise people debugging with `mosquitto_sub`.

## Configuration model

Runtime configuration lives in NVS and is edited from the dashboard. The Kconfig values
under *"PluTO Firmware Configuration"* only **seed** NVS on the first boot of a blank
device — changing them in `menuconfig` afterwards does nothing, which is a normal source
of confusion. `CONFIG_PLUTO_FORCE_KCONFIG_SEED` makes menuconfig authoritative on every
boot for development; it must never be enabled in a build that ships.

Leaving `CONFIG_WIFI_SSID` empty is the intended production flow: the device starts
unprovisioned and raises the setup portal. Setting it is a development shortcut.

## Build and flash

    idf.py set-target esp32
    idf.py menuconfig          # optional; only seeds a blank NVS
    idf.py build
    idf.py -p /dev/ttyUSB0 flash monitor

`sdkconfig` is committed; `sdkconfig.defaults` holds the choices that carry a reason:

- a **custom partition table** (`partitions.csv`), because the stock single-app layout
  caps the app at 1 MB and the captive-portal build does not fit comfortably. It is sized
  for 2 MB flash so it is valid on any ESP32 module, and `nvs` keeps the stock offset and
  size so stored configuration survives a change of table.
- `CONFIG_LWIP_MAX_SOCKETS=16`, because `esp_http_server` reserves
  `max_open_sockets + 3` descriptors and with the default 7 it would claim all ten,
  leaving none for MQTT or the captive DNS server — which then fail at runtime without
  an obvious error.
- `CONFIG_COMPILER_OPTIMIZATION_SIZE`, since `-Og` costs roughly 15% of the image.

`.vscode/settings.json` points the ESP-IDF extension at `~/esp/esp-idf` and
`/dev/ttyUSB0`; adjust locally rather than committing a different path.

## Conventions

- Code, comments and commit messages are in **English**; the requirements documents are
  in Spanish. Keep that split.
- Comments explain *why* a thing is the way it is — the constraint, the failure it
  avoids, the API quirk behind it. The existing ones are a good model; match that bar
  rather than restating the code.
- Conventional commits with a scope: `feat(wifi):`, `chore:`, `fix:`. Branches are
  `feature/<issue>-<slug>`; PRs merge into `develop`.
- One concern per component, public API in `include/` only, and no component reaching
  back into another's internals. `main` is the only place allowed to wire components to
  each other.
- Static state is file-scoped and `s_`-prefixed. Errors propagate as `esp_err_t`;
  `ESP_ERROR_CHECK` is for genuinely unrecoverable setup, not for anything a retry could fix.

## What is not implemented yet

Roughly in the order the SRS prioritizes them:

- **Motor control** (HU-01 / RF-01.3) — no PWM, no servo driver, no axes. The received
  coordinates are logged and dropped.
- **Binary DTO** (HU-10) — the format is specified in `docs/contract/coordinates-dto.md`
  but neither the firmware decoder nor the server encoder is written.
- **Software mechanical limits** (HU-07) — must live in firmware per RNF-07.1.
- **Homing / zero calibration** (HU-28) and north offset (HU-09 / RF-09.1).
- **Speed and acceleration limiting** (HU-36), telemetry of actual position (HU-24, HU-29),
  elevation threshold (HU-30).
- **OTA updates** (HU-32) — the partition table has a single `factory` app slot, so
  adding OTA means changing it.
