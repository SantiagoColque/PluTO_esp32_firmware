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
7. `start_rotor()`: the stored calibration (an invalid one keeps the servos stopped),
   the servo driver, the encoders if built in, the rotor state and the rotor task.
8. `mqtt_manager_init()`, with `coordinates_message_handler` as the data callback.

Once MQTT is up, the device publishes `online` on `device/<device_id>/status`, its
`device_id` on `device/subscription`, and subscribes to
`device/<device_id>/coordinates/polar`. From there the pointing pipeline runs in `main.c`:

- `coordinates_message_handler` hands each payload to `rotor_handle_payload()` under
  `s_rotor_lock` (decode, validate, load the trajectory) and then publishes the state. It
  never touches the servos. Its own tick can set the very first pose as is (there is no
  position to ramp from), and `rotor_task` sends it on its next tick.
- `rotor_task` runs every 50 ms with a monotonic `dt_ms` from `esp_timer`. With
  `CONFIG_PLUTO_ENCODERS` it first reads both encoders, outside the lock, and hands the
  angles to `rotor_set_measured()`. `rotor_tick()` follows the trajectory through the
  motion profile, and outside the lock the commanded pose goes to `servo_driver_set()`,
  or `servo_driver_release()` once `rotor_outputs_resting()` says the axes have rested
  (HU-35). **This is where the servos are driven.**
- The same task publishes the state on `device/<device_id>/state`: every second while
  tracking, every ten at rest, and at once when the mode changes.

## Components

Each is a standard IDF component under `components/`, with its public API in
`include/` and nothing else exported.

- **config_store** — the whole runtime configuration in one NVS-backed struct
  (`pluto_config_t`): WiFi, MQTT, NTP, dashboard password, home channel, provisioned
  flag. Holds a RAM cache. `config_store_peek()` lends a pointer to that cache rather
  than copying, because some IDF APIs store the pointer they are handed —
  `esp_sntp_setservername()` does — and a field of a stack-allocated struct would dangle.
  It also keeps the mount's calibration (`pluto_calibration_t`: `az_ref`, the pan and
  tilt limits, each servo's pulse at 0 and at 180 degrees and each encoder's zero and
  direction) in a namespace of its own, `pluto_cal`, so that
  `config_store_factory_reset()`, which erases `pluto_cfg` to re-provision the WiFi,
  leaves it alone. It stores plain integers and checks nothing: `start_rotor()` checks
  the mount with `pan_tilt_config_valid()` and the driver checks the pulses; if either is
  invalid the servos are not started. `POST /api/calibration` makes the same checks
  before saving.
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
- **mqtt_manager** — `esp-mqtt` client: the `status` publish with its last will, the
  registration publish, the control subscription with a `mqtt_data_cb_t` callback
  handed in by `main`, and `mqtt_manager_publish_state()`.

The pointing pipeline is split the same way: pure components with host tests
(`test_host/`), and `main` holding the task, the lock and the MQTT side.

- **coord_dto** — decodes and validates the binary setpoint batch of
  `coordinates-dto.md`, with the reject codes of `device-state.md` §5.
- **trajectory** — what the board does between messages: a new batch replaces the
  pending points from its first instant on (the earlier ones stay, as in the server's
  `merge_trajectory`), the target is interpolated in time with the azimuth taking the
  short way round, it stays on the last point, and a `HOLD` drops everything pending.
- **pan_tilt** — geographic az/el to the angles of two 180-degree servos, flipping past
  the zenith for the back half of the sky (`coordinates-dto.md` §9). Kinematics only:
  trims and pulses belong to `servo`.
- **motion_profile** — per axis, in servo space: speed and acceleration limits (HU-36), a
  braking ramp after a `HOLD`, and when an axis has rested (HU-35). See its header.
- **rotor** — the board's side of both contracts: takes payloads, applies the rules that
  need the clock and the mount, follows the trajectory through `motion_profile`, clamps
  the commanded pose to the mount and writes the JSON state, which reports that pose
  and, when the encoders give one, the measured position and its error against the
  target (`meas` and `err_cdeg`, `device-state.md` §4). See its header.
- **servo** — the two DS3218 on LEDC: 50 Hz, a 16-bit duty, one channel per servo.
  `servo_pulse` is the pure half: servo angle to pulse width with a two-point
  calibration per servo, which covers trim, scale, direction and the 270-degree version,
  and a hard 500–2500 µs clamp. `servo_driver` is the LEDC half. Pins and calibration
  come in through `servo_driver_init()` (the driver never reads Kconfig; `main` hands it
  the Kconfig pins and the stored pulses), and no pulse goes out until the first pose.
  `servo_driver_release()` stops the pulses on both channels for HU-35 (duty 0, so no
  pulse is cut short), and the next `servo_driver_set()` resumes them. Not yet checked on
  hardware.
- **encoder** — one AS5600 per axis, each alone on one of the two I2C controllers because
  its address is fixed. `encoder_angle` is the pure half: raw count to servo angle with
  a zero and a direction per axis. `as5600` is the I2C half, on the new `i2c_master`
  driver only (the legacy `driver/i2c.h` aborts at boot next to it): raw angle and
  magnet status, with each problem logged once as it appears and clears. Behind
  `CONFIG_PLUTO_ENCODERS`, off by default; the chip is not confirmed and nothing is
  checked on hardware.

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
| `/api/factory-reset` | POST | erase the network and dashboard settings (`pluto_cfg`); the mount calibration stays |
| `/logs` | GET | the log page (gzipped `logs.html`), linked from the Status card |
| `/api/logs` | GET | the newest `CONFIG_PLUTO_LOG_RING_LINES` lines from `log_ring`, oldest first |
| `/api/calibration` | GET | the stored mount calibration, keyed by the fields of `pluto_calibration_t` |
| `/api/calibration` | POST | replace it: all thirteen fields, whole numbers, checked like at boot; applies at the next restart |

The dashboard's default password is derived from the MAC: `pluto-xxxxxx`, logged at boot.

## MQTT contract

| topic | direction | payload |
|---|---|---|
| `device/<device_id>/coordinates/polar` | server → device | binary setpoint batch, QoS 1, never retained |
| `device/<device_id>/status` | device → server | `online`, retained; `offline` is the last will |
| `device/<device_id>/state` | device → server | JSON state document, QoS 0 |
| `device/subscription` | device → server | the bare `device_id`, published on every connect |

`device_id` is the station MAC as twelve lowercase hex digits.

The contracts live in the PluTO repo, under `docs/contracts/`: `coordinates-dto.md` for
the batch (a packed little-endian list of az/el setpoints, each carrying the instant it
must be reached, at `19 + 8*count` bytes) and `device-state.md` for `status` and
`state`. Read them before touching either side.

On the server, `mqtt_dispatch` bridges `plugin/<uuid>/coordinates/polar` to the device
topic, and the ingest registers a board from its first `status` or `state`. It rejects
`device/subscription`, which the firmware still publishes: the topic is a compile-time
constant (`CONFIG_MQTT_TOPIC_SUBSCRIPTION`), kept until both sides agree to drop it.

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

The mount calibration (`az_ref`, the joint limits, each servo's pulses and each encoder's
zero and direction) follows the same seed rule in its own namespace, `pluto_cal`, which
the dashboard's factory reset keeps. Once stored, the dashboard edits it
(`/api/calibration`) and it applies at the next restart. The servo pins and version, the
motion profile and `CONFIG_PLUTO_ENCODERS` are compile-time; the encoders' I2C pins are
fixed in `main.c`. Every option is in `README.md`, *Configuration*.

## Build and flash

    idf.py menuconfig          # optional; seeds a blank NVS and sets the servo pins
    idf.py build
    idf.py -p /dev/ttyUSB0 flash monitor

The ESP-IDF version (v5.5.x) and the deployment steps are in `README.md`; keep it in
step when flashing, provisioning, recovery or a menuconfig option changes.

`sdkconfig` is not committed (`.gitignore` excludes it): each checkout generates its own
from `sdkconfig.defaults`, which holds the choices that carry a reason:

- a **custom partition table** (`partitions.csv`), because the stock single-app layout
  caps the app at 1 MB and the captive-portal build does not fit comfortably. It is sized
  for 2 MB flash so it is valid on any ESP32 module, and `nvs` keeps the stock offset and
  size so stored configuration survives a change of table.
- `CONFIG_LWIP_MAX_SOCKETS=16`, because `esp_http_server` reserves
  `max_open_sockets + 3` descriptors and with the default 7 it would claim all ten,
  leaving none for MQTT or the captive DNS server — which then fail at runtime without
  an obvious error.
- `CONFIG_COMPILER_OPTIMIZATION_SIZE`, since `-Og` costs roughly 15% of the image.

`.vscode/` is not committed either: point the ESP-IDF extension at your own
`~/esp/esp-idf` and serial port locally.

The pure components have host tests, built with the host compiler under ASan and UBSan:

    make -C components/rotor/test_host
    bash -c 'for d in components/*/test_host; do make -C "$d" || exit 1; done'

`components/coord_dto/test_host/coord_dto_vectors.txt` holds payloads made by the
server's encoder, which the decoder's host test must read back. After a change to PluTO's
`server_core/coord_dto.py`, regenerate it with
`python3 tools/gen_coord_dto_vectors.py <path to PluTO>` and commit the result.

## Conventions

- Code, comments and commit messages are in **English**; the requirements documents are
  in Spanish. Keep that split.
- Comments explain *why* a thing is the way it is — the constraint, the failure it
  avoids, the API quirk behind it. The existing ones are a good model; match that bar
  rather than restating the code.
- Conventional commits with a scope: `feat(wifi):`, `chore:`, `fix:`. Branches are
  `feature/<issue>-<slug>`; PRs merge into `develop`.
- One concern per component, public API in `include/` only, and no component reaching
  back into another's internals. A pure component (no ESP-IDF, host-tested) may build on
  other pure components through their public API, the way `rotor` composes `coord_dto`,
  `trajectory`, `pan_tilt` and `motion_profile`. Whatever touches hardware, the network,
  NVS or a task (`servo`, `config_store`, `mqtt_manager`, the lock) is wired only by
  `main`.
- Static state is file-scoped and `s_`-prefixed. Errors propagate as `esp_err_t`;
  `ESP_ERROR_CHECK` is for genuinely unrecoverable setup, not for anything a retry could fix.

## What is not implemented yet

Roughly in the order the SRS prioritizes them:

- **Homing / zero calibration** (HU-28). With absolute encoders it is capturing an
  offset, not searching for one; today each encoder's zero is typed into the dashboard.
  Missing: capturing it on the board, the MQTT command that triggers it (its topic is to
  be agreed in the server's contracts) and a dashboard jog to move a servo by hand.
- **Bench tuning of the motion profile**: the speed, acceleration and rest timeout are
  compile-time and shared by both axes, and the rest state cuts both outputs at once.
  Whether the DS3218 holds the tilt under the antenna without pulses, and so which
  outputs may be cut, is still to be measured.
- **Telemetry of actual position on hardware** (HU-24, HU-29): `meas` and `err_cdeg` are
  implemented behind `CONFIG_PLUTO_ENCODERS`, but the chip is unconfirmed, the encoders
  are not mounted and their zero is typed in by hand (homing above).
- **Elevation threshold** (HU-30): not for the firmware. The server cuts the trajectory
  below `Rotor.min_elevation_deg`; the board only enforces the mount limits.
- **OTA updates** (HU-32) — the partition table has a single `factory` app slot, so
  adding OTA means changing it.
