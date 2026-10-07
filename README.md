# PluTO_esp32_firmware

Firmware for the ESP32 of PluTO, the PLUgin-based Tracking Orchestrator: it points a
two-axis pan-tilt mount, driven by two DS3218 servos, at the trajectories the
[PluTO server](https://github.com/joaquinpary/PluTO) sends over MQTT.

This README covers building, flashing, provisioning and recovering a board. Each
component documents its API in its `include/` headers. The MQTT contracts live in the
server repository, under `docs/contracts/`: `coordinates-dto.md` for the trajectories
and `device-state.md` for what the board reports.

Items marked **Untested on hardware** are implemented but have not run on a board yet.
[Hardware status](#hardware-status) has the full list.

## Requirements

- **ESP-IDF v5.5.x.** The firmware is developed and built with v5.5.2; other 5.5 patch
  releases should work but have not been tried. Other versions do not build:
  - ESP-IDF 6 no longer ships the `json` and `mqtt` components, which `web_server` and
    `mqtt_manager` require;
  - before 5.5 there is no `CONFIG_LOG_VERSION`, and `log_ring` stops the build with an
    `#error`.

  To install v5.5.2:

      git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
      ~/esp/esp-idf/install.sh esp32
      . ~/esp/esp-idf/export.sh     # in every new shell

- **An ESP32** (the original chip, IDF target `esp32`) with at least 2 MB of flash. The
  partition table is sized for 2 MB. The board used so far is an ESP32-D0WD-V3 devkit.
- **A USB serial port.** On Linux your user must be in the `dialout` group
  (`sudo usermod -aG dialout $USER`, then log in again).
- **For the host tests:** `make` and a C compiler with AddressSanitizer and
  UndefinedBehaviorSanitizer (GCC or Clang).
- **For a working system:** the PluTO server running, at least its Mosquitto broker. See
  [Registering with the server](#registering-with-the-server).

## Build

    git clone https://github.com/SantiagoColque/PluTO_esp32_firmware.git
    cd PluTO_esp32_firmware
    . ~/esp/esp-idf/export.sh
    idf.py build

You do not need `idf.py set-target`: `sdkconfig.defaults` already selects `esp32`. The
first build generates `sdkconfig` from `sdkconfig.defaults`. Git ignores `sdkconfig`, so
each checkout has its own and no credentials get committed. To change an option before
building, run `idf.py menuconfig` (see [Configuration](#configuration)).

The build writes the bootloader, the partition table and the application
(`build/pluto_firmware.bin`) under `build/`.

## Flash

### With ESP-IDF

    idf.py -p /dev/ttyUSB0 flash monitor

`Ctrl+]` leaves the monitor. This writes only the bootloader (at `0x1000`), the
partition table (`0x8000`) and the application (`0x10000`). The NVS partition at
`0x9000` is left alone, so a provisioned board keeps its network settings, its
dashboard password and its mount calibration across reflashes.

### Without ESP-IDF: the merged image

Someone with ESP-IDF builds a single image that holds everything:

    idf.py merge-bin

This writes `build/merged-binary.bin`, a raw image meant for offset `0x0`. The flash
settings (DIO, 40 MHz, 2 MB) are already in its header. On the machine that does the
flashing, only Python and esptool are needed:

    pip install esptool
    python -m esptool --chip esp32 -p /dev/ttyUSB0 -b 460800 write_flash 0x0 merged-binary.bin

The port is `COM3` or similar on Windows, and `/dev/cu.usbserial-*` on macOS. esptool v5
spells the command `write-flash` and accepts `write_flash` with a deprecation warning;
the esptool v4 bundled with ESP-IDF 5.5 only knows `write_flash`.

> **The merged image erases the stored configuration.** It covers `0x0` to the end of
> the application, and the gaps between images are blank bytes, so it overwrites the NVS
> partition too. After flashing it, the board boots like a new one: it brings up the
> setup portal, or joins the network set in menuconfig. That is right for a first flash.
> To update a provisioned board without losing its settings, use `idf.py flash`, or
> write the three images at their own offsets:
>
>     python -m esptool --chip esp32 -p /dev/ttyUSB0 -b 460800 write_flash \
>         0x1000 bootloader.bin 0x8000 partition-table.bin 0x10000 pluto_firmware.bin
>
> They come from `build/bootloader/`, `build/partition_table/` and `build/`.

**Untested on hardware:** neither command has been run against a board yet. The layout
of the merged image has only been checked with a build.

### Serial monitor

The console runs at 115200 baud: use `idf.py -p PORT monitor`, or, without ESP-IDF,
`python -m serial.tools.miniterm PORT 115200` (pyserial comes with esptool). At every
boot the board logs its device id, the name of its setup network and the dashboard
login:

    I (...) pluto_main: Using device_id: 24dcc3a1b2c3
    I (...) wifi_manager: WiFi driver ready; provisioning SSID would be "PluTO-a1b2c3"
    I (...) config_store: Dashboard login: admin / pluto-a1b2c3

## Provisioning from a browser

A blank board has no network to join, so it raises a setup portal. This is the intended
way to deploy one: leave `WIFI_SSID` empty in menuconfig, which is the default.

1. Flash the board and power it. The serial log shows
   `Provisioning access point "PluTO-a1b2c3" is up at 192.168.4.1`.
2. From a phone or laptop, join the open WiFi network `PluTO-xxxxxx`. The six hex digits
   are the end of the board's MAC, the same digits that end its device id. Up to four
   clients can join at once.
3. Most phones open the setup page on their own. If yours does not, browse to
   `http://192.168.4.1`. The portal asks for no password.
4. Fill in the form:
   - **WiFi network:** pick one from **Scan** or type its name; the password must be
     empty (open network) or at least 8 characters.
   - **MQTT broker:** the server's broker URI, for example `mqtt://192.168.1.20:1883`. It
     must start with `mqtt://`, `mqtts://`, `ws://` or `wss://`. Then the broker's
     username and password.
   - **Clock:** an NTP server and a POSIX time zone such as `UTC0` or `<-03>3`. The time
     zone only changes the time shown on the dashboard.

   Fields left blank keep what is stored, which is how you edit one setting later
   without retyping the passwords.
5. Press **Save and restart**. The board saves the settings and restarts 1.5 s later.
   It joins your network and the portal goes away, so reconnect the phone to your usual
   WiFi.
6. Find the board's address: the serial log prints `IP assigned: ...`, and your router
   lists it among its clients. Open `http://<that address>/`.
7. Log in as `admin` with the default password, which is `pluto-` followed by the same
   six hex digits as the setup network: `PluTO-a1b2c3` gives `pluto-a1b2c3`. Change it
   right away under **Administration > Dashboard access** (at least 4 characters):
   anyone who sees the network name, or the device id in the MQTT topics, can work out
   the default one.

On the local network the dashboard shows the board's status and edits the same settings
as the portal. Saving restarts the board. Provisioning from a phone, the login and the
network scan have been checked on a board.

For development, set `WIFI_SSID`, `WIFI_PASSWORD` and the broker in menuconfig before
the first flash: the board then joins directly, without the portal.

## Registering with the server

The server's `docker-compose.yml` runs Mosquitto on port 1883 with anonymous access
off. The board logs in with the server's `MQTT_USERNAME` and `MQTT_PASSWORD`, which are
`pluto` and `change-me` unless its environment changes them. The broker URI is
`mqtt://<server address>:1883`.

1. Once connected, the board publishes `online` (retained) on `device/<device_id>/status`,
   with `offline` as its last will, and subscribes to
   `device/<device_id>/coordinates/polar`. It also publishes its state on
   `device/<device_id>/state` every 10 s, and every second while tracking.
2. The server's `mqtt_ingest` creates a **Rotor** for that `device_id` the first time it
   reports on `status` or `state`. Nobody registers a board by hand, and the admin has no
   button to add one.
3. In the Django admin (`http://<server address>:8000/admin/`), the board shows up under
   **Rotors** as online. There you can give it a name and set its pointing limits:
   minimum elevation (10° by default) and an optional azimuth window.
4. Open the plugin instance that should drive it, under **Plugin instances**, choose the
   rotor in its **Rotor** section and save. From then on `mqtt_dispatch` forwards that
   plugin's trajectories to `device/<device_id>/coordinates/polar`. Only one running
   instance may drive a rotor (RF-31.1). The edit form refuses to give a running
   instance a rotor that another running instance drives, but **Launch selected
   plugins** does not check this. Stop the other instance first.

To watch the board from the server side:

    mosquitto_sub -h <server address> -p 1883 -u pluto -P <password> -t 'device/#' -v

The `coordinates/polar` payload is binary (`coordinates-dto.md`), so it shows up as
noise. The board also publishes its bare `device_id` on `device/subscription` at every
connect. The server does not subscribe to that topic and does not need it to register
the board; the topic stays until both sides agree to drop it.

The registration and the state reports were tested against a real board on 2026-09-19
(`device-state.md` §8 in the server repository).

## Recovery

### The board cannot reach its network

**Untested on hardware.** A provisioned board that has no IP address
`WIFI_AP_FALLBACK_SEC` (45 s) after booting raises the same setup network,
`PluTO-xxxxxx`. It does the same sooner if the network rejects the password three times
in a row. Join it and fix the settings at `http://192.168.4.1`, with no login, as in the
first setup. The board keeps retrying the saved network meanwhile. Once it gets an
address, it takes the setup network down 60 s later, or later still while someone is
connected to it.

The 45 s timer only runs from boot. A board that was connected and then lost its network
keeps retrying without raising the portal, unless the password is rejected three times.
To reach the portal in that case, power-cycle the board.

### Factory reset

**Untested on hardware.** **Administration > Device > Erase configuration** on the
dashboard (`POST /api/factory-reset`) erases the network settings and restarts the board
1.5 s later. It works from the setup network too, where it asks for no login.

| Erased | Kept |
|---|---|
| WiFi network and password, and the WiFi driver's own copy of them | The mount calibration: `az_ref`, the pan and tilt limits and the servo pulses (NVS namespace `pluto_cal`) |
| Broker URI, username and password | The firmware |
| NTP server and time zone | |
| Dashboard password, which goes back to the MAC default | |

Everything erased lives in the NVS namespace `pluto_cfg`. On the next boot it is seeded
again from menuconfig: with `WIFI_SSID` empty the board raises the portal, but a build
that sets `WIFI_SSID` joins that network again. The dashboard's hint says the button
"returns the device to factory state". It does not reset the calibration, on purpose:
moving to another network says nothing about where the antenna points.

### Forgotten dashboard password

The board logs the current login on the serial port at every boot
(`Dashboard login: admin / ...`). The line is left out of `/logs`. A factory reset also
brings back the default password.

### Erase everything

    idf.py -p /dev/ttyUSB0 erase-flash
    python -m esptool --chip esp32 -p /dev/ttyUSB0 erase_flash     # the same, without ESP-IDF

Then flash again. This erases the calibration too. The next boot seeds both namespaces
from menuconfig. Flashing the merged image has the same effect on NVS. The firmware also
erases NVS by itself if it finds the partition full or written in a newer format
(`init_nvs()` in `main/main.c`).

## Configuration

`idf.py menuconfig` has three PluTO menus:

- **PluTO Firmware Configuration**, at the top level, with its submenu **Mount
  calibration (first-boot seed)**;
- **Component config > PluTO servos**;
- **Component config > PluTO motion profile**.

Each option is `CONFIG_<name>` in `sdkconfig`. There are two kinds:

- **Seed:** written to NVS on the first boot of a blank board. From then on the stored
  value wins, and changing the option in menuconfig does nothing on that board, which
  is the usual source of confusion. The network settings (`pluto_cfg`) are edited from
  the dashboard. The calibration (`pluto_cal`) cannot be edited on a running board yet
  (card 4b). Until then, to change it on a board, set the new values, enable
  `PLUTO_FORCE_KCONFIG_SEED`, flash and boot once, then flash again without it. The
  forced seed also overwrites the network settings with the menuconfig ones, and the
  dashboard password with the MAC default.
- **Build:** compiled in and read at every boot. Changing one means rebuilding and
  flashing.

| Option | Default | Kind | What it does |
|---|---|---|---|
| **PluTO Firmware Configuration** | | | |
| `WIFI_SSID` | empty | Seed, dashboard | Network to join. Empty raises the setup portal, which is the deployment flow; setting it is a development shortcut. |
| `WIFI_PASSWORD` | empty | Seed, dashboard | Its password. Empty means an open network. |
| `MQTT_BROKER_URI` | `mqtt://192.168.1.3:1883` | Seed, dashboard | The server's broker. The default is a development LAN address: set your own here or in the portal. |
| `MQTT_BROKER_USERNAME` | empty | Seed, dashboard | The server's `MQTT_USERNAME`. |
| `MQTT_BROKER_PASSWORD` | empty | Seed, dashboard | The server's `MQTT_PASSWORD`. |
| `NTP_SERVER` | `pool.ntp.org` | Seed, dashboard | Clock source. Trajectories are rejected (`clock_not_synced`) until the clock syncs. |
| `NTP_TIMEZONE` | `UTC0` | Seed, dashboard | POSIX time zone for the time shown on the dashboard. |
| `MQTT_TOPIC_SUBSCRIPTION` | `device/subscription` | Build | Topic where the board announces its `device_id`. The server ignores it (see above). Part of the contract, not site configuration. |
| `PLUTO_EXPIRED_POINT_TOLERANCE_MS` | 500 (0–10000) | Build | A trajectory point late by more than this is dropped instead of executed (`coordinates-dto.md` §6.3). |
| `PLUTO_LOG_RING_LINES` | 20 (5–64) | Build | Lines kept for `/logs`, about 180 bytes of RAM each. |
| `PLUTO_FORCE_KCONFIG_SEED` | off | Build | Re-seeds both namespaces from menuconfig at every boot. For development only: never in a build that ships. |
| `WIFI_RECONNECT_MAX_BACKOFF_SEC` | 30 (5–55) | Build | Ceiling of the reconnect delay, which doubles from 1 s. RNF-06.1 asks for reconnection within 60 s, so above about 45 there is no margin. |
| `WIFI_AP_FALLBACK_SEC` | 45 (15–600) | Build | Seconds without an address after boot before a provisioned board raises the portal. |
| `PLUTO_AP_SSID_PREFIX` | `PluTO-` | Build | Name of the setup network, followed by the last six hex digits of the MAC. |
| `PLUTO_ADMIN_USER` | `admin` | Build | Dashboard username. The password lives in NVS. |
| **Mount calibration (first-boot seed)** | | | |
| `PLUTO_CAL_AZ_REF_CDEG` | 0 (0–35999) | Seed | Geographic azimuth the pan faces at 0, clockwise from north, in centidegrees: 9000 means east. |
| `PLUTO_CAL_PAN_MIN_CDEG`, `PLUTO_CAL_PAN_MAX_CDEG` | 0, 18000 | Seed | Usable pan range, minimum below maximum. A stored mount with a range outside 0–18000, or a minimum not below its maximum, is ignored at boot for the default one (north, full ranges), with a warning. |
| `PLUTO_CAL_TILT_MIN_CDEG`, `PLUTO_CAL_TILT_MAX_CDEG` | 0, 18000 | Seed | Usable tilt range: 0 is the horizon ahead, 18000 the horizon behind. |
| **Component config > PluTO servos** | | | |
| `PLUTO_SERVO_DS3218_VERSION` | 270 degrees | Build | The 180- or 270-degree DS3218. Sets the defaults of the pulses below and the nominal pulses a servo falls back to when its stored pair is invalid. 270 is the safe choice until the servos are identified. |
| `PLUTO_SERVO_PAN_GPIO` | 25 | Build | Pan signal pin. The driver refuses the strapping, flash, UART0 and input-only pins and GPIO 14. Keep 21/22 and 32/33 free for the encoders. |
| `PLUTO_SERVO_TILT_GPIO` | 26 | Build | Tilt signal pin, same rules, different from the pan's. |
| `PLUTO_SERVO_PAN_US_AT_0`, `PLUTO_SERVO_PAN_US_AT_180` | 834, 2166 (500, 2500 for the 180° version) | Seed | Pulse in µs that puts the pan servo at 0 and at 180 degrees: covers trim, scale and direction. Swap them to reverse the servo. 500–2500. |
| `PLUTO_SERVO_TILT_US_AT_0`, `PLUTO_SERVO_TILT_US_AT_180` | as the pan | Seed | The same for the tilt servo. |
| **Component config > PluTO motion profile** | | | |
| `PLUTO_MOTION_MAX_SPEED_DEG_S` | 60 (0–600) | Build | Top speed of each axis (HU-36). 0 removes the limit. Below about 20 the pan falls behind at the top of a high pass. |
| `PLUTO_MOTION_MAX_ACCEL_DEG_S2` | 120 (0–5000) | Build | Top acceleration and braking of each axis. 0 removes the limit. |
| `PLUTO_MOTION_REST_AFTER_MS` | 5000 (0–3600000) | Build | Time both axes must stand still before the servo outputs are released (HU-35). 0 keeps them driven. |

Two notes on the servo menu. The pins and the DS3218 version are compiled in, but the
pulses are seeds, stored with the calibration. And changing the version once `sdkconfig`
exists does not change the pulses already saved there. Set them again, or delete their
lines from `sdkconfig` and run `idf.py reconfigure`.

Fixed in the code, not in menuconfig:

- the 50 ms rotor tick;
- the state period: 1 s while tracking, 10 s otherwise;
- the 15 s MQTT keepalive;
- the 60 s before the setup network goes down after reconnecting;
- the three password rejections that raise the portal early;
- the four clients the setup network takes;
- the portal address, `192.168.4.1`.

The encoders have no options yet, because they are not implemented.

`sdkconfig.defaults` holds the choices that carry a reason, each explained in the file:
the custom partition table, the socket count and the size optimization.

## Host tests

The pure components are built and tested on the host with `-Werror`, AddressSanitizer
and UndefinedBehaviorSanitizer. No board or ESP-IDF is needed. To run every suite:

    bash -c 'for d in components/*/test_host; do make -C "$d" || exit 1; done'

To run one:

    make -C components/rotor/test_host

There are seven suites: `coord_dto`, `log_ring`, `motion_profile`, `pan_tilt`, `rotor`,
`servo` (only the pulse conversion) and `trajectory`. Each prints
`<name>: all tests passed`. `make CC=clang` changes the compiler. Overriding `CFLAGS`
replaces the sanitizer flags along with everything else. `make clean` removes the
binaries.

The components that touch hardware, the network or NVS have no host tests:
`servo_driver`, `wifi_manager`, `mqtt_manager`, `web_server`, `config_store` and `main`.

## /logs

`http://<board>/logs`, linked from the dashboard's **Status** card, shows the newest log
lines of this firmware, so a board can be debugged without a serial cable. It refreshes
every 2 s, and **Pause** stops it.

- It keeps the newest `PLUTO_LOG_RING_LINES` lines (20 by default) and drops the oldest.
  Only this firmware's own tags are kept: `pluto_main`, `mqtt_manager`, `wifi_manager`,
  `web_server`, `web_api`, `config_store`, `sntp_manager`, `dns_hijack` and `servo`.
  ESP-IDF's own lines, and everything else, still go to the serial port, which is not
  affected. A new component's tag must be added to `log_ring_core.c` to show up.
- `GET /api/logs` returns the same lines as JSON, oldest first:
  `{"capacity", "dropped", "lines": [{"lvl", "t", "tag", "msg"}]}`. `t` is milliseconds
  since boot, and `dropped` counts lines skipped while the buffer was busy.
- On the local network it asks for the dashboard login. On the setup network it asks for
  none, so the `Dashboard login` line is redacted.
- Each accepted batch logs one line per trajectory point, a debugging aid in `main.c`. A
  full batch of 16 points plus its summary lines takes nearly all of the default 20
  lines, so during a pass the page shows little more than the last batch. The serial
  monitor has everything.
- `log_ring` needs `CONFIG_LOG_VERSION=1` and `CONFIG_LOG_TIMESTAMP_SOURCE_RTOS`, which
  are the ESP-IDF 5.5 defaults. Changing either in menuconfig stops the build with an
  `#error`.

## Hardware status

Checked on an ESP32-D0WD-V3 devkit, built with ESP-IDF v5.5.2:

- provisioning from a phone, the dashboard login, the network scan and the input checks;
- settings kept across reflashes with `idf.py flash`;
- MQTT `status` and `state` against the server, including the board showing up as a
  Rotor (`device-state.md` §8);
- the `/logs` page.

**Untested on hardware:**

- the fallback to the setup network, and the factory reset (both from PR #11);
- reconnecting within 60 s after a real outage (RNF-06.1);
- flashing with esptool alone, with the merged image or the three images;
- everything that moves the mount: the servo output on LEDC, the calibration stored in
  NVS, and the motion profile with the release at rest.

Not implemented yet:

- editing the calibration on a running board (card 4b);
- the encoders and the zero calibration (HU-28);
- telemetry of the measured position (HU-24, HU-29);
- OTA updates (HU-32): the partition table has a single `factory` slot.

## Known limitations

- The setup network is open, and while it is up, the requests that arrive through it
  skip the dashboard login. That includes changing the settings, a factory reset and
  `/logs`. It stays up while a provisioned board is in fallback.
- The default dashboard password comes from the MAC, which the setup network's name and
  the MQTT topics reveal. Change it after provisioning.
- The dashboard uses HTTP Basic Auth over plain HTTP. Only `mqtt://` has been used
  against the server.
