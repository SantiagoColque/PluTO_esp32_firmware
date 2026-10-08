# PluTO_esp32_firmware

Firmware for the ESP32 of PluTO, the PLUgin-based Tracking Orchestrator: it points a
two-axis pan-tilt mount, driven by two DS3218 servos, at the trajectories the
[PluTO server](https://github.com/joaquinpary/PluTO) sends over MQTT.

This README covers building, flashing, provisioning and recovering a board. Each
component documents its API in its `include/` headers. The MQTT contracts live in the
server repository, under `docs/contracts/`: `coordinates-dto.md` for the trajectories
and `device-state.md` for what the board reports.

## Requirements

- **ESP-IDF v5.5.x**, developed with v5.5.2. ESP-IDF 6 drops the `json` and `mqtt`
  components, and versions before 5.5 lack the log settings `log_ring` needs.

      git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git ~/esp/esp-idf
      ~/esp/esp-idf/install.sh esp32
      . ~/esp/esp-idf/export.sh     # in every new shell

- **An ESP32** (the original chip, IDF target `esp32`) with at least 2 MB of flash. The
  partition table is sized for 2 MB. The board used so far is an ESP32-D0WD-V3 devkit.
- **A USB serial port.** On Linux your user must be in the `dialout` group
  (`sudo usermod -aG dialout $USER`, then log in again).
- **For a working system:** the PluTO server running, at least its Mosquitto broker. See
  [Registering with the server](#registering-with-the-server).

## Wiring

The pan servo's signal goes to GPIO25 and the tilt's to GPIO26 (see
[Configuration](#configuration)), with the servos' ground tied to the ESP32's. Put a
10 kΩ pull-down from each signal to GND: the pins float from reset until the firmware
starts the servos, which never happens in the setup portal and waits for WiFi otherwise,
and a floating input can make a powered servo twitch.

The encoders are optional and only read by a build with `PLUTO_ENCODERS` on: one AS5600
per axis, each on its own I2C bus because the chip's address is fixed. By default pan
goes to SDA GPIO21 / SCL GPIO22 and tilt to SDA GPIO32 / SCL GPIO33. Power them from
3V3, never 5 V, and add 2.2–4.7 kΩ pull-ups from SDA and SCL to 3V3.

## Build

    git clone https://github.com/SantiagoColque/PluTO_esp32_firmware.git
    cd PluTO_esp32_firmware
    . ~/esp/esp-idf/export.sh
    idf.py build

You do not need `idf.py set-target`: `sdkconfig.defaults` already selects `esp32`. The
first build generates `sdkconfig` from `sdkconfig.defaults`. Git ignores `sdkconfig`, so
each checkout has its own and no credentials get committed. To change an option before
building, run `idf.py menuconfig` (see [Configuration](#configuration)).

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

The port is `COM3` or similar on Windows, and `/dev/cu.usbserial-*` on macOS.

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

### Serial monitor

The console runs at 115200 baud: use `idf.py -p PORT monitor`, or, without ESP-IDF,
`python -m serial.tools.miniterm PORT 115200` (pyserial comes with esptool). At every
boot the board logs its device id, the name of its setup network and the dashboard
login:

    I (...) pluto_main: Using device_id: 24dcc3a1b2c3
    I (...) wifi_manager: WiFi driver ready; provisioning SSID would be "PluTO-a1b2c3"
    I (...) config_store: Dashboard login: admin / pluto-a1b2c3

The login line is the way back in after a forgotten dashboard password; on `/logs` it
shows as `Dashboard login: <redacted>`.

## Provisioning from a browser

A blank board has no network to join, so it raises a setup portal. This is the intended
way to deploy one: leave `WIFI_SSID` empty in menuconfig, which is the default.

1. Flash the board and power it. The serial log shows
   `Provisioning access point "PluTO-a1b2c3" is up at 192.168.4.1`.
2. From a phone or laptop, join the open WiFi network `PluTO-xxxxxx`. The six hex digits
   are the end of the board's MAC, the same digits that end its device id.
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
as the portal. Saving restarts the board. It also edits the mount calibration: see
[Calibrating the mount](#calibrating-the-mount).

For development, set `WIFI_SSID`, `WIFI_PASSWORD` and the broker in menuconfig before
the first flash: the board then joins directly, without the portal.

## Calibrating the mount

The dashboard's **Mount calibration** section edits the calibration stored on the board
(`GET` and `POST /api/calibration`, all fields at once):

- **az_ref**, the geographic azimuth the pan faces at 0°, and the usable range of each
  joint, in degrees;
- each servo's pulse at 0° and at 180°, in µs (500–2500); swap them to reverse a servo;
- each encoder's zero, the raw count (0–4095) it reads with its servo at 0°, and whether
  the count grows or shrinks toward 180°. They start at 0 and *grows*. To find the zero,
  put the servo at 0° and restart: the log prints `pan encoder reading, raw N` on the
  first good read.

The board refuses values out of range, so a stored calibration never keeps the servos
stopped. **Save calibration** stores them; they apply after a restart, which the board
leaves to you (**Restart to apply**) so that saving does not cut a pass short.

## Registering with the server

The broker URI is `mqtt://<server address>:1883`. The server's Mosquitto has anonymous
access off: the board logs in with the server's `MQTT_USERNAME` and `MQTT_PASSWORD`,
which are `pluto` and `change-me` unless its environment changes them.

1. Once connected, the board reports on `device/<device_id>/status` and
   `device/<device_id>/state`. The server creates a **Rotor** for that `device_id` the
   first time it hears from it: nobody registers a board by hand.
2. In the Django admin (`http://<server address>:8000/admin/`), the board shows up under
   **Rotors**. There you can give it a name and set its pointing limits.
3. Open the plugin instance that should drive it, under **Plugin instances**, choose the
   rotor in its **Rotor** section and save. From then on the server forwards that
   plugin's trajectories to the board.

To watch the board from the server side:

    mosquitto_sub -h <server address> -p 1883 -u pluto -P <password> -t 'device/#' -v

The `coordinates/polar` payload is binary (`coordinates-dto.md`), so it shows up as
noise. Each `state` document reports the commanded pose and, in a build with the encoders,
the measured position (`meas`) and its error against the target (`err_cdeg`); otherwise
both are `null` (`device-state.md`).

## Recovery

### The board cannot reach its network

A provisioned board that has no IP address `WIFI_AP_FALLBACK_SEC` (45 s) after booting
raises the same setup network, `PluTO-xxxxxx`. It does the same sooner if the network
rejects the password three times in a row. Join it and fix the settings at
`http://192.168.4.1`, with no login, as in the first setup. The board keeps retrying the
saved network meanwhile, and takes the setup network down 60 s after it gets an address.

The 45 s timer only runs from boot. A board that was connected and then lost its network
keeps retrying without raising the portal, unless the password is rejected three times.
To reach the portal in that case, power-cycle the board.

### Factory reset

**Administration > Device > Erase configuration** on the dashboard
(`POST /api/factory-reset`) erases the network settings and restarts the board 1.5 s
later. It works from the setup network too, where it asks for no login.

| Erased | Kept |
|---|---|
| WiFi network and password, and the WiFi driver's own copy of them | The mount calibration: `az_ref`, the pan and tilt limits, the servo pulses and the encoders' zero and direction (NVS namespace `pluto_cal`) |
| Broker URI, username and password | The firmware |
| NTP server and time zone | |
| Dashboard password, which goes back to the MAC default | |

Everything erased lives in the NVS namespace `pluto_cfg`. On the next boot it is seeded
again from menuconfig: with `WIFI_SSID` empty the board raises the portal, but a build
that sets `WIFI_SSID` joins that network again.

### Erase everything

    idf.py -p /dev/ttyUSB0 erase-flash
    python -m esptool --chip esp32 -p /dev/ttyUSB0 erase_flash     # the same, without ESP-IDF

Then flash again. This erases the calibration too; the next boot seeds it from
menuconfig.

## Configuration

`idf.py menuconfig` has four PluTO menus: **PluTO Firmware Configuration** (with its
submenu **Mount calibration (first-boot seed)**), and **Component config > PluTO
servos**, **> PluTO motion profile** and **> PluTO encoders**. Each option is
`CONFIG_<name>` in `sdkconfig`.

- **Seed** options are written to NVS on the first boot of a blank board; after that the
  stored value wins, and the dashboard edits it.
- **Build** options are compiled in: change, rebuild, flash.

| Option | Default | Kind | What it does |
|---|---|---|---|
| **PluTO Firmware Configuration** | | | |
| `WIFI_SSID` | empty | Seed | Network to join; empty raises the setup portal. |
| `WIFI_PASSWORD` | empty | Seed | Its password; empty for an open network. |
| `MQTT_BROKER_URI` | `mqtt://192.168.1.3:1883` | Seed | The server's broker. |
| `MQTT_BROKER_USERNAME` | empty | Seed | The server's `MQTT_USERNAME`. |
| `MQTT_BROKER_PASSWORD` | empty | Seed | The server's `MQTT_PASSWORD`. |
| `NTP_SERVER` | `pool.ntp.org` | Seed | Clock source; trajectories are rejected until it syncs. |
| `NTP_TIMEZONE` | `UTC0` | Seed | POSIX time zone for the dashboard. |
| `MQTT_TOPIC_SUBSCRIPTION` | `device/subscription` | Build | Where the board announces its `device_id`; the server ignores it. |
| `PLUTO_EXPIRED_POINT_TOLERANCE_MS` | 500 (0–10000) | Build | How late a trajectory point may be and still run. |
| `PLUTO_LOG_RING_LINES` | 20 (5–64) | Build | Lines kept for `/logs`. |
| `PLUTO_FORCE_KCONFIG_SEED` | off | Build | Re-seed both namespaces at every boot; never in a build that ships. |
| `WIFI_RECONNECT_MAX_BACKOFF_SEC` | 30 (5–55) | Build | Ceiling of the reconnect delay (RNF-06.1). |
| `WIFI_AP_FALLBACK_SEC` | 45 (15–600) | Build | Seconds without an address before the portal comes back. |
| `PLUTO_AP_SSID_PREFIX` | `PluTO-` | Build | Name of the setup network. |
| `PLUTO_ADMIN_USER` | `admin` | Build | Dashboard username. |
| **Mount calibration (first-boot seed)** | | | |
| `PLUTO_CAL_AZ_REF_CDEG` | 0 (0–35999) | Seed | Azimuth the pan faces at 0, in centidegrees from north. |
| `PLUTO_CAL_PAN_MIN_CDEG`, `PLUTO_CAL_PAN_MAX_CDEG` | 0, 18000 | Seed | Usable pan range; an invalid one keeps the servos stopped. |
| `PLUTO_CAL_TILT_MIN_CDEG`, `PLUTO_CAL_TILT_MAX_CDEG` | 0, 18000 | Seed | Usable tilt range, from the horizon ahead to the one behind. |
| **Component config > PluTO servos** | | | |
| `PLUTO_SERVO_DS3218_VERSION` | 270 degrees | Build | Sets the pulses' defaults; 270 is the safe choice until the servos are identified. |
| `PLUTO_SERVO_PAN_GPIO` | 25 | Build | Pan signal pin; keep 21/22 and 32/33 free for the encoders. |
| `PLUTO_SERVO_TILT_GPIO` | 26 | Build | Tilt signal pin. |
| `PLUTO_SERVO_PAN_US_AT_0`, `PLUTO_SERVO_PAN_US_AT_180` | 834, 2166 | Seed | Pan pulse (µs) at 0 and 180 degrees; swap them to reverse the servo. |
| `PLUTO_SERVO_TILT_US_AT_0`, `PLUTO_SERVO_TILT_US_AT_180` | 834, 2166 | Seed | The same for the tilt. |
| **Component config > PluTO motion profile** | | | |
| `PLUTO_MOTION_MAX_SPEED_DEG_S` | 60 (1–600) | Build | Top speed of each axis (HU-36). |
| `PLUTO_MOTION_MAX_ACCEL_DEG_S2` | 120 (1–5000) | Build | Top acceleration and braking of each axis. |
| `PLUTO_MOTION_REST_AFTER_MS` | 5000 (0–3600000) | Build | Stillness before the servo outputs are released (HU-35); 0 never. |
| **Component config > PluTO encoders** | | | |
| `PLUTO_ENCODERS` | off | Build | Read each axis from an AS5600 and report `meas` and `err_cdeg`. |
| `PLUTO_ENCODER_PAN_SDA_GPIO`, `PLUTO_ENCODER_PAN_SCL_GPIO` | 21, 22 | Build | Pan encoder's I2C pins (I2C0). |
| `PLUTO_ENCODER_TILT_SDA_GPIO`, `PLUTO_ENCODER_TILT_SCL_GPIO` | 32, 33 | Build | Tilt encoder's I2C pins (I2C1). |

## Host tests

    bash -c 'for d in components/*/test_host; do make -C "$d" || exit 1; done'

## /logs

`http://<board>/logs`, linked from the dashboard's **Status** card, shows the newest
lines of this firmware's own log, so a board can be debugged without a serial cable.

## Hardware status

**Untested on hardware:**

- the fallback to the setup network, and the factory reset;
- reconnecting within 60 s after a real outage (RNF-06.1);
- flashing with esptool alone, with the merged image or the three images;
- everything that moves the mount: the servo output on LEDC, the calibration stored in
  NVS, and the motion profile with the release at rest;
- the encoders and the measured position; the chip (AS5600) is not confirmed yet.

Not implemented yet: capturing the encoders' zero on the board (it is typed in), and OTA
updates.

## Known limitations

- The setup network is open, and while it is up, the requests that arrive through it
  skip the dashboard login. That includes changing the settings, a factory reset and
  `/logs`. It stays up while a provisioned board is in fallback.
- The default dashboard password comes from the MAC, which the setup network's name and
  the MQTT topics reveal. Change it after provisioning.
- The dashboard uses HTTP Basic Auth over plain HTTP. Only `mqtt://` has been used
  against the server.
