# door_sensor — ESP32-C3 battery-powered reed-switch door sensor

Reed switch → MQTT (QoS 1, confirmed) → Home Assistant (via MQTT
discovery). Wakes on a door open/close, plus a periodic heartbeat, reports
state and battery, then goes back to deep sleep. WiFi/MQTT/device identity
are configured at runtime via a built-in web setup portal (as of v2.0.0),
software-triggered OTA/setup/factory-reset (no spare GPIO for a physical
button), and a last-full-charge date to track battery life — see
**Behavior** below.

## Files

- `door_sensor.ino` — the sketch.
- `config.h.example` — copy to `config.h` and fill in: OTA credentials,
  setup-portal AP password, hardware pins, battery calibration, timing, and
  debounce settings. Keep `config.h` out of git (already covered by
  `.gitignore`). WiFi, MQTT, and device identity are **not** in here as of
  v2.0.0 — see **Setup** below.

## Hardware

- ESP32-C3, battery-powered
- **Reed switch**: one leg → GPIO4, other leg → GND, plus an **external
  10kΩ pull-up** from GPIO4 to 3.3V. The pin runs as plain `INPUT` (not
  `INPUT_PULLUP`) while awake — see the wiring note at the top of the
  `.ino` for why doubling up the pull-up caused close events to be missed.
- **Battery voltage divider**: midpoint → GPIO1 (ADC)
- No status LED (unlike the other battery sensors in this fleet) — OTA
  progress is only visible via the "OTA Active" MQTT entity, not a
  physical indicator.

## Behavior

- Sleeps until the door opens or closes (wake on GPIO level change), or
  every `HEARTBEAT_INTERVAL_US` (default 12h) as a periodic battery/status
  report.
- Every read of the reed switch — at wake, and throughout the cycle —
  requires several consecutive agreeing samples (`DOOR_DEBOUNCE_SAMPLES` /
  `DOOR_DEBOUNCE_MAX_ATTEMPTS` in `config.h`) before it's trusted, to
  reject switch bounce and RF pickup from the WiFi radio's own TX bursts
  landing on this GPIO.
- The reed switch is polled throughout the WiFi-connect, MQTT-connect, and
  publish phases (not just once at wake), so a rapid open-then-close that
  happens while the device is busy on the network still gets caught,
  counted, and reported — not just whichever state it happened to be in
  at the single moment it was first read.
- Caches the last successful WiFi channel across sleeps (RTC memory) to
  skip most of the scan on the next wake, falling back to a full auto-scan
  if the cached channel fails.
- A hardware watchdog force-restarts the device if it's ever awake too
  long (a hang, a stuck library call), independent of everything else in
  the sketch — re-armed with a longer deadline while an OTA window is
  actually in progress.
- Records the date of the last time the battery read 100% in flash/NVS
  (not RTC memory, so it survives an actual battery depletion, not just
  deep sleep) — compare this to whenever the device eventually goes quiet
  to see how long a charge actually lasted. Only re-arms once the battery
  has actually dropped to `FULL_CHARGE_REARM_THRESHOLD_PCT` (default 97%,
  `config.h`) or below, so a reading hovering near the top of the curve
  bouncing back up to 100% doesn't record a "new" full charge every time.
- HA discovery configs carry an `expire_after` so a dead device eventually
  shows "unavailable" instead of a door state frozen forever at its last
  value.
- Battery voltage carries an HA-adjustable calibration offset (a plain
  volts-added correction, "Battery Calibration Offset" number entity), on
  top of the compiled-in `BATT_CAL` curve in `config.h` — same mechanism as
  `temp_humidity_sensor_v4`.

## Setup

As of v2.0.0, WiFi, MQTT, and device identity are configured at runtime via
a built-in WiFiManager web portal, not compiled into `config.h` — same
system as `temp_humidity_sensor_v4`, adapted here for a board with no spare
GPIO for a physical setup button:

- **Never configured yet** (fresh flash, or after a factory reset): the
  portal opens automatically on the very next boot. Connect to the "P@cho
  DS 1 XXXX" WiFi network it broadcasts (password in `config.h`'s
  `AP_PASSWORD`, or open if that's left too short), then browse to
  `192.168.4.1` if it doesn't open on its own. Fill in your WiFi network,
  MQTT broker, and device name/ID, optionally a static IP and/or a BSSID
  pin (see the on-page network scan list), then Save — the device restarts
  into normal operation.
- **Already configured, want to change something**: flip the retained
  **"Setup Mode"** MQTT switch in Home Assistant. The device reopens the
  portal on its next natural wake (whichever comes first: a door event, or
  the heartbeat — open/close the door once to force it immediately rather
  than waiting up to `HEARTBEAT_INTERVAL_US`).
- **Start over completely**: flip the retained **"Factory Reset"** MQTT
  switch. This wipes every saved setting (WiFi/MQTT/identity/static
  IP/BSSID/battery offset) and the radio's own persisted WiFi credentials,
  then restarts unconfigured — the portal opens on the next boot as if the
  device were never set up.
- The portal has no live "in progress" indicator in Home Assistant the way
  OTA does (see below) — WiFiManager takes the radio over into its own
  access point while it's open, so the device can't reach the home MQTT
  broker to report it. The portal's own landing page shows a **Device
  status** box (door state, battery, WiFi/MQTT status, boot/fail counts)
  read fresh each time you open it.
- There's also no physical cancel gesture (no button) — a portal opened by
  mistake just needs to time out (`PORTAL_TIMEOUT_SEC`, default 10 min).
- Static IP is chosen by filling in the IP address field, not a checkbox —
  a `WiFiManagerParameter` checkbox is fundamentally broken (it always
  emits a duplicate HTML `value` attribute, confirmed against the library's
  own source), so this project never uses one.

## Before building

1. **Arduino IDE board package**: "esp32 by Espressif Systems", core 3.x+
   (uses the pin-based LEDC-free GPIO APIs, no special LEDC needed here).
2. **Libraries** (Library Manager):
   - `WiFiManager` by tzapu
   - `espMqttClient` by bertmelis
   - `ArduinoOTA`, `Preferences` (bundled with the ESP32 core)
3. Select board **"ESP32C3 Dev Module"**.
4. Copy `config.h.example` to `config.h` and fill in your OTA password,
   setup-portal AP password, and (if this board's hardware differs)
   battery calibration. WiFi/MQTT/device identity are set later, through
   the setup portal — see **Setup** above.

## OTA updates

This device has no spare GPIO for a physical OTA button, so OTA is
triggered entirely from software: flip the retained **"OTA Update"** MQTT
switch in Home Assistant, and the device picks it up on its next wake
(door event or heartbeat), stays awake for up to `OTA_WINDOW_MS` (default
5 min) listening for a flash over `ArduinoOTA`, then resumes its normal
sleep cycle. The **"OTA Active"** binary sensor is the only way to see OTA
is in progress, since there's no status LED on this board.

## MQTT / Home Assistant

Base topic: `home/<device_id>/...` (device ID set through the setup
portal, default `door_XXXXXX`).

| Purpose | Topic | Payload |
|---|---|---|
| Door state | `home/<id>/door` | `OPEN` / `CLOSED` |
| Battery voltage | `home/<id>/battery` | volts |
| Battery voltage (raw) | `home/<id>/battery_voltage_raw` | volts, pre-calibration -- for comparing against a multimeter |
| Battery percent | `home/<id>/battery_percent` | 0–100 |
| Low-battery flag | `home/<id>/battery_low` | `ON` / `OFF` |
| Battery calibration offset | `home/<id>/battery_cal_offset`, `.../set` | volts, -1 to 1 (persistent number entity, see below) |
| Last full charge date | `home/<id>/last_full_charge` | `YYYY-MM-DD` |
| WiFi signal | `home/<id>/wifi_signal` | dBm |
| Availability (LWT) | `home/<id>/status` | `online` / `offline` |
| Boot count | `home/<id>/boot_count` | integer |
| Reset boot counter | `home/<id>/boot_count_reset/set`, `.../state` | `ON` / `OFF` (retained switch, see below) |
| Connect fail count | `home/<id>/connect_fail_count` | resets on a fully successful wake |
| Total fail count | `home/<id>/total_fail_count` | lifetime, never resets |
| Uptime (zeroes on reset/power loss, keeps counting through deep sleep) | `home/<id>/uptime` | seconds |
| OTA trigger | `home/<id>/ota/set`, `.../state` | `ON` / `OFF` (retained switch) |
| OTA in progress | `home/<id>/ota_active` | `ON` / `OFF` |
| Setup Mode | `home/<id>/setup_mode/set`, `.../state` | `ON` / `OFF` (retained switch -- reopens the setup portal, see above) |
| Factory Reset | `home/<id>/factory_reset/set`, `.../state` | `ON` / `OFF` (retained switch -- wipes settings, see above) |
| Debug Mode | `home/<id>/debug_mode/set`, `.../state` | `ON` / `OFF` (persistent switch -- keeps the device awake + opens a network serial monitor, see below) |

All published via retained HA discovery configs on
`homeassistant/<component>/<device_id>/.../config`, so entities show up in
Home Assistant automatically once MQTT discovery is enabled.

### Retained-switch controls, not plain buttons

**"OTA Update"**, **"Reset Boot Counter"**, **"Setup Mode"**, and
**"Factory Reset"** are all implemented as retained
MQTT switches rather than plain HA `button` entities. A button's press is
a one-shot,
non-retained message — since this device is asleep almost all the time, a
press could easily land while nobody's subscribed and just vanish.
Flipping the switch sets a retained flag; the device consumes it on its
next natural wake and reports back `OFF`, which reads as a momentary
action in the UI even though the wire protocol underneath is a switch.

### Battery calibration offset

A plain volts-added correction (not a ratio), applied on top of the
compiled-in `BATT_CAL` curve: work out the offset as (multimeter reading)
- (Battery Voltage (Raw) sensor) and enter that difference in the
**"Battery Calibration Offset"** number entity in Home Assistant — e.g.
0.15 means "the hardware reads 0.15V low, add 0.15V from now on." Same
persistent (not one-shot) pattern as the other config-style entities here:
applies on the device's next wake and is echoed back as the new state.

### Debug Mode — network serial monitor

Once this device is mounted on a door, USB Serial isn't reachable to watch
its debounce behavior live. Flip the retained **"Debug Mode"** switch in
Home Assistant and, on the device's next wake, it:

- Keeps itself fully awake (no deep sleep) instead of ending the cycle
  normally.
- Opens a raw TCP server on `NETWORK_DEBUG_PORT` (default `23`, the actual
  telnet port) that mirrors everything normally printed over USB Serial —
  connect with plain `telnet <device-ip>`, no client software needed.
- Polls the reed switch tightly and publishes/logs every transition
  immediately, so you can stand at the door, operate it, and watch
  `[door] mid-cycle transition detected...` / `[door] pin never settled
  during debounce -- using last sample.` in real time instead of guessing
  from the eventual HA state.

Unlike the momentary switches above, **Debug Mode is persistent** — it
reflects the device's actual current state at all times, not a
consume-and-reset trigger. It auto-expires after `DEBUG_SESSION_TIMEOUT_MS`
(default 30 min) regardless of whether you remember to flip the HA switch
back, since the device can't sleep while it's on and this is a
battery-powered board. Two things it deliberately does *not* do during a
session: reconnect MQTT if the connection drops (the telnet feed keeps
working either way, just without live HA updates until the next normal
wake), or start `ArduinoOTA` (use the existing "OTA Request" switch on a
separate wake if you need to push firmware).

## Config file

As of v2.0.0, `config.h` (gitignored) only holds the OTA password,
setup-portal AP password/timeout, hardware pins, battery calibration
curve, timing, debounce strength, and timezone — copy `config.h.example`
to `config.h` and fill in real values. WiFi, MQTT credentials, and device
identity are **not** here anymore; they're set at runtime through the web
setup portal and persisted in NVS (see **Setup** above).

## Version History

`FIRMWARE_VERSION` lives in the gitignored `config.h`, so git doesn't
preserve a literal per-commit record. Versions below v1.3.3 are
reconstructed from the commit history using the convention in this
fleet's `CLAUDE.md` (patch +1 for a small change, minor +1 / patch reset
for a bigger one) — treat them as indicative for that stretch. From
v1.3.3 onward this is tracked exactly, one entry per firmware-affecting
change.

**v2.0.0 is a breaking change for the physical unit.** WiFi/MQTT/device
identity moved out of `config.h` into the runtime setup portal (ported
from `temp_humidity_sensor_v4`, adapted for a board with no spare GPIO for
a physical setup button -- see **Setup** above), so after flashing this
version the device boots straight into the setup portal and needs to be
reprovisioned through the web page once; the previously-compiled-in
credentials are no longer read at all.

| Version | Date | Changes |
|---|---|---|
| v1.0.0 | 2026-09-05 | Initial versioned release: credentials/identity/pins extracted into `config.h`, HA discovery device tags aligned with the rest of the fleet. Software-triggered OTA (retained MQTT switch) already present. |
| v1.1.0 | 2026-09-05 | WiFi channel caching across sleeps, hardware awake-watchdog, `expire_after` on HA discovery configs, boot/connect-fail-count diagnostics. |
| v1.2.0 | 2026-09-06 | Daily open/close counters (reset at local midnight, needs NTP), remote boot-counter reset via a retained MQTT switch. |
| v1.2.1 | 2026-09-07 | Battery percentage curve synced to match `temp_humidity_sensor`'s. |
| v1.2.2 | 2026-09-07 | Battery curve rescaled (every breakpoint proportionally compressed) instead of flattening the top into an instant jump at 4.15V. |
| v1.2.3 | 2026-09-07 | "OTA Active" binary sensor — the only OTA-in-progress indicator on a board with no status LED. |
| v1.2.4 | 2026-09-07 | `DEVICE_HW_VERSION` added to device identity and HA discovery. |
| v1.3.0 | 2026-09-07 | Reed switch polled continuously through the WiFi/MQTT/publish cycle instead of read once at wake, so a rapid open-then-close mid-cycle is caught instead of lost. |
| v1.3.1 | 2026-09-09 | Debounce strengthened to require several consecutive agreeing samples, rejecting switch bounce / WiFi-TX RF pickup that a single confirm-read let through. |
| v1.3.2 | 2026-09-11 | Fixed close events not registering: stopped combining `INPUT_PULLUP` with the external pull-up, which made the reed switch fight two parallel pull-ups to read a clean LOW on close. |
| v1.3.3 | 2026-09-12 | Last-full-charge date diagnostic (flash/NVS-backed, survives an actual battery depletion). |
| v1.3.4 | 2026-09-13 | Count-mismatch diagnostic (`ok`/`fail_open`/`fail_close`) — flags when `open_count_today`/`close_count_today` drift more than 1 apart, which is physically impossible for a door and means a real transition was never counted. |
| v1.3.5 | 2026-09-13 | Attempted fix for occasional spurious "unavailable": replaced the fixed post-disconnect delay with a poll on `mqttClient.connected()`. Made it worse (see v1.3.6) — reverted. |
| v1.3.6 | 2026-09-13 | Reverted v1.3.5: `mqttClient.connected()` almost certainly flips false synchronously the instant `disconnect()` is called, before the DISCONNECT packet actually goes out on the library's background task, so polling it exited the wait instantly instead of giving real time. Back to a fixed delay (`MQTT_DISCONNECT_DELAY_MS`), bumped from the original 300ms to 400ms for a little extra margin. |
| v2.0.0 | 2026-09-18 | Ported `temp_humidity_sensor_v4`'s runtime self-provisioning system: a WiFiManager web setup portal replaces compiled-in WiFi/MQTT/device-identity credentials in `config.h`, with runtime-configurable static IP and BSSID pinning (field-based, not a checkbox -- see **Setup** above for why). Adapted for a board with no spare GPIO for a physical setup button: the portal opens automatically on a never-configured device, and on an already-configured one via a new retained "Setup Mode" MQTT switch; factory reset moved from a button-hold gesture to a new retained "Factory Reset" MQTT switch, acted on immediately rather than requiring a hold duration. Also added the "Battery Calibration Offset" HA number entity and "Battery Voltage (Raw)" diagnostic sensor, same additive-offset mechanism as `temp_humidity_sensor_v4`. Breaking change for the physical unit -- see the migration note above. |
| v2.1.0 | 2026-09-19 | Added "Debug Mode" (persistent HA switch): keeps the device fully awake and opens a raw TCP server (`NETWORK_DEBUG_PORT`, default 23) mirroring all existing `Serial` output, for watching reed-switch debounce behavior live once the device is mounted somewhere USB isn't reachable -- see `runDebugSession()`. Every existing `Serial.print`/`println`/`printf` call gets tee'd automatically via a `#define Serial` swap to a small `Print`-derived wrapper class, no per-call-site changes. Auto-expires after `DEBUG_SESSION_TIMEOUT_MS` (default 30 min) so a forgotten toggle can't drain the battery. Prompted by intermittent missed door-close events during fast operation near the edge of the reed switch's magnetic range -- this doesn't fix that on its own, but makes it directly observable. |
| v2.1.1 | 2026-09-19 | Fixed the actual cause of the missed-close reports above: Debug Mode logging confirmed rapid manual door operation was tracked perfectly while the device stayed awake, but real misses still happened during the normal sleep cycle -- pointing at the one remaining blind `delay(MQTT_DISCONNECT_DELAY_MS)` right before `armWakeup()`/`goToSleep()`, which never called `checkDoorPin()` during that 400ms window. A transition landing there went uncounted entirely and could arm the next wake-up level from stale state. Replaced it with the same delay-then-`checkDoorPin()` loop every other wait in this file already uses, keeping the identical total wall-clock delay (still not a poll on `mqttClient.connected()` -- see that comment's own v1.3.5 history). |
| v2.1.2 | 2026-09-19 | Added a `DOOR_LINGER_MS` (default 3s) window at the end of every cycle, still connected, before starting the MQTT/WiFi teardown -- an extra margin on top of v2.1.1's fix, added "just in case" after testing came back clean. A follow-up transition landing in this window now gets its own live publish (not just a counted-but-unreported one, like the disconnect-delay window still handles), at the cost of a few extra seconds awake occasionally. |
| v2.2.0 | 2026-09-19 | Added a "Reset Daily Counters" retained MQTT switch (same momentary-via-echo pattern as Reset Boot Counter) to manually zero `open_count_today`/`close_count_today` -- also clears any `count_mismatch` fault along with them, since a mismatch is only ever a function of those two numbers. |
| **v2.3.0** | 2026-09-19 | **Removed `open_count_today`/`close_count_today`/`count_mismatch` and the "Reset Daily Counters" switch entirely.** Diagnostics-only feature, not the core function (door state) -- decided during unrelated troubleshooting on the `DS_1_v3` light-sleep fork not to keep carrying this weight when door state is what actually matters. Removed: the two RTC_DATA_ATTR counters and `lastCounterDay`, `countResetRequested`, `updateDailyCounters()`, `countMismatchState()`, all their MQTT topics/HA discovery entries/subscriptions, and the tallying in `checkDoorPin()` and `runDebugSession()`. `config.h`'s NTP section comment updated to reflect its one remaining purpose (the last-full-charge date). |
| v2.3.1 | 2026-09-20 | Fixed the last-full-charge diagnostic re-triggering spuriously: a battery reading hovering right at the top of its curve (ADC noise, a charger's trickle-float ripple) could bounce 99%→100%→99%→100% and record a "new" full charge on every single upward bounce, even though the battery was never really any lower than 99%. Replaced the plain `wasAt100` rising-edge flag in `updateAndGetLastFullChargeDate()` with an "armed" flag that only re-arms once the battery actually reads at or below the new `FULL_CHARGE_REARM_THRESHOLD_PCT` (default 97%, `config.h`) -- a reading of 100% only counts as "just charged" if it came from at or below that. Applied fleet-wide to every project sharing this diagnostic (`DS_1_v3`, `TH_2_v4`, `TH_2_v4_L`, `temp_humidity_sensor`, `temp_humidity_sensor_zdravkovec`). |
| v2.3.2 | 2026-09-25 | Added an `Uptime` diagnostic sensor (seconds, `device_class: duration`): time since the last real reset or power loss -- a deep-sleep timer/GPIO wake counts as a continuation, while power-on, manual reset, brownout, watchdog, software restart, or a dead-and-replaced battery all zero it. Uses the RTC counter (`esp_clk_rtc_time()`) since `millis()` does not survive deep sleep. Also: any real (non-deep-sleep) reset now re-sends the HA discovery configs once, so a newly added entity like this one shows up after an OTA/USB flash without needing a power cycle. |
