# door_sensor — ESP32-C3 battery-powered reed-switch door sensor

Reed switch → MQTT (QoS 1, confirmed) → Home Assistant (via MQTT
discovery). Wakes on a door open/close, plus a periodic heartbeat, reports
state and battery, then goes back to deep sleep. Software-triggered OTA
(no spare GPIO for a physical button), daily open/close counters, and a
last-full-charge date to track battery life — see **Behavior** below.

## Files

- `door_sensor.ino` — the sketch.
- `config.h.example` — copy to `config.h` and fill in: WiFi/MQTT/OTA
  credentials, static IP, device identity, hardware pins, timing, and
  debounce settings. Keep `config.h` out of git (already covered by
  `.gitignore`).

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
- Tracks open/close counts for the current local day, reset to 0 in
  firmware the first wake after local midnight — needs an occasional NTP
  sync (`syncLocalTimeIfDue()`), since this device has no RTC backup
  battery of its own.
- Records the date of the last time the battery read 100% in flash/NVS
  (not RTC memory, so it survives an actual battery depletion, not just
  deep sleep) — compare this to whenever the device eventually goes quiet
  to see how long a charge actually lasted.
- HA discovery configs carry an `expire_after` so a dead device eventually
  shows "unavailable" instead of a door state frozen forever at its last
  value.

## Before building

1. **Arduino IDE board package**: "esp32 by Espressif Systems", core 3.x+
   (uses the pin-based LEDC-free GPIO APIs, no special LEDC needed here).
2. **Libraries** (Library Manager):
   - `espMqttClient` by bertmelis
   - `ArduinoOTA`, `Preferences` (bundled with the ESP32 core)
3. Select board **"ESP32C3 Dev Module"**.
4. Copy `config.h.example` to `config.h` and fill in your WiFi/MQTT/OTA
   values, static IP, and device identity.

## OTA updates

This device has no spare GPIO for a physical OTA button (GPIO0 is used
for the battery ADC), so OTA is triggered entirely from software: flip
the retained **"OTA Update"** MQTT switch in Home Assistant, and the
device picks it up on its next wake (door event or heartbeat), stays
awake for up to `OTA_WINDOW_MS` (default 5 min) listening for a flash
over `ArduinoOTA`, then resumes its normal sleep cycle. The **"OTA
Active"** binary sensor is the only way to see OTA is in progress, since
there's no status LED on this board.

## MQTT / Home Assistant

Base topic: `home/<DEVICE_ID>/...`

| Purpose | Topic | Payload |
|---|---|---|
| Door state | `home/<id>/door` | `OPEN` / `CLOSED` |
| Battery voltage | `home/<id>/battery` | volts |
| Battery percent | `home/<id>/battery_percent` | 0–100 |
| Low-battery flag | `home/<id>/battery_low` | `ON` / `OFF` |
| Last full charge date | `home/<id>/last_full_charge` | `YYYY-MM-DD` |
| Open count today | `home/<id>/open_count_today` | integer, resets at local midnight |
| Close count today | `home/<id>/close_count_today` | integer, resets at local midnight |
| Count mismatch | `home/<id>/count_mismatch` | `ok` / `fail_open` / `fail_close` |
| WiFi signal | `home/<id>/wifi_signal` | dBm |
| Availability (LWT) | `home/<id>/status` | `online` / `offline` |
| Boot count | `home/<id>/boot_count` | integer |
| Reset boot counter | `home/<id>/boot_count_reset/set`, `.../state` | `ON` / `OFF` (retained switch, see below) |
| Connect fail count | `home/<id>/connect_fail_count` | resets on a fully successful wake |
| Total fail count | `home/<id>/total_fail_count` | lifetime, never resets |
| OTA trigger | `home/<id>/ota/set`, `.../state` | `ON` / `OFF` (retained switch) |
| OTA in progress | `home/<id>/ota_active` | `ON` / `OFF` |

All published via retained HA discovery configs on
`homeassistant/<component>/<DEVICE_ID>/.../config`, so entities show up in
Home Assistant automatically once MQTT discovery is enabled.

### Retained-switch controls, not plain buttons

Both **"OTA Update"** and **"Reset Boot Counter"** are implemented as
retained MQTT switches rather than plain HA `button` entities. A button's
press is a one-shot, non-retained message — since this device is asleep
almost all the time, a press could easily land while nobody's subscribed
and just vanish. Flipping the switch sets a retained flag; the device
consumes it on its next natural wake and reports back `OFF`, which reads
as a momentary action in the UI even though the wire protocol underneath
is a switch.

### Count-mismatch detection

A door can only open and close one at a time, so `open_count_today` and
`close_count_today` can only ever be equal or differ by exactly 1 — which
side is "ahead" depends on whatever state the door was already in when the
counters last reset, but the gap between them never exceeds 1 for a
physically real door. If it ever does, a real transition was never
counted somewhere (a missed wake, a debounce that gave up and fell back
to the wrong reading, etc.), and `count_mismatch` reports which direction:
`fail_close` if opens are outpacing closes (closes are the ones being
missed), `fail_open` if it's the other way around.

## Config file

Credentials, static IP, device identity, hardware pins, timing, debounce
strength, and timezone all live in `config.h` (gitignored) — copy
`config.h.example` to `config.h` and fill in real values.

## Version History

`FIRMWARE_VERSION` lives in the gitignored `config.h`, so git doesn't
preserve a literal per-commit record. Versions below v1.3.3 are
reconstructed from the commit history using the convention in this
fleet's `CLAUDE.md` (patch +1 for a small change, minor +1 / patch reset
for a bigger one) — treat them as indicative for that stretch. From
v1.3.3 onward this is tracked exactly, one entry per firmware-affecting
change.

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
| v1.3.5 | 2026-09-13 | Fixed occasional spurious "unavailable" in HA with nothing in the logbook (an availability-topic flip, not a value change): the clean MQTT disconnect before sleep now waits (`MQTT_DISCONNECT_TIMEOUT_MS`) for the disconnect to actually finish instead of a blind fixed delay, so the broker is less likely to see an abrupt drop and fire the Last Will. |
