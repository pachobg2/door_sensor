/*
 * ESP32-C3 Zero Battery-Powered Door Sensor
 * Reed switch -> MQTT (QoS 1, confirmed) -> Home Assistant (via MQTT discovery)
 *
 * Wiring:
 *   Reed switch: one leg -> GPIO4, other leg -> GND (+ external 10k pull-up
 *     from GPIO4 to 3.3V -- needed for reliable wake on the closed->open
 *     transition, see notes in armWakeup()). Pin is run as plain INPUT (not
 *     INPUT_PULLUP) while awake -- this external pull-up is the only one on
 *     the line; doubling it with the internal pull-up made the reed
 *     switch's closed-to-GND path fight two parallel pull-ups to read a
 *     clean LOW, while OPEN had nothing opposing it, so real close events
 *     could read as "still open" on a switch whose closed contact isn't a
 *     perfect hard short.
 *   Battery voltage divider: midpoint -> GPIO1 (ADC)
 *
 * Behavior:
 *   - Sleeps until the door opens or closes (wake on level change)
 *   - Also wakes periodically as a heartbeat / battery report
 *   - Publishes state, battery voltage/percent/low-flag, WiFi signal, and
 *     boot/fail-count diagnostics at QoS 1, waiting for PUBACK confirmation
 *     with up to 3 retry attempts per message, then goes back to sleep
 *   - OTA updates, entering the web setup portal, and factory reset are all
 *     triggered from software (retained MQTT switches in HA), not a
 *     physical button -- this board has no spare GPIO for one (GPIO1 is
 *     already the battery ADC, GPIO4 the reed switch). Flipping a switch
 *     sets a retained MQTT flag; the device only checks for it on its next
 *     natural wake (door event or heartbeat), so there's no way to "wake it
 *     early" the way a boot button would -- see the "Setup Mode" and
 *     "Factory Reset" switches below.
 *   - WiFi, MQTT, and device identity are configured at runtime via a
 *     built-in WiFiManager web portal, not compiled into config.h -- same
 *     system as temp_humidity_sensor_v4, adapted here for a device with no
 *     setup button: the portal opens automatically on a never-configured
 *     device, and on an already-configured one via the "Setup Mode" MQTT
 *     switch. Static IP / BSSID pinning are optional fields on that same
 *     portal page (not compiled constants, and not a checkbox -- see
 *     runMaintenanceMode()'s own comment for why a WiFiManagerParameter
 *     checkbox can't actually work).
 *   - Battery voltage carries an HA-adjustable calibration offset (a plain
 *     volts-added correction, "Battery Calibration Offset" number entity),
 *     on top of the compiled-in BATT_CAL curve -- same mechanism as
 *     temp_humidity_sensor_v4.
 *   - Caches the last successful WiFi channel across sleeps (RTC memory) to
 *     skip most of the scan on the next wake, falling back to a full scan
 *     if the cached channel fails
 *   - A hardware watchdog force-restarts the device if it's ever awake too
 *     long (stuck library call, unexpected hang), independent of everything
 *     else in the sketch
 *   - HA discovery configs carry an expire_after so a dead device eventually
 *     shows "unavailable" instead of a door state frozen forever
 *   - Boot counter can be reset remotely via a retained MQTT switch (a
 *     plain "button" entity's press is a one-shot, non-retained message a
 *     sleeping device would simply miss)
 *   - The reed switch is polled throughout the WiFi/MQTT/publish cycle (not
 *     just once at wake), so a rapid open-then-close that happens while the
 *     device is busy on the network still gets caught, counted, and
 *     reported -- not just whichever state it happened to be in at the
 *     single moment it was originally read
 *   - Every read of the reed switch (at wake, and mid-cycle) requires
 *     several consecutive agreeing samples before it's trusted, to reject
 *     switch bounce and RF pickup from the WiFi radio's own TX bursts
 *     landing on this GPIO -- see readStableDoorOpen()
 *   - Records the date of the last time the battery read 100% (in flash/NVS,
 *     not RTC memory, so it survives an actual battery depletion, not just
 *     deep sleep) -- lets you tell how long a charge actually lasted by
 *     comparing this date to whenever the device later goes quiet
 *   - Gives the clean MQTT disconnect a fixed real-time delay
 *     (MQTT_DISCONNECT_DELAY_MS) before tearing down WiFi and sleeping, so
 *     the broker is less likely to see an abrupt drop and fire the Last
 *     Will -- deliberately not a poll on mqttClient.connected(), which
 *     isn't a reliable signal for whether the disconnect packet actually
 *     went out yet
 *   - "Debug Mode" (an HA switch, persistent not one-shot) keeps the
 *     device fully awake and opens a raw TCP server (NETWORK_DEBUG_PORT,
 *     default 23 -- plain `telnet <ip>` works) mirroring everything
 *     normally printed over USB Serial, for watching reed-switch debounce
 *     behavior live once this device is mounted somewhere USB isn't
 *     reachable -- see runDebugSession(). Auto-expires after
 *     DEBUG_SESSION_TIMEOUT_MS (default 30 min) so a forgotten toggle
 *     can't drain the battery indefinitely. Not the same thing as the
 *     DEBUG_MODE compile-time constant in config.h (a bench-testing
 *     convenience, unrelated to this).
 *
 * Libraries required (Library Manager):
 *   - espMqttClient (Bert Melis)
 *   - ArduinoOTA (bundled with the ESP32 core)
 */

#include <WiFi.h>
#include <WiFiManager.h>
#include <espMqttClient.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <time.h>
#include <sys/time.h>
#include <vector>

// ---------------- Config ----------------
// OTA password, hardware pins, battery calibration curve, and timing
// constants live in config.h (same folder — the Arduino IDE shows it as a
// second tab) so you can keep that one file per-device without touching the
// rest of this sketch. config.h is gitignored; copy config.h.example to
// config.h and fill in your real values. WiFi, MQTT, and device identity are
// NOT in config.h -- they're configured at runtime via the web setup portal
// (see runMaintenanceMode()) and persisted in NVS, same system as
// temp_humidity_sensor_v4.
#include "config.h"

// ---------------- Network debug session (Debug Mode) ----------------
// Lets "Debug Mode" (an HA switch, see settings.debugMode below) be
// watched live over the network instead of USB Serial, which isn't
// reachable once this device is mounted on the door. Every existing
// Serial.print/println/printf call in this file (they all funnel through
// Print::write() under the hood) gets mirrored to whatever TCP client is
// currently connected, with zero changes at each of the ~100 existing call
// sites -- the #define below routes the bare name "Serial" to a small tee
// object for the rest of this translation unit. Scoped to this .ino only;
// the ESP32 core's own internals in other .cpp files are unaffected.
//
// NOT the same thing as the DEBUG_MODE compile-time constant in config.h
// (a bench-testing convenience: skips deep sleep entirely, adds a boot
// delay for USB Serial Monitor to attach). This is the runtime,
// HA-toggleable, network-reachable one -- see runDebugSession().
HardwareSerial& RealSerial = Serial; // captured before the macro below shadows "Serial"

WiFiServer debugServer(NETWORK_DEBUG_PORT);
WiFiClient debugClient;

class TeeSerial : public Print {
public:
  void begin(unsigned long baud) { RealSerial.begin(baud); }
  void flush() { RealSerial.flush(); }
  size_t write(uint8_t c) override {
    RealSerial.write(c);
    if (debugClient && debugClient.connected()) debugClient.write(c);
    return 1;
  }
  size_t write(const uint8_t* buf, size_t len) override {
    RealSerial.write(buf, len);
    if (debugClient && debugClient.connected()) debugClient.write(buf, len);
    return len;
  }
};
TeeSerial dbgSerial;
#define Serial dbgSerial

// ---------------- Runtime settings (WiFi/MQTT/identity, via the setup portal) ----------------
// Same system as temp_humidity_sensor_v4: nothing here is compiled in --
// it's all read from NVS at boot (defaults to unconfigured) and only ever
// written by runMaintenanceMode() after a portal save, or wiped by a
// factory reset. Declared before buildTopics() below since that function
// reads settings.deviceId.

struct Settings {
  String wifiSsid;
  String wifiPassword;
  String mqttHost;
  uint16_t mqttPort = 1883;
  String mqttUser;
  String mqttPassword;
  String deviceId;   // used in MQTT topics/unique_ids -- keep stable once this device exists in HA
  String deviceName; // friendly name shown in Home Assistant
  // Added to the raw (BATT_DIVIDER_RATIO reading, before BATT_CAL) plus the
  // BATT_CAL curve correction, to get the final reported battery voltage --
  // e.g. 0.15 means "add 0.15V to whatever the hardware+curve reports".
  // Adjustable at runtime from HA via the "Battery Calibration Offset"
  // number entity; 0 = no correction.
  float battVoltageOffsetV = 0.0f;
  bool configured = false;

  // Static IP, same idea as this device's old compile-time equivalent but
  // runtime-configurable here. useStaticIp gates all of it -- when false,
  // the other four fields are ignored and the device just uses DHCP.
  bool useStaticIp = false;
  String staticIp;
  String gateway;
  String subnet = "255.255.255.0";
  String dns;
  // Optional: pins to one specific access point by MAC instead of whichever
  // AP answers the SSID, for mesh/repeater setups with more than one AP
  // sharing the same network name. Empty = no pinning.
  String bssid;

  // HA-toggleable "Debug Mode" -- see runDebugSession(). Persistent (not a
  // one-shot trigger like OTA/Setup Mode/Factory Reset): stays true across
  // reboots until explicitly toggled off or it auto-expires after
  // DEBUG_SESSION_TIMEOUT_MS, at which point it's saved back to false so a
  // forgotten switch doesn't re-enter the session on every future wake.
  bool debugMode = false;
};
Settings settings;
Preferences settingsPrefs;

String getShortChipId() {
  uint64_t mac = ESP.getEfuseMac();
  char buf[7];
  snprintf(buf, sizeof(buf), "%06X", (uint32_t)(mac & 0xFFFFFFULL));
  return String(buf);
}

void loadSettings() {
  String chipId = getShortChipId();
  settingsPrefs.begin("settings", true); // read-only
  settings.configured   = settingsPrefs.getBool("configured", false);
  settings.wifiSsid     = settingsPrefs.getString("wifiSsid", "");
  settings.wifiPassword = settingsPrefs.getString("wifiPass", "");
  settings.mqttHost     = settingsPrefs.getString("mqttHost", "");
  settings.mqttPort     = settingsPrefs.getUShort("mqttPort", 1883);
  settings.mqttUser     = settingsPrefs.getString("mqttUser", "");
  settings.mqttPassword = settingsPrefs.getString("mqttPass", "");
  settings.deviceId     = settingsPrefs.getString("deviceId", "door_" + chipId);
  settings.deviceName   = settingsPrefs.getString("deviceName", "Door Sensor " + chipId);
  settings.battVoltageOffsetV = settingsPrefs.getFloat("battOffsetV", 0.0f);
  settings.useStaticIp = settingsPrefs.getBool("useStaticIp", false);
  settings.staticIp    = settingsPrefs.getString("staticIp", "");
  settings.gateway     = settingsPrefs.getString("gateway", "");
  settings.subnet      = settingsPrefs.getString("subnet", "255.255.255.0");
  settings.dns         = settingsPrefs.getString("dns", "");
  settings.bssid       = settingsPrefs.getString("bssid", "");
  settings.debugMode   = settingsPrefs.getBool("debugMode", false);
  settingsPrefs.end();
}

void saveSettings() {
  settingsPrefs.begin("settings", false);
  settingsPrefs.putBool("configured", settings.configured);
  settingsPrefs.putString("wifiSsid", settings.wifiSsid);
  settingsPrefs.putString("wifiPass", settings.wifiPassword);
  settingsPrefs.putString("mqttHost", settings.mqttHost);
  settingsPrefs.putUShort("mqttPort", settings.mqttPort);
  settingsPrefs.putString("mqttUser", settings.mqttUser);
  settingsPrefs.putString("mqttPass", settings.mqttPassword);
  settingsPrefs.putString("deviceId", settings.deviceId);
  settingsPrefs.putString("deviceName", settings.deviceName);
  settingsPrefs.putFloat("battOffsetV", settings.battVoltageOffsetV);
  settingsPrefs.putBool("useStaticIp", settings.useStaticIp);
  settingsPrefs.putString("staticIp", settings.staticIp);
  settingsPrefs.putString("gateway", settings.gateway);
  settingsPrefs.putString("subnet", settings.subnet);
  settingsPrefs.putString("dns", settings.dns);
  settingsPrefs.putString("bssid", settings.bssid);
  settingsPrefs.putBool("debugMode", settings.debugMode);
  settingsPrefs.end();
}

// Escapes text for safe embedding in HTML -- used on the setup portal's
// nearby-network scan list, where the SSID is attacker-controlled data (any
// AP in range can broadcast an arbitrary string).
String htmlEscape(const String& in) {
  String out;
  out.reserve(in.length());
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '&':  out += "&amp;";  break;
      case '"':  out += "&quot;"; break;
      case '\'': out += "&#39;";  break;
      case '<':  out += "&lt;";   break;
      case '>':  out += "&gt;";   break;
      default:   out += c;
    }
  }
  return out;
}

// JS-string-escape pass, then HTML-attribute-escape pass -- for embedding
// attacker-controlled text (a nearby SSID) inside an inline onclick handler.
String jsAttrEscape(const String& in) {
  String jsEscaped;
  jsEscaped.reserve(in.length());
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '\\' || c == '\'') jsEscaped += '\\';
    jsEscaped += c;
  }
  return htmlEscape(jsEscaped);
}

bool parseBssid(const String& str, uint8_t out[6]) {
  if (str.length() != 17) return false;
  int b[6];
  int matched = sscanf(str.c_str(), "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]);
  if (matched != 6) return false;
  for (int i = 0; i < 6; i++) out[i] = (uint8_t)b[i];
  return true;
}

// MQTT topics and Home Assistant discovery topics -- built once at the top
// of setup() by buildTopics(), after settings are loaded (deviceId isn't
// known at compile time here, unlike DEVICE_MANUFACTURER/MODEL/etc.).
String TOPIC_STATE, TOPIC_BATTERY, TOPIC_BATTERY_PCT, TOPIC_BATTERY_V_RAW, TOPIC_RSSI,
       TOPIC_AVAILABILITY, TOPIC_OTA_CMD, TOPIC_OTA_STATE, TOPIC_BATTERY_LOW,
       TOPIC_BOOT_COUNT, TOPIC_FAIL_COUNT, TOPIC_TOTAL_FAIL_COUNT,
       TOPIC_BOOT_RESET_CMD, TOPIC_BOOT_RESET_STATE, TOPIC_OTA_ACTIVE,
       TOPIC_LAST_FULL_CHARGE, TOPIC_SETUP_MODE_CMD, TOPIC_SETUP_MODE_STATE,
       TOPIC_FACTORY_RESET_CMD, TOPIC_FACTORY_RESET_STATE,
       TOPIC_BATTERY_CAL_OFFSET, TOPIC_BATTERY_CAL_OFFSET_SET,
       TOPIC_DEBUG_MODE_CMD, TOPIC_DEBUG_MODE_STATE;
String DISCOVERY_DOOR, DISCOVERY_BATTERY, DISCOVERY_BATTERY_PCT, DISCOVERY_BATTERY_V_RAW,
       DISCOVERY_RSSI, DISCOVERY_OTA, DISCOVERY_BATTERY_LOW, DISCOVERY_BOOT_COUNT,
       DISCOVERY_FAIL_COUNT, DISCOVERY_TOTAL_FAIL_COUNT, DISCOVERY_BOOT_RESET,
       DISCOVERY_OTA_ACTIVE, DISCOVERY_LAST_FULL_CHARGE, DISCOVERY_SETUP_MODE,
       DISCOVERY_FACTORY_RESET, DISCOVERY_BATTERY_CAL_OFFSET, DISCOVERY_DEBUG_MODE;

void buildTopics() {
  String base = String("home/") + settings.deviceId;
  TOPIC_STATE       = base + "/door";
  TOPIC_BATTERY     = base + "/battery";
  TOPIC_BATTERY_PCT = base + "/battery_percent";
  TOPIC_BATTERY_V_RAW = base + "/battery_voltage_raw"; // pre-calibration, for comparing against a multimeter
  TOPIC_RSSI        = base + "/wifi_signal";
  TOPIC_AVAILABILITY= base + "/status";
  TOPIC_OTA_CMD     = base + "/ota/set";
  TOPIC_OTA_STATE   = base + "/ota/state";
  TOPIC_BATTERY_LOW = base + "/battery_low";
  TOPIC_BOOT_COUNT  = base + "/boot_count";
  TOPIC_FAIL_COUNT  = base + "/connect_fail_count";
  TOPIC_TOTAL_FAIL_COUNT = base + "/total_fail_count";
  TOPIC_BOOT_RESET_CMD    = base + "/boot_count_reset/set";
  TOPIC_BOOT_RESET_STATE  = base + "/boot_count_reset/state";
  TOPIC_OTA_ACTIVE = base + "/ota_active";
  TOPIC_LAST_FULL_CHARGE = base + "/last_full_charge";
  TOPIC_SETUP_MODE_CMD     = base + "/setup_mode/set";
  TOPIC_SETUP_MODE_STATE   = base + "/setup_mode/state";
  TOPIC_FACTORY_RESET_CMD   = base + "/factory_reset/set";
  TOPIC_FACTORY_RESET_STATE = base + "/factory_reset/state";
  TOPIC_BATTERY_CAL_OFFSET     = base + "/battery_cal_offset";
  TOPIC_BATTERY_CAL_OFFSET_SET = base + "/battery_cal_offset/set";
  TOPIC_DEBUG_MODE_CMD   = base + "/debug_mode/set";
  TOPIC_DEBUG_MODE_STATE = base + "/debug_mode/state";

  String sbase = String("homeassistant/sensor/") + settings.deviceId;
  DISCOVERY_DOOR    = String("homeassistant/binary_sensor/") + settings.deviceId + "/door/config";
  DISCOVERY_BATTERY = sbase + "/battery/config";
  DISCOVERY_BATTERY_PCT = sbase + "/battery_percent/config";
  DISCOVERY_BATTERY_V_RAW = sbase + "/battery_voltage_raw/config";
  DISCOVERY_RSSI    = sbase + "/wifi_signal/config";
  DISCOVERY_OTA     = String("homeassistant/switch/") + settings.deviceId + "/ota/config";
  DISCOVERY_BATTERY_LOW = String("homeassistant/binary_sensor/") + settings.deviceId + "/battery_low/config";
  DISCOVERY_BOOT_COUNT  = sbase + "/boot_count/config";
  DISCOVERY_FAIL_COUNT  = sbase + "/connect_fail_count/config";
  DISCOVERY_TOTAL_FAIL_COUNT = sbase + "/total_fail_count/config";
  DISCOVERY_BOOT_RESET = String("homeassistant/switch/") + settings.deviceId + "/boot_count_reset/config";
  DISCOVERY_OTA_ACTIVE = String("homeassistant/binary_sensor/") + settings.deviceId + "/ota_active/config";
  DISCOVERY_LAST_FULL_CHARGE = sbase + "/last_full_charge/config";
  DISCOVERY_SETUP_MODE    = String("homeassistant/switch/") + settings.deviceId + "/setup_mode/config";
  DISCOVERY_FACTORY_RESET = String("homeassistant/switch/") + settings.deviceId + "/factory_reset/config";
  DISCOVERY_BATTERY_CAL_OFFSET = String("homeassistant/number/") + settings.deviceId + "/battery_cal_offset/config";
  DISCOVERY_DEBUG_MODE = String("homeassistant/switch/") + settings.deviceId + "/debug_mode/config";
}

// ---------------- Persisted state (survives deep sleep) ----------------

RTC_DATA_ATTR bool discoverySent = false;
RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t connectFailCount = 0; // increments on any wake that fails to publish, resets on success
RTC_DATA_ATTR uint32_t totalFailCount = 0;   // lifetime total failed wakes -- never resets, mirrors bootCount
RTC_DATA_ATTR uint8_t cachedWifiChannel = 0; // 0 = unknown yet, let WiFi.begin() auto-select
RTC_DATA_ATTR bool g_timeSynced = false;     // true once any cycle has completed a real NTP sync

// ---------------- Globals ----------------

espMqttClient mqttClient; // uses its own background task on ESP32 -- no manual loop() needed
Preferences batteryPrefs; // flash/NVS, not RTC memory -- survives an actual battery depletion

// Set by the onConnect/onPublish callbacks, which fire from the client's
// background task. Polled from the main setup()/loop() flow below.
volatile bool mqttConnectedFlag = false;
volatile uint16_t lastAckedPacketId = 0;
volatile bool otaRequested = false;
volatile bool bootCountResetRequested = false;
volatile bool setupModeRequested = false;
volatile bool factoryResetRequested = false;

// Same persistent (not one-shot) pattern as temp_humidity_sensor_v4's LED
// brightness/battery-offset entities: a plain calibration offset in volts,
// applied on the device's next wake and echoed back as the new state.
volatile bool g_battCalCmdReceived = false;
char g_battCalCmdPayload[12] = {0};

// Same persistent pattern as battery calibration above -- a plain ON/OFF
// toggle rather than a one-shot trigger. Checked again live inside
// runDebugSession()'s own loop (the library's background task keeps
// delivering messages throughout the session, not just at connect), so a
// mid-session toggle-off is picked up without re-subscribing.
volatile bool g_debugModeCmdReceived = false;
char g_debugModeCmdPayload[8] = {0};

// Latest known door state for this wake cycle. Seeded from the initial read
// in setup(), then kept fresh by checkDoorPin() polling throughout the WiFi
// connect / MQTT connect / publish wait loops, so a rapid open-then-close
// (or vice versa) that happens while the device is busy on the network gets
// caught and reported instead of only the net state by the time it sleeps.
bool currentDoorOpen = false;

// ---------------- Awake watchdog ----------------
// A hardware timer that force-restarts the device if it's ever awake too
// long -- a hang, an unexpected infinite loop, a stuck library call. This
// is independent of everything else in the sketch: even if the main flow
// gets stuck somewhere no one anticipated, this guarantees the device
// eventually resets rather than draining the battery all night awake.

esp_timer_handle_t g_watchdogTimer = nullptr;

void watchdogTimeoutHandler(void* arg) {
  esp_restart();
}

void startAwakeWatchdog(uint32_t timeoutMs) {
  if (g_watchdogTimer != nullptr) {
    esp_timer_stop(g_watchdogTimer);
    esp_timer_delete(g_watchdogTimer);
    g_watchdogTimer = nullptr;
  }
  esp_timer_create_args_t args = {};
  args.callback = &watchdogTimeoutHandler;
  args.arg = nullptr;
  args.dispatch_method = ESP_TIMER_TASK;
  args.name = "awake_wdt";
  esp_timer_create(&args, &g_watchdogTimer);
  esp_timer_start_once(g_watchdogTimer, (uint64_t)timeoutMs * 1000ULL);
}

void stopAwakeWatchdog() {
  if (g_watchdogTimer != nullptr) {
    esp_timer_stop(g_watchdogTimer);
    esp_timer_delete(g_watchdogTimer);
    g_watchdogTimer = nullptr;
  }
}

// ---------------- Function declarations ----------------

void connectWiFi();
bool attemptWifiConnect(uint8_t channel);
void initMqttClient();
bool connectMQTT();
bool publishQos1(const String& topic, const String& payload, bool retain);
void sendDiscoveryConfig();
int publishState(bool doorOpen, float batteryVoltage, float batteryPercent, float rawBatteryVoltage, int rssi, uint32_t failCount);
void armWakeup(int currentPinLevel);
void goToSleep();
float batteryPercentage(float v);
float calibrateBatteryVoltage(float raw);
String wakeupCauseToString(esp_sleep_wakeup_cause_t cause);
void syncLocalTimeIfDue();
void checkDoorPin();
bool readStableDoorOpen();
String updateAndGetLastFullChargeDate(float batteryPercent);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason);
void onMqttPublish(uint16_t packetId);
void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic,
                    const uint8_t* payload, size_t len, size_t index, size_t total);
void runOtaWindow();
void runMaintenanceMode(bool viaMqttRequest);
void runDebugSession();

// ---------------- Setup / main flow ----------------

void setup() {
  Serial.begin(115200);
  startAwakeWatchdog(AWAKE_WATCHDOG_TIMEOUT_MS); // armed immediately -- extended later if OTA/portal mode is entered

  if (DEBUG_MODE) {
    // Give native USB CDC time to enumerate and give you time to open
    // Serial Monitor before the board does anything else.
    delay(DEBUG_BOOT_DELAY_MS);
    Serial.println("=== DEBUG_MODE is ON: deep sleep disabled, staying awake ===");
  } else {
    delay(100);
  }

  bootCount++;

  loadSettings();
  buildTopics(); // topics depend on settings.deviceId -- must run after loadSettings()

  // Plain INPUT, not INPUT_PULLUP: the board already has an external 10k
  // pull-up to 3.3V on this pin (see the wiring note above). Adding the
  // internal pull-up (~45k) on top of that means the reed switch's
  // closed-to-GND path has to fight TWO parallel pull-ups to read a clean
  // LOW, while OPEN has nothing opposing it at all (pure float pulled
  // high) -- that asymmetry is enough for a switch whose closed contact
  // isn't a perfect hard short (ordinary contact wear, a slightly marginal
  // magnet gap) to never read reliably LOW, so real close events get
  // misread as "still open." Relying on the external pull-up alone halves
  // what the switch has to overcome on close, with no downside for open.
  pinMode(REED_PIN, INPUT);
  delay(DEBOUNCE_SETTLE_MS); // let the pin electrically settle

  bool doorOpen = readStableDoorOpen(); // multi-sample debounce -- see its own comment for why
  currentDoorOpen = doorOpen; // kept fresh by checkDoorPin() as this cycle progresses
  bool lastReportedDoorOpen = doorOpen; // what MQTT was last actually told, for the final catch-up check

  // Never-configured device (no saved WiFi/MQTT yet) -- always go straight
  // to the setup portal, regardless of wake cause. The reed-pin read above
  // still has to happen first even on this path: armWakeup() below needs a
  // real door-state reading on every path, since goToSleep() itself arms no
  // wakeup source of its own.
  if (!settings.configured) {
    runMaintenanceMode(false);
    // Only reached if the portal timed out / failed to connect -- a
    // successful save restarts the device itself and never returns here.
    armWakeup(currentDoorOpen ? HIGH : LOW);
    stopAwakeWatchdog();
    goToSleep();
    return;
  }

  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  Serial.printf("Boot #%lu, wakeup cause: %s (%d), door: %s\n",
                bootCount, wakeupCauseToString(wakeupCause).c_str(), wakeupCause,
                doorOpen ? "OPEN" : "CLOSED");

  // analogReadMilliVolts() uses the ESP32's factory ADC calibration (eFuse)
  // for an accurate mV reading -- far more accurate than manually mapping
  // raw analogRead() counts against an assumed 3.3V reference, which the
  // ADC doesn't actually use internally.
  uint32_t rawMillivolts = analogReadMilliVolts(BATT_PIN);
  float rawVoltage = (rawMillivolts / 1000.0f) * BATT_DIVIDER_RATIO;
  // The HA-adjustable correction (e.g. entering 0.15 in "Battery Calibration
  // Offset" means "add 0.15V to whatever the hardware+curve report"),
  // applied last, on top of BATT_CAL. 0 by default.
  float batteryVoltage = calibrateBatteryVoltage(rawVoltage) + settings.battVoltageOffsetV;
  float batteryPercent = batteryPercentage(batteryVoltage);

  connectWiFi();

  int rssi = 0;

  if (WiFi.status() == WL_CONNECTED) {
    rssi = WiFi.RSSI();
    if (connectMQTT()) {
      if (factoryResetRequested) {
        Serial.println("Factory reset requested via MQTT switch -- wiping settings and restarting unconfigured.");
        publishQos1(TOPIC_FACTORY_RESET_CMD, "OFF", true);
        publishQos1(TOPIC_FACTORY_RESET_STATE, "OFF", true);
        mqttClient.disconnect();
        delay(MQTT_DISCONNECT_DELAY_MS);
        settingsPrefs.begin("settings", false);
        settingsPrefs.clear();
        settingsPrefs.end();
        WiFi.disconnect(true, true); // also erase the radio's own persisted WiFi credentials
        stopAwakeWatchdog();
        Serial.println("Restarting into unconfigured state...");
        Serial.flush();
        delay(200);
        ESP.restart();
      }

      if (!discoverySent) {
        sendDiscoveryConfig();
        discoverySent = true;
      }

      if (bootCountResetRequested) {
        Serial.println("Boot counter reset requested via MQTT switch.");
        bootCount = 0;
        // Clear the retained command immediately so we don't re-trigger on
        // every subsequent wake, and reflect the reset back to the HA UI.
        publishQos1(TOPIC_BOOT_RESET_CMD, "OFF", true);
        publishQos1(TOPIC_BOOT_RESET_STATE, "OFF", true);
      }

      int failedTopics = publishState(currentDoorOpen, batteryVoltage, batteryPercent, rawVoltage, rssi, connectFailCount);
      lastReportedDoorOpen = currentDoorOpen; // whatever publishState() just told MQTT
      if (failedTopics == 0) {
        connectFailCount = 0; // every topic confirmed by the broker, counter clears
      } else {
        Serial.printf("%d topic(s) never got a PUBACK this cycle.\n", failedTopics);
        connectFailCount++;
        totalFailCount++;
      }

      // Resync last (and only periodically) so a slow or failed NTP round
      // trip can only cost next cycle's date accuracy, never delay or risk
      // this cycle's actual door-state/battery publish.
      syncLocalTimeIfDue();

      // Battery calibration offset: a plain volts-added correction, same
      // pattern as temp_humidity_sensor_v4 -- apply if changed, then always
      // echo the current value back as state.
      if (g_battCalCmdReceived) {
        float requested = atof(g_battCalCmdPayload);
        if (requested < -1.0f) requested = -1.0f;
        if (requested > 1.0f) requested = 1.0f;
        if (requested != settings.battVoltageOffsetV) {
          settings.battVoltageOffsetV = requested;
          saveSettings();
          Serial.printf("Battery calibration offset set to %.3fV via HA.\n", settings.battVoltageOffsetV);
        }
      }
      publishQos1(TOPIC_BATTERY_CAL_OFFSET, String(settings.battVoltageOffsetV, 3), true);

      if (otaRequested) {
        Serial.println("OTA requested via MQTT switch -- entering OTA window.");
        // Clear the retained command immediately so we don't re-trigger on
        // every subsequent wake, and reflect the reset back to the HA UI.
        publishQos1(TOPIC_OTA_CMD, "OFF", true);
        publishQos1(TOPIC_OTA_STATE, "OFF", true);

        startAwakeWatchdog(OTA_WINDOW_MS + 30000); // OTA legitimately needs to stay awake this long
        publishQos1(TOPIC_OTA_ACTIVE, "ON", true); // visible in HA even though this device has no status LED
        ArduinoOTA.setHostname(settings.deviceId.c_str());
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.begin();
        Serial.printf("OTA ready, staying awake for up to %lu ms...\n", OTA_WINDOW_MS);
        runOtaWindow();
        publishQos1(TOPIC_OTA_ACTIVE, "OFF", true);
        Serial.println("OTA window elapsed, resuming normal cycle.");
      }

      if (setupModeRequested) {
        Serial.println("Setup mode requested via MQTT switch -- reopening the setup portal.");
        // Clear the retained command immediately, same as every other
        // switch here -- this device only ever gets one more chance to see
        // it (the next connected wake), so leaving it ON would just reopen
        // the portal again immediately after this one closes.
        publishQos1(TOPIC_SETUP_MODE_CMD, "OFF", true);
        publishQos1(TOPIC_SETUP_MODE_STATE, "OFF", true);
        // Clean disconnect before handing the radio to WiFiManager, same
        // sequence as the normal end-of-cycle teardown below.
        mqttClient.disconnect();
        delay(MQTT_DISCONNECT_DELAY_MS);
        WiFi.disconnect(true);
        runMaintenanceMode(true);
        // Only reached if the portal timed out -- falls through to the
        // normal end-of-cycle sleep below with whatever settings it had.
      }

      // Debug Mode: a plain ON/OFF toggle, same persistent pattern as the
      // battery calibration offset above -- apply if changed, then always
      // echo the actual current value back (not a one-shot trigger, so
      // no "clear the retained command" step like OTA/Setup Mode/Factory
      // Reset above).
      if (g_debugModeCmdReceived) {
        bool requested = (strcmp(g_debugModeCmdPayload, "ON") == 0);
        if (requested != settings.debugMode) {
          settings.debugMode = requested;
          saveSettings();
          Serial.printf("Debug Mode set to %s via HA.\n", settings.debugMode ? "ON" : "OFF");
        }
      }
      publishQos1(TOPIC_DEBUG_MODE_STATE, settings.debugMode ? "ON" : "OFF", true);
      if (settings.debugMode) {
        runDebugSession();
        // Always returns (auto-expiry or a live toggle-off) and falls
        // through to the normal end-of-cycle teardown below.
      }
    } else {
      Serial.println("MQTT connect failed after all attempts, skipping publish this cycle.");
      connectFailCount++;
      totalFailCount++;
    }
  } else {
    Serial.println("WiFi connect failed, skipping publish this cycle.");
    connectFailCount++;
    totalFailCount++;
  }

  if (DEBUG_MODE) {
    // Stay connected in debug mode so MQTT stays "online" and we can watch
    // live door toggles propagate to HA without triggering the Last Will.
    stopAwakeWatchdog(); // staying awake indefinitely is intentional in debug mode
    Serial.println("=== DEBUG_MODE: staying connected, entering loop() ===");
    return;
  }

  // Linger a few extra seconds here, still connected, before tearing
  // anything down -- not just one last check, but a whole DOOR_LINGER_MS
  // window of it. Closes the residual gap between the last poll during
  // publishing and now, catches any transition that checkDoorPin() saw
  // (and counted) during publishState()'s own publish calls but couldn't
  // publish itself (see checkDoorPin()'s comment on why), and -- being a
  // window rather than an instant -- also catches a follow-up flip that
  // happens shortly after the main publish, while MQTT is still up to
  // actually report it live rather than just silently updating the
  // counters. Cheap in battery terms: a few extra seconds occasionally,
  // not continuous wake. This call site is safe to publish from: nothing
  // else is waiting on a PUBACK right now, so publishing here can't race it.
  unsigned long lingerStart = millis();
  while (millis() - lingerStart < DOOR_LINGER_MS) {
    checkDoorPin();
    if (mqttClient.connected() && currentDoorOpen != lastReportedDoorOpen) {
      Serial.printf("[door] publishing follow-up state during linger: %s\n",
                    currentDoorOpen ? "OPEN" : "CLOSED");
      publishQos1(TOPIC_STATE, currentDoorOpen ? "OPEN" : "CLOSED", true);
      lastReportedDoorOpen = currentDoorOpen;
    }
    delay(20);
  }

  // Clean (non-forced) disconnect: the library sends any remaining queued
  // messages before closing the connection. A fixed total delay, not a
  // poll on connected() -- that flag almost certainly flips false
  // synchronously the instant disconnect() is called, before the actual
  // DISCONNECT packet goes out on the library's background task, which
  // would make a connected()-based wait exit immediately every time
  // instead of only occasionally being too short (confirmed: v1.3.5 made
  // this worse, not better, replacing a "sometimes" broker-side Last Will
  // misfire with an "every time" one). Give it real, unconditional
  // wall-clock time instead -- but spent in the same
  // delay-a-little-then-checkDoorPin() loop every other wait in this file
  // already uses, not a single blind delay() like this used to be. This
  // was the one remaining unwatched window: a transition landing here
  // previously went uncounted entirely (checkDoorPin() never got called
  // again to notice it) and could arm armWakeup() below with stale state,
  // which is consistent with real closes only ever going missing around
  // sleep, never while the device stays continuously awake (Debug Mode).
  // MQTT is already on its way down by this point, so a transition caught
  // here doesn't get its own publish -- but it does update currentDoorOpen
  // and today's open/close counters correctly, and the next wake (now
  // armed for the right level) publishes the fresh state itself.
  mqttClient.disconnect();
  unsigned long disconnectStart = millis();
  while (millis() - disconnectStart < MQTT_DISCONNECT_DELAY_MS) {
    delay(20);
    checkDoorPin();
  }
  WiFi.disconnect(true);

  stopAwakeWatchdog(); // about to sleep on our own terms, no need for the failsafe to fire mid-sleep
  armWakeup(currentDoorOpen ? HIGH : LOW); // fresh state, not the stale reading from the top of this cycle
  goToSleep();
}

void loop() {
  if (DEBUG_MODE) {
    static unsigned long lastPrint = 0;
    static int lastLevel = -1;
    int level = digitalRead(REED_PIN);

    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      Serial.printf("[debug] door pin level: %d (%s)\n", level, level == HIGH ? "OPEN" : "CLOSED");
    }

    // Republish immediately on any live change, so you can watch it show up in HA
    if (level != lastLevel && lastLevel != -1) {
      bool doorOpen = (level == HIGH);
      publishQos1(TOPIC_STATE, doorOpen ? "OPEN" : "CLOSED", true);
      Serial.printf("[debug] live state change published: %s\n", doorOpen ? "OPEN" : "CLOSED");
    }
    lastLevel = level;
  }
  // In normal (non-debug) operation this is never reached — device sleeps at the end of setup()
}

// ---------------- WiFi ----------------

// Single connection attempt on the given channel (0 = let the radio auto-scan/pick).
// Returns true if connected within WIFI_CONNECT_TIMEOUT_MS.
bool attemptWifiConnect(uint8_t channel) {
  if (settings.useStaticIp) {
    IPAddress ip, gw, sn, dnsServer;
    if (ip.fromString(settings.staticIp) && gw.fromString(settings.gateway) && sn.fromString(settings.subnet)) {
      if (settings.dns.length() && dnsServer.fromString(settings.dns)) {
        WiFi.config(ip, gw, sn, dnsServer);
      } else {
        WiFi.config(ip, gw, sn);
      }
    } else {
      Serial.println("[debug] Static IP enabled but IP/gateway/subnet fields don't parse -- using DHCP this cycle.");
    }
  }

  uint8_t bssidBytes[6];
  if (settings.bssid.length() && parseBssid(settings.bssid, bssidBytes)) {
    WiFi.begin(settings.wifiSsid.c_str(), settings.wifiPassword.c_str(), channel, bssidBytes);
  } else {
    WiFi.begin(settings.wifiSsid.c_str(), settings.wifiPassword.c_str(), channel);
  }

  unsigned long start = millis();
  wl_status_t lastStatus = WiFi.status();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(50);
    checkDoorPin();
    wl_status_t s = WiFi.status();
    if (s != lastStatus) {
      Serial.printf("[debug] WiFi status changed: %d\n", s);
      lastStatus = s;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected in %lums, IP: %s, channel: %u\n",
                  millis() - start, WiFi.localIP().toString().c_str(), WiFi.channel());
    return true;
  }

  Serial.printf("[debug] Final WiFi status: %d (0=IDLE,1=NO_SSID,3=CONNECTED,4=CONNECT_FAILED,6=DISCONNECTED)\n",
                WiFi.status());
  return false;
}

void connectWiFi() {
  WiFi.mode(WIFI_STA);

  // Reusing the channel from the last successful connect skips a small
  // amount of scan/negotiation time -- worth it here since this device
  // wakes far more often than a timer-only sensor (every door open/close,
  // plus the heartbeat). cachedWifiChannel is 0 (auto) until the first
  // successful connect populates it below.
  bool connected = attemptWifiConnect(cachedWifiChannel);

  // If the cached channel attempt failed (e.g. the router switched channels
  // since we last connected), fall back to one auto-scan retry before
  // giving up on this cycle entirely.
  if (!connected && cachedWifiChannel != 0) {
    Serial.println("[debug] Cached-channel connect failed, retrying with auto channel scan...");
    WiFi.disconnect();
    delay(100);
    connected = attemptWifiConnect(0);
  }

  cachedWifiChannel = connected ? WiFi.channel() : 0;
}

// ---------------- MQTT ----------------

void onMqttConnect(bool sessionPresent) {
  mqttConnectedFlag = true;
}

void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason) {
  mqttConnectedFlag = false;
}

void onMqttPublish(uint16_t packetId) {
  lastAckedPacketId = packetId;
}

void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic,
                    const uint8_t* payload, size_t len, size_t index, size_t total) {
  // Payloads aren't null-terminated -- compare the raw bytes directly.
  bool isOn = (len == 2 && payload[0] == 'O' && payload[1] == 'N');
  if (TOPIC_OTA_CMD.equals(topic)) {
    if (isOn) otaRequested = true;
  } else if (TOPIC_BOOT_RESET_CMD.equals(topic)) {
    if (isOn) bootCountResetRequested = true;
  } else if (TOPIC_SETUP_MODE_CMD.equals(topic)) {
    if (isOn) setupModeRequested = true;
  } else if (TOPIC_FACTORY_RESET_CMD.equals(topic)) {
    if (isOn) factoryResetRequested = true;
  } else if (TOPIC_BATTERY_CAL_OFFSET_SET.equals(topic)) {
    size_t copyLen = len < sizeof(g_battCalCmdPayload) - 1 ? len : sizeof(g_battCalCmdPayload) - 1;
    memcpy(g_battCalCmdPayload, payload, copyLen);
    g_battCalCmdPayload[copyLen] = '\0';
    g_battCalCmdReceived = true;
  } else if (TOPIC_DEBUG_MODE_CMD.equals(topic)) {
    size_t copyLen = len < sizeof(g_debugModeCmdPayload) - 1 ? len : sizeof(g_debugModeCmdPayload) - 1;
    memcpy(g_debugModeCmdPayload, payload, copyLen);
    g_debugModeCmdPayload[copyLen] = '\0';
    g_debugModeCmdReceived = true;
  }
}

void initMqttClient() {
  mqttClient.setServer(settings.mqttHost.c_str(), settings.mqttPort);
  mqttClient.setCredentials(settings.mqttUser.c_str(), settings.mqttPassword.c_str());
  mqttClient.setClientId(settings.deviceId.c_str());
  // LWT: broker marks device "offline" automatically if it drops without a clean disconnect
  mqttClient.setWill(TOPIC_AVAILABILITY.c_str(), 1, true, "offline");
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.onMessage(onMqttMessage);
}

bool connectMQTT() {
  initMqttClient();

  for (uint32_t attempt = 1; attempt <= MQTT_CONNECT_ATTEMPTS; attempt++) {
    Serial.printf("MQTT connect attempt %lu/%lu...\n", attempt, MQTT_CONNECT_ATTEMPTS);
    mqttConnectedFlag = false;
    mqttClient.connect();

    unsigned long start = millis();
    while (!mqttConnectedFlag && millis() - start < MQTT_CONNECT_TIMEOUT_MS) {
      delay(20);
      checkDoorPin();
    }

    if (mqttConnectedFlag) {
      Serial.printf("MQTT connected in %lums (attempt %lu)\n", millis() - start, attempt);
      publishQos1(TOPIC_AVAILABILITY, "online", true);

      // Subscribe and give the broker a moment to deliver any retained OTA
      // or boot-reset command -- this is how a request made while we were
      // asleep gets seen.
      mqttClient.subscribe(TOPIC_OTA_CMD.c_str(), 1);
      mqttClient.subscribe(TOPIC_BOOT_RESET_CMD.c_str(), 1);
      mqttClient.subscribe(TOPIC_SETUP_MODE_CMD.c_str(), 1);
      mqttClient.subscribe(TOPIC_FACTORY_RESET_CMD.c_str(), 1);
      mqttClient.subscribe(TOPIC_BATTERY_CAL_OFFSET_SET.c_str(), 1);
      mqttClient.subscribe(TOPIC_DEBUG_MODE_CMD.c_str(), 1);
      unsigned long subStart = millis();
      while (millis() - subStart < OTA_SUBSCRIBE_WAIT_MS) {
        delay(20);
        checkDoorPin();
      }

      return true;
    }

    Serial.printf("MQTT connect attempt %lu timed out.\n", attempt);
    mqttClient.disconnect(true); // force-clear state before retrying
    delay(200);
  }

  return false;
}

// Publishes at QoS 1 and waits for the broker's PUBACK before returning.
// Retries as a brand-new publish (new packet ID) up to MQTT_PUBLISH_ATTEMPTS
// times if no ack arrives in time.
bool publishQos1(const String& topic, const String& payload, bool retain) {
  for (uint32_t attempt = 1; attempt <= MQTT_PUBLISH_ATTEMPTS; attempt++) {
    lastAckedPacketId = 0;
    uint16_t packetId = mqttClient.publish(topic.c_str(), 1, retain, payload.c_str());

    if (packetId == 0) {
      Serial.printf("[MQTT] publish() failed to queue '%s' (attempt %lu/%lu)\n",
                    topic.c_str(), attempt, MQTT_PUBLISH_ATTEMPTS);
      delay(100);
      continue;
    }

    unsigned long start = millis();
    while (lastAckedPacketId != packetId && millis() - start < MQTT_PUBLISH_ACK_TIMEOUT_MS) {
      delay(10);
      checkDoorPin();
    }

    if (lastAckedPacketId == packetId) {
      return true;
    }

    Serial.printf("[MQTT] No PUBACK for '%s' within %lums (attempt %lu/%lu), retrying...\n",
                  topic.c_str(), MQTT_PUBLISH_ACK_TIMEOUT_MS, attempt, MQTT_PUBLISH_ATTEMPTS);
  }

  Serial.printf("[MQTT] Giving up on '%s' after %lu attempts\n", topic.c_str(), MQTT_PUBLISH_ATTEMPTS);
  return false;
}

void sendDiscoveryConfig() {
  // Binary sensor (door) discovery payload
  String doorPayload = String("{")
    + "\"name\":\"" + settings.deviceName + "\","
    + "\"unique_id\":\"" + settings.deviceId + "_door\","
    + "\"device_class\":\"door\","
    + "\"state_topic\":\"" + TOPIC_STATE + "\","
    + "\"payload_on\":\"OPEN\","
    + "\"payload_off\":\"CLOSED\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"],\"name\":\"" + settings.deviceName + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\",\"model\":\"" + DEVICE_MODEL + "\",\"sw_version\":\"" + FIRMWARE_VERSION + "\",\"hw_version\":\"" + DEVICE_HW_VERSION + "\"}"
    + "}";
  bool doorOk = publishQos1(DISCOVERY_DOOR, doorPayload, true);
  Serial.printf("[debug] Door discovery publish: %s\n", doorOk ? "OK" : "FAILED");

  // Battery voltage sensor discovery payload
  String battPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Battery Voltage\","
    + "\"unique_id\":\"" + settings.deviceId + "_battery\","
    + "\"device_class\":\"voltage\","
    + "\"unit_of_measurement\":\"V\","
    + "\"state_class\":\"measurement\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_BATTERY + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  bool battOk = publishQos1(DISCOVERY_BATTERY, battPayload, true);
  Serial.printf("[debug] Battery discovery publish: %s\n", battOk ? "OK" : "FAILED");

  // Pre-calibration reading -- the same ADC sample and divider ratio, but
  // no BATT_CAL curve or HA offset applied. Exists purely so there's always
  // an honest "what the hardware actually measured" number to compare
  // against a multimeter, regardless of whatever calibration is currently
  // applied (the Battery Voltage sensor above reflects THAT).
  String battVRawPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Battery Voltage (Raw)\","
    + "\"unique_id\":\"" + settings.deviceId + "_battery_voltage_raw\","
    + "\"device_class\":\"voltage\","
    + "\"unit_of_measurement\":\"V\","
    + "\"state_class\":\"measurement\","
    + "\"entity_category\":\"diagnostic\","
    + "\"suggested_display_precision\":3,"
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_BATTERY_V_RAW + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BATTERY_V_RAW, battVRawPayload, true);

  // Battery percentage sensor discovery payload (device_class: battery gives it the standard battery icon in HA)
  String battPctPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Battery\","
    + "\"unique_id\":\"" + settings.deviceId + "_battery_percent\","
    + "\"device_class\":\"battery\","
    + "\"unit_of_measurement\":\"%\","
    + "\"state_class\":\"measurement\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_BATTERY_PCT + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BATTERY_PCT, battPctPayload, true);

  // A control, not a reading -- no expire_after, same reasoning as the OTA/
  // boot-reset switches below. A plain volts offset added to every future
  // battery reading: work out the offset as (multimeter reading) - (Battery
  // Voltage (Raw) above) and enter THAT difference here -- e.g. 0.15 means
  // "the hardware reads 0.15V low, add 0.15V from now on." Applies on the
  // device's next wake and is echoed back as the new state.
  String battCalPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Battery Calibration Offset\","
    + "\"unique_id\":\"" + settings.deviceId + "_battery_cal_offset\","
    + "\"entity_category\":\"config\","
    + "\"icon\":\"mdi:tune-variant\","
    + "\"min\":-1,"
    + "\"max\":1,"
    + "\"step\":0.01,"
    + "\"unit_of_measurement\":\"V\","
    + "\"mode\":\"box\","
    + "\"optimistic\":false,"
    + "\"retain\":true,"
    + "\"command_topic\":\"" + TOPIC_BATTERY_CAL_OFFSET_SET + "\","
    + "\"state_topic\":\"" + TOPIC_BATTERY_CAL_OFFSET + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BATTERY_CAL_OFFSET, battCalPayload, true);

  // Debug Mode: keeps the device awake (no deep sleep) and opens a raw TCP
  // server (NETWORK_DEBUG_PORT, see runDebugSession()) mirroring everything
  // normally printed over USB Serial, for watching door-pin debounce
  // behavior live once this device is mounted somewhere USB isn't
  // reachable. Persistent, not a one-shot trigger -- reflects the actual
  // current state, including after it auto-expires
  // (DEBUG_SESSION_TIMEOUT_MS) on its own.
  String debugModePayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Debug Mode\","
    + "\"unique_id\":\"" + settings.deviceId + "_debug_mode\","
    + "\"command_topic\":\"" + TOPIC_DEBUG_MODE_CMD + "\","
    + "\"state_topic\":\"" + TOPIC_DEBUG_MODE_STATE + "\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"optimistic\":false,"
    + "\"retain\":true,"
    + "\"icon\":\"mdi:bug\","
    + "\"entity_category\":\"config\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_DEBUG_MODE, debugModePayload, true);

  // Low-battery binary sensor discovery payload
  String battLowPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Low Battery\","
    + "\"unique_id\":\"" + settings.deviceId + "_battery_low\","
    + "\"device_class\":\"battery\","
    + "\"entity_category\":\"diagnostic\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_BATTERY_LOW + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BATTERY_LOW, battLowPayload, true);

  // Last full charge date -- device_class "date" (not "timestamp": we only
  // ever record a calendar day, not a time-of-day). No expire_after: this
  // is a record of a past event, not a live reading, and should stay
  // visible even across a long gap between full charges.
  String lastFullChargePayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Last Full Charge\","
    + "\"unique_id\":\"" + settings.deviceId + "_last_full_charge\","
    + "\"device_class\":\"date\","
    + "\"entity_category\":\"diagnostic\","
    + "\"icon\":\"mdi:battery-charging-100\","
    + "\"state_topic\":\"" + TOPIC_LAST_FULL_CHARGE + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_LAST_FULL_CHARGE, lastFullChargePayload, true);

  // WiFi signal strength sensor discovery payload
  String rssiPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " WiFi Signal\","
    + "\"unique_id\":\"" + settings.deviceId + "_wifi_signal\","
    + "\"device_class\":\"signal_strength\","
    + "\"unit_of_measurement\":\"dBm\","
    + "\"state_class\":\"measurement\","
    + "\"entity_category\":\"diagnostic\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_RSSI + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_RSSI, rssiPayload, true);

  // Boot count sensor discovery payload (diagnostic -- total wakes since last full reset)
  String bootCountPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Boot Count\","
    + "\"unique_id\":\"" + settings.deviceId + "_boot_count\","
    + "\"entity_category\":\"diagnostic\","
    + "\"state_class\":\"total_increasing\","
    + "\"icon\":\"mdi:counter\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_BOOT_COUNT + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BOOT_COUNT, bootCountPayload, true);

  // Consecutive connect-fail count (resets to 0 on the next fully successful wake)
  String failCountPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Connect Fail Count\","
    + "\"unique_id\":\"" + settings.deviceId + "_connect_fail_count\","
    + "\"entity_category\":\"diagnostic\","
    + "\"state_class\":\"measurement\","
    + "\"icon\":\"mdi:wifi-alert\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_FAIL_COUNT + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_FAIL_COUNT, failCountPayload, true);

  // Lifetime total failed-wake count (never resets)
  String totalFailCountPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Total Fail Count\","
    + "\"unique_id\":\"" + settings.deviceId + "_total_fail_count\","
    + "\"entity_category\":\"diagnostic\","
    + "\"state_class\":\"total_increasing\","
    + "\"icon\":\"mdi:counter\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_TOTAL_FAIL_COUNT + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_TOTAL_FAIL_COUNT, totalFailCountPayload, true);

  // OTA trigger switch discovery payload. "retain: true" makes HA publish
  // the ON command with the retain flag set, so it survives on the broker
  // until this (sleeping) device actually wakes up and subscribes to see it.
  // Note: no expire_after here -- this is a control, not a reading, and
  // should stay usable in the UI even if the device has been quiet a while.
  String otaPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " OTA Update\","
    + "\"unique_id\":\"" + settings.deviceId + "_ota\","
    + "\"command_topic\":\"" + TOPIC_OTA_CMD + "\","
    + "\"state_topic\":\"" + TOPIC_OTA_STATE + "\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"retain\":true,"
    + "\"entity_category\":\"config\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_OTA, otaPayload, true);

  // Reset-boot-counter control. Implemented as a retained switch, same
  // trick as OTA Update above, not a plain MQTT "button" entity -- a
  // button's press is a one-shot, non-retained message that a sleeping
  // device would simply miss if it happened to be pressed between wakes.
  // HA publishes ON with retain, the device consumes it on its next wake
  // and immediately reports back OFF, which reads as a momentary action
  // in the UI even though the wire protocol underneath is a switch.
  String bootResetPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Reset Boot Counter\","
    + "\"unique_id\":\"" + settings.deviceId + "_boot_count_reset\","
    + "\"command_topic\":\"" + TOPIC_BOOT_RESET_CMD + "\","
    + "\"state_topic\":\"" + TOPIC_BOOT_RESET_STATE + "\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"retain\":true,"
    + "\"icon\":\"mdi:restore\","
    + "\"entity_category\":\"config\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BOOT_RESET, bootResetPayload, true);

  // OTA-active indicator. This device has no status LED (unlike the other
  // battery sensors in this fleet) to show OTA is in progress, so this is
  // the only indication of it -- ON only for the ~OTA_WINDOW_MS the device
  // stays awake listening for a flash, OFF the rest of the time.
  String otaActivePayload = String("{")
    + "\"name\":\"" + settings.deviceName + " OTA Active\","
    + "\"unique_id\":\"" + settings.deviceId + "_ota_active\","
    + "\"entity_category\":\"diagnostic\","
    + "\"icon\":\"mdi:upload\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"state_topic\":\"" + TOPIC_OTA_ACTIVE + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_OTA_ACTIVE, otaActivePayload, true);

  // Setup Mode: reopens the web setup portal on the device's next connected
  // wake -- this device has no spare GPIO for a physical setup button, so
  // this is the only way to revisit WiFi/MQTT/static-IP/BSSID settings once
  // it's already configured (a never-configured device opens the portal on
  // its own, no switch needed). Retained, same momentary-via-echo pattern
  // as the other switches here. Takes effect on whichever happens first --
  // the next door event, or the heartbeat (up to HEARTBEAT_INTERVAL_US
  // away) -- opening/closing the door once forces it immediately.
  String setupModePayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Setup Mode\","
    + "\"unique_id\":\"" + settings.deviceId + "_setup_mode\","
    + "\"command_topic\":\"" + TOPIC_SETUP_MODE_CMD + "\","
    + "\"state_topic\":\"" + TOPIC_SETUP_MODE_STATE + "\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"retain\":true,"
    + "\"icon\":\"mdi:wifi-cog\","
    + "\"entity_category\":\"config\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_SETUP_MODE, setupModePayload, true);

  // Factory Reset: wipes every saved setting (WiFi/MQTT/device identity/
  // static IP/BSSID/battery offset) and the radio's own persisted WiFi
  // credentials, then restarts unconfigured -- equivalent to a fresh,
  // never-set-up unit. Same MQTT-triggered mechanism as Setup Mode above
  // (no button to hold), but acted on immediately upon receipt rather than
  // waiting for the rest of the cycle, since there's nothing left worth
  // publishing afterward.
  String factoryResetPayload = String("{")
    + "\"name\":\"" + settings.deviceName + " Factory Reset\","
    + "\"unique_id\":\"" + settings.deviceId + "_factory_reset\","
    + "\"command_topic\":\"" + TOPIC_FACTORY_RESET_CMD + "\","
    + "\"state_topic\":\"" + TOPIC_FACTORY_RESET_STATE + "\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"retain\":true,"
    + "\"icon\":\"mdi:restore-alert\","
    + "\"entity_category\":\"config\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + settings.deviceId + "\"]}"
    + "}";
  publishQos1(DISCOVERY_FACTORY_RESET, factoryResetPayload, true);
}

// Returns the number of topics that never got a PUBACK this cycle (0 = fully
// successful). failCount is this device's connect-fail count as of the
// START of this cycle (i.e. not yet updated for this cycle's own outcome) --
// the caller updates it afterwards based on the return value, same pattern
// as temp_humidity_sensor.
int publishState(bool doorOpen, float batteryVoltage, float batteryPercent, float rawBatteryVoltage, int rssi, uint32_t failCount) {
  int failed = 0;

  if (!publishQos1(TOPIC_STATE, doorOpen ? "OPEN" : "CLOSED", true)) failed++;

  char battStr[8];
  dtostrf(batteryVoltage, 4, 2, battStr);
  if (!publishQos1(TOPIC_BATTERY, battStr, true)) failed++;

  char battRawStr[8];
  dtostrf(rawBatteryVoltage, 4, 3, battRawStr);
  if (!publishQos1(TOPIC_BATTERY_V_RAW, battRawStr, true)) failed++;

  char battPctStr[8];
  dtostrf(batteryPercent, 4, 0, battPctStr);
  if (!publishQos1(TOPIC_BATTERY_PCT, battPctStr, true)) failed++;

  bool batteryLow = batteryPercent < BATTERY_LOW_THRESHOLD_PCT;
  if (!publishQos1(TOPIC_BATTERY_LOW, batteryLow ? "ON" : "OFF", true)) failed++;

  String lastFullChargeDate = updateAndGetLastFullChargeDate(batteryPercent);
  if (lastFullChargeDate.length() > 0) {
    if (!publishQos1(TOPIC_LAST_FULL_CHARGE, lastFullChargeDate, true)) failed++;
  }

  char rssiStr[8];
  snprintf(rssiStr, sizeof(rssiStr), "%d", rssi);
  if (!publishQos1(TOPIC_RSSI, rssiStr, true)) failed++;

  if (!publishQos1(TOPIC_BOOT_COUNT, String(bootCount), true)) failed++;
  if (!publishQos1(TOPIC_FAIL_COUNT, String(failCount), true)) failed++;
  if (!publishQos1(TOPIC_TOTAL_FAIL_COUNT, String(totalFailCount), true)) failed++;

  // Always OFF here -- this runs before the OTA check below. Also acts as a
  // safety net: if the device somehow died mid-OTA on a previous wake, the
  // next normal wake clears a stale retained "ON" automatically.
  if (!publishQos1(TOPIC_OTA_ACTIVE, "OFF", true)) failed++;

  Serial.printf("Published: door=%s, battery=%sV (%s%%, low=%s), rssi=%sdBm, boot=%lu, failCount=%lu (failed topics this cycle: %d)\n",
                doorOpen ? "OPEN" : "CLOSED", battStr, battPctStr, batteryLow ? "yes" : "no",
                rssiStr, (unsigned long)bootCount, (unsigned long)failCount, failed);

  return failed;
}

// ---------------- Battery ----------------

// Detects a rising edge into 100% battery (i.e. "just charged", not "still
// sitting at 100% from before") and records today's date as the new last-
// full-charge date. Persisted in flash/NVS rather than RTC memory
// specifically so it survives an actual battery depletion -- comparing
// this date to whenever the device later goes quiet is how you tell how
// long a charge actually lasted. Returns whatever date is currently
// stored (possibly still empty, if the battery has never yet read 100%
// since this was added).
//
// If the clock hasn't synced yet (g_timeSynced false) when a rising edge
// happens, wasAt100 still gets set so this doesn't re-trigger every wake,
// but no date gets recorded -- a one-time, cosmetic gap on a device's
// very first-ever boot.
String updateAndGetLastFullChargeDate(float batteryPercent) {
  batteryPrefs.begin("battery", false);
  bool wasAt100 = batteryPrefs.getBool("wasAt100", false);
  bool isAt100 = batteryPercent >= 100.0f;

  if (isAt100 && !wasAt100 && g_timeSynced) {
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    char buf[11];
    strftime(buf, sizeof(buf), "%Y-%m-%d", &t);
    batteryPrefs.putString("lastFullDate", buf);
    Serial.printf("[battery] reached 100%% -- recorded last full charge date: %s\n", buf);
  }
  if (isAt100 != wasAt100) {
    batteryPrefs.putBool("wasAt100", isAt100);
  }

  String result = batteryPrefs.getString("lastFullDate", "");
  batteryPrefs.end();
  return result;
}

// Applies this board's own raw-vs-actual voltage correction (BATT_CAL in
// config.h). Defaults to identity (no correction) until calibrated.
float calibrateBatteryVoltage(float raw) {
  if (raw <= BATT_CAL[0].raw) {
    float slope = (BATT_CAL[1].actual - BATT_CAL[0].actual) / (BATT_CAL[1].raw - BATT_CAL[0].raw);
    return BATT_CAL[0].actual + (raw - BATT_CAL[0].raw) * slope;
  }
  if (raw >= BATT_CAL[BATT_CAL_POINTS - 1].raw) {
    float slope = (BATT_CAL[BATT_CAL_POINTS - 1].actual - BATT_CAL[BATT_CAL_POINTS - 2].actual)
                 / (BATT_CAL[BATT_CAL_POINTS - 1].raw - BATT_CAL[BATT_CAL_POINTS - 2].raw);
    return BATT_CAL[BATT_CAL_POINTS - 1].actual + (raw - BATT_CAL[BATT_CAL_POINTS - 1].raw) * slope;
  }
  for (int i = 0; i < BATT_CAL_POINTS - 1; i++) {
    if (raw >= BATT_CAL[i].raw && raw <= BATT_CAL[i + 1].raw) {
      float slope = (BATT_CAL[i + 1].actual - BATT_CAL[i].actual) / (BATT_CAL[i + 1].raw - BATT_CAL[i].raw);
      return BATT_CAL[i].actual + (raw - BATT_CAL[i].raw) * slope;
    }
  }
  return raw; // unreachable, keeps the compiler happy
}

// Piecewise-linear state-of-charge curve for a typical single-cell Li-ion,
// same curve used across the other battery-powered sensors so readings are
// consistent device to device.
//
// Rescaled from the original 4.20V-100%/3.20V-0% curve: every breakpoint
// proportionally compressed by 0.95 (= 0.95V new span / 1.00V old span)
// so the curve's shape (same relative tier widths, same emphasis on the
// 3.5-3.9V "knee") is preserved but fit to 4.15V-100%/3.20V-0% instead of
// just flattening the top into an instant jump at 4.15V.
float batteryPercentage(float v) {
  float pct;

  if (v >= 4.15)      pct = 100.0;
  else if (v >= 4.10) pct = 95.0 + (v - 4.10) / 0.05 * 5.0;
  else if (v >= 4.06) pct = 90.0 + (v - 4.06) / 0.04 * 5.0;
  else if (v >= 4.01) pct = 85.0 + (v - 4.01) / 0.05 * 5.0;
  else if (v >= 3.96) pct = 80.0 + (v - 3.96) / 0.05 * 5.0;
  else if (v >= 3.91) pct = 75.0 + (v - 3.91) / 0.05 * 5.0;
  else if (v >= 3.87) pct = 70.0 + (v - 3.87) / 0.04 * 5.0;
  else if (v >= 3.80) pct = 65.0 + (v - 3.80) / 0.07 * 5.0;
  else if (v >= 3.72) pct = 60.0 + (v - 3.72) / 0.08 * 5.0;
  else if (v >= 3.68) pct = 55.0 + (v - 3.68) / 0.04 * 5.0;
  else if (v >= 3.66) pct = 50.0 + (v - 3.66) / 0.02 * 5.0;
  else if (v >= 3.62) pct = 45.0 + (v - 3.62) / 0.04 * 5.0;
  else if (v >= 3.58) pct = 40.0 + (v - 3.58) / 0.04 * 5.0;
  else if (v >= 3.54) pct = 35.0 + (v - 3.54) / 0.04 * 5.0;
  else if (v >= 3.49) pct = 30.0 + (v - 3.49) / 0.05 * 5.0;
  else if (v >= 3.44) pct = 25.0 + (v - 3.44) / 0.05 * 5.0;
  else if (v >= 3.39) pct = 20.0 + (v - 3.39) / 0.05 * 5.0;
  else if (v >= 3.34) pct = 15.0 + (v - 3.34) / 0.05 * 5.0;
  else if (v >= 3.30) pct = 10.0 + (v - 3.30) / 0.04 * 5.0;
  else if (v >= 3.25) pct =  5.0 + (v - 3.25) / 0.05 * 5.0;
  else if (v >= 3.20) pct =  0.0 + (v - 3.20) / 0.05 * 5.0;
  else                pct = 0.0;

  return roundf(pct);
}

// ---------------- Setup portal ----------------

// Reached two ways: a never-configured device (settings.configured false)
// goes here unconditionally at boot, or an already-configured one is
// retargeted here mid-cycle via the "Setup Mode" MQTT switch. viaMqttRequest
// only affects the opening log line below -- same as temp_humidity_sensor_v4,
// where the equivalent bool (there, "held via button") never branched
// anything else either.
void runMaintenanceMode(bool viaMqttRequest) {
  Serial.println(viaMqttRequest
    ? "Setup Mode requested via MQTT -- reopening setup portal."
    : "No saved WiFi config yet -- entering first-time setup.");
  startAwakeWatchdog((PORTAL_TIMEOUT_SEC + 60) * 1000UL);

  // Snapshot of current readings plus last-known connectivity, shown as a
  // read-only section on the portal page -- replaces the portal's default
  // "Info" menu page (generic ESP32 chip/heap/uptime trivia, hidden below)
  // with something actually about this sensor. Taken once, right now, not
  // a live dashboard -- the page doesn't refresh itself while it's open.
  uint32_t statusRawMv = analogReadMilliVolts(BATT_PIN);
  float statusRawV = (statusRawMv / 1000.0f) * BATT_DIVIDER_RATIO;
  float statusBattV = calibrateBatteryVoltage(statusRawV) + settings.battVoltageOffsetV;
  float statusBattPct = batteryPercentage(statusBattV);

  // Scanned once here, before the portal's own AP+STA setup, purely for
  // the BSSID reference list below the network fields -- WiFiManager's own
  // network picker has no concept of BSSID at all (SSID only), so this is
  // a separate scan of our own. Blocking, adds a few seconds to entering
  // the portal; WiFi.scanDelete() frees the result buffer once we've
  // copied what we need out of it.
  //
  // Each entry is clickable (inline onclick, not a <script> block -- about
  // as compatible as JS gets, works even in the more restrictive captive-
  // portal browsers some phones use) and fills the BSSID field below by
  // id. A nearby network's SSID is attacker-controlled data -- any AP in
  // range can broadcast whatever string it wants -- so it's run through
  // jsAttrEscape() before going anywhere near that onclick attribute;
  // skipping that would let a maliciously-named nearby network inject
  // script into this device's own setup page. If JS genuinely isn't
  // available, clicking just does nothing -- the field is still a normal
  // text input either way.
  WiFi.mode(WIFI_STA);
  int scanCount = WiFi.scanNetworks();
  String bssidListHtml = "<div style='font-size:0.85em;max-height:140px;overflow-y:auto;"
    "background:#f9f9f9;border-radius:6px;padding:6px;margin:4px 0;'>";
  if (scanCount <= 0) {
    bssidListHtml += "No networks seen during scan.";
  } else {
    for (int i = 0; i < scanCount; i++) {
      String bssidStr = WiFi.BSSIDstr(i);
      bssidListHtml += "<span onclick=\"document.getElementById('bssid').value='"
        + jsAttrEscape(bssidStr) + "'\" style='cursor:pointer;text-decoration:underline;color:#0a7d3c;'>"
        + htmlEscape(WiFi.SSID(i)) + " &mdash; " + bssidStr
        + " (" + String(WiFi.RSSI(i)) + " dBm)</span><br>";
    }
  }
  bssidListHtml += "</div>";
  WiFi.scanDelete();

  String doorStatusStr = currentDoorOpen ? "OPEN" : "CLOSED";
  String wifiStatusStr = settings.configured
    ? (settings.wifiSsid.length() ? ("last connected: " + settings.wifiSsid) : String("no WiFi saved yet"))
    : String("not yet configured");
  // Independent of settings.configured (unlike wifiStatusStr/mqttStatusStr
  // above) -- static IP/BSSID can be filled in on a portal visit that
  // otherwise leaves the device unconfigured (e.g. MQTT host still blank),
  // and are meaningful to show either way.
  String staticIpStatusStr = settings.useStaticIp
    ? (settings.staticIp + " (gateway " + settings.gateway + ", subnet " + settings.subnet
       + (settings.dns.length() ? (", DNS " + settings.dns) : "") + ")")
    : String("DHCP");
  String bssidStatusStr = settings.bssid.length() ? settings.bssid : String("none (any AP)");
  String mqttStatusStr = (settings.configured && settings.mqttHost.length())
    ? (settings.mqttHost + ":" + String(settings.mqttPort))
    : String("not yet configured");
  String statusHtml = String("<div style='background:#f4f4f4;border-radius:6px;padding:10px;margin:10px 0;font-size:0.9em;'>")
    + "<strong>Device status</strong> (just read)<br>"
    + "Door: " + doorStatusStr + "<br>"
    + "Battery: " + String(statusBattV, 2) + "V (" + String(statusBattPct, 0) + "%)<br>"
    + "WiFi: " + wifiStatusStr + "<br>"
    + "IP config: " + staticIpStatusStr + "<br>"
    + "BSSID pin: " + bssidStatusStr + "<br>"
    + "MQTT broker: " + mqttStatusStr + "<br>"
    + "Boot count: " + String(bootCount) + " &middot; connect fails: " + String(connectFailCount)
    + " today / " + String(totalFailCount) + " total"
    + "</div>";
  // Lime green circular "P@cho" badge -- inline SVG rather than a
  // base64-encoded raster image, since it's plain text (a few hundred
  // bytes) and needs no separate HTTP request. Prepended to statusHtml
  // below so both render together as one "custom" menu-slot block,
  // positioned at the very top of the landing page -- branding and status
  // first, then the action buttons.
  String logoSvg = "<svg width='120' height='120' viewBox='250 30 180 180' xmlns='http://www.w3.org/2000/svg' "
    "style='display:block;margin:8px auto;'>"
    "<circle cx='340' cy='120' r='90' fill='#65A30D'/>"
    "<circle cx='340' cy='120' r='90' fill='none' stroke='#A3E635' stroke-width='3'/>"
    "<g transform='translate(340,120) scale(1.15) translate(-323.5,-125.5)'>"
    "<text x='272' y='160' font-size='102' font-weight='600' font-family='Arial, sans-serif' fill='#FFFFFF'>P</text>"
    "<text x='300' y='156' font-size='30' font-weight='500' font-family='Arial, sans-serif' fill='#FFFFFF'>@cho</text>"
    "</g>"
    "<circle cx='388' cy='64' r='6' fill='#D9F99D'/>"
    "<circle cx='388' cy='64' r='16' fill='none' stroke='#D9F99D' stroke-width='2.5' opacity='0.8'/>"
    "<circle cx='388' cy='64' r='27' fill='none' stroke='#D9F99D' stroke-width='2' opacity='0.5'/>"
    "</svg>";
  // Rendered on the landing menu page (above the button list), not the
  // "Configure WiFi" form -- via setCustomMenuHTML() + the "custom" menu
  // token below, rather than as a WiFiManagerParameter. These have to
  // stay alive for as long as wm does, same reasoning as versionHeader
  // further down.
  String customMenuHtml = logoSvg + statusHtml;

  char mqttPortStr[6];
  snprintf(mqttPortStr, sizeof(mqttPortStr), "%u", settings.mqttPort);

  // Network settings (static IP + BSSID pin). NOT a checkbox: a checkbox
  // was tried first in temp_humidity_sensor_v4 and never actually worked --
  // WiFiManagerParameter's template always emits value='{defaultValue}'
  // itself, so ANY custom "value=" attribute added alongside it (checked or
  // not, empty default or not) collides with that and produces a duplicate
  // HTML `value` attribute, which browsers resolve unpredictably. There's
  // no way to build a working checkbox through this API at all -- so:
  // static IP is chosen implicitly by filling in the IP address field
  // below, not by a separate checkbox next to it.
  String bssidSectionHtml = "<p style='margin-bottom:4px;font-size:0.9em;'>Networks seen just now "
    "(SSID &mdash; BSSID &mdash; signal) &mdash; tap one to fill in its BSSID below, "
    "or type/paste one in manually:</p>" + bssidListHtml;
  WiFiManagerParameter p_net_heading(
    "<hr><p style='margin-bottom:0;'><strong>Network settings (optional)</strong><br>"
    "Leave the IP address field below blank for DHCP (recommended unless you have "
    "a specific reason for a static IP) -- fill it in, along with Gateway and "
    "Subnet, to use a static IP instead.</p>");
  WiFiManagerParameter p_static_ip("static_ip", "Static IP address (blank = DHCP)", settings.staticIp.c_str(), 15);
  WiFiManagerParameter p_gateway("gateway", "Gateway", settings.gateway.c_str(), 15);
  WiFiManagerParameter p_subnet("subnet", "Subnet mask", settings.subnet.c_str(), 15);
  WiFiManagerParameter p_dns("dns", "DNS server (optional)", settings.dns.c_str(), 15);
  WiFiManagerParameter p_bssid_heading(bssidSectionHtml.c_str());
  WiFiManagerParameter p_bssid("bssid", "WiFi BSSID / MAC (optional, pins to one access point)", settings.bssid.c_str(), 17);

  // Raw-HTML parameter (no id/value, just markup) to visually separate the
  // MQTT/device fields below from the WiFi network picker above them on
  // the same "Configure WiFi" page -- both sections are one form, saved
  // together in a single submission.
  WiFiManagerParameter p_mqtt_heading(
    "<hr><p style='margin-bottom:0;'><strong>MQTT &amp; device settings</strong><br>"
    "(same form as the WiFi network above -- fill in both, then Save once)</p>");
  WiFiManagerParameter p_mqtt_host("mqtt_host", "MQTT broker host or IP", settings.mqttHost.c_str(), 64, "required");
  WiFiManagerParameter p_mqtt_port("mqtt_port", "MQTT broker port", mqttPortStr, 6);
  WiFiManagerParameter p_mqtt_user("mqtt_user", "MQTT username", settings.mqttUser.c_str(), 32);
  WiFiManagerParameter p_mqtt_pass("mqtt_pass", "MQTT password", settings.mqttPassword.c_str(), 32, "type='password'");
  WiFiManagerParameter p_device_name("device_name", "Device name (shown in Home Assistant)", settings.deviceName.c_str(), 40);
  WiFiManagerParameter p_device_id("device_id", "Device ID (MQTT topics, no spaces)", settings.deviceId.c_str(), 32);

  WiFiManager wm;
  // Shown at the top of every portal page -- so you can tell which unit and
  // which build you're looking at without digging through Serial or Home
  // Assistant. Also relabels the "Configure WiFi" button to just
  // "Configure" and hides the landing page's own default "WiFiManager"
  // header -- see temp_humidity_sensor_v4's identical block for the full
  // reasoning (confirmed against the library's actual markup, not guessed).
  String versionHeader = "<style>body::before{content:'" + String(DEVICE_MANUFACTURER) + " " + String(DEVICE_MODEL)
                          + " Sensor\\A Firmware v" + String(FIRMWARE_VERSION)
                          + "';white-space:pre-line;display:block;text-align:center;color:#888;margin:4px 0;}"
                          + "form[action='/wifi'] button{font-size:0;}"
                          + "form[action='/wifi'] button::after{content:'Configure';font-size:1rem;}"
                          + "h1,h3{display:none;}</style>";
  wm.setCustomHeadElement(versionHeader.c_str());
  wm.addParameter(&p_net_heading);
  wm.addParameter(&p_static_ip);
  wm.addParameter(&p_gateway);
  wm.addParameter(&p_subnet);
  wm.addParameter(&p_dns);
  wm.addParameter(&p_bssid_heading);
  wm.addParameter(&p_bssid);
  wm.addParameter(&p_mqtt_heading);
  wm.addParameter(&p_mqtt_host);
  wm.addParameter(&p_mqtt_port);
  wm.addParameter(&p_mqtt_user);
  wm.addParameter(&p_mqtt_pass);
  wm.addParameter(&p_device_name);
  wm.addParameter(&p_device_id);
  wm.setConfigPortalTimeout(PORTAL_TIMEOUT_SEC);
  // Deliberately NOT calling setParamsPage() and NOT listing "param" in the
  // menu below -- see temp_humidity_sensor_v4's identical comment: this
  // keeps the WiFi picker and the MQTT/device fields on one page instead of
  // two disconnected ones. The "custom" token renders _customMenuHTML (set
  // below) at this position in the landing menu, above the button list.
  // Also hides the built-in "Erase" button (only clears the radio's own
  // WiFi credentials, not our settings -- easy to mistake for a real
  // factory reset) and the built-in "Info" page (generic chip/heap trivia).
  std::vector<const char*> menu = {"custom", "wifi", "sep", "restart", "exit"};
  wm.setMenu(menu);
  wm.setCustomMenuHTML(customMenuHtml.c_str());
  wm.setCaptivePortalEnable(true);

  // WPA2 requires an 8-63 character password -- anything shorter and
  // WiFi.softAP() fails to bring the AP up at all (no visible error, it
  // just never appears). Rather than fail silently on a bad config.h
  // value, fall back to an open setup network instead.
  const char* apPassword = AP_PASSWORD;
  size_t apPasswordLen = strlen(apPassword);
  if (apPasswordLen > 0 && apPasswordLen < 8) {
    Serial.printf("[setup] AP_PASSWORD is %u characters -- WPA2 needs at least 8, "
                  "falling back to an OPEN setup network instead of failing silently.\n",
                  (unsigned)apPasswordLen);
    apPassword = nullptr; // WiFiManager treats null as "open network, no password"
  }

  // Always startConfigPortal(), never autoConnect(): this function is only
  // ever reached when settings.configured is false or Setup Mode was
  // requested. Either way there's no known-good config worth trying first
  // -- autoConnect()'s "try the radio's own last-saved network" step reads
  // whatever the ESP32 WiFi driver itself last connected to (persisted
  // independently, chip-wide), not our own settings.
  //
  // Non-blocking, same as temp_humidity_sensor_v4 -- WiFiManager hands
  // control back to us via process() instead of blocking inside
  // startConfigPortal() itself. Unlike that project, there's no LED to
  // drive during the wait here, so the loop below is otherwise idle.
  String apName = String(DEVICE_MANUFACTURER) + " " + String(DEVICE_MODEL) + " " + getShortChipId().substring(2);
  wm.setConfigPortalBlocking(false);
  wm.startConfigPortal(apName.c_str(), apPassword);

  // No button on this board, so unlike temp_humidity_sensor_v4 there's no
  // in-portal cancel gesture or factory-reset hold here -- Factory Reset is
  // its own separate MQTT switch (see setup()), and the only way out of a
  // portal opened by mistake is to let it time out.
  while (wm.getConfigPortalActive() && WiFi.status() != WL_CONNECTED) {
    wm.process();
    delay(10);
  }
  bool connected = (WiFi.status() == WL_CONNECTED);

  if (!connected) {
    Serial.println("Setup portal timed out / no connection -- resuming normal cycle with existing settings.");
    stopAwakeWatchdog();
    return;
  }

  settings.mqttHost     = p_mqtt_host.getValue();
  int parsedPort        = atoi(p_mqtt_port.getValue());
  settings.mqttPort     = (parsedPort > 0 && parsedPort <= 65535) ? (uint16_t)parsedPort : 1883;
  settings.mqttUser     = p_mqtt_user.getValue();
  settings.mqttPassword = p_mqtt_pass.getValue();
  settings.deviceName   = p_device_name.getValue();
  settings.deviceId     = p_device_id.getValue();
  settings.deviceId.replace(" ", "_"); // MQTT topics can't contain spaces
  // WiFiManager already connected us using whatever credentials it saved
  // (freshly entered in the portal, or previously-saved ones) -- capture
  // them from the live connection rather than re-parsing the portal's own
  // internal state.
  settings.wifiSsid     = WiFi.SSID();
  settings.wifiPassword = WiFi.psk();

  // Static IP / BSSID pin. Fields are saved as entered regardless of
  // validity (so a typo is still there to fix on the next portal visit,
  // not silently wiped), but useStaticIp only gets set true if the IP
  // field is non-empty AND it/gateway/subnet actually parse.
  bool staticRequested = (strlen(p_static_ip.getValue()) > 0);
  settings.staticIp = p_static_ip.getValue();
  settings.gateway  = p_gateway.getValue();
  settings.subnet   = p_subnet.getValue();
  settings.dns      = p_dns.getValue();
  settings.bssid    = p_bssid.getValue();
  if (staticRequested) {
    IPAddress checkIp, checkGw, checkSn;
    if (checkIp.fromString(settings.staticIp) && checkGw.fromString(settings.gateway) && checkSn.fromString(settings.subnet)) {
      settings.useStaticIp = true;
    } else {
      Serial.println("Static IP requested but IP/gateway/subnet don't parse as valid addresses -- falling back to DHCP.");
      settings.useStaticIp = false;
    }
  } else {
    settings.useStaticIp = false;
  }

  // Backstop, not the primary defense -- the mqtt_host field is marked HTML
  // `required` now, so a normal browser won't submit the form with it blank
  // in the first place. An empty MQTT host means this device could never
  // actually publish anything, so don't mark it "configured" on a
  // submission like that. Leaving `configured` false means the next boot
  // goes straight back to setup on its own.
  if (settings.mqttHost.length() == 0) {
    saveSettings(); // still keep the WiFi/device fields that were filled in
    Serial.println("Setup portal closed with an empty MQTT broker host -- not marking as configured.");
    stopAwakeWatchdog();
    return;
  }

  settings.configured   = true;
  buildTopics(); // deviceId may have just changed -- rebuild before anything else uses the old topics
  saveSettings();
  Serial.printf("Setup saved: device_id=%s mqtt=%s:%u\n",
                settings.deviceId.c_str(), settings.mqttHost.c_str(), settings.mqttPort);

  stopAwakeWatchdog();
  Serial.println("Restarting into normal operation...");
  Serial.flush();
  delay(200);
  ESP.restart();
}

// ---------------- Diagnostics ----------------

String wakeupCauseToString(esp_sleep_wakeup_cause_t cause) {
  switch (cause) {
    case ESP_SLEEP_WAKEUP_GPIO:      return "door_event";
    case ESP_SLEEP_WAKEUP_TIMER:     return "heartbeat";
    case ESP_SLEEP_WAKEUP_UNDEFINED: return "power_on_or_reset";
    default:                         return "other";
  }
}

// ---------------- Door pin polling ----------------

// Re-reads the reed switch and, if it's genuinely changed since the last
// known state (currentDoorOpen), debounces it and updates currentDoorOpen.
// Called from inside the WiFi-connect, MQTT-connect, per-publish
// PUBACK-wait, and OTA-window loops
// (all of which already poll in a delay() loop), so a rapid open-then-close
// that happens while the device is busy on the network -- not just the
// transition that caused this wake -- gets caught, at no extra awake-time
// cost since it rides on delays the cycle is already spending.
//
// Deliberately does NOT publish anything itself: publishQos1() tracks its
// one in-flight PUBACK in a single shared variable (see its own comment),
// assuming only one publish is ever outstanding at a time. This function is
// called from inside publishQos1()'s own wait loop, so publishing here
// would nest a second publish inside the first and race that tracking,
// potentially causing the outer publish to miss its own ack. Instead,
// callers publish the up-to-date currentDoorOpen themselves at safe,
// non-nested points -- see publishState()'s caller and the final check
// right before sleep in setup().
void checkDoorPin() {
  bool open = (digitalRead(REED_PIN) == HIGH);
  if (open == currentDoorOpen) return; // fast path: no change, cheap single read

  // Candidate transition -- hand off to the same multi-sample debounce used
  // at wake, rather than trusting a single confirm-read. A brief single
  // glitch (switch bounce, or RF pickup from the WiFi radio's own TX
  // bursts landing on this GPIO) can otherwise read as a real transition.
  open = readStableDoorOpen();
  if (open == currentDoorOpen) return; // settled back to where it started -- was noise

  currentDoorOpen = open;
  Serial.printf("[door] mid-cycle transition detected: now %s\n", open ? "OPEN" : "CLOSED");
}

// Requires DOOR_DEBOUNCE_SAMPLES consecutive agreeing reads (each
// DEBOUNCE_SETTLE_MS apart, restarting the streak whenever a read disagrees)
// before trusting the reed switch's level. A single confirm-read isn't a
// strong enough filter against switch bounce or RF pickup from the WiFi
// radio's own TX bursts landing on this GPIO -- both can hold a wrong level
// for longer than one sample interval. Falls back to whatever the last
// sample was if the line never fully settles within DOOR_DEBOUNCE_MAX_ATTEMPTS,
// rather than hanging indefinitely on a genuinely noisy line.
bool readStableDoorOpen() {
  bool candidate = (digitalRead(REED_PIN) == HIGH);
  int agreeCount = 1;
  int attempts = 1;
  while (agreeCount < DOOR_DEBOUNCE_SAMPLES && attempts < DOOR_DEBOUNCE_MAX_ATTEMPTS) {
    delay(DEBOUNCE_SETTLE_MS);
    bool sample = (digitalRead(REED_PIN) == HIGH);
    attempts++;
    if (sample == candidate) {
      agreeCount++;
    } else {
      candidate = sample;
      agreeCount = 1;
    }
  }
  if (agreeCount < DOOR_DEBOUNCE_SAMPLES) {
    Serial.println("[door] pin never settled during debounce -- using last sample.");
  }
  return candidate;
}

// Syncs the system clock to local time (needed only so
// updateAndGetLastFullChargeDate() can record a real calendar date). The
// system clock survives deep sleep once synced, so this only needs to run
// occasionally to correct drift, not on every wake -- called after MQTT is
// up, and deliberately last in the connected branch so a slow/failed NTP
// round trip can only cost date accuracy, never delay or risk the
// door-state/battery publish.
void syncLocalTimeIfDue() {
  bool dueForResync = !g_timeSynced || (bootCount % NTP_RESYNC_EVERY_N_BOOTS == 0);
  if (!dueForResync) return;

  configTzTime(TZ_STRING, NTP_SERVER);
  struct tm timeinfo;
  if (getLocalTime(&timeinfo, NTP_SYNC_TIMEOUT_MS)) {
    g_timeSynced = true;
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
    Serial.printf("[NTP] Local time synced: %s\n", buf);
  } else {
    Serial.println("[NTP] Sync failed this cycle.");
  }
}

// ---------------- OTA ----------------

void runOtaWindow() {
  unsigned long start = millis();
  while (millis() - start < OTA_WINDOW_MS) {
    ArduinoOTA.handle();
    delay(10);
    checkDoorPin();
  }
}

// ---------------- Network debug session ----------------

// Entered from the connected cycle in setup() when settings.debugMode is
// true (see the apply-if-changed-then-echo block there). Keeps the device
// fully awake -- no deep sleep -- for up to DEBUG_SESSION_TIMEOUT_MS,
// polling the reed switch tightly and publishing/logging every transition
// immediately, so door-pin debounce behavior (see checkDoorPin() /
// readStableDoorOpen()) can be watched live over a plain TCP connection
// instead of USB Serial, which isn't reachable once this device is
// mounted on the door.
//
// Deliberately does NOT attempt an MQTT reconnect if the connection drops
// mid-session (a 30-minute continuously-awake window is long enough that
// a WiFi hiccup is plausible) -- the local telnet feed keeps working
// either way, just without live HA updates until the next normal wake.
// Also doesn't start ArduinoOTA here; use the existing OTA Request switch
// on a separate wake if firmware needs pushing.
void runDebugSession() {
  startAwakeWatchdog(DEBUG_SESSION_TIMEOUT_MS + 30000); // extend to cover the whole session, plus margin
  debugServer.begin();
  Serial.printf("[debug] Debug Mode active -- connect with: telnet %s\n", WiFi.localIP().toString().c_str());
  Serial.println("[debug] Auto-expires in ~30 min, or flip the Debug Mode switch off.");

  unsigned long sessionStart = millis();
  while (millis() - sessionStart < DEBUG_SESSION_TIMEOUT_MS) {
    if (debugServer.hasClient()) {
      if (debugClient.connected()) debugClient.stop();
      debugClient = debugServer.available(); // .available() here returns the pending client, not a byte count -- WiFiServer's own override, not Stream's
      Serial.println("[debug] Network client connected.");
    }

    bool before = currentDoorOpen;
    checkDoorPin(); // same debounce/logging as the normal cycle -- this is the actual thing being watched
    if (currentDoorOpen != before) {
      publishQos1(TOPIC_STATE, currentDoorOpen ? "OPEN" : "CLOSED", true);
    }

    // Re-check for a live toggle-off mid-session -- onMqttMessage() fires
    // from the library's own background task regardless of what this loop
    // is doing, so this picks up a fresh command without re-subscribing.
    if (g_debugModeCmdReceived && strcmp(g_debugModeCmdPayload, "OFF") == 0) {
      Serial.println("[debug] Debug Mode switched off via MQTT -- ending session early.");
      break;
    }
    delay(20);
  }

  settings.debugMode = false;
  saveSettings();
  publishQos1(TOPIC_DEBUG_MODE_STATE, "OFF", true);
  if (debugClient) debugClient.stop();
  debugServer.end();
  Serial.println("[debug] Debug session ended -- resuming normal sleep cycle.");
}

// ---------------- Sleep ----------------

void armWakeup(int currentPinLevel) {
  // The ESP32-C3 has no ext0/ext1 RTC GPIO block (that's only on the
  // original ESP32 and S2/S3). Instead it uses esp_deep_sleep_enable_gpio_wakeup,
  // which works on any GPIO since the C3 doesn't have a separate RTC IO domain.
  //
  // Wake on the OPPOSITE level from the current reading — this is the
  // "invert" trick: whichever state the door is in now, wake on the
  // next transition away from it.
  //
  // NOTE: this function auto-configures an internal pull resistor biased
  // AWAY from the trigger level, which fights the reed switch's floating
  // "open" state -- that's why there's an external 10k pull-up on GPIO4
  // (see wiring note at the top of this file).
  uint64_t pinMask = 1ULL << REED_PIN;
  esp_deepsleep_gpio_wake_up_mode_t wakeMode =
      (currentPinLevel == HIGH) ? ESP_GPIO_WAKEUP_GPIO_LOW : ESP_GPIO_WAKEUP_GPIO_HIGH;
  esp_deep_sleep_enable_gpio_wakeup(pinMask, wakeMode);

  // Also wake periodically regardless of door state, as a heartbeat
  esp_sleep_enable_timer_wakeup(HEARTBEAT_INTERVAL_US);
}

void goToSleep() {
  Serial.println("Going to sleep...");
  Serial.flush();
  esp_deep_sleep_start();
}
