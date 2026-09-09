/*
 * ESP32-C3 Zero Battery-Powered Door Sensor
 * Reed switch -> MQTT (QoS 1, confirmed) -> Home Assistant (via MQTT discovery)
 *
 * Wiring:
 *   Reed switch: one leg -> GPIO4, other leg -> GND (+ external 10k pull-up
 *     from GPIO4 to 3.3V -- needed for reliable wake on the closed->open
 *     transition, see notes in armWakeup())
 *   Battery voltage divider: midpoint -> GPIO1 (ADC)
 *
 * Behavior:
 *   - Sleeps until the door opens or closes (wake on level change)
 *   - Also wakes periodically as a heartbeat / battery report
 *   - Publishes state, battery voltage/percent/low-flag, WiFi signal, and
 *     boot/fail-count diagnostics at QoS 1, waiting for PUBACK confirmation
 *     with up to 3 retry attempts per message, then goes back to sleep
 *   - OTA updates are triggered from software (an MQTT switch in HA), not a
 *     physical button -- GPIO0 is already used for the battery ADC here.
 *     Flipping the switch sets a retained MQTT flag; the device only checks
 *     for it on its next natural wake (door event or heartbeat), so there's
 *     no way to "wake it early" the way a boot button would.
 *   - Caches the last successful WiFi channel across sleeps (RTC memory) to
 *     skip most of the scan on the next wake, falling back to a full scan
 *     if the cached channel fails
 *   - A hardware watchdog force-restarts the device if it's ever awake too
 *     long (stuck library call, unexpected hang), independent of everything
 *     else in the sketch
 *   - HA discovery configs carry an expire_after so a dead device eventually
 *     shows "unavailable" instead of a door state frozen forever
 *   - Tracks open/close counts for the current local day, reset to 0 in
 *     firmware the first wake after local midnight (needs an occasional NTP
 *     sync -- see syncLocalTimeIfDue() -- since this device has no RTC
 *     backup battery of its own)
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
 *
 * Libraries required (Library Manager):
 *   - espMqttClient (Bert Melis)
 *   - ArduinoOTA (bundled with the ESP32 core)
 */

#include <WiFi.h>
#include <espMqttClient.h>
#include <ArduinoOTA.h>
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_timer.h"
#include <time.h>
#include <sys/time.h>

// ---------------- Config ----------------
// WiFi/MQTT/OTA credentials, static IP, device identity, hardware pins, and
// timing constants live in config.h (same folder — the Arduino IDE shows it
// as a second tab) so you can keep that one file per-device without
// touching the rest of this sketch. config.h is gitignored; copy
// config.h.example to config.h and fill in your real values.
#include "config.h"

// MQTT topics
String TOPIC_STATE       = String("home/") + DEVICE_ID + "/door";
String TOPIC_BATTERY     = String("home/") + DEVICE_ID + "/battery";
String TOPIC_BATTERY_PCT = String("home/") + DEVICE_ID + "/battery_percent";
String TOPIC_RSSI        = String("home/") + DEVICE_ID + "/wifi_signal";
String TOPIC_AVAILABILITY= String("home/") + DEVICE_ID + "/status";
String TOPIC_OTA_CMD     = String("home/") + DEVICE_ID + "/ota/set";
String TOPIC_OTA_STATE   = String("home/") + DEVICE_ID + "/ota/state";
String TOPIC_BATTERY_LOW = String("home/") + DEVICE_ID + "/battery_low";
String TOPIC_BOOT_COUNT  = String("home/") + DEVICE_ID + "/boot_count";
String TOPIC_FAIL_COUNT  = String("home/") + DEVICE_ID + "/connect_fail_count";
String TOPIC_TOTAL_FAIL_COUNT = String("home/") + DEVICE_ID + "/total_fail_count";
String TOPIC_OPEN_COUNT_TODAY  = String("home/") + DEVICE_ID + "/open_count_today";
String TOPIC_CLOSE_COUNT_TODAY = String("home/") + DEVICE_ID + "/close_count_today";
String TOPIC_BOOT_RESET_CMD    = String("home/") + DEVICE_ID + "/boot_count_reset/set";
String TOPIC_BOOT_RESET_STATE  = String("home/") + DEVICE_ID + "/boot_count_reset/state";
String TOPIC_OTA_ACTIVE = String("home/") + DEVICE_ID + "/ota_active";

// Home Assistant MQTT discovery topics
String DISCOVERY_DOOR    = String("homeassistant/binary_sensor/") + DEVICE_ID + "/door/config";
String DISCOVERY_BATTERY = String("homeassistant/sensor/") + DEVICE_ID + "/battery/config";
String DISCOVERY_BATTERY_PCT = String("homeassistant/sensor/") + DEVICE_ID + "/battery_percent/config";
String DISCOVERY_RSSI    = String("homeassistant/sensor/") + DEVICE_ID + "/wifi_signal/config";
String DISCOVERY_OTA     = String("homeassistant/switch/") + DEVICE_ID + "/ota/config";
String DISCOVERY_BATTERY_LOW = String("homeassistant/binary_sensor/") + DEVICE_ID + "/battery_low/config";
String DISCOVERY_BOOT_COUNT  = String("homeassistant/sensor/") + DEVICE_ID + "/boot_count/config";
String DISCOVERY_FAIL_COUNT  = String("homeassistant/sensor/") + DEVICE_ID + "/connect_fail_count/config";
String DISCOVERY_TOTAL_FAIL_COUNT = String("homeassistant/sensor/") + DEVICE_ID + "/total_fail_count/config";
String DISCOVERY_OPEN_COUNT_TODAY  = String("homeassistant/sensor/") + DEVICE_ID + "/open_count_today/config";
String DISCOVERY_CLOSE_COUNT_TODAY = String("homeassistant/sensor/") + DEVICE_ID + "/close_count_today/config";
String DISCOVERY_BOOT_RESET = String("homeassistant/switch/") + DEVICE_ID + "/boot_count_reset/config";
String DISCOVERY_OTA_ACTIVE = String("homeassistant/binary_sensor/") + DEVICE_ID + "/ota_active/config";

// ---------------- Persisted state (survives deep sleep) ----------------

RTC_DATA_ATTR bool discoverySent = false;
RTC_DATA_ATTR uint32_t bootCount = 0;
RTC_DATA_ATTR uint32_t connectFailCount = 0; // increments on any wake that fails to publish, resets on success
RTC_DATA_ATTR uint32_t totalFailCount = 0;   // lifetime total failed wakes -- never resets, mirrors bootCount
RTC_DATA_ATTR uint8_t cachedWifiChannel = 0; // 0 = unknown yet, let WiFi.begin() auto-select
RTC_DATA_ATTR bool g_timeSynced = false;     // true once any cycle has completed a real NTP sync
RTC_DATA_ATTR int32_t lastCounterDay = -1;   // -1 = unknown yet; YYYYMMDD of the day the counters below are for
RTC_DATA_ATTR uint32_t openCountToday = 0;
RTC_DATA_ATTR uint32_t closeCountToday = 0;

// ---------------- Globals ----------------

espMqttClient mqttClient; // uses its own background task on ESP32 -- no manual loop() needed

// Set by the onConnect/onPublish callbacks, which fire from the client's
// background task. Polled from the main setup()/loop() flow below.
volatile bool mqttConnectedFlag = false;
volatile uint16_t lastAckedPacketId = 0;
volatile bool otaRequested = false;
volatile bool bootCountResetRequested = false;

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
bool attemptWifiConnect(uint8_t channel, bool useBSSID);
void initMqttClient();
bool connectMQTT();
bool publishQos1(const String& topic, const String& payload, bool retain);
void sendDiscoveryConfig();
int publishState(bool doorOpen, float batteryVoltage, float batteryPercent, int rssi, uint32_t failCount);
void armWakeup(int currentPinLevel);
void goToSleep();
float batteryPercentage(float v);
float calibrateBatteryVoltage(float raw);
String wakeupCauseToString(esp_sleep_wakeup_cause_t cause);
void updateDailyCounters(bool doorOpen, esp_sleep_wakeup_cause_t wakeupCause);
void syncLocalTimeIfDue();
void checkDoorPin();
bool readStableDoorOpen();
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason);
void onMqttPublish(uint16_t packetId);
void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic,
                    const uint8_t* payload, size_t len, size_t index, size_t total);
void runOtaWindow();

// ---------------- Setup / main flow ----------------

void setup() {
  Serial.begin(115200);
  startAwakeWatchdog(AWAKE_WATCHDOG_TIMEOUT_MS); // armed immediately -- extended later if OTA mode is entered

  if (DEBUG_MODE) {
    // Give native USB CDC time to enumerate and give you time to open
    // Serial Monitor before the board does anything else.
    delay(DEBUG_BOOT_DELAY_MS);
    Serial.println("=== DEBUG_MODE is ON: deep sleep disabled, staying awake ===");
  } else {
    delay(100);
  }

  bootCount++;

  pinMode(REED_PIN, INPUT_PULLUP);
  delay(DEBOUNCE_SETTLE_MS); // let the pin electrically settle after enabling the pull-up

  bool doorOpen = readStableDoorOpen(); // multi-sample debounce -- see its own comment for why
  currentDoorOpen = doorOpen; // kept fresh by checkDoorPin() as this cycle progresses
  bool lastReportedDoorOpen = doorOpen; // what MQTT was last actually told, for the final catch-up check

  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  Serial.printf("Boot #%lu, wakeup cause: %s (%d), door: %s\n",
                bootCount, wakeupCauseToString(wakeupCause).c_str(), wakeupCause,
                doorOpen ? "OPEN" : "CLOSED");

  // Uses whatever the system clock already holds (it survives deep sleep
  // once synced -- see syncLocalTimeIfDue()) rather than requiring a fresh
  // network round trip on every wake just to check the date.
  updateDailyCounters(doorOpen, wakeupCause);

  // analogReadMilliVolts() uses the ESP32's factory ADC calibration (eFuse)
  // for an accurate mV reading -- far more accurate than manually mapping
  // raw analogRead() counts against an assumed 3.3V reference, which the
  // ADC doesn't actually use internally.
  uint32_t rawMillivolts = analogReadMilliVolts(BATT_PIN);
  float rawVoltage = (rawMillivolts / 1000.0f) * BATT_DIVIDER_RATIO;
  float batteryVoltage = calibrateBatteryVoltage(rawVoltage);
  float batteryPercent = batteryPercentage(batteryVoltage);

  connectWiFi();

  int rssi = 0;

  if (WiFi.status() == WL_CONNECTED) {
    rssi = WiFi.RSSI();
    if (connectMQTT()) {
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

      int failedTopics = publishState(currentDoorOpen, batteryVoltage, batteryPercent, rssi, connectFailCount);
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

      if (otaRequested) {
        Serial.println("OTA requested via MQTT switch -- entering OTA window.");
        // Clear the retained command immediately so we don't re-trigger on
        // every subsequent wake, and reflect the reset back to the HA UI.
        publishQos1(TOPIC_OTA_CMD, "OFF", true);
        publishQos1(TOPIC_OTA_STATE, "OFF", true);

        startAwakeWatchdog(OTA_WINDOW_MS + 30000); // OTA legitimately needs to stay awake this long
        publishQos1(TOPIC_OTA_ACTIVE, "ON", true); // visible in HA even though this device has no status LED
        ArduinoOTA.setHostname(DEVICE_ID);
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.begin();
        Serial.printf("OTA ready, staying awake for up to %lu ms...\n", OTA_WINDOW_MS);
        runOtaWindow();
        publishQos1(TOPIC_OTA_ACTIVE, "OFF", true);
        Serial.println("OTA window elapsed, resuming normal cycle.");
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

  // Final check right before tearing down the connection -- closes the
  // residual gap between the last poll during publishing and now, plus
  // catches any transition that checkDoorPin() saw (and counted) during
  // publishState()'s own publish calls but couldn't publish itself (see
  // checkDoorPin()'s comment on why). This call site is safe: nothing else
  // is waiting on a PUBACK right now, so publishing here can't race it.
  checkDoorPin();
  if (mqttClient.connected() && currentDoorOpen != lastReportedDoorOpen) {
    Serial.printf("[door] publishing corrected final state before sleep: %s\n",
                  currentDoorOpen ? "OPEN" : "CLOSED");
    publishQos1(TOPIC_STATE, currentDoorOpen ? "OPEN" : "CLOSED", true);
    lastReportedDoorOpen = currentDoorOpen;
  }

  // Clean (non-forced) disconnect: the library sends any remaining queued
  // messages before closing the connection. Give it a moment to actually
  // complete before tearing down WiFi, or the broker sees an abrupt drop
  // and fires the Last Will (marking the device "offline") anyway.
  mqttClient.disconnect();
  delay(300);
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
bool attemptWifiConnect(uint8_t channel, bool useBSSID) {
  if (useBSSID) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, channel, WIFI_BSSID);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, channel);
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

  if (USE_STATIC_IP) {
    WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS_SERVER);
  }

  bool useBSSID = false;
  for (int i = 0; i < 6; i++) {
    if (WIFI_BSSID[i] != 0x00) { useBSSID = true; break; }
  }

  // Reusing the channel from the last successful connect skips a small
  // amount of scan/negotiation time -- worth it here since this device
  // wakes far more often than a timer-only sensor (every door open/close,
  // plus the heartbeat). cachedWifiChannel is 0 (auto) until the first
  // successful connect populates it below.
  bool connected = attemptWifiConnect(cachedWifiChannel, useBSSID);

  // If the cached channel attempt failed (e.g. the router switched channels
  // since we last connected), fall back to one auto-scan retry before
  // giving up on this cycle entirely.
  if (!connected && cachedWifiChannel != 0) {
    Serial.println("[debug] Cached-channel connect failed, retrying with auto channel scan...");
    WiFi.disconnect();
    delay(100);
    connected = attemptWifiConnect(0, useBSSID);
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
  }
}

void initMqttClient() {
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCredentials(MQTT_USER, MQTT_PASSWORD);
  mqttClient.setClientId(DEVICE_ID);
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
    + "\"name\":\"" + DEVICE_NAME + "\","
    + "\"unique_id\":\"" + DEVICE_ID + "_door\","
    + "\"device_class\":\"door\","
    + "\"state_topic\":\"" + TOPIC_STATE + "\","
    + "\"payload_on\":\"OPEN\","
    + "\"payload_off\":\"CLOSED\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"],\"name\":\"" + DEVICE_NAME + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\",\"model\":\"" + DEVICE_MODEL + "\",\"sw_version\":\"" + FIRMWARE_VERSION + "\",\"hw_version\":\"" + DEVICE_HW_VERSION + "\"}"
    + "}";
  bool doorOk = publishQos1(DISCOVERY_DOOR, doorPayload, true);
  Serial.printf("[debug] Door discovery publish: %s\n", doorOk ? "OK" : "FAILED");

  // Battery voltage sensor discovery payload
  String battPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " Battery Voltage\","
    + "\"unique_id\":\"" + DEVICE_ID + "_battery\","
    + "\"device_class\":\"voltage\","
    + "\"unit_of_measurement\":\"V\","
    + "\"state_class\":\"measurement\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_BATTERY + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  bool battOk = publishQos1(DISCOVERY_BATTERY, battPayload, true);
  Serial.printf("[debug] Battery discovery publish: %s\n", battOk ? "OK" : "FAILED");

  // Battery percentage sensor discovery payload (device_class: battery gives it the standard battery icon in HA)
  String battPctPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " Battery\","
    + "\"unique_id\":\"" + DEVICE_ID + "_battery_percent\","
    + "\"device_class\":\"battery\","
    + "\"unit_of_measurement\":\"%\","
    + "\"state_class\":\"measurement\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_BATTERY_PCT + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BATTERY_PCT, battPctPayload, true);

  // Low-battery binary sensor discovery payload
  String battLowPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " Low Battery\","
    + "\"unique_id\":\"" + DEVICE_ID + "_battery_low\","
    + "\"device_class\":\"battery\","
    + "\"entity_category\":\"diagnostic\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_BATTERY_LOW + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BATTERY_LOW, battLowPayload, true);

  // WiFi signal strength sensor discovery payload
  String rssiPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " WiFi Signal\","
    + "\"unique_id\":\"" + DEVICE_ID + "_wifi_signal\","
    + "\"device_class\":\"signal_strength\","
    + "\"unit_of_measurement\":\"dBm\","
    + "\"state_class\":\"measurement\","
    + "\"entity_category\":\"diagnostic\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_RSSI + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_RSSI, rssiPayload, true);

  // Daily open/close counters -- reset to 0 in firmware at local midnight
  // (see updateDailyCounters()), so total_increasing here matches the same
  // "periodically-resetting counter" convention energy_meter uses for its
  // day/night tariff accumulators.
  String openCountPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " Open Count Today\","
    + "\"unique_id\":\"" + DEVICE_ID + "_open_count_today\","
    + "\"state_class\":\"total_increasing\","
    + "\"icon\":\"mdi:door-open\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_OPEN_COUNT_TODAY + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_OPEN_COUNT_TODAY, openCountPayload, true);

  String closeCountPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " Close Count Today\","
    + "\"unique_id\":\"" + DEVICE_ID + "_close_count_today\","
    + "\"state_class\":\"total_increasing\","
    + "\"icon\":\"mdi:door-closed\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_CLOSE_COUNT_TODAY + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_CLOSE_COUNT_TODAY, closeCountPayload, true);

  // Boot count sensor discovery payload (diagnostic -- total wakes since last full reset)
  String bootCountPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " Boot Count\","
    + "\"unique_id\":\"" + DEVICE_ID + "_boot_count\","
    + "\"entity_category\":\"diagnostic\","
    + "\"state_class\":\"total_increasing\","
    + "\"icon\":\"mdi:counter\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_BOOT_COUNT + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BOOT_COUNT, bootCountPayload, true);

  // Consecutive connect-fail count (resets to 0 on the next fully successful wake)
  String failCountPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " Connect Fail Count\","
    + "\"unique_id\":\"" + DEVICE_ID + "_connect_fail_count\","
    + "\"entity_category\":\"diagnostic\","
    + "\"state_class\":\"measurement\","
    + "\"icon\":\"mdi:wifi-alert\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_FAIL_COUNT + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_FAIL_COUNT, failCountPayload, true);

  // Lifetime total failed-wake count (never resets)
  String totalFailCountPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " Total Fail Count\","
    + "\"unique_id\":\"" + DEVICE_ID + "_total_fail_count\","
    + "\"entity_category\":\"diagnostic\","
    + "\"state_class\":\"total_increasing\","
    + "\"icon\":\"mdi:counter\","
    + "\"expire_after\":" + String(EXPIRE_AFTER_SEC) + ","
    + "\"state_topic\":\"" + TOPIC_TOTAL_FAIL_COUNT + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_TOTAL_FAIL_COUNT, totalFailCountPayload, true);

  // OTA trigger switch discovery payload. "retain: true" makes HA publish
  // the ON command with the retain flag set, so it survives on the broker
  // until this (sleeping) device actually wakes up and subscribes to see it.
  // Note: no expire_after here -- this is a control, not a reading, and
  // should stay usable in the UI even if the device has been quiet a while.
  String otaPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " OTA Update\","
    + "\"unique_id\":\"" + DEVICE_ID + "_ota\","
    + "\"command_topic\":\"" + TOPIC_OTA_CMD + "\","
    + "\"state_topic\":\"" + TOPIC_OTA_STATE + "\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"retain\":true,"
    + "\"entity_category\":\"config\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
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
    + "\"name\":\"" + DEVICE_NAME + " Reset Boot Counter\","
    + "\"unique_id\":\"" + DEVICE_ID + "_boot_count_reset\","
    + "\"command_topic\":\"" + TOPIC_BOOT_RESET_CMD + "\","
    + "\"state_topic\":\"" + TOPIC_BOOT_RESET_STATE + "\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"retain\":true,"
    + "\"icon\":\"mdi:restore\","
    + "\"entity_category\":\"config\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BOOT_RESET, bootResetPayload, true);

  // OTA-active indicator. This device has no status LED (unlike the other
  // battery sensors in this fleet) to show OTA is in progress, so this is
  // the only indication of it -- ON only for the ~OTA_WINDOW_MS the device
  // stays awake listening for a flash, OFF the rest of the time.
  String otaActivePayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " OTA Active\","
    + "\"unique_id\":\"" + DEVICE_ID + "_ota_active\","
    + "\"entity_category\":\"diagnostic\","
    + "\"icon\":\"mdi:upload\","
    + "\"payload_on\":\"ON\","
    + "\"payload_off\":\"OFF\","
    + "\"state_topic\":\"" + TOPIC_OTA_ACTIVE + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_OTA_ACTIVE, otaActivePayload, true);
}

// Returns the number of topics that never got a PUBACK this cycle (0 = fully
// successful). failCount is this device's connect-fail count as of the
// START of this cycle (i.e. not yet updated for this cycle's own outcome) --
// the caller updates it afterwards based on the return value, same pattern
// as temp_humidity_sensor.
int publishState(bool doorOpen, float batteryVoltage, float batteryPercent, int rssi, uint32_t failCount) {
  int failed = 0;

  if (!publishQos1(TOPIC_STATE, doorOpen ? "OPEN" : "CLOSED", true)) failed++;

  char battStr[8];
  dtostrf(batteryVoltage, 4, 2, battStr);
  if (!publishQos1(TOPIC_BATTERY, battStr, true)) failed++;

  char battPctStr[8];
  dtostrf(batteryPercent, 4, 0, battPctStr);
  if (!publishQos1(TOPIC_BATTERY_PCT, battPctStr, true)) failed++;

  bool batteryLow = batteryPercent < BATTERY_LOW_THRESHOLD_PCT;
  if (!publishQos1(TOPIC_BATTERY_LOW, batteryLow ? "ON" : "OFF", true)) failed++;

  char rssiStr[8];
  snprintf(rssiStr, sizeof(rssiStr), "%d", rssi);
  if (!publishQos1(TOPIC_RSSI, rssiStr, true)) failed++;

  if (!publishQos1(TOPIC_BOOT_COUNT, String(bootCount), true)) failed++;
  if (!publishQos1(TOPIC_FAIL_COUNT, String(failCount), true)) failed++;
  if (!publishQos1(TOPIC_TOTAL_FAIL_COUNT, String(totalFailCount), true)) failed++;
  if (!publishQos1(TOPIC_OPEN_COUNT_TODAY, String(openCountToday), true)) failed++;
  if (!publishQos1(TOPIC_CLOSE_COUNT_TODAY, String(closeCountToday), true)) failed++;

  // Always OFF here -- this runs before the OTA check below. Also acts as a
  // safety net: if the device somehow died mid-OTA on a previous wake, the
  // next normal wake clears a stale retained "ON" automatically.
  if (!publishQos1(TOPIC_OTA_ACTIVE, "OFF", true)) failed++;

  Serial.printf("Published: door=%s, battery=%sV (%s%%, low=%s), rssi=%sdBm, boot=%lu, failCount=%lu, opens_today=%lu, closes_today=%lu (failed topics this cycle: %d)\n",
                doorOpen ? "OPEN" : "CLOSED", battStr, battPctStr, batteryLow ? "yes" : "no",
                rssiStr, (unsigned long)bootCount, (unsigned long)failCount,
                (unsigned long)openCountToday, (unsigned long)closeCountToday, failed);

  return failed;
}

// ---------------- Battery ----------------

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

// ---------------- Diagnostics ----------------

String wakeupCauseToString(esp_sleep_wakeup_cause_t cause) {
  switch (cause) {
    case ESP_SLEEP_WAKEUP_GPIO:      return "door_event";
    case ESP_SLEEP_WAKEUP_TIMER:     return "heartbeat";
    case ESP_SLEEP_WAKEUP_UNDEFINED: return "power_on_or_reset";
    default:                         return "other";
  }
}

// ---------------- Time / daily counters ----------------

// Resets openCountToday/closeCountToday to 0 the first time this runs on a
// new local calendar day, then (only on an actual door-transition wake, not
// the periodic heartbeat) tallies this wake's event. Reads whatever the
// system clock already holds -- no network call here, see syncLocalTimeIfDue()
// below for why. Until the very first NTP sync ever completes (g_timeSynced
// still false, i.e. this device's first-ever boot), the day is unknown, so
// this cycle's event simply isn't tallied -- a one-time, cosmetic gap.
void updateDailyCounters(bool doorOpen, esp_sleep_wakeup_cause_t wakeupCause) {
  if (!g_timeSynced) {
    Serial.println("[counters] clock not yet synced -- skipping daily open/close tally this cycle.");
    return;
  }

  time_t now = time(nullptr);
  struct tm t;
  localtime_r(&now, &t);
  int32_t today = (t.tm_year + 1900) * 10000L + (t.tm_mon + 1) * 100L + t.tm_mday;

  if (today != lastCounterDay) {
    Serial.printf("[counters] new local day (%ld -> %ld), resetting open/close counters.\n",
                  (long)lastCounterDay, (long)today);
    openCountToday = 0;
    closeCountToday = 0;
    lastCounterDay = today;
  }

  if (wakeupCause == ESP_SLEEP_WAKEUP_GPIO) {
    if (doorOpen) openCountToday++; else closeCountToday++;
  }
}

// ---------------- Door pin polling ----------------

// Re-reads the reed switch and, if it's genuinely changed since the last
// known state (currentDoorOpen), debounces it, updates currentDoorOpen, and
// tallies it into today's open/close counters. Called from inside the
// WiFi-connect, MQTT-connect, per-publish PUBACK-wait, and OTA-window loops
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
  if (open) openCountToday++; else closeCountToday++;
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

// Syncs the system clock to local time (needed only so updateDailyCounters()
// can tell when local midnight has passed). The system clock survives deep
// sleep once synced, so this only needs to run occasionally to correct
// drift, not on every wake -- called after MQTT is up, and deliberately
// last in the connected branch so a slow/failed NTP round trip can only
// cost date accuracy, never delay or risk the door-state/battery publish.
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
