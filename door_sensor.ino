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
 *   - Publishes state, battery voltage/percent, and WiFi signal at QoS 1,
 *     waiting for PUBACK confirmation with up to 3 retry attempts per
 *     message, then goes back to sleep
 *   - OTA updates are triggered from software (an MQTT switch in HA), not a
 *     physical button -- GPIO0 is already used for the battery ADC here.
 *     Flipping the switch sets a retained MQTT flag; the device only checks
 *     for it on its next natural wake (door event or heartbeat), so there's
 *     no way to "wake it early" the way a boot button would.
 *
 * Libraries required (Library Manager):
 *   - espMqttClient (Bert Melis)
 *   - ArduinoOTA (bundled with the ESP32 core)
 */

#include <WiFi.h>
#include <espMqttClient.h>
#include <ArduinoOTA.h>
#include "esp_sleep.h"

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

// Home Assistant MQTT discovery topics
String DISCOVERY_DOOR    = String("homeassistant/binary_sensor/") + DEVICE_ID + "/door/config";
String DISCOVERY_BATTERY = String("homeassistant/sensor/") + DEVICE_ID + "/battery/config";
String DISCOVERY_BATTERY_PCT = String("homeassistant/sensor/") + DEVICE_ID + "/battery_percent/config";
String DISCOVERY_RSSI    = String("homeassistant/sensor/") + DEVICE_ID + "/wifi_signal/config";
String DISCOVERY_OTA     = String("homeassistant/switch/") + DEVICE_ID + "/ota/config";

// ---------------- Persisted state (survives deep sleep) ----------------

RTC_DATA_ATTR bool discoverySent = false;
RTC_DATA_ATTR int lastDoorState  = -1; // -1 = unknown, 0 = closed, 1 = open
RTC_DATA_ATTR uint32_t bootCount = 0;

// ---------------- Globals ----------------

espMqttClient mqttClient; // uses its own background task on ESP32 -- no manual loop() needed

// Set by the onConnect/onPublish callbacks, which fire from the client's
// background task. Polled from the main setup()/loop() flow below.
volatile bool mqttConnectedFlag = false;
volatile uint16_t lastAckedPacketId = 0;
volatile bool otaRequested = false;

// ---------------- Function declarations ----------------

void connectWiFi();
void initMqttClient();
bool connectMQTT();
bool publishQos1(const String& topic, const String& payload, bool retain);
void sendDiscoveryConfig();
void publishState(bool doorOpen, float batteryVoltage, float batteryPercent, int rssi);
void armWakeup(int currentPinLevel);
void goToSleep();
float batteryPercentage(float v);
void onMqttConnect(bool sessionPresent);
void onMqttDisconnect(espMqttClientTypes::DisconnectReason reason);
void onMqttPublish(uint16_t packetId);
void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic,
                    const uint8_t* payload, size_t len, size_t index, size_t total);
void runOtaWindow();

// ---------------- Setup / main flow ----------------

void setup() {
  Serial.begin(115200);

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
  delay(DEBOUNCE_SETTLE_MS); // let the contact settle before reading

  int pinLevel = digitalRead(REED_PIN);
  bool doorOpen = (pinLevel == HIGH); // circuit broken (no magnet) = open

  esp_sleep_wakeup_cause_t wakeupCause = esp_sleep_get_wakeup_cause();
  Serial.printf("Boot #%lu, wakeup cause: %d, door pin level: %d\n",
                bootCount, wakeupCause, pinLevel);

  // analogReadMilliVolts() uses the ESP32's factory ADC calibration (eFuse)
  // for an accurate mV reading -- far more accurate than manually mapping
  // raw analogRead() counts against an assumed 3.3V reference, which the
  // ADC doesn't actually use internally.
  uint32_t rawMillivolts = analogReadMilliVolts(BATT_PIN);
  float batteryVoltage = (rawMillivolts / 1000.0f) * BATT_DIVIDER_RATIO;
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
      publishState(doorOpen, batteryVoltage, batteryPercent, rssi);

      if (otaRequested) {
        Serial.println("OTA requested via MQTT switch -- entering OTA window.");
        // Clear the retained command immediately so we don't re-trigger on
        // every subsequent wake, and reflect the reset back to the HA UI.
        publishQos1(TOPIC_OTA_CMD, "OFF", true);
        publishQos1(TOPIC_OTA_STATE, "OFF", true);

        ArduinoOTA.setHostname(DEVICE_ID);
        ArduinoOTA.setPassword(OTA_PASSWORD);
        ArduinoOTA.begin();
        Serial.printf("OTA ready, staying awake for up to %lu ms...\n", OTA_WINDOW_MS);
        runOtaWindow();
        Serial.println("OTA window elapsed, resuming normal cycle.");
      }
    } else {
      Serial.println("MQTT connect failed after all attempts, skipping publish this cycle.");
    }
  } else {
    Serial.println("WiFi connect failed, skipping publish this cycle.");
  }

  lastDoorState = doorOpen ? 1 : 0;

  if (DEBUG_MODE) {
    // Stay connected in debug mode so MQTT stays "online" and we can watch
    // live door toggles propagate to HA without triggering the Last Will.
    Serial.println("=== DEBUG_MODE: staying connected, entering loop() ===");
    return;
  }

  // Clean (non-forced) disconnect: the library sends any remaining queued
  // messages before closing the connection. Give it a moment to actually
  // complete before tearing down WiFi, or the broker sees an abrupt drop
  // and fires the Last Will (marking the device "offline") anyway.
  mqttClient.disconnect();
  delay(300);
  WiFi.disconnect(true);

  armWakeup(pinLevel);
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

void connectWiFi() {
  WiFi.mode(WIFI_STA);

  if (USE_STATIC_IP) {
    WiFi.config(STATIC_IP, GATEWAY, SUBNET, DNS_SERVER);
  }

  bool useBSSID = false;
  for (int i = 0; i < 6; i++) {
    if (WIFI_BSSID[i] != 0x00) { useBSSID = true; break; }
  }

  if (useBSSID) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD, 0, WIFI_BSSID);
  } else {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }

  unsigned long start = millis();
  wl_status_t lastStatus = WiFi.status();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(50);
    wl_status_t s = WiFi.status();
    if (s != lastStatus) {
      Serial.printf("[debug] WiFi status changed: %d\n", s);
      lastStatus = s;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("WiFi connected in %lums, IP: %s\n",
                  millis() - start, WiFi.localIP().toString().c_str());
  } else {
    Serial.printf("[debug] Final WiFi status: %d (0=IDLE,1=NO_SSID,3=CONNECTED,4=CONNECT_FAILED,6=DISCONNECTED)\n",
                  WiFi.status());
  }
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
  if (TOPIC_OTA_CMD.equals(topic)) {
    // Payloads aren't null-terminated -- compare the raw bytes directly.
    if (len == 2 && payload[0] == 'O' && payload[1] == 'N') {
      otaRequested = true;
    }
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
    }

    if (mqttConnectedFlag) {
      Serial.printf("MQTT connected in %lums (attempt %lu)\n", millis() - start, attempt);
      publishQos1(TOPIC_AVAILABILITY, "online", true);

      // Subscribe and give the broker a moment to deliver any retained OTA
      // command -- this is how a request made while we were asleep gets seen.
      mqttClient.subscribe(TOPIC_OTA_CMD.c_str(), 1);
      unsigned long subStart = millis();
      while (millis() - subStart < OTA_SUBSCRIBE_WAIT_MS) {
        delay(20);
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
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"],\"name\":\"" + DEVICE_NAME + "\",\"manufacturer\":\"" + DEVICE_MANUFACTURER + "\",\"model\":\"" + DEVICE_MODEL + "\",\"sw_version\":\"" + FIRMWARE_VERSION + "\"}"
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
    + "\"state_topic\":\"" + TOPIC_BATTERY_PCT + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_BATTERY_PCT, battPctPayload, true);

  // WiFi signal strength sensor discovery payload
  String rssiPayload = String("{")
    + "\"name\":\"" + DEVICE_NAME + " WiFi Signal\","
    + "\"unique_id\":\"" + DEVICE_ID + "_wifi_signal\","
    + "\"device_class\":\"signal_strength\","
    + "\"unit_of_measurement\":\"dBm\","
    + "\"state_class\":\"measurement\","
    + "\"entity_category\":\"diagnostic\","
    + "\"state_topic\":\"" + TOPIC_RSSI + "\","
    + "\"availability_topic\":\"" + TOPIC_AVAILABILITY + "\","
    + "\"device\":{\"identifiers\":[\"" + DEVICE_ID + "\"]}"
    + "}";
  publishQos1(DISCOVERY_RSSI, rssiPayload, true);

  // OTA trigger switch discovery payload. "retain: true" makes HA publish
  // the ON command with the retain flag set, so it survives on the broker
  // until this (sleeping) device actually wakes up and subscribes to see it.
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
}

void publishState(bool doorOpen, float batteryVoltage, float batteryPercent, int rssi) {
  publishQos1(TOPIC_STATE, doorOpen ? "OPEN" : "CLOSED", true);

  char battStr[8];
  dtostrf(batteryVoltage, 4, 2, battStr);
  publishQos1(TOPIC_BATTERY, battStr, true);

  char battPctStr[8];
  dtostrf(batteryPercent, 4, 0, battPctStr);
  publishQos1(TOPIC_BATTERY_PCT, battPctStr, true);

  char rssiStr[8];
  snprintf(rssiStr, sizeof(rssiStr), "%d", rssi);
  publishQos1(TOPIC_RSSI, rssiStr, true);

  Serial.printf("Published: door=%s, battery=%sV (%s%%), rssi=%sdBm\n",
                doorOpen ? "OPEN" : "CLOSED", battStr, battPctStr, rssiStr);
}

// ---------------- Battery ----------------

// Piecewise-linear state-of-charge curve for a typical single-cell Li-ion,
// same curve used across the other battery-powered sensors so readings are
// consistent device to device.
float batteryPercentage(float v) {
  float pct;
  if (v >= 4.15)      pct = 100.0;
  else if (v >= 4.10) pct = 95.0 + (v - 4.10) / (4.15 - 4.10) * 5.0;
  else if (v >= 4.05) pct = 90.0 + (v - 4.05) / (4.10 - 4.05) * 5.0;
  else if (v >= 4.00) pct = 85.0 + (v - 4.00) / (4.05 - 4.00) * 5.0;
  else if (v >= 3.95) pct = 80.0 + (v - 3.95) / (4.00 - 3.95) * 5.0;
  else if (v >= 3.90) pct = 75.0 + (v - 3.90) / (3.95 - 3.90) * 5.0;
  else if (v >= 3.83) pct = 70.0 + (v - 3.83) / (3.90 - 3.83) * 5.0;
  else if (v >= 3.75) pct = 65.0 + (v - 3.75) / (3.83 - 3.75) * 5.0;
  else if (v >= 3.71) pct = 60.0 + (v - 3.71) / (3.75 - 3.71) * 5.0;
  else if (v >= 3.68) pct = 55.0 + (v - 3.68) / (3.71 - 3.68) * 5.0;
  else if (v >= 3.64) pct = 50.0 + (v - 3.64) / (3.68 - 3.64) * 5.0;
  else if (v >= 3.60) pct = 45.0 + (v - 3.60) / (3.64 - 3.60) * 5.0;
  else if (v >= 3.56) pct = 40.0 + (v - 3.56) / (3.60 - 3.56) * 5.0;
  else if (v >= 3.50) pct = 35.0 + (v - 3.50) / (3.56 - 3.50) * 5.0;
  else if (v >= 3.45) pct = 30.0 + (v - 3.45) / (3.50 - 3.45) * 5.0;
  else if (v >= 3.40) pct = 25.0 + (v - 3.40) / (3.45 - 3.40) * 5.0;
  else if (v >= 3.35) pct = 20.0 + (v - 3.35) / (3.40 - 3.35) * 5.0;
  else if (v >= 3.25) pct = 15.0 + (v - 3.25) / (3.35 - 3.25) * 5.0;
  else if (v >= 3.15) pct = 10.0 + (v - 3.15) / (3.25 - 3.15) * 5.0;
  else if (v >= 3.10) pct =  5.0 + (v - 3.10) / (3.15 - 3.10) * 5.0;
  else pct = 0.0;
  return roundf(pct);
}

// ---------------- OTA ----------------

void runOtaWindow() {
  unsigned long start = millis();
  while (millis() - start < OTA_WINDOW_MS) {
    ArduinoOTA.handle();
    delay(10);
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
