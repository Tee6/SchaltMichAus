#include <Arduino.h>
#define MQTT_MAX_PACKET_SIZE 1024

#include <WiFi.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <string.h>
#include <PubSubClient.h>

// ---------------- WLAN ----------------
const char* ssid = "";
const char* password = "";

// ---------------- MQTT ----------------
const char* mqttBroker   = "192.168.178.113"; // ggf. ändern
const int   mqttPort     = 1883;
const char* mqttUser     = "esp32";
const char* mqttPassword = "esp32";

#define MQTT_PREFIX        "homeassistant"
#define DEVICE_ID          "esp32_audiocontrol"

#define TOPIC_SWITCH_CMD    DEVICE_ID "/switch/control/set"
#define TOPIC_SWITCH_STATE  DEVICE_ID "/switch/control/state"
#define TOPIC_TRIGGER_STATE DEVICE_ID "/binary_sensor/trigger/state"
#define TOPIC_MUSIC_STATE   DEVICE_ID "/binary_sensor/music/state"
#define TOPIC_IR_CMD        DEVICE_ID "/button/ir_send/set"
#define TOPIC_IR_EVENT      DEVICE_ID "/sensor/ir_event/state"

#define TOPIC_DISC_SWITCH   MQTT_PREFIX "/switch/"        DEVICE_ID "_control/config"
#define TOPIC_DISC_TRIGGER  MQTT_PREFIX "/binary_sensor/" DEVICE_ID "_trigger/config"
#define TOPIC_DISC_MUSIC    MQTT_PREFIX "/binary_sensor/" DEVICE_ID "_music/config"
#define TOPIC_DISC_IR       MQTT_PREFIX "/button/"        DEVICE_ID "_ir_send/config"
#define TOPIC_DISC_IR_EVENT MQTT_PREFIX "/sensor/"        DEVICE_ID "_ir_event/config"

// NEU: Discovery-Topics für die vier zusätzlichen Buttons
#define TOPIC_DISC_VOL_UP   MQTT_PREFIX "/button/" DEVICE_ID "_vol_up/config"
#define TOPIC_DISC_VOL_DOWN MQTT_PREFIX "/button/" DEVICE_ID "_vol_down/config"
#define TOPIC_DISC_PHONO    MQTT_PREFIX "/button/" DEVICE_ID "_in_phono/config"
#define TOPIC_DISC_AUDIO1   MQTT_PREFIX "/button/" DEVICE_ID "_in_audio1/config"

// ---------------- Pins ----------------
#define PIN_TRIGGER 32
#define PIN_IR      18
#define AUX_PIN     34

// ---------------- Audio ----------------
#define SAMPLE_COUNT      1000
const float THRESH_ON   =  8.0f;
const float THRESH_OFF  = 12.0f;
#define MUSIC_ON_CONFIRM  3
#define MUSIC_OFF_CONFIRM 6

float g_silenceMinutes = 20.0f;

// Mindest-Wartezeit nach Trigger-HIGH bevor Silence-Timeout greifen darf
const unsigned long TRIGGER_GRACE_MS = 30000UL;  // 30 Sekunden

// Schutzfenster nach MQTT-Connect: retained Messages in diesem Fenster ignorieren
const unsigned long MQTT_RETAINED_IGNORE_MS = 2000UL;

// Debounce für manuellen POWER-Press: verhindert Doppel-Toggle egal welche Ursache
const unsigned long MANUAL_IR_DEBOUNCE_MS = 3000UL;

// Kurzer Debounce für Vol/Input: nur gegen MQTT-Doppelzustellung,
// darf schnelles wiederholtes Drücken (Lautstärke!) NICHT blockieren
const unsigned long SHORT_IR_DEBOUNCE_MS = 250UL;

// ---------------- Timing ----------------
const unsigned long MUSIC_CHECK_MS        = 3000UL;
const unsigned long STATE_PRINT_MS        = 5000UL;
const unsigned long IR_INTERVAL_MS        = 2000UL;
const unsigned long MQTT_RECONNECT_MS     = 10000UL;
const unsigned long WIFI_RECONNECT_MS     = 15000UL;
const unsigned long DISCOVERY_INTERVAL_MS = 300000UL;

// ---------------- Yamaha IR Codes (aus LIRC RAV294 / RX-V1065) ----------------
#define YAMAHA_KEY_POWER   0x7E8154AB
#define YAMAHA_VOL_UP      0x5EA158A7   // KEY_VOLUMEUP
#define YAMAHA_VOL_DOWN    0x5EA1D827   // KEY_VOLUMEDOWN
#define YAMAHA_IN_PHONO    0x5EA128D7   // INPUT_PHONO
#define YAMAHA_IN_AUDIO1   0x5EA1A658   // INPUT_AUDIO1

// ---------------- Globals ----------------
WiFiClient   wifiClient;
PubSubClient mqtt(wifiClient);
IRsend       irsend(PIN_IR);

bool g_wifiReady       = false;
bool g_haControlOn     = false;
bool g_haControlOff    = false;

bool g_lastTriggerPub  = false;
bool g_lastMusicPub    = false;
bool g_musicActive     = false;
bool g_shutdownMode    = false;
bool g_triggerState    = false;
bool g_musicEverActive = false;

int  g_musicOnCounter  = 0;
int  g_musicOffCounter = 0;

unsigned long g_lastIrSendTime    = 0;
unsigned long g_lastMqttAttempt   = 0;
unsigned long g_lastWifiAttempt   = 0;
unsigned long g_lastMusicTime     = 0;
unsigned long g_triggerHighTime   = 0;
unsigned long g_mqttConnectedAt   = 0;
unsigned long g_lastManualIrTime  = 0;
unsigned long g_lastShortIrTime   = 0;

// ================================================================
// Forward Declarations
// ================================================================
void sendIR(const char* reason);
void sendIRCode(uint32_t code, const char* reason);
void publishSwitchState(bool on);
void publishTriggerState(bool on);
void publishMusicState(bool on);
void publishIrEvent(const char* reason);
void publishDiscovery();
void ensureMQTT();
void ensureWiFi();

// ================================================================
// WiFi – non-blocking
// ================================================================
void ensureWiFi() {
  if (WiFi.status() == WL_CONNECTED) { g_wifiReady = true; return; }
  wl_status_t s = WiFi.status();
  if (s == WL_DISCONNECTED || s == WL_NO_SSID_AVAIL || s == WL_CONNECT_FAILED) {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
  }
  g_wifiReady = (WiFi.status() == WL_CONNECTED);
}

// ================================================================
// MQTT Discovery
// ================================================================
void publishDiscovery() {
  mqtt.publish(TOPIC_DISC_SWITCH,
    "{"
      "\"name\":\"Audio Control\","
      "\"unique_id\":\"" DEVICE_ID "_control\","
      "\"command_topic\":\"" TOPIC_SWITCH_CMD "\","
      "\"state_topic\":\"" TOPIC_SWITCH_STATE "\","
      "\"payload_on\":\"ON\","
      "\"payload_off\":\"OFF\","
      "\"retain\":true,"
      "\"device\":{"
        "\"identifiers\":[\"" DEVICE_ID "\"],"
        "\"name\":\"ESP32 Audio Controller\","
        "\"model\":\"ESP32\","
        "\"manufacturer\":\"DIY\""
      "}"
    "}", true);

  mqtt.publish(TOPIC_DISC_TRIGGER,
    "{"
      "\"name\":\"Trigger Pin\","
      "\"unique_id\":\"" DEVICE_ID "_trigger\","
      "\"state_topic\":\"" TOPIC_TRIGGER_STATE "\","
      "\"payload_on\":\"ON\","
      "\"payload_off\":\"OFF\","
      "\"device_class\":\"connectivity\","
      "\"retain\":true,"
      "\"device\":{\"identifiers\":[\"" DEVICE_ID "\"],\"name\":\"ESP32 Audio Controller\"}"
    "}", true);

  mqtt.publish(TOPIC_DISC_MUSIC,
    "{"
      "\"name\":\"Musik aktiv\","
      "\"unique_id\":\"" DEVICE_ID "_music\","
      "\"state_topic\":\"" TOPIC_MUSIC_STATE "\","
      "\"payload_on\":\"ON\","
      "\"payload_off\":\"OFF\","
      "\"device_class\":\"sound\","
      "\"retain\":true,"
      "\"device\":{\"identifiers\":[\"" DEVICE_ID "\"],\"name\":\"ESP32 Audio Controller\"}"
    "}", true);

  mqtt.publish(TOPIC_DISC_IR,
    "{"
      "\"name\":\"IR Senden\","
      "\"unique_id\":\"" DEVICE_ID "_ir_send\","
      "\"command_topic\":\"" TOPIC_IR_CMD "\","
      "\"payload_press\":\"PRESS\","
      "\"device\":{"
        "\"identifiers\":[\"" DEVICE_ID "\"],"
        "\"name\":\"ESP32 Audio Controller\","
        "\"model\":\"ESP32\","
        "\"manufacturer\":\"DIY\""
      "}"
    "}", true);

  // NEU: Vier weitere Buttons. Gleiches command_topic, unterschiedliche payload_press.
  // Vorteil: nur eine Subscription, Retained-Handling & Startup-Schutz gelten automatisch.
  mqtt.publish(TOPIC_DISC_VOL_UP,
    "{"
      "\"name\":\"Lautstärke +\","
      "\"unique_id\":\"" DEVICE_ID "_vol_up\","
      "\"command_topic\":\"" TOPIC_IR_CMD "\","
      "\"payload_press\":\"VOL_UP\","
      "\"icon\":\"mdi:volume-plus\","
      "\"device\":{\"identifiers\":[\"" DEVICE_ID "\"],\"name\":\"ESP32 Audio Controller\"}"
    "}", true);

  mqtt.publish(TOPIC_DISC_VOL_DOWN,
    "{"
      "\"name\":\"Lautstärke -\","
      "\"unique_id\":\"" DEVICE_ID "_vol_down\","
      "\"command_topic\":\"" TOPIC_IR_CMD "\","
      "\"payload_press\":\"VOL_DOWN\","
      "\"icon\":\"mdi:volume-minus\","
      "\"device\":{\"identifiers\":[\"" DEVICE_ID "\"],\"name\":\"ESP32 Audio Controller\"}"
    "}", true);

  mqtt.publish(TOPIC_DISC_PHONO,
    "{"
      "\"name\":\"Input Phono\","
      "\"unique_id\":\"" DEVICE_ID "_in_phono\","
      "\"command_topic\":\"" TOPIC_IR_CMD "\","
      "\"payload_press\":\"PHONO\","
      "\"icon\":\"mdi:album\","
      "\"device\":{\"identifiers\":[\"" DEVICE_ID "\"],\"name\":\"ESP32 Audio Controller\"}"
    "}", true);

  mqtt.publish(TOPIC_DISC_AUDIO1,
    "{"
      "\"name\":\"Input Audio1\","
      "\"unique_id\":\"" DEVICE_ID "_in_audio1\","
      "\"command_topic\":\"" TOPIC_IR_CMD "\","
      "\"payload_press\":\"AUDIO1\","
      "\"icon\":\"mdi:audio-input-rca\","
      "\"device\":{\"identifiers\":[\"" DEVICE_ID "\"],\"name\":\"ESP32 Audio Controller\"}"
    "}", true);

  mqtt.publish(TOPIC_DISC_IR_EVENT,
    "{"
      "\"name\":\"IR Letzter Grund\","
      "\"unique_id\":\"" DEVICE_ID "_ir_event\","
      "\"state_topic\":\"" TOPIC_IR_EVENT "\","
      "\"retain\":true,"
      "\"icon\":\"mdi:remote\","
      "\"device\":{"
        "\"identifiers\":[\"" DEVICE_ID "\"],"
        "\"name\":\"ESP32 Audio Controller\","
        "\"model\":\"ESP32\","
        "\"manufacturer\":\"DIY\""
      "}"
    "}", true);
}

// ================================================================
// MQTT Callback
// ================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  char msg[16] = {0};
  unsigned int copyLen = (length < 15) ? length : 15;
  memcpy(msg, payload, copyLen);
  Serial.printf("MQTT [%s]: %s\n", topic, msg);

  if (strcmp(topic, TOPIC_SWITCH_CMD) == 0) {
    if (strcmp(msg, "ON") == 0) {
      g_haControlOn  = true;
      g_haControlOff = false;
    } else if (strcmp(msg, "OFF") == 0) {
      if (millis() - g_mqttConnectedAt < MQTT_RETAINED_IGNORE_MS) {
        Serial.println("MQTT OFF ignoriert: retained (Startup)");
        publishSwitchState(true);
      } else {
        g_haControlOff = true;
        g_haControlOn  = false;
      }
    }
  }

  if (strcmp(topic, TOPIC_IR_CMD) == 0) {

    // FIX: nur clearen wenn die Nachricht tatsächlich Inhalt hatte.
    // Vorher wurde hier IMMER (auch bei leerer Nachricht) erneut auf
    // dasselbe Topic publiziert, auf das der ESP32 selbst subscribed ist.
    // Das erzeugte eine Self-Feedback-Loop: publish("") -> eigener Empfang
    // -> erneutes publish("") -> ... im Millisekundentakt.
    if (length > 0) {
      mqtt.publish(TOPIC_IR_CMD, "", true);  // Retained Press löschen
    } else {
      return;  // leere Clear-Nachricht: nichts weiter tun
    }

    unsigned long now = millis();

    // Schutzfenster: Press kurz nach Connect = retained, ignorieren.
    // Gilt für ALLE Button-Payloads (POWER, VOL, INPUT).
    if (millis() - g_mqttConnectedAt < MQTT_RETAINED_IGNORE_MS) {
      Serial.println("MQTT Press ignoriert: retained (Startup)");
      return;
    }

    // ---- POWER (langer Debounce, Doppel-Toggle wäre fatal) ----
    if (strcmp(msg, "PRESS") == 0) {
      if (now - g_lastManualIrTime < MANUAL_IR_DEBOUNCE_MS) {
        Serial.println("MQTT IR PRESS ignoriert: Debounce (zu kurz nach letztem Press)");
        return;
      }
      g_lastManualIrTime = now;
      Serial.println("HA Button: IR manuell (POWER)");
      sendIR("manual_ha");
      return;
    }

    // ---- Vol/Input (kurzer Debounce: schnelles Drücken erlaubt) ----
    if (now - g_lastShortIrTime < SHORT_IR_DEBOUNCE_MS) {
      Serial.println("MQTT Press ignoriert: Kurz-Debounce");
      return;
    }

    if (strcmp(msg, "VOL_UP") == 0) {
      g_lastShortIrTime = now;
      sendIRCode(YAMAHA_VOL_UP, "vol_up");
    } else if (strcmp(msg, "VOL_DOWN") == 0) {
      g_lastShortIrTime = now;
      sendIRCode(YAMAHA_VOL_DOWN, "vol_down");
    } else if (strcmp(msg, "PHONO") == 0) {
      g_lastShortIrTime = now;
      sendIRCode(YAMAHA_IN_PHONO, "input_phono");
    } else if (strcmp(msg, "AUDIO1") == 0) {
      g_lastShortIrTime = now;
      sendIRCode(YAMAHA_IN_AUDIO1, "input_audio1");
    }
  }
}

// ================================================================
// MQTT Reconnect
// ================================================================
void ensureMQTT() {
  if (mqtt.connected() || !g_wifiReady) return;
  Serial.print("MQTT verbinden...");
  mqtt.setKeepAlive(60);
  bool ok = (strlen(mqttUser) > 0)
    ? mqtt.connect(DEVICE_ID, mqttUser, mqttPassword)
    : mqtt.connect(DEVICE_ID);
  if (ok) {
    Serial.println(" OK");
    g_mqttConnectedAt = millis();  // bei JEDEM Connect (auch Reconnect) setzen
    mqtt.subscribe(TOPIC_SWITCH_CMD);
    mqtt.subscribe(TOPIC_IR_CMD);
    delay(500);
    publishDiscovery();
  } else {
    Serial.printf(" fehlgeschlagen rc=%d\n", mqtt.state());
  }
}

// ================================================================
// Publish Helpers
// ================================================================
void publishSwitchState(bool on) {
  if (mqtt.connected()) mqtt.publish(TOPIC_SWITCH_STATE, on ? "ON" : "OFF", true);
}
void publishTriggerState(bool on) {
  if (mqtt.connected()) mqtt.publish(TOPIC_TRIGGER_STATE, on ? "ON" : "OFF", true);
}
void publishMusicState(bool on) {
  if (mqtt.connected()) mqtt.publish(TOPIC_MUSIC_STATE, on ? "ON" : "OFF", true);
}

void publishIrEvent(const char* reason) {
  if (!mqtt.connected()) return;
  char payload[128];
  snprintf(payload, sizeof(payload),
    "{\"reason\":\"%s\",\"ts\":%lu}", reason, millis());
  mqtt.publish(TOPIC_IR_EVENT, payload, true);
}

// ================================================================
// IR – generisch mit Code + Grund
// ================================================================
void sendIRCode(uint32_t code, const char* reason) {
  Serial.printf("IR: 0x%08X | Grund: %s\n", code, reason);
  irsend.sendNEC(code, 32);
  publishIrEvent(reason);
}

// POWER bleibt als eigene Funktion (wird auch vom Silence-Timeout genutzt)
void sendIR(const char* reason) {
  sendIRCode(YAMAHA_KEY_POWER, reason);
}

// ================================================================
// Audio Messung
// ================================================================
float measureStddev() {
  long sum = 0;
  int samples[SAMPLE_COUNT];
  for (int i = 0; i < SAMPLE_COUNT; i++) { samples[i] = analogRead(AUX_PIN); sum += samples[i]; }
  float mean = (float)sum / SAMPLE_COUNT;
  float variance = 0.0f;
  for (int i = 0; i < SAMPLE_COUNT; i++) { float d = samples[i] - mean; variance += d * d; }
  return sqrtf(variance / SAMPLE_COUNT);
}

// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_TRIGGER, INPUT);
  pinMode(PIN_IR, OUTPUT);
  analogReadResolution(12);
  irsend.begin();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 8000) delay(100);
  g_wifiReady = (WiFi.status() == WL_CONNECTED);

  mqtt.setServer(mqttBroker, mqttPort);
  mqtt.setKeepAlive(60);
  mqtt.setBufferSize(1024);
  mqtt.setCallback(mqttCallback);
  ensureMQTT();

  g_triggerState    = (digitalRead(PIN_TRIGGER) == HIGH);
  g_lastMusicTime   = millis();
  g_triggerHighTime = millis();

  publishTriggerState(g_triggerState);
  publishSwitchState(g_triggerState);

  Serial.printf("Gestartet. Trigger=%s Silence=%.2f min\n",
    g_triggerState ? "HIGH" : "LOW", g_silenceMinutes);
}

// ================================================================
// LOOP
// ================================================================
void loop() {
  unsigned long now = millis();

  if (!g_wifiReady && now - g_lastWifiAttempt > WIFI_RECONNECT_MS) {
    g_lastWifiAttempt = now; ensureWiFi();
  }
  if (!mqtt.connected() && now - g_lastMqttAttempt > MQTT_RECONNECT_MS) {
    g_lastMqttAttempt = now; ensureWiFi(); ensureMQTT();
  }
  mqtt.loop();

  static unsigned long lastDiscovery = 0;
  if (mqtt.connected() && now - lastDiscovery >= DISCOVERY_INTERVAL_MS) {
    lastDiscovery = now; publishDiscovery();
  }

  // ---- HA Befehle ----
  if (g_haControlOff) { g_haControlOff = false; publishSwitchState(false); }
  if (g_haControlOn)  { g_haControlOn  = false; publishSwitchState(true);  }

  // ---- Trigger Pin ----
  bool triggerNow = (digitalRead(PIN_TRIGGER) == HIGH);
  if (triggerNow != g_lastTriggerPub) {
    g_lastTriggerPub = triggerNow;
    publishTriggerState(triggerNow);
    Serial.printf("Trigger: %s\n", triggerNow ? "HIGH" : "LOW");

    if (triggerNow) {
      g_shutdownMode    = false;
      g_musicActive     = false;
      g_musicEverActive = false;
      g_musicOnCounter  = 0;
      g_musicOffCounter = 0;
      g_lastMusicTime   = now;
      g_triggerHighTime = now;
      g_lastMusicPub    = false;
      publishSwitchState(true);
      publishMusicState(false);
    } else {
      g_musicActive     = false;
      g_musicEverActive = false;
      g_shutdownMode    = false;
      if (g_lastMusicPub) { g_lastMusicPub = false; publishMusicState(false); }
      publishSwitchState(false);
    }
  }
  g_triggerState = triggerNow;

  // ---- Audio Monitoring ----
  static unsigned long lastMusicCheck = 0;
  if (g_triggerState && !g_shutdownMode && now - lastMusicCheck >= MUSIC_CHECK_MS) {
    lastMusicCheck = now;
    float stddev = measureStddev();
    Serial.printf("Stddev: %.2f | OnCnt=%d OffCnt=%d\n",
      stddev, g_musicOnCounter, g_musicOffCounter);

    if (!g_musicActive) {
      if (stddev > THRESH_ON)          g_musicOnCounter++;
      else if (g_musicOnCounter > 0)   g_musicOnCounter--;
      if (g_musicOnCounter >= MUSIC_ON_CONFIRM) {
        g_musicActive     = true;
        g_musicEverActive = true;
        g_musicOffCounter = 0;
        Serial.println("Musik: AN");
      }
    } else {
      if (stddev < THRESH_OFF)          g_musicOffCounter++;
      else if (g_musicOffCounter > 0)   g_musicOffCounter--;
      if (g_musicOffCounter >= MUSIC_OFF_CONFIRM) {
        g_musicActive    = false;
        g_musicOnCounter = 0;
        Serial.println("Musik: AUS");
      }
    }

    if (g_musicActive) g_lastMusicTime = now;

    if (g_musicActive != g_lastMusicPub) {
      g_lastMusicPub = g_musicActive;
      publishMusicState(g_musicActive);
    }

    unsigned long silenceMs = (unsigned long)(g_silenceMinutes * 60.0f * 1000.0f);
    bool graceOver = (now - g_triggerHighTime >= TRIGGER_GRACE_MS);

    if ((g_musicEverActive || graceOver) && now - g_lastMusicTime >= silenceMs) {
      Serial.println("Stille-Timeout -> Shutdown");
      g_shutdownMode   = true;
      g_lastIrSendTime = 0;
      publishSwitchState(false);
    }
  }

  // ---- IR Shutdown Sequenz ----
  if (g_shutdownMode) {
    if (g_lastIrSendTime == 0 || now - g_lastIrSendTime >= IR_INTERVAL_MS) {
      g_lastIrSendTime = now;
      sendIR("silence_timeout");
    }
  }

  // ---- Serial Status ----
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= STATE_PRINT_MS) {
    lastPrint = now;
    float silenceSec = g_triggerState ? (now - g_lastMusicTime) / 1000.0f : 0.0f;
    Serial.printf(
      "[%s] Trigger=%s Music=%s EverActive=%s MQTT=%s | Stille=%.1fs / Timeout=%.0fs\n",
      g_shutdownMode ? "SHUTDOWN" : (g_triggerState ? "AKTIV" : "IDLE"),
      g_triggerState ? "HIGH" : "LOW",
      g_musicActive  ? "JA"  : "NEIN",
      g_musicEverActive ? "JA" : "NEIN",
      mqtt.connected() ? "OK" : "ERR",
      silenceSec,
      g_silenceMinutes * 60.0f
    );
  }

  delay(5);
}
