#include <Arduino.h>
#define MQTT_MAX_PACKET_SIZE 1024

#include <WiFi.h>
#include <IRremoteESP8266.h>
#include <IRsend.h>
#include <string.h>
#include <PubSubClient.h>

// ---------------- WLAN ----------------
const char* ssid = "XXXX";
const char* password = "XXXXX";

// ---------------- MQTT ----------------
const char* mqttBroker   = "192.168.XXX.XXX"; // ggf. ändern
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

// Discovery-Topics für die vier zusätzlichen Buttons
#define TOPIC_DISC_VOL_UP   MQTT_PREFIX "/button/" DEVICE_ID "_vol_up/config"
#define TOPIC_DISC_VOL_DOWN MQTT_PREFIX "/button/" DEVICE_ID "_vol_down/config"
#define TOPIC_DISC_PHONO    MQTT_PREFIX "/button/" DEVICE_ID "_in_phono/config"
#define TOPIC_DISC_AUDIO1   MQTT_PREFIX "/button/" DEVICE_ID "_in_audio1/config"

// NEU: 1 = alle IR-Buttons in HA aktiv.
//      0 = Buttons für Fremdgeräte (BD/CD/USB/Dock/Ext) werden in HA als
//          "deaktiviert" angelegt (existieren, aber ausgeblendet; per Klick aktivierbar)
#define SHOW_ALL_BUTTONS 1

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

// ================================================================
// NEU: Tabelle aller weiteren IR-Codes aus der LIRC-Datei
// ================================================================
// Nicht enthalten (existieren bereits als eigene Buttons oben):
//   KEY_POWER, KEY_VOLUMEUP, KEY_VOLUMEDOWN, INPUT_PHONO, INPUT_AUDIO1
// Nicht enthalten (in LIRC auskommentiert):
//   CD_STOP, EXT_FWD, EXT_BACK_SKIP, EXT_FWD_SKIP
//
// id   = MQTT-Payload + Teil der unique_id / Discovery-Topic (nur a-z, 0-9, _)
// name = Anzeigename in Home Assistant

#define IRF_CORE   0x01   // Receiver-Funktion (immer aktiv in HA)
#define IRF_TOGGLE 0x02   // Toggle-Funktion -> langer Debounce (3 s)

struct IrButton {
  const char* id;
  const char* name;
  uint32_t    code;
  const char* icon;
  uint8_t     flags;
};

const IrButton IR_BUTTONS[] = {
  // ---- Allgemein ----
  {"key_sleep",           "Sleep",              0x5EA10CF3, "mdi:sleep",                 IRF_CORE},
  {"vol_mute",            "Mute",               0x5EA138C7, "mdi:volume-mute",           IRF_CORE | IRF_TOGGLE},
  {"key_info",            "Info",               0x5EA1E41A, "mdi:information-outline",   IRF_CORE},
  {"key_option",          "Option",             0x5EA1D628, "mdi:tune",                  IRF_CORE},
  {"key_epg",             "On Screen",          0x5EA121DE, "mdi:monitor",               IRF_CORE},
  {"display",             "Display",            0xFE8006F9, "mdi:monitor-dashboard",     IRF_CORE},
  {"top_menu",            "Top Menu",           0x5EA1758A, "mdi:menu",                  IRF_CORE},
  {"key_menu",            "Menu",               0x5EA1D52A, "mdi:menu",                  IRF_CORE},

  // ---- Source Power (Fremdgeräte, Toggle) ----
  {"bd_source_power",     "BD Power",           0x3EC101FE, "mdi:power",                 IRF_TOGGLE},
  {"cd_source_power",     "CD Power",           0x9E6106F9, "mdi:power",                 IRF_TOGGLE},
  {"radio_source_power",  "Radio Power",        0x8B74D827, "mdi:power",                 IRF_TOGGLE},
  {"usb_source_power",    "USB Power",          0xFE8004FB, "mdi:power",                 IRF_TOGGLE},
  {"dock_source_power",   "Dock Power",         0xFE8000FF, "mdi:power",                 IRF_TOGGLE},

  // ---- Inputs ----
  {"input_hdmi1",         "Input HDMI1",        0x5EA1E21C, "mdi:video-input-hdmi",      IRF_CORE},
  {"input_hdmi2",         "Input HDMI2",        0x5EA152AC, "mdi:video-input-hdmi",      IRF_CORE},
  {"input_hdmi3",         "Input HDMI3",        0x5EA1B24C, "mdi:video-input-hdmi",      IRF_CORE},
  {"input_hdmi4",         "Input HDMI4",        0x5EA10AF4, "mdi:video-input-hdmi",      IRF_CORE},
  {"input_av1",           "Input AV1",          0x5EA1CA34, "mdi:video-input-component", IRF_CORE},
  {"input_av2",           "Input AV2",          0x5EA16A94, "mdi:video-input-component", IRF_CORE},
  {"input_av3",           "Input AV3",          0x5EA19A64, "mdi:video-input-component", IRF_CORE},
  {"input_av4",           "Input AV4",          0x5EA13AC4, "mdi:video-input-component", IRF_CORE},
  {"input_av5",           "Input AV5",          0x5EA1FA04, "mdi:video-input-component", IRF_CORE},
  {"input_av6",           "Input AV6",          0x5EA146B8, "mdi:video-input-component", IRF_CORE},
  {"input_audio2",        "Input Audio2",       0x5EA116E8, "mdi:audio-input-rca",       IRF_CORE},
  {"input_v_aux",         "Input V-AUX",        0x5EA1AA55, "mdi:audio-input-stereo-minijack", IRF_CORE},
  {"input_multi",         "Input Multi Ch",     0x5EA1E11E, "mdi:surround-sound",        IRF_CORE},
  {"input_tuner",         "Input Tuner",        0x5EA16897, "mdi:radio",                 IRF_CORE},
  {"input_usb",           "Input USB",          0xFE80FC03, "mdi:usb",                   IRF_CORE},
  {"input_dock",          "Input Dock",         0xFE8052AD, "mdi:ipod",                  IRF_CORE},

  // ---- Tuner ----
  {"tuner_fm",            "Tuner FM",           0xFE801AE4, "mdi:radio",                 IRF_CORE},
  {"tuner_am",            "Tuner AM",           0xFE80AA54, "mdi:radio",                 IRF_CORE},
  {"tuner_memory",        "Tuner Memory",       0xFE80E618, "mdi:content-save",          IRF_CORE},
  {"tuner_preset_up",     "Tuner Preset +",     0xFE80DA24, "mdi:chevron-up",            IRF_CORE},
  {"tuner_preset_down",   "Tuner Preset -",     0xFE807A84, "mdi:chevron-down",          IRF_CORE},
  {"tuner_tunning_up",    "Tuner Tuning +",     0xFE808678, "mdi:chevron-double-up",     IRF_CORE},
  {"tuner_tunning_down",  "Tuner Tuning -",     0xFE8026D8, "mdi:chevron-double-down",   IRF_CORE},

  // ---- DSP ----
  {"dsp_movie",           "DSP Movie",          0x5EA111EE, "mdi:movie-open",            IRF_CORE},
  {"dsp_music",           "DSP Music",          0x5EA1916E, "mdi:music",                 IRF_CORE},
  {"dsp_stereo_enhancer", "DSP Stereo/Enhancer",0x5EA129D6, "mdi:speaker-multiple",      IRF_CORE},
  {"dsp_straight",        "DSP Straight",       0x5EA16A95, "mdi:arrow-right-bold",      IRF_CORE},
  {"dsp_pure_direct",     "DSP Pure Direct",    0x5EA1BB44, "mdi:waveform",              IRF_CORE},
  {"dsp_sur_decode",      "DSP Surround Decode",0x5EA1B14E, "mdi:surround-sound",        IRF_CORE},

  // ---- Scene ----
  {"scene_bd",            "Scene BD",           0x5EA100FE, "mdi:disc-player",           IRF_CORE},
  {"scene_tv",            "Scene TV",           0x5EA1C03E, "mdi:television",            IRF_CORE},
  {"scene_cd",            "Scene CD",           0x5EA1609E, "mdi:disc",                  IRF_CORE},
  {"scene_radio",         "Scene Radio",        0x5EA1906E, "mdi:radio",                 IRF_CORE},

  // ---- Navigation (Receiver) ----
  {"nav_up",              "Nav Up",             0x5EA1B946, "mdi:arrow-up-bold",         IRF_CORE},
  {"nav_down",            "Nav Down",           0x5EA139C6, "mdi:arrow-down-bold",       IRF_CORE},
  {"nav_left",            "Nav Left",           0x5EA1F906, "mdi:arrow-left-bold",       IRF_CORE},
  {"nav_right",           "Nav Right",          0x5EA17986, "mdi:arrow-right-bold",      IRF_CORE},
  {"nav_enter",           "Nav Enter",          0x5EA17B84, "mdi:keyboard-return",       IRF_CORE},
  {"nav_return",          "Nav Return",         0x5EA155AA, "mdi:undo",                  IRF_CORE},

  // ---- BD ----
  {"bd_nav_up",           "BD Nav Up",          0x3EC12DD2, "mdi:arrow-up-bold",         0},
  {"bd_nav_down",         "BD Nav Down",        0x3EC1CD32, "mdi:arrow-down-bold",       0},
  {"bd_nav_left",         "BD Nav Left",        0x3EC1AD52, "mdi:arrow-left-bold",       0},
  {"bd_nav_right",        "BD Nav Right",       0x3EC16D92, "mdi:arrow-right-bold",      0},
  {"bd_nav_enter",        "BD Nav Enter",       0x3EC11DE2, "mdi:keyboard-return",       0},
  {"bd_nav_return",       "BD Nav Return",      0x3EC1ED12, "mdi:undo",                  0},
  {"bd_display",          "BD Display",         0x3EC1659A, "mdi:monitor-dashboard",     0},
  {"bd_top_menu",         "BD Top Menu",        0x3EC18D72, "mdi:menu",                  0},
  {"bd_menu",             "BD Menu",            0x3EC10BF4, "mdi:menu",                  0},
  {"bd_rec",              "BD Rec",             0x3EC1D12E, "mdi:record",                0},
  {"bd_play",             "BD Play",            0x3EC141BE, "mdi:play",                  0},
  {"bd_stop",             "BD Stop",            0x3EC1A15E, "mdi:stop",                  0},
  {"bd_pause",            "BD Pause",           0x3EC1C13E, "mdi:pause",                 0},
  {"bd_back",             "BD Rewind",          0x3EC1619E, "mdi:rewind",                0},
  {"bd_fwd",              "BD Fast Forward",    0x3EC1E11E, "mdi:fast-forward",          0},
  {"bd_back_skip",        "BD Skip Back",       0x3EC19D62, "mdi:skip-previous",         0},
  {"bd_fwd_skip",         "BD Skip Forward",    0x3EC15DA2, "mdi:skip-next",             0},
  {"bd_num_1",            "BD 1",               0x3EC129D6, "mdi:numeric-1",             0},
  {"bd_num_2",            "BD 2",               0x3EC1A956, "mdi:numeric-2",             0},
  {"bd_num_3",            "BD 3",               0x3EC16996, "mdi:numeric-3",             0},
  {"bd_num_4",            "BD 4",               0x3EC1E916, "mdi:numeric-4",             0},
  {"bd_num_5",            "BD 5",               0x3EC119E6, "mdi:numeric-5",             0},
  {"bd_num_6",            "BD 6",               0x3EC19966, "mdi:numeric-6",             0},
  {"bd_num_7",            "BD 7",               0x3EC159A6, "mdi:numeric-7",             0},
  {"bd_num_8",            "BD 8",               0x3EC1D926, "mdi:numeric-8",             0},
  {"bd_num_9",            "BD 9",               0x3EC139C6, "mdi:numeric-9",             0},
  {"bd_num_0",            "BD 0",               0x3EC1C936, "mdi:numeric-0",             0},
  {"bd_num_plus10",       "BD +10",             0x3EC1B946, "mdi:numeric-10-box",        0},
  {"bd_num_ent",          "BD Num Enter",       0x3EC17986, "mdi:keyboard-return",       0},

  // ---- CD ----
  {"cd_display",          "CD Display",         0x9E6150AF, "mdi:monitor-dashboard",     0},
  {"cd_rec",              "CD Rec",             0x5EA1F20D, "mdi:record",                0},
  {"cd_play",             "CD Play",            0x5EA110EF, "mdi:play",                  0},
  {"cd_pause",            "CD Pause/Stop",      0x5EA1906F, "mdi:pause",                 0},
  {"cd_back",             "CD Rewind",          0x5EA1B04F, "mdi:rewind",                0},
  {"cd_fwd",              "CD Fast Forward",    0x5EA130CF, "mdi:fast-forward",          0},
  {"cd_back_skip",        "CD Skip Back",       0x5EA1D02F, "mdi:skip-previous",         0},
  {"cd_fwd_skip",         "CD Skip Forward",    0x5EA150AF, "mdi:skip-next",             0},
  {"cd_num_1",            "CD 1",               0x9E618877, "mdi:numeric-1",             0},
  {"cd_num_2",            "CD 2",               0x9E6148B7, "mdi:numeric-2",             0},
  {"cd_num_3",            "CD 3",               0x9E61C837, "mdi:numeric-3",             0},
  {"cd_num_4",            "CD 4",               0x9E6128D7, "mdi:numeric-4",             0},
  {"cd_num_5",            "CD 5",               0x9E61A857, "mdi:numeric-5",             0},
  {"cd_num_6",            "CD 6",               0x9E616897, "mdi:numeric-6",             0},
  {"cd_num_7",            "CD 7",               0x9E61E817, "mdi:numeric-7",             0},
  {"cd_num_8",            "CD 8",               0x9E6118E7, "mdi:numeric-8",             0},
  {"cd_num_9",            "CD 9",               0x9E619867, "mdi:numeric-9",             0},
  {"cd_num_0",            "CD 0",               0x9E6108F7, "mdi:numeric-0",             0},
  {"cd_num_plus10",       "CD +10",             0x9E6158A7, "mdi:numeric-10-box",        0},
  {"cd_num_ent",          "CD Num Enter",       0x9E61D02F, "mdi:keyboard-return",       0},

  // ---- USB ----
  {"usb_nav_up",          "USB Nav Up",         0xFE80748B, "mdi:arrow-up-bold",         0},
  {"usb_nav_down",        "USB Nav Down",       0xFE802CD3, "mdi:arrow-down-bold",       0},
  {"usb_nav_left",        "USB Nav Left",       0xFE800CF3, "mdi:arrow-left-bold",       0},
  {"usb_nav_right",       "USB Nav Right",      0xFE804CB3, "mdi:arrow-right-bold",      0},
  {"usb_nav_enter",       "USB Nav Enter",      0xFE808C73, "mdi:keyboard-return",       0},
  {"usb_nav_return",      "USB Nav Return",     0xFE80CC33, "mdi:undo",                  0},
  {"usb_display",         "USB Display",        0xFE80AC53, "mdi:monitor-dashboard",     0},
  {"usb_top_menu",        "USB Top Menu",       0xFE80B44B, "mdi:menu",                  0},
  {"usb_menu",            "USB Menu",           0xFE80F40B, "mdi:menu",                  0},
  {"usb_rec",             "USB Rec",            0xFE806C93, "mdi:record",                0},
  {"usb_play",            "USB Play",           0xFE807C83, "mdi:play",                  0},
  {"usb_stop",            "USB Stop",           0xFE80BC43, "mdi:stop",                  0},
  {"usb_pause",           "USB Pause",          0xFE805CA3, "mdi:pause",                 0},
  {"usb_back",            "USB Rewind",         0xFE80EC13, "mdi:rewind",                0},
  {"usb_fwd",             "USB Fast Forward",   0xFE801CE3, "mdi:fast-forward",          0},
  {"usb_back_skip",       "USB Skip Back",      0xFE80DC23, "mdi:skip-previous",         0},
  {"usb_fwd_skip",        "USB Skip Forward",   0xFE803CC3, "mdi:skip-next",             0},
  {"usb_num_1",           "USB 1",              0xFE80847B, "mdi:numeric-1",             0},
  {"usb_num_2",           "USB 2",              0xFE8044BB, "mdi:numeric-2",             0},
  {"usb_num_3",           "USB 3",              0xFE80C43B, "mdi:numeric-3",             0},
  {"usb_num_4",           "USB 4",              0xFE8024DB, "mdi:numeric-4",             0},
  {"usb_num_5",           "USB 5",              0xFE80A45B, "mdi:numeric-5",             0},
  {"usb_num_6",           "USB 6",              0xFE80649B, "mdi:numeric-6",             0},
  {"usb_num_7",           "USB 7",              0xFE80E41B, "mdi:numeric-7",             0},
  {"usb_num_8",           "USB 8",              0xFE8014EB, "mdi:numeric-8",             0},
  {"usb_num_9",           "USB 9",              0xFE80946B, "mdi:numeric-9",             0},
  {"usb_num_0",           "USB 0",              0xFE8054AB, "mdi:numeric-0",             0},
  {"usb_num_plus10",      "USB +10",            0xFE80D42B, "mdi:numeric-10-box",        0},
  {"usb_num_ent",         "USB Num Enter",      0xFE8034CB, "mdi:keyboard-return",       0},

  // ---- Dock ----
  {"dock_nav_up",         "Dock Nav Up",        0xFE80708F, "mdi:arrow-up-bold",         0},
  {"dock_nav_down",       "Dock Nav Down",      0xFE8028D7, "mdi:arrow-down-bold",       0},
  {"dock_nav_left",       "Dock Nav Left",      0xFE8008F7, "mdi:arrow-left-bold",       0},
  {"dock_nav_right",      "Dock Nav Right",     0xFE8048B7, "mdi:arrow-right-bold",      0},
  {"dock_nav_enter",      "Dock Nav Enter",     0xFE808877, "mdi:keyboard-return",       0},
  {"dock_nav_return",     "Dock Nav Return",    0xFE80C837, "mdi:undo",                  0},
  {"dock_display",        "Dock Display",       0xFE80A857, "mdi:monitor-dashboard",     0},
  {"dock_top_menu",       "Dock Top Menu",      0xFE80B04F, "mdi:menu",                  0},
  {"dock_menu",           "Dock Menu",          0xFE80F00F, "mdi:menu",                  0},
  {"dock_rec",            "Dock Rec",           0xFE806897, "mdi:record",                0},
  {"dock_play",           "Dock Play",          0xFE807887, "mdi:play",                  0},
  {"dock_stop",           "Dock Stop",          0xFE80B847, "mdi:stop",                  0},
  {"dock_pause",          "Dock Pause",         0xFE8058A7, "mdi:pause",                 0},
  {"dock_back",           "Dock Rewind",        0xFE80E817, "mdi:rewind",                0},
  {"dock_fwd",            "Dock Fast Forward",  0xFE8018E7, "mdi:fast-forward",          0},
  {"dock_back_skip",      "Dock Skip Back",     0xFE80D827, "mdi:skip-previous",         0},
  {"dock_fwd_skip",       "Dock Skip Forward",  0xFE8038C7, "mdi:skip-next",             0},
  {"dock_num_1",          "Dock 1",             0xFE80807F, "mdi:numeric-1",             0},
  {"dock_num_2",          "Dock 2",             0xFE8040BF, "mdi:numeric-2",             0},
  {"dock_num_3",          "Dock 3",             0xFE80C03F, "mdi:numeric-3",             0},
  {"dock_num_4",          "Dock 4",             0xFE8020DF, "mdi:numeric-4",             0},
  {"dock_num_5",          "Dock 5",             0xFE80A05F, "mdi:numeric-5",             0},
  {"dock_num_6",          "Dock 6",             0xFE80609F, "mdi:numeric-6",             0},
  {"dock_num_7",          "Dock 7",             0xFE80E01F, "mdi:numeric-7",             0},
  {"dock_num_8",          "Dock 8",             0xFE8010EF, "mdi:numeric-8",             0},
  {"dock_num_9",          "Dock 9",             0xFE80906F, "mdi:numeric-9",             0},
  {"dock_num_0",          "Dock 0",             0xFE8050AF, "mdi:numeric-0",             0},
  {"dock_num_plus10",     "Dock +10",           0xFE80D02F, "mdi:numeric-10-box",        0},
  {"dock_num_ent",        "Dock Num Enter",     0xFE8030CF, "mdi:keyboard-return",       0},

  // ---- EXT ----
  {"ext_num_1",           "Ext 1",              0x5EA1A758, "mdi:numeric-1",             0},
  {"ext_num_2",           "Ext 2",              0x5EA16798, "mdi:numeric-2",             0},
  {"ext_num_3",           "Ext 3",              0x5EA1E718, "mdi:numeric-3",             0},
  {"ext_num_4",           "Ext 4",              0x5EA117E8, "mdi:numeric-4",             0},
  {"ext_num_5",           "Ext 5",              0x5EA19768, "mdi:numeric-5",             0},
  {"ext_num_6",           "Ext 6",              0x5EA157A8, "mdi:numeric-6",             0},
  {"ext_num_7",           "Ext 7",              0x5EA1D728, "mdi:numeric-7",             0},
  {"ext_num_8",           "Ext 8",              0x5EA137C8, "mdi:numeric-8",             0},
  {"ext_num_9",           "Ext 9",              0x5EA18D72, "mdi:numeric-9",             0},
  {"ext_num_0",           "Ext 0",              0x5EA14DB2, "mdi:numeric-0",             0},
  {"ext_num_ent",         "Ext Num Enter",      0x5EA1CD32, "mdi:keyboard-return",       0},
};

const size_t IR_BUTTON_COUNT = sizeof(IR_BUTTONS) / sizeof(IR_BUTTONS[0]);

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

// NEU: eigener Debounce-Timer für Toggle-Codes aus der Tabelle (Mute, Source Power),
// getrennt vom POWER-Timer, damit dessen Verhalten unverändert bleibt
unsigned long g_lastToggleIrTime  = 0;

// NEU: Index für gestaffelte Discovery der Tabellen-Buttons (-1 = inaktiv)
int g_extDiscIdx = -1;

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
void publishExtendedDiscoveryStep();
bool handleExtendedIr(const char* msg, unsigned long now);
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

  // Vier weitere Buttons. Gleiches command_topic, unterschiedliche payload_press.
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

  // NEU: Tabellen-Buttons gestaffelt aus loop() publizieren (1 pro Durchlauf),
  // damit ~170 Publishes die Loop / Audio-Messung nicht blockieren
  g_extDiscIdx = 0;
}

// ================================================================
// NEU: Discovery für einen Tabellen-Button pro Aufruf
// ================================================================
void publishExtendedDiscoveryStep() {
  if (g_extDiscIdx < 0) return;
  if ((size_t)g_extDiscIdx >= IR_BUTTON_COUNT) {
    Serial.printf("Erweiterte Discovery fertig (%u Buttons)\n", (unsigned)IR_BUTTON_COUNT);
    g_extDiscIdx = -1;
    return;
  }

  const IrButton& b = IR_BUTTONS[g_extDiscIdx];

  bool enabled = SHOW_ALL_BUTTONS || (b.flags & IRF_CORE);

  char topic[128];
  snprintf(topic, sizeof(topic), MQTT_PREFIX "/button/" DEVICE_ID "_%s/config", b.id);

  char payload[512];
  snprintf(payload, sizeof(payload),
    "{"
      "\"name\":\"%s\","
      "\"unique_id\":\"" DEVICE_ID "_%s\","
      "\"command_topic\":\"" TOPIC_IR_CMD "\","
      "\"payload_press\":\"%s\","
      "\"icon\":\"%s\","
      "\"enabled_by_default\":%s,"
      "\"device\":{\"identifiers\":[\"" DEVICE_ID "\"],\"name\":\"ESP32 Audio Controller\"}"
    "}",
    b.name, b.id, b.id, b.icon, enabled ? "true" : "false");

  if (!mqtt.publish(topic, payload, true)) {
    Serial.printf("Discovery fehlgeschlagen: %s\n", b.id);
  }
  g_extDiscIdx++;
}

// ================================================================
// NEU: Tabellen-Lookup für IR-Payloads (lowercase IDs, kollidieren
// nicht mit den bestehenden Payloads PRESS/VOL_UP/VOL_DOWN/PHONO/AUDIO1)
// Rückgabe: true wenn Payload bekannt war
// ================================================================
bool handleExtendedIr(const char* msg, unsigned long now) {
  for (size_t i = 0; i < IR_BUTTON_COUNT; i++) {
    const IrButton& b = IR_BUTTONS[i];
    if (strcmp(msg, b.id) != 0) continue;

    if (b.flags & IRF_TOGGLE) {
      if (now - g_lastToggleIrTime < MANUAL_IR_DEBOUNCE_MS) {
        Serial.printf("MQTT %s ignoriert: Toggle-Debounce\n", b.id);
        return true;
      }
      g_lastToggleIrTime = now;
    }

    g_lastShortIrTime = now;
    sendIRCode(b.code, b.id);
    return true;
  }
  return false;
}

// ================================================================
// MQTT Callback
// ================================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Puffer vergrößert (vorher 16), damit lange Tabellen-IDs
  // wie "dsp_stereo_enhancer" nicht abgeschnitten werden
  char msg[48] = {0};
  unsigned int copyLen = (length < sizeof(msg) - 1) ? length : sizeof(msg) - 1;
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
    // Gilt für ALLE Button-Payloads (POWER, VOL, INPUT, Tabelle).
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

    // ---- Vol/Input/Tabelle (kurzer Debounce: schnelles Drücken erlaubt) ----
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
    } else if (!handleExtendedIr(msg, now)) {
      Serial.printf("MQTT unbekannter IR-Payload: %s\n", msg);
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

  Serial.printf("Gestartet. Trigger=%s Silence=%.2f min | IR-Buttons=%u\n",
    g_triggerState ? "HIGH" : "LOW", g_silenceMinutes, (unsigned)IR_BUTTON_COUNT);
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

  // NEU: gestaffelte Discovery der Tabellen-Buttons (1 pro Loop-Durchlauf)
  if (mqtt.connected() && g_extDiscIdx >= 0) {
    publishExtendedDiscoveryStep();
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
