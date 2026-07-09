#include <Arduino.h>
#include <math.h>
#include <cstring>
#ifndef USE_ETHERNET
#define USE_ETHERNET 1
#endif

#if USE_ETHERNET
#include <ETH.h>
#endif
#include <DNSServer.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_system.h>
#include <esp_log.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFiUdp.h>
#include <time.h>
#include <sys/time.h>
#include "driver/pcnt.h"
#ifndef WEBSERVER_MAX_POST_ARGS
#define WEBSERVER_MAX_POST_ARGS 2000
#endif
#include <WebServer.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include "web_ui.h"

// =========================
// Config (edit these)
// =========================
// Defaults (editable via web UI)
const char* DEFAULT_HOSTNAME = "stagemod";
const IPAddress DEFAULT_CLIENT_IP(192, 168, 0, 100);
const uint16_t DEFAULT_CLIENT_PORT = 8000;
const bool DEFAULT_USE_STATIC = false;
const IPAddress DEFAULT_STATIC_IP(192, 168, 0, 50);
const IPAddress DEFAULT_GATEWAY(192, 168, 0, 1);
const IPAddress DEFAULT_SUBNET(255, 255, 255, 0);
const IPAddress DEFAULT_DNS1(192, 168, 0, 1);
const IPAddress DEFAULT_DNS2(8, 8, 8, 8);
const char* DEFAULT_WIFI_SSID = "";
const char* DEFAULT_WIFI_PASS = "";
const char* DEFAULT_USERNAME = "admin";
const char* FIRMWARE_VERSION = "0.15.0";
const char* DEFAULT_NTP_SERVER = "pool.ntp.org";
const uint8_t DEFAULT_TIME_MODE = 0; // 0 = NTP, 1 = Manual
const char* DEFAULT_TIMEZONE = "PST8PDT,M3.2.0/2,M11.1.0/2";
const bool DEFAULT_TIME_DISPLAY_24H = false;
const bool DEFAULT_OSC_IN_ENABLED = true;
const uint16_t DEFAULT_OSC_IN_PORT = 9001;
const bool DEFAULT_ARTNET_ENABLED = true;
const bool DEFAULT_SACN_ENABLED = true;
const bool DEFAULT_DMX_IN_ARTNET_ENABLED = false;
const bool DEFAULT_DMX_IN_SACN_ENABLED = false;
const bool DEFAULT_MQTT_ENABLED = false;
const char* DEFAULT_MQTT_HOST = "";
const uint16_t DEFAULT_MQTT_PORT = 1883;
const char* DEFAULT_MQTT_USER = "";
const char* DEFAULT_MQTT_PASS = "";
const char* DEFAULT_MQTT_CLIENT_ID = "";
const char* DEFAULT_MQTT_BASE_TOPIC = "stagemod";
const uint8_t DEFAULT_MQTT_DEVICE = 0;

const int MAX_TRIGGERS = 20;
const int MAX_OSC_DEVICES = 8;
const int MAX_OUTPUTS_PER_TRIGGER = 5;
const int MAX_DMX_PRESETS = 100;
const int MAX_DMX_PRESET_POINTS = 1024;
const int MAX_DMX_PRESET_UNIVERSE = 40;
const uint8_t MQTT_DEVICE_CUSTOM = 255;

enum NetMode {
  NET_ETHERNET = 0,
  NET_WIFI = 1
};

enum SensorType {
  SENSOR_ANALOG = 0,
  SENSOR_BUTTON = 1,
  SENSOR_TOGGLE = 2,
  SENSOR_DIGITAL = 3,
  SENSOR_ENCODER = 4,
  SENSOR_BUTTON_DOUBLE = 5,
  SENSOR_BUTTON_TRIPLE = 6,
  SENSOR_BUTTON_LONG = 7,
  SENSOR_GPIO = 8,
  SENSOR_HCSR04 = 9
};

enum SensorSource {
  SRC_SENSORS = 0,
  SRC_TIME = 1,
  SRC_OSC = 2,
  SRC_DMX = 3
};

enum DmxInTriggerMode {
  DMX_IN_TRIG_LEVEL = 0,      // channel value drives outputs continuously (fader-style)
  DMX_IN_TRIG_THRESHOLD = 1,  // fires on threshold crossing (button-style)
  DMX_IN_TRIG_RANGE = 2       // maps a sub-range of 0-255 to the output range
};

enum OscInArgType {
  OSC_IN_ARG_ANY = 0,
  OSC_IN_ARG_INT = 1,
  OSC_IN_ARG_FLOAT = 2,
  OSC_IN_ARG_STRING = 3
};

enum OscInMatchMode {
  OSC_IN_MATCH_ANY = 0,
  OSC_IN_MATCH_EQUAL = 1,
  OSC_IN_MATCH_RANGE = 2
};

enum TimeTriggerType {
  TIME_ONCE = 0,
  TIME_DAILY = 1,
  TIME_WEEKLY = 2,
  TIME_INTERVAL = 3
};

enum OutputMode {
  OUT_INT = 0,
  OUT_FLOAT = 1,
  OUT_STRING = 2
};

enum TimeMode {
  TIME_NTP = 0,
  TIME_MANUAL = 1
};

enum OutputTarget {
  OUT_TARGET_OSC = 0,
  OUT_TARGET_REBOOT = 1,
  OUT_TARGET_RESET_COOLDOWNS = 2,
  OUT_TARGET_UDP = 3,
  OUT_TARGET_GPIO = 4,
  OUT_TARGET_HTTP = 5,
  OUT_TARGET_MQTT = 6,
  OUT_TARGET_DMX = 7,
  OUT_TARGET_DMX_PRESET = 8,
  OUT_TARGET_SET_VAR = 9
};

enum DmxProtocol {
  DMX_PROTO_ARTNET = 0,
  DMX_PROTO_SACN = 1
};

enum DmxDestinationMode {
  DMX_DEST_AUTO = 0,
  DMX_DEST_UNICAST = 1
};

enum DmxInputSourceMode {
  DMX_IN_SOURCE_RECENT = 0,
  DMX_IN_SOURCE_PREFER_ARTNET = 1,
  DMX_IN_SOURCE_PREFER_SACN = 2
};

enum GpioOutMode {
  GPIO_OUT_HIGH = 0,
  GPIO_OUT_LOW = 1,
  GPIO_OUT_PULSE = 2,
  GPIO_OUT_PWM = 3
};

enum HttpMethod {
  HTTPM_GET = 0,
  HTTPM_POST = 1
};

const int DEFAULT_ANALOG_PIN = 32;
const int DEFAULT_DIGITAL_PIN = 5;
const bool DEFAULT_INVERT = false;
const float DEFAULT_OUT_MIN = 0.0f;
const float DEFAULT_OUT_MAX = 100.0f;
const OutputMode DEFAULT_OUT_MODE = OUT_INT;
const bool DEFAULT_DIGITAL_ACTIVE_HIGH = false;
const bool DEFAULT_DIGITAL_PULLUP = true;
const char* DEFAULT_ON_STRING = "on";
const char* DEFAULT_OFF_STRING = "off";
#if USE_ETHERNET
const NetMode DEFAULT_NET_MODE = NET_ETHERNET;
#else
const NetMode DEFAULT_NET_MODE = NET_WIFI;
#endif

const int RESET_BUTTON_PIN = 34;
const unsigned long RESET_HOLD_MS = 5000;
const unsigned long RESET_ARM_MS = 500;

// Behavior
const int LOOP_DELAY_MS = 10;                  // read/send cycle
const int SAMPLES       = 8;                   // oversample to calm noise
const float ALPHA       = 0.30f;               // potentiometer smoothing 0..1
const float ALPHA_HC    = 0.20f;               // ultrasonic smoothing 0..1
const int DEAD_BAND     = 1;                   // percent hysteresis to stop flicker

const uint32_t DEFAULT_CLICK_DEBOUNCE_MS = 30;   // debounce for buttons
const uint32_t DEFAULT_MULTI_CLICK_GAP_MS = 350; // max gap between clicks
const uint32_t DEFAULT_LONG_PRESS_MS = 2000;     // long-press threshold
const uint8_t DEFAULT_ENC_STEPS = 2;             // encoder steps per output (fixed)
const bool DEFAULT_ANALOG_SMOOTHING = true;

const uint16_t LOCAL_PORT = 9000;              // Local UDP port
const float SNAP_PERCENT = 1.0f;               // snap edges to min/max
const uint32_t GPIO_PWM_FREQ_HZ = 1000;
const uint8_t GPIO_PWM_RES_BITS = 8;
const uint16_t GPIO_PWM_MAX = 255;

struct OscDevice {
  bool enabled;
  String name;
  IPAddress ip;
  uint16_t port;
  bool hbEnabled;
  String hbAddress;
  uint32_t hbMs;
};

struct SensorConfig {
  bool enabled;
  String name;
  SensorSource source;
  String oscInAddress;
  uint8_t oscInArgType;
  uint8_t oscInMatchMode;
  float oscInValue;
  float oscInMin;
  float oscInMax;
  String oscInString;
  uint16_t dmxInUniverse;
  uint16_t dmxInChannel;
  uint8_t dmxInMode;       // DmxInTriggerMode
  uint8_t dmxInThreshold;
  uint8_t dmxInRangeMin;
  uint8_t dmxInRangeMax;
  SensorType type;
  int pin;
  int encClkPin;
  int encDtPin;
  int encSwPin;
  bool encAppendSign;
  bool encSignArg;
  bool encInvert;
  uint8_t encSteps;
  bool invert;
  bool activeHigh;
  bool pullup;
  int hcTrigPin;
  int hcEchoPin;
  float hcMinCm;
  float hcMaxCm;
  bool cooldownEnabled;
  uint32_t cooldownMs;
  bool conditionEnabled;
  String conditionVar;
  uint8_t conditionOp;  // 0=eq, 1=neq, 2=gt, 3=lt, 4=gte, 5=lte
  float conditionValue;
  uint8_t outputCount;
  struct OutputConfig {
    OutputTarget target;
    uint32_t delayMs;
    uint8_t device;
    String oscAddress;
    String buttonAddress;
    float outMin;
    float outMax;
    OutputMode outMode;
    float buttonOutMin;
    float buttonOutMax;
    OutputMode buttonOutMode;
    bool sendMinOnRelease;
    String udpPayload;
    int8_t gpioPin;
    GpioOutMode gpioMode;
    uint32_t gpioPulseMs;
    bool gpioInvert;
    HttpMethod httpMethod;
    String httpUrl;
    String httpBody;
    String httpIp;
    uint16_t httpPort;
    uint8_t httpDevice;
    String mqttTopic;
    String mqttPayload;
    bool mqttRetain;
    uint8_t dmxProtocol;
    uint8_t dmxDest;
    uint8_t dmxArtNet;
    uint8_t dmxArtSubnet;
    uint8_t dmxArtUniverse;
    uint16_t dmxUniverse;
    String dmxChannels;
    uint8_t dmxPreset;
    uint32_t dmxFadeMs;
    String onString;
    String offString;
    String buttonOnString;
    String buttonOffString;
    String setVarName;
    float setVarValue;
  } outputs[MAX_OUTPUTS_PER_TRIGGER];
  TimeTriggerType timeType;
  uint16_t timeYear;
  uint8_t timeMonth;
  uint8_t timeDay;
  uint8_t timeHour;
  uint8_t timeMinute;
  uint8_t timeSecond;
  uint8_t weeklyMask; // bit0=Sun ... bit6=Sat
  uint32_t intervalSeconds;
};

struct Config {
  String hostname;
  NetMode netMode;
  bool oscInEnabled;
  uint16_t oscInPort;
  bool artnetEnabled;
  bool sacnEnabled;
  bool dmxInArtNetEnabled;
  bool dmxInSacnEnabled;
  uint8_t dmxInSourceMode;
  IPAddress clientIp;
  uint16_t clientPort;
  bool useStatic;
  IPAddress staticIp;
  IPAddress gateway;
  IPAddress subnet;
  IPAddress dns1;
  IPAddress dns2;
  String wifiSsid;
  String wifiPass;
  String username;
  bool passwordEnabled;
  String password;
  uint8_t testDevice;
  uint32_t clickDebounceMs;
  uint32_t multiClickGapMs;
  uint32_t longPressMs;
  bool analogSmoothing;
  uint8_t timeMode;
  String ntpServer;
  String timeZone;
  uint32_t manualEpoch;
  bool timeDisplay24h;
  bool mqttEnabled;
  uint8_t mqttDevice;
  String mqttHost;
  uint16_t mqttPort;
  String mqttUser;
  String mqttPass;
  String mqttClientId;
  String mqttBaseTopic;
  uint8_t triggerCount;
  uint8_t triggerOrder[MAX_TRIGGERS];
  uint8_t oscDeviceCount;
  uint8_t oscDeviceOrder[MAX_OSC_DEVICES];
  OscDevice oscDevices[MAX_OSC_DEVICES];
  SensorConfig sensors[MAX_TRIGGERS];
};

Config config;
String dmxPresetText[MAX_DMX_PRESETS];

float ema[MAX_TRIGGERS];     // smoothed ADC per sensor
int lastInt[MAX_TRIGGERS][MAX_OUTPUTS_PER_TRIGGER];   // last sent int value per output
float lastFloat[MAX_TRIGGERS][MAX_OUTPUTS_PER_TRIGGER]; // last sent float value per output
int8_t lastBool[MAX_TRIGGERS][MAX_OUTPUTS_PER_TRIGGER]; // last sent on/off state per output
int16_t dmxInLastValue[MAX_TRIGGERS];  // DMX-In: last raw value acted on, -1 = none yet
int8_t dmxInGateState[MAX_TRIGGERS];   // DMX-In threshold: -1 unknown, 0 below, 1 above
int8_t lastLevel[MAX_TRIGGERS]; // last raw level per sensor
bool toggleState[MAX_TRIGGERS]; // toggle state per sensor
int8_t lastEncA[MAX_TRIGGERS];
int8_t lastEncB[MAX_TRIGGERS];
int8_t lastEncState[MAX_TRIGGERS];
int8_t encAccum[MAX_TRIGGERS];
int8_t encUnitByTrigger[MAX_TRIGGERS];
uint32_t lastButtonTriggerMs[MAX_TRIGGERS];
uint32_t hbLastSent[MAX_OSC_DEVICES];
uint8_t clickCount[MAX_TRIGGERS];
bool multiPrevPressed[MAX_TRIGGERS];
bool multiLongFired[MAX_TRIGGERS];
unsigned long multiLastChangeMs[MAX_TRIGGERS];
unsigned long multiPressStartMs[MAX_TRIGGERS];
unsigned long multiLastReleaseMs[MAX_TRIGGERS];
unsigned long lastAnalogPrintMs[MAX_TRIGGERS];
time_t lastTimeFireEpoch[MAX_TRIGGERS];
uint32_t lastTimeFireDay[MAX_TRIGGERS];
uint32_t gpioPulseUntilByPin[40];
int8_t gpioPwmChannelByPin[40];
uint8_t nextGpioPwmChannel = 0;
int8_t gpioPulseEndLevelByPin[40];

struct PendingOutput {
  bool active;
  uint32_t fireAtMs;
  uint8_t sensorIndex;
  uint8_t outputIndex;
  float norm;
};
const int MAX_PENDING_OUTPUTS = 20;
PendingOutput pendingOutputs[MAX_PENDING_OUTPUTS];

const int MAX_GLOBAL_VARS = 16;
struct GlobalVar {
  String name;
  float value;
};
GlobalVar globalVars[MAX_GLOBAL_VARS];
int globalVarCount = 0;

static float getGlobalVar(const String& name) {
  for (int i = 0; i < globalVarCount; i++) {
    if (globalVars[i].name == name) return globalVars[i].value;
  }
  return 0.0f;
}

static void setGlobalVar(const String& name, float value) {
  for (int i = 0; i < globalVarCount; i++) {
    if (globalVars[i].name == name) { globalVars[i].value = value; return; }
  }
  if (globalVarCount < MAX_GLOBAL_VARS) {
    globalVars[globalVarCount].name = name;
    globalVars[globalVarCount].value = value;
    globalVarCount++;
  }
}

enum ConditionOp { COND_EQ=0, COND_NEQ=1, COND_GT=2, COND_LT=3, COND_GTE=4, COND_LTE=5 };

static bool checkCondition(const SensorConfig& s) {
  if (!s.conditionEnabled || s.conditionVar.length() == 0) return true;
  float v = getGlobalVar(s.conditionVar);
  switch (s.conditionOp) {
    case COND_EQ:  return v == s.conditionValue;
    case COND_NEQ: return v != s.conditionValue;
    case COND_GT:  return v > s.conditionValue;
    case COND_LT:  return v < s.conditionValue;
    case COND_GTE: return v >= s.conditionValue;
    case COND_LTE: return v <= s.conditionValue;
    default: return true;
  }
}

const uint16_t ARTNET_PORT = 6454;
const uint16_t SACN_PORT = 5568;
const int MAX_DMX_STREAMS = 24;
struct DmxStreamState {
  bool used;
  uint8_t protocol;
  uint8_t device;
  uint16_t universe;
  uint8_t sequence;
  uint8_t data[512];
};
DmxStreamState dmxStreams[MAX_DMX_STREAMS];
uint8_t sacnCid[16];
bool sacnCidReady = false;
uint8_t* dmxInData = nullptr;
uint8_t dmxInSource[MAX_DMX_PRESET_UNIVERSE];
uint32_t dmxInSeenMs[MAX_DMX_PRESET_UNIVERSE];
uint32_t dmxInArtSeenMs[MAX_DMX_PRESET_UNIVERSE];
uint32_t dmxInSacnSeenMs[MAX_DMX_PRESET_UNIVERSE];

WiFiUDP udp;
WiFiUDP oscUdp;
WiFiUDP artnetInUdp;
WiFiUDP sacnInUdp;
uint16_t oscListenPort = 0;
bool artnetListenActive = false;
bool sacnListenActive = false;
WiFiClient mqttNetClient;
PubSubClient mqttClient(mqttNetClient);
WebServer server(80);
Preferences prefs;
bool pendingNetApply = false;
bool pendingMdnsRestart = false;
unsigned long pendingApplyAt = 0;
unsigned long ntpLastAttemptMs = 0;
bool ntpPending = false;
bool wifiApMode = false;
unsigned long wifiReconnectStarted = 0;
DNSServer dnsServer;
unsigned long resetPressStartMs = 0;
bool resetButtonArmed = false;
unsigned long resetHighStartMs = 0;
unsigned long lastHeartbeatSentMs = 0;
const int LOG_LINES = 120;
String logLines[LOG_LINES];
int logHead = 0;
int logCount = 0;
String importPayload;
unsigned long mqttLastConnectAttemptMs = 0;
bool mqttReconnectRequested = false;
bool oscInputTokenOverrideActive = false;
String oscInputTokenValue = "";
float oscInputTokenNorm = 1.0f;
bool oscInputTokenState = true;

struct DmxFadePoint {
  uint16_t universe;
  uint16_t channel;
  uint8_t fromValue;
  uint8_t toValue;
};

struct DmxFadeState {
  bool active;
  uint8_t protocol;
  uint8_t dest;
  uint8_t device;
  uint32_t startMs;
  uint32_t durationMs;
  uint32_t lastFrameMs;
  uint16_t pointCount;
  uint8_t usedUniverseCount;
  uint16_t usedUniverses[MAX_DMX_PRESET_UNIVERSE];
  DmxFadePoint points[MAX_DMX_PRESET_POINTS];
};

DmxFadeState dmxFadeState;

struct OscInMessage {
  String address;
  bool hasArg;
  uint8_t argType;
  int32_t intValue;
  float floatValue;
  String stringValue;
};

static String ipToString(const IPAddress& ip);
static bool parseIpString(const String& s, IPAddress& out);
static bool isValidIp(const IPAddress& ip);
static uint8_t sanitizeDmxProtocol(int value);
static uint8_t sanitizeDmxDest(int value);
static uint8_t sanitizeDmxArtNet(int value);
static uint8_t sanitizeDmxArtSubnet(int value);
static uint8_t sanitizeDmxArtUniverse(int value);
static uint8_t sanitizeDmxPresetId(int value);
static uint32_t sanitizeDmxFadeMs(int value);
static uint8_t sanitizeDmxInputSourceMode(int value);
static OutputTarget sanitizeOutputTarget(int value);
static uint16_t encodeArtNetPortAddressOneBased(uint8_t net, uint8_t subnet, uint8_t universe);
static void decodeArtNetPortAddressOneBased(uint16_t oneBased, uint8_t& net, uint8_t& subnet, uint8_t& universe);
static String buildApSsid();
static void startWifiAp();
static void addLog(const String& msg);
static bool parseOrderList(const String& s, uint8_t* order, uint8_t& count);
static String buildOrderList(const uint8_t* order, uint8_t count);
static int findUnusedTrigger();
static bool isAllowedAnalogPin(int pin);
static bool isAllowedDigitalPin(int pin);
static int defaultSensorPin(int index);
static String jsonEscape(const String& s);
static void clearSensorKeys(int index);
static void clearDeviceKeys(int index);
static void removeIfExists(const String& key);
static void applyTimeConfig();
static String formatTimeLocal();
static String formatDateLocal();
static String formatTimeLocalInput();
static bool parseDateTimeLocal(const String& dateStr, const String& timeStr, uint32_t& epochOut);
static void pollNtpSync();
static bool parseDateYMD(const String& dateStr, uint16_t& y, uint8_t& m, uint8_t& d);
static bool parseTimeHMS(const String& timeStr, uint8_t& h, uint8_t& m, uint8_t& s);
static void sendOutputNow(const SensorConfig& sensor, const SensorConfig::OutputConfig& out, const String& addr, float norm);
static void queueOrSendOutput(const SensorConfig& sensor, const SensorConfig::OutputConfig& out, const String& addr, float norm);
static bool handleTimeTrigger(int index, SensorConfig& s);
static int compareVersion(const String& a, const String& b);
static String exportConfigJson();
static bool applyConfigFromJson(const JsonObject& cfg, String& err);
static String defaultMqttClientId();
static String sanitizeMqttTopicSegment(const String& raw);
static String buildDefaultMqttTopic(const SensorConfig& sensor, int triggerIndex, int outputIndex);
static bool isAutoMqttTopic(const String& topic,
                            const SensorConfig& previousSensor,
                            const SensorConfig& currentSensor,
                            int triggerIndex,
                            int outputIndex);
static void configureMqttClient();
static bool ensureMqttConnected();
static bool sendMqttMessage(const String& topic, const String& payload, bool retain);
static void configureUdpListener();
static bool decodeArtNetDmxPacket(const uint8_t* data, size_t len, uint16_t& universeOneBased, uint8_t* dmxData, uint16_t& dmxLen);
static bool decodeSacnDmxPacket(const uint8_t* data, size_t len, uint16_t& universeOneBased, uint8_t* dmxData, uint16_t& dmxLen);
static uint8_t* getDmxInputUniverseBuffer(int index);
static void updateDmxInputUniverse(uint8_t protocol, uint16_t universeOneBased, const uint8_t* dmxData, uint16_t dmxLen);
static bool getLiveDmxUniverseValues(uint16_t universeOneBased, uint8_t viewMode, uint8_t outData[512], uint8_t& selectedSource, uint32_t& selectedAgeMs);
static int parseDmxPresetText(const String& text, DmxFadePoint* points, int maxPoints, String* normalizedOut, String& err);
static bool parseDmxPresetRow(const String& line, int& universe, String& channelsExpr, int& value);
static void fillDmxUniverseValues(const String& text, int universe, int16_t values[512]);
static String removeDmxUniverseRows(const String& text, int universe);
static String serializeDmxUniverseRows(int universe, const int16_t values[512]);
static int totalDmxPresetPointsExcluding(int excludeIndex);
static bool triggerDmxPreset(const SensorConfig::OutputConfig& out);
static void tickDmxPresetFade();
static void handleDmxPresetEditor();
static bool matchOscAddressPattern(const String& pattern, const String& value);
static bool parseOscInPacket(const uint8_t* data, size_t len, OscInMessage& msg);
static bool matchOscInTrigger(const SensorConfig& s, const OscInMessage& msg, float& normOut, String& valueOut);
static void processOscInput();
static void processDmxInput();

static int readAveragedADC(int pin) {
  uint32_t acc = 0;
  for (int i = 0; i < SAMPLES; i++) {
    acc += analogRead(pin);
    delayMicroseconds(50);
  }
  return acc / SAMPLES;   // 0..4095
}

static bool waitForNetwork(unsigned long timeoutMs) {
  unsigned long start = millis();
#if USE_ETHERNET
  if (config.netMode == NET_ETHERNET) {
    while (!ETH.linkUp() || !isValidIp(ETH.localIP())) {
      delay(100);
      if (millis() - start > timeoutMs) return false;
    }
    return true;
  }
#endif
  // WiFi mode (with or without Ethernet compiled in)
  while (WiFi.status() != WL_CONNECTED || !isValidIp(WiFi.localIP())) {
    delay(100);
    if (millis() - start > timeoutMs) return false;
  }
  return true;
}

static float clampFloat(float v, float minV, float maxV) {
  if (v < minV) return minV;
  if (v > maxV) return maxV;
  return v;
}

static float readAnalogNormalized(int index, const SensorConfig& sensor) {
  int raw = readAveragedADC(sensor.pin);                 // 0..4095
  if (millis() - lastAnalogPrintMs[index] >= 500) {
    lastAnalogPrintMs[index] = millis();
    Serial.print("ADC GPIO");
    Serial.print(sensor.pin);
    Serial.print(" raw: ");
    Serial.println(raw);
    addLog(String("ADC GPIO") + String(sensor.pin) + " raw " + String(raw));
  }
  float norm = 0.0f;
  if (config.analogSmoothing) {
    if (ema[index] < 0) ema[index] = raw;
    ema[index] = ALPHA * raw + (1.0f - ALPHA) * ema[index];
    norm = ema[index] / 4095.0f;
  } else {
    norm = raw / 4095.0f;
  }
  norm = clampFloat(norm, 0.0f, 1.0f);
  if (sensor.invert) norm = 1.0f - norm;
  return norm;
}

static float readHcSr04Normalized(int index, const SensorConfig& sensor) {
  digitalWrite(sensor.hcTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(sensor.hcTrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(sensor.hcTrigPin, LOW);

  unsigned long duration = pulseIn(sensor.hcEchoPin, HIGH, 25000); // ~4m max
  float minCm = sensor.hcMinCm;
  float maxCm = sensor.hcMaxCm;
  bool invertRange = false;
  if (maxCm < minCm) {
    float tmp = minCm;
    minCm = maxCm;
    maxCm = tmp;
    invertRange = true;
  }
  if (maxCm == minCm) {
    return 0.0f;
  }

  // No echo usually means "out of range / nothing in front".
  // Map that to far distance endpoint instead of freezing the last value.
  float dist = (duration == 0) ? (maxCm + 1.0f) : (duration / 58.0f);
  float norm = (dist - minCm) / (maxCm - minCm);
  if (invertRange) norm = 1.0f - norm;
  norm = clampFloat(norm, 0.0f, 1.0f);
  if (sensor.invert) norm = 1.0f - norm;

  if (config.analogSmoothing) {
    float raw = norm * 4095.0f;
    if (ema[index] < 0) ema[index] = raw;
    ema[index] = ALPHA_HC * raw + (1.0f - ALPHA_HC) * ema[index];
    norm = ema[index] / 4095.0f;
    norm = clampFloat(norm, 0.0f, 1.0f);
  }
  return norm;
}

static bool readDigitalActive(const SensorConfig& sensor) {
  int value = digitalRead(sensor.pin);
  bool isOn = sensor.activeHigh ? (value == HIGH) : (value == LOW);
  return isOn;
}

static float mapOutput(const SensorConfig::OutputConfig& outCfg, float norm) {
  if (norm <= 0.015f) return outCfg.outMin;
  if (norm >= 0.985f) return outCfg.outMax;
  float out = outCfg.outMin + norm * (outCfg.outMax - outCfg.outMin);
  if (outCfg.outMin < outCfg.outMax) {
    return clampFloat(out, outCfg.outMin, outCfg.outMax);
  }
  return clampFloat(out, outCfg.outMax, outCfg.outMin);
}

static float mapOutputRange(float norm, float minV, float maxV) {
  if (norm <= 0.015f) return minV;
  if (norm >= 0.985f) return maxV;
  float out = minV + norm * (maxV - minV);
  if (minV < maxV) {
    return clampFloat(out, minV, maxV);
  }
  return clampFloat(out, maxV, minV);
}

static float snapOutput(const SensorConfig::OutputConfig& outCfg, float value) {
  float minV = outCfg.outMin;
  float maxV = outCfg.outMax;
  float span = fabsf(maxV - minV);
  if (span <= 0.0f) return value;
  float snap = (SNAP_PERCENT / 100.0f) * span;
  float low = (minV < maxV) ? minV : maxV;
  float high = (minV < maxV) ? maxV : minV;
  if (value <= low + snap) return minV;
  if (value >= high - snap) return maxV;
  return value;
}

static float snapOutputRange(float value, float minV, float maxV) {
  float span = fabsf(maxV - minV);
  if (span <= 0.0f) return value;
  float snap = (SNAP_PERCENT / 100.0f) * span;
  float low = (minV < maxV) ? minV : maxV;
  float high = (minV < maxV) ? maxV : minV;
  if (value <= low + snap) return minV;
  if (value >= high - snap) return maxV;
  return value;
}

static String sanitizeHostname(const String& raw) {
  String out;
  out.reserve(32);
  for (size_t i = 0; i < raw.length() && out.length() < 32; i++) {
    char c = raw[i];
    if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '-') {
      out += c;
    }
  }
  if (out.length() == 0) {
    out = DEFAULT_HOSTNAME;
  }
  return out;
}

static String defaultMqttClientId() {
  uint64_t chip = ESP.getEfuseMac();
  char buf[32];
  snprintf(buf, sizeof(buf), "stagemod-%06llx", static_cast<unsigned long long>(chip & 0xFFFFFFULL));
  return String(buf);
}

static String sanitizeMqttTopicSegment(const String& raw) {
  String in = raw;
  in.trim();
  String out;
  out.reserve(in.length());
  bool wroteSep = false;
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    bool ok = ((c >= 'a' && c <= 'z') ||
               (c >= 'A' && c <= 'Z') ||
               (c >= '0' && c <= '9') ||
               c == '_' || c == '-');
    if (ok) {
      out += c;
      wroteSep = false;
      continue;
    }
    if (out.length() > 0 && !wroteSep) {
      out += '_';
      wroteSep = true;
    }
  }
  while (out.startsWith("_")) out.remove(0, 1);
  while (out.endsWith("_")) out.remove(out.length() - 1, 1);
  return out;
}

static String buildDefaultMqttTopic(const SensorConfig& sensor, int triggerIndex, int outputIndex) {
  String base = config.mqttBaseTopic;
  base.trim();
  if (base.length() == 0) base = DEFAULT_MQTT_BASE_TOPIC;
  while (base.endsWith("/")) base.remove(base.length() - 1, 1);

  String triggerSegment = sanitizeMqttTopicSegment(sensor.name);
  if (triggerSegment.length() == 0) {
    triggerSegment = String("trigger_") + String(triggerIndex + 1);
  }
  return base + "/" + triggerSegment + "/output/" + String(outputIndex + 1);
}

static bool isAutoMqttTopic(const String& topic,
                            const SensorConfig& previousSensor,
                            const SensorConfig& currentSensor,
                            int triggerIndex,
                            int outputIndex) {
  String t = topic;
  t.trim();
  if (t.length() == 0) return true;

  String trigNum = String(triggerIndex + 1);
  String outNum = String(outputIndex + 1);
  String legacyNoOut = "trigger/" + trigNum;
  String legacyWithOut = legacyNoOut + "/output/" + outNum;
  if (t == legacyNoOut || t == legacyWithOut ||
      t.endsWith("/" + legacyNoOut) || t.endsWith("/" + legacyWithOut)) {
    return true;
  }

  String prevSeg = sanitizeMqttTopicSegment(previousSensor.name);
  if (prevSeg.length() > 0) {
    String prevWithOut = prevSeg + "/output/" + outNum;
    if (t == prevWithOut || t.endsWith("/" + prevWithOut)) return true;
  }

  String curSeg = sanitizeMqttTopicSegment(currentSensor.name);
  if (curSeg.length() > 0) {
    String curWithOut = curSeg + "/output/" + outNum;
    if (t == curWithOut || t.endsWith("/" + curWithOut)) return true;
  }

  return false;
}

static IPAddress getLocalIp() {
  IPAddress ip;
#if USE_ETHERNET
  if (config.netMode == NET_ETHERNET) {
    ip = ETH.localIP();
    if (ip[0] != 0) return ip;
  }
#endif
  if (WiFi.status() == WL_CONNECTED) return WiFi.localIP();
  return WiFi.softAPIP();
}

static bool isValidIp(const IPAddress& ip) {
  if (ip[0] == 0) return false;
  if (ip[0] == 169 && ip[1] == 254) return false;
  return true;
}

static String buildApSsid() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
  char buf[32];
  snprintf(buf, sizeof(buf), "StageMod-Setup-%02X%02X", mac[4], mac[5]);
  return String(buf);
}

static String jsonEscape(const String& s) {
  String out;
  out.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (c == '\n') {
      out += "\\n";
    } else if (c == '\r') {
      out += "\\r";
    } else if (c == '\t') {
      out += "\\t";
    } else {
      out += c;
    }
  }
  return out;
}

static String htmlEscape(const String& s) {
  String out;
  out.reserve(s.length() + 16);
  for (char c : s) {
    switch (c) {
      case '&':  out += "&amp;";  break;
      case '<':  out += "&lt;";   break;
      case '>':  out += "&gt;";   break;
      case '"':  out += "&quot;"; break;
      case '\'': out += "&#39;";  break;
      default:   out += c;        break;
    }
  }
  return out;
}

static int compareVersion(const String& a, const String& b) {
  int ap[3] = {0, 0, 0};
  int bp[3] = {0, 0, 0};
  auto parse = [](const String& v, int* out) {
    int idx = 0;
    int val = 0;
    for (size_t i = 0; i < v.length(); i++) {
      char c = v[i];
      if (c >= '0' && c <= '9') {
        val = val * 10 + (c - '0');
      } else if (c == '.') {
        if (idx < 3) out[idx++] = val;
        val = 0;
      }
    }
    if (idx < 3) out[idx++] = val;
    while (idx < 3) out[idx++] = 0;
  };
  parse(a, ap);
  parse(b, bp);
  for (int i = 0; i < 3; i++) {
    if (ap[i] < bp[i]) return -1;
    if (ap[i] > bp[i]) return 1;
  }
  return 0;
}

static String exportConfigJson() {
  DynamicJsonDocument doc(32768);
  doc["version"] = FIRMWARE_VERSION;
  JsonObject c = doc.createNestedObject("config");
  c["hostname"] = config.hostname;
  c["netMode"] = static_cast<int>(config.netMode);
  c["wifiSsid"] = config.wifiSsid;
  c["useStatic"] = config.useStatic;
  c["staticIp"] = ipToString(config.staticIp);
  c["gateway"] = ipToString(config.gateway);
  c["subnet"] = ipToString(config.subnet);
  c["dns1"] = ipToString(config.dns1);
  c["dns2"] = ipToString(config.dns2);
  c["passwordEnabled"] = config.passwordEnabled;
  c["username"] = config.username;
  c["testDevice"] = config.testDevice;
  c["clickDebounceMs"] = config.clickDebounceMs;
  c["multiClickGapMs"] = config.multiClickGapMs;
  c["longPressMs"] = config.longPressMs;
  c["analogSmoothing"] = config.analogSmoothing;
  c["timeMode"] = config.timeMode;
  c["ntpServer"] = config.ntpServer;
  c["timeZone"] = config.timeZone;
  c["manualEpoch"] = config.manualEpoch;
  c["timeDisplay24h"] = config.timeDisplay24h;
  c["oscInEnabled"] = config.oscInEnabled;
  c["oscInPort"] = config.oscInPort;
  c["artnetEnabled"] = config.artnetEnabled;
  c["sacnEnabled"] = config.sacnEnabled;
  c["dmxInArtNetEnabled"] = config.dmxInArtNetEnabled;
  c["dmxInSacnEnabled"] = config.dmxInSacnEnabled;
  c["dmxInSourceMode"] = config.dmxInSourceMode;
  c["mqttEnabled"] = config.mqttEnabled;
  c["mqttDevice"] = config.mqttDevice;
  c["mqttHost"] = config.mqttHost;
  c["mqttPort"] = config.mqttPort;
  c["mqttUser"] = config.mqttUser;
  c["mqttClientId"] = config.mqttClientId;
  c["mqttBaseTopic"] = config.mqttBaseTopic;

  c["oscDeviceCount"] = config.oscDeviceCount;
  JsonArray devOrder = c.createNestedArray("oscDeviceOrder");
  for (int i = 0; i < config.oscDeviceCount; i++) {
    devOrder.add(config.oscDeviceOrder[i]);
  }
  JsonArray devs = c.createNestedArray("oscDevices");
  for (int i = 0; i < MAX_OSC_DEVICES; i++) {
    const OscDevice& d = config.oscDevices[i];
    JsonObject od = devs.createNestedObject();
    od["index"] = i;
    od["enabled"] = d.enabled;
    od["name"] = d.name;
    od["ip"] = ipToString(d.ip);
    od["port"] = d.port;
    od["hbEnabled"] = d.hbEnabled;
    od["hbAddress"] = d.hbAddress;
    od["hbMs"] = d.hbMs;
  }

  c["triggerCount"] = config.triggerCount;
  JsonArray trOrder = c.createNestedArray("triggerOrder");
  for (int i = 0; i < config.triggerCount; i++) {
    trOrder.add(config.triggerOrder[i]);
  }
  JsonArray sensors = c.createNestedArray("sensors");
  for (int ti = 0; ti < config.triggerCount; ti++) {
    int i = config.triggerOrder[ti];
    const SensorConfig& s = config.sensors[i];
    JsonObject so = sensors.createNestedObject();
    so["index"] = i;
    so["enabled"] = s.enabled;
    so["name"] = s.name;
    so["source"] = static_cast<int>(s.source);
    so["oscInAddress"] = s.oscInAddress;
    so["oscInArgType"] = s.oscInArgType;
    so["oscInMatchMode"] = s.oscInMatchMode;
    so["oscInValue"] = s.oscInValue;
    so["oscInMin"] = s.oscInMin;
    so["oscInMax"] = s.oscInMax;
    so["oscInString"] = s.oscInString;
  so["dmxInUniverse"] = s.dmxInUniverse;
  so["dmxInChannel"] = s.dmxInChannel;
  so["dmxInMode"] = s.dmxInMode;
  so["dmxInThreshold"] = s.dmxInThreshold;
  so["dmxInRangeMin"] = s.dmxInRangeMin;
  so["dmxInRangeMax"] = s.dmxInRangeMax;
    so["type"] = static_cast<int>(s.type);
    so["pin"] = s.pin;
    so["encClkPin"] = s.encClkPin;
    so["encDtPin"] = s.encDtPin;
    so["encSwPin"] = s.encSwPin;
    so["encAppendSign"] = s.encAppendSign;
    so["encSignArg"] = s.encSignArg;
    so["encInvert"] = s.encInvert;
    so["invert"] = s.invert;
    so["activeHigh"] = s.activeHigh;
    so["pullup"] = s.pullup;
    so["hcTrigPin"] = s.hcTrigPin;
    so["hcEchoPin"] = s.hcEchoPin;
    so["hcMinCm"] = s.hcMinCm;
    so["hcMaxCm"] = s.hcMaxCm;
    so["cooldownEnabled"] = s.cooldownEnabled;
    so["cooldownMs"] = s.cooldownMs;
    so["conditionEnabled"] = s.conditionEnabled;
    so["conditionVar"] = s.conditionVar;
    so["conditionOp"] = s.conditionOp;
    so["conditionValue"] = s.conditionValue;
    so["outputCount"] = s.outputCount;
    JsonArray outs = so.createNestedArray("outputs");
    for (int o = 0; o < s.outputCount && o < MAX_OUTPUTS_PER_TRIGGER; o++) {
      const SensorConfig::OutputConfig& out = s.outputs[o];
      JsonObject oo = outs.createNestedObject();
      oo["index"] = o;
      oo["target"] = static_cast<int>(out.target);
      oo["delayMs"] = out.delayMs;
      oo["device"] = out.device;
      oo["oscAddress"] = out.oscAddress;
      oo["buttonAddress"] = out.buttonAddress;
      oo["outMin"] = out.outMin;
      oo["outMax"] = out.outMax;
      oo["outMode"] = static_cast<int>(out.outMode);
      oo["buttonOutMin"] = out.buttonOutMin;
      oo["buttonOutMax"] = out.buttonOutMax;
      oo["buttonOutMode"] = static_cast<int>(out.buttonOutMode);
      oo["sendMinOnRelease"] = out.sendMinOnRelease;
      oo["udpPayload"] = out.udpPayload;
      oo["gpioPin"] = out.gpioPin;
      oo["gpioMode"] = static_cast<int>(out.gpioMode);
      oo["gpioPulseMs"] = out.gpioPulseMs;
      oo["gpioInvert"] = out.gpioInvert;
      oo["httpMethod"] = static_cast<int>(out.httpMethod);
      oo["httpUrl"] = out.httpUrl;
      oo["httpBody"] = out.httpBody;
      oo["httpIp"] = out.httpIp;
      oo["httpPort"] = out.httpPort;
      oo["httpDevice"] = out.httpDevice;
      oo["mqttTopic"] = out.mqttTopic;
      oo["mqttPayload"] = out.mqttPayload;
      oo["mqttRetain"] = out.mqttRetain;
      oo["dmxProtocol"] = out.dmxProtocol;
      oo["dmxDest"] = out.dmxDest;
      oo["dmxArtNet"] = out.dmxArtNet;
      oo["dmxArtSubnet"] = out.dmxArtSubnet;
      oo["dmxArtUniverse"] = out.dmxArtUniverse;
      oo["dmxUniverse"] = out.dmxUniverse;
      oo["dmxChannels"] = out.dmxChannels;
      oo["dmxPreset"] = out.dmxPreset;
      oo["dmxFadeMs"] = out.dmxFadeMs;
      oo["onString"] = out.onString;
      oo["offString"] = out.offString;
      oo["buttonOnString"] = out.buttonOnString;
      oo["buttonOffString"] = out.buttonOffString;
      oo["setVarName"] = out.setVarName;
      oo["setVarValue"] = out.setVarValue;
    }
    so["timeType"] = static_cast<int>(s.timeType);
    so["timeYear"] = s.timeYear;
    so["timeMonth"] = s.timeMonth;
    so["timeDay"] = s.timeDay;
    so["timeHour"] = s.timeHour;
    so["timeMinute"] = s.timeMinute;
    so["timeSecond"] = s.timeSecond;
    so["weeklyMask"] = s.weeklyMask;
    so["intervalSeconds"] = s.intervalSeconds;
  }

  JsonArray dmxPresets = c.createNestedArray("dmxPresets");
  for (int i = 0; i < MAX_DMX_PRESETS; i++) {
    if (dmxPresetText[i].length() == 0) continue;
    JsonObject po = dmxPresets.createNestedObject();
    po["index"] = i;
    po["text"] = dmxPresetText[i];
  }

  String out;
  serializeJson(doc, out);
  return out;
}

static bool applyConfigFromJson(const JsonObject& cfg, String& err) {
  if (cfg.isNull()) {
    err = "Missing config object.";
    return false;
  }

  if (cfg.containsKey("hostname")) config.hostname = sanitizeHostname(String(cfg["hostname"].as<const char*>()));
  if (cfg.containsKey("netMode")) { int m = cfg["netMode"].as<int>(); if (m == NET_ETHERNET || m == NET_WIFI) config.netMode = static_cast<NetMode>(m); }
  if (cfg.containsKey("wifiSsid")) config.wifiSsid = String(cfg["wifiSsid"].as<const char*>());
  if (cfg.containsKey("wifiPass")) { String p = String(cfg["wifiPass"].as<const char*>()); if (p.length() > 0) config.wifiPass = p; }
  if (cfg.containsKey("useStatic")) config.useStatic = cfg["useStatic"];
  if (cfg.containsKey("staticIp")) { IPAddress p; if (parseIpString(String(cfg["staticIp"].as<const char*>()), p)) config.staticIp = p; }
  if (cfg.containsKey("gateway")) { IPAddress p; if (parseIpString(String(cfg["gateway"].as<const char*>()), p)) config.gateway = p; }
  if (cfg.containsKey("subnet")) { IPAddress p; if (parseIpString(String(cfg["subnet"].as<const char*>()), p)) config.subnet = p; }
  if (cfg.containsKey("dns1")) { IPAddress p; if (parseIpString(String(cfg["dns1"].as<const char*>()), p)) config.dns1 = p; }
  if (cfg.containsKey("dns2")) { IPAddress p; if (parseIpString(String(cfg["dns2"].as<const char*>()), p)) config.dns2 = p; }
  if (cfg.containsKey("passwordEnabled")) config.passwordEnabled = cfg["passwordEnabled"];
  if (cfg.containsKey("username")) config.username = String(cfg["username"].as<const char*>());
  if (cfg.containsKey("password")) { String p = String(cfg["password"].as<const char*>()); if (p.length() > 0) config.password = p; }
  if (cfg.containsKey("testDevice")) {
    int v = cfg["testDevice"];
    if (v >= 0 && v < MAX_OSC_DEVICES) config.testDevice = static_cast<uint8_t>(v);
  }
  if (cfg.containsKey("clickDebounceMs")) config.clickDebounceMs = cfg["clickDebounceMs"];
  if (cfg.containsKey("multiClickGapMs")) config.multiClickGapMs = cfg["multiClickGapMs"];
  if (cfg.containsKey("longPressMs")) config.longPressMs = cfg["longPressMs"];
  if (cfg.containsKey("analogSmoothing")) config.analogSmoothing = cfg["analogSmoothing"];
  if (cfg.containsKey("timeMode")) {
    int mode = cfg["timeMode"];
    if (mode == TIME_NTP || mode == TIME_MANUAL) config.timeMode = mode;
  }
  if (cfg.containsKey("ntpServer")) config.ntpServer = String(cfg["ntpServer"].as<const char*>());
  if (cfg.containsKey("timeZone")) config.timeZone = String(cfg["timeZone"].as<const char*>());
  if (cfg.containsKey("manualEpoch")) config.manualEpoch = cfg["manualEpoch"];
  if (cfg.containsKey("timeDisplay24h")) config.timeDisplay24h = cfg["timeDisplay24h"];
  if (cfg.containsKey("oscInEnabled")) config.oscInEnabled = cfg["oscInEnabled"];
  if (cfg.containsKey("oscInPort")) {
    uint16_t p = cfg["oscInPort"];
    if (p > 0) config.oscInPort = p;
  }
  if (cfg.containsKey("artnetEnabled")) config.artnetEnabled = cfg["artnetEnabled"];
  if (cfg.containsKey("sacnEnabled")) config.sacnEnabled = cfg["sacnEnabled"];
  if (cfg.containsKey("dmxInArtNetEnabled")) config.dmxInArtNetEnabled = cfg["dmxInArtNetEnabled"];
  if (cfg.containsKey("dmxInSacnEnabled")) config.dmxInSacnEnabled = cfg["dmxInSacnEnabled"];
  if (cfg.containsKey("dmxInSourceMode")) config.dmxInSourceMode = sanitizeDmxInputSourceMode(cfg["dmxInSourceMode"]);
  if (cfg.containsKey("mqttEnabled")) config.mqttEnabled = cfg["mqttEnabled"];
  if (cfg.containsKey("mqttDevice")) {
    int d = cfg["mqttDevice"];
    if ((d >= 0 && d < MAX_OSC_DEVICES) || d == MQTT_DEVICE_CUSTOM) {
      config.mqttDevice = static_cast<uint8_t>(d);
    }
  }
  if (cfg.containsKey("mqttHost")) config.mqttHost = String(cfg["mqttHost"].as<const char*>());
  if (cfg.containsKey("mqttPort")) {
    uint16_t p = cfg["mqttPort"];
    if (p > 0) config.mqttPort = p;
  }
  if (cfg.containsKey("mqttUser")) config.mqttUser = String(cfg["mqttUser"].as<const char*>());
  if (cfg.containsKey("mqttPass")) { String p = String(cfg["mqttPass"].as<const char*>()); if (p.length() > 0) config.mqttPass = p; }
  if (cfg.containsKey("mqttClientId")) config.mqttClientId = String(cfg["mqttClientId"].as<const char*>());
  if (cfg.containsKey("mqttBaseTopic")) config.mqttBaseTopic = String(cfg["mqttBaseTopic"].as<const char*>());

  if (cfg.containsKey("oscDeviceCount")) {
    int count = cfg["oscDeviceCount"];
    if (count < 1) count = 1;
    if (count > MAX_OSC_DEVICES) count = MAX_OSC_DEVICES;
    config.oscDeviceCount = static_cast<uint8_t>(count);
  }
  JsonArray devOrder = cfg["oscDeviceOrder"].as<JsonArray>();
  if (!devOrder.isNull()) {
    uint8_t count = 0;
    for (JsonVariant v : devOrder) {
      int idx = v.as<int>();
      if (idx >= 0 && idx < MAX_OSC_DEVICES) config.oscDeviceOrder[count++] = static_cast<uint8_t>(idx);
      if (count >= config.oscDeviceCount) break;
    }
    if (count == 0) {
      for (int i = 0; i < config.oscDeviceCount; i++) config.oscDeviceOrder[i] = i;
    }
  }
  JsonArray devs = cfg["oscDevices"].as<JsonArray>();
  if (!devs.isNull()) {
    for (JsonObject d : devs) {
      int idx = d["index"] | -1;
      if (idx < 0 || idx >= MAX_OSC_DEVICES) continue;
      OscDevice od = config.oscDevices[idx];
      if (d.containsKey("enabled")) od.enabled = d["enabled"];
      if (d.containsKey("name")) od.name = String(d["name"].as<const char*>());
      if (d.containsKey("ip")) {
        IPAddress ip;
        if (parseIpString(String(d["ip"].as<const char*>()), ip)) od.ip = ip;
      }
      if (d.containsKey("port")) od.port = d["port"];
      if (d.containsKey("hbEnabled")) od.hbEnabled = d["hbEnabled"];
      if (d.containsKey("hbAddress")) od.hbAddress = String(d["hbAddress"].as<const char*>());
      if (d.containsKey("hbMs")) od.hbMs = d["hbMs"];
      config.oscDevices[idx] = od;
    }
  }

  if (cfg.containsKey("triggerCount")) {
    int count = cfg["triggerCount"];
    if (count < 1) count = 1;
    if (count > MAX_TRIGGERS) count = MAX_TRIGGERS;
    config.triggerCount = static_cast<uint8_t>(count);
  }
  JsonArray trOrder = cfg["triggerOrder"].as<JsonArray>();
  if (!trOrder.isNull()) {
    uint8_t count = 0;
    for (JsonVariant v : trOrder) {
      int idx = v.as<int>();
      if (idx >= 0 && idx < MAX_TRIGGERS) config.triggerOrder[count++] = static_cast<uint8_t>(idx);
      if (count >= config.triggerCount) break;
    }
    if (count == 0) {
      for (int i = 0; i < config.triggerCount; i++) config.triggerOrder[i] = i;
    }
  }
  JsonArray sensors = cfg["sensors"].as<JsonArray>();
  if (!sensors.isNull()) {
    for (JsonObject so : sensors) {
      int idx = so["index"] | -1;
      if (idx < 0 || idx >= MAX_TRIGGERS) continue;
      SensorConfig s = config.sensors[idx];
      if (so.containsKey("enabled")) s.enabled = so["enabled"];
      if (so.containsKey("name")) s.name = String(so["name"].as<const char*>());
      if (so.containsKey("source")) s.source = static_cast<SensorSource>(so["source"].as<int>());
      if (so.containsKey("oscInAddress")) s.oscInAddress = String(so["oscInAddress"].as<const char*>());
      if (so.containsKey("oscInArgType")) s.oscInArgType = so["oscInArgType"];
      if (so.containsKey("oscInMatchMode")) s.oscInMatchMode = so["oscInMatchMode"];
      if (so.containsKey("oscInValue")) s.oscInValue = so["oscInValue"];
      if (so.containsKey("oscInMin")) s.oscInMin = so["oscInMin"];
      if (so.containsKey("oscInMax")) s.oscInMax = so["oscInMax"];
      if (so.containsKey("oscInString")) s.oscInString = String(so["oscInString"].as<const char*>());
  if (so.containsKey("dmxInUniverse")) { int v = so["dmxInUniverse"]; if (v >= 1 && v <= MAX_DMX_PRESET_UNIVERSE) s.dmxInUniverse = static_cast<uint16_t>(v); }
  if (so.containsKey("dmxInChannel")) { int v = so["dmxInChannel"]; if (v >= 1 && v <= 512) s.dmxInChannel = static_cast<uint16_t>(v); }
  if (so.containsKey("dmxInMode")) { int v = so["dmxInMode"]; if (v >= DMX_IN_TRIG_LEVEL && v <= DMX_IN_TRIG_RANGE) s.dmxInMode = static_cast<uint8_t>(v); }
  if (so.containsKey("dmxInThreshold")) { int v = so["dmxInThreshold"]; if (v >= 0 && v <= 255) s.dmxInThreshold = static_cast<uint8_t>(v); }
  if (so.containsKey("dmxInRangeMin")) { int v = so["dmxInRangeMin"]; if (v >= 0 && v <= 255) s.dmxInRangeMin = static_cast<uint8_t>(v); }
  if (so.containsKey("dmxInRangeMax")) { int v = so["dmxInRangeMax"]; if (v >= 0 && v <= 255) s.dmxInRangeMax = static_cast<uint8_t>(v); }
      if (so.containsKey("type")) s.type = static_cast<SensorType>(so["type"].as<int>());
      if (so.containsKey("pin")) s.pin = so["pin"];
      if (so.containsKey("encClkPin")) s.encClkPin = so["encClkPin"];
      if (so.containsKey("encDtPin")) s.encDtPin = so["encDtPin"];
      if (so.containsKey("encSwPin")) s.encSwPin = so["encSwPin"];
      if (so.containsKey("encAppendSign")) s.encAppendSign = so["encAppendSign"];
      if (so.containsKey("encSignArg")) s.encSignArg = so["encSignArg"];
      if (so.containsKey("encInvert")) s.encInvert = so["encInvert"];
      if (so.containsKey("invert")) s.invert = so["invert"];
      if (so.containsKey("activeHigh")) s.activeHigh = so["activeHigh"];
      if (so.containsKey("pullup")) s.pullup = so["pullup"];
      if (so.containsKey("hcTrigPin")) s.hcTrigPin = so["hcTrigPin"];
      if (so.containsKey("hcEchoPin")) s.hcEchoPin = so["hcEchoPin"];
      if (so.containsKey("hcMinCm")) s.hcMinCm = so["hcMinCm"];
      if (so.containsKey("hcMaxCm")) s.hcMaxCm = so["hcMaxCm"];
      if (so.containsKey("cooldownEnabled")) s.cooldownEnabled = so["cooldownEnabled"];
      if (so.containsKey("cooldownMs")) s.cooldownMs = so["cooldownMs"];
      if (so.containsKey("conditionEnabled")) s.conditionEnabled = so["conditionEnabled"];
      if (so.containsKey("conditionVar")) s.conditionVar = String(so["conditionVar"].as<const char*>());
      if (so.containsKey("conditionOp")) s.conditionOp = so["conditionOp"];
      if (so.containsKey("conditionValue")) s.conditionValue = so["conditionValue"];
      if (so.containsKey("outputCount")) {
        int oc = so["outputCount"];
        if (oc < 1) oc = 1;
        if (oc > MAX_OUTPUTS_PER_TRIGGER) oc = MAX_OUTPUTS_PER_TRIGGER;
        s.outputCount = static_cast<uint8_t>(oc);
      }
      bool importedArtAddr[MAX_OUTPUTS_PER_TRIGGER];
      for (int ai = 0; ai < MAX_OUTPUTS_PER_TRIGGER; ai++) importedArtAddr[ai] = false;
      JsonArray outs = so["outputs"].as<JsonArray>();
      if (!outs.isNull()) {
        for (JsonObject oo : outs) {
          int oidx = oo["index"] | -1;
          if (oidx < 0 || oidx >= MAX_OUTPUTS_PER_TRIGGER) continue;
          SensorConfig::OutputConfig out = s.outputs[oidx];
          if (oo.containsKey("target")) out.target = sanitizeOutputTarget(oo["target"].as<int>());
          if (oo.containsKey("delayMs")) out.delayMs = oo["delayMs"];
          if (oo.containsKey("device")) out.device = oo["device"];
          if (oo.containsKey("oscAddress")) out.oscAddress = String(oo["oscAddress"].as<const char*>());
          if (oo.containsKey("buttonAddress")) out.buttonAddress = String(oo["buttonAddress"].as<const char*>());
          if (oo.containsKey("outMin")) out.outMin = oo["outMin"];
          if (oo.containsKey("outMax")) out.outMax = oo["outMax"];
          if (oo.containsKey("outMode")) out.outMode = static_cast<OutputMode>(oo["outMode"].as<int>());
          if (oo.containsKey("buttonOutMin")) out.buttonOutMin = oo["buttonOutMin"];
          if (oo.containsKey("buttonOutMax")) out.buttonOutMax = oo["buttonOutMax"];
          if (oo.containsKey("buttonOutMode")) out.buttonOutMode = static_cast<OutputMode>(oo["buttonOutMode"].as<int>());
          if (oo.containsKey("sendMinOnRelease")) out.sendMinOnRelease = oo["sendMinOnRelease"];
          if (oo.containsKey("udpPayload")) out.udpPayload = String(oo["udpPayload"].as<const char*>());
          if (oo.containsKey("gpioPin")) out.gpioPin = oo["gpioPin"];
          if (oo.containsKey("gpioMode")) out.gpioMode = static_cast<GpioOutMode>(oo["gpioMode"].as<int>());
          if (oo.containsKey("gpioPulseMs")) out.gpioPulseMs = oo["gpioPulseMs"];
          if (oo.containsKey("gpioInvert")) out.gpioInvert = oo["gpioInvert"];
          if (oo.containsKey("httpMethod")) out.httpMethod = static_cast<HttpMethod>(oo["httpMethod"].as<int>());
          if (oo.containsKey("httpUrl")) out.httpUrl = String(oo["httpUrl"].as<const char*>());
          if (oo.containsKey("httpBody")) out.httpBody = String(oo["httpBody"].as<const char*>());
          if (oo.containsKey("httpIp")) out.httpIp = String(oo["httpIp"].as<const char*>());
          if (oo.containsKey("httpPort")) out.httpPort = oo["httpPort"];
          if (oo.containsKey("httpDevice")) out.httpDevice = oo["httpDevice"];
          if (oo.containsKey("mqttTopic")) out.mqttTopic = String(oo["mqttTopic"].as<const char*>());
          if (oo.containsKey("mqttPayload")) out.mqttPayload = String(oo["mqttPayload"].as<const char*>());
          if (oo.containsKey("mqttRetain")) out.mqttRetain = oo["mqttRetain"];
          if (oo.containsKey("dmxProtocol")) out.dmxProtocol = sanitizeDmxProtocol(oo["dmxProtocol"].as<int>());
          if (oo.containsKey("dmxDest")) out.dmxDest = sanitizeDmxDest(oo["dmxDest"].as<int>());
          if (oo.containsKey("dmxArtNet")) out.dmxArtNet = sanitizeDmxArtNet(oo["dmxArtNet"].as<int>());
          if (oo.containsKey("dmxArtSubnet")) out.dmxArtSubnet = sanitizeDmxArtSubnet(oo["dmxArtSubnet"].as<int>());
          if (oo.containsKey("dmxArtUniverse")) out.dmxArtUniverse = sanitizeDmxArtUniverse(oo["dmxArtUniverse"].as<int>());
          importedArtAddr[oidx] = oo.containsKey("dmxArtNet") || oo.containsKey("dmxArtSubnet") || oo.containsKey("dmxArtUniverse");
          if (oo.containsKey("dmxUniverse")) {
            uint16_t u = oo["dmxUniverse"];
            if (u > 0) out.dmxUniverse = u;
          }
          if (oo.containsKey("dmxChannels")) out.dmxChannels = String(oo["dmxChannels"].as<const char*>());
          if (oo.containsKey("dmxPreset")) out.dmxPreset = sanitizeDmxPresetId(oo["dmxPreset"].as<int>());
          if (oo.containsKey("dmxFadeMs")) out.dmxFadeMs = sanitizeDmxFadeMs(oo["dmxFadeMs"].as<int>());
          if (oo.containsKey("onString")) out.onString = String(oo["onString"].as<const char*>());
          if (oo.containsKey("offString")) out.offString = String(oo["offString"].as<const char*>());
          if (oo.containsKey("buttonOnString")) out.buttonOnString = String(oo["buttonOnString"].as<const char*>());
          if (oo.containsKey("buttonOffString")) out.buttonOffString = String(oo["buttonOffString"].as<const char*>());
          if (oo.containsKey("setVarName")) out.setVarName = String(oo["setVarName"].as<const char*>());
          if (oo.containsKey("setVarValue")) out.setVarValue = oo["setVarValue"];
          if ((out.target == OUT_TARGET_DMX || out.target == OUT_TARGET_DMX_PRESET) && out.outMode == OUT_STRING) out.outMode = OUT_INT;
          s.outputs[oidx] = out;
        }
      }
      if (so.containsKey("timeType")) s.timeType = static_cast<TimeTriggerType>(so["timeType"].as<int>());
      if (so.containsKey("timeYear")) s.timeYear = so["timeYear"];
      if (so.containsKey("timeMonth")) s.timeMonth = so["timeMonth"];
      if (so.containsKey("timeDay")) s.timeDay = so["timeDay"];
      if (so.containsKey("timeHour")) s.timeHour = so["timeHour"];
      if (so.containsKey("timeMinute")) s.timeMinute = so["timeMinute"];
      if (so.containsKey("timeSecond")) s.timeSecond = so["timeSecond"];
      if (so.containsKey("weeklyMask")) s.weeklyMask = so["weeklyMask"];
      if (so.containsKey("intervalSeconds")) s.intervalSeconds = so["intervalSeconds"];

      if (!isAllowedDigitalPin(s.pin) && s.type != SENSOR_ANALOG) s.pin = defaultSensorPin(idx);
      if (!isAllowedAnalogPin(s.pin) && s.type == SENSOR_ANALOG) s.pin = defaultSensorPin(idx);
      if (!isAllowedDigitalPin(s.encClkPin)) s.encClkPin = 13;
      if (!isAllowedDigitalPin(s.encDtPin)) s.encDtPin = 16;
      if (!isAllowedDigitalPin(s.encSwPin)) s.encSwPin = 33;
      if (!isAllowedDigitalPin(s.hcTrigPin)) s.hcTrigPin = 13;
      if (!isAllowedDigitalPin(s.hcEchoPin)) s.hcEchoPin = 16;
      if (s.source != SRC_TIME && s.source != SRC_OSC && s.source != SRC_DMX) s.source = SRC_SENSORS;
      if (s.oscInAddress.length() == 0) s.oscInAddress = "/trigger/*";
      if (s.oscInArgType > OSC_IN_ARG_STRING) s.oscInArgType = OSC_IN_ARG_ANY;
      if (s.oscInMatchMode > OSC_IN_MATCH_RANGE) s.oscInMatchMode = OSC_IN_MATCH_ANY;
      if (s.oscInString.length() == 0) s.oscInString = "go";
      for (int o = 0; o < s.outputCount; o++) {
        if (s.outputs[o].dmxUniverse == 0) s.outputs[o].dmxUniverse = 1;
        if (importedArtAddr[o] && s.outputs[o].dmxProtocol == DMX_PROTO_ARTNET) {
          s.outputs[o].dmxUniverse = encodeArtNetPortAddressOneBased(
              s.outputs[o].dmxArtNet, s.outputs[o].dmxArtSubnet, s.outputs[o].dmxArtUniverse);
        }
        if (!importedArtAddr[o]) {
          decodeArtNetPortAddressOneBased(s.outputs[o].dmxUniverse,
                                          s.outputs[o].dmxArtNet,
                                          s.outputs[o].dmxArtSubnet,
                                          s.outputs[o].dmxArtUniverse);
        }
        s.outputs[o].dmxArtNet = sanitizeDmxArtNet(s.outputs[o].dmxArtNet);
        s.outputs[o].dmxArtSubnet = sanitizeDmxArtSubnet(s.outputs[o].dmxArtSubnet);
        s.outputs[o].dmxArtUniverse = sanitizeDmxArtUniverse(s.outputs[o].dmxArtUniverse);
        if (s.outputs[o].dmxChannels.length() == 0) s.outputs[o].dmxChannels = "1";
        s.outputs[o].dmxPreset = sanitizeDmxPresetId(s.outputs[o].dmxPreset);
        s.outputs[o].dmxFadeMs = sanitizeDmxFadeMs(s.outputs[o].dmxFadeMs);
      }
      config.sensors[idx] = s;
    }
  }

  for (int i = 0; i < MAX_DMX_PRESETS; i++) dmxPresetText[i] = "";
  JsonArray dmxPresets = cfg["dmxPresets"].as<JsonArray>();
  if (!dmxPresets.isNull()) {
    for (JsonObject po : dmxPresets) {
      int idx = po["index"] | -1;
      if (idx < 0 || idx >= MAX_DMX_PRESETS) continue;
      if (!po.containsKey("text")) continue;
      dmxPresetText[idx] = String(po["text"].as<const char*>());
      dmxPresetText[idx].trim();
    }
  }

  if (config.mqttPort == 0) config.mqttPort = DEFAULT_MQTT_PORT;
  if (config.oscInPort == 0) config.oscInPort = DEFAULT_OSC_IN_PORT;
  if (config.mqttDevice >= MAX_OSC_DEVICES && config.mqttDevice != MQTT_DEVICE_CUSTOM) {
    config.mqttDevice = DEFAULT_MQTT_DEVICE;
  }
  if (config.mqttClientId.length() == 0) config.mqttClientId = defaultMqttClientId();
  if (config.mqttBaseTopic.length() == 0) config.mqttBaseTopic = DEFAULT_MQTT_BASE_TOPIC;

  return true;
}

static void startWifiAp() {
  String apSsid = buildApSsid();
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(apSsid.c_str());
  delay(500);
  dnsServer.start(53, "*", WiFi.softAPIP());
  wifiApMode = true;
  Serial.print("AP mode: ");
  Serial.print(apSsid);
  Serial.print(" IP: ");
  Serial.println(WiFi.softAPIP());
  addLog(String("AP mode: ") + apSsid + " IP " + WiFi.softAPIP().toString());
}

static String normalizeOscAddress(String addr) {
  addr.trim();
  if (addr.length() == 0) return "/";
  if (!addr.startsWith("/")) addr = "/" + addr;
  return addr;
}

static IPAddress parseOrDefault(const String& s, const IPAddress& fallback) {
  IPAddress parsed;
  if (parseIpString(s, parsed)) return parsed;
  return fallback;
}

static String defaultOscAddress(int index) {
  return String("/Fader") + String(401 + index);
}

static int defaultSensorPin(int index) {
  switch (index) {
    case 0: return 32;
    case 1: return 33;
    case 2: return 35;
    case 3: return 36;
    case 4: return 13;
    default: return 16;
  }
}

static SensorType sanitizeSensorType(int value) {
  if (value < SENSOR_ANALOG || value > SENSOR_HCSR04) return SENSOR_ANALOG;
  if (value == SENSOR_DIGITAL) return SENSOR_BUTTON;
  return static_cast<SensorType>(value);
}

static OutputMode sanitizeOutputMode(int value) {
  if (value < OUT_INT || value > OUT_STRING) return DEFAULT_OUT_MODE;
  return static_cast<OutputMode>(value);
}

static OutputTarget sanitizeOutputTarget(int value) {
  if (value < OUT_TARGET_OSC || value > OUT_TARGET_SET_VAR) return OUT_TARGET_OSC;
  return static_cast<OutputTarget>(value);
}

static uint8_t sanitizeDmxProtocol(int value) {
  if (value < DMX_PROTO_ARTNET || value > DMX_PROTO_SACN) return DMX_PROTO_ARTNET;
  return static_cast<uint8_t>(value);
}

static uint8_t sanitizeDmxDest(int value) {
  if (value < DMX_DEST_AUTO || value > DMX_DEST_UNICAST) return DMX_DEST_AUTO;
  return static_cast<uint8_t>(value);
}

static uint8_t sanitizeDmxArtNet(int value) {
  if (value < 0) return 0;
  if (value > 127) return 127;
  return static_cast<uint8_t>(value);
}

static uint8_t sanitizeDmxArtSubnet(int value) {
  if (value < 0) return 0;
  if (value > 15) return 15;
  return static_cast<uint8_t>(value);
}

static uint8_t sanitizeDmxArtUniverse(int value) {
  if (value < 0) return 0;
  if (value > 15) return 15;
  return static_cast<uint8_t>(value);
}

static uint8_t sanitizeDmxPresetId(int value) {
  if (value < 1) return 1;
  if (value > MAX_DMX_PRESETS) return MAX_DMX_PRESETS;
  return static_cast<uint8_t>(value);
}

static uint32_t sanitizeDmxFadeMs(int value) {
  if (value < 0) return 0;
  if (value > 600000) return 600000;
  return static_cast<uint32_t>(value);
}

static uint8_t sanitizeDmxInputSourceMode(int value) {
  if (value < DMX_IN_SOURCE_RECENT || value > DMX_IN_SOURCE_PREFER_SACN) return DMX_IN_SOURCE_RECENT;
  return static_cast<uint8_t>(value);
}

static NetMode sanitizeNetworkMode(int value) {
  if (value < NET_ETHERNET || value > NET_WIFI) return DEFAULT_NET_MODE;
  return static_cast<NetMode>(value);
}

static bool isReservedPin(int pin) {
  return pin == RESET_BUTTON_PIN || pin == 0 || pin == 1 || pin == 2 || pin == 3 || pin == 4 || pin == 12 || pin == 14 || pin == 15;
}

static bool isClickPatternType(SensorType type) {
  return type == SENSOR_BUTTON_DOUBLE || type == SENSOR_BUTTON_TRIPLE || type == SENSOR_BUTTON_LONG;
}

static bool isAllowedAnalogPin(int pin) {
  return pin == 32 || pin == 35 || pin == 36;
}

static bool isAllowedDigitalPin(int pin) {
  return pin == 5 || pin == 13 || pin == 16 || pin == 17 || pin == 18 || pin == 19 || pin == 21 || pin == 22 ||
         pin == 23 || pin == 25 || pin == 26 || pin == 27 || pin == 32 || pin == 33 || pin == 35 || pin == 36 || pin == 39;
}

static bool isAllowedGpioOutputPin(int pin) {
  return pin == 5 || pin == 13 || pin == 16 || pin == 17 || pin == 18 || pin == 19 || pin == 21 || pin == 22 ||
         pin == 23 || pin == 25 || pin == 26 || pin == 27 || pin == 32 || pin == 33;
}

static bool isInputPinInUse(int pin, int triggerIndex, const SensorConfig& current) {
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    const SensorConfig& s = (i == triggerIndex) ? current : config.sensors[i];
    if (!s.enabled) continue;
    if (s.source != SRC_SENSORS) continue;
    if (s.type == SENSOR_ENCODER) {
      if (s.encClkPin == pin || s.encDtPin == pin || s.encSwPin == pin) return true;
    } else if (s.type == SENSOR_HCSR04) {
      if (s.hcTrigPin == pin || s.hcEchoPin == pin) return true;
    } else {
      if (s.pin == pin) return true;
    }
  }
  return false;
}

static int pickFreeGpioOutputPin(int triggerIndex, const SensorConfig& current) {
  const int pins[] = {5,13,16,17,18,19,21,22,23,25,26,27,32,33};
  for (size_t i = 0; i < sizeof(pins)/sizeof(pins[0]); i++) {
    int pin = pins[i];
    if (!isInputPinInUse(pin, triggerIndex, current)) return pin;
  }
  return -1;
}

static int ensureGpioPwmChannel(int pin) {
  if (!isAllowedGpioOutputPin(pin) || isReservedPin(pin)) return -1;
  if (gpioPwmChannelByPin[pin] >= 0) return gpioPwmChannelByPin[pin];
  if (nextGpioPwmChannel >= 16) return -1;
  int ch = nextGpioPwmChannel++;
  ledcSetup(ch, GPIO_PWM_FREQ_HZ, GPIO_PWM_RES_BITS);
  ledcAttachPin(pin, ch);
  gpioPwmChannelByPin[pin] = ch;
  return ch;
}

static bool isPasswordRequired() {
  return config.passwordEnabled && config.password.length() > 0;
}

// CSRF defense: browsers attach Origin/Referer to cross-site POSTs but pages
// can't forge them. Requests without either header (curl, native apps) pass.
static bool checkSameOrigin() {
  auto hostOf = [](const String& url) -> String {
    int p = url.indexOf("://");
    if (p < 0) return String();
    int s = p + 3;
    int e = url.indexOf('/', s);
    return (e < 0) ? url.substring(s) : url.substring(s, e);
  };
  String host = server.hostHeader();
  if (host.length() == 0) return true;
  String origin = server.header("Origin");
  if (origin.length() > 0 && origin != "null") return hostOf(origin) == host;
  String referer = server.header("Referer");
  if (referer.length() > 0) return hostOf(referer) == host;
  return true;
}

static bool ensureAuthenticated() {
  bool mutating = (server.method() == HTTP_POST || server.method() == HTTP_DELETE);
  if (mutating && !checkSameOrigin()) {
    addLog("Rejected cross-origin request: " + server.uri());
    server.send(403, "text/plain", "Cross-origin request rejected");
    return false;
  }
  if (!isPasswordRequired()) return true;
  if (server.authenticate(config.username.c_str(), config.password.c_str())) return true;
  server.requestAuthentication();
  return false;
}

static SensorConfig defaultSensorConfig(int index) {
  SensorConfig s;
  s.enabled = (index == 0);
  s.name = String("Trigger ") + String(index + 1);
  s.source = SRC_SENSORS;
  s.oscInAddress = "/trigger/*";
  s.oscInArgType = OSC_IN_ARG_ANY;
  s.oscInMatchMode = OSC_IN_MATCH_ANY;
  s.oscInValue = 1.0f;
  s.oscInMin = 0.0f;
  s.oscInMax = 1.0f;
  s.oscInString = "go";
  s.dmxInUniverse = 1;
  s.dmxInChannel = 1;
  s.dmxInMode = DMX_IN_TRIG_LEVEL;
  s.dmxInThreshold = 128;
  s.dmxInRangeMin = 0;
  s.dmxInRangeMax = 255;
  s.type = SENSOR_BUTTON;
  s.pin = defaultSensorPin(index);
  s.encClkPin = 13;
  s.encDtPin = 16;
  s.encSwPin = 33;
  s.encAppendSign = false;
  s.encSignArg = false;
  s.encInvert = true;
  s.encSteps = DEFAULT_ENC_STEPS;
  s.hcTrigPin = 13;
  s.hcEchoPin = 16;
  s.hcMinCm = 2.0f;
  s.hcMaxCm = 400.0f;
  s.invert = DEFAULT_INVERT;
  s.activeHigh = DEFAULT_DIGITAL_ACTIVE_HIGH;
  s.pullup = DEFAULT_DIGITAL_PULLUP;
  s.cooldownEnabled = false;
  s.cooldownMs = 5000;
  s.conditionEnabled = false;
  s.conditionVar = "";
  s.conditionOp = COND_EQ;
  s.conditionValue = 0.0f;
  s.outputCount = 1;
  bool defaultSendMin = (s.type == SENSOR_ANALOG || s.type == SENSOR_ENCODER);
  for (int o = 0; o < MAX_OUTPUTS_PER_TRIGGER; o++) {
    s.outputs[o].target = OUT_TARGET_OSC;
    s.outputs[o].delayMs = 0;
    s.outputs[o].device = 0;
    s.outputs[o].oscAddress = defaultOscAddress(index);
    s.outputs[o].buttonAddress = "/button";
    s.outputs[o].outMin = DEFAULT_OUT_MIN;
    s.outputs[o].outMax = DEFAULT_OUT_MAX;
    s.outputs[o].outMode = DEFAULT_OUT_MODE;
    s.outputs[o].buttonOutMin = DEFAULT_OUT_MIN;
    s.outputs[o].buttonOutMax = DEFAULT_OUT_MAX;
    s.outputs[o].buttonOutMode = DEFAULT_OUT_MODE;
    s.outputs[o].sendMinOnRelease = defaultSendMin;
    s.outputs[o].udpPayload = "";
    s.outputs[o].gpioPin = 5;
    s.outputs[o].gpioMode = GPIO_OUT_PULSE;
    s.outputs[o].gpioPulseMs = 100;
    s.outputs[o].gpioInvert = false;
    s.outputs[o].httpMethod = HTTPM_GET;
    s.outputs[o].httpUrl = "http://{IP}:{Port}/";
    s.outputs[o].httpBody = "";
    s.outputs[o].httpIp = "";
    s.outputs[o].httpPort = 80;
    s.outputs[o].httpDevice = 0;
    s.outputs[o].mqttTopic = buildDefaultMqttTopic(s, index, o);
    s.outputs[o].mqttPayload = "{value}";
    s.outputs[o].mqttRetain = false;
    s.outputs[o].dmxProtocol = DMX_PROTO_ARTNET;
    s.outputs[o].dmxDest = DMX_DEST_AUTO;
    s.outputs[o].dmxArtNet = 0;
    s.outputs[o].dmxArtSubnet = 0;
    s.outputs[o].dmxArtUniverse = 0;
    s.outputs[o].dmxUniverse = 1;
    s.outputs[o].dmxChannels = "1";
    s.outputs[o].dmxPreset = 1;
    s.outputs[o].dmxFadeMs = 1000;
    s.outputs[o].onString = DEFAULT_ON_STRING;
    s.outputs[o].offString = DEFAULT_OFF_STRING;
    s.outputs[o].buttonOnString = DEFAULT_ON_STRING;
    s.outputs[o].buttonOffString = DEFAULT_OFF_STRING;
    s.outputs[o].setVarName = "";
    s.outputs[o].setVarValue = 0.0f;
  }
  s.timeType = TIME_DAILY;
  s.timeYear = 2025;
  s.timeMonth = 1;
  s.timeDay = 1;
  s.timeHour = 12;
  s.timeMinute = 0;
  s.timeSecond = 0;
  s.weeklyMask = 0x7F;
  s.intervalSeconds = 3600;
  return s;
}

static String sensorKey(int index, const char* suffix) {
  return String("s") + String(index) + "_" + suffix;
}

static String outputKey(int sensorIndex, int outputIndex, const char* suffix) {
  return String("s") + String(sensorIndex) + "_o" + String(outputIndex) + "_" + suffix;
}

static String formatTimeLocal() {
  time_t now = time(nullptr);
  if (now < 100000) return "Not set";
  struct tm tmLocal;
  localtime_r(&now, &tmLocal);
  char buf[32];
  if (config.timeDisplay24h) {
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmLocal);
  } else {
    strftime(buf, sizeof(buf), "%Y-%m-%d %I:%M:%S %p", &tmLocal);
  }
  return String(buf);
}

static String formatDateLocal() {
  time_t now = time(nullptr);
  if (now < 100000) return "";
  struct tm tmLocal;
  localtime_r(&now, &tmLocal);
  char buf[16];
  strftime(buf, sizeof(buf), "%Y-%m-%d", &tmLocal);
  return String(buf);
}

static String formatTimeLocalInput() {
  time_t now = time(nullptr);
  if (now < 100000) return "";
  struct tm tmLocal;
  localtime_r(&now, &tmLocal);
  char buf[16];
  strftime(buf, sizeof(buf), "%H:%M:%S", &tmLocal);
  return String(buf);
}

static bool parseDateTimeLocal(const String& dateStr, const String& timeStr, uint32_t& epochOut) {
  int year = 0, mon = 0, day = 0;
  int hour = 0, min = 0, sec = 0;
  if (sscanf(dateStr.c_str(), "%d-%d-%d", &year, &mon, &day) != 3) return false;
  int timeParts = sscanf(timeStr.c_str(), "%d:%d:%d", &hour, &min, &sec);
  if (timeParts < 2) return false;
  if (year < 1970 || mon < 1 || mon > 12 || day < 1 || day > 31) return false;
  if (hour < 0 || hour > 23 || min < 0 || min > 59 || sec < 0 || sec > 59) return false;
  struct tm tmLocal = {};
  tmLocal.tm_year = year - 1900;
  tmLocal.tm_mon = mon - 1;
  tmLocal.tm_mday = day;
  tmLocal.tm_hour = hour;
  tmLocal.tm_min = min;
  tmLocal.tm_sec = sec;
  tmLocal.tm_isdst = -1;
  time_t t = mktime(&tmLocal);
  if (t < 0) return false;
  epochOut = static_cast<uint32_t>(t);
  return true;
}

static bool parseDateYMD(const String& dateStr, uint16_t& y, uint8_t& m, uint8_t& d) {
  int year = 0, mon = 0, day = 0;
  if (sscanf(dateStr.c_str(), "%d-%d-%d", &year, &mon, &day) != 3) return false;
  if (year < 1970 || mon < 1 || mon > 12 || day < 1 || day > 31) return false;
  y = static_cast<uint16_t>(year);
  m = static_cast<uint8_t>(mon);
  d = static_cast<uint8_t>(day);
  return true;
}

static bool parseTimeHMS(const String& timeStr, uint8_t& h, uint8_t& m, uint8_t& s) {
  String t = timeStr;
  t.trim();
  String upper = t;
  upper.toUpperCase();
  int ampm = -1;
  if (upper.indexOf("PM") >= 0) ampm = 1;
  if (upper.indexOf("AM") >= 0) ampm = 0;
  String filtered;
  filtered.reserve(t.length());
  for (size_t i = 0; i < t.length(); i++) {
    char c = t[i];
    if ((c >= '0' && c <= '9') || c == ':') filtered += c;
  }
  t = filtered;
  int hour = 0, min = 0, sec = 0;
  int parts = sscanf(t.c_str(), "%d:%d:%d", &hour, &min, &sec);
  if (parts < 2) return false;
  if (ampm >= 0) {
    if (hour == 12) hour = (ampm == 0) ? 0 : 12;
    else if (ampm == 1) hour += 12;
  }
  if (hour < 0 || hour > 23 || min < 0 || min > 59 || sec < 0 || sec > 59) return false;
  h = static_cast<uint8_t>(hour);
  m = static_cast<uint8_t>(min);
  s = static_cast<uint8_t>(sec);
  return true;
}

static bool handleTimeTrigger(int index, SensorConfig& s) {
  time_t now = time(nullptr);
  if (now < 100000) return true;
  struct tm tmLocal;
  localtime_r(&now, &tmLocal);
  uint32_t todayKey = static_cast<uint32_t>((tmLocal.tm_year + 1900) * 10000 +
                                            (tmLocal.tm_mon + 1) * 100 +
                                            tmLocal.tm_mday);
  auto fire = [&]() {
    if (!checkCondition(s)) return;
    addLog(String("Time trigger: ") + s.name);
    for (int o = 0; o < s.outputCount; o++) {
      const SensorConfig::OutputConfig& out = s.outputs[o];
      queueOrSendOutput(s, out, out.oscAddress, 1.0f);
    }
  };
  if (s.timeType == TIME_ONCE) {
    struct tm tmTarget = {};
    tmTarget.tm_year = s.timeYear - 1900;
    tmTarget.tm_mon = s.timeMonth - 1;
    tmTarget.tm_mday = s.timeDay;
    tmTarget.tm_hour = s.timeHour;
    tmTarget.tm_min = s.timeMinute;
    tmTarget.tm_sec = s.timeSecond;
    tmTarget.tm_isdst = -1;
    time_t target = mktime(&tmTarget);
    if (target > 0 && now >= target && lastTimeFireEpoch[index] < target) {
      fire();
      lastTimeFireEpoch[index] = target;
    }
  } else if (s.timeType == TIME_DAILY) {
    if (tmLocal.tm_hour == s.timeHour &&
        tmLocal.tm_min == s.timeMinute &&
        tmLocal.tm_sec == s.timeSecond &&
        lastTimeFireDay[index] != todayKey) {
      fire();
      lastTimeFireDay[index] = todayKey;
    }
  } else if (s.timeType == TIME_WEEKLY) {
    if ((s.weeklyMask & (1 << tmLocal.tm_wday)) &&
        tmLocal.tm_hour == s.timeHour &&
        tmLocal.tm_min == s.timeMinute &&
        tmLocal.tm_sec == s.timeSecond &&
        lastTimeFireDay[index] != todayKey) {
      fire();
      lastTimeFireDay[index] = todayKey;
    }
  } else if (s.timeType == TIME_INTERVAL) {
    if (s.intervalSeconds > 0) {
      if (lastTimeFireEpoch[index] == 0 || now - lastTimeFireEpoch[index] >= static_cast<time_t>(s.intervalSeconds)) {
        fire();
        lastTimeFireEpoch[index] = now;
      }
    }
  }
  return true;
}

static void resetRuntimeState(bool keepOutputState = false) {
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    ema[i] = -1.0f;
    if (!keepOutputState) {
      for (int o = 0; o < MAX_OUTPUTS_PER_TRIGGER; o++) {
        lastInt[i][o] = -1;
        lastFloat[i][o] = NAN;
        lastBool[i][o] = -1;
      }
    }
    lastLevel[i] = -1;
    dmxInLastValue[i] = -1;
    dmxInGateState[i] = -1;
    toggleState[i] = false;
    lastEncA[i] = -1;
    lastEncB[i] = -1;
    lastEncState[i] = -1;
    encAccum[i] = 0;
    encUnitByTrigger[i] = -1;
    lastButtonTriggerMs[i] = 0;
    clickCount[i] = 0;
    multiPrevPressed[i] = false;
    multiLongFired[i] = false;
    multiLastChangeMs[i] = 0;
    multiPressStartMs[i] = 0;
    multiLastReleaseMs[i] = 0;
    lastAnalogPrintMs[i] = 0;
    lastTimeFireEpoch[i] = 0;
    lastTimeFireDay[i] = 0;
  }
  for (int i = 0; i < MAX_OSC_DEVICES; i++) {
    hbLastSent[i] = 0;
  }
  for (int pin = 0; pin < 40; pin++) {
    gpioPulseUntilByPin[pin] = 0;
    gpioPwmChannelByPin[pin] = -1;
    gpioPulseEndLevelByPin[pin] = 0;
  }
  for (int i = 0; i < MAX_DMX_STREAMS; i++) {
    dmxStreams[i].used = false;
    dmxStreams[i].protocol = DMX_PROTO_ARTNET;
    dmxStreams[i].device = 0;
    dmxStreams[i].universe = 1;
    dmxStreams[i].sequence = 0;
    memset(dmxStreams[i].data, 0, sizeof(dmxStreams[i].data));
  }
  dmxFadeState.active = false;
  dmxFadeState.protocol = DMX_PROTO_ARTNET;
  dmxFadeState.dest = DMX_DEST_AUTO;
  dmxFadeState.device = 0;
  dmxFadeState.startMs = 0;
  dmxFadeState.durationMs = 0;
  dmxFadeState.lastFrameMs = 0;
  dmxFadeState.pointCount = 0;
  dmxFadeState.usedUniverseCount = 0;
  for (int u = 0; u < MAX_DMX_PRESET_UNIVERSE; u++) {
    dmxInSource[u] = 0;
    dmxInSeenMs[u] = 0;
    dmxInArtSeenMs[u] = 0;
    dmxInSacnSeenMs[u] = 0;
    uint8_t* uni = getDmxInputUniverseBuffer(u);
    if (uni != nullptr) memset(uni, 0, 512);
  }
  nextGpioPwmChannel = 0;
  lastHeartbeatSentMs = 0;
}

static void resetAllCooldowns() {
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    lastButtonTriggerMs[i] = 0;
  }
  addLog("Cooldowns reset.");
}

static void removeIfExists(const String& key) {
  if (prefs.isKey(key.c_str())) {
    prefs.remove(key.c_str());
  }
}

static void clearSensorKeys(int index) {
  removeIfExists(sensorKey(index, "en"));
  removeIfExists(sensorKey(index, "name"));
  removeIfExists(sensorKey(index, "src"));
  removeIfExists(sensorKey(index, "osia"));
  removeIfExists(sensorKey(index, "osit"));
  removeIfExists(sensorKey(index, "osim"));
  removeIfExists(sensorKey(index, "osiv"));
  removeIfExists(sensorKey(index, "osmn"));
  removeIfExists(sensorKey(index, "osmx"));
  removeIfExists(sensorKey(index, "osis"));
  removeIfExists(sensorKey(index, "diu"));
  removeIfExists(sensorKey(index, "dic"));
  removeIfExists(sensorKey(index, "dim"));
  removeIfExists(sensorKey(index, "dit"));
  removeIfExists(sensorKey(index, "din"));
  removeIfExists(sensorKey(index, "dix"));
  removeIfExists(sensorKey(index, "type"));
  removeIfExists(sensorKey(index, "pin"));
  removeIfExists(sensorKey(index, "clk"));
  removeIfExists(sensorKey(index, "dt"));
  removeIfExists(sensorKey(index, "sw"));
  removeIfExists(sensorKey(index, "enc_app"));
  removeIfExists(sensorKey(index, "enc_arg"));
  removeIfExists(sensorKey(index, "enc_inv"));
  removeIfExists(sensorKey(index, "hct"));
  removeIfExists(sensorKey(index, "hce"));
  removeIfExists(sensorKey(index, "hcmin"));
  removeIfExists(sensorKey(index, "hcmax"));
  removeIfExists(sensorKey(index, "inv"));
  removeIfExists(sensorKey(index, "ah"));
  removeIfExists(sensorKey(index, "pu"));
  removeIfExists(sensorKey(index, "cd_en"));
  removeIfExists(sensorKey(index, "cd_ms"));
  removeIfExists(sensorKey(index, "cden"));
  removeIfExists(sensorKey(index, "cdvar"));
  removeIfExists(sensorKey(index, "cdop"));
  removeIfExists(sensorKey(index, "cdval"));
  removeIfExists(sensorKey(index, "oc"));
  removeIfExists(sensorKey(index, "addr"));
  removeIfExists(sensorKey(index, "baddr"));
  removeIfExists(sensorKey(index, "omin"));
  removeIfExists(sensorKey(index, "omax"));
  removeIfExists(sensorKey(index, "omode"));
  removeIfExists(sensorKey(index, "bmin"));
  removeIfExists(sensorKey(index, "bmax"));
  removeIfExists(sensorKey(index, "bmode"));
  removeIfExists(sensorKey(index, "bon"));
  removeIfExists(sensorKey(index, "boff"));
  removeIfExists(sensorKey(index, "ot"));
  removeIfExists(sensorKey(index, "od"));
  removeIfExists(sensorKey(index, "on"));
  removeIfExists(sensorKey(index, "off"));
  for (int o = 0; o < MAX_OUTPUTS_PER_TRIGGER; o++) {
    removeIfExists(outputKey(index, o, "tgt"));
    removeIfExists(outputKey(index, o, "dly"));
    removeIfExists(outputKey(index, o, "svn"));
    removeIfExists(outputKey(index, o, "svv"));
    removeIfExists(outputKey(index, o, "dev"));
    removeIfExists(outputKey(index, o, "addr"));
    removeIfExists(outputKey(index, o, "baddr"));
    removeIfExists(outputKey(index, o, "omin"));
    removeIfExists(outputKey(index, o, "omax"));
    removeIfExists(outputKey(index, o, "omode"));
    removeIfExists(outputKey(index, o, "bmin"));
    removeIfExists(outputKey(index, o, "bmax"));
    removeIfExists(outputKey(index, o, "bmode"));
    removeIfExists(outputKey(index, o, "bon"));
    removeIfExists(outputKey(index, o, "boff"));
    removeIfExists(outputKey(index, o, "smin"));
    removeIfExists(outputKey(index, o, "udp"));
    removeIfExists(outputKey(index, o, "gpin"));
    removeIfExists(outputKey(index, o, "gmode"));
    removeIfExists(outputKey(index, o, "gpms"));
    removeIfExists(outputKey(index, o, "ginv"));
    removeIfExists(outputKey(index, o, "hm"));
    removeIfExists(outputKey(index, o, "hu"));
    removeIfExists(outputKey(index, o, "hb"));
    removeIfExists(outputKey(index, o, "hip"));
    removeIfExists(outputKey(index, o, "hpt"));
    removeIfExists(outputKey(index, o, "hdev"));
    removeIfExists(outputKey(index, o, "mt"));
    removeIfExists(outputKey(index, o, "mp"));
    removeIfExists(outputKey(index, o, "mr"));
    removeIfExists(outputKey(index, o, "dmxp"));
    removeIfExists(outputKey(index, o, "dmxd"));
    removeIfExists(outputKey(index, o, "dmxn"));
    removeIfExists(outputKey(index, o, "dmxs"));
    removeIfExists(outputKey(index, o, "dmxa"));
    removeIfExists(outputKey(index, o, "dmxu"));
    removeIfExists(outputKey(index, o, "dmxc"));
    removeIfExists(outputKey(index, o, "dmxpr"));
    removeIfExists(outputKey(index, o, "dmxfd"));
    removeIfExists(outputKey(index, o, "on"));
    removeIfExists(outputKey(index, o, "off"));
  }
  removeIfExists(sensorKey(index, "tt"));
  removeIfExists(sensorKey(index, "ty"));
  removeIfExists(sensorKey(index, "tm"));
  removeIfExists(sensorKey(index, "td"));
  removeIfExists(sensorKey(index, "th"));
  removeIfExists(sensorKey(index, "tmin"));
  removeIfExists(sensorKey(index, "tsec"));
  removeIfExists(sensorKey(index, "twm"));
  removeIfExists(sensorKey(index, "tint"));
}

static void clearDeviceKeys(int index) {
  String keyEn = String("dev") + index + "_en";
  String keyName = String("dev") + index + "_name";
  String keyIp = String("dev") + index + "_ip";
  String keyPort = String("dev") + index + "_port";
  String keyHbEn = String("dev") + index + "_hb_en";
  String keyHbAddr = String("dev") + index + "_hb_addr";
  String keyHbMs = String("dev") + index + "_hb_ms";
  removeIfExists(keyEn);
  removeIfExists(keyName);
  removeIfExists(keyIp);
  removeIfExists(keyPort);
  removeIfExists(keyHbEn);
  removeIfExists(keyHbAddr);
  removeIfExists(keyHbMs);
}

static void applySensorPinModes() {
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    const SensorConfig& sensor = config.sensors[i];
    if (!sensor.enabled) continue;
    if (sensor.source != SRC_SENSORS) continue;
    if (sensor.type == SENSOR_ENCODER) {
      pinMode(sensor.encClkPin, INPUT_PULLUP);
      pinMode(sensor.encDtPin, INPUT_PULLUP);
      pinMode(sensor.encSwPin, INPUT_PULLUP);
    } else if (sensor.type == SENSOR_HCSR04) {
      pinMode(sensor.hcTrigPin, OUTPUT);
      pinMode(sensor.hcEchoPin, INPUT);
    } else if (sensor.type != SENSOR_ANALOG) {
      pinMode(sensor.pin, sensor.pullup ? INPUT_PULLUP : INPUT);
    }
  }
}

static void setupEncoderCounters() {
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    encUnitByTrigger[i] = -1;
  }
  int unitsUsed = 0;
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    const SensorConfig& s = config.sensors[i];
    if (!s.enabled || s.source != SRC_SENSORS || s.type != SENSOR_ENCODER) continue;
    if (unitsUsed >= 2) {
      addLog("Encoder PCNT limit reached. Extra encoder uses fallback polling.");
      continue;
    }
    pcnt_unit_t unit = (unitsUsed == 0) ? PCNT_UNIT_0 : PCNT_UNIT_1;
    pcnt_config_t cfg = {};
    cfg.pulse_gpio_num = s.encClkPin;
    cfg.ctrl_gpio_num = s.encDtPin;
    cfg.channel = PCNT_CHANNEL_0;
    cfg.unit = unit;
    cfg.pos_mode = PCNT_COUNT_INC;
    cfg.neg_mode = PCNT_COUNT_DEC;
    cfg.lctrl_mode = PCNT_MODE_REVERSE;
    cfg.hctrl_mode = PCNT_MODE_KEEP;
    cfg.counter_h_lim = 32767;
    cfg.counter_l_lim = -32768;
    pcnt_unit_config(&cfg);
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_set_filter_value(unit, 1000); // ~12.5us filter
    pcnt_filter_enable(unit);
    pcnt_counter_resume(unit);
    encUnitByTrigger[i] = static_cast<int8_t>(unit);
    unitsUsed++;
  }
}

static bool hasPinConflict(int index) {
  if (!config.sensors[index].enabled) return false;
  if (config.sensors[index].source == SRC_TIME || config.sensors[index].source == SRC_OSC) return false;
  int pinsA[3];
  int countA = 0;
  const SensorConfig& a = config.sensors[index];
  if (a.type == SENSOR_ENCODER) {
    pinsA[countA++] = a.encClkPin;
    pinsA[countA++] = a.encDtPin;
    pinsA[countA++] = a.encSwPin;
  } else if (a.type == SENSOR_HCSR04) {
    pinsA[countA++] = a.hcTrigPin;
    pinsA[countA++] = a.hcEchoPin;
  } else {
    pinsA[countA++] = a.pin;
  }
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    if (i == index || !config.sensors[i].enabled) continue;
    int pinsB[3];
    int countB = 0;
    const SensorConfig& b = config.sensors[i];
    if (b.source != SRC_SENSORS) continue;
    if (b.type == SENSOR_ENCODER) {
      pinsB[countB++] = b.encClkPin;
      pinsB[countB++] = b.encDtPin;
      pinsB[countB++] = b.encSwPin;
    } else if (b.type == SENSOR_HCSR04) {
      pinsB[countB++] = b.hcTrigPin;
      pinsB[countB++] = b.hcEchoPin;
    } else {
      pinsB[countB++] = b.pin;
    }
    for (int ia = 0; ia < countA; ia++) {
      for (int ib = 0; ib < countB; ib++) {
        if (pinsA[ia] == pinsB[ib]) {
          bool aShared = (a.type == SENSOR_BUTTON) || isClickPatternType(a.type);
          bool bShared = (b.type == SENSOR_BUTTON) || isClickPatternType(b.type);
          if (aShared && bShared && a.pin == b.pin &&
              a.activeHigh == b.activeHigh && a.pullup == b.pullup) {
            continue;
          }
          return true;
        }
      }
    }
  }
  return false;
}

static bool hasSharedClickPattern(int index) {
  const SensorConfig& a = config.sensors[index];
  if (!a.enabled || a.source != SRC_SENSORS) return false;
  if (a.type != SENSOR_BUTTON) return false;
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    if (i == index) continue;
    const SensorConfig& b = config.sensors[i];
    if (!b.enabled || b.source != SRC_SENSORS) continue;
    if (!isClickPatternType(b.type)) continue;
    if (b.pin == a.pin && b.activeHigh == a.activeHigh && b.pullup == a.pullup) {
      return true;
    }
  }
  return false;
}

static void loadConfig() {
  prefs.begin("osc", true);
  config.hostname = sanitizeHostname(prefs.getString("hostname", DEFAULT_HOSTNAME));
  config.netMode = sanitizeNetworkMode(prefs.getInt("net_mode", DEFAULT_NET_MODE));
  config.oscInEnabled = prefs.getBool("osci_en", DEFAULT_OSC_IN_ENABLED);
  config.oscInPort = static_cast<uint16_t>(prefs.getUInt("osci_port", DEFAULT_OSC_IN_PORT));
  if (config.oscInPort == 0) config.oscInPort = DEFAULT_OSC_IN_PORT;
  config.artnetEnabled = prefs.getBool("art_en", DEFAULT_ARTNET_ENABLED);
  config.sacnEnabled = prefs.getBool("sacn_en", DEFAULT_SACN_ENABLED);
  config.dmxInArtNetEnabled = prefs.getBool("dmi_art", DEFAULT_DMX_IN_ARTNET_ENABLED);
  config.dmxInSacnEnabled = prefs.getBool("dmi_sac", DEFAULT_DMX_IN_SACN_ENABLED);
  config.dmxInSourceMode = sanitizeDmxInputSourceMode(prefs.getInt("dmi_src", DMX_IN_SOURCE_RECENT));
#if !USE_ETHERNET
  config.netMode = NET_WIFI;
#endif
  config.clientIp = parseOrDefault(prefs.getString("client_ip", ipToString(DEFAULT_CLIENT_IP)), DEFAULT_CLIENT_IP);
  config.clientPort = prefs.getInt("client_port", DEFAULT_CLIENT_PORT);
  config.useStatic = prefs.getBool("use_static", DEFAULT_USE_STATIC);
  config.staticIp = parseOrDefault(prefs.getString("static_ip", ipToString(DEFAULT_STATIC_IP)), DEFAULT_STATIC_IP);
  config.gateway = parseOrDefault(prefs.getString("gateway", ipToString(DEFAULT_GATEWAY)), DEFAULT_GATEWAY);
  config.subnet = parseOrDefault(prefs.getString("subnet", ipToString(DEFAULT_SUBNET)), DEFAULT_SUBNET);
  config.dns1 = parseOrDefault(prefs.getString("dns1", ipToString(DEFAULT_DNS1)), DEFAULT_DNS1);
  config.dns2 = parseOrDefault(prefs.getString("dns2", ipToString(DEFAULT_DNS2)), DEFAULT_DNS2);
  config.wifiSsid = prefs.getString("wifi_ssid", DEFAULT_WIFI_SSID);
  config.wifiPass = prefs.getString("wifi_pass", DEFAULT_WIFI_PASS);
  config.username = prefs.getString("username", DEFAULT_USERNAME);
  if (config.username.length() == 0) config.username = DEFAULT_USERNAME;
  bool legacyHbEnabled = prefs.getBool("hb_en", false);
  String legacyHbAddr = prefs.getString("hb_addr", "/heartbeat");
  if (legacyHbAddr.length() == 0) legacyHbAddr = "/heartbeat";
  uint32_t legacyHbMs = prefs.getUInt("hb_ms", 5000);
  if (legacyHbMs == 0) legacyHbMs = 5000;
  config.testDevice = prefs.getUInt("test_dev", 0);
  if (config.testDevice >= MAX_OSC_DEVICES) config.testDevice = 0;
  config.clickDebounceMs = prefs.getUInt("click_db", DEFAULT_CLICK_DEBOUNCE_MS);
  if (config.clickDebounceMs == 0) config.clickDebounceMs = DEFAULT_CLICK_DEBOUNCE_MS;
  config.multiClickGapMs = prefs.getUInt("click_gap", DEFAULT_MULTI_CLICK_GAP_MS);
  if (config.multiClickGapMs == 0) config.multiClickGapMs = DEFAULT_MULTI_CLICK_GAP_MS;
  config.longPressMs = prefs.getUInt("long_ms", DEFAULT_LONG_PRESS_MS);
  if (config.longPressMs == 0) config.longPressMs = DEFAULT_LONG_PRESS_MS;
  config.analogSmoothing = prefs.getBool("analog_smooth", DEFAULT_ANALOG_SMOOTHING);
  config.timeMode = prefs.getUInt("time_mode", DEFAULT_TIME_MODE);
  if (config.timeMode > TIME_MANUAL) config.timeMode = DEFAULT_TIME_MODE;
  config.ntpServer = prefs.getString("ntp_srv", DEFAULT_NTP_SERVER);
  if (config.ntpServer.length() == 0) config.ntpServer = DEFAULT_NTP_SERVER;
  config.timeZone = prefs.getString("tz", DEFAULT_TIMEZONE);
  if (config.timeZone.length() == 0) config.timeZone = DEFAULT_TIMEZONE;
  config.manualEpoch = prefs.getUInt("time_manual", 0);
  config.timeDisplay24h = prefs.getBool("time_24", DEFAULT_TIME_DISPLAY_24H);
  config.mqttEnabled = prefs.getBool("mq_en", DEFAULT_MQTT_ENABLED);
  config.mqttDevice = static_cast<uint8_t>(prefs.getUInt("mq_dev", DEFAULT_MQTT_DEVICE));
  if (config.mqttDevice >= MAX_OSC_DEVICES && config.mqttDevice != MQTT_DEVICE_CUSTOM) {
    config.mqttDevice = DEFAULT_MQTT_DEVICE;
  }
  config.mqttHost = prefs.getString("mq_host", DEFAULT_MQTT_HOST);
  config.mqttPort = static_cast<uint16_t>(prefs.getUInt("mq_port", DEFAULT_MQTT_PORT));
  if (config.mqttPort == 0) config.mqttPort = DEFAULT_MQTT_PORT;
  config.mqttUser = prefs.getString("mq_user", DEFAULT_MQTT_USER);
  config.mqttPass = prefs.getString("mq_pass", DEFAULT_MQTT_PASS);
  config.mqttClientId = prefs.getString("mq_cid", DEFAULT_MQTT_CLIENT_ID);
  if (config.mqttClientId.length() == 0) config.mqttClientId = defaultMqttClientId();
  config.mqttBaseTopic = prefs.getString("mq_base", DEFAULT_MQTT_BASE_TOPIC);
  if (config.mqttBaseTopic.length() == 0) config.mqttBaseTopic = DEFAULT_MQTT_BASE_TOPIC;
  config.passwordEnabled = prefs.getBool("pwd_en", false);
  config.password = prefs.getString("pwd", "");
  if (!config.passwordEnabled || config.password.length() == 0) {
    config.passwordEnabled = false;
    config.password = "";
  }

  config.oscDeviceCount = prefs.getUInt("dev_count", 1);
  if (config.oscDeviceCount == 0 || config.oscDeviceCount > MAX_OSC_DEVICES) config.oscDeviceCount = 1;
  String devOrderStr = prefs.getString("dev_order", "");
  uint8_t devOrderCount = 0;
  if (!parseOrderList(devOrderStr, config.oscDeviceOrder, devOrderCount) || devOrderCount == 0) {
    devOrderCount = config.oscDeviceCount;
    for (int i = 0; i < devOrderCount; i++) {
      config.oscDeviceOrder[i] = i;
    }
  }
  bool hasZero = false;
  for (int i = 0; i < devOrderCount; i++) {
    if (config.oscDeviceOrder[i] == 0) {
      hasZero = true;
      break;
    }
  }
  if (!hasZero) {
    if (devOrderCount < MAX_OSC_DEVICES) {
      for (int i = devOrderCount; i > 0; i--) {
        config.oscDeviceOrder[i] = config.oscDeviceOrder[i - 1];
      }
      config.oscDeviceOrder[0] = 0;
      devOrderCount++;
    } else {
      config.oscDeviceOrder[0] = 0;
    }
  }
  config.oscDeviceCount = devOrderCount;

  for (int i = 0; i < MAX_OSC_DEVICES; i++) {
    OscDevice d;
    d.enabled = (i == 0);
    d.name = (i == 0) ? "Default" : String("Device ") + String(i + 1);
    d.ip = config.clientIp;
    d.port = config.clientPort;
    d.hbEnabled = false;
    d.hbAddress = "/heartbeat";
    d.hbMs = 5000;
    String keyEn = String("dev") + i + "_en";
    String keyName = String("dev") + i + "_name";
    String keyIp = String("dev") + i + "_ip";
    String keyPort = String("dev") + i + "_port";
    String keyHbEn = String("dev") + i + "_hb_en";
    String keyHbAddr = String("dev") + i + "_hb_addr";
    String keyHbMs = String("dev") + i + "_hb_ms";
    d.enabled = prefs.getBool(keyEn.c_str(), d.enabled);
    d.name = prefs.getString(keyName.c_str(), d.name);
    d.ip = parseOrDefault(prefs.getString(keyIp.c_str(), ipToString(d.ip)), d.ip);
    d.port = prefs.getInt(keyPort.c_str(), d.port);
    bool hasHb = prefs.isKey(keyHbEn.c_str());
    if (hasHb) {
      d.hbEnabled = prefs.getBool(keyHbEn.c_str(), d.hbEnabled);
      d.hbAddress = prefs.getString(keyHbAddr.c_str(), d.hbAddress);
      d.hbMs = prefs.getUInt(keyHbMs.c_str(), d.hbMs);
      if (d.hbAddress.length() == 0) d.hbAddress = "/heartbeat";
      if (d.hbMs == 0) d.hbMs = 5000;
    } else if (i == 0 && legacyHbEnabled) {
      d.hbEnabled = legacyHbEnabled;
      d.hbAddress = legacyHbAddr;
      d.hbMs = legacyHbMs;
    }
    if (d.port == 0) d.port = config.clientPort;
    config.oscDevices[i] = d;
  }
  config.clientIp = config.oscDevices[0].ip;
  config.clientPort = config.oscDevices[0].port;

  config.triggerCount = prefs.getUInt("tr_count", 1);
  if (config.triggerCount == 0 || config.triggerCount > MAX_TRIGGERS) config.triggerCount = 1;
  String orderStr = prefs.getString("tr_order", "");
  uint8_t orderCount = 0;
  if (!parseOrderList(orderStr, config.triggerOrder, orderCount) || orderCount == 0) {
    orderCount = config.triggerCount;
    for (int i = 0; i < orderCount; i++) {
      config.triggerOrder[i] = i;
    }
  }
  config.triggerCount = orderCount;

  for (int i = 0; i < MAX_TRIGGERS; i++) {
    config.sensors[i] = defaultSensorConfig(i);
  }

  for (int i = 0; i < MAX_TRIGGERS; i++) {
    SensorConfig s = config.sensors[i];
    s.enabled = prefs.getBool(sensorKey(i, "en").c_str(), s.enabled);
    s.name = prefs.getString(sensorKey(i, "name").c_str(), s.name);
    s.source = static_cast<SensorSource>(prefs.getInt(sensorKey(i, "src").c_str(), s.source));
    if (s.source != SRC_TIME && s.source != SRC_OSC && s.source != SRC_DMX) s.source = SRC_SENSORS;
    s.oscInAddress = prefs.getString(sensorKey(i, "osia").c_str(), s.oscInAddress);
    s.oscInArgType = static_cast<uint8_t>(prefs.getUInt(sensorKey(i, "osit").c_str(), s.oscInArgType));
    if (s.oscInArgType > OSC_IN_ARG_STRING) s.oscInArgType = OSC_IN_ARG_ANY;
    s.oscInMatchMode = static_cast<uint8_t>(prefs.getUInt(sensorKey(i, "osim").c_str(), s.oscInMatchMode));
    if (s.oscInMatchMode > OSC_IN_MATCH_RANGE) s.oscInMatchMode = OSC_IN_MATCH_ANY;
    s.oscInValue = prefs.getFloat(sensorKey(i, "osiv").c_str(), s.oscInValue);
    s.oscInMin = prefs.getFloat(sensorKey(i, "osmn").c_str(), s.oscInMin);
    s.oscInMax = prefs.getFloat(sensorKey(i, "osmx").c_str(), s.oscInMax);
    s.oscInString = prefs.getString(sensorKey(i, "osis").c_str(), s.oscInString);
    if (s.oscInAddress.length() == 0) s.oscInAddress = "/trigger/*";
    if (s.oscInString.length() == 0) s.oscInString = "go";
    s.dmxInUniverse = prefs.getUShort(sensorKey(i, "diu").c_str(), s.dmxInUniverse);
    s.dmxInChannel = prefs.getUShort(sensorKey(i, "dic").c_str(), s.dmxInChannel);
    s.dmxInMode = prefs.getUChar(sensorKey(i, "dim").c_str(), s.dmxInMode);
    if (s.dmxInMode > DMX_IN_TRIG_RANGE) s.dmxInMode = DMX_IN_TRIG_LEVEL;
    s.dmxInThreshold = prefs.getUChar(sensorKey(i, "dit").c_str(), s.dmxInThreshold);
    s.dmxInRangeMin = prefs.getUChar(sensorKey(i, "din").c_str(), s.dmxInRangeMin);
    s.dmxInRangeMax = prefs.getUChar(sensorKey(i, "dix").c_str(), s.dmxInRangeMax);
    if (s.dmxInUniverse < 1 || s.dmxInUniverse > MAX_DMX_PRESET_UNIVERSE) s.dmxInUniverse = 1;
    if (s.dmxInChannel < 1 || s.dmxInChannel > 512) s.dmxInChannel = 1;
    s.type = sanitizeSensorType(prefs.getInt(sensorKey(i, "type").c_str(), s.type));
    s.pin = prefs.getInt(sensorKey(i, "pin").c_str(), s.pin);
    s.encClkPin = prefs.getInt(sensorKey(i, "clk").c_str(), s.encClkPin);
    s.encDtPin = prefs.getInt(sensorKey(i, "dt").c_str(), s.encDtPin);
    s.encSwPin = prefs.getInt(sensorKey(i, "sw").c_str(), s.encSwPin);
    s.encAppendSign = prefs.getBool(sensorKey(i, "enc_app").c_str(), s.encAppendSign);
    s.encSignArg = prefs.getBool(sensorKey(i, "enc_arg").c_str(), s.encSignArg);
    s.encInvert = prefs.getBool(sensorKey(i, "enc_inv").c_str(), s.encInvert);
    s.hcTrigPin = prefs.getInt(sensorKey(i, "hct").c_str(), s.hcTrigPin);
    s.hcEchoPin = prefs.getInt(sensorKey(i, "hce").c_str(), s.hcEchoPin);
    s.hcMinCm = prefs.getFloat(sensorKey(i, "hcmin").c_str(), s.hcMinCm);
    s.hcMaxCm = prefs.getFloat(sensorKey(i, "hcmax").c_str(), s.hcMaxCm);
    if (s.hcMaxCm == s.hcMinCm) s.hcMaxCm = s.hcMinCm + 1.0f;
    s.encSteps = DEFAULT_ENC_STEPS;
    if (s.source == SRC_SENSORS && s.type == SENSOR_ANALOG) {
      if (!isAllowedAnalogPin(s.pin)) s.pin = defaultSensorPin(i);
    } else if (s.source == SRC_SENSORS) {
      if (!isAllowedDigitalPin(s.pin)) s.pin = defaultSensorPin(i);
    }
    if (s.source == SRC_SENSORS) {
      if (!isAllowedDigitalPin(s.encClkPin)) s.encClkPin = 13;
      if (!isAllowedDigitalPin(s.encDtPin)) s.encDtPin = 16;
      if (!isAllowedDigitalPin(s.encSwPin)) s.encSwPin = 33;
      if (!isAllowedDigitalPin(s.hcTrigPin)) s.hcTrigPin = 13;
      if (!isAllowedDigitalPin(s.hcEchoPin)) s.hcEchoPin = 16;
    }
    s.invert = prefs.getBool(sensorKey(i, "inv").c_str(), s.invert);
    s.activeHigh = prefs.getBool(sensorKey(i, "ah").c_str(), s.activeHigh);
    s.pullup = prefs.getBool(sensorKey(i, "pu").c_str(), s.pullup);
    s.cooldownEnabled = prefs.getBool(sensorKey(i, "cd_en").c_str(), s.cooldownEnabled);
    s.cooldownMs = prefs.getUInt(sensorKey(i, "cd_ms").c_str(), s.cooldownMs);
    s.conditionEnabled = prefs.getBool(sensorKey(i, "cden").c_str(), s.conditionEnabled);
    s.conditionVar = prefs.getString(sensorKey(i, "cdvar").c_str(), s.conditionVar);
    s.conditionOp = prefs.getUInt(sensorKey(i, "cdop").c_str(), s.conditionOp);
    s.conditionValue = prefs.getFloat(sensorKey(i, "cdval").c_str(), s.conditionValue);
    bool hasOutputCount = prefs.isKey(sensorKey(i, "oc").c_str());
    if (hasOutputCount) {
      s.outputCount = prefs.getUInt(sensorKey(i, "oc").c_str(), s.outputCount);
    } else {
      s.outputCount = 1;
    }
    if (s.outputCount == 0 || s.outputCount > MAX_OUTPUTS_PER_TRIGGER) s.outputCount = 1;
    for (int o = 0; o < MAX_OUTPUTS_PER_TRIGGER; o++) {
      if (hasOutputCount || o == 0) {
        String baseAddr = hasOutputCount ? outputKey(i, o, "addr") : sensorKey(i, "addr");
        String baseBAddr = hasOutputCount ? outputKey(i, o, "baddr") : sensorKey(i, "baddr");
        String baseOmin = hasOutputCount ? outputKey(i, o, "omin") : sensorKey(i, "omin");
        String baseOmax = hasOutputCount ? outputKey(i, o, "omax") : sensorKey(i, "omax");
        String baseOmode = hasOutputCount ? outputKey(i, o, "omode") : sensorKey(i, "omode");
        String baseBmin = hasOutputCount ? outputKey(i, o, "bmin") : sensorKey(i, "bmin");
        String baseBmax = hasOutputCount ? outputKey(i, o, "bmax") : sensorKey(i, "bmax");
        String baseBmode = hasOutputCount ? outputKey(i, o, "bmode") : sensorKey(i, "bmode");
        String baseBon = hasOutputCount ? outputKey(i, o, "bon") : sensorKey(i, "bon");
        String baseBoff = hasOutputCount ? outputKey(i, o, "boff") : sensorKey(i, "boff");
        String baseOt = hasOutputCount ? outputKey(i, o, "tgt") : sensorKey(i, "ot");
        String baseOd = hasOutputCount ? outputKey(i, o, "dev") : sensorKey(i, "od");
        String baseOn = hasOutputCount ? outputKey(i, o, "on") : sensorKey(i, "on");
        String baseOff = hasOutputCount ? outputKey(i, o, "off") : sensorKey(i, "off");

        s.outputs[o].oscAddress = prefs.getString(baseAddr.c_str(), s.outputs[o].oscAddress);
        s.outputs[o].buttonAddress = prefs.getString(baseBAddr.c_str(), s.outputs[o].buttonAddress);
        s.outputs[o].outMin = prefs.getFloat(baseOmin.c_str(), s.outputs[o].outMin);
        s.outputs[o].outMax = prefs.getFloat(baseOmax.c_str(), s.outputs[o].outMax);
        s.outputs[o].outMode = sanitizeOutputMode(prefs.getInt(baseOmode.c_str(), s.outputs[o].outMode));
        s.outputs[o].buttonOutMin = prefs.getFloat(baseBmin.c_str(), s.outputs[o].buttonOutMin);
        s.outputs[o].buttonOutMax = prefs.getFloat(baseBmax.c_str(), s.outputs[o].buttonOutMax);
        s.outputs[o].buttonOutMode = sanitizeOutputMode(prefs.getInt(baseBmode.c_str(), s.outputs[o].buttonOutMode));
        s.outputs[o].target = sanitizeOutputTarget(prefs.getInt(baseOt.c_str(), s.outputs[o].target));
        s.outputs[o].delayMs = prefs.getUInt(outputKey(i, o, "dly").c_str(), 0);
        s.outputs[o].device = prefs.getInt(baseOd.c_str(), s.outputs[o].device);
        if (s.outputs[o].device >= MAX_OSC_DEVICES) s.outputs[o].device = 0;
        String baseSmin = hasOutputCount ? outputKey(i, o, "smin") : String();
        if (hasOutputCount) {
          s.outputs[o].sendMinOnRelease = prefs.getBool(baseSmin.c_str(), s.outputs[o].sendMinOnRelease);
        }
        String baseUdp = hasOutputCount ? outputKey(i, o, "udp") : String();
        if (hasOutputCount) {
          s.outputs[o].udpPayload = prefs.getString(baseUdp.c_str(), s.outputs[o].udpPayload);
        }
        String baseGpin = hasOutputCount ? outputKey(i, o, "gpin") : String();
        if (hasOutputCount) {
          int gpin = prefs.getInt(baseGpin.c_str(), s.outputs[o].gpioPin);
          if (isAllowedGpioOutputPin(gpin)) s.outputs[o].gpioPin = gpin;
        }
        String baseGmode = hasOutputCount ? outputKey(i, o, "gmode") : String();
        if (hasOutputCount) {
          int gm = prefs.getInt(baseGmode.c_str(), s.outputs[o].gpioMode);
          if (gm == GPIO_OUT_LOW) {
            s.outputs[o].gpioMode = GPIO_OUT_HIGH;
            s.outputs[o].gpioInvert = true;
          } else if (gm >= GPIO_OUT_HIGH && gm <= GPIO_OUT_PWM) {
            s.outputs[o].gpioMode = static_cast<GpioOutMode>(gm);
          }
        }
        String baseGpms = hasOutputCount ? outputKey(i, o, "gpms") : String();
        if (hasOutputCount) {
          uint32_t ms = prefs.getUInt(baseGpms.c_str(), s.outputs[o].gpioPulseMs);
          if (ms > 0) s.outputs[o].gpioPulseMs = ms;
        }
        String baseGinv = hasOutputCount ? outputKey(i, o, "ginv") : String();
        if (hasOutputCount) {
          s.outputs[o].gpioInvert = prefs.getBool(baseGinv.c_str(), s.outputs[o].gpioInvert);
        }
        String baseHm = hasOutputCount ? outputKey(i, o, "hm") : String();
        if (hasOutputCount) {
          int hm = prefs.getInt(baseHm.c_str(), s.outputs[o].httpMethod);
          if (hm >= HTTPM_GET && hm <= HTTPM_POST) s.outputs[o].httpMethod = static_cast<HttpMethod>(hm);
        }
        String baseHu = hasOutputCount ? outputKey(i, o, "hu") : String();
        if (hasOutputCount) {
          s.outputs[o].httpUrl = prefs.getString(baseHu.c_str(), s.outputs[o].httpUrl);
        }
        String baseHb = hasOutputCount ? outputKey(i, o, "hb") : String();
        if (hasOutputCount) {
          s.outputs[o].httpBody = prefs.getString(baseHb.c_str(), s.outputs[o].httpBody);
        }
        String baseHip = hasOutputCount ? outputKey(i, o, "hip") : String();
        if (hasOutputCount) {
          s.outputs[o].httpIp = prefs.getString(baseHip.c_str(), s.outputs[o].httpIp);
        }
        String baseHpt = hasOutputCount ? outputKey(i, o, "hpt") : String();
        if (hasOutputCount) {
          uint16_t hp = prefs.getUInt(baseHpt.c_str(), s.outputs[o].httpPort);
          if (hp > 0) s.outputs[o].httpPort = hp;
        }
        String baseHdev = hasOutputCount ? outputKey(i, o, "hdev") : String();
        if (hasOutputCount) {
          uint16_t hd = prefs.getUInt(baseHdev.c_str(), s.outputs[o].httpDevice);
          if (hd < MAX_OSC_DEVICES || hd == 255) s.outputs[o].httpDevice = static_cast<uint8_t>(hd);
        }
        String baseMqttTopic = hasOutputCount ? outputKey(i, o, "mt") : String();
        if (hasOutputCount) {
          s.outputs[o].mqttTopic = prefs.getString(baseMqttTopic.c_str(), s.outputs[o].mqttTopic);
        }
        String baseMqttPayload = hasOutputCount ? outputKey(i, o, "mp") : String();
        if (hasOutputCount) {
          s.outputs[o].mqttPayload = prefs.getString(baseMqttPayload.c_str(), s.outputs[o].mqttPayload);
        }
        String baseMqttRetain = hasOutputCount ? outputKey(i, o, "mr") : String();
        if (hasOutputCount) {
          s.outputs[o].mqttRetain = prefs.getBool(baseMqttRetain.c_str(), s.outputs[o].mqttRetain);
        }
        String baseDmxProto = hasOutputCount ? outputKey(i, o, "dmxp") : String();
        if (hasOutputCount) {
          s.outputs[o].dmxProtocol = sanitizeDmxProtocol(prefs.getUInt(baseDmxProto.c_str(), s.outputs[o].dmxProtocol));
        }
        String baseDmxDest = hasOutputCount ? outputKey(i, o, "dmxd") : String();
        if (hasOutputCount) {
          s.outputs[o].dmxDest = sanitizeDmxDest(prefs.getUInt(baseDmxDest.c_str(), s.outputs[o].dmxDest));
        }
        String baseDmxNet = hasOutputCount ? outputKey(i, o, "dmxn") : String();
        if (hasOutputCount) {
          s.outputs[o].dmxArtNet = sanitizeDmxArtNet(prefs.getUInt(baseDmxNet.c_str(), s.outputs[o].dmxArtNet));
        }
        String baseDmxSubnet = hasOutputCount ? outputKey(i, o, "dmxs") : String();
        if (hasOutputCount) {
          s.outputs[o].dmxArtSubnet = sanitizeDmxArtSubnet(prefs.getUInt(baseDmxSubnet.c_str(), s.outputs[o].dmxArtSubnet));
        }
        String baseDmxUni = hasOutputCount ? outputKey(i, o, "dmxa") : String();
        if (hasOutputCount) {
          s.outputs[o].dmxArtUniverse = sanitizeDmxArtUniverse(prefs.getUInt(baseDmxUni.c_str(), s.outputs[o].dmxArtUniverse));
        }
        String baseDmxUniverse = hasOutputCount ? outputKey(i, o, "dmxu") : String();
        if (hasOutputCount) {
          uint16_t u = static_cast<uint16_t>(prefs.getUInt(baseDmxUniverse.c_str(), s.outputs[o].dmxUniverse));
          if (u > 0) s.outputs[o].dmxUniverse = u;
        }
        String baseDmxChannels = hasOutputCount ? outputKey(i, o, "dmxc") : String();
        if (hasOutputCount) {
          s.outputs[o].dmxChannels = prefs.getString(baseDmxChannels.c_str(), s.outputs[o].dmxChannels);
        }
        String baseDmxPreset = hasOutputCount ? outputKey(i, o, "dmxpr") : String();
        if (hasOutputCount) {
          s.outputs[o].dmxPreset = sanitizeDmxPresetId(prefs.getUInt(baseDmxPreset.c_str(), s.outputs[o].dmxPreset));
        }
        String baseDmxFade = hasOutputCount ? outputKey(i, o, "dmxfd") : String();
        if (hasOutputCount) {
          s.outputs[o].dmxFadeMs = sanitizeDmxFadeMs(prefs.getUInt(baseDmxFade.c_str(), s.outputs[o].dmxFadeMs));
        }
        s.outputs[o].onString = prefs.getString(baseOn.c_str(), s.outputs[o].onString);
        s.outputs[o].offString = prefs.getString(baseOff.c_str(), s.outputs[o].offString);
        s.outputs[o].buttonOnString = prefs.getString(baseBon.c_str(), s.outputs[o].buttonOnString);
        s.outputs[o].buttonOffString = prefs.getString(baseBoff.c_str(), s.outputs[o].buttonOffString);
        if (hasOutputCount) {
          s.outputs[o].setVarName = prefs.getString(outputKey(i, o, "svn").c_str(), s.outputs[o].setVarName);
          s.outputs[o].setVarValue = prefs.getFloat(outputKey(i, o, "svv").c_str(), s.outputs[o].setVarValue);
        }
      }
      if (s.outputs[o].oscAddress.length() == 0) s.outputs[o].oscAddress = defaultOscAddress(i);
      if (s.outputs[o].buttonAddress.length() == 0) s.outputs[o].buttonAddress = "/button";
      if (s.outputs[o].dmxUniverse == 0) s.outputs[o].dmxUniverse = 1;
      if (hasOutputCount) {
        bool hasArtNetKeys = prefs.isKey(outputKey(i, o, "dmxn").c_str()) ||
                             prefs.isKey(outputKey(i, o, "dmxs").c_str()) ||
                             prefs.isKey(outputKey(i, o, "dmxa").c_str());
        if (!hasArtNetKeys) {
          decodeArtNetPortAddressOneBased(s.outputs[o].dmxUniverse,
                                          s.outputs[o].dmxArtNet,
                                          s.outputs[o].dmxArtSubnet,
                                          s.outputs[o].dmxArtUniverse);
        } else if (s.outputs[o].dmxProtocol == DMX_PROTO_ARTNET) {
          s.outputs[o].dmxUniverse = encodeArtNetPortAddressOneBased(
              s.outputs[o].dmxArtNet, s.outputs[o].dmxArtSubnet, s.outputs[o].dmxArtUniverse);
        }
      }
      s.outputs[o].dmxArtNet = sanitizeDmxArtNet(s.outputs[o].dmxArtNet);
      s.outputs[o].dmxArtSubnet = sanitizeDmxArtSubnet(s.outputs[o].dmxArtSubnet);
      s.outputs[o].dmxArtUniverse = sanitizeDmxArtUniverse(s.outputs[o].dmxArtUniverse);
      if (s.outputs[o].dmxChannels.length() == 0) s.outputs[o].dmxChannels = "1";
      s.outputs[o].dmxPreset = sanitizeDmxPresetId(s.outputs[o].dmxPreset);
      s.outputs[o].dmxFadeMs = sanitizeDmxFadeMs(s.outputs[o].dmxFadeMs);
      if (isAutoMqttTopic(s.outputs[o].mqttTopic, s, s, i, o)) {
        s.outputs[o].mqttTopic = buildDefaultMqttTopic(s, i, o);
      }
      if (s.type == SENSOR_ANALOG && s.outputs[o].outMode == OUT_STRING) s.outputs[o].outMode = OUT_FLOAT;
      if ((s.outputs[o].target == OUT_TARGET_DMX || s.outputs[o].target == OUT_TARGET_DMX_PRESET) && s.outputs[o].outMode == OUT_STRING) s.outputs[o].outMode = OUT_INT;
    }
    s.timeType = static_cast<TimeTriggerType>(prefs.getInt(sensorKey(i, "tt").c_str(), s.timeType));
    if (s.timeType > TIME_INTERVAL) s.timeType = TIME_DAILY;
    s.timeYear = prefs.getInt(sensorKey(i, "ty").c_str(), s.timeYear);
    s.timeMonth = prefs.getInt(sensorKey(i, "tm").c_str(), s.timeMonth);
    s.timeDay = prefs.getInt(sensorKey(i, "td").c_str(), s.timeDay);
    s.timeHour = prefs.getInt(sensorKey(i, "th").c_str(), s.timeHour);
    s.timeMinute = prefs.getInt(sensorKey(i, "tmin").c_str(), s.timeMinute);
    s.timeSecond = prefs.getInt(sensorKey(i, "tsec").c_str(), s.timeSecond);
    s.weeklyMask = prefs.getInt(sensorKey(i, "twm").c_str(), s.weeklyMask);
    s.intervalSeconds = prefs.getUInt(sensorKey(i, "tint").c_str(), s.intervalSeconds);

    if (s.name.length() == 0) s.name = String("Trigger ") + String(i + 1);
    config.sensors[i] = s;
  }

  bool hasLegacy = prefs.isKey("sensor_type");
  if (hasLegacy && !prefs.isKey(sensorKey(0, "type").c_str())) {
    SensorConfig s0 = config.sensors[0];
    String legacyType = prefs.getString("sensor_type", "analog");
    s0.type = (legacyType == "digital") ? SENSOR_BUTTON : SENSOR_ANALOG;
    s0.pin = (s0.type == SENSOR_ANALOG)
               ? prefs.getInt("pot_pin", DEFAULT_ANALOG_PIN)
               : prefs.getInt("digital_pin", DEFAULT_DIGITAL_PIN);
    s0.activeHigh = prefs.getBool("digital_active_high", DEFAULT_DIGITAL_ACTIVE_HIGH);
    s0.pullup = prefs.getBool("digital_pullup", DEFAULT_DIGITAL_PULLUP);
    s0.outputs[0].oscAddress = prefs.getString("osc_address", s0.outputs[0].oscAddress);
    s0.invert = prefs.getBool("invert", DEFAULT_INVERT);
    s0.outputs[0].outMin = prefs.getFloat("out_min", DEFAULT_OUT_MIN);
    s0.outputs[0].outMax = prefs.getFloat("out_max", DEFAULT_OUT_MAX);
    bool outIsFloat = prefs.getBool("out_is_float", false);
    s0.outputs[0].outMode = outIsFloat ? OUT_FLOAT : OUT_INT;
    config.sensors[0] = s0;
  }

  for (int i = 0; i < MAX_DMX_PRESETS; i++) {
    String key = String("dmxp_") + String(i);
    dmxPresetText[i] = prefs.getString(key.c_str(), "");
    dmxPresetText[i].trim();
  }

  prefs.end();
}

static void saveConfig() {
  prefs.begin("osc", false);
  prefs.putString("hostname", config.hostname);
  prefs.putInt("net_mode", config.netMode);
  prefs.putBool("osci_en", config.oscInEnabled);
  prefs.putUInt("osci_port", config.oscInPort);
  prefs.putBool("art_en", config.artnetEnabled);
  prefs.putBool("sacn_en", config.sacnEnabled);
  prefs.putBool("dmi_art", config.dmxInArtNetEnabled);
  prefs.putBool("dmi_sac", config.dmxInSacnEnabled);
  prefs.putUInt("dmi_src", config.dmxInSourceMode);
  prefs.putString("client_ip", ipToString(config.clientIp));
  prefs.putInt("client_port", config.clientPort);
  prefs.putBool("use_static", config.useStatic);
  prefs.putString("static_ip", ipToString(config.staticIp));
  prefs.putString("gateway", ipToString(config.gateway));
  prefs.putString("subnet", ipToString(config.subnet));
  prefs.putString("dns1", ipToString(config.dns1));
  prefs.putString("dns2", ipToString(config.dns2));
  prefs.putString("wifi_ssid", config.wifiSsid);
  prefs.putString("wifi_pass", config.wifiPass);
  prefs.putString("username", config.username);
  prefs.putUInt("test_dev", config.testDevice);
  prefs.putUInt("click_db", config.clickDebounceMs);
  prefs.putUInt("click_gap", config.multiClickGapMs);
  prefs.putUInt("long_ms", config.longPressMs);
  prefs.putBool("analog_smooth", config.analogSmoothing);
  prefs.putUInt("time_mode", config.timeMode);
  prefs.putString("ntp_srv", config.ntpServer);
  prefs.putString("tz", config.timeZone);
  prefs.putUInt("time_manual", config.manualEpoch);
  prefs.putBool("time_24", config.timeDisplay24h);
  prefs.putBool("mq_en", config.mqttEnabled);
  prefs.putUInt("mq_dev", config.mqttDevice);
  prefs.putString("mq_host", config.mqttHost);
  prefs.putUInt("mq_port", config.mqttPort);
  prefs.putString("mq_user", config.mqttUser);
  prefs.putString("mq_pass", config.mqttPass);
  prefs.putString("mq_cid", config.mqttClientId);
  prefs.putString("mq_base", config.mqttBaseTopic);
  prefs.putBool("pwd_en", config.passwordEnabled);
  prefs.putString("pwd", config.password);
  prefs.putUInt("tr_count", config.triggerCount);
  prefs.putString("tr_order", buildOrderList(config.triggerOrder, config.triggerCount));

  prefs.putUInt("dev_count", config.oscDeviceCount);
  prefs.putString("dev_order", buildOrderList(config.oscDeviceOrder, config.oscDeviceCount));

  bool saveDev[MAX_OSC_DEVICES];
  for (int i = 0; i < MAX_OSC_DEVICES; i++) saveDev[i] = false;
  for (int i = 0; i < config.oscDeviceCount; i++) {
    int idx = config.oscDeviceOrder[i];
    if (idx >= 0 && idx < MAX_OSC_DEVICES) saveDev[idx] = true;
  }
  for (int i = 0; i < MAX_OSC_DEVICES; i++) {
    if (!saveDev[i]) clearDeviceKeys(i);
  }
  for (int i = 0; i < MAX_OSC_DEVICES; i++) {
    if (!saveDev[i]) continue;
    const OscDevice& d = config.oscDevices[i];
    String keyEn = String("dev") + i + "_en";
    String keyName = String("dev") + i + "_name";
    String keyIp = String("dev") + i + "_ip";
    String keyPort = String("dev") + i + "_port";
    String keyHbEn = String("dev") + i + "_hb_en";
    String keyHbAddr = String("dev") + i + "_hb_addr";
    String keyHbMs = String("dev") + i + "_hb_ms";
    prefs.putBool(keyEn.c_str(), d.enabled);
    prefs.putString(keyName.c_str(), d.name);
    prefs.putString(keyIp.c_str(), ipToString(d.ip));
    prefs.putInt(keyPort.c_str(), d.port);
    prefs.putBool(keyHbEn.c_str(), d.hbEnabled);
    prefs.putString(keyHbAddr.c_str(), d.hbAddress);
    prefs.putUInt(keyHbMs.c_str(), d.hbMs);
  }

  bool saveTrigger[MAX_TRIGGERS];
  for (int i = 0; i < MAX_TRIGGERS; i++) saveTrigger[i] = false;
  for (int i = 0; i < config.triggerCount; i++) {
    int idx = config.triggerOrder[i];
    if (idx >= 0 && idx < MAX_TRIGGERS) saveTrigger[idx] = true;
  }
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    if (!saveTrigger[i]) clearSensorKeys(i);
  }
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    if (!saveTrigger[i]) continue;
    const SensorConfig& s = config.sensors[i];
    prefs.putBool(sensorKey(i, "en").c_str(), s.enabled);
    prefs.putString(sensorKey(i, "name").c_str(), s.name);
    prefs.putInt(sensorKey(i, "src").c_str(), s.source);
    prefs.putInt(sensorKey(i, "type").c_str(), s.type);
    prefs.putUInt(sensorKey(i, "oc").c_str(), s.outputCount);

    // Source-specific fields only
    if (s.source == SRC_OSC) {
      prefs.putString(sensorKey(i, "osia").c_str(), s.oscInAddress);
      prefs.putUInt(sensorKey(i, "osit").c_str(), s.oscInArgType);
      prefs.putUInt(sensorKey(i, "osim").c_str(), s.oscInMatchMode);
      prefs.putFloat(sensorKey(i, "osiv").c_str(), s.oscInValue);
      prefs.putFloat(sensorKey(i, "osmn").c_str(), s.oscInMin);
      prefs.putFloat(sensorKey(i, "osmx").c_str(), s.oscInMax);
      prefs.putString(sensorKey(i, "osis").c_str(), s.oscInString);
    }
    if (s.source == SRC_DMX) {
      prefs.putUShort(sensorKey(i, "diu").c_str(), s.dmxInUniverse);
      prefs.putUShort(sensorKey(i, "dic").c_str(), s.dmxInChannel);
      prefs.putUChar(sensorKey(i, "dim").c_str(), s.dmxInMode);
      prefs.putUChar(sensorKey(i, "dit").c_str(), s.dmxInThreshold);
      prefs.putUChar(sensorKey(i, "din").c_str(), s.dmxInRangeMin);
      prefs.putUChar(sensorKey(i, "dix").c_str(), s.dmxInRangeMax);
    }
    if (s.source == SRC_TIME) {
      prefs.putInt(sensorKey(i, "tt").c_str(), s.timeType);
      prefs.putInt(sensorKey(i, "ty").c_str(), s.timeYear);
      prefs.putInt(sensorKey(i, "tm").c_str(), s.timeMonth);
      prefs.putInt(sensorKey(i, "td").c_str(), s.timeDay);
      prefs.putInt(sensorKey(i, "th").c_str(), s.timeHour);
      prefs.putInt(sensorKey(i, "tmin").c_str(), s.timeMinute);
      prefs.putInt(sensorKey(i, "tsec").c_str(), s.timeSecond);
      prefs.putInt(sensorKey(i, "twm").c_str(), s.weeklyMask);
      prefs.putUInt(sensorKey(i, "tint").c_str(), s.intervalSeconds);
    }
    if (s.source == SRC_SENSORS) {
      prefs.putInt(sensorKey(i, "pin").c_str(), s.pin);
      prefs.putBool(sensorKey(i, "inv").c_str(), s.invert);
      if (s.type == SENSOR_ANALOG) {
        // analog-only: smoothing handled globally
      }
      if (s.type == SENSOR_BUTTON || s.type == SENSOR_TOGGLE || s.type == SENSOR_BUTTON_DOUBLE ||
          s.type == SENSOR_BUTTON_TRIPLE || s.type == SENSOR_BUTTON_LONG || s.type == SENSOR_DIGITAL || s.type == SENSOR_GPIO) {
        prefs.putBool(sensorKey(i, "ah").c_str(), s.activeHigh);
        prefs.putBool(sensorKey(i, "pu").c_str(), s.pullup);
      }
      if (s.type == SENSOR_BUTTON || s.type == SENSOR_BUTTON_DOUBLE || s.type == SENSOR_BUTTON_TRIPLE || s.type == SENSOR_BUTTON_LONG) {
        prefs.putBool(sensorKey(i, "cd_en").c_str(), s.cooldownEnabled);
        prefs.putUInt(sensorKey(i, "cd_ms").c_str(), s.cooldownMs);
      }
      if (s.type == SENSOR_ENCODER) {
        prefs.putInt(sensorKey(i, "clk").c_str(), s.encClkPin);
        prefs.putInt(sensorKey(i, "dt").c_str(), s.encDtPin);
        prefs.putInt(sensorKey(i, "sw").c_str(), s.encSwPin);
        prefs.putBool(sensorKey(i, "enc_app").c_str(), s.encAppendSign);
        prefs.putBool(sensorKey(i, "enc_arg").c_str(), s.encSignArg);
        prefs.putBool(sensorKey(i, "enc_inv").c_str(), s.encInvert);
      }
      if (s.type == SENSOR_HCSR04) {
        prefs.putInt(sensorKey(i, "hct").c_str(), s.hcTrigPin);
        prefs.putInt(sensorKey(i, "hce").c_str(), s.hcEchoPin);
        prefs.putFloat(sensorKey(i, "hcmin").c_str(), s.hcMinCm);
        prefs.putFloat(sensorKey(i, "hcmax").c_str(), s.hcMaxCm);
      }
    }
    if (s.conditionEnabled) {
      prefs.putBool(sensorKey(i, "cden").c_str(), s.conditionEnabled);
      prefs.putString(sensorKey(i, "cdvar").c_str(), s.conditionVar);
      prefs.putUInt(sensorKey(i, "cdop").c_str(), s.conditionOp);
      prefs.putFloat(sensorKey(i, "cdval").c_str(), s.conditionValue);
    } else {
      // Remove stale keys, else a disabled condition resurrects on reboot
      removeIfExists(sensorKey(i, "cden"));
      removeIfExists(sensorKey(i, "cdvar"));
      removeIfExists(sensorKey(i, "cdop"));
      removeIfExists(sensorKey(i, "cdval"));
    }

    for (int o = 0; o < s.outputCount; o++) {
      const SensorConfig::OutputConfig& out = s.outputs[o];
      prefs.putInt(outputKey(i, o, "tgt").c_str(), out.target);
      if (out.delayMs > 0) prefs.putUInt(outputKey(i, o, "dly").c_str(), out.delayMs);
      else removeIfExists(outputKey(i, o, "dly"));
      // Common output fields
      prefs.putInt(outputKey(i, o, "omode").c_str(), out.outMode);
      prefs.putFloat(outputKey(i, o, "omin").c_str(), out.outMin);
      prefs.putFloat(outputKey(i, o, "omax").c_str(), out.outMax);
      prefs.putBool(outputKey(i, o, "smin").c_str(), out.sendMinOnRelease);
      if (out.outMode == OUT_STRING) {
        prefs.putString(outputKey(i, o, "on").c_str(), out.onString);
        prefs.putString(outputKey(i, o, "off").c_str(), out.offString);
      }
      // Target-specific fields
      if (out.target == OUT_TARGET_OSC) {
        prefs.putInt(outputKey(i, o, "dev").c_str(), out.device);
        prefs.putString(outputKey(i, o, "addr").c_str(), out.oscAddress);
        if (s.type == SENSOR_ENCODER) {
          prefs.putString(outputKey(i, o, "baddr").c_str(), out.buttonAddress);
          prefs.putFloat(outputKey(i, o, "bmin").c_str(), out.buttonOutMin);
          prefs.putFloat(outputKey(i, o, "bmax").c_str(), out.buttonOutMax);
          prefs.putInt(outputKey(i, o, "bmode").c_str(), out.buttonOutMode);
          if (out.buttonOutMode == OUT_STRING) {
            prefs.putString(outputKey(i, o, "bon").c_str(), out.buttonOnString);
            prefs.putString(outputKey(i, o, "boff").c_str(), out.buttonOffString);
          }
        }
      }
      if (out.target == OUT_TARGET_UDP) {
        prefs.putInt(outputKey(i, o, "dev").c_str(), out.device);
        prefs.putString(outputKey(i, o, "udp").c_str(), out.udpPayload);
      }
      if (out.target == OUT_TARGET_GPIO) {
        prefs.putInt(outputKey(i, o, "gpin").c_str(), out.gpioPin);
        prefs.putInt(outputKey(i, o, "gmode").c_str(), out.gpioMode);
        if (out.gpioMode == GPIO_OUT_PULSE) prefs.putUInt(outputKey(i, o, "gpms").c_str(), out.gpioPulseMs);
        prefs.putBool(outputKey(i, o, "ginv").c_str(), out.gpioInvert);
      }
      if (out.target == OUT_TARGET_HTTP) {
        prefs.putInt(outputKey(i, o, "hm").c_str(), out.httpMethod);
        prefs.putString(outputKey(i, o, "hu").c_str(), out.httpUrl);
        if (out.httpMethod == HTTPM_POST) prefs.putString(outputKey(i, o, "hb").c_str(), out.httpBody);
        prefs.putString(outputKey(i, o, "hip").c_str(), out.httpIp);
        prefs.putUInt(outputKey(i, o, "hpt").c_str(), out.httpPort);
        prefs.putUInt(outputKey(i, o, "hdev").c_str(), out.httpDevice);
        prefs.putString(outputKey(i, o, "addr").c_str(), out.oscAddress);
      }
      if (out.target == OUT_TARGET_MQTT) {
        prefs.putString(outputKey(i, o, "mt").c_str(), out.mqttTopic);
        prefs.putString(outputKey(i, o, "mp").c_str(), out.mqttPayload);
        prefs.putBool(outputKey(i, o, "mr").c_str(), out.mqttRetain);
        prefs.putString(outputKey(i, o, "addr").c_str(), out.oscAddress);
      }
      if (out.target == OUT_TARGET_DMX) {
        prefs.putUInt(outputKey(i, o, "dmxp").c_str(), out.dmxProtocol);
        prefs.putUInt(outputKey(i, o, "dmxd").c_str(), out.dmxDest);
        if (out.dmxProtocol == DMX_PROTO_ARTNET) {
          prefs.putUInt(outputKey(i, o, "dmxn").c_str(), out.dmxArtNet);
          prefs.putUInt(outputKey(i, o, "dmxs").c_str(), out.dmxArtSubnet);
          prefs.putUInt(outputKey(i, o, "dmxa").c_str(), out.dmxArtUniverse);
        }
        prefs.putUInt(outputKey(i, o, "dmxu").c_str(), out.dmxUniverse);
        prefs.putString(outputKey(i, o, "dmxc").c_str(), out.dmxChannels);
        if (out.dmxDest == DMX_DEST_UNICAST) prefs.putInt(outputKey(i, o, "dev").c_str(), out.device);
        prefs.putString(outputKey(i, o, "addr").c_str(), out.oscAddress);
      }
      if (out.target == OUT_TARGET_DMX_PRESET) {
        prefs.putUInt(outputKey(i, o, "dmxp").c_str(), out.dmxProtocol);
        prefs.putUInt(outputKey(i, o, "dmxd").c_str(), out.dmxDest);
        prefs.putUInt(outputKey(i, o, "dmxpr").c_str(), out.dmxPreset);
        prefs.putUInt(outputKey(i, o, "dmxfd").c_str(), out.dmxFadeMs);
        if (out.dmxDest == DMX_DEST_UNICAST) prefs.putInt(outputKey(i, o, "dev").c_str(), out.device);
      }
      if (out.target == OUT_TARGET_SET_VAR) {
        prefs.putString(outputKey(i, o, "svn").c_str(), out.setVarName);
        prefs.putFloat(outputKey(i, o, "svv").c_str(), out.setVarValue);
      }
    }
  }

  for (int i = 0; i < MAX_DMX_PRESETS; i++) {
    String key = String("dmxp_") + String(i);
    String text = dmxPresetText[i];
    text.trim();
    if (text.length() == 0) {
      removeIfExists(key);
    } else {
      prefs.putString(key.c_str(), text);
    }
  }

  prefs.end();
}

static void applyNetworkConfig() {
#if USE_ETHERNET
  if (config.netMode == NET_ETHERNET) {
    if (config.useStatic) {
      bool ok = ETH.config(config.staticIp, config.gateway, config.subnet, config.dns1, config.dns2);
      Serial.print("Static IP ");
      Serial.println(ok ? "configured." : "config failed.");
    } else {
      ETH.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
      Serial.println("DHCP enabled.");
    }
    if (!wifiApMode) {
      WiFi.mode(WIFI_OFF);
    }
    return;
  }
#endif
  WiFi.disconnect(true);
  delay(50);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  wifiApMode = false;
  if (config.useStatic) {
    WiFi.config(config.staticIp, config.gateway, config.subnet, config.dns1, config.dns2);
  } else {
    WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE);
  }
  if (config.wifiSsid.length() > 0) {
    WiFi.begin(config.wifiSsid.c_str(), config.wifiPass.c_str());
  }
}

static void pollNtpSync() {
  if (!ntpPending) return;
  time_t now = time(nullptr);
  if (now > 100000) {
    ntpPending = false;
    addLog("NTP time synchronized.");
    return;
  }
  if (millis() - ntpLastAttemptMs > 15000) {
    ntpLastAttemptMs = millis();
    addLog("NTP sync still pending...");
  }
}

static void applyTimeConfig() {
  if (config.timeMode == TIME_NTP) {
    setenv("TZ", config.timeZone.c_str(), 1);
    tzset();
    configTzTime(config.timeZone.c_str(), config.ntpServer.c_str());
    addLog(String("Time sync via NTP: ") + config.ntpServer + " TZ " + config.timeZone);
    ntpPending = true;
    ntpLastAttemptMs = millis();
  } else if (config.manualEpoch > 0) {
    struct timeval tv;
    tv.tv_sec = static_cast<time_t>(config.manualEpoch);
    tv.tv_usec = 0;
    settimeofday(&tv, nullptr);
    setenv("TZ", config.timeZone.c_str(), 1);
    tzset();
    addLog("Time set manually.");
    ntpPending = false;
  }
}

static void startMdns() {
  if (MDNS.begin(config.hostname.c_str())) {
    Serial.print("mDNS ready: http://");
    Serial.print(config.hostname);
    Serial.println(".local/");
  } else {
    Serial.println("mDNS failed to start.");
  }
}

static bool getOscTarget(uint8_t device, IPAddress& ip, uint16_t& port) {
  if (device < MAX_OSC_DEVICES && config.oscDevices[device].enabled) {
    ip = config.oscDevices[device].ip;
    port = config.oscDevices[device].port;
    if (port == 0) port = config.clientPort;
    return true;
  }
  for (int i = 0; i < config.oscDeviceCount; i++) {
    int idx = config.oscDeviceOrder[i];
    if (idx < 0 || idx >= MAX_OSC_DEVICES) continue;
    if (!config.oscDevices[idx].enabled) continue;
    ip = config.oscDevices[idx].ip;
    port = config.oscDevices[idx].port;
    if (port == 0) port = config.clientPort;
    return true;
  }
  return false;
}

static void sendOscIntTo(const IPAddress& ip, uint16_t port, const char* address, int32_t value) {
  uint8_t packet[128];
  size_t idx = 0;

  auto addPaddedString = [&](const char* s) {
    size_t len = strlen(s) + 1; // include null
    if (idx + len >= sizeof(packet)) return;
    memcpy(packet + idx, s, len);
    idx += len;
    while (idx % 4 != 0 && idx < sizeof(packet)) {
      packet[idx++] = 0;
    }
  };

  addPaddedString(address);
  addPaddedString(",i");
  if (idx + 4 > sizeof(packet)) return;
  packet[idx++] = (value >> 24) & 0xFF;
  packet[idx++] = (value >> 16) & 0xFF;
  packet[idx++] = (value >> 8) & 0xFF;
  packet[idx++] = value & 0xFF;

  udp.beginPacket(ip, port);
  udp.write(packet, idx);
  udp.endPacket();
  addLog(String("OSC int ") + address + " " + String(value));
}

static void sendOscInt(const char* address, int32_t value) {
  IPAddress ip;
  uint16_t port;
  if (!getOscTarget(0, ip, port)) return;
  sendOscIntTo(ip, port, address, value);
}

static void sendOscFloatTo(const IPAddress& ip, uint16_t port, const char* address, float value) {
  uint8_t packet[128];
  size_t idx = 0;

  auto addPaddedString = [&](const char* s) {
    size_t len = strlen(s) + 1; // include null
    if (idx + len >= sizeof(packet)) return;
    memcpy(packet + idx, s, len);
    idx += len;
    while (idx % 4 != 0 && idx < sizeof(packet)) {
      packet[idx++] = 0;
    }
  };

  addPaddedString(address);
  addPaddedString(",f");
  if (idx + 4 > sizeof(packet)) return;
  union {
    float f;
    uint32_t u;
  } conv;
  conv.f = value;
  packet[idx++] = (conv.u >> 24) & 0xFF;
  packet[idx++] = (conv.u >> 16) & 0xFF;
  packet[idx++] = (conv.u >> 8) & 0xFF;
  packet[idx++] = conv.u & 0xFF;

  udp.beginPacket(ip, port);
  udp.write(packet, idx);
  udp.endPacket();
  addLog(String("OSC float ") + address + " " + String(value, 4));
}

static void sendOscFloat(const char* address, float value) {
  IPAddress ip;
  uint16_t port;
  if (!getOscTarget(0, ip, port)) return;
  sendOscFloatTo(ip, port, address, value);
}

static void sendOscStringTo(const IPAddress& ip, uint16_t port, const char* address, const char* value) {
  uint8_t packet[256];
  size_t idx = 0;

  auto addPaddedString = [&](const char* s) {
    size_t len = strlen(s) + 1; // include null
    if (idx + len >= sizeof(packet)) return;
    memcpy(packet + idx, s, len);
    idx += len;
    while (idx % 4 != 0 && idx < sizeof(packet)) {
      packet[idx++] = 0;
    }
  };

  addPaddedString(address);
  addPaddedString(",s");
  addPaddedString(value);

  udp.beginPacket(ip, port);
  udp.write(packet, idx);
  udp.endPacket();
  addLog(String("OSC string ") + address + " " + String(value));
}

static void sendOscStringIntTo(const IPAddress& ip, uint16_t port, const char* address, const char* svalue, int32_t ivalue) {
  uint8_t packet[256];
  size_t idx = 0;

  auto addPaddedString = [&](const char* s) {
    size_t len = strlen(s) + 1; // include null
    if (idx + len >= sizeof(packet)) return;
    memcpy(packet + idx, s, len);
    idx += len;
    while (idx % 4 != 0 && idx < sizeof(packet)) {
      packet[idx++] = 0;
    }
  };

  addPaddedString(address);
  addPaddedString(",si");
  addPaddedString(svalue);
  if (idx + 4 > sizeof(packet)) return;
  packet[idx++] = (ivalue >> 24) & 0xFF;
  packet[idx++] = (ivalue >> 16) & 0xFF;
  packet[idx++] = (ivalue >> 8) & 0xFF;
  packet[idx++] = ivalue & 0xFF;

  udp.beginPacket(ip, port);
  udp.write(packet, idx);
  udp.endPacket();
  addLog(String("OSC string+int ") + address + " " + String(svalue) + " " + String(ivalue));
}

static void sendOscStringFloatTo(const IPAddress& ip, uint16_t port, const char* address, const char* svalue, float fvalue) {
  uint8_t packet[256];
  size_t idx = 0;

  auto addPaddedString = [&](const char* s) {
    size_t len = strlen(s) + 1; // include null
    if (idx + len >= sizeof(packet)) return;
    memcpy(packet + idx, s, len);
    idx += len;
    while (idx % 4 != 0 && idx < sizeof(packet)) {
      packet[idx++] = 0;
    }
  };

  addPaddedString(address);
  addPaddedString(",sf");
  addPaddedString(svalue);
  if (idx + 4 > sizeof(packet)) return;
  union {
    float f;
    uint32_t u;
  } conv;
  conv.f = fvalue;
  packet[idx++] = (conv.u >> 24) & 0xFF;
  packet[idx++] = (conv.u >> 16) & 0xFF;
  packet[idx++] = (conv.u >> 8) & 0xFF;
  packet[idx++] = conv.u & 0xFF;

  udp.beginPacket(ip, port);
  udp.write(packet, idx);
  udp.endPacket();
  addLog(String("OSC string+float ") + address + " " + String(svalue) + " " + String(fvalue, 3));
}

static void sendOscString(const char* address, const char* value) {
  IPAddress ip;
  uint16_t port;
  if (!getOscTarget(0, ip, port)) return;
  sendOscStringTo(ip, port, address, value);
}

static void sendUdpRawTo(const IPAddress& ip, uint16_t port, const String& payload) {
  if (payload.length() == 0) return;
  udp.beginPacket(ip, port);
  udp.write(reinterpret_cast<const uint8_t*>(payload.c_str()), payload.length());
  udp.endPacket();
  addLog(String("UDP raw ") + ipToString(ip) + ":" + String(port) + " " + payload);
}

static void initSacnCid() {
  if (sacnCidReady) return;
  memset(sacnCid, 0, sizeof(sacnCid));
  uint64_t mac = ESP.getEfuseMac();
  sacnCid[0] = 0x53;
  sacnCid[1] = 0x74;
  sacnCid[2] = 0x61;
  sacnCid[3] = 0x67;
  sacnCid[4] = 0x65;
  sacnCid[5] = 0x4d;
  sacnCid[6] = 0x6f;
  sacnCid[7] = 0x64;
  for (int i = 0; i < 8; i++) {
    sacnCid[8 + i] = static_cast<uint8_t>((mac >> ((7 - i) * 8)) & 0xFF);
  }
  sacnCidReady = true;
}

static int parseDmxChannels(const String& specRaw, uint16_t* channels, int maxChannels) {
  if (channels == nullptr || maxChannels <= 0) return 0;
  bool used[513];
  for (int i = 0; i <= 512; i++) used[i] = false;
  int count = 0;
  String spec = specRaw;
  spec.trim();
  if (spec.length() == 0) spec = "1";
  int start = 0;
  while (start < static_cast<int>(spec.length())) {
    int comma = spec.indexOf(',', start);
    if (comma < 0) comma = spec.length();
    String part = spec.substring(start, comma);
    part.trim();
    if (part.length() > 0) {
      int dash = part.indexOf('-');
      int from = 0;
      int to = 0;
      if (dash >= 0) {
        String lhs = part.substring(0, dash);
        String rhs = part.substring(dash + 1);
        lhs.trim();
        rhs.trim();
        from = lhs.toInt();
        to = rhs.toInt();
      } else {
        from = part.toInt();
        to = from;
      }
      if (from > to) {
        int tmp = from;
        from = to;
        to = tmp;
      }
      if (from < 1) from = 1;
      if (to > 512) to = 512;
      for (int c = from; c <= to; c++) {
        if (!used[c]) {
          used[c] = true;
          if (count < maxChannels) channels[count++] = static_cast<uint16_t>(c);
        }
      }
    }
    start = comma + 1;
  }
  if (count == 0) {
    channels[0] = 1;
    return 1;
  }
  return count;
}

static int parseDmxPresetText(const String& text, DmxFadePoint* points, int maxPoints, String* normalizedOut, String& err) {
  if (normalizedOut != nullptr) *normalizedOut = "";
  err = "";
  int total = 0;
  int start = 0;
  while (start <= static_cast<int>(text.length())) {
    int nl = text.indexOf('\n', start);
    if (nl < 0) nl = text.length();
    String line = text.substring(start, nl);
    line.trim();
    start = nl + 1;
    if (line.length() == 0) continue;
    if (line.startsWith("#")) continue;

    int c1 = line.indexOf(',');
    int c2 = (c1 >= 0) ? line.indexOf(',', c1 + 1) : -1;
    if (c1 <= 0 || c2 <= c1 + 1 || c2 >= static_cast<int>(line.length()) - 1) {
      err = String("Invalid preset row: ") + line;
      return -1;
    }
    String uniStr = line.substring(0, c1);
    String chStr = line.substring(c1 + 1, c2);
    String valStr = line.substring(c2 + 1);
    uniStr.trim();
    chStr.trim();
    valStr.trim();

    int universe = uniStr.toInt();
    if (universe < 1 || universe > MAX_DMX_PRESET_UNIVERSE) {
      err = String("Universe out of range (1-") + String(MAX_DMX_PRESET_UNIVERSE) + "): " + uniStr;
      return -1;
    }

    int value = valStr.toInt();
    if (value < 0 || value > 255) {
      err = String("Value out of range (0-255): ") + valStr;
      return -1;
    }

    uint16_t channels[512];
    int channelCount = parseDmxChannels(chStr, channels, 512);
    if (channelCount <= 0) {
      err = String("No channels in row: ") + line;
      return -1;
    }

    if (normalizedOut != nullptr) {
      if (normalizedOut->length() > 0) *normalizedOut += "\n";
      *normalizedOut += String(universe) + "," + chStr + "," + String(value);
    }

    for (int i = 0; i < channelCount; i++) {
      if (total >= maxPoints) {
        err = String("Preset exceeds point limit (") + String(maxPoints) + ").";
        return -1;
      }
      if (points != nullptr) {
        points[total].universe = static_cast<uint16_t>(universe);
        points[total].channel = channels[i];
        points[total].fromValue = 0;
        points[total].toValue = static_cast<uint8_t>(value);
      }
      total++;
    }
  }
  return total;
}

static bool parseDmxPresetRow(const String& line, int& universe, String& channelsExpr, int& value) {
  String row = line;
  row.trim();
  if (row.length() == 0 || row.startsWith("#")) return false;
  int c1 = row.indexOf(',');
  int c2 = (c1 >= 0) ? row.indexOf(',', c1 + 1) : -1;
  if (c1 <= 0 || c2 <= c1 + 1 || c2 >= static_cast<int>(row.length()) - 1) return false;

  String uniStr = row.substring(0, c1);
  channelsExpr = row.substring(c1 + 1, c2);
  String valStr = row.substring(c2 + 1);
  uniStr.trim();
  channelsExpr.trim();
  valStr.trim();

  universe = uniStr.toInt();
  value = valStr.toInt();
  if (universe < 1 || universe > MAX_DMX_PRESET_UNIVERSE) return false;
  if (value < 0 || value > 255) return false;
  return channelsExpr.length() > 0;
}

static void fillDmxUniverseValues(const String& text, int universe, int16_t values[512]) {
  for (int i = 0; i < 512; i++) values[i] = -1;
  int start = 0;
  while (start <= static_cast<int>(text.length())) {
    int nl = text.indexOf('\n', start);
    if (nl < 0) nl = text.length();
    String line = text.substring(start, nl);
    start = nl + 1;

    int rowUni = 0;
    int rowVal = 0;
    String rowChannels;
    if (!parseDmxPresetRow(line, rowUni, rowChannels, rowVal)) continue;
    if (rowUni != universe) continue;

    uint16_t channels[512];
    int channelCount = parseDmxChannels(rowChannels, channels, 512);
    for (int i = 0; i < channelCount; i++) {
      int ch = channels[i];
      if (ch >= 1 && ch <= 512) values[ch - 1] = static_cast<int16_t>(rowVal);
    }
  }
}

static String removeDmxUniverseRows(const String& text, int universe) {
  String out;
  int start = 0;
  while (start <= static_cast<int>(text.length())) {
    int nl = text.indexOf('\n', start);
    if (nl < 0) nl = text.length();
    String line = text.substring(start, nl);
    start = nl + 1;
    line.trim();
    if (line.length() == 0) continue;

    int rowUni = 0;
    int rowVal = 0;
    String rowChannels;
    bool isRow = parseDmxPresetRow(line, rowUni, rowChannels, rowVal);
    if (isRow && rowUni == universe) continue;

    if (out.length() > 0) out += "\n";
    out += line;
  }
  return out;
}

static String serializeDmxUniverseRows(int universe, const int16_t values[512]) {
  String out;
  for (int ch = 1; ch <= 512; ) {
    int16_t value = values[ch - 1];
    if (value < 0) {
      ch++;
      continue;
    }
    int start = ch;
    int end = ch;
    while (end + 1 <= 512 && values[end] == value) end++;

    if (out.length() > 0) out += "\n";
    out += String(universe) + ",";
    if (start == end) {
      out += String(start);
    } else {
      out += String(start) + "-" + String(end);
    }
    out += "," + String(value);
    ch = end + 1;
  }
  return out;
}

static int totalDmxPresetPointsExcluding(int excludeIndex) {
  int total = 0;
  for (int i = 0; i < MAX_DMX_PRESETS; i++) {
    if (i == excludeIndex) continue;
    const String& text = dmxPresetText[i];
    if (text.length() == 0) continue;
    String err;
    int count = parseDmxPresetText(text, nullptr, MAX_DMX_PRESET_POINTS, nullptr, err);
    if (count > 0) total += count;
  }
  return total;
}

static uint16_t encodeArtNetPortAddressOneBased(uint8_t net, uint8_t subnet, uint8_t universe) {
  uint16_t abs = (static_cast<uint16_t>(net) << 8) |
                 (static_cast<uint16_t>(subnet) << 4) |
                 static_cast<uint16_t>(universe);
  return abs + 1;
}

static void decodeArtNetPortAddressOneBased(uint16_t oneBased, uint8_t& net, uint8_t& subnet, uint8_t& universe) {
  uint16_t abs = (oneBased > 0) ? static_cast<uint16_t>(oneBased - 1) : 0;
  net = static_cast<uint8_t>((abs >> 8) & 0x7F);
  subnet = static_cast<uint8_t>((abs >> 4) & 0x0F);
  universe = static_cast<uint8_t>(abs & 0x0F);
}

static uint16_t getArtNetUniverseOneBased(const SensorConfig::OutputConfig& out) {
  uint8_t net = sanitizeDmxArtNet(out.dmxArtNet);
  uint8_t subnet = sanitizeDmxArtSubnet(out.dmxArtSubnet);
  uint8_t universe = sanitizeDmxArtUniverse(out.dmxArtUniverse);
  return encodeArtNetPortAddressOneBased(net, subnet, universe);
}

static DmxStreamState* getDmxStream(uint8_t protocol, uint8_t device, uint16_t universe) {
  DmxStreamState* freeSlot = nullptr;
  for (int i = 0; i < MAX_DMX_STREAMS; i++) {
    DmxStreamState& st = dmxStreams[i];
    if (!st.used) {
      if (freeSlot == nullptr) freeSlot = &st;
      continue;
    }
    if (st.protocol == protocol && st.device == device && st.universe == universe) {
      return &st;
    }
  }
  if (freeSlot != nullptr) {
    freeSlot->used = true;
    freeSlot->protocol = protocol;
    freeSlot->device = device;
    freeSlot->universe = universe;
    freeSlot->sequence = 0;
    memset(freeSlot->data, 0, sizeof(freeSlot->data));
    return freeSlot;
  }
  return nullptr;
}

static uint8_t toDmxValue(const SensorConfig::OutputConfig& out, float norm) {
  float outputValue = mapOutput(out, norm);
  outputValue = snapOutput(out, outputValue);
  if (out.outMode == OUT_STRING) {
    return norm >= 0.5f ? 255 : 0;
  }
  int v = lroundf(outputValue);
  if (v < 0) v = 0;
  if (v > 255) v = 255;
  return static_cast<uint8_t>(v);
}

static void sendArtNetDmx(const IPAddress& ip, uint16_t universeOneBased, const uint8_t* dmxData, uint16_t channels, uint8_t sequence) {
  if (!isValidIp(ip) || dmxData == nullptr) return;
  if (channels == 0) channels = 2;
  if (channels > 512) channels = 512;
  if (channels & 1) channels++;
  if (channels > 512) channels = 512;

  uint8_t packet[18 + 512];
  memset(packet, 0, sizeof(packet));
  memcpy(packet, "Art-Net", 7);
  packet[7] = 0x00;
  packet[8] = 0x00;   // OpCode low byte
  packet[9] = 0x50;   // OpCode high byte (ArtDMX)
  packet[10] = 0x00;  // ProtVerHi
  packet[11] = 14;    // ProtVerLo
  packet[12] = sequence;
  packet[13] = 0x00;  // Physical
  uint16_t universe = (universeOneBased > 0) ? static_cast<uint16_t>(universeOneBased - 1) : 0;
  packet[14] = static_cast<uint8_t>(universe & 0xFF);
  packet[15] = static_cast<uint8_t>((universe >> 8) & 0xFF);
  packet[16] = static_cast<uint8_t>((channels >> 8) & 0xFF);
  packet[17] = static_cast<uint8_t>(channels & 0xFF);
  memcpy(packet + 18, dmxData, channels);
  udp.beginPacket(ip, ARTNET_PORT);
  udp.write(packet, 18 + channels);
  udp.endPacket();
}

static void putFlagsLength(uint8_t* p, uint16_t length) {
  p[0] = static_cast<uint8_t>(0x70 | ((length >> 8) & 0x0F));
  p[1] = static_cast<uint8_t>(length & 0xFF);
}

static void sendSacnDmx(const IPAddress& ip, uint16_t universeOneBased, const uint8_t* dmxData, uint8_t sequence) {
  if (!isValidIp(ip) || dmxData == nullptr) return;
  initSacnCid();

  uint8_t packet[638];
  memset(packet, 0, sizeof(packet));

  packet[0] = 0x00; packet[1] = 0x10; // Preamble Size
  packet[2] = 0x00; packet[3] = 0x00; // Post-amble Size
  const uint8_t acnId[12] = {0x41,0x53,0x43,0x2d,0x45,0x31,0x2e,0x31,0x37,0x00,0x00,0x00};
  memcpy(packet + 4, acnId, 12);

  putFlagsLength(packet + 16, 0x026e); // Root length
  packet[18] = 0x00; packet[19] = 0x00; packet[20] = 0x00; packet[21] = 0x04; // Root vector
  memcpy(packet + 22, sacnCid, 16);

  putFlagsLength(packet + 38, 0x0258); // Framing length
  packet[40] = 0x00; packet[41] = 0x00; packet[42] = 0x00; packet[43] = 0x02; // Framing vector
  String sourceName = "StageMod";
  if (config.hostname.length() > 0) sourceName = config.hostname;
  if (sourceName.length() > 63) sourceName = sourceName.substring(0, 63);
  memcpy(packet + 44, sourceName.c_str(), sourceName.length());
  packet[108] = 100; // priority
  packet[109] = 0x00; packet[110] = 0x00; // sync address
  packet[111] = sequence;
  packet[112] = 0x00; // options
  uint16_t universe = (universeOneBased > 0) ? universeOneBased : 1;
  packet[113] = static_cast<uint8_t>((universe >> 8) & 0xFF);
  packet[114] = static_cast<uint8_t>(universe & 0xFF);

  putFlagsLength(packet + 115, 0x020b); // DMP length
  packet[117] = 0x02; // DMP vector
  packet[118] = 0xa1; // address and data type
  packet[119] = 0x00; packet[120] = 0x00; // first property address
  packet[121] = 0x00; packet[122] = 0x01; // address increment
  packet[123] = 0x02; packet[124] = 0x01; // property value count (513)
  packet[125] = 0x00; // START Code
  memcpy(packet + 126, dmxData, 512);

  udp.beginPacket(ip, SACN_PORT);
  udp.write(packet, sizeof(packet));
  udp.endPacket();
}

static IPAddress getArtNetBroadcastAddress() {
  IPAddress local = getLocalIp();
  if (!isValidIp(local)) return IPAddress(255, 255, 255, 255);

  IPAddress mask(255, 255, 255, 0);
#if USE_ETHERNET
  if (config.netMode == NET_ETHERNET && ETH.linkUp()) {
    IPAddress m = ETH.subnetMask();
    if (m[0] != 0) mask = m;
  } else
#endif
  if (WiFi.status() == WL_CONNECTED) {
    IPAddress m = WiFi.subnetMask();
    if (m[0] != 0) mask = m;
  }

  IPAddress out;
  for (int i = 0; i < 4; i++) {
    out[i] = static_cast<uint8_t>((local[i] & mask[i]) | (~mask[i]));
  }
  return out;
}

static IPAddress getSacnMulticastAddress(uint16_t universeOneBased) {
  uint16_t u = (universeOneBased > 0) ? universeOneBased : 1;
  if (u > 63999) u = 63999;
  return IPAddress(239, 255, static_cast<uint8_t>((u >> 8) & 0xFF), static_cast<uint8_t>(u & 0xFF));
}

static bool resolveDmxDestinationRaw(uint8_t dmxDest, uint8_t device, uint8_t proto, uint16_t universe, IPAddress& ip) {
  if (sanitizeDmxDest(dmxDest) == DMX_DEST_UNICAST) {
    uint16_t ignoredPort;
    return getOscTarget(device, ip, ignoredPort);
  }
  if (proto == DMX_PROTO_ARTNET) {
    ip = getArtNetBroadcastAddress();
  } else {
    ip = getSacnMulticastAddress(universe);
  }
  return true;
}

static bool resolveDmxDestination(const SensorConfig::OutputConfig& out, uint8_t proto, uint16_t universe, IPAddress& ip) {
  return resolveDmxDestinationRaw(out.dmxDest, out.device, proto, universe, ip);
}

static bool sendDmxStreamFrame(uint8_t proto, uint8_t dmxDest, uint8_t device, uint16_t universe, DmxStreamState* stream) {
  if (stream == nullptr) return false;
  IPAddress ip;
  if (!resolveDmxDestinationRaw(dmxDest, device, proto, universe, ip)) return false;
  uint8_t sequence = ++stream->sequence;
  if (sequence == 0) sequence = ++stream->sequence;
  if (proto == DMX_PROTO_ARTNET) {
    sendArtNetDmx(ip, universe, stream->data, 512, sequence);
  } else {
    sendSacnDmx(ip, universe, stream->data, sequence);
  }
  return true;
}

static bool triggerDmxPreset(const SensorConfig::OutputConfig& out) {
  uint8_t proto = sanitizeDmxProtocol(out.dmxProtocol);
  if (proto == DMX_PROTO_ARTNET && !config.artnetEnabled) return false;
  if (proto == DMX_PROTO_SACN && !config.sacnEnabled) return false;

  uint8_t preset = sanitizeDmxPresetId(out.dmxPreset);
  int presetIndex = static_cast<int>(preset) - 1;
  if (presetIndex < 0 || presetIndex >= MAX_DMX_PRESETS) return false;
  String text = dmxPresetText[presetIndex];
  text.trim();
  if (text.length() == 0) {
    addLog(String("DMX preset ") + String(preset) + " is empty.");
    return false;
  }

  String parseErr;
  int pointCount = parseDmxPresetText(text, dmxFadeState.points, MAX_DMX_PRESET_POINTS, nullptr, parseErr);
  if (pointCount <= 0) {
    addLog(String("DMX preset parse failed: ") + parseErr);
    return false;
  }

  uint8_t dmxDest = sanitizeDmxDest(out.dmxDest);
  uint8_t streamDevice = (dmxDest == DMX_DEST_UNICAST) ? out.device : 255;
  const int DMX_CHANNEL_BYTES = 64; // 512 bits
  uint8_t targeted[MAX_DMX_PRESET_UNIVERSE][DMX_CHANNEL_BYTES];
  memset(targeted, 0, sizeof(targeted));
  auto setTargeted = [&](uint16_t universe, uint16_t channel) {
    if (universe < 1 || universe > MAX_DMX_PRESET_UNIVERSE) return;
    if (channel < 1 || channel > 512) return;
    int u = static_cast<int>(universe - 1);
    int c = static_cast<int>(channel - 1);
    targeted[u][c >> 3] |= static_cast<uint8_t>(1 << (c & 7));
  };
  auto isTargeted = [&](uint16_t universe, uint16_t channel) -> bool {
    if (universe < 1 || universe > MAX_DMX_PRESET_UNIVERSE) return false;
    if (channel < 1 || channel > 512) return false;
    int u = static_cast<int>(universe - 1);
    int c = static_cast<int>(channel - 1);
    return (targeted[u][c >> 3] & static_cast<uint8_t>(1 << (c & 7))) != 0;
  };

  dmxFadeState.active = true;
  dmxFadeState.protocol = proto;
  dmxFadeState.dest = dmxDest;
  dmxFadeState.device = out.device;
  dmxFadeState.startMs = millis();
  dmxFadeState.durationMs = sanitizeDmxFadeMs(out.dmxFadeMs);
  dmxFadeState.lastFrameMs = 0;
  dmxFadeState.pointCount = 0;
  dmxFadeState.usedUniverseCount = 0;
  auto addUsedUniverse = [&](uint16_t universe) {
    for (int u = 0; u < dmxFadeState.usedUniverseCount; u++) {
      if (dmxFadeState.usedUniverses[u] == universe) return;
    }
    if (dmxFadeState.usedUniverseCount < MAX_DMX_PRESET_UNIVERSE) {
      dmxFadeState.usedUniverses[dmxFadeState.usedUniverseCount++] = universe;
    }
  };

  for (int i = 0; i < pointCount; i++) {
    DmxFadePoint& p = dmxFadeState.points[i];
    uint16_t universe = p.universe;
    uint16_t channel = p.channel;
    if (channel < 1 || channel > 512) continue;
    DmxStreamState* stream = getDmxStream(proto, streamDevice, universe);
    if (stream == nullptr) {
      addLog("DMX preset failed: stream capacity reached.");
      dmxFadeState.active = false;
      return false;
    }
    p.fromValue = stream->data[channel - 1];
    setTargeted(universe, channel);
    addUsedUniverse(universe);
  }
  dmxFadeState.pointCount = static_cast<uint16_t>(pointCount);

  // Crossfade-down: any currently active DMX level that is not in the new preset
  // is faded to 0 so transitions between looks are smooth in both directions.
  bool pointLimitHit = false;
  for (int si = 0; si < MAX_DMX_STREAMS && !pointLimitHit; si++) {
    DmxStreamState& st = dmxStreams[si];
    if (!st.used) continue;
    if (st.protocol != proto) continue;
    if (st.device != streamDevice) continue;
    if (st.universe < 1 || st.universe > MAX_DMX_PRESET_UNIVERSE) continue;

    for (int ch = 1; ch <= 512; ch++) {
      uint8_t current = st.data[ch - 1];
      if (current == 0) continue;
      if (isTargeted(st.universe, static_cast<uint16_t>(ch))) continue;
      if (pointCount >= MAX_DMX_PRESET_POINTS) {
        pointLimitHit = true;
        break;
      }
      DmxFadePoint& p = dmxFadeState.points[pointCount++];
      p.universe = st.universe;
      p.channel = static_cast<uint16_t>(ch);
      p.fromValue = current;
      p.toValue = 0;
      setTargeted(st.universe, static_cast<uint16_t>(ch));
      addUsedUniverse(st.universe);
    }
  }
  dmxFadeState.pointCount = static_cast<uint16_t>(pointCount);
  if (pointLimitHit) {
    addLog(String("DMX preset ") + String(preset) + ": crossfade limited by point cap (" + String(MAX_DMX_PRESET_POINTS) + ").");
  }

  addLog(String("DMX preset ") + String(preset) + " fade " + String(dmxFadeState.durationMs) + "ms (" + String(pointCount) + " points)");
  tickDmxPresetFade();
  return true;
}

static void tickDmxPresetFade() {
  if (!dmxFadeState.active) return;

  uint32_t now = millis();
  if (dmxFadeState.lastFrameMs > 0 && (now - dmxFadeState.lastFrameMs) < 33 && dmxFadeState.durationMs > 0) {
    return;
  }

  float progress = 1.0f;
  if (dmxFadeState.durationMs > 0) {
    uint32_t elapsed = now - dmxFadeState.startMs;
    progress = static_cast<float>(elapsed) / static_cast<float>(dmxFadeState.durationMs);
    if (progress > 1.0f) progress = 1.0f;
    if (progress < 0.0f) progress = 0.0f;
  }
  bool done = (progress >= 1.0f) || (dmxFadeState.durationMs == 0);
  uint8_t streamDevice = (dmxFadeState.dest == DMX_DEST_UNICAST) ? dmxFadeState.device : 255;

  for (int i = 0; i < dmxFadeState.pointCount; i++) {
    const DmxFadePoint& p = dmxFadeState.points[i];
    if (p.channel < 1 || p.channel > 512) continue;
    DmxStreamState* stream = getDmxStream(dmxFadeState.protocol, streamDevice, p.universe);
    if (stream == nullptr) continue;
    int value = p.toValue;
    if (!done) {
      float delta = static_cast<float>(static_cast<int>(p.toValue) - static_cast<int>(p.fromValue));
      value = static_cast<int>(lroundf(static_cast<float>(p.fromValue) + (delta * progress)));
    }
    if (value < 0) value = 0;
    if (value > 255) value = 255;
    stream->data[p.channel - 1] = static_cast<uint8_t>(value);
  }

  for (int i = 0; i < dmxFadeState.usedUniverseCount; i++) {
    uint16_t universe = dmxFadeState.usedUniverses[i];
    DmxStreamState* stream = getDmxStream(dmxFadeState.protocol, streamDevice, universe);
    if (stream == nullptr) continue;
    sendDmxStreamFrame(dmxFadeState.protocol, dmxFadeState.dest, dmxFadeState.device, universe, stream);
  }

  dmxFadeState.lastFrameMs = now;
  if (done) {
    dmxFadeState.active = false;
  }
}

// Keep-alive: sACN receivers drop output after ~2.5s without data and many
// Art-Net nodes behave the same, so a single frame per trigger event is not
// enough — re-emit every active universe once a second. An active fade
// already streams at ~30fps, so the keep-alive stays quiet during fades.
static uint32_t dmxKeepAliveLastMs = 0;
static void tickDmxKeepAlive() {
  uint32_t now = millis();
  if (dmxFadeState.active) { dmxKeepAliveLastMs = now; return; }
  if (now - dmxKeepAliveLastMs < 1000) return;
  dmxKeepAliveLastMs = now;
  for (int i = 0; i < MAX_DMX_STREAMS; i++) {
    DmxStreamState& st = dmxStreams[i];
    if (!st.used) continue;
    if (st.protocol == DMX_PROTO_ARTNET && !config.artnetEnabled) continue;
    if (st.protocol == DMX_PROTO_SACN && !config.sacnEnabled) continue;
    uint8_t dest = (st.device == 255) ? DMX_DEST_AUTO : DMX_DEST_UNICAST;
    sendDmxStreamFrame(st.protocol, dest, st.device, st.universe, &st);
  }
}

// Blackout: zero every active output universe, send one final all-zero frame,
// then stop streaming them (keep-alive skips unused slots).
static void releaseDmxOutputs() {
  dmxFadeState.active = false;
  for (int i = 0; i < MAX_DMX_STREAMS; i++) {
    DmxStreamState& st = dmxStreams[i];
    if (!st.used) continue;
    memset(st.data, 0, sizeof(st.data));
    uint8_t dest = (st.device == 255) ? DMX_DEST_AUTO : DMX_DEST_UNICAST;
    sendDmxStreamFrame(st.protocol, dest, st.device, st.universe, &st);
    st.used = false;
  }
  addLog("DMX output released (all universes zeroed).");
}

// Fade support for plain DMX outputs, sharing the preset fade engine. If a
// fade with an incompatible protocol/destination is already running the
// caller falls back to snapping the value, so a running preset look is never
// corrupted. New channels merge into a compatible in-flight fade; its points
// are re-anchored at their current level first so nothing jumps.
static bool startDmxOutputFade(const SensorConfig::OutputConfig& out, uint8_t proto, uint16_t universe, uint8_t value) {
  uint32_t fadeMs = sanitizeDmxFadeMs(out.dmxFadeMs);
  if (fadeMs == 0) return false;
  uint8_t dmxDest = sanitizeDmxDest(out.dmxDest);
  uint8_t streamDevice = (dmxDest == DMX_DEST_UNICAST) ? out.device : 255;
  if (dmxFadeState.active) {
    uint8_t fadeDevice = (dmxFadeState.dest == DMX_DEST_UNICAST) ? dmxFadeState.device : 255;
    if (dmxFadeState.protocol != proto || fadeDevice != streamDevice) return false;
  }
  uint16_t channels[512];
  int channelCount = parseDmxChannels(out.dmxChannels, channels, 512);
  if (channelCount <= 0) return false;
  DmxStreamState* stream = getDmxStream(proto, streamDevice, universe);
  if (stream == nullptr) return false;

  uint32_t now = millis();
  uint32_t durationMs = fadeMs;
  if (dmxFadeState.active) {
    float progress = 1.0f;
    uint32_t elapsed = now - dmxFadeState.startMs;
    if (dmxFadeState.durationMs > 0) {
      progress = static_cast<float>(elapsed) / static_cast<float>(dmxFadeState.durationMs);
      if (progress > 1.0f) progress = 1.0f;
      if (progress < 0.0f) progress = 0.0f;
    }
    for (int i = 0; i < dmxFadeState.pointCount; i++) {
      DmxFadePoint& p = dmxFadeState.points[i];
      float delta = static_cast<float>(static_cast<int>(p.toValue) - static_cast<int>(p.fromValue));
      int cur = static_cast<int>(lroundf(static_cast<float>(p.fromValue) + delta * progress));
      if (cur < 0) cur = 0;
      if (cur > 255) cur = 255;
      p.fromValue = static_cast<uint8_t>(cur);
    }
    uint32_t remaining = (dmxFadeState.durationMs > elapsed) ? (dmxFadeState.durationMs - elapsed) : 0;
    if (remaining > durationMs) durationMs = remaining;
  } else {
    dmxFadeState.pointCount = 0;
    dmxFadeState.usedUniverseCount = 0;
    dmxFadeState.protocol = proto;
    dmxFadeState.dest = dmxDest;
    dmxFadeState.device = out.device;
  }

  for (int i = 0; i < channelCount; i++) {
    uint16_t ch = channels[i];
    if (ch < 1 || ch > 512) continue;
    bool found = false;
    for (int pIdx = 0; pIdx < dmxFadeState.pointCount; pIdx++) {
      DmxFadePoint& p = dmxFadeState.points[pIdx];
      if (p.universe == universe && p.channel == ch) { p.toValue = value; found = true; break; }
    }
    if (found) continue;
    if (dmxFadeState.pointCount >= MAX_DMX_PRESET_POINTS) {
      stream->data[ch - 1] = value;  // cap hit: snap the overflow channels
      continue;
    }
    DmxFadePoint& p = dmxFadeState.points[dmxFadeState.pointCount++];
    p.universe = universe;
    p.channel = ch;
    p.fromValue = stream->data[ch - 1];
    p.toValue = value;
  }
  bool haveUniverse = false;
  for (int i = 0; i < dmxFadeState.usedUniverseCount; i++) {
    if (dmxFadeState.usedUniverses[i] == universe) { haveUniverse = true; break; }
  }
  if (!haveUniverse && dmxFadeState.usedUniverseCount < MAX_DMX_PRESET_UNIVERSE) {
    dmxFadeState.usedUniverses[dmxFadeState.usedUniverseCount++] = universe;
  }
  dmxFadeState.startMs = now;
  dmxFadeState.durationMs = durationMs;
  dmxFadeState.lastFrameMs = 0;
  dmxFadeState.active = true;
  return true;
}

static bool sendDmxOutput(const SensorConfig::OutputConfig& out, float norm) {
  uint8_t proto = sanitizeDmxProtocol(out.dmxProtocol);
  if (proto == DMX_PROTO_ARTNET && !config.artnetEnabled) return false;
  if (proto == DMX_PROTO_SACN && !config.sacnEnabled) return false;

  uint16_t universe = (proto == DMX_PROTO_ARTNET) ? getArtNetUniverseOneBased(out) : (out.dmxUniverse > 0 ? out.dmxUniverse : 1);
  IPAddress ip;
  if (!resolveDmxDestination(out, proto, universe, ip)) return false;

  uint16_t channels[512];
  int channelCount = parseDmxChannels(out.dmxChannels, channels, 512);
  if (channelCount <= 0) return false;

  uint8_t value = toDmxValue(out, norm);
  if (startDmxOutputFade(out, proto, universe, value)) {
    addLog(String(proto == DMX_PROTO_ARTNET ? "Art-Net" : "sACN") + " u" + String(universe) +
           " ch " + out.dmxChannels + " fade to " + String(value) + " over " + String(sanitizeDmxFadeMs(out.dmxFadeMs)) + "ms");
    tickDmxPresetFade();
    return true;
  }
  uint8_t streamDevice = (sanitizeDmxDest(out.dmxDest) == DMX_DEST_UNICAST) ? out.device : 255;
  DmxStreamState* stream = getDmxStream(proto, streamDevice, universe);
  uint8_t tempData[512];
  uint8_t* frame = tempData;
  uint8_t sequence = 0;
  if (stream != nullptr) {
    frame = stream->data;
    sequence = ++stream->sequence;
    if (sequence == 0) sequence = ++stream->sequence;
  } else {
    memset(tempData, 0, sizeof(tempData));
  }
  for (int i = 0; i < channelCount; i++) {
    uint16_t ch = channels[i];
    if (ch >= 1 && ch <= 512) frame[ch - 1] = value;
  }

  if (proto == DMX_PROTO_ARTNET) {
    sendArtNetDmx(ip, universe, frame, 512, sequence);
    uint8_t net = 0, subnet = 0, portUni = 0;
    decodeArtNetPortAddressOneBased(universe, net, subnet, portUni);
    addLog(String("Art-Net N") + String(net) + " S" + String(subnet) + " U" + String(portUni) +
           " ch " + out.dmxChannels + " = " + String(value) + " -> " + ip.toString());
  } else {
    sendSacnDmx(ip, universe, frame, sequence);
    addLog(String("sACN u") + String(universe) + " ch " + out.dmxChannels + " = " + String(value) + " -> " + ip.toString());
  }
  return true;
}

static String buildHttpValue(const SensorConfig::OutputConfig& out, float norm) {
  if (oscInputTokenOverrideActive) {
    return oscInputTokenValue;
  }
  if (out.outMode == OUT_STRING) {
    return (norm >= 0.5f) ? out.onString : out.offString;
  }
  float outputValue = mapOutput(out, norm);
  outputValue = snapOutput(out, outputValue);
  if (out.outMode == OUT_FLOAT) {
    return String(outputValue, 3);
  }
  int intValue = lroundf(outputValue);
  return String(intValue);
}

static String replaceTokens(const String& input, const String& value, const String& ip, uint16_t port) {
  String out = input;
  if (out.indexOf("{value}") >= 0) out.replace("{value}", value);
  if (out.indexOf("{IP}") >= 0) out.replace("{IP}", ip);
  if (out.indexOf("{ip}") >= 0) out.replace("{ip}", ip);
  if (out.indexOf("{Port}") >= 0) out.replace("{Port}", String(port));
  if (out.indexOf("{port}") >= 0) out.replace("{port}", String(port));
  return out;
}

static String replaceValueTokens(const String& input, const String& value, float norm, bool state) {
  String out = input;
  String effectiveValue = value;
  float effectiveNorm = norm;
  bool effectiveState = state;
  if (oscInputTokenOverrideActive) {
    effectiveValue = oscInputTokenValue;
    effectiveNorm = oscInputTokenNorm;
    effectiveState = oscInputTokenState;
  }
  String normText = String(effectiveNorm, 3);
  String stateText = effectiveState ? "1" : "0";
  if (out.indexOf("{value}") >= 0) out.replace("{value}", effectiveValue);
  if (out.indexOf("{VALUE}") >= 0) out.replace("{VALUE}", effectiveValue);
  if (out.indexOf("{norm}") >= 0) out.replace("{norm}", normText);
  if (out.indexOf("{NORM}") >= 0) out.replace("{NORM}", normText);
  if (out.indexOf("{state}") >= 0) out.replace("{state}", stateText);
  if (out.indexOf("{STATE}") >= 0) out.replace("{STATE}", stateText);
  return out;
}

static void configureMqttClient() {
  if (config.mqttDevice < MAX_OSC_DEVICES) {
    const OscDevice& dev = config.oscDevices[config.mqttDevice];
    uint16_t port = dev.port > 0 ? dev.port : DEFAULT_MQTT_PORT;
    if (isValidIp(dev.ip)) {
      mqttClient.setServer(dev.ip, port);
      return;
    }
  }
  uint16_t port = config.mqttPort > 0 ? config.mqttPort : DEFAULT_MQTT_PORT;
  mqttClient.setServer(config.mqttHost.c_str(), port);
}

static bool ensureMqttConnected() {
  if (!config.mqttEnabled) return false;
  String brokerLabel = "";
  uint16_t brokerPort = DEFAULT_MQTT_PORT;
  bool brokerResolved = false;
  if (config.mqttDevice < MAX_OSC_DEVICES) {
    const OscDevice& dev = config.oscDevices[config.mqttDevice];
    if (isValidIp(dev.ip)) {
      brokerPort = dev.port > 0 ? dev.port : DEFAULT_MQTT_PORT;
      brokerLabel = ipToString(dev.ip);
      mqttClient.setServer(dev.ip, brokerPort);
      brokerResolved = true;
    }
  }
  if (!brokerResolved) {
    String host = config.mqttHost;
    host.trim();
    if (host.length() == 0) return false;
    brokerPort = config.mqttPort > 0 ? config.mqttPort : DEFAULT_MQTT_PORT;
    brokerLabel = host;
    mqttClient.setServer(config.mqttHost.c_str(), brokerPort);
  }
  if (mqttClient.connected()) return true;
  unsigned long now = millis();
  if (now - mqttLastConnectAttemptMs < 3000) return false;
  mqttLastConnectAttemptMs = now;
  String clientId = config.mqttClientId;
  clientId.trim();
  if (clientId.length() == 0) clientId = defaultMqttClientId();
  bool ok = false;
  if (config.mqttUser.length() > 0 || config.mqttPass.length() > 0) {
    ok = mqttClient.connect(clientId.c_str(), config.mqttUser.c_str(), config.mqttPass.c_str());
  } else {
    ok = mqttClient.connect(clientId.c_str());
  }
  if (ok) {
    addLog(String("MQTT connected to ") + brokerLabel + ":" + String(brokerPort));
  } else {
    addLog(String("MQTT connect failed (state ") + String(mqttClient.state()) + ")");
  }
  return ok;
}

static bool sendMqttMessage(const String& topic, const String& payload, bool retain) {
  if (topic.length() == 0) return false;
  if (!ensureMqttConnected()) return false;
  bool ok = mqttClient.publish(topic.c_str(), payload.c_str(), retain);
  addLog(String("MQTT ") + (ok ? "pub " : "pub failed ") + topic + " => " + payload);
  return ok;
}

static void sendMqttOutput(const SensorConfig::OutputConfig& out, float norm, const SensorConfig* sensor, int triggerIndex, int outputIndex) {
  String topic = out.mqttTopic;
  topic.trim();
  if (topic.length() == 0) {
    if (sensor != nullptr && triggerIndex >= 0 && outputIndex >= 0) {
      topic = buildDefaultMqttTopic(*sensor, triggerIndex, outputIndex);
    } else {
      topic = config.mqttBaseTopic;
      topic.trim();
      if (topic.length() == 0) topic = DEFAULT_MQTT_BASE_TOPIC;
      if (!topic.endsWith("/")) topic += "/";
      topic += "output";
    }
  }
  String value = buildHttpValue(out, norm);
  bool state = (norm >= 0.5f);
  String payload = out.mqttPayload;
  if (payload.length() == 0) {
    payload = value;
  } else {
    payload = replaceValueTokens(payload, value, norm, state);
  }
  sendMqttMessage(topic, payload, out.mqttRetain);
}

static void sendHttpRequest(const SensorConfig::OutputConfig& out, float norm) {
  if (out.httpUrl.length() == 0) return;
  String value = buildHttpValue(out, norm);
  String ip = out.httpIp;
  uint16_t port = out.httpPort;
  if (out.httpDevice < MAX_OSC_DEVICES) {
    IPAddress dip;
    uint16_t dport;
    if (getOscTarget(out.httpDevice, dip, dport)) {
      ip = ipToString(dip);
      port = dport;
    }
  }
  if (port == 0) port = 80;
  String url = replaceTokens(out.httpUrl, value, ip, port);
  String body = replaceTokens(out.httpBody, value, ip, port);

  HTTPClient http;
  WiFiClient client;
  if (!http.begin(client, url)) {
    addLog("HTTP begin failed");
    return;
  }
  int code = -1;
  if (out.httpMethod == HTTPM_POST) {
    http.addHeader("Content-Type", "text/plain");
    code = http.POST(body);
  } else {
    code = http.GET();
  }
  addLog(String("HTTP ") + (out.httpMethod == HTTPM_POST ? "POST " : "GET ") + url + " => " + String(code));
  http.end();
}

static void setGpioLevel(int pin, bool high) {
  if (!isAllowedGpioOutputPin(pin) || isReservedPin(pin)) return;
  if (gpioPwmChannelByPin[pin] >= 0) {
    ledcDetachPin(pin);
    gpioPwmChannelByPin[pin] = -1;
  }
  pinMode(pin, OUTPUT);
  digitalWrite(pin, high ? HIGH : LOW);
}

static void scheduleGpioPulse(int pin, uint32_t ms, bool invert) {
  if (!isAllowedGpioOutputPin(pin) || isReservedPin(pin)) return;
  if (ms == 0) ms = 100;
  setGpioLevel(pin, invert ? LOW : HIGH);
  gpioPulseEndLevelByPin[pin] = invert ? 1 : 0;
  gpioPulseUntilByPin[pin] = millis() + ms;
}

static void setGpioPwmDuty(int pin, int duty) {
  int ch = ensureGpioPwmChannel(pin);
  if (ch < 0) return;
  if (duty < 0) duty = 0;
  if (duty > GPIO_PWM_MAX) duty = GPIO_PWM_MAX;
  ledcWrite(ch, duty);
}

static void applyGpioOutput(const SensorConfig::OutputConfig& out, float norm, bool pressOnly) {
  if (!isAllowedGpioOutputPin(out.gpioPin) || isReservedPin(out.gpioPin)) return;
  if (out.gpioMode == GPIO_OUT_PWM) {
    if (pressOnly) {
      float outputValue = mapOutput(out, 1.0f);
      outputValue = snapOutput(out, outputValue);
      int duty = lroundf(outputValue);
      if (duty < 0) duty = 0;
      if (duty > GPIO_PWM_MAX) duty = GPIO_PWM_MAX;
      if (out.gpioInvert) duty = GPIO_PWM_MAX - duty;
      setGpioPwmDuty(out.gpioPin, duty);
      return;
    }
    float outputValue = mapOutput(out, norm);
    outputValue = snapOutput(out, outputValue);
    int duty = lroundf(outputValue);
    if (duty < 0) duty = 0;
    if (duty > GPIO_PWM_MAX) duty = GPIO_PWM_MAX;
    if (out.gpioInvert) duty = GPIO_PWM_MAX - duty;
    setGpioPwmDuty(out.gpioPin, duty);
    return;
  }
  if (out.gpioMode == GPIO_OUT_PULSE) {
    if (norm >= 0.5f) {
      scheduleGpioPulse(out.gpioPin, out.gpioPulseMs, out.gpioInvert);
    }
    return;
  }
  bool high = (norm >= 0.5f);
  if (out.gpioInvert) high = !high;
  setGpioLevel(out.gpioPin, high);
}

static void applyGpioOutputsNow() {
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    const SensorConfig& s = config.sensors[i];
    if (!s.enabled) continue;
    if (s.source != SRC_SENSORS) continue;
    if (hasPinConflict(i)) continue;
    float norm = 0.0f;
    if (s.type == SENSOR_ANALOG) {
      norm = readAnalogNormalized(i, s);
    } else if (s.type == SENSOR_ENCODER) {
      bool pressed = (digitalRead(s.encSwPin) == LOW);
      norm = pressed ? 1.0f : 0.0f;
    } else {
      norm = readDigitalActive(s) ? 1.0f : 0.0f;
    }
    for (int o = 0; o < s.outputCount; o++) {
      const SensorConfig::OutputConfig& out = s.outputs[o];
      if (out.target != OUT_TARGET_GPIO) continue;
      if (out.gpioMode == GPIO_OUT_PULSE) continue;
      applyGpioOutput(out, norm, false);
    }
  }
}

static String ipToString(const IPAddress& ip) {
  return String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]);
}

static bool parseIpString(const String& s, IPAddress& out) {
  int parts[4] = {0, 0, 0, 0};
  int partIndex = 0;
  int value = 0;
  bool hasDigit = false;

  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c >= '0' && c <= '9') {
      value = value * 10 + (c - '0');
      if (value > 255) return false;
      hasDigit = true;
    } else if (c == '.') {
      if (!hasDigit || partIndex >= 3) return false;
      parts[partIndex++] = value;
      value = 0;
      hasDigit = false;
    } else {
      return false;
    }
  }

  if (!hasDigit || partIndex != 3) return false;
  parts[partIndex] = value;
  out = IPAddress(parts[0], parts[1], parts[2], parts[3]);
  return true;
}

static void addLog(const String& msg) {
  String line = String(millis()) + " " + msg;
  logLines[logHead] = line;
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) logCount++;
}

static String buildLogText() {
  String out;
  out.reserve(logCount * 64);
  int start = (logHead - logCount + LOG_LINES) % LOG_LINES;
  for (int i = 0; i < logCount; i++) {
    int idx = (start + i) % LOG_LINES;
    out += logLines[idx];
    out += "\n";
  }
  return out;
}

static bool parseOrderList(const String& s, uint8_t* order, uint8_t& count) {
  count = 0;
  int start = 0;
  while (start < (int)s.length() && count < MAX_TRIGGERS) {
    int comma = s.indexOf(',', start);
    if (comma < 0) comma = s.length();
    String part = s.substring(start, comma);
    part.trim();
    if (part.length() > 0) {
      int val = part.toInt();
      if (val >= 0 && val < MAX_TRIGGERS) {
        bool dup = false;
        for (int i = 0; i < count; i++) {
          if (order[i] == val) {
            dup = true;
            break;
          }
        }
        if (!dup) {
          order[count++] = static_cast<uint8_t>(val);
        }
      }
    }
    start = comma + 1;
  }
  return count > 0;
}

static String buildOrderList(const uint8_t* order, uint8_t count) {
  String out;
  for (int i = 0; i < count; i++) {
    if (i > 0) out += ",";
    out += String(order[i]);
  }
  return out;
}

static int findUnusedTrigger() {
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    bool inOrder = false;
    for (int j = 0; j < config.triggerCount; j++) {
      if (config.triggerOrder[j] == static_cast<uint8_t>(i)) { inOrder = true; break; }
    }
    if (!inOrder) return i;
  }
  return -1;
}

static int findUnusedOscDevice() {
  for (int i = 0; i < MAX_OSC_DEVICES; i++) {
    bool used = false;
    for (int j = 0; j < config.oscDeviceCount; j++) {
      if (config.oscDeviceOrder[j] == i) {
        used = true;
        break;
      }
    }
    if (!used) return i;
  }
  return -1;
}

static void sendOutputNow(const SensorConfig& sensor, const SensorConfig::OutputConfig& out, const String& addr, float norm) {
  if (out.target == OUT_TARGET_SET_VAR) {
    if (out.setVarName.length() > 0) {
      setGlobalVar(out.setVarName, out.setVarValue);
      addLog(String("Set var ") + out.setVarName + " = " + String(out.setVarValue));
    }
    return;
  }
  if (out.target == OUT_TARGET_REBOOT) {
    if (sensor.type == SENSOR_ANALOG || sensor.type == SENSOR_ENCODER) return;
    addLog("Reboot triggered by output action.");
    delay(100);
    ESP.restart();
    return;
  }
  if (out.target == OUT_TARGET_RESET_COOLDOWNS) {
    if (sensor.type == SENSOR_ANALOG || sensor.type == SENSOR_ENCODER) return;
    resetAllCooldowns();
    return;
  }
  if (out.target == OUT_TARGET_GPIO) {
    addLog(String("GPIO out ") + String(out.gpioPin) + (norm >= 0.5f ? " HIGH" : " LOW"));
    applyGpioOutput(out, norm, false);
    return;
  }
  if (out.target == OUT_TARGET_HTTP) {
    sendHttpRequest(out, norm);
    return;
  }
  if (out.target == OUT_TARGET_MQTT) {
    int triggerIndex = -1;
    for (int i = 0; i < MAX_TRIGGERS; i++) {
      if (&config.sensors[i] == &sensor) {
        triggerIndex = i;
        break;
      }
    }
    int outputIndex = static_cast<int>(&out - &sensor.outputs[0]);
    if (outputIndex < 0 || outputIndex >= MAX_OUTPUTS_PER_TRIGGER) outputIndex = -1;
    sendMqttOutput(out, norm, &sensor, triggerIndex, outputIndex);
    return;
  }
  if (out.target == OUT_TARGET_DMX) {
    sendDmxOutput(out, norm);
    return;
  }
  if (out.target == OUT_TARGET_DMX_PRESET) {
    triggerDmxPreset(out);
    return;
  }
  IPAddress ip;
  uint16_t port;
  if (!getOscTarget(out.device, ip, port)) return;
  if (out.target == OUT_TARGET_UDP) {
    sendUdpRawTo(ip, port, out.udpPayload);
    return;
  }
  if (addr.length() == 0) return;
  if (out.outMode == OUT_STRING) {
    const char* value = (norm >= 0.5f) ? out.onString.c_str() : out.offString.c_str();
    if (value[0] == '\0') return;
    sendOscStringTo(ip, port, addr.c_str(), value);
  } else if (out.outMode == OUT_FLOAT) {
    float outputValue = mapOutput(out, norm);
    outputValue = snapOutput(out, outputValue);
    sendOscFloatTo(ip, port, addr.c_str(), outputValue);
  } else {
    float outputValue = mapOutput(out, norm);
    outputValue = snapOutput(out, outputValue);
    int intValue = lroundf(outputValue);
    sendOscIntTo(ip, port, addr.c_str(), intValue);
  }
}

static void queueOrSendOutput(const SensorConfig& sensor, const SensorConfig::OutputConfig& out, const String& addr, float norm) {
  if (out.delayMs == 0) {
    sendOutputNow(sensor, out, addr, norm);
    return;
  }
  // Find sensor and output indices
  int sIdx = -1, oIdx = -1;
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    if (&config.sensors[i] == &sensor) { sIdx = i; break; }
  }
  oIdx = static_cast<int>(&out - &sensor.outputs[0]);
  if (sIdx < 0 || oIdx < 0 || oIdx >= MAX_OUTPUTS_PER_TRIGGER) {
    sendOutputNow(sensor, out, addr, norm);
    return;
  }
  for (int i = 0; i < MAX_PENDING_OUTPUTS; i++) {
    if (!pendingOutputs[i].active) {
      pendingOutputs[i].active = true;
      pendingOutputs[i].fireAtMs = millis() + out.delayMs;
      pendingOutputs[i].sensorIndex = static_cast<uint8_t>(sIdx);
      pendingOutputs[i].outputIndex = static_cast<uint8_t>(oIdx);
      pendingOutputs[i].norm = norm;
      addLog(String("Delayed ") + String(out.delayMs) + "ms: " + addr);
      return;
    }
  }
  // Queue full, fire immediately
  sendOutputNow(sensor, out, addr, norm);
}

static void tickPendingOutputs() {
  uint32_t now = millis();
  for (int i = 0; i < MAX_PENDING_OUTPUTS; i++) {
    if (pendingOutputs[i].active && static_cast<int32_t>(now - pendingOutputs[i].fireAtMs) >= 0) {
      pendingOutputs[i].active = false;
      const SensorConfig& s = config.sensors[pendingOutputs[i].sensorIndex];
      const SensorConfig::OutputConfig& out = s.outputs[pendingOutputs[i].outputIndex];
      sendOutputNow(s, out, out.oscAddress, pendingOutputs[i].norm);
    }
  }
}

static void configureUdpListener() {
  uint16_t desiredOscPort = 0;
  if (config.oscInEnabled) {
    desiredOscPort = (config.oscInPort > 0) ? config.oscInPort : DEFAULT_OSC_IN_PORT;
  }
  if (oscListenPort != desiredOscPort) {
    oscUdp.stop();
    if (desiredOscPort > 0) {
      if (oscUdp.begin(desiredOscPort)) {
        oscListenPort = desiredOscPort;
        addLog(String("OSC UDP listener on port ") + String(desiredOscPort));
      } else {
        oscListenPort = 0;
        addLog(String("OSC UDP listener failed on port ") + String(desiredOscPort));
      }
    } else {
      oscListenPort = 0;
    }
  }

  bool wantArtNetIn = config.dmxInArtNetEnabled;
  if (artnetListenActive != wantArtNetIn) {
    artnetInUdp.stop();
    artnetListenActive = false;
    if (wantArtNetIn) {
      if (artnetInUdp.begin(ARTNET_PORT)) {
        artnetListenActive = true;
        addLog(String("Art-Net input listener on port ") + String(ARTNET_PORT));
      } else {
        addLog(String("Art-Net input listener failed on port ") + String(ARTNET_PORT));
      }
    }
  }

  bool wantSacnIn = config.dmxInSacnEnabled;
  if (sacnListenActive != wantSacnIn) {
    sacnInUdp.stop();
    sacnListenActive = false;
    if (wantSacnIn) {
      if (sacnInUdp.begin(SACN_PORT)) {
        sacnListenActive = true;
        addLog(String("sACN input listener on port ") + String(SACN_PORT));
      } else {
        addLog(String("sACN input listener failed on port ") + String(SACN_PORT));
      }
    }
  }
}

static bool decodeArtNetDmxPacket(const uint8_t* data, size_t len, uint16_t& universeOneBased, uint8_t* dmxData, uint16_t& dmxLen) {
  if (data == nullptr || dmxData == nullptr || len < 18) return false;
  if (memcmp(data, "Art-Net", 7) != 0 || data[7] != 0x00) return false;
  if (data[8] != 0x00 || data[9] != 0x50) return false; // OpDmx (little-endian)

  uint16_t fieldUniverse = static_cast<uint16_t>(data[14]) | (static_cast<uint16_t>(data[15]) << 8);
  uint16_t dlen = (static_cast<uint16_t>(data[16]) << 8) | static_cast<uint16_t>(data[17]);
  if (dlen == 0) return false;
  if (dlen > 512) dlen = 512;
  if (18 + dlen > len) dlen = static_cast<uint16_t>(len - 18);
  if (dlen == 0) return false;

  memset(dmxData, 0, 512);
  memcpy(dmxData, data + 18, dlen);
  universeOneBased = static_cast<uint16_t>(fieldUniverse + 1);
  dmxLen = dlen;
  return true;
}

static bool decodeSacnDmxPacket(const uint8_t* data, size_t len, uint16_t& universeOneBased, uint8_t* dmxData, uint16_t& dmxLen) {
  static const uint8_t kAcnPreamble[12] = {0x41, 0x53, 0x43, 0x2D, 0x45, 0x31, 0x2E, 0x31, 0x37, 0x00, 0x00, 0x00};
  if (data == nullptr || dmxData == nullptr || len < 126) return false;
  if (data[0] != 0x00 || data[1] != 0x10) return false;
  if (memcmp(data + 4, kAcnPreamble, sizeof(kAcnPreamble)) != 0) return false;
  if (data[117] != 0x02) return false; // DMP set property message

  uint16_t propValCount = (static_cast<uint16_t>(data[123]) << 8) | static_cast<uint16_t>(data[124]);
  if (propValCount < 1) return false;
  size_t payloadBytes = static_cast<size_t>(propValCount);
  if (125 + payloadBytes > len) {
    payloadBytes = (len > 125) ? (len - 125) : 0;
  }
  if (payloadBytes < 1) return false;

  size_t slotCount = payloadBytes - 1; // first byte is start code
  if (slotCount > 512) slotCount = 512;
  if (slotCount == 0) return false;

  memset(dmxData, 0, 512);
  memcpy(dmxData, data + 126, slotCount);
  universeOneBased = (static_cast<uint16_t>(data[113]) << 8) | static_cast<uint16_t>(data[114]);
  dmxLen = static_cast<uint16_t>(slotCount);
  return universeOneBased > 0;
}

static uint8_t* getDmxInputUniverseBuffer(int index) {
  if (dmxInData == nullptr) return nullptr;
  if (index < 0 || index >= MAX_DMX_PRESET_UNIVERSE) return nullptr;
  return dmxInData + (index * 512);
}

static void updateDmxInputUniverse(uint8_t protocol, uint16_t universeOneBased, const uint8_t* dmxData, uint16_t dmxLen) {
  if (dmxData == nullptr || dmxLen == 0) return;
  if (universeOneBased < 1 || universeOneBased > MAX_DMX_PRESET_UNIVERSE) return;
  int idx = static_cast<int>(universeOneBased - 1);
  uint8_t* uni = getDmxInputUniverseBuffer(idx);
  if (uni == nullptr) return;
  uint32_t now = millis();
  memcpy(uni, dmxData, 512);
  dmxInSource[idx] = protocol;
  dmxInSeenMs[idx] = now;
  if (protocol == DMX_PROTO_ARTNET) dmxInArtSeenMs[idx] = now;
  if (protocol == DMX_PROTO_SACN) dmxInSacnSeenMs[idx] = now;
}

static bool getLiveDmxUniverseValues(uint16_t universeOneBased, uint8_t viewMode, uint8_t outData[512], uint8_t& selectedSource, uint32_t& selectedAgeMs) {
  if (outData == nullptr) return false;
  if (universeOneBased < 1 || universeOneBased > MAX_DMX_PRESET_UNIVERSE) return false;
  int idx = static_cast<int>(universeOneBased - 1);

  uint8_t mode = sanitizeDmxInputSourceMode(viewMode);
  if (mode == DMX_IN_SOURCE_RECENT) mode = sanitizeDmxInputSourceMode(config.dmxInSourceMode);
  bool hasLive = dmxInSeenMs[idx] > 0;
  if (!hasLive) return false;
  uint8_t* uni = getDmxInputUniverseBuffer(idx);
  if (uni == nullptr) return false;

  uint8_t src = dmxInSource[idx];
  if (mode == DMX_IN_SOURCE_PREFER_ARTNET && src != DMX_PROTO_ARTNET) return false;
  if (mode == DMX_IN_SOURCE_PREFER_SACN && src != DMX_PROTO_SACN) return false;

  memcpy(outData, uni, 512);
  selectedSource = src;
  uint32_t now = millis();
  selectedAgeMs = now - dmxInSeenMs[idx];
  return true;
}

static bool matchOscAddressPattern(const String& pattern, const String& value) {
  String p = normalizeOscAddress(pattern);
  String v = normalizeOscAddress(value);

  int pi = 0;
  int vi = 0;
  int star = -1;
  int match = 0;
  int plen = p.length();
  int vlen = v.length();

  while (vi < vlen) {
    if (pi < plen && p[pi] == v[vi]) {
      pi++;
      vi++;
      continue;
    }
    if (pi < plen && p[pi] == '*') {
      star = pi++;
      match = vi;
      continue;
    }
    if (star != -1) {
      pi = star + 1;
      vi = ++match;
      continue;
    }
    return false;
  }
  while (pi < plen && p[pi] == '*') pi++;
  return pi == plen;
}

static bool parseOscPaddedString(const uint8_t* data, size_t len, size_t& pos, String& out) {
  if (pos >= len) return false;
  size_t start = pos;
  while (pos < len && data[pos] != 0) pos++;
  if (pos >= len) return false;

  out = "";
  out.reserve(pos - start);
  for (size_t i = start; i < pos; i++) {
    out += static_cast<char>(data[i]);
  }

  pos++;  // skip null terminator
  while ((pos & 0x03u) != 0u) {
    if (pos >= len) return false;
    pos++;
  }
  return true;
}

static bool parseOscInPacket(const uint8_t* data, size_t len, OscInMessage& msg) {
  if (data == nullptr || len < 8) return false;
  if (len >= 7 && memcmp(data, "#bundle", 7) == 0) return false;

  size_t pos = 0;
  String address;
  if (!parseOscPaddedString(data, len, pos, address)) return false;
  if (address.length() == 0 || address[0] != '/') return false;

  String types;
  if (!parseOscPaddedString(data, len, pos, types)) return false;
  if (types.length() == 0 || types[0] != ',') return false;

  msg.address = address;
  msg.hasArg = false;
  msg.argType = OSC_IN_ARG_ANY;
  msg.intValue = 0;
  msg.floatValue = 0.0f;
  msg.stringValue = "";

  if (types.length() < 2) {
    return true;
  }

  char t = types[1];
  msg.hasArg = true;

  if (t == 'i') {
    if (pos + 4 > len) return false;
    uint32_t raw = (static_cast<uint32_t>(data[pos]) << 24) |
                   (static_cast<uint32_t>(data[pos + 1]) << 16) |
                   (static_cast<uint32_t>(data[pos + 2]) << 8) |
                   static_cast<uint32_t>(data[pos + 3]);
    msg.argType = OSC_IN_ARG_INT;
    msg.intValue = static_cast<int32_t>(raw);
    msg.floatValue = static_cast<float>(msg.intValue);
    pos += 4;
    return true;
  }

  if (t == 'f') {
    if (pos + 4 > len) return false;
    uint32_t bits = (static_cast<uint32_t>(data[pos]) << 24) |
                    (static_cast<uint32_t>(data[pos + 1]) << 16) |
                    (static_cast<uint32_t>(data[pos + 2]) << 8) |
                    static_cast<uint32_t>(data[pos + 3]);
    float f = 0.0f;
    memcpy(&f, &bits, sizeof(float));
    msg.argType = OSC_IN_ARG_FLOAT;
    msg.floatValue = f;
    msg.intValue = static_cast<int32_t>(lroundf(f));
    pos += 4;
    return true;
  }

  if (t == 's') {
    String sval;
    if (!parseOscPaddedString(data, len, pos, sval)) return false;
    msg.argType = OSC_IN_ARG_STRING;
    msg.stringValue = sval;
    return true;
  }

  // Unsupported first argument type for now.
  return false;
}

static bool matchOscInTrigger(const SensorConfig& s, const OscInMessage& msg, float& normOut, String& valueOut) {
  if (!matchOscAddressPattern(s.oscInAddress, msg.address)) return false;

  normOut = 1.0f;
  valueOut = "1";

  if (!msg.hasArg) {
    if (s.oscInArgType != OSC_IN_ARG_ANY) return false;
    if (s.oscInMatchMode != OSC_IN_MATCH_ANY) return false;
    return true;
  }

  if (s.oscInArgType != OSC_IN_ARG_ANY && s.oscInArgType != msg.argType) return false;

  bool numeric = (msg.argType == OSC_IN_ARG_INT || msg.argType == OSC_IN_ARG_FLOAT);
  float rawNumeric = numeric ? msg.floatValue : 0.0f;

  if (msg.argType == OSC_IN_ARG_STRING) {
    valueOut = msg.stringValue;
    String low = msg.stringValue;
    low.toLowerCase();
    normOut = (low == "0" || low == "off" || low == "false") ? 0.0f : 1.0f;
  } else if (msg.argType == OSC_IN_ARG_INT) {
    valueOut = String(msg.intValue);
    if (rawNumeric >= 0.0f && rawNumeric <= 1.0f) {
      normOut = rawNumeric;
    }
  } else if (msg.argType == OSC_IN_ARG_FLOAT) {
    valueOut = String(rawNumeric, 3);
    if (rawNumeric >= 0.0f && rawNumeric <= 1.0f) {
      normOut = rawNumeric;
    }
  }

  if (s.oscInMatchMode == OSC_IN_MATCH_ANY) {
    return true;
  }

  if (s.oscInMatchMode == OSC_IN_MATCH_EQUAL) {
    if (msg.argType == OSC_IN_ARG_STRING) {
      return msg.stringValue == s.oscInString;
    }
    if (!numeric) return false;
    return fabsf(rawNumeric - s.oscInValue) <= 0.0005f;
  }

  if (s.oscInMatchMode == OSC_IN_MATCH_RANGE) {
    if (!numeric) return false;
    float minV = s.oscInMin;
    float maxV = s.oscInMax;
    bool inverted = false;
    if (maxV < minV) {
      float t = minV;
      minV = maxV;
      maxV = t;
      inverted = true;
    }
    if (rawNumeric < minV || rawNumeric > maxV) return false;
    float span = maxV - minV;
    if (span <= 0.000001f) {
      normOut = 1.0f;
    } else {
      normOut = (rawNumeric - minV) / span;
    }
    if (inverted) normOut = 1.0f - normOut;
    normOut = clampFloat(normOut, 0.0f, 1.0f);
    return true;
  }

  return false;
}

static void processOscInput() {
  if (!config.oscInEnabled || oscListenPort == 0) return;

  uint8_t packet[512];
  int packetSize = oscUdp.parsePacket();
  while (packetSize > 0) {
    int readLen = packetSize;
    if (readLen > static_cast<int>(sizeof(packet))) readLen = sizeof(packet);
    int len = oscUdp.read(packet, readLen);
    if (len > 0) {
      OscInMessage msg;
      if (parseOscInPacket(packet, static_cast<size_t>(len), msg)) {
        String argText = "";
        if (msg.hasArg) {
          if (msg.argType == OSC_IN_ARG_INT) argText = String(msg.intValue);
          else if (msg.argType == OSC_IN_ARG_FLOAT) argText = String(msg.floatValue, 3);
          else if (msg.argType == OSC_IN_ARG_STRING) argText = msg.stringValue;
        }
        addLog(String("OSC In ") + msg.address + (msg.hasArg ? (" " + argText) : ""));

        bool anyMatched = false;
        for (int i = 0; i < MAX_TRIGGERS; i++) {
          const SensorConfig& s = config.sensors[i];
          if (!s.enabled || s.source != SRC_OSC) continue;

          float norm = 1.0f;
          String value = "1";
          if (!matchOscInTrigger(s, msg, norm, value)) continue;
          anyMatched = true;

          if (!checkCondition(s)) continue;

          oscInputTokenOverrideActive = true;
          oscInputTokenValue = value;
          oscInputTokenNorm = clampFloat(norm, 0.0f, 1.0f);
          oscInputTokenState = (oscInputTokenNorm >= 0.5f);
          for (int o = 0; o < s.outputCount; o++) {
            const SensorConfig::OutputConfig& out = s.outputs[o];
            float sendNorm = out.sendMinOnRelease ? oscInputTokenNorm : 1.0f;
            queueOrSendOutput(s, out, out.oscAddress, sendNorm);
          }
          oscInputTokenOverrideActive = false;
          addLog(String("OSC In matched trigger: ") + s.name);
        }

        if (!anyMatched) {
          addLog(String("OSC In no match: ") + msg.address);
        }
      } else {
        addLog("OSC In parse skipped.");
      }
    }
    packetSize = oscUdp.parsePacket();
  }
}

static void processDmxInput() {
  uint8_t packet[640];
  uint8_t dmxData[512];

  if (artnetListenActive) {
    int packetSize = artnetInUdp.parsePacket();
    while (packetSize > 0) {
      int readLen = packetSize;
      if (readLen > static_cast<int>(sizeof(packet))) readLen = sizeof(packet);
      int len = artnetInUdp.read(packet, readLen);
      if (len > 0) {
        uint16_t universe = 0;
        uint16_t dmxLen = 0;
        if (decodeArtNetDmxPacket(packet, static_cast<size_t>(len), universe, dmxData, dmxLen)) {
          updateDmxInputUniverse(DMX_PROTO_ARTNET, universe, dmxData, dmxLen);
        }
      }
      packetSize = artnetInUdp.parsePacket();
    }
  }

  if (sacnListenActive) {
    int packetSize = sacnInUdp.parsePacket();
    while (packetSize > 0) {
      int readLen = packetSize;
      if (readLen > static_cast<int>(sizeof(packet))) readLen = sizeof(packet);
      int len = sacnInUdp.read(packet, readLen);
      if (len > 0) {
        uint16_t universe = 0;
        uint16_t dmxLen = 0;
        if (decodeSacnDmxPacket(packet, static_cast<size_t>(len), universe, dmxData, dmxLen)) {
          updateDmxInputUniverse(DMX_PROTO_SACN, universe, dmxData, dmxLen);
        }
      }
      packetSize = sacnInUdp.parsePacket();
    }
  }
}

// Current value of one received DMX channel, honoring the configured source
// priority. Returns -1 when there is no (matching) data for that universe.
static int getDmxInChannelValue(uint16_t universeOneBased, uint16_t channel) {
  if (channel < 1 || channel > 512) return -1;
  if (universeOneBased < 1 || universeOneBased > MAX_DMX_PRESET_UNIVERSE) return -1;
  int idx = static_cast<int>(universeOneBased - 1);
  if (dmxInSeenMs[idx] == 0) return -1;
  uint8_t* uni = getDmxInputUniverseBuffer(idx);
  if (uni == nullptr) return -1;
  uint8_t mode = sanitizeDmxInputSourceMode(config.dmxInSourceMode);
  uint8_t src = dmxInSource[idx];
  if (mode == DMX_IN_SOURCE_PREFER_ARTNET && src != DMX_PROTO_ARTNET) return -1;
  if (mode == DMX_IN_SOURCE_PREFER_SACN && src != DMX_PROTO_SACN) return -1;
  return uni[channel - 1];
}

// DMX-In trigger sources: fire outputs from received DMX channel values.
// Level/range modes act like a fader (outputs follow the channel), threshold
// mode acts like a button (fires on crossings, honoring Send-Off-on-release).
static void tickDmxInTriggers() {
  if (!config.dmxInArtNetEnabled && !config.dmxInSacnEnabled) return;
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    const SensorConfig& s = config.sensors[i];
    if (!s.enabled || s.source != SRC_DMX) continue;
    int v = getDmxInChannelValue(s.dmxInUniverse, s.dmxInChannel);
    if (v < 0) continue;

    if (s.dmxInMode == DMX_IN_TRIG_THRESHOLD) {
      int8_t above = (v >= s.dmxInThreshold) ? 1 : 0;
      if (dmxInGateState[i] < 0) { dmxInGateState[i] = above; continue; }  // arm without firing at boot
      if (above == dmxInGateState[i]) continue;
      dmxInGateState[i] = above;
      dmxInLastValue[i] = static_cast<int16_t>(v);
      if (!checkCondition(s)) continue;
      addLog(String("DMX In ") + s.name + " u" + String(s.dmxInUniverse) + " ch" + String(s.dmxInChannel) +
             (above ? " above " : " below ") + String(s.dmxInThreshold));
      for (int o = 0; o < s.outputCount; o++) {
        const SensorConfig::OutputConfig& out = s.outputs[o];
        if (above) {
          queueOrSendOutput(s, out, out.oscAddress, 1.0f);
          lastBool[i][o] = 1;
        } else if (out.sendMinOnRelease) {
          queueOrSendOutput(s, out, out.oscAddress, 0.0f);
          lastBool[i][o] = 0;
        }
      }
      continue;
    }

    // Level / range modes: act on value changes only
    if (dmxInLastValue[i] == static_cast<int16_t>(v)) continue;
    float norm;
    if (s.dmxInMode == DMX_IN_TRIG_RANGE) {
      int lo = s.dmxInRangeMin;
      int hi = s.dmxInRangeMax;
      bool inverted = false;
      if (hi < lo) { int t = lo; lo = hi; hi = t; inverted = true; }
      if (v < lo || v > hi) continue;
      norm = (hi == lo) ? 1.0f : static_cast<float>(v - lo) / static_cast<float>(hi - lo);
      if (inverted) norm = 1.0f - norm;
    } else {
      norm = static_cast<float>(v) / 255.0f;
    }
    dmxInLastValue[i] = static_cast<int16_t>(v);
    if (!checkCondition(s)) continue;
    for (int o = 0; o < s.outputCount; o++) {
      const SensorConfig::OutputConfig& out = s.outputs[o];
      queueOrSendOutput(s, out, out.oscAddress, norm);
      lastFloat[i][o] = norm;
      lastInt[i][o] = v;
    }
  }
}

static void processClickPattern(int index, const SensorConfig& sensor, bool pressed, int targetClicks, bool longPress) {
  unsigned long now = millis();

  if (pressed != multiPrevPressed[index]) {
    if (now - multiLastChangeMs[index] >= config.clickDebounceMs) {
      multiPrevPressed[index] = pressed;
      multiLastChangeMs[index] = now;
      if (pressed) {
        multiPressStartMs[index] = now;
        multiLongFired[index] = false;
      } else {
        if (!multiLongFired[index]) {
          if (!longPress && (now - multiPressStartMs[index] >= config.longPressMs)) {
            // Suppress single-click if this press exceeded long-press duration
            // when sharing a button with long-press triggers.
            multiLongFired[index] = false;
          } else {
          clickCount[index]++;
          multiLastReleaseMs[index] = now;
          }
        }
      }
    }
  }

  if (longPress && pressed && !multiLongFired[index] && (now - multiPressStartMs[index] >= config.longPressMs)) {
    if (checkCondition(sensor)) {
      addLog(String("Long press: ") + sensor.name);
      for (int o = 0; o < sensor.outputCount; o++) {
        const SensorConfig::OutputConfig& out = sensor.outputs[o];
        queueOrSendOutput(sensor, out, out.oscAddress, 1.0f);
      }
    }
    multiLongFired[index] = true;
    clickCount[index] = 0;
  }

  if (!pressed && clickCount[index] > 0 && (now - multiLastReleaseMs[index] >= config.multiClickGapMs)) {
    if (targetClicks > 0 && clickCount[index] == targetClicks) {
      if (checkCondition(sensor)) {
        if (targetClicks == 2) addLog(String("Double click: ") + sensor.name);
        else if (targetClicks == 3) addLog(String("Triple click: ") + sensor.name);
        else addLog(String("Click x") + String(targetClicks) + ": " + sensor.name);
        for (int o = 0; o < sensor.outputCount; o++) {
          const SensorConfig::OutputConfig& out = sensor.outputs[o];
          queueOrSendOutput(sensor, out, out.oscAddress, 1.0f);
        }
      }
    }
    clickCount[index] = 0;
  }
}

static void appendOscDeviceOptions(String& html, uint8_t selected) {
  for (int i = 0; i < config.oscDeviceCount; i++) {
    int idx = config.oscDeviceOrder[i];
    const OscDevice& d = config.oscDevices[idx];
    String label = htmlEscape(d.name) + " (" + ipToString(d.ip) + ":" + String(d.port) + ")";
    html += "<option value='" + String(idx) + "' " + String(idx == selected ? "selected" : "") + ">" + label + "</option>";
  }
}

static void handleSaveTriggers() {
  if (!ensureAuthenticated()) return;
  String action = server.hasArg("action") ? server.arg("action") : "save";

  for (int i = 0; i < MAX_TRIGGERS; i++) {
    String idx = String(i);
    if (!server.hasArg("s" + idx + "_present")) continue;
    SensorConfig previous = config.sensors[i];
    SensorConfig s = previous;
    s.enabled = server.hasArg("s" + idx + "_en");
    if (server.hasArg("s" + idx + "_name")) {
      s.name = server.arg("s" + idx + "_name");
      s.name.trim();
      if (s.name.length() == 0) s.name = String("Trigger ") + String(i + 1);
    }
    if (server.hasArg("s" + idx + "_src")) {
      int src = server.arg("s" + idx + "_src").toInt();
      if (src == 1) s.source = SRC_TIME;
      else if (src == 2) s.source = SRC_OSC;
      else if (src == 3) s.source = SRC_DMX;
      else s.source = SRC_SENSORS;
    }
    if (server.hasArg("s" + idx + "_type")) {
      s.type = sanitizeSensorType(server.arg("s" + idx + "_type").toInt());
    }
    if (server.hasArg("s" + idx + "_tt")) {
      int tt = server.arg("s" + idx + "_tt").toInt();
      if (tt >= 0 && tt <= TIME_INTERVAL) s.timeType = static_cast<TimeTriggerType>(tt);
    }
    if (s.timeType == TIME_ONCE) {
      if (server.hasArg("s" + idx + "_date") && server.hasArg("s" + idx + "_time")) {
        uint16_t y; uint8_t mo; uint8_t da;
        uint8_t h; uint8_t mi; uint8_t se = 0;
        if (parseDateYMD(server.arg("s" + idx + "_date"), y, mo, da) &&
            parseTimeHMS(server.arg("s" + idx + "_time"), h, mi, se)) {
          s.timeYear = y;
          s.timeMonth = mo;
          s.timeDay = da;
          s.timeHour = h;
          s.timeMinute = mi;
          s.timeSecond = se;
        }
      }
    } else if (s.timeType == TIME_DAILY) {
      if (server.hasArg("s" + idx + "_time_daily")) {
        uint8_t h; uint8_t mi; uint8_t se = 0;
        if (parseTimeHMS(server.arg("s" + idx + "_time_daily"), h, mi, se)) {
          s.timeHour = h;
          s.timeMinute = mi;
          s.timeSecond = se;
        }
      }
    } else if (s.timeType == TIME_WEEKLY) {
      if (server.hasArg("s" + idx + "_time_weekly")) {
        uint8_t h; uint8_t mi; uint8_t se = 0;
        if (parseTimeHMS(server.arg("s" + idx + "_time_weekly"), h, mi, se)) {
          s.timeHour = h;
          s.timeMinute = mi;
          s.timeSecond = se;
        }
      }
      uint8_t mask = 0;
      for (int d = 0; d < 7; d++) {
        if (server.hasArg("s" + idx + "_w" + String(d))) {
          mask |= (1 << d);
        }
      }
      s.weeklyMask = mask;
    }
    if (server.hasArg("s" + idx + "_interval_val")) {
      uint32_t val = static_cast<uint32_t>(server.arg("s" + idx + "_interval_val").toInt());
      if (val == 0) val = 1;
      String unit = server.arg("s" + idx + "_interval_unit");
      if (unit == "hours") {
        s.intervalSeconds = val * 3600;
      } else {
        s.intervalSeconds = val * 60;
      }
    }
    if (server.hasArg("s" + idx + "_osia")) {
      String addr = server.arg("s" + idx + "_osia");
      addr.trim();
      s.oscInAddress = normalizeOscAddress(addr);
    }
    if (server.hasArg("s" + idx + "_osit")) {
      int at = server.arg("s" + idx + "_osit").toInt();
      if (at < OSC_IN_ARG_ANY || at > OSC_IN_ARG_STRING) at = OSC_IN_ARG_ANY;
      s.oscInArgType = static_cast<uint8_t>(at);
    }
    if (server.hasArg("s" + idx + "_osim")) {
      int mm = server.arg("s" + idx + "_osim").toInt();
      if (mm < OSC_IN_MATCH_ANY || mm > OSC_IN_MATCH_RANGE) mm = OSC_IN_MATCH_ANY;
      s.oscInMatchMode = static_cast<uint8_t>(mm);
    }
    if (server.hasArg("s" + idx + "_osiv")) {
      s.oscInValue = server.arg("s" + idx + "_osiv").toFloat();
    }
    if (server.hasArg("s" + idx + "_osmn")) {
      s.oscInMin = server.arg("s" + idx + "_osmn").toFloat();
    }
    if (server.hasArg("s" + idx + "_osmx")) {
      s.oscInMax = server.arg("s" + idx + "_osmx").toFloat();
    }
    if (server.hasArg("s" + idx + "_osis")) {
      s.oscInString = server.arg("s" + idx + "_osis");
      s.oscInString.trim();
      if (s.oscInString.length() == 0) s.oscInString = "go";
    }
    if (s.oscInAddress.length() == 0) s.oscInAddress = "/trigger/*";
    int outCount = 0;
    for (int o = 0; o < MAX_OUTPUTS_PER_TRIGGER; o++) {
      String oidx = String(o);
      if (!server.hasArg("s" + idx + "_o" + oidx + "_present")) continue;
      if (outCount >= MAX_OUTPUTS_PER_TRIGGER) break;
      SensorConfig::OutputConfig out = s.outputs[outCount];
      if (server.hasArg("s" + idx + "_o" + oidx + "_tgt")) {
        out.target = sanitizeOutputTarget(server.arg("s" + idx + "_o" + oidx + "_tgt").toInt());
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_dly")) out.delayMs = static_cast<uint32_t>(server.arg("s" + idx + "_o" + oidx + "_dly").toInt());
      if (server.hasArg("s" + idx + "_o" + oidx + "_dev")) {
        int dev = server.arg("s" + idx + "_o" + oidx + "_dev").toInt();
        if (dev >= 0 && dev < MAX_OSC_DEVICES) out.device = static_cast<uint8_t>(dev);
      }
      if (s.type == SENSOR_ENCODER) {
        if (server.hasArg("s" + idx + "_o" + oidx + "_enc_addr")) {
          out.oscAddress = normalizeOscAddress(server.arg("s" + idx + "_o" + oidx + "_enc_addr"));
        }
        if (server.hasArg("s" + idx + "_o" + oidx + "_btn_addr")) {
          out.buttonAddress = normalizeOscAddress(server.arg("s" + idx + "_o" + oidx + "_btn_addr"));
        }
      } else {
        if (server.hasArg("s" + idx + "_o" + oidx + "_addr")) {
          out.oscAddress = normalizeOscAddress(server.arg("s" + idx + "_o" + oidx + "_addr"));
        }
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_omin")) {
        out.outMin = server.arg("s" + idx + "_o" + oidx + "_omin").toFloat();
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_omax")) {
        out.outMax = server.arg("s" + idx + "_o" + oidx + "_omax").toFloat();
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_omode")) {
        out.outMode = sanitizeOutputMode(server.arg("s" + idx + "_o" + oidx + "_omode").toInt());
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_bmode")) {
        out.buttonOutMode = sanitizeOutputMode(server.arg("s" + idx + "_o" + oidx + "_bmode").toInt());
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_bmin")) {
        out.buttonOutMin = server.arg("s" + idx + "_o" + oidx + "_bmin").toFloat();
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_bmax")) {
        out.buttonOutMax = server.arg("s" + idx + "_o" + oidx + "_bmax").toFloat();
      }
      out.sendMinOnRelease = !server.hasArg("s" + idx + "_o" + oidx + "_smin");
      if (server.hasArg("s" + idx + "_o" + oidx + "_udp")) {
        out.udpPayload = server.arg("s" + idx + "_o" + oidx + "_udp");
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_gpin")) {
        int gpin = server.arg("s" + idx + "_o" + oidx + "_gpin").toInt();
        if (isAllowedGpioOutputPin(gpin)) out.gpioPin = gpin;
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_gmode")) {
        int gm = server.arg("s" + idx + "_o" + oidx + "_gmode").toInt();
        if (gm == GPIO_OUT_LOW) {
          out.gpioMode = GPIO_OUT_HIGH;
          out.gpioInvert = true;
        } else if (gm >= GPIO_OUT_HIGH && gm <= GPIO_OUT_PWM) {
          out.gpioMode = static_cast<GpioOutMode>(gm);
        }
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_gpms")) {
        uint32_t ms = static_cast<uint32_t>(server.arg("s" + idx + "_o" + oidx + "_gpms").toInt());
        if (ms > 0) out.gpioPulseMs = ms;
      }
      out.gpioInvert = server.hasArg("s" + idx + "_o" + oidx + "_ginv");
      if (server.hasArg("s" + idx + "_o" + oidx + "_hm")) {
        int hm = server.arg("s" + idx + "_o" + oidx + "_hm").toInt();
        if (hm >= HTTPM_GET && hm <= HTTPM_POST) out.httpMethod = static_cast<HttpMethod>(hm);
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_hu")) {
        out.httpUrl = server.arg("s" + idx + "_o" + oidx + "_hu");
        out.httpUrl.trim();
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_hb")) {
        out.httpBody = server.arg("s" + idx + "_o" + oidx + "_hb");
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_hip")) {
        out.httpIp = server.arg("s" + idx + "_o" + oidx + "_hip");
        out.httpIp.trim();
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_hpt")) {
        uint16_t hp = static_cast<uint16_t>(server.arg("s" + idx + "_o" + oidx + "_hpt").toInt());
        if (hp > 0) out.httpPort = hp;
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_hdev")) {
        uint16_t hd = static_cast<uint16_t>(server.arg("s" + idx + "_o" + oidx + "_hdev").toInt());
        if (hd < MAX_OSC_DEVICES || hd == 255) out.httpDevice = static_cast<uint8_t>(hd);
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_mt")) {
        out.mqttTopic = server.arg("s" + idx + "_o" + oidx + "_mt");
        out.mqttTopic.trim();
      }
      if (isAutoMqttTopic(out.mqttTopic, previous, s, i, outCount)) {
        out.mqttTopic = buildDefaultMqttTopic(s, i, outCount);
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_mp")) {
        out.mqttPayload = server.arg("s" + idx + "_o" + oidx + "_mp");
      }
      out.mqttRetain = server.hasArg("s" + idx + "_o" + oidx + "_mr");
      if (server.hasArg("s" + idx + "_o" + oidx + "_dmxp")) {
        out.dmxProtocol = sanitizeDmxProtocol(server.arg("s" + idx + "_o" + oidx + "_dmxp").toInt());
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_dmxd")) {
        out.dmxDest = sanitizeDmxDest(server.arg("s" + idx + "_o" + oidx + "_dmxd").toInt());
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_dmxn")) {
        out.dmxArtNet = sanitizeDmxArtNet(server.arg("s" + idx + "_o" + oidx + "_dmxn").toInt());
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_dmxs")) {
        out.dmxArtSubnet = sanitizeDmxArtSubnet(server.arg("s" + idx + "_o" + oidx + "_dmxs").toInt());
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_dmxa")) {
        out.dmxArtUniverse = sanitizeDmxArtUniverse(server.arg("s" + idx + "_o" + oidx + "_dmxa").toInt());
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_dmxu")) {
        uint16_t u = static_cast<uint16_t>(server.arg("s" + idx + "_o" + oidx + "_dmxu").toInt());
        if (u > 0) out.dmxUniverse = u;
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_dmxc")) {
        out.dmxChannels = server.arg("s" + idx + "_o" + oidx + "_dmxc");
        out.dmxChannels.trim();
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_dmxpr")) {
        out.dmxPreset = sanitizeDmxPresetId(server.arg("s" + idx + "_o" + oidx + "_dmxpr").toInt());
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_dmxfd")) {
        out.dmxFadeMs = sanitizeDmxFadeMs(server.arg("s" + idx + "_o" + oidx + "_dmxfd").toInt());
      }
      if (out.dmxProtocol == DMX_PROTO_ARTNET) {
        out.dmxUniverse = encodeArtNetPortAddressOneBased(out.dmxArtNet, out.dmxArtSubnet, out.dmxArtUniverse);
      } else {
        decodeArtNetPortAddressOneBased(out.dmxUniverse, out.dmxArtNet, out.dmxArtSubnet, out.dmxArtUniverse);
      }
      if (out.dmxUniverse == 0) out.dmxUniverse = 1;
      if (out.dmxChannels.length() == 0) out.dmxChannels = "1";
      out.dmxPreset = sanitizeDmxPresetId(out.dmxPreset);
      out.dmxFadeMs = sanitizeDmxFadeMs(out.dmxFadeMs);

      if (out.gpioMode == GPIO_OUT_PWM) {
        if (out.outMin < 0.0f) out.outMin = 0.0f;
        if (out.outMax > GPIO_PWM_MAX) out.outMax = GPIO_PWM_MAX;
      }

      if (out.target == OUT_TARGET_GPIO) {
        if (isInputPinInUse(out.gpioPin, i, s)) {
          int alt = pickFreeGpioOutputPin(i, s);
          if (alt >= 0) {
            addLog(String("GPIO Out pin ") + String(out.gpioPin) + " conflicts with input. Using " + String(alt));
            out.gpioPin = alt;
          } else {
            addLog("GPIO Out pin conflicts with input and no free pins available.");
          }
        }
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_on")) {
        out.onString = server.arg("s" + idx + "_o" + oidx + "_on");
        if (out.onString.length() == 0) out.onString = DEFAULT_ON_STRING;
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_off")) {
        out.offString = server.arg("s" + idx + "_o" + oidx + "_off");
        if (out.offString.length() == 0) out.offString = DEFAULT_OFF_STRING;
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_bon")) {
        out.buttonOnString = server.arg("s" + idx + "_o" + oidx + "_bon");
        if (out.buttonOnString.length() == 0) out.buttonOnString = DEFAULT_ON_STRING;
      }
      if (server.hasArg("s" + idx + "_o" + oidx + "_boff")) {
        out.buttonOffString = server.arg("s" + idx + "_o" + oidx + "_boff");
        if (out.buttonOffString.length() == 0) out.buttonOffString = DEFAULT_OFF_STRING;
      }
      if (out.oscAddress.length() == 0) out.oscAddress = defaultOscAddress(i);
      if (out.buttonAddress.length() == 0) out.buttonAddress = "/button";
      if ((s.type == SENSOR_ANALOG || s.type == SENSOR_ENCODER) && out.outMode == OUT_STRING) out.outMode = OUT_FLOAT;
      if ((out.target == OUT_TARGET_DMX || out.target == OUT_TARGET_DMX_PRESET) && out.outMode == OUT_STRING) out.outMode = OUT_INT;
      if (out.buttonOutMode == OUT_STRING) {
        if (out.buttonOnString.length() == 0) out.buttonOnString = DEFAULT_ON_STRING;
        if (out.buttonOffString.length() == 0) out.buttonOffString = DEFAULT_OFF_STRING;
      }
      if (s.type == SENSOR_ANALOG) out.sendMinOnRelease = true;
      if (server.hasArg("s" + idx + "_o" + oidx + "_svn")) out.setVarName = server.arg("s" + idx + "_o" + oidx + "_svn");
      if (server.hasArg("s" + idx + "_o" + oidx + "_svv")) out.setVarValue = server.arg("s" + idx + "_o" + oidx + "_svv").toFloat();
      s.outputs[outCount++] = out;
    }
    if (outCount == 0) {
      outCount = 1;
    }
    s.outputCount = outCount;

    s.invert = server.hasArg("s" + idx + "_inv");
    s.activeHigh = server.hasArg("s" + idx + "_ah");
    s.pullup = server.hasArg("s" + idx + "_pu");
    s.cooldownEnabled = server.hasArg("s" + idx + "_cd_en");
    if (server.hasArg("s" + idx + "_cd_ms")) {
      uint32_t ms = static_cast<uint32_t>(server.arg("s" + idx + "_cd_ms").toInt());
      if (ms > 0) s.cooldownMs = ms;
    }
    s.conditionEnabled = server.hasArg("s" + idx + "_cden");
    if (server.hasArg("s" + idx + "_cdvar")) s.conditionVar = server.arg("s" + idx + "_cdvar");
    if (server.hasArg("s" + idx + "_cdop")) s.conditionOp = static_cast<uint8_t>(server.arg("s" + idx + "_cdop").toInt());
    if (server.hasArg("s" + idx + "_cdval")) s.conditionValue = server.arg("s" + idx + "_cdval").toFloat();

    if (s.source == SRC_SENSORS) {
      if (server.hasArg("s" + idx + "_pin")) {
        int pin = server.arg("s" + idx + "_pin").toInt();
        if (pin >= 0 && pin <= 39 && !isReservedPin(pin)) {
          if (s.type == SENSOR_ANALOG ? isAllowedAnalogPin(pin) : isAllowedDigitalPin(pin)) {
            s.pin = pin;
          }
        }
      }
    if (s.type == SENSOR_ENCODER) {
      int clk = server.arg("s" + idx + "_clk").toInt();
      int dt = server.arg("s" + idx + "_dt").toInt();
      int sw = server.arg("s" + idx + "_sw").toInt();
      if (clk >= 0 && clk <= 39 && !isReservedPin(clk) && isAllowedDigitalPin(clk)) s.encClkPin = clk;
      if (dt >= 0 && dt <= 39 && !isReservedPin(dt) && isAllowedDigitalPin(dt)) s.encDtPin = dt;
      if (sw >= 0 && sw <= 39 && !isReservedPin(sw) && isAllowedDigitalPin(sw)) s.encSwPin = sw;
      s.encAppendSign = server.hasArg("s" + idx + "_enc_app");
      s.encSignArg = server.hasArg("s" + idx + "_enc_arg");
      s.encInvert = server.hasArg("s" + idx + "_enc_inv");
    } else if (s.type == SENSOR_HCSR04) {
      int trig = server.arg("s" + idx + "_hc_t").toInt();
      int echo = server.arg("s" + idx + "_hc_e").toInt();
      if (trig >= 0 && trig <= 39 && !isReservedPin(trig) && isAllowedDigitalPin(trig)) s.hcTrigPin = trig;
      if (echo >= 0 && echo <= 39 && !isReservedPin(echo) && isAllowedDigitalPin(echo)) s.hcEchoPin = echo;
      if (server.hasArg("s" + idx + "_hc_min")) s.hcMinCm = server.arg("s" + idx + "_hc_min").toFloat();
      if (server.hasArg("s" + idx + "_hc_max")) s.hcMaxCm = server.arg("s" + idx + "_hc_max").toFloat();
      if (s.hcMaxCm == s.hcMinCm) s.hcMaxCm = s.hcMinCm + 1.0f;
    }
    }

    config.sensors[i] = s;
  }

  if (action == "add") {
    if (config.triggerCount < MAX_TRIGGERS) {
      int idx = findUnusedTrigger();
      if (idx >= 0) {
        config.sensors[idx] = defaultSensorConfig(idx);
        config.sensors[idx].enabled = true;
        config.triggerOrder[config.triggerCount++] = static_cast<uint8_t>(idx);
      }
    }
  } else if (action.startsWith("out_add:")) {
    int idx = action.substring(8).toInt();
    if (idx >= 0 && idx < MAX_TRIGGERS) {
      SensorConfig& s = config.sensors[idx];
      if (s.outputCount < MAX_OUTPUTS_PER_TRIGGER) {
        int newOut = s.outputCount;
        s.outputs[newOut] = s.outputs[0];
        s.outputs[newOut].mqttTopic = buildDefaultMqttTopic(s, idx, newOut);
        s.outputCount++;
      }
    }
  } else if (action.startsWith("out_remove:")) {
    int sep = action.indexOf(':', 11);
    if (sep > 0) {
      int idx = action.substring(11, sep).toInt();
      int outIdx = action.substring(sep + 1).toInt();
      if (idx >= 0 && idx < MAX_TRIGGERS) {
        SensorConfig& s = config.sensors[idx];
        if (outIdx > 0 && outIdx < s.outputCount) {
          for (int o = outIdx; o < s.outputCount - 1; o++) {
            s.outputs[o] = s.outputs[o + 1];
          }
          s.outputCount--;
        }
      }
    }
  } else if (action.startsWith("remove:")) {
    int idx = action.substring(7).toInt();
    for (int i = 0; i < config.triggerCount; i++) {
      if (config.triggerOrder[i] == idx) {
        for (int j = i; j < config.triggerCount - 1; j++) {
          config.triggerOrder[j] = config.triggerOrder[j + 1];
        }
        config.triggerCount--;
        break;
      }
    }
    if (idx >= 0 && idx < MAX_TRIGGERS) {
      config.sensors[idx].enabled = false;
    }
  } else if (action.startsWith("up:")) {
    int idx = action.substring(3).toInt();
    for (int i = 1; i < config.triggerCount; i++) {
      if (config.triggerOrder[i] == idx) {
        uint8_t tmp = config.triggerOrder[i - 1];
        config.triggerOrder[i - 1] = config.triggerOrder[i];
        config.triggerOrder[i] = tmp;
        break;
      }
    }
  } else if (action.startsWith("down:")) {
    int idx = action.substring(5).toInt();
    for (int i = 0; i < config.triggerCount - 1; i++) {
      if (config.triggerOrder[i] == idx) {
        uint8_t tmp = config.triggerOrder[i + 1];
        config.triggerOrder[i + 1] = config.triggerOrder[i];
        config.triggerOrder[i] = tmp;
        break;
      }
    }
  }

  saveConfig();
  addLog("Triggers saved.");
  resetRuntimeState(true);
  applySensorPinModes();
  setupEncoderCounters();
  applyGpioOutputsNow();

  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain", "Saved.");
}

static void handleSaveSettings() {
  if (!ensureAuthenticated()) return;
  String action = server.hasArg("action") ? server.arg("action") : "save";
  String oldHostname = config.hostname;
  bool wantPassword = server.hasArg("pwd_enabled");
  String newPassword = server.arg("pwd_new");

  if (server.hasArg("hostname")) {
    config.hostname = sanitizeHostname(server.arg("hostname"));
  }
  if (server.hasArg("username")) {
    String user = server.arg("username");
    user.trim();
    if (user.length() == 0) user = DEFAULT_USERNAME;
    config.username = user;
  }

  if (server.hasArg("net_mode")) {
    config.netMode = sanitizeNetworkMode(server.arg("net_mode").toInt());
  }
#if !USE_ETHERNET
  config.netMode = NET_WIFI;
#endif

  config.oscInEnabled = server.hasArg("osci_en");
  if (server.hasArg("osci_port")) {
    int p = server.arg("osci_port").toInt();
    if (p > 0 && p <= 65535) config.oscInPort = static_cast<uint16_t>(p);
  }
  if (config.oscInPort == 0) config.oscInPort = DEFAULT_OSC_IN_PORT;
  config.artnetEnabled = server.hasArg("artnet_en");
  config.sacnEnabled = server.hasArg("sacn_en");
  config.dmxInArtNetEnabled = server.hasArg("dmxi_art");
  config.dmxInSacnEnabled = server.hasArg("dmxi_sac");
  if (server.hasArg("dmxi_src")) {
    config.dmxInSourceMode = sanitizeDmxInputSourceMode(server.arg("dmxi_src").toInt());
  }

  String wifiSsid = "";
  if (server.hasArg("wifi_ssid")) {
    wifiSsid = server.arg("wifi_ssid");
  }
  if (wifiSsid.length() == 0 && server.hasArg("wifi_ssid_select")) {
    String sel = server.arg("wifi_ssid_select");
    if (sel.length() > 0 && sel != "__custom__") {
      wifiSsid = sel;
    }
  }
  if (wifiSsid.length() > 0) {
    config.wifiSsid = wifiSsid;
  }
  if (server.hasArg("wifi_pass")) {
    String pass = server.arg("wifi_pass");
    if (pass.length() > 0) config.wifiPass = pass;
  }

  config.useStatic = !server.hasArg("dhcp_enabled");

  if (server.hasArg("static_ip")) {
    IPAddress parsed;
    if (parseIpString(server.arg("static_ip"), parsed)) {
      config.staticIp = parsed;
    }
  }

  if (server.hasArg("gateway")) {
    IPAddress parsed;
    if (parseIpString(server.arg("gateway"), parsed)) {
      config.gateway = parsed;
    }
  }

  if (server.hasArg("subnet")) {
    IPAddress parsed;
    if (parseIpString(server.arg("subnet"), parsed)) {
      config.subnet = parsed;
    }
  }

  if (server.hasArg("dns1")) {
    IPAddress parsed;
    if (parseIpString(server.arg("dns1"), parsed)) {
      config.dns1 = parsed;
    }
  }

  if (server.hasArg("dns2")) {
    IPAddress parsed;
    if (parseIpString(server.arg("dns2"), parsed)) {
      config.dns2 = parsed;
    }
  }

  for (int i = 0; i < MAX_OSC_DEVICES; i++) {
    if (!server.hasArg("d" + String(i) + "_present")) continue;
    OscDevice d = config.oscDevices[i];
    d.enabled = server.hasArg("dev" + String(i) + "_en");
    if (server.hasArg("dev" + String(i) + "_name")) {
      d.name = server.arg("dev" + String(i) + "_name");
    }
    if (server.hasArg("dev" + String(i) + "_ip")) {
      d.ip = parseOrDefault(server.arg("dev" + String(i) + "_ip"), d.ip);
    }
    if (server.hasArg("dev" + String(i) + "_port")) {
      int port = server.arg("dev" + String(i) + "_port").toInt();
      if (port > 0 && port <= 65535) d.port = static_cast<uint16_t>(port);
    }
    d.hbEnabled = server.hasArg("dev" + String(i) + "_hb_en");
    if (server.hasArg("dev" + String(i) + "_hb_addr")) {
      d.hbAddress = normalizeOscAddress(server.arg("dev" + String(i) + "_hb_addr"));
    }
    if (server.hasArg("dev" + String(i) + "_hb_ms")) {
      uint32_t ms = static_cast<uint32_t>(server.arg("dev" + String(i) + "_hb_ms").toInt());
      if (ms > 0) d.hbMs = ms;
    }
    if (d.name.length() == 0) d.name = String("Device ") + String(i + 1);
    if (d.hbAddress.length() == 0) d.hbAddress = "/heartbeat";
    if (d.hbMs == 0) d.hbMs = 5000;
    config.oscDevices[i] = d;
  }
  if (action == "dev_add") {
    if (config.oscDeviceCount < MAX_OSC_DEVICES) {
      int idx = findUnusedOscDevice();
      if (idx >= 0) {
        OscDevice d;
        d.enabled = true;
        d.name = String("Device ") + String(idx + 1);
        d.ip = config.clientIp;
        d.port = config.clientPort;
        d.hbEnabled = false;
        d.hbAddress = "/heartbeat";
        d.hbMs = 5000;
        config.oscDevices[idx] = d;
        config.oscDeviceOrder[config.oscDeviceCount++] = static_cast<uint8_t>(idx);
      }
    }
  } else if (action.startsWith("dev_remove:")) {
    int idx = action.substring(11).toInt();
    if (idx != 0) {
      for (int i = 0; i < config.oscDeviceCount; i++) {
        if (config.oscDeviceOrder[i] == idx) {
          for (int j = i; j < config.oscDeviceCount - 1; j++) {
            config.oscDeviceOrder[j] = config.oscDeviceOrder[j + 1];
          }
          config.oscDeviceCount--;
          break;
        }
      }
      config.oscDevices[idx].enabled = false;
    }
  } else if (action.startsWith("dev_up:")) {
    int idx = action.substring(7).toInt();
    for (int i = 1; i < config.oscDeviceCount; i++) {
      if (config.oscDeviceOrder[i] == idx) {
        uint8_t tmp = config.oscDeviceOrder[i - 1];
        config.oscDeviceOrder[i - 1] = config.oscDeviceOrder[i];
        config.oscDeviceOrder[i] = tmp;
        break;
      }
    }
  } else if (action.startsWith("dev_down:")) {
    int idx = action.substring(9).toInt();
    for (int i = 0; i < config.oscDeviceCount - 1; i++) {
      if (config.oscDeviceOrder[i] == idx) {
        uint8_t tmp = config.oscDeviceOrder[i + 1];
        config.oscDeviceOrder[i + 1] = config.oscDeviceOrder[i];
        config.oscDeviceOrder[i] = tmp;
        break;
      }
    }
  }
  if (config.mqttDevice < MAX_OSC_DEVICES) {
    bool mqttDevPresent = false;
    for (int i = 0; i < config.oscDeviceCount; i++) {
      if (config.oscDeviceOrder[i] == config.mqttDevice) {
        mqttDevPresent = true;
        break;
      }
    }
    if (!mqttDevPresent) config.mqttDevice = DEFAULT_MQTT_DEVICE;
  }
  config.clientIp = config.oscDevices[0].ip;
  config.clientPort = config.oscDevices[0].port;

  if (server.hasArg("test_dev")) {
    int v = server.arg("test_dev").toInt();
    if (v >= 0 && v < MAX_OSC_DEVICES) config.testDevice = static_cast<uint8_t>(v);
  }

  if (server.hasArg("click_db")) {
    uint32_t ms = static_cast<uint32_t>(server.arg("click_db").toInt());
    if (ms > 0) config.clickDebounceMs = ms;
  }
  if (server.hasArg("click_gap")) {
    uint32_t ms = static_cast<uint32_t>(server.arg("click_gap").toInt());
    if (ms > 0) config.multiClickGapMs = ms;
  }
  if (server.hasArg("long_ms")) {
    uint32_t ms = static_cast<uint32_t>(server.arg("long_ms").toInt());
    if (ms > 0) config.longPressMs = ms;
  }
  config.analogSmoothing = server.hasArg("analog_smooth");
  if (server.hasArg("time_mode")) {
    uint8_t mode = static_cast<uint8_t>(server.arg("time_mode").toInt());
    if (mode <= TIME_MANUAL) config.timeMode = mode;
  }
  if (server.hasArg("ntp_srv")) {
    String srv = server.arg("ntp_srv");
    srv.trim();
    if (srv.length() == 0) srv = DEFAULT_NTP_SERVER;
    config.ntpServer = srv;
  }
  if (server.hasArg("tz")) {
    String tz = server.arg("tz");
    tz.trim();
    if (tz.length() == 0) tz = DEFAULT_TIMEZONE;
    config.timeZone = tz;
  }
  if (server.hasArg("time_24")) {
    config.timeDisplay24h = (server.arg("time_24").toInt() == 1);
  }
  if (server.hasArg("manual_date") && server.hasArg("manual_time")) {
    uint32_t epoch = 0;
    setenv("TZ", config.timeZone.c_str(), 1);
    tzset();
    if (parseDateTimeLocal(server.arg("manual_date"), server.arg("manual_time"), epoch)) {
      config.manualEpoch = epoch;
    }
  }

  config.mqttEnabled = server.hasArg("mqtt_en");
  if (server.hasArg("mqtt_dev")) {
    int d = server.arg("mqtt_dev").toInt();
    if ((d >= 0 && d < MAX_OSC_DEVICES) || d == MQTT_DEVICE_CUSTOM) {
      config.mqttDevice = static_cast<uint8_t>(d);
    }
  }
  if (config.mqttDevice == MQTT_DEVICE_CUSTOM) {
    if (server.hasArg("mqtt_host")) {
      config.mqttHost = server.arg("mqtt_host");
      config.mqttHost.trim();
    }
    if (server.hasArg("mqtt_port")) {
      int p = server.arg("mqtt_port").toInt();
      if (p > 0 && p <= 65535) config.mqttPort = static_cast<uint16_t>(p);
    }
  }
  if (server.hasArg("mqtt_user")) {
    config.mqttUser = server.arg("mqtt_user");
    config.mqttUser.trim();
  }
  if (server.hasArg("mqtt_client_id")) {
    config.mqttClientId = server.arg("mqtt_client_id");
    config.mqttClientId.trim();
    if (config.mqttClientId.length() == 0) config.mqttClientId = defaultMqttClientId();
  }
  if (server.hasArg("mqtt_base")) {
    config.mqttBaseTopic = server.arg("mqtt_base");
    config.mqttBaseTopic.trim();
    if (config.mqttBaseTopic.length() == 0) config.mqttBaseTopic = DEFAULT_MQTT_BASE_TOPIC;
  }
  if (server.hasArg("mqtt_pass")) {
    String pass = server.arg("mqtt_pass");
    if (pass.length() > 0) config.mqttPass = pass;
  }
  if (config.mqttPort == 0) config.mqttPort = DEFAULT_MQTT_PORT;
  mqttReconnectRequested = true;

  if (!wantPassword) {
    config.passwordEnabled = false;
    config.password = "";
  } else if (newPassword.length() > 0) {
    config.passwordEnabled = true;
    config.password = newPassword;
  } else if (config.password.length() > 0) {
    config.passwordEnabled = true;
  } else {
    config.passwordEnabled = false;
  }

  saveConfig();
  addLog(String("WiFi saved SSID=") + (config.wifiSsid.length() ? config.wifiSsid : "<blank>") +
         " DHCP=" + String(config.useStatic ? "off" : "on"));
  addLog("Settings saved.");
  applyTimeConfig();
  pendingNetApply = true;
  pendingApplyAt = millis();
  pendingMdnsRestart = (config.hostname != oldHostname);
  setupEncoderCounters();

  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain", "Saved.");
}

static void handleExportConfig() {
  if (!ensureAuthenticated()) return;
  String payload = exportConfigJson();
  server.sendHeader("Content-Disposition", "attachment; filename=stagemod-config.json");
  server.send(200, "application/json", payload);
}

static void handleImportUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    importPayload = "";
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    importPayload += String((const char*)upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    // no-op
  }
}

static void handleImportConfig() {
  if (!ensureAuthenticated()) return;
  if (importPayload.length() == 0) {
    server.send(400, "text/plain", "Import failed: no file content received.");
    return;
  }

  DynamicJsonDocument doc(32768);
  DeserializationError err = deserializeJson(doc, importPayload);
  if (err) {
    server.send(400, "text/plain", String("Import failed: invalid JSON (") + err.c_str() + ").");
    return;
  }

  String fileVersion = doc["version"] | "";
  JsonObject cfg = doc["config"].as<JsonObject>();
  String applyErr;
  if (!applyConfigFromJson(cfg, applyErr)) {
    server.send(400, "text/plain", String("Import failed: ") + applyErr);
    return;
  }

  saveConfig();
  setupEncoderCounters();
  applyTimeConfig();
  mqttReconnectRequested = true;

  String msg = "<html><body><h3>Import complete</h3>";
  if (fileVersion.length() > 0) {
    int cmp = compareVersion(fileVersion, FIRMWARE_VERSION);
    if (cmp < 0) {
      msg += "<p>Warning: Imported config is from an older firmware (" + fileVersion + ") than this device (" + String(FIRMWARE_VERSION) + ").</p>";
    } else if (cmp > 0) {
      msg += "<p>Warning: Imported config is from a newer firmware (" + fileVersion + ") than this device (" + String(FIRMWARE_VERSION) + ").</p>";
    } else {
      msg += "<p>Config version matches firmware (" + String(FIRMWARE_VERSION) + ").</p>";
    }
  } else {
    msg += "<p>Config version not found in file.</p>";
  }
  msg += "<p>Network settings and passwords are not imported.</p>";
  msg += "<p><a href='/settings'>Back to settings</a></p></body></html>";
  server.send(200, "text/html", msg);
  importPayload = "";
}

static void handleDmxPresetEditor() {
  if (!ensureAuthenticated()) return;

  int preset = server.hasArg("preset") ? server.arg("preset").toInt() : 1;
  if (preset < 1) preset = 1;
  if (preset > MAX_DMX_PRESETS) preset = MAX_DMX_PRESETS;
  int presetIndex = preset - 1;
  int universe = server.hasArg("u") ? server.arg("u").toInt() : 1;
  if (universe < 1) universe = 1;
  if (universe > MAX_DMX_PRESET_UNIVERSE) universe = MAX_DMX_PRESET_UNIVERSE;
  int liveSourceMode = server.hasArg("live_src") ? server.arg("live_src").toInt() : DMX_IN_SOURCE_RECENT;
  liveSourceMode = sanitizeDmxInputSourceMode(liveSourceMode);
  int liveFrom = server.hasArg("lu_from") ? server.arg("lu_from").toInt() : universe;
  int liveTo = server.hasArg("lu_to") ? server.arg("lu_to").toInt() : universe;
  if (liveFrom < 1) liveFrom = 1;
  if (liveFrom > MAX_DMX_PRESET_UNIVERSE) liveFrom = MAX_DMX_PRESET_UNIVERSE;
  if (liveTo < 1) liveTo = 1;
  if (liveTo > MAX_DMX_PRESET_UNIVERSE) liveTo = MAX_DMX_PRESET_UNIVERSE;
  if (liveTo < liveFrom) {
    int t = liveFrom;
    liveFrom = liveTo;
    liveTo = t;
  }
  bool hasLiveArgs = server.hasArg("live_src") || server.hasArg("lu_from") || server.hasArg("lu_to") || server.hasArg("live_nonzero");
  bool liveNonZeroOnly = hasLiveArgs ? server.hasArg("live_nonzero") : true;

  String notice = "";
  String error = "";

  if (server.method() == HTTP_POST) {
    String action = server.hasArg("action") ? server.arg("action") : "save_grid";
    if (action == "clear") {
      dmxPresetText[presetIndex] = "";
      saveConfig();
      notice = String("Preset ") + String(preset) + " cleared.";
    } else if (action == "clear_universe") {
      String merged = removeDmxUniverseRows(dmxPresetText[presetIndex], universe);
      String normalized;
      String parseErr;
      int pointCount = parseDmxPresetText(merged, nullptr, MAX_DMX_PRESET_POINTS, &normalized, parseErr);
      if (pointCount < 0) {
        error = parseErr;
      } else {
        int otherPoints = totalDmxPresetPointsExcluding(presetIndex);
        if (otherPoints + pointCount > MAX_DMX_PRESET_POINTS) {
          error = String("Point budget exceeded. Used by other presets: ") + String(otherPoints) +
                  ". Remaining: " + String(MAX_DMX_PRESET_POINTS - otherPoints) + ".";
        } else {
          dmxPresetText[presetIndex] = normalized;
          saveConfig();
          notice = String("Preset ") + String(preset) + ": cleared universe " + String(universe) + ".";
        }
      }
    } else if (action == "snapshot_live") {
      String merged = dmxPresetText[presetIndex];
      int snapshotPoints = 0;
      int snapshotUniverses = 0;
      uint8_t liveData[512];
      uint8_t selectedSource = DMX_PROTO_ARTNET;
      uint32_t selectedAgeMs = 0;
      for (int u = liveFrom; u <= liveTo; u++) {
        uint16_t uni = static_cast<uint16_t>(u);
        if (!getLiveDmxUniverseValues(uni, static_cast<uint8_t>(liveSourceMode), liveData, selectedSource, selectedAgeMs)) continue;

        int16_t values[512];
        for (int ch = 0; ch < 512; ch++) {
          uint8_t v = liveData[ch];
          if (liveNonZeroOnly && v == 0) {
            values[ch] = -1;
          } else {
            values[ch] = static_cast<int16_t>(v);
            snapshotPoints++;
          }
        }

        merged = removeDmxUniverseRows(merged, uni);
        String uniRows = serializeDmxUniverseRows(uni, values);
        if (uniRows.length() > 0) {
          if (merged.length() > 0) merged += "\n";
          merged += uniRows;
        }
        snapshotUniverses++;
      }

      if (snapshotUniverses == 0) {
        error = "No DMX input data found in selected universe range.";
      } else {
        String normalized;
        String parseErr;
        int pointCount = parseDmxPresetText(merged, nullptr, MAX_DMX_PRESET_POINTS, &normalized, parseErr);
        if (pointCount < 0) {
          error = parseErr;
        } else {
          int otherPoints = totalDmxPresetPointsExcluding(presetIndex);
          if (otherPoints + pointCount > MAX_DMX_PRESET_POINTS) {
            error = String("Point budget exceeded. Used by other presets: ") + String(otherPoints) +
                    ". Remaining: " + String(MAX_DMX_PRESET_POINTS - otherPoints) + ".";
          } else {
            dmxPresetText[presetIndex] = normalized;
            saveConfig();
            notice = String("Preset ") + String(preset) + ": snapshot saved from universes " + String(liveFrom) + "-" + String(liveTo) +
                     " (" + String(snapshotPoints) + " points).";
          }
        }
      }
    } else if (action == "save_grid") {
      int16_t values[512];
      for (int i = 0; i < 512; i++) values[i] = -1;
      for (int ch = 1; ch <= 512; ch++) {
        String key = "ch" + String(ch);
        if (!server.hasArg(key)) continue;
        String raw = server.arg(key);
        raw.trim();
        if (raw.length() == 0) continue;
        int v = raw.toInt();
        if (v < 0) v = 0;
        if (v > 255) v = 255;
        values[ch - 1] = static_cast<int16_t>(v);
      }

      String merged = removeDmxUniverseRows(dmxPresetText[presetIndex], universe);
      String uniRows = serializeDmxUniverseRows(universe, values);
      if (uniRows.length() > 0) {
        if (merged.length() > 0) merged += "\n";
        merged += uniRows;
      }

      String normalized;
      String parseErr;
      int pointCount = parseDmxPresetText(merged, nullptr, MAX_DMX_PRESET_POINTS, &normalized, parseErr);
      if (pointCount < 0) {
        error = parseErr;
      } else {
        int otherPoints = totalDmxPresetPointsExcluding(presetIndex);
        if (otherPoints + pointCount > MAX_DMX_PRESET_POINTS) {
          error = String("Point budget exceeded. Used by other presets: ") + String(otherPoints) +
                  ". Remaining: " + String(MAX_DMX_PRESET_POINTS - otherPoints) + ".";
        } else {
          dmxPresetText[presetIndex] = normalized;
          saveConfig();
          notice = String("Preset ") + String(preset) + ": saved universe " + String(universe) + " grid.";
        }
      }
    } else if (action == "preview") {
      uint8_t proto = server.hasArg("pproto") ? sanitizeDmxProtocol(server.arg("pproto").toInt()) : DMX_PROTO_ARTNET;
      SensorConfig defaults = defaultSensorConfig(0);
      SensorConfig::OutputConfig out = defaults.outputs[0];
      out.target = OUT_TARGET_DMX_PRESET;
      out.dmxProtocol = proto;
      out.dmxDest = DMX_DEST_AUTO;
      out.dmxPreset = static_cast<uint8_t>(preset);
      out.dmxFadeMs = 0;
      if (triggerDmxPreset(out)) {
        notice = String("Previewing preset ") + String(preset) + " over " +
                 String(proto == DMX_PROTO_ARTNET ? "Art-Net" : "sACN") +
                 " — streaming until you Release.";
      } else {
        error = String("Preview failed — check that ") +
                String(proto == DMX_PROTO_ARTNET ? "Art-Net" : "sACN") +
                " Output is enabled in Settings and the preset has saved data.";
      }
    } else if (action == "release") {
      releaseDmxOutputs();
      notice = "DMX output released — all universes zeroed and streaming stopped.";
    } else {
      String raw = server.hasArg("preset_text") ? server.arg("preset_text") : "";
      String normalized;
      String parseErr;
      int pointCount = parseDmxPresetText(raw, nullptr, MAX_DMX_PRESET_POINTS, &normalized, parseErr);
      if (pointCount < 0) {
        error = parseErr;
      } else {
        int otherPoints = totalDmxPresetPointsExcluding(presetIndex);
        if (otherPoints + pointCount > MAX_DMX_PRESET_POINTS) {
          error = String("Point budget exceeded. Used by other presets: ") + String(otherPoints) +
                  ". Remaining: " + String(MAX_DMX_PRESET_POINTS - otherPoints) + ".";
        } else {
          dmxPresetText[presetIndex] = normalized;
          saveConfig();
          notice = String("Preset ") + String(preset) + " saved (" + String(pointCount) + " points).";
        }
      }
    }
  }

  String currentText = dmxPresetText[presetIndex];
  String parseErr;
  int currentPoints = parseDmxPresetText(currentText, nullptr, MAX_DMX_PRESET_POINTS, nullptr, parseErr);
  if (currentPoints < 0) currentPoints = 0;
  int totalPoints = totalDmxPresetPointsExcluding(-1);
  int16_t universeValues[512];
  fillDmxUniverseValues(currentText, universe, universeValues);
  int activeChannels = 0;
  for (int i = 0; i < 512; i++) {
    if (universeValues[i] >= 0) activeChannels++;
  }

  int liveIndex = universe - 1;
  uint32_t artSeenMs = dmxInArtSeenMs[liveIndex];
  uint32_t sacSeenMs = dmxInSacnSeenMs[liveIndex];
  bool hasArtLive = artSeenMs > 0;
  bool hasSacLive = sacSeenMs > 0;
  uint8_t livePreviewData[512];
  uint8_t livePreviewSource = DMX_PROTO_ARTNET;
  uint32_t livePreviewAgeMs = 0;
  bool hasLivePreview = getLiveDmxUniverseValues(static_cast<uint16_t>(universe), static_cast<uint8_t>(liveSourceMode), livePreviewData, livePreviewSource, livePreviewAgeMs);
  int livePreviewActiveChannels = 0;
  if (hasLivePreview) {
    for (int ch = 0; ch < 512; ch++) {
      if (livePreviewData[ch] > 0) livePreviewActiveChannels++;
    }
  }

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Connection", "close");
  server.send(200, "text/html", "");
  String html;
  html.reserve(4096);
  auto flushHtml = [&]() {
    if (html.length() > 0) {
      server.sendContent(html);
      html = "";
    }
  };
  auto appendHtml = [&](const String& s) {
    html += s;
    if (html.length() > 3000) flushHtml();
  };
  auto appendHtmlRaw = [&](const char* s) {
    html += s;
    if (html.length() > 3000) flushHtml();
  };

  appendHtmlRaw("<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>");
  appendHtmlRaw("<title>DMX Preset Editor</title>");
  appendHtmlRaw("<style>");
  appendHtmlRaw("body{margin:0;font-family:'Avenir Next','Trebuchet MS','Segoe UI',sans-serif;background:linear-gradient(135deg,#f4f1ea 0%,#e7f0ef 100%);color:#1a1a1a;}");
  appendHtmlRaw(".page{max-width:920px;margin:0 auto;padding:20px 16px 40px;}");
  appendHtmlRaw(".card{background:#fff;border:1px solid #e2e2e2;border-radius:12px;padding:12px;margin:10px 0;}");
  appendHtmlRaw("input,select,textarea{width:100%;max-width:860px;padding:6px 8px;margin:2px 0 8px;border:1px solid #cfcfcf;border-radius:6px;background:#fff;}");
  appendHtmlRaw("textarea{min-height:220px;font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;}");
  appendHtmlRaw("button{border:1px solid #b9b9b9;background:#fff;color:#222;padding:6px 10px;border-radius:8px;cursor:pointer;}");
  appendHtmlRaw("button.primary{background:#0b6b6f;color:#fff;border-color:#0b6b6f;}");
  appendHtmlRaw(".top{display:flex;gap:10px;align-items:center;flex-wrap:wrap;justify-content:space-between;}");
  appendHtmlRaw(".notice{color:#0b6b6f;font-weight:600;}");
  appendHtmlRaw(".error{color:#a02a2a;font-weight:600;}");
  appendHtmlRaw(".grid-wrap{overflow:auto;border:1px solid #ddd;border-radius:10px;background:#fbfbfb;max-height:70vh;}");
  appendHtmlRaw("table.grid{border-collapse:collapse;min-width:980px;font-size:12px;}");
  appendHtmlRaw("table.grid td{border:1px solid #e6e6e6;padding:2px;text-align:center;}");
  appendHtmlRaw("table.grid .cell{display:flex;flex-direction:column;align-items:stretch;gap:2px;}");
  appendHtmlRaw("table.grid .ch{font-size:10px;line-height:1;color:#888;padding-top:2px;}");
  appendHtmlRaw("table.grid .ch.on{color:#0b6b6f;font-weight:700;}");
  appendHtmlRaw("table.grid input{width:52px;max-width:none;margin:0;padding:4px 4px;font-size:12px;text-align:center;border-color:transparent;background:transparent;}");
  appendHtmlRaw("table.grid input:focus{border-color:#0b6b6f;background:#fff;outline:none;}");
  appendHtmlRaw("form.inline{display:inline-flex;gap:8px;align-items:center;flex-wrap:wrap;margin:0;}");
  appendHtmlRaw("form.inline select,form.inline input{margin:0;width:auto;}");
  appendHtmlRaw(".lbl{font-size:12px;color:#666;font-weight:600;}");
  appendHtmlRaw(".meta{font-size:12px;color:#666;}");
  appendHtmlRaw("a{color:#0b6b6f;}");
  appendHtmlRaw("@media(prefers-color-scheme:dark){");
  appendHtmlRaw("body{background:#161616;color:#e2e2e2;}");
  appendHtmlRaw(".card,fieldset,select,input,textarea{background:#222;border-color:#383838;color:#e2e2e2;}");
  appendHtmlRaw("button{background:#2a2a2a;border-color:#444;color:#e2e2e2;}button.primary{background:#0d9488;border-color:#0d9488;color:#fff;}");
  appendHtmlRaw("a{color:#0d9488;}.notice{color:#0d9488;}");
  appendHtmlRaw(".grid-wrap{background:#1c1c1c;border-color:#333;}");
  appendHtmlRaw("table.grid td{border-color:#2e2e2e;}");
  appendHtmlRaw("table.grid .ch{color:#777;}table.grid .ch.on{color:#0d9488;}");
  appendHtmlRaw("table.grid input{background:transparent;color:#e2e2e2;}");
  appendHtmlRaw("table.grid input:focus{border-color:#0d9488;background:#222;}");
  appendHtmlRaw(".lbl,.meta{color:#999;}");
  appendHtmlRaw("}");
  appendHtmlRaw("</style></head><body><div class='page'>");

  // Hidden fields so each small form round-trips the full editor state.
  // A form must NOT include a hidden copy of the field its own visible
  // control submits, or the query string carries duplicates and the server
  // reads the stale first value.
  auto stateFields = [&](bool withPreset, bool withU, bool withLive) -> String {
    String f = "";
    if (withPreset) f += String("<input type='hidden' name='preset' value='") + String(preset) + "'>";
    if (withU) f += String("<input type='hidden' name='u' value='") + String(universe) + "'>";
    if (withLive) {
      f += String("<input type='hidden' name='live_src' value='") + String(liveSourceMode) + "'>";
      f += String("<input type='hidden' name='lu_from' value='") + String(liveFrom) + "'>";
      f += String("<input type='hidden' name='lu_to' value='") + String(liveTo) + "'>";
      if (liveNonZeroOnly) f += "<input type='hidden' name='live_nonzero' value='1'>";
    }
    return f;
  };

  appendHtmlRaw("<div class='top'><h2 style='margin:0;'>DMX Preset Editor</h2><a href='/'>Back to StageMod</a></div>");

  appendHtmlRaw("<div class='card'><div class='top'>");
  appendHtmlRaw("<form method='GET' action='/dmx_presets' class='inline'>");
  appendHtml(stateFields(false, true, true));
  appendHtmlRaw("<label class='lbl'>Preset</label><select name='preset' onchange='this.form.submit()' style='min-width:180px;'>");
  for (int i = 1; i <= MAX_DMX_PRESETS; i++) {
    String pText = dmxPresetText[i - 1];
    pText.trim();
    appendHtml(String("<option value='") + String(i) + "' " + String(i == preset ? "selected" : "") + ">Preset " + String(i) + String(pText.length() > 0 ? " *" : "") + "</option>");
  }
  appendHtmlRaw("</select></form>");
  appendHtml(String("<div class='meta'>Point budget <b>") + String(totalPoints) + "</b>/" + String(MAX_DMX_PRESET_POINTS) +
             " &middot; this preset <b>" + String(currentPoints) + "</b> points &middot; * = has data</div>");
  appendHtmlRaw("</div>");
  if (notice.length() > 0) appendHtml(String("<div class='notice'>") + notice + "</div>");
  if (error.length() > 0) appendHtml(String("<div class='error'>") + error + "</div>");
  appendHtmlRaw("</div>");

  appendHtmlRaw("<div class='card'><b>Live DMX Snapshot</b><br>");
  appendHtmlRaw("<small>Capture what your console is outputting right now into this preset. Set the universe range, then snapshot.</small>");
  appendHtmlRaw("<form method='GET' action='/dmx_presets' class='inline' style='margin-top:8px;'>");
  appendHtml(stateFields(true, true, false));
  appendHtmlRaw("<label class='lbl'>Source</label><select name='live_src' onchange='this.form.submit()'>");
  appendHtml(String("<option value='0' ") + String(liveSourceMode == DMX_IN_SOURCE_RECENT ? "selected" : "") + ">Auto</option>");
  appendHtml(String("<option value='1' ") + String(liveSourceMode == DMX_IN_SOURCE_PREFER_ARTNET ? "selected" : "") + ">Art-Net</option>");
  appendHtml(String("<option value='2' ") + String(liveSourceMode == DMX_IN_SOURCE_PREFER_SACN ? "selected" : "") + ">sACN</option>");
  appendHtmlRaw("</select>");
  appendHtml(String("<label class='lbl'>Universes</label><input name='lu_from' type='number' min='1' max='") + String(MAX_DMX_PRESET_UNIVERSE) + "' value='" + String(liveFrom) + "' style='max-width:80px;' onchange='this.form.submit()'>");
  appendHtml(String("<span class='lbl'>to</span><input name='lu_to' type='number' min='1' max='") + String(MAX_DMX_PRESET_UNIVERSE) + "' value='" + String(liveTo) + "' style='max-width:80px;' onchange='this.form.submit()'>");
  appendHtml(String("<label class='lbl' style='display:inline-flex;align-items:center;gap:6px;'><input name='live_nonzero' type='checkbox' value='1' style='width:auto;margin:0;' ") + String(liveNonZeroOnly ? "checked" : "") + " onchange='this.form.submit()'>Non-zero only</label>");
  appendHtmlRaw("</form>");
  appendHtmlRaw("<form method='POST' action='/dmx_presets' style='margin-top:8px;'>");
  appendHtml(stateFields(true, true, true));
  appendHtmlRaw("<button type='submit' name='action' value='snapshot_live' class='primary'>Snapshot Into This Preset</button>");
  appendHtmlRaw("</form>");
  appendHtml(String("<div class='meta' style='margin-top:4px;'>Universe numbers here match sACN; Art-Net wire numbering is one less (QLab/console Art-Net universe 0 = Universe 1).</div>"));
  appendHtml(String("<div class='meta' style='margin-top:8px;'>Live input on U") + String(universe) + " (Art-Net " + String(universe - 1) + "): Art-Net " +
             String(!artnetListenActive ? "<b>input disabled in Settings</b>" : (hasArtLive ? ("seen " + String(millis() - artSeenMs) + "ms ago") : "listening, no data yet")) +
             " &middot; sACN " +
             String(!sacnListenActive ? "<b>input disabled in Settings</b>" : (hasSacLive ? ("seen " + String(millis() - sacSeenMs) + "ms ago") : "listening, no data yet")) + "</div>");
  if (hasLivePreview) {
    appendHtml(String("<div class='meta'>Preview source <b>") + String(livePreviewSource == DMX_PROTO_ARTNET ? "Art-Net" : "sACN") +
               "</b> (" + String(livePreviewAgeMs) + "ms ago) &middot; " + String(livePreviewActiveChannels) + " channels above zero</div>");
  }
  appendHtmlRaw("</div>");

  appendHtmlRaw("<div class='card'><div class='top'>");
  appendHtmlRaw("<form method='GET' action='/dmx_presets' class='inline'>");
  appendHtml(stateFields(true, false, true));
  appendHtmlRaw("<b>Universe Grid</b><label class='lbl' style='margin-left:10px;'>Universe</label><select name='u' onchange='this.form.submit()' style='min-width:140px;'>");
  // Mark universes that have stored data (rows start with "<universe>,")
  bool uniHasData[MAX_DMX_PRESET_UNIVERSE + 1] = {false};
  {
    int lineStart = 0;
    while (lineStart < static_cast<int>(currentText.length())) {
      int lineEnd = currentText.indexOf('\n', lineStart);
      if (lineEnd < 0) lineEnd = currentText.length();
      int comma = currentText.indexOf(',', lineStart);
      if (comma > lineStart && comma < lineEnd) {
        int u = currentText.substring(lineStart, comma).toInt();
        if (u >= 1 && u <= MAX_DMX_PRESET_UNIVERSE) uniHasData[u] = true;
      }
      lineStart = lineEnd + 1;
    }
  }
  for (int i = 1; i <= MAX_DMX_PRESET_UNIVERSE; i++) {
    appendHtml(String("<option value='") + String(i) + "' " + String(i == universe ? "selected" : "") + ">Universe " + String(i) + " &middot; Art-Net " + String(i - 1) + String(uniHasData[i] ? " *" : "") + "</option>");
  }
  appendHtmlRaw("</select></form>");
  appendHtml(String("<div class='meta'><b>") + String(activeChannels) + "</b> channels stored in this universe</div>");
  appendHtmlRaw("</div>");
  appendHtmlRaw("<small>Each cell is one DMX channel (number above, value 0-255 below). Blank = not stored in this preset.</small>");
  appendHtmlRaw("<form method='POST' action='/dmx_presets'>");
  appendHtml(stateFields(true, true, true));
  appendHtmlRaw("<div class='grid-wrap'><table class='grid'>");
  for (int row = 0; row < 32; row++) {
    int rowStart = row * 16 + 1;
    appendHtmlRaw("<tr>");
    for (int col = 0; col < 16; col++) {
      int ch = rowStart + col;
      String v = universeValues[ch - 1] >= 0 ? String(universeValues[ch - 1]) : "";
      bool stored = universeValues[ch - 1] >= 0;
      appendHtml(String("<td><div class='cell'><div class='ch") + String(stored ? " on" : "") + "'>" + String(ch) + "</div><input type='number' min='0' max='255' name='ch" + String(ch) + "' value='" + v + "' placeholder='-'></div></td>");
    }
    appendHtmlRaw("</tr>");
  }
  appendHtmlRaw("</table></div><br>");
  appendHtmlRaw("<button type='submit' name='action' value='save_grid' class='primary'>Save Universe</button> ");
  appendHtmlRaw("<button type='submit' name='action' value='clear_universe'>Clear Universe</button>");
  appendHtmlRaw("</form>");
  appendHtmlRaw("<hr style='border:none;border-top:1px solid #38383833;margin:12px 0;'>");
  appendHtmlRaw("<form method='POST' action='/dmx_presets' class='inline'>");
  appendHtml(stateFields(true, true, true));
  appendHtmlRaw("<label class='lbl'>Preview on rig</label><select name='pproto'>");
  appendHtml(String("<option value='0'") + String(config.artnetEnabled ? "" : " disabled") + ">Art-Net</option>");
  appendHtml(String("<option value='1'") + String(config.sacnEnabled ? "" : " disabled") + ">sACN</option>");
  appendHtmlRaw("</select>");
  appendHtmlRaw("<button type='submit' name='action' value='preview' class='primary'>Preview Preset</button>");
  appendHtmlRaw("<button type='submit' name='action' value='release'>Release Output</button>");
  appendHtmlRaw("</form>");
  appendHtmlRaw("<div class='meta' style='margin-top:6px;'>Preview broadcasts the <b>saved</b> preset to your fixtures and keeps streaming it (save grid edits first). Release zeroes and stops all StageMod DMX output. If a console is outputting the same universes, the two sources will fight &mdash; pause the console's output while building looks.</div>");
  appendHtmlRaw("</div>");

  appendHtmlRaw("<details class='card'><summary><b>Advanced Text Editor</b></summary>");
  appendHtmlRaw("<small>Format: universe,channels,value. Example: 1,1-10,255. Channels support ranges and comma lists.</small>");
  appendHtmlRaw("<form method='POST' action='/dmx_presets'>");
  appendHtml(String("<input type='hidden' name='preset' value='") + String(preset) + "'>");
  appendHtml(String("<input type='hidden' name='u' value='") + String(universe) + "'>");
  appendHtml(String("<input type='hidden' name='live_src' value='") + String(liveSourceMode) + "'>");
  appendHtml(String("<input type='hidden' name='lu_from' value='") + String(liveFrom) + "'>");
  appendHtml(String("<input type='hidden' name='lu_to' value='") + String(liveTo) + "'>");
  if (liveNonZeroOnly) appendHtmlRaw("<input type='hidden' name='live_nonzero' value='1'>");
  appendHtml(String("<textarea name='preset_text' placeholder='1,1-10,255&#10;2,1,128'>") + htmlEscape(currentText) + "</textarea><br>");
  appendHtmlRaw("<button type='submit' name='action' value='save' class='primary'>Save Text</button> ");
  appendHtmlRaw("<button type='submit' name='action' value='clear'>Clear Entire Preset</button>");
  appendHtmlRaw("</form>");
  appendHtmlRaw("</details>");

  appendHtmlRaw("</div></body></html>");
  flushHtml();
  server.sendContent("");
}

static void handleSendTestOsc() {
  if (!ensureAuthenticated()) return;
  String addr = "/test";
  String type = "int";
  String value = "1";
  uint8_t dev = config.testDevice;
  if (server.hasArg("test_addr")) {
    addr = normalizeOscAddress(server.arg("test_addr"));
  }
  if (server.hasArg("test_type")) {
    type = server.arg("test_type");
  }
  if (server.hasArg("test_value")) {
    value = server.arg("test_value");
  }
  if (server.hasArg("test_dev")) {
    int v = server.arg("test_dev").toInt();
    if (v >= 0 && v < MAX_OSC_DEVICES) dev = static_cast<uint8_t>(v);
  }
  IPAddress ip;
  uint16_t port;
  if (!getOscTarget(dev, ip, port)) {
    server.send(400, "text/plain", "No OSC device enabled.");
    return;
  }
  if (type == "string") {
    sendOscStringTo(ip, port, addr.c_str(), value.c_str());
  } else if (type == "float") {
    sendOscFloatTo(ip, port, addr.c_str(), value.toFloat());
  } else {
    sendOscIntTo(ip, port, addr.c_str(), value.toInt());
  }
  addLog(String("Test OSC sent to ") + addr);
  server.send(200, "text/plain", "Sent.");
}

static void handleSendTestUdp() {
  if (!ensureAuthenticated()) return;
  String payload = server.hasArg("payload") ? server.arg("payload") : "";
  if (payload.length() == 0) {
    server.send(400, "text/plain", "Missing payload");
    return;
  }
  int dev = server.hasArg("test_dev") ? server.arg("test_dev").toInt() : 0;
  if (dev < 0 || dev >= MAX_OSC_DEVICES) dev = 0;
  IPAddress ip;
  uint16_t port;
  if (!getOscTarget(static_cast<uint8_t>(dev), ip, port)) {
    server.send(400, "text/plain", "No target");
    return;
  }
  sendUdpRawTo(ip, port, payload);
  server.send(200, "text/plain", "Sent.");
}

static void handleSendTestMqtt() {
  if (!ensureAuthenticated()) return;
  String topic = server.hasArg("topic") ? server.arg("topic") : "";
  String payload = server.hasArg("payload") ? server.arg("payload") : "";
  bool retain = server.hasArg("retain");
  topic.trim();
  if (topic.length() == 0) {
    server.send(400, "text/plain", "Missing topic");
    return;
  }
  if (!config.mqttEnabled) {
    server.send(400, "text/plain", "MQTT is disabled in settings.");
    return;
  }
  if (config.mqttHost.length() == 0 || config.mqttPort == 0) {
    server.send(400, "text/plain", "MQTT broker is not configured.");
    return;
  }
  if (!sendMqttMessage(topic, payload, retain)) {
    server.send(503, "text/plain", "MQTT publish failed.");
    return;
  }
  server.send(200, "text/plain", "Sent.");
}

static void handleSendTestGpio() {
  if (!ensureAuthenticated()) return;
  int gpin = server.hasArg("gpin") ? server.arg("gpin").toInt() : -1;
  int gmode = server.hasArg("gmode") ? server.arg("gmode").toInt() : GPIO_OUT_PULSE;
  uint32_t gpms = server.hasArg("gpms") ? static_cast<uint32_t>(server.arg("gpms").toInt()) : 100;
  bool ginv = server.hasArg("ginv");
  if (!isAllowedGpioOutputPin(gpin) || isReservedPin(gpin)) {
    server.send(400, "text/plain", "Invalid pin");
    return;
  }
  if (gmode < GPIO_OUT_HIGH || gmode > GPIO_OUT_PWM) gmode = GPIO_OUT_PULSE;
  if (gpms == 0) gpms = 100;
  SensorConfig::OutputConfig out;
  out.gpioPin = gpin;
  if (gmode == GPIO_OUT_LOW) {
    out.gpioMode = GPIO_OUT_HIGH;
    ginv = true;
  } else {
    out.gpioMode = static_cast<GpioOutMode>(gmode);
  }
  out.gpioPulseMs = gpms;
  out.gpioInvert = ginv;
  applyGpioOutput(out, 1.0f, false);
  server.send(200, "text/plain", "Sent.");
}

static void handleSendTestDmx() {
  if (!ensureAuthenticated()) return;

  SensorConfig defaults = defaultSensorConfig(0);
  SensorConfig::OutputConfig out = defaults.outputs[0];
  out.target = OUT_TARGET_DMX;
  if (server.hasArg("target")) {
    int tgt = server.arg("target").toInt();
    if (tgt == OUT_TARGET_DMX_PRESET) out.target = OUT_TARGET_DMX_PRESET;
  }

  if (server.hasArg("test_dev")) {
    int dev = server.arg("test_dev").toInt();
    if (dev >= 0 && dev < MAX_OSC_DEVICES) out.device = static_cast<uint8_t>(dev);
  }
  if (server.hasArg("dmxp")) out.dmxProtocol = sanitizeDmxProtocol(server.arg("dmxp").toInt());
  if (server.hasArg("dmxd")) out.dmxDest = sanitizeDmxDest(server.arg("dmxd").toInt());
  if (server.hasArg("dmxn")) out.dmxArtNet = sanitizeDmxArtNet(server.arg("dmxn").toInt());
  if (server.hasArg("dmxs")) out.dmxArtSubnet = sanitizeDmxArtSubnet(server.arg("dmxs").toInt());
  if (server.hasArg("dmxa")) out.dmxArtUniverse = sanitizeDmxArtUniverse(server.arg("dmxa").toInt());
  if (server.hasArg("dmxu")) {
    uint16_t u = static_cast<uint16_t>(server.arg("dmxu").toInt());
    if (u > 0) out.dmxUniverse = u;
  }
  if (server.hasArg("dmxc")) {
    out.dmxChannels = server.arg("dmxc");
    out.dmxChannels.trim();
    if (out.dmxChannels.length() == 0) out.dmxChannels = "1";
  }
  if (server.hasArg("dmxpr")) out.dmxPreset = sanitizeDmxPresetId(server.arg("dmxpr").toInt());
  if (server.hasArg("dmxfd")) out.dmxFadeMs = sanitizeDmxFadeMs(server.arg("dmxfd").toInt());
  if (server.hasArg("omode")) {
    out.outMode = sanitizeOutputMode(server.arg("omode").toInt());
    if (out.outMode == OUT_STRING) out.outMode = OUT_INT;
  }
  if (server.hasArg("omin")) out.outMin = server.arg("omin").toFloat();
  if (server.hasArg("omax")) out.outMax = server.arg("omax").toFloat();
  if (out.dmxProtocol == DMX_PROTO_ARTNET && server.hasArg("dmxn")) {
    // Explicit net/subnet/universe triple provided
    out.dmxUniverse = encodeArtNetPortAddressOneBased(out.dmxArtNet, out.dmxArtSubnet, out.dmxArtUniverse);
  } else {
    // Single universe number provided (quick-test panel) — decode it into the
    // Art-Net triple; previously this branch never ran for Art-Net, so every
    // Art-Net test went to universe 1 regardless of what the user entered
    decodeArtNetPortAddressOneBased(out.dmxUniverse, out.dmxArtNet, out.dmxArtSubnet, out.dmxArtUniverse);
  }

  if (out.target == OUT_TARGET_DMX_PRESET) {
    if (!triggerDmxPreset(out)) {
      server.send(400, "text/plain", "DMX preset test failed.");
      return;
    }
    server.send(200, "text/plain", "Sent.");
    return;
  }

  String kind = server.hasArg("kind") ? server.arg("kind") : "max";
  float testValue = (kind == "min" || kind == "bmin") ? out.outMin : out.outMax;
  if (!sendDmxOutput(out, testValue)) {
    server.send(400, "text/plain", "DMX send failed (check protocol toggle and destination).");
    return;
  }
  server.send(200, "text/plain", "Sent.");
}

static void doFactoryReset(bool respond) {
  addLog("Factory reset requested. Clearing settings.");
  prefs.begin("osc", false);
  prefs.clear();
  prefs.end();
  if (respond) {
    server.send(200, "text/plain", "Factory reset. Rebooting...");
  }
  delay(200);
  ESP.restart();
}

static void handleReset() {
  if (!ensureAuthenticated()) return;
  doFactoryReset(true);
}

static void handleReboot() {
  if (!ensureAuthenticated()) return;
  server.send(200, "text/plain", "Rebooting...");
  addLog("Reboot requested.");
  delay(200);
  ESP.restart();
}

static void handleScanNetworks() {
  if (!ensureAuthenticated()) return;
  server.sendHeader("Access-Control-Allow-Origin", "*");
  // Use async scan to avoid blocking the web server
  int n = WiFi.scanComplete();
  if (n == WIFI_SCAN_FAILED) {
    WiFi.scanNetworks(true, true);  // async, show hidden
    server.send(200, "application/json", "[]");
    return;
  }
  if (n == WIFI_SCAN_RUNNING) {
    server.send(200, "application/json", "[]");
    return;
  }
  String json = "[";
  bool first = true;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) continue;
    if (!first) json += ",";
    first = false;
    json += "{\"ssid\":\"" + jsonEscape(ssid) + "\",\"rssi\":" + String(WiFi.RSSI(i)) + ",\"enc\":" + String(WiFi.encryptionType(i)) + "}";
  }
  json += "]";
  WiFi.scanDelete();
  server.send(200, "application/json", json);
}

static void handleLogsPage() {
  if (!ensureAuthenticated()) return;
  String html;
  html.reserve(1400);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>StageMod Logs</title>";
  html += "<style>body{font-family:'Avenir Next','Trebuchet MS','Segoe UI',sans-serif;background:#f4f1ea;color:#1a1a1a;padding:20px;}";
  html += ".card{background:#fff;border:1px solid #ddd;border-radius:12px;padding:16px;max-width:820px;}";
  html += "pre{background:#111;color:#e6e6e6;padding:12px;border-radius:10px;white-space:pre-wrap;min-height:260px;}";
  html += "a{color:#0b6b6f;}";
  html += "@media(prefers-color-scheme:dark){body{background:#161616;color:#e2e2e2;}.card{background:#222;border-color:#383838;}a{color:#0d9488;}}";
  html += "</style></head><body>";
  html += "<div class='card'><h2>Serial Log</h2>";
  html += "<p><a href='/'>Back to settings</a></p>";
  html += "<pre id='logbox'>Loading...</pre></div>";
  html += "<script>";
  html += "async function refresh(){try{const r=await fetch('/logs.txt');const t=await r.text();";
  html += "document.getElementById('logbox').textContent=t||'(no logs yet)';}catch(e){}}";
  html += "refresh();setInterval(refresh,1000);";
  html += "</script></body></html>";
  server.send(200, "text/html", html);
}

static void handleLogsText() {
  if (!ensureAuthenticated()) return;
  server.send(200, "text/plain", buildLogText());
}

static void handleUpdatePage() {
  if (!ensureAuthenticated()) return;
  String html;
  html.reserve(1800);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>StageMod Firmware Update</title>";
  html += "<style>*{box-sizing:border-box;}body{font-family:system-ui,sans-serif;background:#1e1e1e;color:#e2e2e2;padding:20px;margin:0;}";
  html += ".card{background:#2a2a2a;border:1px solid #383838;border-radius:12px;padding:20px;max-width:480px;margin:20px auto;}";
  html += "h2{color:#0d9488;margin-top:0;}";
  html += "button{background:#0d9488;color:#fff;border:none;padding:10px 20px;border-radius:8px;font-size:16px;cursor:pointer;width:100%;}";
  html += "input[type=file]{width:100%;padding:8px;margin:8px 0 16px;border:1px solid #444;border-radius:8px;background:#1a1a1a;color:#e2e2e2;}";
  html += "#msg{margin-top:16px;padding:12px;border-radius:8px;background:#1a1a1a;text-align:center;display:none;}";
  html += "a{color:#0d9488;}</style></head><body><div class='card'>";
  html += "<h2>Firmware Update</h2>";
  html += "<p style='color:#999;'>Upload a .bin file. Internet is not required.</p>";
  html += "<form method='POST' action='/update' enctype='multipart/form-data' target='uf' id='f'>";
  html += "<input type='file' name='update' accept='.bin' id='file'>";
  html += "<button type='submit' id='btn'>Install Update</button>";
  html += "</form>";
  html += "<iframe name='uf' style='display:none' id='uf'></iframe>";
  html += "<div id='msg'></div>";
  html += "<p><a href='/'>Back</a></p>";
  html += "</div><script>";
  html += "document.getElementById('f').onsubmit=function(){";
  html += "if(!document.getElementById('file').files[0]){alert('Select a .bin file');return false;}";
  html += "document.getElementById('btn').disabled=true;";
  html += "document.getElementById('btn').textContent='Uploading...';";
  html += "document.getElementById('msg').style.display='block';";
  html += "document.getElementById('msg').textContent='Uploading firmware — do not close this page...';";
  html += "return true;};";
  html += "document.getElementById('uf').onload=function(){";
  html += "document.getElementById('msg').textContent='Update complete! Rebooting...';";
  html += "document.getElementById('btn').textContent='Done';";
  html += "setTimeout(function(){window.location.replace('/');},8000);};";
  html += "</script></body></html>";
  server.send(200, "text/html", html);
}

static void handleUpdateFinished() {
  if (!ensureAuthenticated()) return;
  bool ok = !Update.hasError();
  server.sendHeader("Connection", "close");
  if (ok) {
    server.send(200, "text/html", "<html><body>OK</body></html>");
  } else {
    String msg = String("Update failed. ") + Update.errorString();
    server.send(200, "text/plain", msg);
  }
  addLog(ok ? "Firmware update OK. Rebooting." : "Firmware update failed.");
  delay(200);
  if (ok) {
    ESP.restart();
  }
}

// Auth must be checked in the upload callback, not just the finished handler:
// the callback runs while the body streams in, so without this an
// unauthenticated POST would write and arm the new boot partition before
// handleUpdateFinished ever gets to reject it.
static bool otaAuthorized = false;

static void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    otaAuthorized = checkSameOrigin() &&
                    (!isPasswordRequired() ||
                     server.authenticate(config.username.c_str(), config.password.c_str()));
    if (!otaAuthorized) {
      addLog("Firmware update rejected: not authenticated.");
      return;
    }
    Serial.println("OTA: starting, free heap: " + String(ESP.getFreeHeap()) +
                   ", largest block: " + String(ESP.getMaxAllocHeap()));
    // Free resources for OTA — device reboots after anyway
    MDNS.end();
    if (dmxInData) { free(dmxInData); dmxInData = nullptr; }
    oscUdp.stop();
    artnetInUdp.stop();
    sacnInUdp.stop();
    mqttClient.disconnect();
    Serial.println("OTA: freed resources, heap now: " + String(ESP.getFreeHeap()) +
                   ", largest block: " + String(ESP.getMaxAllocHeap()));
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
      addLog(String("Firmware update begin failed: ") + Update.errorString());
      Serial.println("OTA begin failed: " + String(Update.errorString()));
    } else {
      addLog("Firmware update upload started.");
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (!otaAuthorized) return;
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
      Serial.println("OTA write failed at " + String(upload.totalSize) + " bytes: " + String(Update.errorString()) +
                     ", heap: " + String(ESP.getFreeHeap()) + ", largest block: " + String(ESP.getMaxAllocHeap()));
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!otaAuthorized) return;
    Serial.println("OTA: upload done, " + String(upload.totalSize) + " bytes, heap: " + String(ESP.getFreeHeap()));
    if (!Update.end(true)) {
      addLog(String("Firmware update end failed: ") + Update.errorString());
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    Serial.println("OTA: aborted");
  }
}

// =====================================================================
// JSON REST API
// =====================================================================

static void apiCors() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET,POST,DELETE,OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

static void apiJson(int code, const String& body) {
  apiCors();
  server.send(code, "application/json", body);
}

static void apiError(int code, const String& msg) {
  apiJson(code, "{\"error\":\"" + jsonEscape(msg) + "\"}");
}

// Parse trailing integer from URI like /api/trigger/3 → 3
static int apiPathIndex() {
  String uri = server.uri();
  int slash = uri.lastIndexOf('/');
  if (slash < 0) return -1;
  String num = uri.substring(slash + 1);
  if (num.length() == 0 || !isDigit(num[0])) return -1;
  int n = num.toInt();
  if (n < 0 || n >= MAX_TRIGGERS) return -1;
  return n;
}

// Serialize one trigger into an existing JsonObject
static void serializeTriggerTo(int i, JsonObject so) {
  const SensorConfig& s = config.sensors[i];
  so["index"] = i;
  so["enabled"] = s.enabled;
  so["name"] = s.name;
  so["source"] = static_cast<int>(s.source);
  so["oscInAddress"] = s.oscInAddress;
  so["oscInArgType"] = s.oscInArgType;
  so["oscInMatchMode"] = s.oscInMatchMode;
  so["oscInValue"] = s.oscInValue;
  so["oscInMin"] = s.oscInMin;
  so["oscInMax"] = s.oscInMax;
  so["oscInString"] = s.oscInString;
  so["dmxInUniverse"] = s.dmxInUniverse;
  so["dmxInChannel"] = s.dmxInChannel;
  so["dmxInMode"] = s.dmxInMode;
  so["dmxInThreshold"] = s.dmxInThreshold;
  so["dmxInRangeMin"] = s.dmxInRangeMin;
  so["dmxInRangeMax"] = s.dmxInRangeMax;
  so["type"] = static_cast<int>(s.type);
  so["pin"] = s.pin;
  so["encClkPin"] = s.encClkPin;
  so["encDtPin"] = s.encDtPin;
  so["encSwPin"] = s.encSwPin;
  so["encAppendSign"] = s.encAppendSign;
  so["encSignArg"] = s.encSignArg;
  so["encInvert"] = s.encInvert;
  so["encSteps"] = s.encSteps;
  so["invert"] = s.invert;
  so["activeHigh"] = s.activeHigh;
  so["pullup"] = s.pullup;
  so["hcTrigPin"] = s.hcTrigPin;
  so["hcEchoPin"] = s.hcEchoPin;
  so["hcMinCm"] = s.hcMinCm;
  so["hcMaxCm"] = s.hcMaxCm;
  so["cooldownEnabled"] = s.cooldownEnabled;
  so["cooldownMs"] = s.cooldownMs;
  so["conditionEnabled"] = s.conditionEnabled;
  so["conditionVar"] = s.conditionVar;
  so["conditionOp"] = s.conditionOp;
  so["conditionValue"] = s.conditionValue;
  so["outputCount"] = s.outputCount;
  JsonArray outs = so.createNestedArray("outputs");
  for (int o = 0; o < s.outputCount && o < MAX_OUTPUTS_PER_TRIGGER; o++) {
    const SensorConfig::OutputConfig& out = s.outputs[o];
    JsonObject oo = outs.createNestedObject();
    oo["index"] = o;
    oo["target"] = static_cast<int>(out.target);
    oo["delayMs"] = out.delayMs;
    oo["device"] = out.device;
    oo["oscAddress"] = out.oscAddress;
    oo["buttonAddress"] = out.buttonAddress;
    oo["outMin"] = out.outMin;
    oo["outMax"] = out.outMax;
    oo["outMode"] = static_cast<int>(out.outMode);
    oo["buttonOutMin"] = out.buttonOutMin;
    oo["buttonOutMax"] = out.buttonOutMax;
    oo["buttonOutMode"] = static_cast<int>(out.buttonOutMode);
    oo["sendMinOnRelease"] = out.sendMinOnRelease;
    oo["udpPayload"] = out.udpPayload;
    oo["gpioPin"] = out.gpioPin;
    oo["gpioMode"] = static_cast<int>(out.gpioMode);
    oo["gpioPulseMs"] = out.gpioPulseMs;
    oo["gpioInvert"] = out.gpioInvert;
    oo["httpMethod"] = static_cast<int>(out.httpMethod);
    oo["httpUrl"] = out.httpUrl;
    oo["httpBody"] = out.httpBody;
    oo["httpIp"] = out.httpIp;
    oo["httpPort"] = out.httpPort;
    oo["httpDevice"] = out.httpDevice;
    oo["mqttTopic"] = out.mqttTopic;
    oo["mqttPayload"] = out.mqttPayload;
    oo["mqttRetain"] = out.mqttRetain;
    oo["dmxProtocol"] = out.dmxProtocol;
    oo["dmxDest"] = out.dmxDest;
    oo["dmxArtNet"] = out.dmxArtNet;
    oo["dmxArtSubnet"] = out.dmxArtSubnet;
    oo["dmxArtUniverse"] = out.dmxArtUniverse;
    oo["dmxUniverse"] = out.dmxUniverse;
    oo["dmxChannels"] = out.dmxChannels;
    oo["dmxPreset"] = out.dmxPreset;
    oo["dmxFadeMs"] = out.dmxFadeMs;
    oo["onString"] = out.onString;
    oo["offString"] = out.offString;
    oo["buttonOnString"] = out.buttonOnString;
    oo["buttonOffString"] = out.buttonOffString;
    oo["setVarName"] = out.setVarName;
    oo["setVarValue"] = out.setVarValue;
  }
  so["timeType"] = static_cast<int>(s.timeType);
  so["timeYear"] = s.timeYear;
  so["timeMonth"] = s.timeMonth;
  so["timeDay"] = s.timeDay;
  so["timeHour"] = s.timeHour;
  so["timeMinute"] = s.timeMinute;
  so["timeSecond"] = s.timeSecond;
  so["weeklyMask"] = s.weeklyMask;
  so["intervalSeconds"] = s.intervalSeconds;
}

// Apply a sensor JsonObject to config.sensors[idx]
static void applySensorFromJson(int idx, JsonObject so) {
  SensorConfig& s = config.sensors[idx];
  if (so.containsKey("enabled")) s.enabled = so["enabled"];
  if (so.containsKey("name")) s.name = String(so["name"].as<const char*>());
  if (so.containsKey("source")) s.source = static_cast<SensorSource>(so["source"].as<int>());
  if (so.containsKey("oscInAddress")) s.oscInAddress = String(so["oscInAddress"].as<const char*>());
  if (so.containsKey("oscInArgType")) s.oscInArgType = so["oscInArgType"];
  if (so.containsKey("oscInMatchMode")) s.oscInMatchMode = so["oscInMatchMode"];
  if (so.containsKey("oscInValue")) s.oscInValue = so["oscInValue"];
  if (so.containsKey("oscInMin")) s.oscInMin = so["oscInMin"];
  if (so.containsKey("oscInMax")) s.oscInMax = so["oscInMax"];
  if (so.containsKey("oscInString")) s.oscInString = String(so["oscInString"].as<const char*>());
  if (so.containsKey("dmxInUniverse")) { int v = so["dmxInUniverse"]; if (v >= 1 && v <= MAX_DMX_PRESET_UNIVERSE) s.dmxInUniverse = static_cast<uint16_t>(v); }
  if (so.containsKey("dmxInChannel")) { int v = so["dmxInChannel"]; if (v >= 1 && v <= 512) s.dmxInChannel = static_cast<uint16_t>(v); }
  if (so.containsKey("dmxInMode")) { int v = so["dmxInMode"]; if (v >= DMX_IN_TRIG_LEVEL && v <= DMX_IN_TRIG_RANGE) s.dmxInMode = static_cast<uint8_t>(v); }
  if (so.containsKey("dmxInThreshold")) { int v = so["dmxInThreshold"]; if (v >= 0 && v <= 255) s.dmxInThreshold = static_cast<uint8_t>(v); }
  if (so.containsKey("dmxInRangeMin")) { int v = so["dmxInRangeMin"]; if (v >= 0 && v <= 255) s.dmxInRangeMin = static_cast<uint8_t>(v); }
  if (so.containsKey("dmxInRangeMax")) { int v = so["dmxInRangeMax"]; if (v >= 0 && v <= 255) s.dmxInRangeMax = static_cast<uint8_t>(v); }
  if (so.containsKey("type")) s.type = static_cast<SensorType>(so["type"].as<int>());
  if (so.containsKey("pin")) s.pin = so["pin"];
  if (so.containsKey("encClkPin")) s.encClkPin = so["encClkPin"];
  if (so.containsKey("encDtPin")) s.encDtPin = so["encDtPin"];
  if (so.containsKey("encSwPin")) s.encSwPin = so["encSwPin"];
  if (so.containsKey("encAppendSign")) s.encAppendSign = so["encAppendSign"];
  if (so.containsKey("encSignArg")) s.encSignArg = so["encSignArg"];
  if (so.containsKey("encInvert")) s.encInvert = so["encInvert"];
  if (so.containsKey("encSteps")) s.encSteps = so["encSteps"];
  if (so.containsKey("invert")) s.invert = so["invert"];
  if (so.containsKey("activeHigh")) s.activeHigh = so["activeHigh"];
  if (so.containsKey("pullup")) s.pullup = so["pullup"];
  if (so.containsKey("hcTrigPin")) s.hcTrigPin = so["hcTrigPin"];
  if (so.containsKey("hcEchoPin")) s.hcEchoPin = so["hcEchoPin"];
  if (so.containsKey("hcMinCm")) s.hcMinCm = so["hcMinCm"];
  if (so.containsKey("hcMaxCm")) s.hcMaxCm = so["hcMaxCm"];
  if (so.containsKey("cooldownEnabled")) s.cooldownEnabled = so["cooldownEnabled"];
  if (so.containsKey("cooldownMs")) s.cooldownMs = so["cooldownMs"];
  if (so.containsKey("conditionEnabled")) s.conditionEnabled = so["conditionEnabled"];
  if (so.containsKey("conditionVar")) s.conditionVar = String(so["conditionVar"].as<const char*>());
  if (so.containsKey("conditionOp")) s.conditionOp = so["conditionOp"];
  if (so.containsKey("conditionValue")) s.conditionValue = so["conditionValue"];
  if (so.containsKey("outputCount")) {
    int oc = so["outputCount"];
    s.outputCount = static_cast<uint8_t>(constrain(oc, 1, MAX_OUTPUTS_PER_TRIGGER));
  }
  bool importedArtAddr[MAX_OUTPUTS_PER_TRIGGER] = {};
  JsonArray outs = so["outputs"].as<JsonArray>();
  if (!outs.isNull()) {
    for (JsonObject oo : outs) {
      int oidx = oo["index"] | -1;
      if (oidx < 0 || oidx >= MAX_OUTPUTS_PER_TRIGGER) continue;
      SensorConfig::OutputConfig& out = s.outputs[oidx];
      if (oo.containsKey("target")) out.target = sanitizeOutputTarget(oo["target"].as<int>());
      if (oo.containsKey("delayMs")) out.delayMs = oo["delayMs"];
      if (oo.containsKey("device")) out.device = oo["device"];
      if (oo.containsKey("oscAddress")) out.oscAddress = String(oo["oscAddress"].as<const char*>());
      if (oo.containsKey("buttonAddress")) out.buttonAddress = String(oo["buttonAddress"].as<const char*>());
      if (oo.containsKey("outMin")) out.outMin = oo["outMin"];
      if (oo.containsKey("outMax")) out.outMax = oo["outMax"];
      if (oo.containsKey("outMode")) out.outMode = static_cast<OutputMode>(oo["outMode"].as<int>());
      if (oo.containsKey("buttonOutMin")) out.buttonOutMin = oo["buttonOutMin"];
      if (oo.containsKey("buttonOutMax")) out.buttonOutMax = oo["buttonOutMax"];
      if (oo.containsKey("buttonOutMode")) out.buttonOutMode = static_cast<OutputMode>(oo["buttonOutMode"].as<int>());
      if (oo.containsKey("sendMinOnRelease")) out.sendMinOnRelease = oo["sendMinOnRelease"];
      if (oo.containsKey("udpPayload")) out.udpPayload = String(oo["udpPayload"].as<const char*>());
      if (oo.containsKey("gpioPin")) out.gpioPin = oo["gpioPin"];
      if (oo.containsKey("gpioMode")) out.gpioMode = static_cast<GpioOutMode>(oo["gpioMode"].as<int>());
      if (oo.containsKey("gpioPulseMs")) out.gpioPulseMs = oo["gpioPulseMs"];
      if (oo.containsKey("gpioInvert")) out.gpioInvert = oo["gpioInvert"];
      if (oo.containsKey("httpMethod")) out.httpMethod = static_cast<HttpMethod>(oo["httpMethod"].as<int>());
      if (oo.containsKey("httpUrl")) out.httpUrl = String(oo["httpUrl"].as<const char*>());
      if (oo.containsKey("httpBody")) out.httpBody = String(oo["httpBody"].as<const char*>());
      if (oo.containsKey("httpIp")) out.httpIp = String(oo["httpIp"].as<const char*>());
      if (oo.containsKey("httpPort")) out.httpPort = oo["httpPort"];
      if (oo.containsKey("httpDevice")) out.httpDevice = oo["httpDevice"];
      if (oo.containsKey("mqttTopic")) out.mqttTopic = String(oo["mqttTopic"].as<const char*>());
      if (oo.containsKey("mqttPayload")) out.mqttPayload = String(oo["mqttPayload"].as<const char*>());
      if (oo.containsKey("mqttRetain")) out.mqttRetain = oo["mqttRetain"];
      if (oo.containsKey("dmxProtocol")) out.dmxProtocol = sanitizeDmxProtocol(oo["dmxProtocol"].as<int>());
      if (oo.containsKey("dmxDest")) out.dmxDest = sanitizeDmxDest(oo["dmxDest"].as<int>());
      if (oo.containsKey("dmxArtNet")) out.dmxArtNet = sanitizeDmxArtNet(oo["dmxArtNet"].as<int>());
      if (oo.containsKey("dmxArtSubnet")) out.dmxArtSubnet = sanitizeDmxArtSubnet(oo["dmxArtSubnet"].as<int>());
      if (oo.containsKey("dmxArtUniverse")) out.dmxArtUniverse = sanitizeDmxArtUniverse(oo["dmxArtUniverse"].as<int>());
      importedArtAddr[oidx] = oo.containsKey("dmxArtNet") || oo.containsKey("dmxArtSubnet") || oo.containsKey("dmxArtUniverse");
      if (oo.containsKey("dmxUniverse")) { uint16_t u = oo["dmxUniverse"]; if (u > 0) out.dmxUniverse = u; }
      if (oo.containsKey("dmxChannels")) out.dmxChannels = String(oo["dmxChannels"].as<const char*>());
      if (oo.containsKey("dmxPreset")) out.dmxPreset = sanitizeDmxPresetId(oo["dmxPreset"].as<int>());
      if (oo.containsKey("dmxFadeMs")) out.dmxFadeMs = sanitizeDmxFadeMs(oo["dmxFadeMs"].as<int>());
      if (oo.containsKey("onString")) out.onString = String(oo["onString"].as<const char*>());
      if (oo.containsKey("offString")) out.offString = String(oo["offString"].as<const char*>());
      if (oo.containsKey("buttonOnString")) out.buttonOnString = String(oo["buttonOnString"].as<const char*>());
      if (oo.containsKey("buttonOffString")) out.buttonOffString = String(oo["buttonOffString"].as<const char*>());
      if (oo.containsKey("setVarName")) out.setVarName = String(oo["setVarName"].as<const char*>());
      if (oo.containsKey("setVarValue")) out.setVarValue = oo["setVarValue"];
      if ((out.target == OUT_TARGET_DMX || out.target == OUT_TARGET_DMX_PRESET) && out.outMode == OUT_STRING) out.outMode = OUT_INT;
    }
  }
  if (so.containsKey("timeType")) s.timeType = static_cast<TimeTriggerType>(so["timeType"].as<int>());
  if (so.containsKey("timeYear")) s.timeYear = so["timeYear"];
  if (so.containsKey("timeMonth")) s.timeMonth = so["timeMonth"];
  if (so.containsKey("timeDay")) s.timeDay = so["timeDay"];
  if (so.containsKey("timeHour")) s.timeHour = so["timeHour"];
  if (so.containsKey("timeMinute")) s.timeMinute = so["timeMinute"];
  if (so.containsKey("timeSecond")) s.timeSecond = so["timeSecond"];
  if (so.containsKey("weeklyMask")) s.weeklyMask = so["weeklyMask"];
  if (so.containsKey("intervalSeconds")) s.intervalSeconds = so["intervalSeconds"];
  // Pin validation
  if (!isAllowedDigitalPin(s.pin) && s.type != SENSOR_ANALOG) s.pin = defaultSensorPin(idx);
  if (!isAllowedAnalogPin(s.pin) && s.type == SENSOR_ANALOG) s.pin = defaultSensorPin(idx);
  if (!isAllowedDigitalPin(s.encClkPin)) s.encClkPin = 13;
  if (!isAllowedDigitalPin(s.encDtPin)) s.encDtPin = 16;
  if (!isAllowedDigitalPin(s.encSwPin)) s.encSwPin = 33;
  if (!isAllowedDigitalPin(s.hcTrigPin)) s.hcTrigPin = 13;
  if (!isAllowedDigitalPin(s.hcEchoPin)) s.hcEchoPin = 16;
  if (s.source != SRC_TIME && s.source != SRC_OSC && s.source != SRC_DMX) s.source = SRC_SENSORS;
  if (s.oscInAddress.length() == 0) s.oscInAddress = "/trigger/*";
  if (s.oscInArgType > OSC_IN_ARG_STRING) s.oscInArgType = OSC_IN_ARG_ANY;
  if (s.oscInMatchMode > OSC_IN_MATCH_RANGE) s.oscInMatchMode = OSC_IN_MATCH_ANY;
  if (s.oscInString.length() == 0) s.oscInString = "go";
  for (int o = 0; o < s.outputCount; o++) {
    if (s.outputs[o].dmxUniverse == 0) s.outputs[o].dmxUniverse = 1;
    if (importedArtAddr[o] && s.outputs[o].dmxProtocol == DMX_PROTO_ARTNET)
      s.outputs[o].dmxUniverse = encodeArtNetPortAddressOneBased(s.outputs[o].dmxArtNet, s.outputs[o].dmxArtSubnet, s.outputs[o].dmxArtUniverse);
    if (!importedArtAddr[o])
      decodeArtNetPortAddressOneBased(s.outputs[o].dmxUniverse, s.outputs[o].dmxArtNet, s.outputs[o].dmxArtSubnet, s.outputs[o].dmxArtUniverse);
    s.outputs[o].dmxArtNet = sanitizeDmxArtNet(s.outputs[o].dmxArtNet);
    s.outputs[o].dmxArtSubnet = sanitizeDmxArtSubnet(s.outputs[o].dmxArtSubnet);
    s.outputs[o].dmxArtUniverse = sanitizeDmxArtUniverse(s.outputs[o].dmxArtUniverse);
    if (s.outputs[o].dmxChannels.length() == 0) s.outputs[o].dmxChannels = "1";
    s.outputs[o].dmxPreset = sanitizeDmxPresetId(s.outputs[o].dmxPreset);
  }
}

// GET /api/config — full config JSON
static void handleApiGetConfig() {
  if (!ensureAuthenticated()) return;
  apiCors();
  String json = exportConfigJson();
  server.setContentLength(json.length());
  server.send(200, "application/json", json);
}

// POST /api/config — save settings (non-trigger fields)
static void handleApiPostConfig() {
  if (!ensureAuthenticated()) return;
  String body = server.arg("plain");
  if (body.length() == 0) { apiError(400, "Empty body"); return; }
  // Full exports run up to ~32KB; must be able to parse what we emit
  DynamicJsonDocument doc(32768);
  if (deserializeJson(doc, body) != DeserializationError::Ok) { apiError(400, "Invalid JSON"); return; }
  String err;
  if (!applyConfigFromJson(doc.as<JsonObject>(), err)) { apiError(400, err); return; }
  saveConfig();
  // Apply runtime side effects so toggles take effect without a reboot —
  // previously only the legacy /settings form did this, so e.g. enabling
  // Art-Net input from the SPA never actually opened the listener port.
  applyTimeConfig();
  configureUdpListener();
  mqttReconnectRequested = true;
  apiJson(200, "{\"ok\":true}");
}

// GET /api/status
static void handleApiGetStatus() {
  apiCors();
  DynamicJsonDocument doc(512);
  doc["firmware"] = FIRMWARE_VERSION;
  doc["ip"] = getLocalIp().toString();
  doc["hostname"] = config.hostname;
  doc["uptime"] = millis() / 1000;
  doc["apMode"] = wifiApMode;
  doc["netMode"] = static_cast<int>(config.netMode);
  doc["time"] = formatTimeLocal();
  doc["date"] = formatDateLocal();
  String out;
  serializeJson(doc, out);
  apiJson(200, out);
}

// GET /api/live — real-time sensor values and trigger states
static void handleApiGetLive() {
  apiCors();
  DynamicJsonDocument doc(4096);
  doc["uptime"] = millis() / 1000;
  doc["heap"] = ESP.getFreeHeap();
  doc["ip"] = getLocalIp().toString();
  doc["apMode"] = wifiApMode;
  doc["time"] = formatTimeLocal();
  JsonArray triggers = doc.createNestedArray("triggers");
  for (int i = 0; i < config.triggerCount; i++) {
    int idx = config.triggerOrder[i];
    const SensorConfig& s = config.sensors[idx];
    if (!s.enabled) continue;
    JsonObject t = triggers.createNestedObject();
    t["index"] = idx;
    t["name"] = s.name;
    t["source"] = static_cast<int>(s.source);
    t["type"] = static_cast<int>(s.type);
    // Current sensor reading
    for (int o = 0; o < s.outputCount && o < MAX_OUTPUTS_PER_TRIGGER; o++) {
      if (o == 0) {
        t["lastInt"] = lastInt[idx][o];
        t["lastFloat"] = serialized(String(lastFloat[idx][o], 3));
        t["lastBool"] = lastBool[idx][o];
      }
    }
    t["toggleState"] = toggleState[idx];
    t["cooldownActive"] = (s.cooldownEnabled && lastButtonTriggerMs[idx] > 0 && (millis() - lastButtonTriggerMs[idx]) < s.cooldownMs);
  }
  JsonObject vars = doc.createNestedObject("vars");
  for (int i = 0; i < globalVarCount; i++) {
    vars[globalVars[i].name] = serialized(String(globalVars[i].value, 3));
  }
  String out;
  serializeJson(doc, out);
  apiJson(200, out);
}

// GET /api/trigger/N
static void handleApiGetTrigger() {
  if (!ensureAuthenticated()) return;
  int n = apiPathIndex();
  if (n < 0) { apiError(400, "Invalid trigger index"); return; }
  DynamicJsonDocument doc(8192);
  JsonObject obj = doc.to<JsonObject>();
  serializeTriggerTo(n, obj);
  String out;
  serializeJson(doc, out);
  apiJson(200, out);
}

// POST /api/trigger/N — body is a full trigger JSON object
static void handleApiPostTrigger() {
  if (!ensureAuthenticated()) return;
  int n = apiPathIndex();
  if (n < 0) { apiError(400, "Invalid trigger index"); return; }
  String body = server.arg("plain");
  if (body.length() == 0) { apiError(400, "Empty body"); return; }
  DynamicJsonDocument doc(8192);
  if (deserializeJson(doc, body) != DeserializationError::Ok) { apiError(400, "Invalid JSON"); return; }
  applySensorFromJson(n, doc.as<JsonObject>());
  saveConfig();
  applySensorPinModes();
  setupEncoderCounters();
  // Echo back what was saved so the UI can confirm
  DynamicJsonDocument resp(8192);
  JsonObject ro = resp.to<JsonObject>();
  serializeTriggerTo(n, ro);
  String out;
  serializeJson(resp, out);
  apiJson(200, out);
}

// DELETE /api/trigger/N — remove from order and disable
static void handleApiDeleteTrigger() {
  if (!ensureAuthenticated()) return;
  int n = apiPathIndex();
  if (n < 0) { apiError(400, "Invalid trigger index"); return; }
  config.sensors[n].enabled = false;
  uint8_t newOrder[MAX_TRIGGERS];
  uint8_t newCount = 0;
  for (int i = 0; i < config.triggerCount; i++) {
    if (config.triggerOrder[i] != static_cast<uint8_t>(n))
      newOrder[newCount++] = config.triggerOrder[i];
  }
  config.triggerCount = newCount;
  memcpy(config.triggerOrder, newOrder, newCount);
  clearSensorKeys(n);
  saveConfig();
  apiJson(200, "{\"ok\":true}");
}

// POST /api/triggers/add — create new empty trigger, return it
static void handleApiAddTrigger() {
  if (!ensureAuthenticated()) return;
  if (config.triggerCount >= MAX_TRIGGERS) { apiError(400, "Max triggers reached"); return; }
  int idx = findUnusedTrigger();
  if (idx < 0) { apiError(400, "No free trigger slot"); return; }
  config.sensors[idx] = defaultSensorConfig(idx);
  config.sensors[idx].enabled = true;
  config.triggerOrder[config.triggerCount++] = static_cast<uint8_t>(idx);
  saveConfig();
  DynamicJsonDocument doc(8192);
  JsonObject obj = doc.to<JsonObject>();
  serializeTriggerTo(idx, obj);
  String out;
  serializeJson(doc, out);
  apiJson(200, out);
}

// POST /api/triggers/reorder — body: {"order":[2,0,1,...]}
static void handleApiReorderTriggers() {
  if (!ensureAuthenticated()) return;
  String body = server.arg("plain");
  DynamicJsonDocument doc(512);
  if (deserializeJson(doc, body) != DeserializationError::Ok) { apiError(400, "Invalid JSON"); return; }
  JsonArray arr = doc["order"].as<JsonArray>();
  if (arr.isNull()) { apiError(400, "Missing order array"); return; }
  uint8_t newCount = 0;
  for (JsonVariant v : arr) {
    int idx = v.as<int>();
    if (idx >= 0 && idx < MAX_TRIGGERS && newCount < MAX_TRIGGERS)
      config.triggerOrder[newCount++] = static_cast<uint8_t>(idx);
  }
  config.triggerCount = newCount;
  saveConfig();
  apiJson(200, "{\"ok\":true}");
}

// POST /api/devices/add — add new network client device
static void handleApiAddDevice() {
  if (!ensureAuthenticated()) return;
  if (config.oscDeviceCount >= MAX_OSC_DEVICES) { apiError(400, "Max devices reached"); return; }
  int idx = findUnusedOscDevice();
  if (idx < 0) { apiError(400, "No free device slot"); return; }
  OscDevice d;
  d.enabled = true;
  d.name = String("Device ") + String(idx + 1);
  d.ip = config.clientIp;
  d.port = config.clientPort;
  d.hbEnabled = false;
  d.hbAddress = "/heartbeat";
  d.hbMs = 5000;
  config.oscDevices[idx] = d;
  config.oscDeviceOrder[config.oscDeviceCount++] = static_cast<uint8_t>(idx);
  saveConfig();
  apiJson(200, "{\"ok\":true}");
}

// POST /api/devices/remove — body: {"index": N}
static void handleApiRemoveDevice() {
  if (!ensureAuthenticated()) return;
  String body = server.arg("plain");
  DynamicJsonDocument doc(128);
  if (deserializeJson(doc, body) != DeserializationError::Ok) { apiError(400, "Invalid JSON"); return; }
  int idx = doc["index"] | -1;
  if (idx <= 0 || idx >= MAX_OSC_DEVICES) { apiError(400, "Invalid device index"); return; }
  config.oscDevices[idx].enabled = false;
  uint8_t newOrder[MAX_OSC_DEVICES];
  uint8_t newCount = 0;
  for (int i = 0; i < config.oscDeviceCount; i++) {
    if (config.oscDeviceOrder[i] != static_cast<uint8_t>(idx))
      newOrder[newCount++] = config.oscDeviceOrder[i];
  }
  config.oscDeviceCount = newCount;
  memcpy(config.oscDeviceOrder, newOrder, newCount);
  clearDeviceKeys(idx);
  saveConfig();
  apiJson(200, "{\"ok\":true}");
}

// POST /api/trigger/N/fire — test-fire a trigger's outputs at full level.
// Runs the real dispatch path (including Prewait) so a pre-show checkout
// exercises exactly what the show will. Reboot/reset outputs are skipped.
static void handleApiFireTrigger(int n) {
  if (!ensureAuthenticated()) return;
  if (n < 0 || n >= MAX_TRIGGERS) { apiError(400, "Invalid trigger index"); return; }
  const SensorConfig& s = config.sensors[n];
  addLog(String("Test fire: ") + s.name);
  for (int o = 0; o < s.outputCount; o++) {
    const SensorConfig::OutputConfig& out = s.outputs[o];
    if (out.target == OUT_TARGET_REBOOT || out.target == OUT_TARGET_RESET_COOLDOWNS) continue;
    queueOrSendOutput(s, out, out.oscAddress, 1.0f);
  }
  apiJson(200, "{\"ok\":true}");
}

// Dispatch /api/trigger/N by HTTP method
static void handleApiTriggerDispatch() {
  if (server.method() == HTTP_OPTIONS) { apiCors(); server.send(204); return; }
  String uri = server.uri();
  if (uri.endsWith("/fire")) {
    if (server.method() != HTTP_POST) { apiError(405, "Method not allowed"); return; }
    int start = strlen("/api/trigger/");
    int end = uri.indexOf('/', start);
    int n = (end > start) ? uri.substring(start, end).toInt() : -1;
    handleApiFireTrigger(n);
    return;
  }
  if (server.method() == HTTP_GET)    { handleApiGetTrigger();    return; }
  if (server.method() == HTTP_POST)   { handleApiPostTrigger();   return; }
  if (server.method() == HTTP_DELETE) { handleApiDeleteTrigger(); return; }
  apiError(405, "Method not allowed");
}

// Lightweight handler — only saves network fields, doesn't touch protocols/triggers
static void handleSetupSave() {
  if (!ensureAuthenticated()) return;
  if (server.hasArg("net_mode")) config.netMode = sanitizeNetworkMode(server.arg("net_mode").toInt());
  if (server.hasArg("hostname")) config.hostname = sanitizeHostname(server.arg("hostname"));
  if (server.hasArg("wifi_ssid") && server.arg("wifi_ssid").length() > 0) config.wifiSsid = server.arg("wifi_ssid");
  if (server.hasArg("wifi_pass") && server.arg("wifi_pass").length() > 0) config.wifiPass = server.arg("wifi_pass");
  config.useStatic = !server.hasArg("dhcp_enabled");
  if (server.hasArg("static_ip")) { IPAddress p; if (parseIpString(server.arg("static_ip"), p)) config.staticIp = p; }
  if (server.hasArg("gateway")) { IPAddress p; if (parseIpString(server.arg("gateway"), p)) config.gateway = p; }
  if (server.hasArg("subnet")) { IPAddress p; if (parseIpString(server.arg("subnet"), p)) config.subnet = p; }
  saveConfig();
  addLog("Setup saved. Rebooting...");
  server.send(200, "text/html",
    "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
    "<style>body{font-family:system-ui,sans-serif;background:#1e1e1e;color:#e2e2e2;text-align:center;padding:40px;}"
    "h2{color:#0d9488;}</style></head><body>"
    "<h2>Saved!</h2><p>StageMod is connecting to your network.<br>This may take up to 30 seconds.</p>"
    "<p>Try: <a href='http://" + config.hostname + ".local' style='color:#0d9488'>"
    "http://" + config.hostname + ".local</a></p>"
    "</body></html>");
  delay(500);
  ESP.restart();
}

static void serveSetupPage() {
  if (!ensureAuthenticated()) return;
  // Synchronous WiFi scan — default 300ms/channel ≈ 4 seconds total
  int n = WiFi.scanNetworks(false, true);
  String netOptions;
  for (int i = 0; i < n; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) continue;
    netOptions += "<div class='net' onclick=\"document.getElementById('ssid').value='" + htmlEscape(ssid) + "'\">"
      + htmlEscape(ssid) + "<span class='sig'>" + String(WiFi.RSSI(i)) + " dBm</span></div>";
  }
  WiFi.scanDelete();
  if (n <= 0) netOptions = "<div style='color:#888'>No networks found — try refreshing</div>";

  String html = "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1,user-scalable=no'>"
    "<title>StageMod Setup</title><style>"
    "*{box-sizing:border-box;}"
    "body{font-family:-apple-system,system-ui,sans-serif;background:#1e1e1e;color:#e2e2e2;padding:0;margin:0;}"
    ".card{background:#2a2a2a;border-radius:12px;padding:16px;max-width:420px;margin:12px auto;}"
    "h2{color:#0d9488;margin:0 0 4px;font-size:22px;} .sub{color:#999;font-size:13px;margin:0 0 16px;}"
    "input,select{width:100%;padding:10px;margin:4px 0 12px;border:1px solid #444;border-radius:8px;background:#1a1a1a;color:#e2e2e2;font-size:15px;-webkit-appearance:none;}"
    "select{background:#1a1a1a url(\"data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='12' height='12' fill='%23999'%3E%3Cpath d='M2 4l4 4 4-4'/%3E%3C/svg%3E\") no-repeat right 10px center;padding-right:30px;}"
    "button,.btn{background:#0d9488;color:#fff;border:none;padding:12px;border-radius:8px;font-size:16px;font-weight:600;cursor:pointer;width:100%;display:block;text-align:center;text-decoration:none;margin:8px 0;}"
    ".btn-sec{background:#444;font-size:13px;padding:8px;margin:4px 0 12px;}"
    "label{font-size:13px;color:#999;display:block;margin-bottom:2px;}"
    ".status{text-align:center;color:#666;font-size:11px;margin-top:12px;}"
    ".nets{max-height:200px;overflow-y:auto;margin:4px 0 8px;}"
    ".net{padding:10px 12px;background:#333;border-radius:8px;margin:4px 0;cursor:pointer;display:flex;justify-content:space-between;align-items:center;-webkit-tap-highlight-color:transparent;}"
    ".net:active{background:#444;} .sig{color:#0d9488;font-size:12px;} .wifi{} .hidden{display:none;}"
    "</style></head><body><div class='card'><h2>StageMod Setup</h2>"
    "<p class='sub'>Configure your network connection.</p>"
    "<form method='POST' action='/setup'>"
    "<label>Connection Mode</label>"
    "<select name='net_mode' id='mode' onchange=\"document.getElementById('wifi').className=this.value==='1'?'wifi':'hidden'\">"
#if USE_ETHERNET
    "<option value='0'>Ethernet</option>"
#endif
    "<option value='1' selected>WiFi</option>"
    "</select>"
    "<div id='wifi' class='wifi'>"
    "<label>WiFi Networks" + String(n > 0 ? " (" + String(n) + " found)" : "") + "</label>"
    "<div class='nets'>" + netOptions + "</div>"
    "<a href='/' class='btn btn-sec'>Rescan</a>"
    "<label>WiFi SSID</label>"
    "<input name='wifi_ssid' id='ssid' type='text' placeholder='Enter or tap above'>"
    "<label>WiFi Password</label>"
    "<input name='wifi_pass' type='password' placeholder='Password'>"
    "</div>"
    "<label>Hostname</label>"
    "<input name='hostname' type='text' value='" + htmlEscape(config.hostname) + "'>"
    "<input type='hidden' name='dhcp_enabled' value='1'>"
    "<br><button type='submit'>Save & Connect</button>"
    "</form>"
    "<div class='status'>StageMod v" + String(FIRMWARE_VERSION) + " | AP: " + buildApSsid() + "</div>"
    "</div></body></html>";
  server.send(200, "text/html", html);
}

void setup() {
  Serial.begin(115200);
  delay(300);
  esp_log_level_set("Preferences", ESP_LOG_NONE);
  Serial.print("StageMod firmware v");
  Serial.println(FIRMWARE_VERSION);
  loadConfig();
  configureMqttClient();
  resetRuntimeState(true);
  size_t dmxInBytes = static_cast<size_t>(MAX_DMX_PRESET_UNIVERSE) * 512;
  dmxInData = static_cast<uint8_t*>(malloc(dmxInBytes));
  if (dmxInData != nullptr) {
    memset(dmxInData, 0, dmxInBytes);
  } else {
    config.dmxInArtNetEnabled = false;
    config.dmxInSacnEnabled = false;
    addLog("DMX input buffer allocation failed; DMX input disabled.");
  }
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);
  resetButtonArmed = false;
  for (int i = 0; i < MAX_PENDING_OUTPUTS; i++) pendingOutputs[i].active = false;

  Serial.println("Starting network...");
  // If WiFi mode with no SSID saved, skip straight to AP mode
  bool freshSetup = (config.netMode == NET_WIFI && config.wifiSsid.length() == 0);
  if (freshSetup) {
    Serial.println("No WiFi SSID configured. Starting AP for setup.");
    addLog("No SSID. Starting AP for setup.");
    startWifiAp();
    configureUdpListener();
  } else {
#if USE_ETHERNET
    if (config.netMode == NET_ETHERNET) ETH.begin();
#endif
    applyNetworkConfig();
    if (waitForNetwork(30000)) {
      Serial.print("Network OK. ESP32 IP: ");
      Serial.println(getLocalIp());
      addLog(String("Network OK. IP ") + getLocalIp().toString());
      configureUdpListener();
      startMdns();
      applyTimeConfig();
    } else {
      Serial.println("Network failed to connect.");
      addLog("Network failed. Starting AP.");
      startWifiAp();
      configureUdpListener();
    }
  }

  server.on("/", HTTP_GET, []() {
    if (!ensureAuthenticated()) return;
    if (wifiApMode) {
      serveSetupPage();

      return;
    }
    server.sendHeader("Content-Encoding", "gzip");
    server.sendHeader("Cache-Control", "no-cache");
    server.send_P(200, "text/html", (const char*)WEB_UI_GZ, WEB_UI_GZ_LEN);
  });
  server.on("/triggers", HTTP_POST, handleSaveTriggers);
  server.on("/settings", HTTP_POST, handleSaveSettings);
  server.on("/setup", HTTP_POST, handleSetupSave);
  server.on("/dmx_presets", HTTP_GET, handleDmxPresetEditor);
  server.on("/dmx_presets", HTTP_POST, handleDmxPresetEditor);
  server.on("/send_test", HTTP_POST, handleSendTestOsc);
  server.on("/send_udp_test", HTTP_POST, handleSendTestUdp);
  server.on("/send_mqtt_test", HTTP_POST, handleSendTestMqtt);
  server.on("/send_gpio_test", HTTP_POST, handleSendTestGpio);
  server.on("/send_dmx_test", HTTP_POST, handleSendTestDmx);
  server.on("/scan_wifi", HTTP_GET, handleScanNetworks);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.on("/logs", HTTP_GET, handleLogsPage);
  server.on("/logs.txt", HTTP_GET, handleLogsText);
  server.on("/update", HTTP_GET, handleUpdatePage);
  server.on("/update", HTTP_POST, handleUpdateFinished, handleUpdateUpload);
  server.on("/export_config", HTTP_GET, handleExportConfig);
  server.on("/import_config", HTTP_POST, handleImportConfig, handleImportUpload);

  // JSON REST API
  server.on("/api/config",           HTTP_GET,     handleApiGetConfig);
  server.on("/api/config",           HTTP_POST,    handleApiPostConfig);
  server.on("/api/config",           HTTP_OPTIONS, []() { apiCors(); server.send(204); });
  server.on("/api/status",           HTTP_GET,     handleApiGetStatus);
  server.on("/api/live",             HTTP_GET,     handleApiGetLive);
  server.on("/api/triggers/add",     HTTP_POST,    handleApiAddTrigger);
  server.on("/api/triggers/reorder", HTTP_POST,    handleApiReorderTriggers);
  server.on("/api/devices/add",      HTTP_POST,    handleApiAddDevice);
  server.on("/api/devices/remove",   HTTP_POST,    handleApiRemoveDevice);
  server.on("/api/vars",             HTTP_GET,     []() {
    apiCors();
    String json = "[";
    for (int i = 0; i < globalVarCount; i++) {
      if (i > 0) json += ",";
      json += "{\"name\":\"" + jsonEscape(globalVars[i].name) + "\",\"value\":" + String(globalVars[i].value, 3) + "}";
    }
    json += "]";
    apiJson(200, json);
  });

  // Captive portal detection — serve setup form directly
  server.on("/generate_204", HTTP_GET, []() { serveSetupPage(); });
  server.on("/hotspot-detect.html", HTTP_GET, []() { serveSetupPage(); });
  server.on("/fwlink", HTTP_GET, []() { serveSetupPage(); });
  server.on("/ncsi.txt", HTTP_GET, []() { serveSetupPage(); });
  server.on("/favicon.ico", HTTP_GET, []() { server.send(204); });
  server.onNotFound([]() {
    String uri = server.uri();
    if (uri.startsWith("/api/trigger/")) { handleApiTriggerDispatch(); return; }
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  // Needed by checkSameOrigin() — WebServer only stores headers it's told to collect
  const char* collectHdrs[] = {"Origin", "Referer"};
  server.collectHeaders(collectHdrs, 2);
  server.begin();
  Serial.println("Web server started.");

  applySensorPinModes();
  setupEncoderCounters();
}

void loop() {
  int resetState = digitalRead(RESET_BUTTON_PIN);
  if (!resetButtonArmed) {
    if (resetState == HIGH) {
      if (resetHighStartMs == 0) resetHighStartMs = millis();
      if (millis() - resetHighStartMs >= RESET_ARM_MS) {
        resetButtonArmed = true;
      }
    } else {
      resetHighStartMs = 0;
    }
    resetPressStartMs = 0;
  } else {
    if (resetState == LOW) {
      if (resetPressStartMs == 0) resetPressStartMs = millis();
      if (millis() - resetPressStartMs >= RESET_HOLD_MS) {
        addLog("Factory reset via BUT1 hold.");
        doFactoryReset(false);
      }
    } else {
      resetPressStartMs = 0;
    }
  }

  unsigned long pulseNow = millis();
  for (int pin = 0; pin < 40; pin++) {
    if (gpioPulseUntilByPin[pin] > 0 && static_cast<int32_t>(pulseNow - gpioPulseUntilByPin[pin]) >= 0) {
      setGpioLevel(pin, gpioPulseEndLevelByPin[pin] != 0);
      gpioPulseUntilByPin[pin] = 0;
    }
  }

  processOscInput();
  processDmxInput();
  tickDmxInTriggers();

  for (int i = 0; i < MAX_TRIGGERS; i++) {
    SensorConfig& s = config.sensors[i];
    if (!s.enabled) continue;
    if (s.source == SRC_TIME) {
      handleTimeTrigger(i, s);
      continue;
    }
    if (s.source == SRC_OSC || s.source == SRC_DMX) continue;
    if (hasPinConflict(i)) continue;

    float norm = 0.0f;
    bool active = false;
    bool isEncoder = (s.type == SENSOR_ENCODER);
    int8_t prevLevel = lastLevel[i];
    bool cooldownTriggered = false;

    if (isEncoder) {
      int8_t unit = encUnitByTrigger[i];
      if (unit >= 0) {
        int16_t count = 0;
        pcnt_get_counter_value(static_cast<pcnt_unit_t>(unit), &count);
        if (count != 0) {
          if (s.encInvert) count = -count;
          encAccum[i] += count;
          pcnt_counter_clear(static_cast<pcnt_unit_t>(unit));
        }
      } else {
        int a = digitalRead(s.encClkPin);
        int b = digitalRead(s.encDtPin);
        int state = (a << 1) | b;
        if (lastEncState[i] < 0) {
          lastEncState[i] = state;
        }
        if (lastEncState[i] >= 0) {
          static const int8_t kEncTable[16] = {
            0, -1, 1, 0,
            1, 0, 0, -1,
            -1, 0, 0, 1,
            0, 1, -1, 0
          };
          int idx = (lastEncState[i] << 2) | state;
          int8_t delta = kEncTable[idx];
          if (delta != 0) {
            if (s.encInvert) delta = -delta;
            encAccum[i] += delta;
          }
        }
        lastEncState[i] = state;
        lastEncA[i] = a;
        lastEncB[i] = b;
      }

      int encSteps = s.encSteps;
      if (encSteps < 1) encSteps = 1;
      if (encSteps > 4) encSteps = 4;
      while (encAccum[i] >= encSteps) {
        addLog(String("Encoder ") + s.name + " step +");
        for (int o = 0; o < s.outputCount; o++) {
          const SensorConfig::OutputConfig& out = s.outputs[o];
          if (out.target != OUT_TARGET_OSC) continue;
          if (out.oscAddress.length() == 0) continue;
          IPAddress ip;
          uint16_t port;
          if (!getOscTarget(out.device, ip, port)) continue;
          const char* sign = "+";
          String addr = out.oscAddress;
          if (s.encAppendSign) addr += "/+";
          int step = lroundf(fabsf(mapOutput(out, 1.0f)));
          if (step == 0) step = 1;
          if (s.encSignArg) {
            if (out.outMode == OUT_FLOAT) {
              sendOscStringFloatTo(ip, port, addr.c_str(), sign, static_cast<float>(step));
            } else {
              sendOscStringIntTo(ip, port, addr.c_str(), sign, step);
            }
          } else {
            if (s.encAppendSign) {
              if (out.outMode == OUT_FLOAT) {
                sendOscFloatTo(ip, port, addr.c_str(), static_cast<float>(step));
              } else {
                sendOscIntTo(ip, port, addr.c_str(), step);
              }
            } else {
              if (out.outMode == OUT_FLOAT) {
                sendOscFloatTo(ip, port, addr.c_str(), static_cast<float>(step));
              } else {
                sendOscIntTo(ip, port, addr.c_str(), step);
              }
            }
          }
        }
        encAccum[i] -= encSteps;
      }
      while (encAccum[i] <= -encSteps) {
        addLog(String("Encoder ") + s.name + " step -");
        for (int o = 0; o < s.outputCount; o++) {
          const SensorConfig::OutputConfig& out = s.outputs[o];
          if (out.target != OUT_TARGET_OSC) continue;
          if (out.oscAddress.length() == 0) continue;
          IPAddress ip;
          uint16_t port;
          if (!getOscTarget(out.device, ip, port)) continue;
          const char* sign = "-";
          String addr = out.oscAddress;
          if (s.encAppendSign) addr += "/-";
          int step = lroundf(fabsf(mapOutput(out, 1.0f)));
          if (step == 0) step = 1;
          if (s.encSignArg) {
            if (out.outMode == OUT_FLOAT) {
              sendOscStringFloatTo(ip, port, addr.c_str(), sign, static_cast<float>(step));
            } else {
              sendOscStringIntTo(ip, port, addr.c_str(), sign, step);
            }
          } else {
            if (s.encAppendSign) {
              if (out.outMode == OUT_FLOAT) {
                sendOscFloatTo(ip, port, addr.c_str(), static_cast<float>(step));
              } else {
                sendOscIntTo(ip, port, addr.c_str(), step);
              }
            } else {
              int signedStep = -step;
              if (out.outMode == OUT_FLOAT) {
                sendOscFloatTo(ip, port, addr.c_str(), static_cast<float>(signedStep));
              } else {
                sendOscIntTo(ip, port, addr.c_str(), signedStep);
              }
            }
          }
        }
        encAccum[i] += encSteps;
      }

      bool pressed = (digitalRead(s.encSwPin) == LOW);
      norm = pressed ? 1.0f : 0.0f;

      // Prime encoder outputs on first loop after reset/save to avoid sending
      // a "default/off" message when nothing actually changed.
      if (prevLevel == -1) {
        lastLevel[i] = pressed ? 1 : 0;
        for (int o = 0; o < s.outputCount; o++) {
          const SensorConfig::OutputConfig& out = s.outputs[o];
          const bool useButtonOsc = (out.target == OUT_TARGET_OSC);
          const OutputMode outMode = useButtonOsc ? out.buttonOutMode : out.outMode;
          const float outMin = useButtonOsc ? out.buttonOutMin : out.outMin;
          const float outMax = useButtonOsc ? out.buttonOutMax : out.outMax;
          if (out.target == OUT_TARGET_GPIO) {
            if (out.gpioMode == GPIO_OUT_PWM) {
              float outputValue = mapOutputRange(norm, outMin, outMax);
              outputValue = snapOutputRange(outputValue, outMin, outMax);
              int duty = lroundf(outputValue);
              if (duty < 0) duty = 0;
              if (duty > GPIO_PWM_MAX) duty = GPIO_PWM_MAX;
              if (out.gpioInvert) duty = GPIO_PWM_MAX - duty;
              lastInt[i][o] = duty;
            } else {
              lastBool[i][o] = pressed ? 1 : 0;
            }
          } else if (out.target == OUT_TARGET_UDP || out.target == OUT_TARGET_HTTP || out.target == OUT_TARGET_MQTT || out.target == OUT_TARGET_DMX || out.target == OUT_TARGET_OSC) {
            if (outMode == OUT_STRING) {
              lastBool[i][o] = pressed ? 1 : 0;
            } else if (outMode == OUT_FLOAT) {
              float outputValue = mapOutputRange(norm, outMin, outMax);
              outputValue = snapOutputRange(outputValue, outMin, outMax);
              lastFloat[i][o] = outputValue;
            } else {
              float outputValue = mapOutputRange(norm, outMin, outMax);
              outputValue = snapOutputRange(outputValue, outMin, outMax);
              int intValue = lroundf(outputValue);
              lastInt[i][o] = intValue;
            }
          }
        }
        continue;
      }
    }

    if (s.type == SENSOR_BUTTON_DOUBLE) {
      bool pressed = readDigitalActive(s);
      processClickPattern(i, s, pressed, 2, false);
      continue;
    }
    if (s.type == SENSOR_BUTTON_TRIPLE) {
      bool pressed = readDigitalActive(s);
      processClickPattern(i, s, pressed, 3, false);
      continue;
    }
    if (s.type == SENSOR_BUTTON_LONG) {
      bool pressed = readDigitalActive(s);
      processClickPattern(i, s, pressed, 0, true);
      continue;
    }
    if (s.type == SENSOR_BUTTON && hasSharedClickPattern(i)) {
      bool pressed = readDigitalActive(s);
      processClickPattern(i, s, pressed, 1, false);
      continue;
    }

    if (s.type == SENSOR_ANALOG) {
      norm = readAnalogNormalized(i, s);
    } else if (s.type == SENSOR_HCSR04) {
      norm = readHcSr04Normalized(i, s);
    } else if (!isEncoder) {
      active = readDigitalActive(s);
      if (s.type == SENSOR_TOGGLE) {
        if (lastLevel[i] == -1) lastLevel[i] = active ? 1 : 0;
        if (active && lastLevel[i] == 0) {
          toggleState[i] = !toggleState[i];
          addLog(String("Toggle ") + s.name + (toggleState[i] ? " ON" : " OFF"));
        }
        lastLevel[i] = active ? 1 : 0;
        norm = toggleState[i] ? 1.0f : 0.0f;
      } else {
        lastLevel[i] = active ? 1 : 0;
        norm = active ? 1.0f : 0.0f;
      }
    }

    if ((s.type == SENSOR_BUTTON || s.type == SENSOR_GPIO) && s.cooldownEnabled) {
      bool pressed = (norm >= 0.5f);
      bool rising = pressed && (prevLevel <= 0);
      if (!rising) {
        continue;
      }
      unsigned long now = millis();
      if (now - lastButtonTriggerMs[i] < s.cooldownMs) {
        continue;
      }
      lastButtonTriggerMs[i] = now;
      cooldownTriggered = true;
    }

    if (cooldownTriggered) {
      for (int o = 0; o < s.outputCount; o++) {
        lastBool[i][o] = -1;
        lastInt[i][o] = -1;
        lastFloat[i][o] = NAN;
      }
    }

    bool risingEdge = false;
    if (isEncoder) {
      bool pressed = (norm >= 0.5f);
      risingEdge = pressed && (prevLevel <= 0);
      lastLevel[i] = pressed ? 1 : 0;
    } else if (s.type != SENSOR_ANALOG) {
      risingEdge = active && (prevLevel <= 0);
    }

    if (risingEdge && !isEncoder) {
      if (s.type == SENSOR_BUTTON) {
        addLog(String("Button press: ") + s.name);
      } else if (s.type == SENSOR_GPIO) {
        addLog(String("GPIO input: ") + s.name);
      }
    }

    if (!isEncoder && s.type == SENSOR_BUTTON && prevLevel > 0 && !active) {
      addLog(String("Button release: ") + s.name);
    }

    if (!checkCondition(s)) continue;

    for (int o = 0; o < s.outputCount; o++) {
      const SensorConfig::OutputConfig& out = s.outputs[o];
      const bool useButtonOsc = isEncoder && out.target == OUT_TARGET_OSC;
      const String& outAddress = useButtonOsc ? out.buttonAddress : out.oscAddress;
      const OutputMode outMode = useButtonOsc ? out.buttonOutMode : out.outMode;
      const float outMin = useButtonOsc ? out.buttonOutMin : out.outMin;
      const float outMax = useButtonOsc ? out.buttonOutMax : out.outMax;
      const String& onStr = useButtonOsc ? out.buttonOnString : out.onString;
      const String& offStr = useButtonOsc ? out.buttonOffString : out.offString;
      bool changed = false;

      if (out.target == OUT_TARGET_REBOOT) {
        if (s.type == SENSOR_ANALOG || s.type == SENSOR_ENCODER) continue;
        bool pressed = (norm >= 0.5f);
        bool rising = pressed && (prevLevel <= 0);
        if (rising) {
          addLog("Reboot triggered by trigger.");
          delay(100);
          ESP.restart();
        }
        continue;
      }

      if (out.target == OUT_TARGET_RESET_COOLDOWNS) {
        if (s.type == SENSOR_ANALOG || s.type == SENSOR_ENCODER) continue;
        bool pressed = (norm >= 0.5f);
        bool rising = pressed && (prevLevel <= 0);
        if (rising) {
          resetAllCooldowns();
        }
        continue;
      }

      if (out.target == OUT_TARGET_GPIO) {
        bool pressed = (norm >= 0.5f);
        if (out.gpioMode == GPIO_OUT_PULSE) {
          bool rising = pressed && (prevLevel <= 0);
          if (rising) {
            addLog(String("GPIO ") + String(out.gpioPin) + " pulse");
            scheduleGpioPulse(out.gpioPin, out.gpioPulseMs, out.gpioInvert);
          }
        } else if (out.gpioMode == GPIO_OUT_PWM) {
          if (!out.sendMinOnRelease && !isEncoder && s.type != SENSOR_ANALOG) {
            bool rising = pressed && (prevLevel <= 0);
            if (rising) {
              float outputValue = mapOutput(out, 1.0f);
              outputValue = snapOutput(out, outputValue);
              int duty = lroundf(outputValue);
              if (duty < 0) duty = 0;
              if (duty > GPIO_PWM_MAX) duty = GPIO_PWM_MAX;
              if (out.gpioInvert) duty = GPIO_PWM_MAX - duty;
              setGpioPwmDuty(out.gpioPin, duty);
              lastInt[i][o] = duty;
            }
          } else {
            float outputValue = mapOutput(out, norm);
            outputValue = snapOutput(out, outputValue);
            int duty = lroundf(outputValue);
            if (duty < 0) duty = 0;
            if (duty > GPIO_PWM_MAX) duty = GPIO_PWM_MAX;
            if (out.gpioInvert) duty = GPIO_PWM_MAX - duty;
            if (lastInt[i][o] == -1 || duty != lastInt[i][o]) {
              setGpioPwmDuty(out.gpioPin, duty);
              lastInt[i][o] = duty;
            }
          }
        } else {
          int8_t desired = pressed ? 1 : 0;
          if (out.gpioInvert) {
            desired = desired ? 0 : 1;
          }
          if (lastBool[i][o] == -1 || desired != lastBool[i][o]) {
            addLog(String("GPIO ") + String(out.gpioPin) + (desired ? " HIGH" : " LOW"));
            setGpioLevel(out.gpioPin, desired == 1);
            lastBool[i][o] = desired;
          }
        }
        continue;
      }

      if (out.target == OUT_TARGET_HTTP) {
        if (!out.sendMinOnRelease && s.type != SENSOR_ANALOG) {
          bool rising = (norm >= 0.5f) && (prevLevel <= 0);
          if (!rising) {
            continue;
          }
          sendHttpRequest(out, 1.0f);
          continue;
        }
        if (outMode == OUT_STRING) {
          int8_t state = (norm >= 0.5f) ? 1 : 0;
          if (lastBool[i][o] == -1 || state != lastBool[i][o]) {
            sendHttpRequest(out, norm);
            lastBool[i][o] = state;
          }
        } else if (outMode == OUT_FLOAT) {
          float outputValue = mapOutputRange(norm, outMin, outMax);
          outputValue = snapOutputRange(outputValue, outMin, outMax);
          if (isnan(lastFloat[i][o]) || fabsf(outputValue - lastFloat[i][o]) > 0.0005f) {
            sendHttpRequest(out, norm);
            lastFloat[i][o] = outputValue;
          }
      } else {
        float outputValue = mapOutputRange(norm, outMin, outMax);
        outputValue = snapOutputRange(outputValue, outMin, outMax);
        int intValue = lroundf(outputValue);
        bool canSend = true;
        if (s.type == SENSOR_ANALOG) {
          if (lastInt[i][o] >= 0 && abs(intValue - lastInt[i][o]) <= DEAD_BAND) {
            intValue = lastInt[i][o];
            canSend = false;
          }
        }
        if (intValue != lastInt[i][o] && canSend) {
          sendHttpRequest(out, norm);
          lastInt[i][o] = intValue;
        }
        }
        continue;
      }

      if (out.target == OUT_TARGET_MQTT) {
        if (!out.sendMinOnRelease && s.type != SENSOR_ANALOG) {
          bool rising = (norm >= 0.5f) && (prevLevel <= 0);
          if (!rising) {
            continue;
          }
          sendMqttOutput(out, 1.0f, &s, i, o);
          continue;
        }
        if (outMode == OUT_STRING) {
          int8_t state = (norm >= 0.5f) ? 1 : 0;
          if (lastBool[i][o] == -1 || state != lastBool[i][o]) {
            sendMqttOutput(out, norm, &s, i, o);
            lastBool[i][o] = state;
          }
        } else if (outMode == OUT_FLOAT) {
          float outputValue = mapOutputRange(norm, outMin, outMax);
          outputValue = snapOutputRange(outputValue, outMin, outMax);
          if (isnan(lastFloat[i][o]) || fabsf(outputValue - lastFloat[i][o]) > 0.0005f) {
            sendMqttOutput(out, norm, &s, i, o);
            lastFloat[i][o] = outputValue;
          }
        } else {
          float outputValue = mapOutputRange(norm, outMin, outMax);
          outputValue = snapOutputRange(outputValue, outMin, outMax);
          int intValue = lroundf(outputValue);
          bool canSend = true;
          if (s.type == SENSOR_ANALOG) {
            if (lastInt[i][o] >= 0 && abs(intValue - lastInt[i][o]) <= DEAD_BAND) {
              intValue = lastInt[i][o];
              canSend = false;
            }
          }
          if (intValue != lastInt[i][o] && canSend) {
            sendMqttOutput(out, norm, &s, i, o);
            lastInt[i][o] = intValue;
          }
        }
        continue;
      }

      if (out.target == OUT_TARGET_DMX) {
        if (!out.sendMinOnRelease && s.type != SENSOR_ANALOG) {
          bool rising = (norm >= 0.5f) && (prevLevel <= 0);
          if (!rising) {
            continue;
          }
          if (sendDmxOutput(out, 1.0f)) {
            lastInt[i][o] = static_cast<int>(toDmxValue(out, 1.0f));
          }
          continue;
        }
        int intValue = static_cast<int>(toDmxValue(out, norm));
        bool canSend = true;
        if (s.type == SENSOR_ANALOG) {
          if (lastInt[i][o] >= 0 && abs(intValue - lastInt[i][o]) <= DEAD_BAND) {
            intValue = lastInt[i][o];
            canSend = false;
          }
        }
        if (intValue != lastInt[i][o] && canSend) {
          if (sendDmxOutput(out, norm)) {
            lastInt[i][o] = intValue;
          }
        }
        continue;
      }

      if (out.target == OUT_TARGET_DMX_PRESET) {
        bool rising = (norm >= 0.5f) && (prevLevel <= 0);
        if (!rising) {
          continue;
        }
        triggerDmxPreset(out);
        continue;
      }

      IPAddress outIp;
      uint16_t outPort;
      if (!getOscTarget(out.device, outIp, outPort)) {
        continue;
      }

      if (out.target == OUT_TARGET_UDP) {
        if (out.udpPayload.length() == 0) continue;
        bool pressed = (norm >= 0.5f);
        bool rising = pressed && (prevLevel <= 0);
        if (!out.sendMinOnRelease && s.type != SENSOR_ANALOG) {
          if (!rising) {
            continue;
          }
          sendUdpRawTo(outIp, outPort, out.udpPayload);
          lastBool[i][o] = 1;
          continue;
        }
        int8_t state = pressed ? 1 : 0;
        if (lastBool[i][o] == -1 || state != lastBool[i][o]) {
          sendUdpRawTo(outIp, outPort, out.udpPayload);
          lastBool[i][o] = state;
        }
        continue;
      }

      if (outAddress.length() == 0) continue;

      if (!out.sendMinOnRelease && s.type != SENSOR_ANALOG) {
        if (!risingEdge) {
          continue;
        }
        if (outMode == OUT_STRING) {
          const char* value = onStr.c_str();
          if (value[0] != '\0') {
            sendOscStringTo(outIp, outPort, outAddress.c_str(), value);
            lastBool[i][o] = 1;
          }
        } else if (outMode == OUT_FLOAT) {
          float outputValue = mapOutputRange(1.0f, outMin, outMax);
          outputValue = snapOutputRange(outputValue, outMin, outMax);
          sendOscFloatTo(outIp, outPort, outAddress.c_str(), outputValue);
          lastFloat[i][o] = outputValue;
        } else {
          float outputValue = mapOutputRange(1.0f, outMin, outMax);
          outputValue = snapOutputRange(outputValue, outMin, outMax);
          int intValue = lroundf(outputValue);
          sendOscIntTo(outIp, outPort, outAddress.c_str(), intValue);
          lastInt[i][o] = intValue;
        }
        continue;
      }

      if (outMode == OUT_STRING) {
        int8_t state = (norm >= 0.5f) ? 1 : 0;
        changed = (lastBool[i][o] == -1) || (state != lastBool[i][o]);
        if (changed) {
          const char* value = (state == 1) ? onStr.c_str() : offStr.c_str();
          if (value[0] != '\0') {
            sendOscStringTo(outIp, outPort, outAddress.c_str(), value);
            lastBool[i][o] = state;
          } else {
            changed = false;
          }
        }
      } else if (outMode == OUT_FLOAT) {
        float outputValue = mapOutputRange(norm, outMin, outMax);
        outputValue = snapOutputRange(outputValue, outMin, outMax);
        changed = (isnan(lastFloat[i][o]) || fabsf(outputValue - lastFloat[i][o]) > 0.0005f);
        if (changed) {
          sendOscFloatTo(outIp, outPort, outAddress.c_str(), outputValue);
          lastFloat[i][o] = outputValue;
        }
      } else {
        float outputValue = mapOutputRange(norm, outMin, outMax);
        outputValue = snapOutputRange(outputValue, outMin, outMax);
        int intValue = lroundf(outputValue);
        bool canSend = true;
        if (s.type == SENSOR_ANALOG) {
          if (lastInt[i][o] >= 0 && abs(intValue - lastInt[i][o]) <= DEAD_BAND) {
            intValue = lastInt[i][o];
            canSend = false;
          }
        }
        changed = (intValue != lastInt[i][o]) && canSend;
        if (changed) {
          sendOscIntTo(outIp, outPort, outAddress.c_str(), intValue);
          lastInt[i][o] = intValue;
        }
      }

      if (changed) {
        Serial.print("Sent ");
        Serial.print(outAddress);
        Serial.print(" = ");
        if (outMode == OUT_STRING) {
          Serial.println((norm >= 0.5f) ? onStr : offStr);
        } else if (outMode == OUT_FLOAT) {
          Serial.println(lastFloat[i][o], 4);
        } else {
          Serial.println(lastInt[i][o]);
        }
      }
    }
  }

  tickDmxPresetFade();
  tickDmxKeepAlive();
  tickPendingOutputs();

  delay(LOOP_DELAY_MS);

  server.handleClient();

  if (pendingNetApply && millis() - pendingApplyAt > 250) {
    pendingNetApply = false;
    applyNetworkConfig();
    configureUdpListener();
    wifiReconnectStarted = (config.netMode == NET_WIFI) ? millis() : 0;
    if (pendingMdnsRestart) {
      pendingMdnsRestart = false;
      MDNS.end();
      startMdns();
    }
  }
  if (mqttReconnectRequested) {
    mqttReconnectRequested = false;
    mqttClient.disconnect();
    configureMqttClient();
    mqttLastConnectAttemptMs = 0;
  }
  if (config.mqttEnabled && isValidIp(getLocalIp())) {
    ensureMqttConnected();
  }
  if (mqttClient.connected()) {
    mqttClient.loop();
  }
  if (wifiReconnectStarted > 0 && config.netMode == NET_WIFI) {
    if (WiFi.status() == WL_CONNECTED && isValidIp(WiFi.localIP())) {
      wifiReconnectStarted = 0;
      if (wifiApMode) {
        WiFi.softAPdisconnect(true);
        wifiApMode = false;
        dnsServer.stop();
      }
      configureUdpListener();
      startMdns();
      Serial.print("WiFi connected. IP: ");
      Serial.println(getLocalIp());
    } else if (!wifiApMode && millis() - wifiReconnectStarted > 20000) {
      wifiReconnectStarted = 0;
      startWifiAp();
    }
  }

  // Debounce link-down before falling back to AP mode: a single bad sample
  // (DHCP renew, cable wiggle, transient WiFi drop) must not flap us into AP.
  static unsigned long netDownSince = 0;
  const unsigned long NET_DOWN_GRACE_MS = 5000;

#if USE_ETHERNET
  if (config.netMode == NET_ETHERNET) {
    bool netOk = ETH.linkUp() && isValidIp(ETH.localIP());
    if (!netOk && !wifiApMode) {
      if (netDownSince == 0) {
        netDownSince = millis();
      } else if (millis() - netDownSince > NET_DOWN_GRACE_MS) {
        netDownSince = 0;
        startWifiAp();
        configureUdpListener();
      }
    } else if (netOk) {
      netDownSince = 0;
      if (wifiApMode) {
        WiFi.softAPdisconnect(true);
        wifiApMode = false;
        dnsServer.stop();
        configureUdpListener();
      }
    }
  }
#endif

  if (config.netMode == NET_WIFI && wifiReconnectStarted == 0) {
    bool netOk = (WiFi.status() == WL_CONNECTED) && isValidIp(WiFi.localIP());
    if (!netOk && !wifiApMode) {
      if (netDownSince == 0) {
        netDownSince = millis();
      } else if (millis() - netDownSince > NET_DOWN_GRACE_MS) {
        netDownSince = 0;
        // Try rejoining the configured network first; AP fallback happens
        // via the wifiReconnectStarted timeout if the rejoin fails.
        WiFi.reconnect();
        wifiReconnectStarted = millis();
      }
    } else if (netOk) {
      netDownSince = 0;
      if (wifiApMode) {
        WiFi.softAPdisconnect(true);
        wifiApMode = false;
        dnsServer.stop();
        configureUdpListener();
      }
    }
  }

  if (config.timeMode == TIME_NTP) {
    pollNtpSync();
  }

  if (wifiApMode) {
    dnsServer.processNextRequest();
  }

  unsigned long now = millis();
  for (int i = 0; i < config.oscDeviceCount; i++) {
    int idx = config.oscDeviceOrder[i];
    const OscDevice& d = config.oscDevices[idx];
    if (!d.enabled || !d.hbEnabled || d.hbAddress.length() == 0) continue;
    if (now - hbLastSent[idx] >= d.hbMs) {
      hbLastSent[idx] = now;
      sendOscIntTo(d.ip, d.port, d.hbAddress.c_str(), 1);
    }
  }
}
