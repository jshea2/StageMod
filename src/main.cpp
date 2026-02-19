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
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFiUdp.h>
#include <time.h>
#include <sys/time.h>
#include "driver/pcnt.h"
#ifndef WEBSERVER_MAX_POST_ARGS
#define WEBSERVER_MAX_POST_ARGS 800
#endif
#include <WebServer.h>
#include <Update.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

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
const char* FIRMWARE_VERSION = "0.13.0";
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
const int MAX_DMX_PRESET_POINTS = 4096;
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
  SRC_OSC = 2
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
  OUT_TARGET_DMX_PRESET = 8
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
  uint8_t outputCount;
  struct OutputConfig {
    OutputTarget target;
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
#else
  while (WiFi.status() != WL_CONNECTED || !isValidIp(WiFi.localIP())) {
    delay(100);
    if (millis() - start > timeoutMs) return false;
  }
#endif
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
  DynamicJsonDocument doc(98304);
  doc["version"] = FIRMWARE_VERSION;
  JsonObject c = doc.createNestedObject("config");
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
  for (int i = 0; i < MAX_TRIGGERS; i++) {
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
    so["outputCount"] = s.outputCount;
    JsonArray outs = so.createNestedArray("outputs");
    for (int o = 0; o < s.outputCount && o < MAX_OUTPUTS_PER_TRIGGER; o++) {
      const SensorConfig::OutputConfig& out = s.outputs[o];
      JsonObject oo = outs.createNestedObject();
      oo["index"] = o;
      oo["target"] = static_cast<int>(out.target);
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

  if (cfg.containsKey("username")) config.username = String(cfg["username"].as<const char*>());
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
      if (s.source != SRC_TIME && s.source != SRC_OSC) s.source = SRC_SENSORS;
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
  if (value < OUT_TARGET_OSC || value > OUT_TARGET_DMX_PRESET) return OUT_TARGET_OSC;
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
    if (s.source == SRC_TIME || s.source == SRC_OSC) continue;
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

static bool ensureAuthenticated() {
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
  s.outputCount = 1;
  bool defaultSendMin = (s.type == SENSOR_ANALOG || s.type == SENSOR_ENCODER);
  for (int o = 0; o < MAX_OUTPUTS_PER_TRIGGER; o++) {
    s.outputs[o].target = OUT_TARGET_OSC;
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
    for (int o = 0; o < s.outputCount; o++) {
      const SensorConfig::OutputConfig& out = s.outputs[o];
      sendOutputNow(s, out, out.oscAddress, 1.0f);
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
    if (sensor.source == SRC_TIME || sensor.source == SRC_OSC) continue;
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
    if (!s.enabled || s.source == SRC_TIME || s.source == SRC_OSC || s.type != SENSOR_ENCODER) continue;
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
    if (b.source == SRC_TIME || b.source == SRC_OSC) continue;
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
  if (!a.enabled || a.source == SRC_TIME || a.source == SRC_OSC) return false;
  if (a.type != SENSOR_BUTTON) return false;
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    if (i == index) continue;
    const SensorConfig& b = config.sensors[i];
    if (!b.enabled || b.source == SRC_TIME || b.source == SRC_OSC) continue;
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
    if (s.source != SRC_TIME && s.source != SRC_OSC) s.source = SRC_SENSORS;
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
    prefs.putString(sensorKey(i, "osia").c_str(), s.oscInAddress);
    prefs.putUInt(sensorKey(i, "osit").c_str(), s.oscInArgType);
    prefs.putUInt(sensorKey(i, "osim").c_str(), s.oscInMatchMode);
    prefs.putFloat(sensorKey(i, "osiv").c_str(), s.oscInValue);
    prefs.putFloat(sensorKey(i, "osmn").c_str(), s.oscInMin);
    prefs.putFloat(sensorKey(i, "osmx").c_str(), s.oscInMax);
    prefs.putString(sensorKey(i, "osis").c_str(), s.oscInString);
    prefs.putInt(sensorKey(i, "type").c_str(), s.type);
    prefs.putInt(sensorKey(i, "pin").c_str(), s.pin);
    prefs.putInt(sensorKey(i, "clk").c_str(), s.encClkPin);
    prefs.putInt(sensorKey(i, "dt").c_str(), s.encDtPin);
    prefs.putInt(sensorKey(i, "sw").c_str(), s.encSwPin);
    prefs.putBool(sensorKey(i, "enc_app").c_str(), s.encAppendSign);
    prefs.putBool(sensorKey(i, "enc_arg").c_str(), s.encSignArg);
    prefs.putBool(sensorKey(i, "enc_inv").c_str(), s.encInvert);
    prefs.putInt(sensorKey(i, "hct").c_str(), s.hcTrigPin);
    prefs.putInt(sensorKey(i, "hce").c_str(), s.hcEchoPin);
    prefs.putFloat(sensorKey(i, "hcmin").c_str(), s.hcMinCm);
    prefs.putFloat(sensorKey(i, "hcmax").c_str(), s.hcMaxCm);
    prefs.putBool(sensorKey(i, "inv").c_str(), s.invert);
    prefs.putBool(sensorKey(i, "ah").c_str(), s.activeHigh);
    prefs.putBool(sensorKey(i, "pu").c_str(), s.pullup);
    prefs.putBool(sensorKey(i, "cd_en").c_str(), s.cooldownEnabled);
    prefs.putUInt(sensorKey(i, "cd_ms").c_str(), s.cooldownMs);
    prefs.putUInt(sensorKey(i, "oc").c_str(), s.outputCount);
    for (int o = 0; o < s.outputCount; o++) {
      const SensorConfig::OutputConfig& out = s.outputs[o];
      prefs.putInt(outputKey(i, o, "tgt").c_str(), out.target);
      prefs.putInt(outputKey(i, o, "dev").c_str(), out.device);
      prefs.putString(outputKey(i, o, "addr").c_str(), out.oscAddress);
      prefs.putString(outputKey(i, o, "baddr").c_str(), out.buttonAddress);
      prefs.putFloat(outputKey(i, o, "omin").c_str(), out.outMin);
      prefs.putFloat(outputKey(i, o, "omax").c_str(), out.outMax);
      prefs.putInt(outputKey(i, o, "omode").c_str(), out.outMode);
      prefs.putFloat(outputKey(i, o, "bmin").c_str(), out.buttonOutMin);
      prefs.putFloat(outputKey(i, o, "bmax").c_str(), out.buttonOutMax);
      prefs.putInt(outputKey(i, o, "bmode").c_str(), out.buttonOutMode);
      prefs.putBool(outputKey(i, o, "smin").c_str(), out.sendMinOnRelease);
      prefs.putString(outputKey(i, o, "udp").c_str(), out.udpPayload);
      prefs.putInt(outputKey(i, o, "gpin").c_str(), out.gpioPin);
      prefs.putInt(outputKey(i, o, "gmode").c_str(), out.gpioMode);
      prefs.putUInt(outputKey(i, o, "gpms").c_str(), out.gpioPulseMs);
      prefs.putBool(outputKey(i, o, "ginv").c_str(), out.gpioInvert);
      prefs.putInt(outputKey(i, o, "hm").c_str(), out.httpMethod);
      prefs.putString(outputKey(i, o, "hu").c_str(), out.httpUrl);
      prefs.putString(outputKey(i, o, "hb").c_str(), out.httpBody);
      prefs.putString(outputKey(i, o, "hip").c_str(), out.httpIp);
      prefs.putUInt(outputKey(i, o, "hpt").c_str(), out.httpPort);
      prefs.putUInt(outputKey(i, o, "hdev").c_str(), out.httpDevice);
      prefs.putString(outputKey(i, o, "mt").c_str(), out.mqttTopic);
      prefs.putString(outputKey(i, o, "mp").c_str(), out.mqttPayload);
      prefs.putBool(outputKey(i, o, "mr").c_str(), out.mqttRetain);
      prefs.putUInt(outputKey(i, o, "dmxp").c_str(), out.dmxProtocol);
      prefs.putUInt(outputKey(i, o, "dmxd").c_str(), out.dmxDest);
      prefs.putUInt(outputKey(i, o, "dmxn").c_str(), out.dmxArtNet);
      prefs.putUInt(outputKey(i, o, "dmxs").c_str(), out.dmxArtSubnet);
      prefs.putUInt(outputKey(i, o, "dmxa").c_str(), out.dmxArtUniverse);
      prefs.putUInt(outputKey(i, o, "dmxu").c_str(), out.dmxUniverse);
      prefs.putString(outputKey(i, o, "dmxc").c_str(), out.dmxChannels);
      prefs.putUInt(outputKey(i, o, "dmxpr").c_str(), out.dmxPreset);
      prefs.putUInt(outputKey(i, o, "dmxfd").c_str(), out.dmxFadeMs);
      prefs.putString(outputKey(i, o, "on").c_str(), out.onString);
      prefs.putString(outputKey(i, o, "off").c_str(), out.offString);
      prefs.putString(outputKey(i, o, "bon").c_str(), out.buttonOnString);
      prefs.putString(outputKey(i, o, "boff").c_str(), out.buttonOffString);
    }
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
    if (s.source == SRC_TIME || s.source == SRC_OSC) continue;
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
    if (!config.sensors[i].enabled) return i;
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

          oscInputTokenOverrideActive = true;
          oscInputTokenValue = value;
          oscInputTokenNorm = clampFloat(norm, 0.0f, 1.0f);
          oscInputTokenState = (oscInputTokenNorm >= 0.5f);
          for (int o = 0; o < s.outputCount; o++) {
            const SensorConfig::OutputConfig& out = s.outputs[o];
            float sendNorm = out.sendMinOnRelease ? oscInputTokenNorm : 1.0f;
            sendOutputNow(s, out, out.oscAddress, sendNorm);
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
    for (int o = 0; o < sensor.outputCount; o++) {
      const SensorConfig::OutputConfig& out = sensor.outputs[o];
      sendOutputNow(sensor, out, out.oscAddress, 1.0f);
    }
    multiLongFired[index] = true;
    clickCount[index] = 0;
  }

  if (!pressed && clickCount[index] > 0 && (now - multiLastReleaseMs[index] >= config.multiClickGapMs)) {
    if (targetClicks > 0 && clickCount[index] == targetClicks) {
      for (int o = 0; o < sensor.outputCount; o++) {
        const SensorConfig::OutputConfig& out = sensor.outputs[o];
        sendOutputNow(sensor, out, out.oscAddress, 1.0f);
      }
    }
    clickCount[index] = 0;
  }
}

static void appendOscDeviceOptions(String& html, uint8_t selected) {
  for (int i = 0; i < config.oscDeviceCount; i++) {
    int idx = config.oscDeviceOrder[i];
    const OscDevice& d = config.oscDevices[idx];
    String label = d.name + " (" + ipToString(d.ip) + ":" + String(d.port) + ")";
    html += "<option value='" + String(idx) + "' " + String(idx == selected ? "selected" : "") + ">" + label + "</option>";
  }
}

static void handleTriggers() {
  if (!ensureAuthenticated()) return;
  String html;
  html.reserve(20000);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>StageMod Triggers</title>";
  html += "<style>";
  html += ":root{--bg:#f4f1ea;--ink:#1a1a1a;--muted:#6c6c6c;--card:#ffffff;--accent:#0b6b6f;--danger:#a02a2a;}";
  html += "body{margin:0;font-family:'Avenir Next','Trebuchet MS','Segoe UI',sans-serif;background:linear-gradient(135deg,#f4f1ea 0%,#e7f0ef 100%);color:var(--ink);}";
  html += ".page{max-width:900px;margin:0 auto;padding:20px 16px 40px;}";
  html += ".header{display:flex;flex-wrap:wrap;gap:12px;align-items:center;justify-content:space-between;margin-bottom:8px;}";
  html += ".title{font-size:22px;font-weight:700;letter-spacing:0.5px;}";
  html += ".actions{display:flex;gap:10px;align-items:center;flex-wrap:wrap;}";
  html += ".nav-link{font-size:16px;color:#6b34d6;text-decoration:underline;font-weight:600;}";
  html += ".nav-pill{font-size:14px;color:#666;padding:6px 12px;border:1px solid #ddd;border-radius:999px;background:#fff;text-decoration:none;}";
  html += ".nav-primary{font-size:16px;color:#fff;padding:8px 14px;border-radius:999px;background:var(--accent);text-decoration:none;}";
  html += "a.primary{background:var(--accent);color:#fff;border-color:var(--accent);padding:6px 10px;border-radius:8px;text-decoration:none;}";
  html += "a.primary{background:var(--accent);color:#fff;border-color:var(--accent);padding:6px 10px;border-radius:8px;text-decoration:none;}";
  html += ".status{display:flex;flex-wrap:wrap;gap:10px;font-size:13px;color:var(--muted);margin:8px 0 14px;}";
  html += ".card{background:var(--card);border:1px solid #e2e2e2;border-radius:12px;padding:12px;margin:10px 0;}";
  html += "fieldset{border:1px solid #e2e2e2;border-radius:12px;padding:12px;margin:12px 0;background:#fff;}";
  html += "legend{padding:0 6px;color:var(--muted);}";
  html += "details{border:1px solid #e2e2e2;border-radius:12px;margin:12px 0;background:#fff;}";
  html += "summary{cursor:pointer;list-style:none;padding:10px 12px;display:flex;align-items:center;justify-content:space-between;}";
  html += ".chev{display:inline-block;margin-right:8px;font-size:14px;transition:transform 0.2s ease;}";
  html += "details[open] .chev{transform:rotate(90deg);}";
  html += ".toggle{position:relative;width:36px;height:20px;display:inline-block;}";
  html += ".toggle input{opacity:0;width:0;height:0;}";
  html += ".toggle span{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#c9c9c9;border-radius:999px;transition:0.2s;}";
  html += ".toggle span:before{content:'';position:absolute;height:16px;width:16px;left:2px;top:2px;background:#fff;border-radius:50%;transition:0.2s;}";
  html += ".toggle input:checked + span{background:#0b6b6f;}";
  html += ".toggle input:checked + span:before{transform:translateX(16px);}";
  html += ".chev{display:inline-block;margin-right:8px;font-size:14px;transition:transform 0.2s ease;}";
  html += "details[open] .chev{transform:rotate(90deg);}";
  html += ".chev{display:inline-block;margin-right:8px;font-size:14px;transition:transform 0.2s ease;}";
  html += "details[open] .chev{transform:rotate(90deg);}";
  html += "summary::-webkit-details-marker{display:none;}";
  html += ".trigger-title{font-weight:600;}";
  html += ".toggle{position:relative;width:36px;height:20px;display:inline-block;}";
  html += ".toggle input{opacity:0;width:0;height:0;}";
  html += ".toggle span{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#c9c9c9;border-radius:999px;transition:0.2s;}";
  html += ".toggle span:before{content:'';position:absolute;height:16px;width:16px;left:2px;top:2px;background:#fff;border-radius:50%;transition:0.2s;}";
  html += ".toggle input:checked + span{background:#0b6b6f;}";
  html += ".toggle input:checked + span:before{transform:translateX(16px);}";
  html += ".trigger-actions{display:flex;gap:6px;align-items:center;}";
  html += ".icon-btn{border:1px solid #b9b9b9;background:#fff;color:#222;padding:4px 8px;border-radius:6px;cursor:pointer;}";
  html += "input,select{width:100%;max-width:320px;padding:6px 8px;margin:2px 0 8px;border:1px solid #cfcfcf;border-radius:6px;background:#fff;}";
  html += ".time-row{display:flex;align-items:center;gap:10px;flex-wrap:wrap;margin:4px 0 8px;}";
  html += ".time-row label{min-width:90px;color:#444;}";
  html += ".time-row input,.time-row select{width:auto;min-width:170px;max-width:240px;}";
  html += ".day-pills{display:flex;flex-wrap:wrap;gap:8px;margin:6px 0 4px;}";
  html += ".day-pill{display:flex;align-items:center;gap:6px;border:1px solid #d6d6d6;border-radius:999px;padding:4px 10px;font-size:13px;background:#f9f9f9;}";
  html += "button{border:1px solid #b9b9b9;background:#fff;color:#222;padding:6px 10px;border-radius:8px;cursor:pointer;}";
  html += "button.primary{background:var(--accent);color:#fff;border-color:var(--accent);}";
  html += "button.danger{background:var(--danger);color:#fff;border-color:var(--danger);}";
  html += "small{color:var(--muted);}";
  html += "#pin_help_modal{display:none;position:fixed;inset:0;background:rgba(0,0,0,0.6);}";
  html += "#pin_help_modal .modal{background:#fff;color:#000;max-width:520px;margin:10% auto;padding:16px;border-radius:12px;box-shadow:0 12px 30px rgba(0,0,0,0.2);}";
  html += "</style></head><body>";
  html += "<div class='page'>";
  html += "<div class='header'>";
  html += "<div class='title'>StageMod</div>";
  html += "<div class='actions'>";
  html += "<a href='https://github.com/OLIMEX/ESP32-POE/blob/master/DOCUMENTS/ESP32-POE-PINOUT.png' class='nav-pill'>Board Pin Help</a>";
  html += "<a href='/logs' class='nav-pill'>Serial Log</a>";
  html += "<a href='/settings' class='nav-primary'>Settings</a>";
  html += "</div></div>";

  html += "<div class='status'>";
  html += "<div>IP: " + getLocalIp().toString() + "</div>";
  html += "<div>mDNS: http://" + config.hostname + ".local/</div>";
  html += "</div>";

  bool anyConflict = false;
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    if (hasPinConflict(i)) {
      anyConflict = true;
      break;
    }
  }
  if (anyConflict) {
    html += "<p style='color:#b00;'>Pin conflict detected: two triggers share the same GPIO.</p>";
  }

  html += "<form method='POST' action='/triggers'>";

  for (int tIndex = 0; tIndex < config.triggerCount; tIndex++) {
    int i = config.triggerOrder[tIndex];
    SensorConfig& s = config.sensors[i];
    String idx = String(i);

    html += "<details data-trigger='" + String(idx) + "'>";
    html += "<summary>";
    html += "<span style='display:flex;align-items:center;gap:10px;'>";
    html += "<span class='chev'>&#9654;</span>";
    html += "<label class='toggle' title='Enable trigger'>";
    html += "<input name='s" + idx + "_en' type='checkbox' " + String(s.enabled ? "checked" : "") + ">";
    html += "<span></span></label>";
    html += "<span class='trigger-title'>" + s.name + "</span>";
    html += "</span>";
    html += "<span class='trigger-actions'>";
    html += "<button type='submit' name='action' value='up:" + idx + "' class='icon-btn'>&uarr;</button>";
    html += "<button type='submit' name='action' value='down:" + idx + "' class='icon-btn'>&darr;</button>";
    html += "<button type='submit' name='action' value='remove:" + idx + "' class='icon-btn'>×</button>";
    html += "</span></summary>";
    html += "<div style='padding:0 12px 12px;'>";
    html += "<input type='hidden' name='s" + idx + "_present' value='1'>";
    html += "<!-- Enable toggle moved to header -->";
    html += "Name: <input name='s" + idx + "_name' type='text' value='" + s.name + "'><br>";

    html += "<div class='card'><b>Input</b><br>";
    html += "Source: <select name='s" + idx + "_src' id='s" + idx + "_src'>";
    html += "<option value='0' " + String(s.source == SRC_SENSORS ? "selected" : "") + ">Sensors</option>";
    html += "<option value='1' " + String(s.source == SRC_TIME ? "selected" : "") + ">Time</option>";
    html += "<option value='2' " + String(s.source == SRC_OSC ? "selected" : "") + ">OSC In</option>";
    html += "</select><br>";
    html += "<div id='s" + idx + "_type_block'>Type: <select name='s" + idx + "_type' id='s" + idx + "_type'>";
    html += "<option value='1' " + String(s.type == SENSOR_BUTTON ? "selected" : "") + ">Button (momentary)</option>";
    html += "<option value='2' " + String(s.type == SENSOR_TOGGLE ? "selected" : "") + ">Button (toggle)</option>";
    html += "<option value='5' " + String(s.type == SENSOR_BUTTON_DOUBLE ? "selected" : "") + ">Button (double click)</option>";
    html += "<option value='6' " + String(s.type == SENSOR_BUTTON_TRIPLE ? "selected" : "") + ">Button (triple click)</option>";
    html += "<option value='7' " + String(s.type == SENSOR_BUTTON_LONG ? "selected" : "") + ">Button (long press)</option>";
    html += "<option value='8' " + String(s.type == SENSOR_GPIO ? "selected" : "") + ">GPIO (input)</option>";
    html += "<option value='9' " + String(s.type == SENSOR_HCSR04 ? "selected" : "") + ">Ultrasonic (HC-SR04)</option>";
    html += "<option value='0' " + String(s.type == SENSOR_ANALOG ? "selected" : "") + ">Analog (pot/slider)</option>";
    html += "<option value='4' " + String(s.type == SENSOR_ENCODER ? "selected" : "") + ">Encoder</option>";
    html += "</select><br></div>";
    html += "<div id='s" + idx + "_pin_block'>";
    html += "GPIO Pin: <input name='s" + idx + "_pin' type='number' value='" + String(s.pin) + "'><br>";
    html += "<small>Analog pins allowed: 32,35,36. Digital pins allowed: 5,13,16,17,18,19,21,22,23,25,26,27,32,33,35,36,39</small><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_hcsr04'>";
    html += "<b>HC-SR04</b><br>";
    html += "Trig pin: <input name='s" + idx + "_hc_t' type='number' value='" + String(s.hcTrigPin) + "'><br>";
    html += "Echo pin: <input name='s" + idx + "_hc_e' type='number' value='" + String(s.hcEchoPin) + "'><br>";
    html += "Min cm: <input name='s" + idx + "_hc_min' type='number' step='0.1' value='" + String(s.hcMinCm, 1) + "'><br>";
    html += "Max cm: <input name='s" + idx + "_hc_max' type='number' step='0.1' value='" + String(s.hcMaxCm, 1) + "'><br>";
    html += "<small>Tip: set Min higher than Max to invert.</small><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_analog'>";
    html += "Invert: <input name='s" + idx + "_inv' type='checkbox' " + String(s.invert ? "checked" : "") + "><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_digital'>";
    html += "Active high: <input id='s" + idx + "_ah' name='s" + idx + "_ah' type='checkbox' " + String(s.activeHigh ? "checked" : "") + "><br>";
    html += "Use pullup: <input name='s" + idx + "_pu' type='checkbox' " + String(s.pullup ? "checked" : "") + "><br>";
    html += "<small>Button to GND: pullup ON, active high OFF. Button to 3.3V: pullup OFF, active high ON.</small><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_cooldown'>";
    html += "<b>Cooldown</b><br>";
    html += "Enabled: <input name='s" + idx + "_cd_en' type='checkbox' " + String(s.cooldownEnabled ? "checked" : "") + "><br>";
    html += "Cooldown ms: <input name='s" + idx + "_cd_ms' type='number' value='" + String(s.cooldownMs) + "'><br>";
    html += "<small>Limits how often the trigger can fire. Applies to all outputs.</small><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_encoder'>";
    html += "<b>Encoder</b><br>";
    html += "CLK pin: <input name='s" + idx + "_clk' type='number' value='" + String(s.encClkPin) + "'><br>";
    html += "DT pin: <input name='s" + idx + "_dt' type='number' value='" + String(s.encDtPin) + "'><br>";
    html += "SW pin: <input name='s" + idx + "_sw' type='number' value='" + String(s.encSwPin) + "'><br>";
    html += "Append /+/- to address: <input name='s" + idx + "_enc_app' type='checkbox' " + String(s.encAppendSign ? "checked" : "") + "><br>";
    html += "Insert +/- as 1st arg: <input name='s" + idx + "_enc_arg' type='checkbox' " + String(s.encSignArg ? "checked" : "") + "><br>";
    html += "Invert direction: <input name='s" + idx + "_enc_inv' type='checkbox' " + String(s.encInvert ? "checked" : "") + "><br>";
    html += "<small>Encoder step sends relative changes. Use options above for QLab/Resolume.</small><br>";
    html += "</div>";

    html += "<div id='s" + idx + "_oscin'>";
    html += "<b>OSC In</b><br>";
    html += "Address pattern: <input name='s" + idx + "_osia' type='text' value='" + s.oscInAddress + "' placeholder='/trigger/*'><br>";
    html += "Argument type: <select name='s" + idx + "_osit' id='s" + idx + "_osit'>";
    html += "<option value='0' " + String(s.oscInArgType == OSC_IN_ARG_ANY ? "selected" : "") + ">Any / none</option>";
    html += "<option value='1' " + String(s.oscInArgType == OSC_IN_ARG_INT ? "selected" : "") + ">Int</option>";
    html += "<option value='2' " + String(s.oscInArgType == OSC_IN_ARG_FLOAT ? "selected" : "") + ">Float</option>";
    html += "<option value='3' " + String(s.oscInArgType == OSC_IN_ARG_STRING ? "selected" : "") + ">String</option>";
    html += "</select><br>";
    html += "Match mode: <select name='s" + idx + "_osim' id='s" + idx + "_osim'>";
    html += "<option value='0' " + String(s.oscInMatchMode == OSC_IN_MATCH_ANY ? "selected" : "") + ">Any value</option>";
    html += "<option value='1' " + String(s.oscInMatchMode == OSC_IN_MATCH_EQUAL ? "selected" : "") + ">Equals</option>";
    html += "<option value='2' " + String(s.oscInMatchMode == OSC_IN_MATCH_RANGE ? "selected" : "") + ">Range</option>";
    html += "</select><br>";
    html += "<div id='s" + idx + "_osiv_row'>Value: <input name='s" + idx + "_osiv' type='number' step='0.001' value='" + String(s.oscInValue, 3) + "'><br></div>";
    html += "<div id='s" + idx + "_osir_row'>Min: <input name='s" + idx + "_osmn' type='number' step='0.001' value='" + String(s.oscInMin, 3) + "'><br>Max: <input name='s" + idx + "_osmx' type='number' step='0.001' value='" + String(s.oscInMax, 3) + "'><br></div>";
    html += "<div id='s" + idx + "_osis_row'>String: <input name='s" + idx + "_osis' type='text' value='" + s.oscInString + "'><br></div>";
    html += "<small>Wildcard '*' supported. Examples: /go, /qlab/*</small><br>";
    html += "</div>";

    html += "<div id='s" + idx + "_time_block'>";
    html += "<small>Current time: " + formatTimeLocal() + "</small><br>";
    html += "<div class='time-row'><label>Time type</label><select name='s" + idx + "_tt' id='s" + idx + "_tt'>";
    html += "<option value='0' " + String(s.timeType == TIME_ONCE ? "selected" : "") + ">Date and Time</option>";
    html += "<option value='1' " + String(s.timeType == TIME_DAILY ? "selected" : "") + ">Daily</option>";
    html += "<option value='2' " + String(s.timeType == TIME_WEEKLY ? "selected" : "") + ">Weekly</option>";
    html += "<option value='3' " + String(s.timeType == TIME_INTERVAL ? "selected" : "") + ">Every X</option>";
    html += "</select></div>";
    html += "<div id='s" + idx + "_time_once'>";
    html += "<div class='time-row'><label>Date</label><input name='s" + idx + "_date' type='date' value='" + String(s.timeYear) + "-" + (s.timeMonth < 10 ? "0" : "") + String(s.timeMonth) + "-" + (s.timeDay < 10 ? "0" : "") + String(s.timeDay) + "'></div>";
    html += "<div class='time-row'><label>Time</label><input name='s" + idx + "_time' type='time' step='1' value='" + (s.timeHour < 10 ? "0" : "") + String(s.timeHour) + ":" + (s.timeMinute < 10 ? "0" : "") + String(s.timeMinute) + ":" + (s.timeSecond < 10 ? "0" : "") + String(s.timeSecond) + "'></div>";
    html += "</div>";
    html += "<div id='s" + idx + "_time_daily'>";
    html += "<div class='time-row'><label>Time</label><input name='s" + idx + "_time_daily' type='time' step='1' value='" + (s.timeHour < 10 ? "0" : "") + String(s.timeHour) + ":" + (s.timeMinute < 10 ? "0" : "") + String(s.timeMinute) + ":" + (s.timeSecond < 10 ? "0" : "") + String(s.timeSecond) + "'></div>";
    html += "</div>";
    html += "<div id='s" + idx + "_time_weekly'>";
    html += "<div class='time-row'><label>Days</label></div>";
    html += "<div class='day-pills'>";
    const char* days[7] = {"Su","Mo","Tu","We","Th","Fr","Sa"};
    for (int d = 0; d < 7; d++) {
      bool checked = (s.weeklyMask & (1 << d)) != 0;
      html += "<label class='day-pill'><input type='checkbox' name='s" + idx + "_w" + String(d) + "' " + String(checked ? "checked" : "") + "> " + days[d] + "</label>";
    }
    html += "</div>";
    html += "<div class='time-row'><label>Time</label><input name='s" + idx + "_time_weekly' type='time' step='1' value='" + (s.timeHour < 10 ? "0" : "") + String(s.timeHour) + ":" + (s.timeMinute < 10 ? "0" : "") + String(s.timeMinute) + ":" + (s.timeSecond < 10 ? "0" : "") + String(s.timeSecond) + "'></div>";
    html += "</div>";
    uint32_t intervalSec = s.intervalSeconds;
    String intervalUnit = (intervalSec % 3600 == 0) ? "hours" : "minutes";
    uint32_t intervalValue = (intervalUnit == "hours") ? (intervalSec / 3600) : (intervalSec / 60);
    if (intervalValue == 0) intervalValue = 1;
    html += "<div id='s" + idx + "_time_interval'>";
    html += "<div class='time-row'><label>Every</label><input name='s" + idx + "_interval_val' type='number' min='1' value='" + String(intervalValue) + "' style='max-width:120px;'> ";
    html += "<select name='s" + idx + "_interval_unit'>";
    html += "<option value='minutes' " + String(intervalUnit == "minutes" ? "selected" : "") + ">minutes</option>";
    html += "<option value='hours' " + String(intervalUnit == "hours" ? "selected" : "") + ">hours</option>";
    html += "</select></div>";
    html += "</div>";
    html += "</div>";
    html += "</div>";

    html += "<div class='card'><b>Output</b><br>";
    for (int o = 0; o < s.outputCount; o++) {
      const SensorConfig::OutputConfig& out = s.outputs[o];
      String oidx = String(o);
      html += "<div class='card' style='background:#f9f9f9;border-color:#ddd;'>";
      html += "<div style='display:flex;align-items:center;justify-content:space-between;gap:8px;'>";
      html += "<b>Output " + String(o + 1) + "</b>";
      if (o > 0) {
        html += "<button type='submit' name='action' value='out_remove:" + idx + ":" + oidx + "' class='icon-btn'>×</button>";
      }
      html += "</div>";
      html += "<input type='hidden' name='s" + idx + "_o" + oidx + "_present' value='1'>";
      html += "Output: <select name='s" + idx + "_o" + oidx + "_tgt' id='s" + idx + "_o" + oidx + "_tgt'>";
      html += "<option value='0' " + String(out.target == OUT_TARGET_OSC ? "selected" : "") + ">OSC Out</option>";
      html += "<option value='3' " + String(out.target == OUT_TARGET_UDP ? "selected" : "") + ">UDP Raw</option>";
      html += "<option value='4' " + String(out.target == OUT_TARGET_GPIO ? "selected" : "") + ">GPIO Out</option>";
      html += "<option value='7' " + String(out.target == OUT_TARGET_DMX ? "selected" : "") + ">DMX Net Out</option>";
      html += "<option value='8' " + String(out.target == OUT_TARGET_DMX_PRESET ? "selected" : "") + ">DMX Preset</option>";
      html += "<option value='5' " + String(out.target == OUT_TARGET_HTTP ? "selected" : "") + ">HTTP Webhook</option>";
      html += "<option value='6' " + String(out.target == OUT_TARGET_MQTT ? "selected" : "") + ">MQTT Out</option>";
      html += "<option value='2' " + String(out.target == OUT_TARGET_RESET_COOLDOWNS ? "selected" : "") + ">Reset Cooldowns</option>";
      html += "<option value='1' " + String(out.target == OUT_TARGET_REBOOT ? "selected" : "") + ">Reboot Device</option>";
      html += "</select><br>";
      html += "<div id='s" + idx + "_o" + oidx + "_dev_block'>";
      html += "Device: <select name='s" + idx + "_o" + oidx + "_dev' id='s" + idx + "_o" + oidx + "_dev'>";
      appendOscDeviceOptions(html, out.device);
      html += "</select><br>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_osc_out'>";
      html += "<div id='s" + idx + "_o" + oidx + "_addr_block'>";
      html += "OSC address: <input id='s" + idx + "_o" + oidx + "_addr' name='s" + idx + "_o" + oidx + "_addr' type='text' value='" + out.oscAddress + "'><br>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_enc_addr'>";
      html += "Encoder address: <input name='s" + idx + "_o" + oidx + "_enc_addr' type='text' value='" + out.oscAddress + "'><br>";
      html += "</div>";
      html += "Argument type: <select name='s" + idx + "_o" + oidx + "_omode' id='s" + idx + "_o" + oidx + "_omode'>";
      html += "<option value='0' " + String(out.outMode == OUT_INT ? "selected" : "") + ">Int</option>";
      html += "<option value='1' " + String(out.outMode == OUT_FLOAT ? "selected" : "") + ">Float</option>";
      html += "<option value='2' id='s" + idx + "_o" + oidx + "_opt_string' " + String(out.outMode == OUT_STRING ? "selected" : "") + ">String</option>";
      html += "</select><br>";
      html += "<div id='s" + idx + "_o" + oidx + "_minrel_slot_main'></div>";
      html += "<div id='s" + idx + "_o" + oidx + "_minrel'>";
      html += "Send max on press only: <input id='s" + idx + "_o" + oidx + "_smin' name='s" + idx + "_o" + oidx + "_smin' type='checkbox' " + String(!out.sendMinOnRelease ? "checked" : "") + "><br>";
      html += "</div>";
      String mainStrStyle = (s.type == SENSOR_ENCODER || out.outMode != OUT_STRING) ? " style='display:none;'" : "";
      html += "<div id='s" + idx + "_o" + oidx + "_string'" + mainStrStyle + ">";
      html += "<div id='s" + idx + "_o" + oidx + "_offrow'>Off string: <input name='s" + idx + "_o" + oidx + "_off' type='text' value='" + out.offString + "'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='min' onclick='sendOutputTest(" + idx + "," + oidx + ",\"min\");return false;'>Test</button><br></div>";
      html += "<div id='s" + idx + "_o" + oidx + "_onrow'>On string: <input name='s" + idx + "_o" + oidx + "_on' type='text' value='" + out.onString + "'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='max' onclick='sendOutputTest(" + idx + "," + oidx + ",\"max\");return false;'>Test</button><br></div>";
      html += "</div>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_range'>";
      bool hideMainMin = (s.type == SENSOR_ENCODER) || (out.outMode == OUT_STRING) || (!out.sendMinOnRelease);
      String mainMinStyle = hideMainMin ? " style='display:none;'" : "";
      html += "<div id='s" + idx + "_o" + oidx + "_minrow'" + mainMinStyle + ">Output min: <input name='s" + idx + "_o" + oidx + "_omin' type='number' step='0.001' value='" + String(out.outMin, 3) + "'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='min' onclick='sendOutputTest(" + idx + "," + oidx + ",\"min\");return false;'>Test</button><br></div>";
      html += "<span id='s" + idx + "_o" + oidx + "_omax_label'>Output max</span>: <input name='s" + idx + "_o" + oidx + "_omax' type='number' step='0.001' value='" + String(out.outMax, 3) + "'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='max' onclick='sendOutputTest(" + idx + "," + oidx + ",\"max\");return false;'>Test</button><br>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_btn_block'>";
      html += "<div id='s" + idx + "_o" + oidx + "_button_addr'>";
      html += "Button address: <input name='s" + idx + "_o" + oidx + "_btn_addr' type='text' value='" + out.buttonAddress + "'><br>";
      html += "</div>";
      html += "Button argument type: <select name='s" + idx + "_o" + oidx + "_bmode' id='s" + idx + "_o" + oidx + "_bmode'>";
      html += "<option value='0' " + String(out.buttonOutMode == OUT_INT ? "selected" : "") + ">Int</option>";
      html += "<option value='1' " + String(out.buttonOutMode == OUT_FLOAT ? "selected" : "") + ">Float</option>";
      html += "<option value='2' " + String(out.buttonOutMode == OUT_STRING ? "selected" : "") + ">String</option>";
      html += "</select><br>";
      html += "<div id='s" + idx + "_o" + oidx + "_minrel_slot_btn'></div>";
      String bStrStyle = (out.buttonOutMode == OUT_STRING) ? "" : " style='display:none;'";
      html += "<div id='s" + idx + "_o" + oidx + "_bstring'" + bStrStyle + ">";
      html += "<div id='s" + idx + "_o" + oidx + "_boffrow'>Off string: <input name='s" + idx + "_o" + oidx + "_boff' type='text' value='" + out.buttonOffString + "'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='bmin' onclick='sendOutputTest(" + idx + "," + oidx + ",\"bmin\");return false;'>Test</button><br></div>";
      html += "<div id='s" + idx + "_o" + oidx + "_bonrow'>On string: <input name='s" + idx + "_o" + oidx + "_bon' type='text' value='" + out.buttonOnString + "'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='bmax' onclick='sendOutputTest(" + idx + "," + oidx + ",\"bmax\");return false;'>Test</button><br></div>";
      html += "</div>";
      String bRangeStyle = (out.buttonOutMode == OUT_STRING) ? " style='display:none;'" : "";
      html += "<div id='s" + idx + "_o" + oidx + "_brange'" + bRangeStyle + ">";
      bool hideBtnMin = (!out.sendMinOnRelease) || (out.buttonOutMode == OUT_STRING);
      String bMinStyle = hideBtnMin ? " style='display:none;'" : "";
      html += "<div id='s" + idx + "_o" + oidx + "_bminrow'" + bMinStyle + ">Output min: <input name='s" + idx + "_o" + oidx + "_bmin' type='number' step='0.001' value='" + String(out.buttonOutMin, 3) + "'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='bmin' onclick='sendOutputTest(" + idx + "," + oidx + ",\"bmin\");return false;'>Test</button><br></div>";
      html += "Output max: <input name='s" + idx + "_o" + oidx + "_bmax' type='number' step='0.001' value='" + String(out.buttonOutMax, 3) + "'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='bmax' onclick='sendOutputTest(" + idx + "," + oidx + ",\"bmax\");return false;'>Test</button><br>";
      html += "</div>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_udp_block'>";
      html += "UDP payload: <input name='s" + idx + "_o" + oidx + "_udp' type='text' value='" + out.udpPayload + "'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='udp' onclick='sendOutputTest(" + idx + "," + oidx + ",\"udp\");return false;'>Test</button><br>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_dmx_block'>";
      html += "Protocol: <select name='s" + idx + "_o" + oidx + "_dmxp' id='s" + idx + "_o" + oidx + "_dmxp'>";
      html += "<option value='0' " + String(out.dmxProtocol == DMX_PROTO_ARTNET ? "selected" : "") + ">Art-Net</option>";
      html += "<option value='1' " + String(out.dmxProtocol == DMX_PROTO_SACN ? "selected" : "") + ">sACN (E1.31)</option>";
      html += "</select><br>";
      html += "Destination: <select name='s" + idx + "_o" + oidx + "_dmxd' id='s" + idx + "_o" + oidx + "_dmxd'>";
      html += "<option value='0' " + String(out.dmxDest == DMX_DEST_AUTO ? "selected" : "") + ">Auto (Broadcast/Multicast)</option>";
      html += "<option value='1' " + String(out.dmxDest == DMX_DEST_UNICAST ? "selected" : "") + ">Unicast (use Network client)</option>";
      html += "</select><br>";
      html += "<div id='s" + idx + "_o" + oidx + "_dmx_artnet'>";
      html += "Net: <input name='s" + idx + "_o" + oidx + "_dmxn' type='number' min='0' max='127' value='" + String(out.dmxArtNet) + "' style='max-width:90px;'><br>";
      html += "Subnet: <input name='s" + idx + "_o" + oidx + "_dmxs' type='number' min='0' max='15' value='" + String(out.dmxArtSubnet) + "' style='max-width:90px;'><br>";
      html += "Universe: <input name='s" + idx + "_o" + oidx + "_dmxa' type='number' min='0' max='15' value='" + String(out.dmxArtUniverse) + "' style='max-width:90px;'><br>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_dmx_sacn'>";
      html += "Universe: <input name='s" + idx + "_o" + oidx + "_dmxu' type='number' min='1' max='63999' value='" + String(out.dmxUniverse) + "' style='max-width:120px;'><br>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_dmx_channels'>";
      html += "Channels: <input name='s" + idx + "_o" + oidx + "_dmxc' type='text' value='" + out.dmxChannels + "' placeholder='1,5,10-20'><br>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_dmx_preset'>";
      html += "Preset: <input name='s" + idx + "_o" + oidx + "_dmxpr' type='number' min='1' max='" + String(MAX_DMX_PRESETS) + "' value='" + String(out.dmxPreset) + "' style='max-width:120px;'><br>";
      html += "Fade ms: <input name='s" + idx + "_o" + oidx + "_dmxfd' type='number' min='0' max='600000' value='" + String(out.dmxFadeMs) + "' style='max-width:140px;'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='dmxpreset' onclick='sendOutputTest(" + idx + "," + oidx + ",\"dmxpreset\");return false;'>Test</button><br>";
      html += "</div>";
      html += "<div class='helper'>Art-Net uses Net/Subnet/Universe (0-based). sACN uses Universe (1-based). Preset editor supports universes 1-" + String(MAX_DMX_PRESET_UNIVERSE) + ".</div>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_http_block'>";
      html += "Method: <select name='s" + idx + "_o" + oidx + "_hm'>";
      html += "<option value='0' " + String(out.httpMethod == HTTPM_GET ? "selected" : "") + ">GET</option>";
      html += "<option value='1' " + String(out.httpMethod == HTTPM_POST ? "selected" : "") + ">POST</option>";
      html += "</select><br>";
      html += "Network client: <select name='s" + idx + "_o" + oidx + "_hdev' id='s" + idx + "_o" + oidx + "_hdev'>";
      for (int d = 0; d < config.oscDeviceCount; d++) {
        int didx = config.oscDeviceOrder[d];
        const OscDevice& dev = config.oscDevices[didx];
        String label = dev.name + " (" + ipToString(dev.ip) + ":" + String(dev.port) + ")";
        html += "<option value='" + String(didx) + "' data-ip='" + ipToString(dev.ip) + "' data-port='" + String(dev.port) + "' " + String(out.httpDevice == didx ? "selected" : "") + ">" + label + "</option>";
      }
      html += "<option value='255' " + String(out.httpDevice == 255 ? "selected" : "") + ">Custom...</option>";
      html += "</select><br>";
      html += "Host/IP: <input name='s" + idx + "_o" + oidx + "_hip' type='text' value='" + out.httpIp + "' placeholder='wled.local'><br>";
      html += "Port: <input name='s" + idx + "_o" + oidx + "_hpt' type='number' value='" + String(out.httpPort) + "' style='max-width:120px;'><br>";
      html += "URL: <input name='s" + idx + "_o" + oidx + "_hu' type='text' value='" + out.httpUrl + "' placeholder='http://{ip}:{port}/win&A={value}'><br>";
      html += "<div id='s" + idx + "_o" + oidx + "_http_body'>Body: <input name='s" + idx + "_o" + oidx + "_hb' type='text' value='" + out.httpBody + "' placeholder='{value}'></div>";
      html += "<div class='helper'>Tokens: {value}, {ip}, {port} (case-insensitive). Example: http://{ip}/win&A={value} (0-255).</div>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_mqtt_block'>";
      html += "MQTT topic: <input name='s" + idx + "_o" + oidx + "_mt' type='text' value='" + out.mqttTopic + "' placeholder='stagemod/{TriggerName}/output/1'><br>";
      html += "MQTT payload: <input name='s" + idx + "_o" + oidx + "_mp' type='text' value='" + out.mqttPayload + "' placeholder='{value}'><button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='mqtt' onclick='sendOutputTest(" + idx + "," + oidx + ",\"mqtt\");return false;'>Test</button><br>";
      html += "Retain: <input name='s" + idx + "_o" + oidx + "_mr' type='checkbox' " + String(out.mqttRetain ? "checked" : "") + "><br>";
      html += "<div class='helper'>Tokens: {value}, {norm}, {state}</div>";
      html += "</div>";
      html += "<div id='s" + idx + "_o" + oidx + "_gpio_block'>";
      html += "GPIO pin: <input name='s" + idx + "_o" + oidx + "_gpin' type='number' min='0' max='39' value='" + String(out.gpioPin) + "' style='max-width:100px;'> ";
      html += "<small>Output pins allowed: 5,13,16,17,18,19,21,22,23,25,26,27,32,33 (use a pin not used by inputs)</small><br>";
      html += "Mode: <select name='s" + idx + "_o" + oidx + "_gmode' id='s" + idx + "_o" + oidx + "_gmode'>";
      html += "<option value='0' " + String(out.gpioMode == GPIO_OUT_HIGH ? "selected" : "") + ">Level</option>";
      html += "<option value='2' " + String(out.gpioMode == GPIO_OUT_PULSE ? "selected" : "") + ">Pulse</option>";
      html += "<option value='3' " + String(out.gpioMode == GPIO_OUT_PWM ? "selected" : "") + ">PWM</option>";
      html += "</select><br>";
      html += "Invert: <input type='checkbox' name='s" + idx + "_o" + oidx + "_ginv' " + String(out.gpioInvert ? "checked" : "") + "><br>";
      html += "<div id='s" + idx + "_o" + oidx + "_gpulse'>Pulse ms: <input name='s" + idx + "_o" + oidx + "_gpms' type='number' min='1' value='" + String(out.gpioPulseMs) + "' style='max-width:120px;'></div>";
      html += "<div class='helper'>Level: LOW by default, goes HIGH while active. Invert flips it. Pulse: LOW then HIGH for duration (Invert pulses LOW). PWM uses output min/max as duty (0-255); Invert reverses duty.</div>";
      html += "<button type='button' class='icon-btn' data-idx='" + idx + "' data-oidx='" + oidx + "' data-test='gpio' onclick='sendOutputTest(" + idx + "," + oidx + ",\"gpio\");return false;'>Test</button><br>";
      html += "</div>";
      html += "</div>";
    }
    html += "<button type='submit' name='action' value='out_add:" + idx + "' style='padding:6px 10px;font-size:13px;border:1px solid #b9b9b9;background:#fff;color:#222;border-radius:8px;cursor:pointer;'>Add Output</button>";
    html += "</div>";

    html += "</div></details>";
  }

  html += "<button type='submit' name='action' value='add' style='padding:8px 12px;font-size:14px;border:1px solid #b9b9b9;background:#fff;color:#222;border-radius:8px;cursor:pointer;'>Add Trigger</button>";
  html += "<div style='height:14px;'></div>";
  html += "<button type='submit' name='action' value='save' class='primary'>Save</button>";
  html += "<script>";
  html += "function updateTrigger(i){";
  html += "const srcSel=document.getElementById('s'+i+'_src');";
  html += "const tSel=document.getElementById('s'+i+'_type');";
  html += "const typeBlock=document.getElementById('s'+i+'_type_block');";
  html += "const oscinBlock=document.getElementById('s'+i+'_oscin');";
  html += "const ositSel=document.getElementById('s'+i+'_osit');";
  html += "const osimSel=document.getElementById('s'+i+'_osim');";
  html += "const osivRow=document.getElementById('s'+i+'_osiv_row');";
  html += "const osirRow=document.getElementById('s'+i+'_osir_row');";
  html += "const osisRow=document.getElementById('s'+i+'_osis_row');";
  html += "const ttSel=document.getElementById('s'+i+'_tt');";
  html += "const src=srcSel?srcSel.value:'0';";
  html += "const t=tSel? tSel.value : '0';";
  html += "const a=document.getElementById('s'+i+'_analog');";
  html += "const d=document.getElementById('s'+i+'_digital');";
  html += "const pin=document.getElementById('s'+i+'_pin_block');";
  html += "const enc=document.getElementById('s'+i+'_encoder');";
  html += "const hc=document.getElementById('s'+i+'_hcsr04');";
  html += "const cd=document.getElementById('s'+i+'_cooldown');";
  html += "const timeBlock=document.getElementById('s'+i+'_time_block');";
  html += "const timeOnce=document.getElementById('s'+i+'_time_once');";
  html += "const timeDaily=document.getElementById('s'+i+'_time_daily');";
  html += "const timeWeekly=document.getElementById('s'+i+'_time_weekly');";
  html += "const timeInterval=document.getElementById('s'+i+'_time_interval');";
  html += "const isSensor=(src==='0');";
  html += "const isTime=(src==='1');";
  html += "const isOscIn=(src==='2');";
  html += "const isEnc=(isSensor && t==='4');";
  html += "if(typeBlock){typeBlock.style.display=isSensor?'block':'none';}";
  html += "if(oscinBlock){oscinBlock.style.display=isOscIn?'block':'none';}";
  html += "if(isOscIn){";
  html += "let mt=osimSel?osimSel.value:'0';";
  html += "const at=ositSel?ositSel.value:'0';";
  html += "if(at==='3' && mt==='2' && osimSel){osimSel.value='1';mt='1';}";
  html += "if(osivRow){osivRow.style.display=(mt==='1' && at!=='3')?'block':'none';}";
  html += "if(osirRow){osirRow.style.display=(mt==='2' && at!=='3')?'block':'none';}";
  html += "if(osisRow){osisRow.style.display=(mt==='1' && at==='3')?'block':'none';}";
  html += "}";
  html += "if(timeBlock){timeBlock.style.display=isTime?'block':'none';}";
  html += "if(ttSel){";
  html += "const tt=ttSel.value;";
  html += "if(timeOnce) timeOnce.style.display=(tt==='0')?'block':'none';";
  html += "if(timeDaily) timeDaily.style.display=(tt==='1')?'block':'none';";
  html += "if(timeWeekly) timeWeekly.style.display=(tt==='2')?'block':'none';";
  html += "if(timeInterval) timeInterval.style.display=(tt==='3')?'block':'none';";
  html += "}";
  html += "if(a){a.style.display=(isSensor && t==='0')?'block':'none';}";
  html += "if(d){d.style.display=(isSensor && t!=='0'&&!isEnc)?'block':'none';}";
  html += "if(pin){pin.style.display=(isSensor && !isEnc && t!=='9')?'block':'none';}";
  html += "if(enc){enc.style.display=(isSensor && isEnc)?'block':'none';}";
  html += "if(hc){hc.style.display=(isSensor && t==='9')?'block':'none';}";
  html += "if(cd){cd.style.display=(isSensor && (t==='1' || t==='8'))?'block':'none';}";
  html += "for(let o=0;o<" + String(MAX_OUTPUTS_PER_TRIGGER) + ";o++){";
  html += "const tgt=document.getElementById('s'+i+'_o'+o+'_tgt');";
  html += "if(!tgt) continue;";
  html += "const oscOut=document.getElementById('s'+i+'_o'+o+'_osc_out');";
  html += "const devBlock=document.getElementById('s'+i+'_o'+o+'_dev_block');";
  html += "const udpBlock=document.getElementById('s'+i+'_o'+o+'_udp_block');";
  html += "const gpioBlock=document.getElementById('s'+i+'_o'+o+'_gpio_block');";
  html += "const dmxBlock=document.getElementById('s'+i+'_o'+o+'_dmx_block');";
  html += "const dmxProto=document.getElementById('s'+i+'_o'+o+'_dmxp');";
  html += "const dmxDest=document.getElementById('s'+i+'_o'+o+'_dmxd');";
  html += "const dmxArt=document.getElementById('s'+i+'_o'+o+'_dmx_artnet');";
  html += "const dmxSacn=document.getElementById('s'+i+'_o'+o+'_dmx_sacn');";
  html += "const dmxChannels=document.getElementById('s'+i+'_o'+o+'_dmx_channels');";
  html += "const dmxPreset=document.getElementById('s'+i+'_o'+o+'_dmx_preset');";
  html += "const httpBlock=document.getElementById('s'+i+'_o'+o+'_http_block');";
  html += "const mqttBlock=document.getElementById('s'+i+'_o'+o+'_mqtt_block');";
  html += "const httpBody=document.getElementById('s'+i+'_o'+o+'_http_body');";
  html += "const httpDev=document.getElementById('s'+i+'_o'+o+'_hdev');";
  html += "const httpIp=document.querySelector(\"input[name='s\"+i+\"_o\"+o+\"_hip']\");";
  html += "const httpPort=document.querySelector(\"input[name='s\"+i+\"_o\"+o+\"_hpt']\");";
  html += "const gmode=document.getElementById('s'+i+'_o'+o+'_gmode');";
  html += "const gpulse=document.getElementById('s'+i+'_o'+o+'_gpulse');";
  html += "const addrBlock=document.getElementById('s'+i+'_o'+o+'_addr_block');";
  html += "const encAddr=document.getElementById('s'+i+'_o'+o+'_enc_addr');";
  html += "const btnAddr=document.getElementById('s'+i+'_o'+o+'_button_addr');";
  html += "const btnBlock=document.getElementById('s'+i+'_o'+o+'_btn_block');";
  html += "const om=document.getElementById('s'+i+'_o'+o+'_omode');";
  html += "const opt=document.getElementById('s'+i+'_o'+o+'_opt_string');";
  html += "const range=document.getElementById('s'+i+'_o'+o+'_range');";
  html += "const minRow=document.getElementById('s'+i+'_o'+o+'_minrow');";
  html += "const minRel=document.getElementById('s'+i+'_o'+o+'_minrel');";
  html += "const minRelSlotMain=document.getElementById('s'+i+'_o'+o+'_minrel_slot_main');";
  html += "const minRelSlotBtn=document.getElementById('s'+i+'_o'+o+'_minrel_slot_btn');";
  html += "const smin=document.getElementById('s'+i+'_o'+o+'_smin');";
  html += "const str=document.getElementById('s'+i+'_o'+o+'_string');";
  html += "const onRow=document.getElementById('s'+i+'_o'+o+'_onrow');";
  html += "const offRow=document.getElementById('s'+i+'_o'+o+'_offrow');";
  html += "const bmode=document.getElementById('s'+i+'_o'+o+'_bmode');";
  html += "const bstring=document.getElementById('s'+i+'_o'+o+'_bstring');";
  html += "const bonRow=document.getElementById('s'+i+'_o'+o+'_bonrow');";
  html += "const boffRow=document.getElementById('s'+i+'_o'+o+'_boffrow');";
  html += "const brange=document.getElementById('s'+i+'_o'+o+'_brange');";
  html += "const bminrow=document.getElementById('s'+i+'_o'+o+'_bminrow');";
  html += "const omaxLabel=document.getElementById('s'+i+'_o'+o+'_omax_label');";
  html += "const ominInput=document.querySelector(\"input[name='s\"+i+\"_o\"+o+\"_omin']\");";
  html += "const omaxInput=document.querySelector(\"input[name='s\"+i+\"_o\"+o+\"_omax']\");";
  html += "const isOsc=(tgt.value==='0');";
  html += "const isUdp=(tgt.value==='3');";
  html += "const isGpio=(tgt.value==='4');";
  html += "const isDmx=(tgt.value==='7');";
  html += "const isDmxPreset=(tgt.value==='8');";
  html += "const isDmxLike=(isDmx||isDmxPreset);";
  html += "const dmxIsSacn=(isDmxLike && dmxProto && dmxProto.value==='1');";
  html += "const dmxUnicast=(isDmxLike && dmxDest && dmxDest.value==='1');";
  html += "const isHttp=(tgt.value==='5');";
  html += "const isMqtt=(tgt.value==='6');";
  html += "if(oscOut){oscOut.style.display=(isOsc||isHttp||isMqtt||isDmx)?'block':'none';}";
  html += "if(devBlock){devBlock.style.display=(isOsc||isUdp||dmxUnicast)?'block':'none';}";
  html += "if(udpBlock){udpBlock.style.display=isUdp?'block':'none';}";
  html += "if(gpioBlock){gpioBlock.style.display=isGpio?'block':'none';}";
  html += "if(dmxBlock){dmxBlock.style.display=isDmxLike?'block':'none';}";
  html += "if(dmxArt){dmxArt.style.display=(isDmxLike && !dmxIsSacn)?'block':'none';}";
  html += "if(dmxSacn){dmxSacn.style.display=(isDmxLike && dmxIsSacn)?'block':'none';}";
  html += "if(dmxChannels){dmxChannels.style.display=isDmx?'block':'none';}";
  html += "if(dmxPreset){dmxPreset.style.display=isDmxPreset?'block':'none';}";
  html += "if(httpBlock){httpBlock.style.display=isHttp?'block':'none';}";
  html += "if(mqttBlock){mqttBlock.style.display=isMqtt?'block':'none';}";
  html += "if(omaxLabel){omaxLabel.textContent=isEnc?'Output':'Output max';}";
  html += "if(minRel){";
  html += "if(isEnc){if(minRelSlotBtn){minRelSlotBtn.appendChild(minRel);}}";
  html += "else{if(minRelSlotMain){minRelSlotMain.appendChild(minRel);}}";
  html += "}";
  html += "if(httpBody&&isHttp){";
  html += "const m=document.querySelector(\"select[name='s\"+i+\"_o\"+o+\"_hm']\");";
  html += "if(m){httpBody.style.display=(m.value==='1')?'block':'none';}";
  html += "}";
  html += "if(isHttp && httpDev && httpIp && httpPort){";
  html += "const applyHttpClient=()=>{";
  html += "const opt=httpDev.options[httpDev.selectedIndex];";
  html += "if(!opt){return;}";
  html += "if(opt.value==='255'){httpIp.disabled=false;httpPort.disabled=false;return;}";
  html += "const ip=opt.getAttribute('data-ip')||'';";
  html += "const port=opt.getAttribute('data-port')||'';";
  html += "httpIp.disabled=true;httpPort.disabled=true;";
  html += "if(ip){httpIp.value=ip;}";
  html += "if(port){httpPort.value=port;}";
  html += "};";
  html += "httpDev.onchange=applyHttpClient;applyHttpClient();";
  html += "}";
  html += "if(gpulse&&gmode){gpulse.style.display=(isGpio && gmode.value==='2')?'block':'none';}";
  html += "if(isGpio && gmode && gmode.value==='3' && ominInput && omaxInput){";
  html += "const omin=parseFloat(ominInput.value||'0');";
  html += "const omax=parseFloat(omaxInput.value||'0');";
  html += "if(omax<=100){omaxInput.value='255';}";
  html += "if(omin<0||isNaN(omin)){ominInput.value='0';}";
  html += "}";
  html += "if(om){om.disabled=!(isOsc||isHttp||isMqtt||isDmx);}";
  html += "if(om&&opt){";
  html += "if(!(isOsc||isHttp||isMqtt||isDmx)){opt.disabled=true;}";
  html += "else if(isEnc){";
  html += "opt.disabled=true;";
  html += "if(om.value==='2'){om.value='1';}";
  html += "}else if(isDmx){";
  html += "opt.disabled=true;";
  html += "if(om.value==='2'){om.value='0';}";
  html += "}else if(isSensor && t==='0'){";
  html += "opt.disabled=true;";
  html += "if(om.value==='2'){om.value='1';}";
  html += "}else{";
  html += "opt.disabled=false;";
  html += "}";
  html += "}";
  html += "if(str&&om){";
  html += "const showStr=((isOsc||isHttp||isMqtt||isDmx) && !isEnc && om.value==='2');";
  html += "str.style.display=showStr?'block':'none';";
  html += "if(offRow){offRow.style.display=(showStr && smin && smin.checked)?'none':'block';}";
  html += "}";
  html += "if(range&&om){range.style.display=((isOsc||isHttp||isMqtt||isDmx) && om.value!=='2')?'block':(isGpio && gmode && gmode.value==='3' ? 'block' : 'none');}";
  html += "if(addrBlock){addrBlock.style.display=(isOsc && !isEnc)?'block':'none';}";
  html += "if(encAddr){encAddr.style.display=(isOsc && isEnc)?'block':'none';}";
  html += "if(btnAddr){btnAddr.style.display=(isOsc && isEnc)?'block':'none';}";
  html += "if(btnBlock){btnBlock.style.display=(isOsc && isEnc)?'block':'none';}";
  html += "if(minRel){";
  html += "if(isOsc||isHttp||isMqtt||isDmx){minRel.style.display=(isEnc||!isSensor||t!=='0')?'block':'none';}";
  html += "else{minRel.style.display='none';}";
  html += "}";
  html += "if(minRow&&smin&&om){";
  html += "if(isOsc||isHttp||isMqtt||isDmx){";
  html += "if(isEnc){minRow.style.display='none';}";
  html += "else if(om.value==='2'){minRow.style.display='none';}";
  html += "else{minRow.style.display=smin.checked?'none':'block';}";
  html += "}else if(isGpio && gmode && gmode.value==='3'){minRow.style.display='block';}";
  html += "else{minRow.style.display='none';}";
  html += "}";
  html += "if(bstring&&bmode){";
  html += "const showBStr=(isOsc && isEnc && bmode.value==='2');";
  html += "bstring.style.display=showBStr?'block':'none';";
  html += "if(boffRow){boffRow.style.display=(showBStr && smin && smin.checked)?'none':'block';}";
  html += "}";
  html += "if(brange&&bmode){brange.style.display=(isOsc && isEnc && bmode.value!=='2')?'block':'none';}";
  html += "if(bminrow&&smin&&bmode){";
  html += "if(!(isOsc && isEnc)){bminrow.style.display='none';}";
  html += "else if(bmode.value==='2'){bminrow.style.display='none';}";
  html += "else{bminrow.style.display=smin.checked?'none':'block';}";
  html += "}";
  html += "}";
  html += "}";
  html += "";
  html += "";
  html += "for(let i=0;i<" + String(MAX_TRIGGERS) + ";i++){";
  html += "const tSel=document.getElementById('s'+i+'_type');";
  html += "const srcSel=document.getElementById('s'+i+'_src');";
  html += "const ttSel=document.getElementById('s'+i+'_tt');";
  html += "const ositSel=document.getElementById('s'+i+'_osit');";
  html += "const osimSel=document.getElementById('s'+i+'_osim');";
  html += "if(tSel){tSel.addEventListener('change',()=>updateTrigger(i));}";
  html += "if(srcSel){srcSel.addEventListener('change',()=>updateTrigger(i));}";
  html += "if(ttSel){ttSel.addEventListener('change',()=>updateTrigger(i));}";
  html += "if(ositSel){ositSel.addEventListener('change',()=>updateTrigger(i));}";
  html += "if(osimSel){osimSel.addEventListener('change',()=>updateTrigger(i));}";
  html += "for(let o=0;o<" + String(MAX_OUTPUTS_PER_TRIGGER) + ";o++){";
  html += "const om=document.getElementById('s'+i+'_o'+o+'_omode');";
  html += "if(om){om.addEventListener('change',()=>updateTrigger(i));}";
  html += "const bm=document.getElementById('s'+i+'_o'+o+'_bmode');";
  html += "if(bm){bm.addEventListener('change',()=>updateTrigger(i));}";
  html += "const ot=document.getElementById('s'+i+'_o'+o+'_tgt');";
  html += "if(ot){ot.addEventListener('change',()=>updateTrigger(i));}";
  html += "const smin=document.getElementById('s'+i+'_o'+o+'_smin');";
  html += "if(smin){smin.addEventListener('change',()=>updateTrigger(i));}";
  html += "const gm=document.getElementById('s'+i+'_o'+o+'_gmode');";
  html += "if(gm){gm.addEventListener('change',()=>updateTrigger(i));}";
  html += "const dd=document.getElementById('s'+i+'_o'+o+'_dmxd');";
  html += "if(dd){dd.addEventListener('change',()=>updateTrigger(i));}";
  html += "const dp=document.getElementById('s'+i+'_o'+o+'_dmxp');";
  html += "if(dp){dp.addEventListener('change',()=>updateTrigger(i));}";
  html += "const hm=document.querySelector(\"select[name='s\"+i+\"_o\"+o+\"_hm']\");";
  html += "if(hm){hm.addEventListener('change',()=>updateTrigger(i));}";
  html += "const hd=document.getElementById('s'+i+'_o'+o+'_hdev');";
  html += "if(hd){hd.addEventListener('change',()=>updateTrigger(i));}";
  html += "}";
  html += "updateTrigger(i);";
  html += "}";
  html += "const detailsEls=document.querySelectorAll('details[data-trigger]');";
  html += "const stored=localStorage.getItem('trigger_open');";
  html += "let openIdx=-1; if(stored!==null){openIdx=parseInt(stored,10);}"; 
  html += "if(detailsEls.length){detailsEls.forEach(d=>{const idx=d.getAttribute('data-trigger'); if(openIdx>=0 && idx===String(openIdx)){d.open=true;} else if(openIdx>=0){d.open=false;}});}";
  html += "detailsEls.forEach(d=>{d.addEventListener('toggle',()=>{if(d.open){localStorage.setItem('trigger_open',d.getAttribute('data-trigger'));}});});";
  html += "function sendOutputTest(idx, oidx, kind){";
  html += "const tSel=document.getElementById('s'+idx+'_type');";
  html += "const t=tSel? tSel.value : '0';";
  html += "const isEnc=(t==='4');";
  html += "const tgt=document.getElementById('s'+idx+'_o'+oidx+'_tgt');";
  html += "if(tgt && tgt.value==='3'){";
  html += "const dev=document.getElementById('s'+idx+'_o'+oidx+'_dev');";
  html += "const udp=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_udp']\");";
  html += "const data=new URLSearchParams();";
  html += "if(dev){data.set('test_dev',dev.value);}";
  html += "data.set('payload',udp?udp.value:'');";
  html += "fetch('/send_udp_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString(),credentials:'same-origin'}).then(r=>r.text()).then(t=>console.log('udp test',t)).catch(e=>console.warn('udp test failed',e));";
  html += "return;";
  html += "}";
  html += "if(tgt && tgt.value==='4'){";
  html += "const gpin=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_gpin']\");";
  html += "const gmode=document.getElementById('s'+idx+'_o'+oidx+'_gmode');";
  html += "const gpms=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_gpms']\");";
  html += "const ginv=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_ginv']\");";
  html += "const data=new URLSearchParams();";
  html += "data.set('gpin',gpin?gpin.value:'');";
  html += "data.set('gmode',gmode?gmode.value:'');";
  html += "data.set('gpms',gpms?gpms.value:'');";
  html += "if(ginv&&ginv.checked){data.set('ginv','1');}";
  html += "fetch('/send_gpio_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString(),credentials:'same-origin'}).then(r=>r.text()).then(t=>console.log('gpio test',t)).catch(e=>console.warn('gpio test failed',e));";
  html += "return;";
  html += "}";
  html += "if(tgt && tgt.value==='6'){";
  html += "const mt=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_mt']\");";
  html += "const mp=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_mp']\");";
  html += "const mr=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_mr']\");";
  html += "const data=new URLSearchParams();";
  html += "data.set('topic',mt?mt.value:'');";
  html += "data.set('payload',mp?mp.value:'');";
  html += "if(mr&&mr.checked){data.set('retain','1');}";
  html += "fetch('/send_mqtt_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString(),credentials:'same-origin'}).then(r=>r.text()).then(t=>console.log('mqtt test',t)).catch(e=>console.warn('mqtt test failed',e));";
  html += "return;";
  html += "}";
  html += "if(tgt && tgt.value!=='0'){return;}";
  html += "const addrInput=document.getElementById('s'+idx+'_o'+oidx+'_addr');";
  html += "const encAddrInput=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_enc_addr']\");";
  html += "const btnAddrInput=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_btn_addr']\");";
  html += "const useBtn=(kind==='bmin' || kind==='bmax');";
  html += "const addr=useBtn?(btnAddrInput?btnAddrInput.value:''):(isEnc?(encAddrInput?encAddrInput.value:''):(addrInput?addrInput.value:''));";
  html += "const dev=document.getElementById('s'+idx+'_o'+oidx+'_dev');";
  html += "const om=document.getElementById('s'+idx+'_o'+oidx+'_omode');";
  html += "const onStr=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_on']\");";
  html += "const offStr=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_off']\");";
  html += "const omin=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_omin']\");";
  html += "const omax=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_omax']\");";
  html += "const bm=document.getElementById('s'+idx+'_o'+oidx+'_bmode');";
  html += "const bon=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_bon']\");";
  html += "const boff=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_boff']\");";
  html += "const bmin=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_bmin']\");";
  html += "const bmax=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_bmax']\");";
  html += "if(!addr){return;}";
  html += "const data=new URLSearchParams();";
  html += "data.set('test_addr',addr);";
  html += "if(dev){data.set('test_dev',dev.value);}"; 
  html += "if(useBtn){";
  html += "const mode=bm?bm.value:'0';";
  html += "if(mode==='2'){";
  html += "data.set('test_type','string');";
  html += "const v=(kind==='bmax')?(bon?bon.value:''):(boff?boff.value:'');";
  html += "data.set('test_value',v||'');";
  html += "}else if(mode==='1'){";
  html += "data.set('test_type','float');";
  html += "const v=(kind==='bmax')?(bmax?bmax.value:'0'):(bmin?bmin.value:'0');";
  html += "data.set('test_value',v);";
  html += "}else{";
  html += "data.set('test_type','int');";
  html += "const v=(kind==='bmax')?(bmax?bmax.value:'0'):(bmin?bmin.value:'0');";
  html += "data.set('test_value',v);";
  html += "}";
  html += "}else if(om&&om.value==='2'){";
  html += "data.set('test_type','string');";
  html += "const v=(kind==='max')?(onStr?onStr.value:''):(offStr?offStr.value:'');";
  html += "data.set('test_value',v||'');";
  html += "}else if(om&&om.value==='1'){";
  html += "data.set('test_type','float');";
  html += "const v=(kind==='max')?(omax?omax.value:'0'):(omin?omin.value:'0');";
  html += "data.set('test_value',v);";
  html += "}else{";
  html += "data.set('test_type','int');";
  html += "const v=(kind==='max')?(omax?omax.value:'0'):(omin?omin.value:'0');";
  html += "data.set('test_value',v);";
  html += "}";
  html += "fetch('/send_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString(),credentials:'same-origin'}).then(r=>r.text()).then(t=>console.log('test send',t)).catch(e=>console.warn('test send failed',e));";
  html += "}";
  html += "</script>";
  html += "</form></div></body></html>";
  server.send(200, "text/html", html);
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

static void handleSettings() {
  if (!ensureAuthenticated()) return;
  String html;
  html.reserve(22000);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>StageMod Settings</title>";
  html += "<style>";
  html += ":root{--bg:#f4f1ea;--ink:#1a1a1a;--muted:#6c6c6c;--card:#ffffff;--accent:#0b6b6f;--danger:#a02a2a;}";
  html += "body{margin:0;font-family:'Avenir Next','Trebuchet MS','Segoe UI',sans-serif;background:linear-gradient(135deg,#f4f1ea 0%,#e7f0ef 100%);color:var(--ink);}";
  html += ".page{max-width:900px;margin:0 auto;padding:20px 16px 40px;}";
  html += ".header{display:flex;flex-wrap:wrap;gap:12px;align-items:center;justify-content:space-between;margin-bottom:8px;}";
  html += ".title{font-size:22px;font-weight:700;letter-spacing:0.5px;}";
  html += ".actions{display:flex;gap:10px;align-items:center;flex-wrap:wrap;}";
  html += ".nav-link{font-size:16px;color:#6b34d6;text-decoration:underline;font-weight:600;}";
  html += ".nav-pill{font-size:14px;color:#666;padding:6px 12px;border:1px solid #ddd;border-radius:999px;background:#fff;text-decoration:none;}";
  html += ".nav-primary{font-size:16px;color:#fff;padding:8px 14px;border-radius:999px;background:var(--accent);text-decoration:none;}";
  html += ".card{background:var(--card);border:1px solid #e2e2e2;border-radius:12px;padding:12px;margin:10px 0;}";
  html += "fieldset{border:1px solid #e2e2e2;border-radius:12px;padding:12px;margin:10px 0;background:#fff;}";
  html += "legend{padding:0 6px;color:var(--muted);}";
  html += "details{border:1px solid #e2e2e2;border-radius:12px;margin:10px 0;background:#fff;}";
  html += "summary{cursor:pointer;list-style:none;padding:10px 12px;display:flex;align-items:center;justify-content:space-between;}";
  html += "summary::-webkit-details-marker{display:none;}";
  html += ".chev{display:inline-block;margin-right:8px;font-size:14px;transition:transform 0.2s ease;}";
  html += "details[open] .chev{transform:rotate(90deg);}";
  html += ".trigger-title{font-weight:600;}";
  html += ".toggle{position:relative;width:36px;height:20px;display:inline-block;}";
  html += ".toggle input{opacity:0;width:0;height:0;}";
  html += ".toggle span{position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#c9c9c9;border-radius:999px;transition:0.2s;}";
  html += ".toggle span:before{content:'';position:absolute;height:16px;width:16px;left:2px;top:2px;background:#fff;border-radius:50%;transition:0.2s;}";
  html += ".toggle input:checked + span{background:#0b6b6f;}";
  html += ".toggle input:checked + span:before{transform:translateX(16px);}";
  html += ".toggle input:disabled + span{background:#b9b9b9;cursor:default;}";
  html += ".trigger-actions{display:flex;gap:6px;align-items:center;}";
  html += ".icon-btn{border:1px solid #b9b9b9;background:#fff;color:#222;padding:4px 8px;border-radius:6px;cursor:pointer;}";
  html += "input,select{width:100%;max-width:320px;padding:6px 8px;margin:2px 0 8px;border:1px solid #cfcfcf;border-radius:6px;background:#fff;}";
  html += "button{border:1px solid #b9b9b9;background:#fff;color:#222;padding:6px 10px;border-radius:8px;cursor:pointer;}";
  html += "button.primary{background:var(--accent);color:#fff;border-color:var(--accent);}";
  html += "button.danger{background:var(--danger);color:#fff;border-color:var(--danger);}";
  html += "small{color:var(--muted);}";
  html += "</style></head><body>";
  html += "<div class='page'>";
  html += "<div class='header'>";
  html += "<div class='title'>StageMod Settings</div>";
  html += "<div class='actions'>";
  html += "<a href='https://github.com/OLIMEX/ESP32-POE/blob/master/DOCUMENTS/ESP32-POE-PINOUT.png' class='nav-pill'>Board Pin Help</a>";
  html += "<a href='/logs' class='nav-pill'>Serial Log</a>";
  html += "<a href='/' class='nav-primary'>Triggers</a>";
  html += "</div></div>";

  html += "<form method='POST' action='/settings'>";
  html += "<div class='card'><b>Network</b><br>";
  html += "Hostname: <input name='hostname' type='text' value='" + config.hostname + "'><br>";
  html += "Network: <select name='net_mode' id='net_mode'>";
  html += "<option value='0' " + String(config.netMode == NET_ETHERNET ? "selected" : "") + ">Ethernet</option>";
  html += "<option value='1' " + String(config.netMode == NET_WIFI ? "selected" : "") + ">WiFi</option>";
  html += "</select><br>";
  html += "<div id='wifi_fields'>";
  html += "WiFi SSID: ";
  html += "<select id='wifi_ssid_select' name='wifi_ssid_select' style='display:none;'><option value=''>Scan to list networks...</option></select> ";
  html += "<button type='button' id='scan_wifi'>Scan WiFi</button><br>";
  html += "<input id='wifi_ssid' name='wifi_ssid' type='text' value='" + config.wifiSsid + "' placeholder='Or type SSID manually'><br>";
  html += "WiFi pass: <input name='wifi_pass' type='password' value=''><br>";
  html += "</div>";
  html += "<hr>";
  html += "DHCP: <input name='dhcp_enabled' id='dhcp_enabled' type='checkbox' " + String(config.useStatic ? "" : "checked") + "><br>";
  html += "<div id='static_fields'>";
  html += "Static IP addr: <input name='static_ip' type='text' value='" + ipToString(config.staticIp) + "'><br>";
  html += "Gateway: <input name='gateway' type='text' value='" + ipToString(config.gateway) + "'><br>";
  html += "Subnet: <input name='subnet' type='text' value='" + ipToString(config.subnet) + "'><br>";
  html += "DNS1: <input name='dns1' type='text' value='" + ipToString(config.dns1) + "'><br>";
  html += "DNS2: <input name='dns2' type='text' value='" + ipToString(config.dns2) + "'><br>";
  html += "</div>";
  html += "<small>Static IP changes may require reboot.</small><br>";
  html += "</div>";

  html += "<div class='card'><b>OSC Input</b><br>";
  html += "Enable OSC input triggers: <input name='osci_en' type='checkbox' " + String(config.oscInEnabled ? "checked" : "") + "><br>";
  html += "Listen port: <input name='osci_port' type='number' min='1' max='65535' value='" + String(config.oscInPort) + "' style='max-width:140px;'><br>";
  html += "<small>Trigger-level OSC matching is configured on the Triggers page (Source = OSC In).</small><br>";
  html += "</div>";

  html += "<div class='card'><b>DMX Net Output</b><br>";
  html += "Enable Art-Net out: <input name='artnet_en' type='checkbox' " + String(config.artnetEnabled ? "checked" : "") + "><br>";
  html += "Enable sACN out: <input name='sacn_en' type='checkbox' " + String(config.sacnEnabled ? "checked" : "") + "><br>";
  html += "<small>These toggles gate DMX Net Out and DMX Preset playback protocols.</small><br>";
  html += "</div>";

  html += "<div class='card'><b>DMX Input</b><br>";
  html += "Enable Art-Net in: <input name='dmxi_art' type='checkbox' " + String(config.dmxInArtNetEnabled ? "checked" : "") + "><br>";
  html += "Enable sACN in: <input name='dmxi_sac' type='checkbox' " + String(config.dmxInSacnEnabled ? "checked" : "") + "><br>";
  html += "Auto source mode: <select name='dmxi_src'>";
  html += "<option value='0' " + String(config.dmxInSourceMode == DMX_IN_SOURCE_RECENT ? "selected" : "") + ">Most recent wins</option>";
  html += "<option value='1' " + String(config.dmxInSourceMode == DMX_IN_SOURCE_PREFER_ARTNET ? "selected" : "") + ">Prefer Art-Net</option>";
  html += "<option value='2' " + String(config.dmxInSourceMode == DMX_IN_SOURCE_PREFER_SACN ? "selected" : "") + ">Prefer sACN</option>";
  html += "</select><br>";
  html += "<small>Used by DMX preset snapshots in the preset editor when source view is set to Auto.</small><br>";
  html += "</div>";

  int dmxPresetPointsUsed = totalDmxPresetPointsExcluding(-1);
  html += "<div class='card'><b>DMX Presets</b><br>";
  html += "<div>Stored points: <b>" + String(dmxPresetPointsUsed) + "</b> / " + String(MAX_DMX_PRESET_POINTS) + "</div>";
  html += "<small>Presets can span universes 1-" + String(MAX_DMX_PRESET_UNIVERSE) + " and are recalled by DMX Preset outputs.</small><br>";
  html += "<a href='/dmx_presets' class='nav-primary' style='display:inline-block;margin-top:8px;'>Open DMX Preset Editor</a>";
  html += "</div>";

  html += "<div class='card'><b>Network Clients (OSC / UDP / MQTT / HTTP Webhook)</b><br>";
  html += "<small>Device 1 is the default target for OSC/UDP. MQTT broker and HTTP webhook can also select from this list.</small><br>";
  html += "<div style='margin-top:10px;'><b>Network Client Devices</b><br>";

  for (int dIndex = 0; dIndex < config.oscDeviceCount; dIndex++) {
    int i = config.oscDeviceOrder[dIndex];
    OscDevice& d = config.oscDevices[i];
    html += "<details " + String(dIndex == 0 ? "open" : "") + ">";
    html += "<summary>";
    html += "<span style='display:flex;align-items:center;gap:10px;'>";
    html += "<span class='chev'>&#9654;</span>";
    html += "<label class='toggle' title='Enable device'>";
    html += "<input name='dev" + String(i) + "_en' type='checkbox' " + String(d.enabled ? "checked" : "") + ">";
    html += "<span></span></label>";
    html += "<span class='trigger-title'>" + d.name + "</span>";
    html += "</span>";
    html += "<span class='trigger-actions'>";
    html += "<button type='submit' name='action' value='dev_up:" + String(i) + "' class='icon-btn'>&uarr;</button>";
    html += "<button type='submit' name='action' value='dev_down:" + String(i) + "' class='icon-btn'>&darr;</button>";
    if (i != 0) {
      html += "<button type='submit' name='action' value='dev_remove:" + String(i) + "' class='icon-btn'>×</button>";
    }
    html += "</span></summary>";
    html += "<div style='padding:0 12px 12px;'>";
    html += "<input type='hidden' name='d" + String(i) + "_present' value='1'>";
    html += "Name: <input name='dev" + String(i) + "_name' type='text' value='" + d.name + "'><br>";
    html += "IP: <input name='dev" + String(i) + "_ip' type='text' value='" + ipToString(d.ip) + "'><br>";
    html += "Port: <input name='dev" + String(i) + "_port' type='number' value='" + String(d.port) + "'><br>";
    html += "<b>Heartbeat</b><br>";
    html += "Enable: <input name='dev" + String(i) + "_hb_en' type='checkbox' " + String(d.hbEnabled ? "checked" : "") + "><br>";
    html += "Address: <input name='dev" + String(i) + "_hb_addr' type='text' value='" + d.hbAddress + "'><br>";
    html += "Interval ms: <input name='dev" + String(i) + "_hb_ms' type='number' value='" + String(d.hbMs) + "'><br>";
    html += "</div></details>";
  }
  html += "<button type='submit' name='action' value='dev_add'>Add Network Client</button><br><br>";
  html += "</div>";

  html += "<div style='margin-top:10px;'><b>Test OSC/UDP</b><br>";
  html += "Device: <select id='test_dev' name='test_dev'>";
  appendOscDeviceOptions(html, config.testDevice);
  html += "</select><br>";
  html += "Address: <input id='test_addr' name='test_addr' type='text' value='/test'><br>";
  html += "Output type: <select id='test_type' name='test_type'>";
  html += "<option value='int'>Int</option>";
  html += "<option value='float'>Float</option>";
  html += "<option value='string'>String</option>";
  html += "</select><br>";
  html += "Output: <input id='test_value' name='test_value' type='text' value='1'><br>";
  html += "<button type='button' id='test_send_btn'>Send</button>";
  html += "</div>";
  html += "</div>";

  String mqttHostDisplay = config.mqttHost;
  uint16_t mqttPortDisplay = config.mqttPort > 0 ? config.mqttPort : DEFAULT_MQTT_PORT;
  if (config.mqttDevice < MAX_OSC_DEVICES) {
    const OscDevice& md = config.oscDevices[config.mqttDevice];
    mqttHostDisplay = ipToString(md.ip);
    mqttPortDisplay = md.port > 0 ? md.port : DEFAULT_MQTT_PORT;
  }

  html += "<div class='card'><b>MQTT</b><br>";
  html += "Enable MQTT output: <input id='mqtt_en' name='mqtt_en' type='checkbox' " + String(config.mqttEnabled ? "checked" : "") + "><br>";
  html += "<div id='mqtt_fields'>";
  html += "Broker device: <select id='mqtt_dev' name='mqtt_dev'>";
  for (int d = 0; d < config.oscDeviceCount; d++) {
    int didx = config.oscDeviceOrder[d];
    const OscDevice& dev = config.oscDevices[didx];
    uint16_t devPort = dev.port > 0 ? dev.port : DEFAULT_MQTT_PORT;
    String label = dev.name + " (" + ipToString(dev.ip) + ":" + String(devPort) + ")";
    html += "<option value='" + String(didx) + "' data-ip='" + ipToString(dev.ip) + "' data-port='" + String(devPort) + "' " + String(config.mqttDevice == didx ? "selected" : "") + ">" + label + "</option>";
  }
  html += "<option value='" + String(MQTT_DEVICE_CUSTOM) + "' " + String(config.mqttDevice == MQTT_DEVICE_CUSTOM ? "selected" : "") + ">Custom...</option>";
  html += "</select><br>";
  html += "Broker host/IP: <input id='mqtt_host' name='mqtt_host' type='text' value='" + mqttHostDisplay + "' placeholder='192.168.1.10'><br>";
  html += "Broker port: <input id='mqtt_port' name='mqtt_port' type='number' min='1' max='65535' value='" + String(mqttPortDisplay) + "' style='max-width:120px;'><br>";
  html += "Client ID: <input name='mqtt_client_id' type='text' value='" + config.mqttClientId + "'><br>";
  html += "Username: <input name='mqtt_user' type='text' value='" + config.mqttUser + "'><br>";
  html += "Password: <input name='mqtt_pass' type='password' value=''><br>";
  html += "Base topic: <input name='mqtt_base' type='text' value='" + config.mqttBaseTopic + "' placeholder='stagemod'><br>";
  html += "<small>Leave password blank to keep current password.</small><br>";
  html += "<small>Status: " + String(mqttClient.connected() ? "connected" : "disconnected") + "</small><br>";
  html += "</div>";
  html += "</div>";

  html += "<div class='card'><b>Sensors</b><br>";
  html += "<b>Buttons</b><br>";
  html += "Debounce ms: <input name='click_db' type='number' value='" + String(config.clickDebounceMs) + "'><br>";
  html += "Multi-click gap ms: <input name='click_gap' type='number' value='" + String(config.multiClickGapMs) + "'><br>";
  html += "Long press ms: <input name='long_ms' type='number' value='" + String(config.longPressMs) + "'><br><br>";
  html += "<b>Encoder</b><br>";
  html += "<small>Steps per output is fixed at 2 (more sensitive).</small><br><br>";
  html += "<b>Potentiometer / Ultrasonic</b><br>";
  html += "Smoothing: <input name='analog_smooth' type='checkbox' " + String(config.analogSmoothing ? "checked" : "") + "><br>";
  html += "</div>";

  html += "<div class='card'><b>Time</b><br>";
  html += "<div>Current time: " + formatTimeLocal() + "</div>";
  html += "Mode: <select name='time_mode' id='time_mode'>";
  html += "<option value='0' " + String(config.timeMode == TIME_NTP ? "selected" : "") + ">NTP</option>";
  html += "<option value='1' " + String(config.timeMode == TIME_MANUAL ? "selected" : "") + ">Manual</option>";
  html += "</select><br>";
  html += "<div id='ntp_fields'>";
  html += "NTP server: <input name='ntp_srv' type='text' value='" + config.ntpServer + "'><br>";
  html += "</div>";
  html += "Time zone: <select name='tz'>";
  html += "<option value='PST8PDT,M3.2.0/2,M11.1.0/2' " + String(config.timeZone == "PST8PDT,M3.2.0/2,M11.1.0/2" ? "selected" : "") + ">Pacific (PST/PDT)</option>";
  html += "<option value='MST7MDT,M3.2.0/2,M11.1.0/2' " + String(config.timeZone == "MST7MDT,M3.2.0/2,M11.1.0/2" ? "selected" : "") + ">Mountain (MST/MDT)</option>";
  html += "<option value='CST6CDT,M3.2.0/2,M11.1.0/2' " + String(config.timeZone == "CST6CDT,M3.2.0/2,M11.1.0/2" ? "selected" : "") + ">Central (CST/CDT)</option>";
  html += "<option value='EST5EDT,M3.2.0/2,M11.1.0/2' " + String(config.timeZone == "EST5EDT,M3.2.0/2,M11.1.0/2" ? "selected" : "") + ">Eastern (EST/EDT)</option>";
  html += "<option value='AKST9AKDT,M3.2.0/2,M11.1.0/2' " + String(config.timeZone == "AKST9AKDT,M3.2.0/2,M11.1.0/2" ? "selected" : "") + ">Alaska (AKST/AKDT)</option>";
  html += "<option value='HST10' " + String(config.timeZone == "HST10" ? "selected" : "") + ">Hawaii (HST)</option>";
  html += "<option value='UTC0' " + String(config.timeZone == "UTC0" ? "selected" : "") + ">UTC</option>";
  html += "<option value='GMT0' " + String(config.timeZone == "GMT0" ? "selected" : "") + ">GMT</option>";
  html += "</select><br>";
  html += "Time display: <select name='time_24'>";
  html += "<option value='0' " + String(!config.timeDisplay24h ? "selected" : "") + ">12-hour</option>";
  html += "<option value='1' " + String(config.timeDisplay24h ? "selected" : "") + ">24-hour</option>";
  html += "</select><br>";
  html += "<div id='manual_fields'>";
  html += "Date: <input name='manual_date' type='date' value='" + formatDateLocal() + "'><br>";
  html += "Time: <input name='manual_time' type='time' step='1' value='" + formatTimeLocalInput() + "'><br>";
  html += "<button type='submit' name='action' value='time_set'>Set Time</button><br>";
  html += "</div>";
  html += "</div>";

  html += "<div class='card'><b>Security</b><br>";
  html += "Username: <input name='username' type='text' value='" + config.username + "'><br>";
  html += "Enable password: <input name='pwd_enabled' type='checkbox' " + String(config.passwordEnabled ? "checked" : "") + "><br>";
  html += "New password: <input name='pwd_new' type='password' value=''><br>";
  html += "<small>Leave new password blank to keep current.</small><br>";
  html += "</div>";

  html += "<button type='submit' class='primary'>Save Settings</button>";
  html += "</form>";

  html += "<div class='card'><b>Config Import/Export</b><br>";
  html += "<div style='margin-top:6px;'>";
  html += "<button type='button' onclick=\"window.location.href='/export_config'\">Download Config</button>";
  html += "</div>";
  html += "<form method='POST' action='/import_config' enctype='multipart/form-data' style='margin-top:8px;'>";
  html += "<input type='file' name='config' accept='.json,application/json,text/plain' style='margin-right:8px;'><button type='submit'>Import Config</button>";
  html += "</form>";
  html += "<small>Network settings and passwords are not included.</small><br>";
  html += "</div>";

  html += "<div class='card'><b>Update</b><br>";
  html += "<div>Firmware version: <b>" + String(FIRMWARE_VERSION) + "</b></div>";
  html += "<div style='margin-top:8px;'><b>Firmware Update</b><br>";
  html += "<small>Upload a new firmware .bin over your local network. Internet is not required.</small><br>";
  html += "<form method='POST' action='/update' enctype='multipart/form-data' style='margin-top:8px;'>";
  html += "<div style='display:flex;gap:8px;align-items:center;flex-wrap:wrap;'>";
  html += "<input type='file' name='update' accept='.bin' style='margin:0;'>";
  html += "<button type='submit'>Install Update</button>";
  html += "</div>";
  html += "</form></div><br>";
  html += "</div>";

  html += "<div class='card'><b>Device</b><br>";
  html += "<button type='button' id='reboot_btn'>Reboot Device</button> ";
  html += "<button type='button' id='factory_reset_btn' class='danger'>Factory Reset</button>";
  html += "</div>";

  html += "<script>";
  html += "const dh=document.getElementById('dhcp_enabled');";
  html += "const sf=document.getElementById('static_fields');";
  html += "const nm=document.getElementById('net_mode');";
  html += "const wf=document.getElementById('wifi_fields');";
  html += "const tm=document.getElementById('time_mode');";
  html += "const ntp=document.getElementById('ntp_fields');";
  html += "const man=document.getElementById('manual_fields');";
  html += "const mq=document.getElementById('mqtt_en');";
  html += "const mqf=document.getElementById('mqtt_fields');";
  html += "const mqDev=document.getElementById('mqtt_dev');";
  html += "const mqHost=document.getElementById('mqtt_host');";
  html += "const mqPort=document.getElementById('mqtt_port');";
  html += "function toggleDhcp(){sf.style.display=dh.checked?'none':'block';}";
  html += "function toggleNet(){wf.style.display=(nm.value==='1')?'block':'none';}";
  html += "function toggleTime(){if(!tm){return;}ntp.style.display=(tm.value==='0')?'block':'none';man.style.display=(tm.value==='1')?'block':'none';}";
  html += "function toggleMqtt(){if(!mq||!mqf){return;}mqf.style.display=mq.checked?'block':'none';}";
  html += "function applyMqttDevice(){";
  html += "if(!mqDev||!mqHost||!mqPort){return;}";
  html += "const opt=mqDev.options[mqDev.selectedIndex];";
  html += "const custom=(mqDev.value==='255');";
  html += "mqHost.disabled=!custom;";
  html += "mqPort.disabled=!custom;";
  html += "if(custom||!opt){return;}";
  html += "const ip=opt.getAttribute('data-ip')||'';";
  html += "const port=opt.getAttribute('data-port')||'';";
  html += "if(ip){mqHost.value=ip;}";
  html += "if(port){mqPort.value=port;}";
  html += "}";
  html += "dh.addEventListener('change',toggleDhcp);";
  html += "nm.addEventListener('change',toggleNet);";
  html += "if(tm){tm.addEventListener('change',toggleTime);}";
  html += "if(mq){mq.addEventListener('change',toggleMqtt);}";
  html += "if(mqDev){mqDev.addEventListener('change',applyMqttDevice);}";
  html += "toggleDhcp();toggleNet();toggleTime();toggleMqtt();applyMqttDevice();";
  html += "const scanBtn=document.getElementById('scan_wifi');";
  html += "const wifiSelect=document.getElementById('wifi_ssid_select');";
  html += "const wifiInput=document.getElementById('wifi_ssid');";
  html += "function showWifiSelect(){if(wifiSelect){wifiSelect.style.display='inline-block';}}";
  html += "if(wifiSelect&&wifiInput){wifiSelect.addEventListener('change',()=>{";
  html += "const v=wifiSelect.value; if(v==='__custom__'){wifiInput.focus();return;} wifiInput.value=v;});}";
  html += "if(scanBtn&&wifiSelect&&wifiInput){";
  html += "scanBtn.addEventListener('click',()=>{";
  html += "showWifiSelect();";
  html += "wifiSelect.innerHTML='';";
  html += "const opt=document.createElement('option');opt.textContent='Scanning...';opt.value='';wifiSelect.appendChild(opt);";
  html += "fetch('/scan_wifi').then(r=>r.json()).then(list=>{";
  html += "wifiSelect.innerHTML='';";
  html += "if(!list.length){const o=document.createElement('option');o.textContent='No networks found';o.value='';wifiSelect.appendChild(o);return;}";
  html += "list.forEach(item=>{const o=document.createElement('option');o.value=item.ssid;o.textContent=item.ssid+' ('+item.rssi+'dBm)';wifiSelect.appendChild(o);});";
  html += "const o=document.createElement('option');o.value='__custom__';o.textContent='Custom...';wifiSelect.appendChild(o);";
  html += "const current=wifiInput.value; if(current){for(const opt of wifiSelect.options){if(opt.value===current){wifiSelect.value=current;break;}}}";
  html += "if(!current && wifiSelect.options.length){const first=wifiSelect.options[0]; if(first && first.value && first.value!=='__custom__'){wifiSelect.value=first.value; wifiInput.value=first.value;}}";
  html += "}).catch(()=>{wifiSelect.innerHTML='';const o=document.createElement('option');o.textContent='Scan failed';o.value='';wifiSelect.appendChild(o);});";
  html += "});}";
  html += "const testBtn=document.getElementById('test_send_btn');";
  html += "const testAddr=document.getElementById('test_addr');";
  html += "const testType=document.getElementById('test_type');";
  html += "const testValue=document.getElementById('test_value');";
  html += "const testDev=document.getElementById('test_dev');";
  html += "if(testBtn&&testAddr){testBtn.addEventListener('click',()=>{";
  html += "const data=new URLSearchParams();";
  html += "data.set('test_addr',testAddr.value||'/test');";
  html += "if(testType){data.set('test_type',testType.value);}"; 
  html += "if(testValue){data.set('test_value',testValue.value);}"; 
  html += "if(testDev){data.set('test_dev',testDev.value);}"; 
  html += "fetch('/send_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString(),credentials:'same-origin'}).then(r=>r.text()).then(t=>console.log('test send',t)).catch(e=>console.warn('test send failed',e));";
  html += "});}";
  html += "const resetBtn=document.getElementById('factory_reset_btn');";
  html += "const rebootBtn=document.getElementById('reboot_btn');";
  html += "if(resetBtn){resetBtn.addEventListener('click',()=>{if(!confirm('Factory reset will erase all settings. Continue?')){return;}fetch('/reset',{method:'POST'});});}";
  html += "if(rebootBtn){rebootBtn.addEventListener('click',()=>{if(!confirm('Reboot device now?')){return;}fetch('/reboot',{method:'POST'});});}";
  html += "function sendOutputTest(idx, oidx, kind){";
  html += "const tSel=document.getElementById('s'+idx+'_type');";
  html += "const t=tSel? tSel.value : '0';";
  html += "const isEnc=(t==='4');";
  html += "const tgt=document.getElementById('s'+idx+'_o'+oidx+'_tgt');";
  html += "if(tgt && tgt.value==='3'){";
  html += "const dev=document.getElementById('s'+idx+'_o'+oidx+'_dev');";
  html += "const udp=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_udp']\");";
  html += "const data=new URLSearchParams();";
  html += "if(dev){data.set('test_dev',dev.value);}";
  html += "data.set('payload',udp?udp.value:'');";
  html += "fetch('/send_udp_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString(),credentials:'same-origin'}).then(r=>r.text()).then(t=>console.log('udp test',t)).catch(e=>console.warn('udp test failed',e));";
  html += "return;";
  html += "}";
  html += "if(tgt && tgt.value==='4'){";
  html += "const gpin=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_gpin']\");";
  html += "const gmode=document.getElementById('s'+idx+'_o'+oidx+'_gmode');";
  html += "const gpms=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_gpms']\");";
  html += "const ginv=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_ginv']\");";
  html += "const data=new URLSearchParams();";
  html += "data.set('gpin',gpin?gpin.value:'');";
  html += "data.set('gmode',gmode?gmode.value:'');";
  html += "data.set('gpms',gpms?gpms.value:'');";
  html += "if(ginv&&ginv.checked){data.set('ginv','1');}";
  html += "fetch('/send_gpio_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString(),credentials:'same-origin'}).then(r=>r.text()).then(t=>console.log('gpio test',t)).catch(e=>console.warn('gpio test failed',e));";
  html += "return;";
  html += "}";
  html += "if(tgt && tgt.value==='6'){";
  html += "const topic=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_mt']\");";
  html += "const payload=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_mp']\");";
  html += "const retain=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_mr']\");";
  html += "const data=new URLSearchParams();";
  html += "data.set('topic',topic?topic.value:'');";
  html += "data.set('payload',payload?payload.value:'');";
  html += "if(retain&&retain.checked){data.set('retain','1');}";
  html += "fetch('/send_mqtt_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString(),credentials:'same-origin'}).then(r=>r.text()).then(t=>console.log('mqtt test',t)).catch(e=>console.warn('mqtt test failed',e));";
  html += "return;";
  html += "}";
  html += "if(tgt && (tgt.value==='7' || tgt.value==='8')){";
  html += "const dev=document.getElementById('s'+idx+'_o'+oidx+'_dev');";
  html += "const dmxp=document.querySelector(\"select[name='s\"+idx+\"_o\"+oidx+\"_dmxp']\");";
  html += "const dmxd=document.getElementById('s'+idx+'_o'+oidx+'_dmxd');";
  html += "const dmxn=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_dmxn']\");";
  html += "const dmxs=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_dmxs']\");";
  html += "const dmxa=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_dmxa']\");";
  html += "const dmxu=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_dmxu']\");";
  html += "const dmxc=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_dmxc']\");";
  html += "const dmxpr=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_dmxpr']\");";
  html += "const dmxfd=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_dmxfd']\");";
  html += "const om=document.getElementById('s'+idx+'_o'+oidx+'_omode');";
  html += "const omin=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_omin']\");";
  html += "const omax=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_omax']\");";
  html += "const data=new URLSearchParams();";
  html += "data.set('target',tgt.value);";
  html += "if(dev){data.set('test_dev',dev.value);}";
  html += "data.set('dmxp',dmxp?dmxp.value:'0');";
  html += "data.set('dmxd',dmxd?dmxd.value:'0');";
  html += "data.set('dmxn',dmxn?dmxn.value:'0');";
  html += "data.set('dmxs',dmxs?dmxs.value:'0');";
  html += "data.set('dmxa',dmxa?dmxa.value:'0');";
  html += "data.set('dmxu',dmxu?dmxu.value:'1');";
  html += "data.set('dmxc',dmxc?dmxc.value:'1');";
  html += "data.set('dmxpr',dmxpr?dmxpr.value:'1');";
  html += "data.set('dmxfd',dmxfd?dmxfd.value:'0');";
  html += "data.set('omode',om?om.value:'0');";
  html += "data.set('kind',kind||'max');";
  html += "data.set('omin',omin?omin.value:'0');";
  html += "data.set('omax',omax?omax.value:'100');";
  html += "fetch('/send_dmx_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString(),credentials:'same-origin'}).then(r=>r.text()).then(t=>console.log('dmx test',t)).catch(e=>console.warn('dmx test failed',e));";
  html += "return;";
  html += "}";
  html += "if(tgt && tgt.value!=='0'){return;}";
  html += "const addrInput=document.getElementById('s'+idx+'_o'+oidx+'_addr');";
  html += "const encAddrInput=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_enc_addr']\");";
  html += "const btnAddrInput=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_btn_addr']\");";
  html += "const useBtn=(kind==='bmin' || kind==='bmax');";
  html += "const addr=useBtn?(btnAddrInput?btnAddrInput.value:''):(isEnc?(encAddrInput?encAddrInput.value:''):(addrInput?addrInput.value:''));";
  html += "const dev=document.getElementById('s'+idx+'_o'+oidx+'_dev');";
  html += "const om=document.getElementById('s'+idx+'_o'+oidx+'_omode');";
  html += "const onStr=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_on']\");";
  html += "const offStr=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_off']\");";
  html += "const omin=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_omin']\");";
  html += "const omax=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_omax']\");";
  html += "const bm=document.getElementById('s'+idx+'_o'+oidx+'_bmode');";
  html += "const bon=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_bon']\");";
  html += "const boff=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_boff']\");";
  html += "const bmin=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_bmin']\");";
  html += "const bmax=document.querySelector(\"input[name='s\"+idx+\"_o\"+oidx+\"_bmax']\");";
  html += "if(!addr){return;}";
  html += "const data=new URLSearchParams();";
  html += "data.set('test_addr',addr);";
  html += "if(dev){data.set('test_dev',dev.value);}"; 
  html += "if(useBtn){";
  html += "const mode=bm?bm.value:'0';";
  html += "if(mode==='2'){";
  html += "data.set('test_type','string');";
  html += "const v=(kind==='bmax')?(bon?bon.value:''):(boff?boff.value:'');";
  html += "data.set('test_value',v||'');";
  html += "}else if(mode==='1'){";
  html += "data.set('test_type','float');";
  html += "const v=(kind==='bmax')?(bmax?bmax.value:'0'):(bmin?bmin.value:'0');";
  html += "data.set('test_value',v);";
  html += "}else{";
  html += "data.set('test_type','int');";
  html += "const v=(kind==='bmax')?(bmax?bmax.value:'0'):(bmin?bmin.value:'0');";
  html += "data.set('test_value',v);";
  html += "}";
  html += "}else if(om&&om.value==='2'){";
  html += "data.set('test_type','string');";
  html += "const v=(kind==='max')?(onStr?onStr.value:''):(offStr?offStr.value:'');";
  html += "data.set('test_value',v||'');";
  html += "}else if(om&&om.value==='1'){";
  html += "data.set('test_type','float');";
  html += "const v=(kind==='max')?(omax?omax.value:'0'):(omin?omin.value:'0');";
  html += "data.set('test_value',v);";
  html += "}else{";
  html += "data.set('test_type','int');";
  html += "const v=(kind==='max')?(omax?omax.value:'0'):(omin?omin.value:'0');";
  html += "data.set('test_value',v);";
  html += "}";
  html += "fetch('/send_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString(),credentials:'same-origin'}).then(r=>r.text()).then(t=>console.log('test send',t)).catch(e=>console.warn('test send failed',e));";
  html += "}";
  html += "document.querySelectorAll(\"button[data-test]\").forEach(btn=>{";
  html += "btn.addEventListener('click',()=>{sendOutputTest(btn.dataset.idx,btn.dataset.oidx,btn.dataset.test);});";
  html += "});";
  html += "</script>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
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

  server.sendHeader("Location", "/settings", true);
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

  DynamicJsonDocument doc(98304);
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
  appendHtmlRaw("table.grid th,table.grid td{border:1px solid #e6e6e6;padding:2px;text-align:center;}");
  appendHtmlRaw("table.grid th{position:sticky;top:0;background:#f6f6f6;z-index:1;}");
  appendHtmlRaw("table.grid td.rowh{background:#f8f8f8;font-weight:600;position:sticky;left:0;z-index:2;}");
  appendHtmlRaw("table.grid .cell{display:flex;flex-direction:column;align-items:stretch;gap:2px;}");
  appendHtmlRaw("table.grid .ch{font-size:10px;line-height:1;color:#555;}");
  appendHtmlRaw("table.grid input{width:52px;max-width:none;margin:0;padding:4px 4px;font-size:12px;text-align:center;}");
  appendHtmlRaw("a{color:#0b6b6f;}");
  appendHtmlRaw("</style></head><body><div class='page'>");

  appendHtmlRaw("<div class='top'><h2 style='margin:0;'>DMX Preset Editor</h2><a href='/settings'>Back to settings</a></div>");
  appendHtmlRaw("<div class='card'>");
  appendHtml(String("<div>Point budget: <b>") + String(totalPoints) + "</b> / " + String(MAX_DMX_PRESET_POINTS) + "</div>");
  appendHtml(String("<div>Universe range: 1-") + String(MAX_DMX_PRESET_UNIVERSE) + "</div>");
  if (notice.length() > 0) appendHtml(String("<div class='notice'>") + notice + "</div>");
  if (error.length() > 0) appendHtml(String("<div class='error'>") + error + "</div>");
  appendHtmlRaw("<form method='GET' action='/dmx_presets'>");
  appendHtmlRaw("Preset: <select name='preset' onchange='this.form.submit()'>");
  for (int i = 1; i <= MAX_DMX_PRESETS; i++) {
    appendHtml(String("<option value='") + String(i) + "' " + String(i == preset ? "selected" : "") + ">Preset " + String(i) + "</option>");
  }
  appendHtmlRaw("</select>");
  appendHtmlRaw("Universe: <select name='u' onchange='this.form.submit()'>");
  for (int i = 1; i <= MAX_DMX_PRESET_UNIVERSE; i++) {
    appendHtml(String("<option value='") + String(i) + "' " + String(i == universe ? "selected" : "") + ">Universe " + String(i) + "</option>");
  }
  appendHtmlRaw("</select>");
  appendHtmlRaw("Live source: <select name='live_src' onchange='this.form.submit()'>");
  appendHtml(String("<option value='0' ") + String(liveSourceMode == DMX_IN_SOURCE_RECENT ? "selected" : "") + ">Auto</option>");
  appendHtml(String("<option value='1' ") + String(liveSourceMode == DMX_IN_SOURCE_PREFER_ARTNET ? "selected" : "") + ">Art-Net</option>");
  appendHtml(String("<option value='2' ") + String(liveSourceMode == DMX_IN_SOURCE_PREFER_SACN ? "selected" : "") + ">sACN</option>");
  appendHtmlRaw("</select>");
  appendHtml(String("Snapshot range: <input name='lu_from' type='number' min='1' max='") + String(MAX_DMX_PRESET_UNIVERSE) + "' value='" + String(liveFrom) + "' style='max-width:110px;' onchange='this.form.submit()'>");
  appendHtml(String("<input name='lu_to' type='number' min='1' max='") + String(MAX_DMX_PRESET_UNIVERSE) + "' value='" + String(liveTo) + "' style='max-width:110px;' onchange='this.form.submit()'>");
  appendHtml(String("<label style='display:inline-flex;align-items:center;gap:6px;max-width:none;'><input name='live_nonzero' type='checkbox' value='1' ") + String(liveNonZeroOnly ? "checked" : "") + " onchange='this.form.submit()'>Store non-zero only</label>");
  appendHtmlRaw("</form>");
  appendHtml(String("<div>Current preset points: <b>") + String(currentPoints) + "</b></div>");
  appendHtml(String("<div>Universe ") + String(universe) + " active channels: <b>" + String(activeChannels) + "</b></div>");
  appendHtml(String("<div>Live input (U") + String(universe) + "): Art-Net " + String(hasArtLive ? ("seen " + String(millis() - artSeenMs) + "ms ago") : "not seen") +
             ", sACN " + String(hasSacLive ? ("seen " + String(millis() - sacSeenMs) + "ms ago") : "not seen") + "</div>");
  if (hasLivePreview) {
    appendHtml(String("<div>Live preview source: <b>") + String(livePreviewSource == DMX_PROTO_ARTNET ? "Art-Net" : "sACN") +
               "</b> (" + String(livePreviewAgeMs) + "ms ago), active channels: <b>" + String(livePreviewActiveChannels) + "</b></div>");
  } else {
    appendHtmlRaw("<div>Live preview source: <b>none</b></div>");
  }
  appendHtmlRaw("</div>");

  appendHtmlRaw("<div class='card'><b>Live DMX Snapshot</b><br>");
  appendHtmlRaw("<small>Capture current DMX input values into this preset for one universe or a range.</small><br>");
  appendHtmlRaw("<form method='POST' action='/dmx_presets'>");
  appendHtml(String("<input type='hidden' name='preset' value='") + String(preset) + "'>");
  appendHtml(String("<input type='hidden' name='u' value='") + String(universe) + "'>");
  appendHtml(String("<input type='hidden' name='live_src' value='") + String(liveSourceMode) + "'>");
  appendHtml(String("<input type='hidden' name='lu_from' value='") + String(liveFrom) + "'>");
  appendHtml(String("<input type='hidden' name='lu_to' value='") + String(liveTo) + "'>");
  if (liveNonZeroOnly) appendHtmlRaw("<input type='hidden' name='live_nonzero' value='1'>");
  appendHtmlRaw("<button type='submit' name='action' value='snapshot_live' class='primary'>Get Current Values Snapshot</button>");
  appendHtmlRaw("</form>");
  appendHtmlRaw("</div>");

  appendHtmlRaw("<div class='card'><b>Universe Grid</b><br>");
  appendHtmlRaw("<small>Set DMX values per channel. Blank cell means channel is not stored in this preset/universe.</small>");
  appendHtmlRaw("<form method='POST' action='/dmx_presets'>");
  appendHtml(String("<input type='hidden' name='preset' value='") + String(preset) + "'>");
  appendHtml(String("<input type='hidden' name='u' value='") + String(universe) + "'>");
  appendHtml(String("<input type='hidden' name='live_src' value='") + String(liveSourceMode) + "'>");
  appendHtml(String("<input type='hidden' name='lu_from' value='") + String(liveFrom) + "'>");
  appendHtml(String("<input type='hidden' name='lu_to' value='") + String(liveTo) + "'>");
  if (liveNonZeroOnly) appendHtmlRaw("<input type='hidden' name='live_nonzero' value='1'>");
  appendHtmlRaw("<div class='grid-wrap'><table class='grid'><tr><th>Channels</th>");
  for (int c = 1; c <= 16; c++) appendHtml(String("<th>") + String(c) + "</th>");
  appendHtmlRaw("</tr>");
  for (int row = 0; row < 32; row++) {
    int rowStart = row * 16 + 1;
    int rowEnd = rowStart + 15;
    appendHtml(String("<tr><td class='rowh'>") + String(rowStart) + "-" + String(rowEnd) + "</td>");
    for (int col = 0; col < 16; col++) {
      int ch = rowStart + col;
      String v = universeValues[ch - 1] >= 0 ? String(universeValues[ch - 1]) : "";
      appendHtml(String("<td><div class='cell'><div class='ch'>") + String(ch) + "</div><input type='number' min='0' max='255' name='ch" + String(ch) + "' value='" + v + "' placeholder='-'></div></td>");
    }
    appendHtmlRaw("</tr>");
  }
  appendHtmlRaw("</table></div><br>");
  appendHtmlRaw("<button type='submit' name='action' value='save_grid' class='primary'>Save Universe</button> ");
  appendHtmlRaw("<button type='submit' name='action' value='clear_universe'>Clear Universe</button>");
  appendHtmlRaw("</form>");
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
  appendHtml(String("<textarea name='preset_text' placeholder='1,1-10,255&#10;2,1,128'>") + currentText + "</textarea><br>");
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
  if (out.dmxProtocol == DMX_PROTO_ARTNET) {
    out.dmxUniverse = encodeArtNetPortAddressOneBased(out.dmxArtNet, out.dmxArtSubnet, out.dmxArtUniverse);
  } else {
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
  int n = WiFi.scanNetworks(false, true);
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
  html += "a{color:#0b6b6f;}</style></head><body>";
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
  html.reserve(1600);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>StageMod Firmware Update</title>";
  html += "<style>body{font-family:'Avenir Next','Trebuchet MS','Segoe UI',sans-serif;background:#f4f1ea;color:#1a1a1a;padding:20px;}";
  html += ".card{background:#fff;border:1px solid #ddd;border-radius:12px;padding:16px;max-width:520px;}";
  html += "button{border:1px solid #b9b9b9;background:#0b6b6f;color:#fff;padding:8px 12px;border-radius:8px;cursor:pointer;}";
  html += "</style></head><body>";
  html += "<div class='card'>";
  html += "<h2>Firmware Update</h2>";
  html += "<p>Upload a new firmware .bin over your local network. Internet is not required.</p>";
  html += "<form method='POST' action='/update' enctype='multipart/form-data'>";
  html += "<input type='file' name='update' accept='.bin'><br><br>";
  html += "<button type='submit'>Install Update</button>";
  html += "</form>";
  html += "<p><a href='/'>Back to settings</a></p>";
  html += "</div></body></html>";
  server.send(200, "text/html", html);
}

static void handleUpdateFinished() {
  if (!ensureAuthenticated()) return;
  bool ok = !Update.hasError();
  server.sendHeader("Connection", "close");
  if (ok) {
    String html;
    html.reserve(600);
    html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<title>Update Successful</title>";
    html += "<style>body{font-family:'Avenir Next','Trebuchet MS','Segoe UI',sans-serif;background:#111;color:#f2f2f2;padding:20px;}</style>";
    html += "</head><body>";
    html += "Update successful. Rebooting...<br>Returning to Settings...";
    html += "<script>setTimeout(()=>{window.location='/settings';},8000);</script>";
    html += "</body></html>";
    server.send(200, "text/html", html);
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

static void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
      addLog(String("Firmware update begin failed: ") + Update.errorString());
    } else {
      addLog("Firmware update upload started.");
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) {
      addLog(String("Firmware update end failed: ") + Update.errorString());
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
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

  Serial.println("Starting network...");
#if USE_ETHERNET
  ETH.begin();
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

  server.on("/", HTTP_GET, handleTriggers);
  server.on("/triggers", HTTP_POST, handleSaveTriggers);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/settings", HTTP_POST, handleSaveSettings);
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
  server.on("/generate_204", HTTP_GET, []() {
    server.sendHeader("Location", "/settings", true);
    server.send(302, "text/plain", "");
  });
  server.on("/hotspot-detect.html", HTTP_GET, []() {
    server.sendHeader("Location", "/settings", true);
    server.send(302, "text/plain", "");
  });
  server.on("/fwlink", HTTP_GET, []() {
    server.sendHeader("Location", "/settings", true);
    server.send(302, "text/plain", "");
  });
  server.on("/ncsi.txt", HTTP_GET, []() {
    server.sendHeader("Location", "/settings", true);
    server.send(302, "text/plain", "");
  });
  server.onNotFound([]() {
    server.sendHeader("Location", "/settings", true);
    server.send(302, "text/plain", "");
  });
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

  for (int i = 0; i < MAX_TRIGGERS; i++) {
    SensorConfig& s = config.sensors[i];
    if (!s.enabled) continue;
    if (s.source == SRC_TIME) {
      handleTimeTrigger(i, s);
      continue;
    }
    if (s.source == SRC_OSC) continue;
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

#if USE_ETHERNET
  if (config.netMode == NET_ETHERNET) {
    bool netOk = ETH.linkUp() && isValidIp(ETH.localIP());
    if (!netOk && !wifiApMode) {
      startWifiAp();
      configureUdpListener();
    } else if (netOk && wifiApMode) {
      WiFi.softAPdisconnect(true);
      wifiApMode = false;
      dnsServer.stop();
      configureUdpListener();
    }
  }
#endif

  if (config.netMode == NET_WIFI && wifiReconnectStarted == 0) {
    bool netOk = (WiFi.status() == WL_CONNECTED) && isValidIp(WiFi.localIP());
    if (!netOk && !wifiApMode) {
      startWifiAp();
      configureUdpListener();
    } else if (netOk && wifiApMode) {
      WiFi.softAPdisconnect(true);
      wifiApMode = false;
      dnsServer.stop();
      configureUdpListener();
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
