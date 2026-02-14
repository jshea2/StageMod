#include <Arduino.h>
#include <math.h>
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
const char* FIRMWARE_VERSION = "0.10.0";
const char* DEFAULT_NTP_SERVER = "pool.ntp.org";
const uint8_t DEFAULT_TIME_MODE = 0; // 0 = NTP, 1 = Manual
const char* DEFAULT_TIMEZONE = "PST8PDT,M3.2.0/2,M11.1.0/2";
const bool DEFAULT_TIME_DISPLAY_24H = false;

const int MAX_TRIGGERS = 20;
const int MAX_OSC_DEVICES = 8;
const int MAX_OUTPUTS_PER_TRIGGER = 5;

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
  SRC_TIME = 1
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
  OUT_TARGET_HTTP = 5
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
  uint8_t triggerCount;
  uint8_t triggerOrder[MAX_TRIGGERS];
  uint8_t oscDeviceCount;
  uint8_t oscDeviceOrder[MAX_OSC_DEVICES];
  OscDevice oscDevices[MAX_OSC_DEVICES];
  SensorConfig sensors[MAX_TRIGGERS];
};

Config config;

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
WiFiUDP udp;
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

static String ipToString(const IPAddress& ip);
static bool parseIpString(const String& s, IPAddress& out);
static bool isValidIp(const IPAddress& ip);
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
  float dist = (duration == 0) ? -1.0f : (duration / 58.0f);
  if (dist < 0) {
    if (ema[index] >= 0) {
      float norm = ema[index] / 4095.0f;
      norm = clampFloat(norm, 0.0f, 1.0f);
      if (sensor.invert) norm = 1.0f - norm;
      return norm;
    }
    return 0.0f;
  }

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
  DynamicJsonDocument doc(32768);
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
      JsonArray outs = so["outputs"].as<JsonArray>();
      if (!outs.isNull()) {
        for (JsonObject oo : outs) {
          int oidx = oo["index"] | -1;
          if (oidx < 0 || oidx >= MAX_OUTPUTS_PER_TRIGGER) continue;
          SensorConfig::OutputConfig out = s.outputs[oidx];
          if (oo.containsKey("target")) out.target = static_cast<OutputTarget>(oo["target"].as<int>());
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
          if (oo.containsKey("onString")) out.onString = String(oo["onString"].as<const char*>());
          if (oo.containsKey("offString")) out.offString = String(oo["offString"].as<const char*>());
          if (oo.containsKey("buttonOnString")) out.buttonOnString = String(oo["buttonOnString"].as<const char*>());
          if (oo.containsKey("buttonOffString")) out.buttonOffString = String(oo["buttonOffString"].as<const char*>());
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
      config.sensors[idx] = s;
    }
  }

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
  if (value < OUT_TARGET_OSC || value > OUT_TARGET_HTTP) return OUT_TARGET_OSC;
  return static_cast<OutputTarget>(value);
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
    if (s.source == SRC_TIME) continue;
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
  removeIfExists(sensorKey(index, "type"));
  removeIfExists(sensorKey(index, "pin"));
  removeIfExists(sensorKey(index, "clk"));
  removeIfExists(sensorKey(index, "dt"));
  removeIfExists(sensorKey(index, "sw"));
  removeIfExists(sensorKey(index, "enc_app"));
  removeIfExists(sensorKey(index, "enc_arg"));
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
    if (sensor.source == SRC_TIME) continue;
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
    if (!s.enabled || s.source == SRC_TIME || s.type != SENSOR_ENCODER) continue;
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
  if (config.sensors[index].source == SRC_TIME) return false;
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
  if (!a.enabled || a.source == SRC_TIME) return false;
  if (a.type != SENSOR_BUTTON) return false;
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    if (i == index) continue;
    const SensorConfig& b = config.sensors[i];
    if (!b.enabled || b.source == SRC_TIME) continue;
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
    if (s.source != SRC_TIME) s.source = SRC_SENSORS;
    s.type = sanitizeSensorType(prefs.getInt(sensorKey(i, "type").c_str(), s.type));
    s.pin = prefs.getInt(sensorKey(i, "pin").c_str(), s.pin);
    s.encClkPin = prefs.getInt(sensorKey(i, "clk").c_str(), s.encClkPin);
    s.encDtPin = prefs.getInt(sensorKey(i, "dt").c_str(), s.encDtPin);
    s.encSwPin = prefs.getInt(sensorKey(i, "sw").c_str(), s.encSwPin);
    s.encAppendSign = prefs.getBool(sensorKey(i, "enc_app").c_str(), s.encAppendSign);
    s.encSignArg = prefs.getBool(sensorKey(i, "enc_arg").c_str(), s.encSignArg);
    s.encInvert = prefs.getBool(sensorKey(i, "enc_inv").c_str(), s.encInvert);
    s.encSteps = DEFAULT_ENC_STEPS;
    if (s.type == SENSOR_ANALOG) {
      if (!isAllowedAnalogPin(s.pin)) s.pin = defaultSensorPin(i);
    } else {
      if (!isAllowedDigitalPin(s.pin)) s.pin = defaultSensorPin(i);
    }
    if (!isAllowedDigitalPin(s.encClkPin)) s.encClkPin = 13;
    if (!isAllowedDigitalPin(s.encDtPin)) s.encDtPin = 16;
    if (!isAllowedDigitalPin(s.encSwPin)) s.encSwPin = 33;
    if (!isAllowedDigitalPin(s.hcTrigPin)) s.hcTrigPin = 13;
    if (!isAllowedDigitalPin(s.hcEchoPin)) s.hcEchoPin = 16;
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
        s.outputs[o].onString = prefs.getString(baseOn.c_str(), s.outputs[o].onString);
        s.outputs[o].offString = prefs.getString(baseOff.c_str(), s.outputs[o].offString);
        s.outputs[o].buttonOnString = prefs.getString(baseBon.c_str(), s.outputs[o].buttonOnString);
        s.outputs[o].buttonOffString = prefs.getString(baseBoff.c_str(), s.outputs[o].buttonOffString);
      }
      if (s.outputs[o].oscAddress.length() == 0) s.outputs[o].oscAddress = defaultOscAddress(i);
      if (s.outputs[o].buttonAddress.length() == 0) s.outputs[o].buttonAddress = "/button";
      if (s.type == SENSOR_ANALOG && s.outputs[o].outMode == OUT_STRING) s.outputs[o].outMode = OUT_FLOAT;
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

  prefs.end();
}

static void saveConfig() {
  prefs.begin("osc", false);
  prefs.putString("hostname", config.hostname);
  prefs.putInt("net_mode", config.netMode);
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
    prefs.putInt(sensorKey(i, "pin").c_str(), s.pin);
    prefs.putInt(sensorKey(i, "clk").c_str(), s.encClkPin);
    prefs.putInt(sensorKey(i, "dt").c_str(), s.encDtPin);
    prefs.putInt(sensorKey(i, "sw").c_str(), s.encSwPin);
    prefs.putBool(sensorKey(i, "enc_app").c_str(), s.encAppendSign);
    prefs.putBool(sensorKey(i, "enc_arg").c_str(), s.encSignArg);
    prefs.putBool(sensorKey(i, "enc_inv").c_str(), s.encInvert);
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

static String buildHttpValue(const SensorConfig::OutputConfig& out, float norm) {
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
    if (s.source == SRC_TIME) continue;
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
      html += "<option value='5' " + String(out.target == OUT_TARGET_HTTP ? "selected" : "") + ">HTTP Request</option>";
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
  html += "const isTime=(src==='1');";
  html += "const isEnc=(t==='4');";
  html += "if(typeBlock){typeBlock.style.display=isTime?'none':'block';}";
  html += "if(timeBlock){timeBlock.style.display=isTime?'block':'none';}";
  html += "if(ttSel){";
  html += "const tt=ttSel.value;";
  html += "if(timeOnce) timeOnce.style.display=(tt==='0')?'block':'none';";
  html += "if(timeDaily) timeDaily.style.display=(tt==='1')?'block':'none';";
  html += "if(timeWeekly) timeWeekly.style.display=(tt==='2')?'block':'none';";
  html += "if(timeInterval) timeInterval.style.display=(tt==='3')?'block':'none';";
  html += "}";
  html += "if(a){a.style.display=(!isTime && t==='0')?'block':'none';}";
  html += "if(d){d.style.display=(!isTime && t!=='0'&&!isEnc)?'block':'none';}";
  html += "if(pin){pin.style.display=(isEnc||isTime||t==='9')?'none':'block';}";
  html += "if(enc){enc.style.display=isEnc?'block':'none';}";
  html += "if(hc){hc.style.display=(t==='9')?'block':'none';}";
  html += "if(cd){cd.style.display=(!isTime && (t==='1' || t==='8'))?'block':'none';}";
  html += "for(let o=0;o<" + String(MAX_OUTPUTS_PER_TRIGGER) + ";o++){";
  html += "const tgt=document.getElementById('s'+i+'_o'+o+'_tgt');";
  html += "if(!tgt) continue;";
  html += "const oscOut=document.getElementById('s'+i+'_o'+o+'_osc_out');";
  html += "const devBlock=document.getElementById('s'+i+'_o'+o+'_dev_block');";
  html += "const udpBlock=document.getElementById('s'+i+'_o'+o+'_udp_block');";
  html += "const gpioBlock=document.getElementById('s'+i+'_o'+o+'_gpio_block');";
  html += "const httpBlock=document.getElementById('s'+i+'_o'+o+'_http_block');";
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
  html += "const isHttp=(tgt.value==='5');";
  html += "if(oscOut){oscOut.style.display=isOsc?'block':'none';}";
  html += "if(devBlock){devBlock.style.display=(isOsc||isUdp)?'block':'none';}";
  html += "if(udpBlock){udpBlock.style.display=isUdp?'block':'none';}";
  html += "if(gpioBlock){gpioBlock.style.display=isGpio?'block':'none';}";
  html += "if(httpBlock){httpBlock.style.display=isHttp?'block':'none';}";
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
  html += "if(om){om.disabled=!(isOsc||isHttp);}";
  html += "if(om&&opt){";
  html += "if(!isOsc){opt.disabled=true;}";
  html += "else if(isEnc){";
  html += "opt.disabled=true;";
  html += "if(om.value==='2'){om.value='1';}";
  html += "}else if(!isTime && t==='0'){";
  html += "opt.disabled=true;";
  html += "if(om.value==='2'){om.value='1';}";
  html += "}else{";
  html += "opt.disabled=false;";
  html += "}";
  html += "}";
  html += "if(str&&om){";
  html += "const showStr=((isOsc||isHttp) && !isEnc && om.value==='2');";
  html += "str.style.display=showStr?'block':'none';";
  html += "if(offRow){offRow.style.display=(showStr && smin && smin.checked)?'none':'block';}";
  html += "}";
  html += "if(range&&om){range.style.display=((isOsc||isHttp) && om.value!=='2')?'block':(isGpio && gmode && gmode.value==='3' ? 'block' : 'none');}";
  html += "if(addrBlock){addrBlock.style.display=(isOsc && !isEnc)?'block':'none';}";
  html += "if(encAddr){encAddr.style.display=(isOsc && isEnc)?'block':'none';}";
  html += "if(btnAddr){btnAddr.style.display=(isOsc && isEnc)?'block':'none';}";
  html += "if(btnBlock){btnBlock.style.display=(isOsc && isEnc)?'block':'none';}";
  html += "if(minRel){";
  html += "if(isOsc||isHttp){minRel.style.display=(isEnc||t!=='0')?'block':'none';}";
  html += "else{minRel.style.display='none';}";
  html += "}";
  html += "if(minRow&&smin&&om){";
  html += "if(isOsc||isHttp){";
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
  html += "if(tSel){tSel.addEventListener('change',()=>updateTrigger(i));}";
  html += "if(srcSel){srcSel.addEventListener('change',()=>updateTrigger(i));}";
  html += "if(ttSel){ttSel.addEventListener('change',()=>updateTrigger(i));}";
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
    SensorConfig s = config.sensors[i];
    s.enabled = server.hasArg("s" + idx + "_en");
    if (server.hasArg("s" + idx + "_name")) {
      s.name = server.arg("s" + idx + "_name");
      s.name.trim();
      if (s.name.length() == 0) s.name = String("Trigger ") + String(i + 1);
    }
    if (server.hasArg("s" + idx + "_src")) {
      int src = server.arg("s" + idx + "_src").toInt();
      s.source = (src == 1) ? SRC_TIME : SRC_SENSORS;
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

    if (s.source != SRC_TIME) {
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
        s.outputs[s.outputCount] = s.outputs[0];
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
  html += "<select id='wifi_ssid_select' name='wifi_ssid_select'><option value=''>Scan to list networks...</option></select> ";
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

  html += "<div class='card'><b>Network Clients (OSC/UDP)</b><br>";
  html += "<small>Device 1 is the default target for OSC/UDP.</small><br>";
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
  html += "<a href='/export_config' class='nav-pill'>Download Config</a>";
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
  html += "<input type='file' name='update' accept='.bin'><br><br>";
  html += "<button type='submit' class='primary'>Install Update</button>";
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
  html += "function toggleDhcp(){sf.style.display=dh.checked?'none':'block';}";
  html += "function toggleNet(){wf.style.display=(nm.value==='1')?'block':'none';}";
  html += "function toggleTime(){if(!tm){return;}ntp.style.display=(tm.value==='0')?'block':'none';man.style.display=(tm.value==='1')?'block':'none';}";
  html += "dh.addEventListener('change',toggleDhcp);";
  html += "nm.addEventListener('change',toggleNet);";
  html += "if(tm){tm.addEventListener('change',toggleTime);}";
  html += "toggleDhcp();toggleNet();toggleTime();";
  html += "const scanBtn=document.getElementById('scan_wifi');";
  html += "const wifiSelect=document.getElementById('wifi_ssid_select');";
  html += "const wifiInput=document.getElementById('wifi_ssid');";
  html += "if(wifiSelect&&wifiInput){wifiSelect.addEventListener('change',()=>{";
  html += "const v=wifiSelect.value; if(v==='__custom__'){wifiInput.focus();return;} wifiInput.value=v;});}";
  html += "if(scanBtn&&wifiSelect&&wifiInput){";
  html += "scanBtn.addEventListener('click',()=>{";
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
  resetRuntimeState(true);
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
    udp.begin(LOCAL_PORT);
    startMdns();
    applyTimeConfig();
  } else {
    Serial.println("Network failed to connect.");
    addLog("Network failed. Starting AP.");
    startWifiAp();
  }

  server.on("/", HTTP_GET, handleTriggers);
  server.on("/triggers", HTTP_POST, handleSaveTriggers);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/settings", HTTP_POST, handleSaveSettings);
  server.on("/send_test", HTTP_POST, handleSendTestOsc);
  server.on("/send_udp_test", HTTP_POST, handleSendTestUdp);
  server.on("/send_gpio_test", HTTP_POST, handleSendTestGpio);
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

  for (int i = 0; i < MAX_TRIGGERS; i++) {
    SensorConfig& s = config.sensors[i];
    if (!s.enabled) continue;
    if (s.source == SRC_TIME) {
      handleTimeTrigger(i, s);
      continue;
    }
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
          } else if (out.target == OUT_TARGET_UDP || out.target == OUT_TARGET_HTTP || out.target == OUT_TARGET_OSC) {
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

  delay(LOOP_DELAY_MS);

  server.handleClient();

  if (pendingNetApply && millis() - pendingApplyAt > 250) {
    pendingNetApply = false;
    applyNetworkConfig();
    wifiReconnectStarted = (config.netMode == NET_WIFI) ? millis() : 0;
    if (pendingMdnsRestart) {
      pendingMdnsRestart = false;
      MDNS.end();
      startMdns();
    }
  }
  if (wifiReconnectStarted > 0 && config.netMode == NET_WIFI) {
    if (WiFi.status() == WL_CONNECTED && isValidIp(WiFi.localIP())) {
      wifiReconnectStarted = 0;
      if (wifiApMode) {
        WiFi.softAPdisconnect(true);
        wifiApMode = false;
        dnsServer.stop();
      }
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
    } else if (netOk && wifiApMode) {
      WiFi.softAPdisconnect(true);
      wifiApMode = false;
      dnsServer.stop();
    }
  }
#endif

  if (config.netMode == NET_WIFI && wifiReconnectStarted == 0) {
    bool netOk = (WiFi.status() == WL_CONNECTED) && isValidIp(WiFi.localIP());
    if (!netOk && !wifiApMode) {
      startWifiAp();
    } else if (netOk && wifiApMode) {
      WiFi.softAPdisconnect(true);
      wifiApMode = false;
      dnsServer.stop();
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
