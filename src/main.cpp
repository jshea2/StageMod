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
#include <esp_system.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <WiFiUdp.h>
#include <time.h>
#include <sys/time.h>
#ifndef WEBSERVER_MAX_POST_ARGS
#define WEBSERVER_MAX_POST_ARGS 300
#endif
#include <WebServer.h>
#include <Update.h>

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

const int MAX_TRIGGERS = 20;
const int MAX_OSC_DEVICES = 8;

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
  SENSOR_BUTTON_LONG = 7
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
  OUT_TARGET_REBOOT = 1
};

const int DEFAULT_ANALOG_PIN = 32;
const int DEFAULT_DIGITAL_PIN = 5;
const bool DEFAULT_INVERT = false;
const float DEFAULT_OUT_MIN = 0.0f;
const float DEFAULT_OUT_MAX = 100.0f;
const OutputMode DEFAULT_OUT_MODE = OUT_INT;
const bool DEFAULT_DIGITAL_ACTIVE_HIGH = true;
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
const float ALPHA       = 0.30f;               // exponential smoothing 0..1
const int DEAD_BAND     = 1;                   // percent hysteresis to stop flicker

const uint32_t DEFAULT_CLICK_DEBOUNCE_MS = 30;   // debounce for buttons
const uint32_t DEFAULT_MULTI_CLICK_GAP_MS = 350; // max gap between clicks
const uint32_t DEFAULT_LONG_PRESS_MS = 2000;     // long-press threshold

const uint16_t LOCAL_PORT = 9000;              // Local UDP port
const float SNAP_PERCENT = 1.0f;               // snap edges to min/max

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
  bool invert;
  bool activeHigh;
  bool pullup;
  bool cooldownEnabled;
  uint32_t cooldownMs;
  String oscAddress;
  String buttonAddress;
  float outMin;
  float outMax;
  OutputMode outMode;
  OutputTarget outTarget;
  uint8_t outDevice;
  String onString;
  String offString;
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
  uint8_t timeMode;
  String ntpServer;
  String timeZone;
  uint32_t manualEpoch;
  uint8_t triggerCount;
  uint8_t triggerOrder[MAX_TRIGGERS];
  uint8_t oscDeviceCount;
  uint8_t oscDeviceOrder[MAX_OSC_DEVICES];
  OscDevice oscDevices[MAX_OSC_DEVICES];
  SensorConfig sensors[MAX_TRIGGERS];
};

Config config;

float ema[MAX_TRIGGERS];     // smoothed ADC per sensor
int lastInt[MAX_TRIGGERS];   // last sent int value per sensor
float lastFloat[MAX_TRIGGERS]; // last sent float value per sensor
int8_t lastBool[MAX_TRIGGERS]; // last sent on/off state per sensor
int8_t lastLevel[MAX_TRIGGERS]; // last raw level per sensor
bool toggleState[MAX_TRIGGERS]; // toggle state per sensor
int8_t lastEncA[MAX_TRIGGERS];
int8_t lastEncB[MAX_TRIGGERS];
int8_t lastEncState[MAX_TRIGGERS];
int8_t encAccum[MAX_TRIGGERS];
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

static String ipToString(const IPAddress& ip);
static bool parseIpString(const String& s, IPAddress& out);
static bool isValidIp(const IPAddress& ip);
static String buildApSsid();
static void startWifiAp();
static void addLog(const String& msg);
static bool parseOrderList(const String& s, uint8_t* order, uint8_t& count);
static String buildOrderList(const uint8_t* order, uint8_t count);
static int findUnusedTrigger();
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
static void sendTriggerOutput(const SensorConfig& sensor, const String& addr);
static bool handleTimeTrigger(int index, SensorConfig& s);

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
  if (ema[index] < 0) ema[index] = raw;
  ema[index] = ALPHA * raw + (1.0f - ALPHA) * ema[index];
  float norm = ema[index] / 4095.0f;
  norm = clampFloat(norm, 0.0f, 1.0f);
  if (sensor.invert) norm = 1.0f - norm;
  return norm;
}

static bool readDigitalActive(const SensorConfig& sensor) {
  int value = digitalRead(sensor.pin);
  bool isOn = sensor.activeHigh ? (value == HIGH) : (value == LOW);
  return isOn;
}

static float mapOutput(const SensorConfig& sensor, float norm) {
  if (norm <= 0.015f) return sensor.outMin;
  if (norm >= 0.985f) return sensor.outMax;
  float out = sensor.outMin + norm * (sensor.outMax - sensor.outMin);
  if (sensor.outMin < sensor.outMax) {
    return clampFloat(out, sensor.outMin, sensor.outMax);
  }
  return clampFloat(out, sensor.outMax, sensor.outMin);
}

static float snapOutput(const SensorConfig& sensor, float value) {
  float minV = sensor.outMin;
  float maxV = sensor.outMax;
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
  if (value < SENSOR_ANALOG || value > SENSOR_BUTTON_LONG) return SENSOR_ANALOG;
  if (value == SENSOR_DIGITAL) return SENSOR_BUTTON;
  return static_cast<SensorType>(value);
}

static OutputMode sanitizeOutputMode(int value) {
  if (value < OUT_INT || value > OUT_STRING) return DEFAULT_OUT_MODE;
  return static_cast<OutputMode>(value);
}

static OutputTarget sanitizeOutputTarget(int value) {
  if (value < OUT_TARGET_OSC || value > OUT_TARGET_REBOOT) return OUT_TARGET_OSC;
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
  s.type = (index == 0) ? SENSOR_ANALOG : SENSOR_BUTTON;
  s.pin = defaultSensorPin(index);
  s.encClkPin = 13;
  s.encDtPin = 16;
  s.encSwPin = 33;
  s.invert = DEFAULT_INVERT;
  s.activeHigh = DEFAULT_DIGITAL_ACTIVE_HIGH;
  s.pullup = DEFAULT_DIGITAL_PULLUP;
  s.cooldownEnabled = false;
  s.cooldownMs = 5000;
  s.oscAddress = defaultOscAddress(index);
  s.buttonAddress = "/button";
  s.outMin = DEFAULT_OUT_MIN;
  s.outMax = DEFAULT_OUT_MAX;
  s.outMode = DEFAULT_OUT_MODE;
  s.outTarget = OUT_TARGET_OSC;
  s.outDevice = 0;
  s.onString = DEFAULT_ON_STRING;
  s.offString = DEFAULT_OFF_STRING;
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

static String formatTimeLocal() {
  time_t now = time(nullptr);
  if (now < 100000) return "Not set";
  struct tm tmLocal;
  localtime_r(&now, &tmLocal);
  char buf[32];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tmLocal);
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
  int hour = 0, min = 0, sec = 0;
  int parts = sscanf(timeStr.c_str(), "%d:%d:%d", &hour, &min, &sec);
  if (parts < 2) return false;
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
    sendTriggerOutput(s, s.oscAddress);
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

static void resetRuntimeState() {
  for (int i = 0; i < MAX_TRIGGERS; i++) {
    ema[i] = -1.0f;
    lastInt[i] = -1;
    lastFloat[i] = NAN;
    lastBool[i] = -1;
    lastLevel[i] = -1;
    toggleState[i] = false;
    lastEncA[i] = -1;
    lastEncB[i] = -1;
    lastEncState[i] = -1;
    encAccum[i] = 0;
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
  lastHeartbeatSentMs = 0;
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
  removeIfExists(sensorKey(index, "inv"));
  removeIfExists(sensorKey(index, "ah"));
  removeIfExists(sensorKey(index, "pu"));
  removeIfExists(sensorKey(index, "cd_en"));
  removeIfExists(sensorKey(index, "cd_ms"));
  removeIfExists(sensorKey(index, "addr"));
  removeIfExists(sensorKey(index, "baddr"));
  removeIfExists(sensorKey(index, "omin"));
  removeIfExists(sensorKey(index, "omax"));
  removeIfExists(sensorKey(index, "omode"));
  removeIfExists(sensorKey(index, "ot"));
  removeIfExists(sensorKey(index, "od"));
  removeIfExists(sensorKey(index, "on"));
  removeIfExists(sensorKey(index, "off"));
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
    } else if (sensor.type != SENSOR_ANALOG) {
      pinMode(sensor.pin, sensor.pullup ? INPUT_PULLUP : INPUT);
    }
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
    } else {
      pinsB[countB++] = b.pin;
    }
    for (int ia = 0; ia < countA; ia++) {
      for (int ib = 0; ib < countB; ib++) {
        if (pinsA[ia] == pinsB[ib]) {
          if (isClickPatternType(a.type) && isClickPatternType(b.type) &&
              a.pin == b.pin && a.activeHigh == b.activeHigh && a.pullup == b.pullup) {
            continue;
          }
          return true;
        }
      }
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
  config.timeMode = prefs.getUInt("time_mode", DEFAULT_TIME_MODE);
  if (config.timeMode > TIME_MANUAL) config.timeMode = DEFAULT_TIME_MODE;
  config.ntpServer = prefs.getString("ntp_srv", DEFAULT_NTP_SERVER);
  if (config.ntpServer.length() == 0) config.ntpServer = DEFAULT_NTP_SERVER;
  config.timeZone = prefs.getString("tz", DEFAULT_TIMEZONE);
  if (config.timeZone.length() == 0) config.timeZone = DEFAULT_TIMEZONE;
  config.manualEpoch = prefs.getUInt("time_manual", 0);
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
    s.invert = prefs.getBool(sensorKey(i, "inv").c_str(), s.invert);
    s.activeHigh = prefs.getBool(sensorKey(i, "ah").c_str(), s.activeHigh);
    s.pullup = prefs.getBool(sensorKey(i, "pu").c_str(), s.pullup);
    s.cooldownEnabled = prefs.getBool(sensorKey(i, "cd_en").c_str(), s.cooldownEnabled);
    s.cooldownMs = prefs.getUInt(sensorKey(i, "cd_ms").c_str(), s.cooldownMs);
    s.oscAddress = prefs.getString(sensorKey(i, "addr").c_str(), s.oscAddress);
    s.buttonAddress = prefs.getString(sensorKey(i, "baddr").c_str(), s.buttonAddress);
    s.outMin = prefs.getFloat(sensorKey(i, "omin").c_str(), s.outMin);
    s.outMax = prefs.getFloat(sensorKey(i, "omax").c_str(), s.outMax);
    s.outMode = sanitizeOutputMode(prefs.getInt(sensorKey(i, "omode").c_str(), s.outMode));
    s.outTarget = sanitizeOutputTarget(prefs.getInt(sensorKey(i, "ot").c_str(), s.outTarget));
    s.outDevice = prefs.getInt(sensorKey(i, "od").c_str(), s.outDevice);
    if (s.outDevice >= MAX_OSC_DEVICES) s.outDevice = 0;
    s.onString = prefs.getString(sensorKey(i, "on").c_str(), s.onString);
    s.offString = prefs.getString(sensorKey(i, "off").c_str(), s.offString);
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

    if (s.oscAddress.length() == 0) s.oscAddress = defaultOscAddress(i);
    if (s.buttonAddress.length() == 0) s.buttonAddress = "/button";
    if (s.name.length() == 0) s.name = String("Trigger ") + String(i + 1);
    if (s.type == SENSOR_ANALOG && s.outMode == OUT_STRING) s.outMode = OUT_FLOAT;
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
    s0.oscAddress = prefs.getString("osc_address", s0.oscAddress);
    s0.invert = prefs.getBool("invert", DEFAULT_INVERT);
    s0.outMin = prefs.getFloat("out_min", DEFAULT_OUT_MIN);
    s0.outMax = prefs.getFloat("out_max", DEFAULT_OUT_MAX);
    bool outIsFloat = prefs.getBool("out_is_float", false);
    s0.outMode = outIsFloat ? OUT_FLOAT : OUT_INT;
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
  prefs.putUInt("time_mode", config.timeMode);
  prefs.putString("ntp_srv", config.ntpServer);
  prefs.putString("tz", config.timeZone);
  prefs.putUInt("time_manual", config.manualEpoch);
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
    prefs.putBool(sensorKey(i, "inv").c_str(), s.invert);
    prefs.putBool(sensorKey(i, "ah").c_str(), s.activeHigh);
    prefs.putBool(sensorKey(i, "pu").c_str(), s.pullup);
    prefs.putBool(sensorKey(i, "cd_en").c_str(), s.cooldownEnabled);
    prefs.putUInt(sensorKey(i, "cd_ms").c_str(), s.cooldownMs);
    prefs.putString(sensorKey(i, "addr").c_str(), s.oscAddress);
    prefs.putString(sensorKey(i, "baddr").c_str(), s.buttonAddress);
    prefs.putFloat(sensorKey(i, "omin").c_str(), s.outMin);
    prefs.putFloat(sensorKey(i, "omax").c_str(), s.outMax);
    prefs.putInt(sensorKey(i, "omode").c_str(), s.outMode);
    prefs.putInt(sensorKey(i, "ot").c_str(), s.outTarget);
    prefs.putInt(sensorKey(i, "od").c_str(), s.outDevice);
    prefs.putString(sensorKey(i, "on").c_str(), s.onString);
    prefs.putString(sensorKey(i, "off").c_str(), s.offString);
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

static void sendOscString(const char* address, const char* value) {
  IPAddress ip;
  uint16_t port;
  if (!getOscTarget(0, ip, port)) return;
  sendOscStringTo(ip, port, address, value);
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

static void sendTriggerOutput(const SensorConfig& sensor, const String& addr) {
  if (sensor.outTarget == OUT_TARGET_REBOOT) {
    addLog("Reboot triggered by output action.");
    delay(100);
    ESP.restart();
    return;
  }
  if (addr.length() == 0) return;
  IPAddress ip;
  uint16_t port;
  if (!getOscTarget(sensor.outDevice, ip, port)) return;
  if (sensor.outMode == OUT_STRING) {
    const char* value = sensor.onString.c_str();
    if (value[0] == '\0') return;
    sendOscStringTo(ip, port, addr.c_str(), value);
  } else if (sensor.outMode == OUT_FLOAT) {
    float outputValue = mapOutput(sensor, 1.0f);
    outputValue = snapOutput(sensor, outputValue);
    sendOscFloatTo(ip, port, addr.c_str(), outputValue);
  } else {
    float outputValue = mapOutput(sensor, 1.0f);
    outputValue = snapOutput(sensor, outputValue);
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
          clickCount[index]++;
          multiLastReleaseMs[index] = now;
        }
      }
    }
  }

  if (longPress && pressed && !multiLongFired[index] && (now - multiPressStartMs[index] >= config.longPressMs)) {
    sendTriggerOutput(sensor, sensor.oscAddress);
    multiLongFired[index] = true;
    clickCount[index] = 0;
  }

  if (!pressed && clickCount[index] > 0 && (now - multiLastReleaseMs[index] >= config.multiClickGapMs)) {
    if (targetClicks > 0 && clickCount[index] == targetClicks) {
      sendTriggerOutput(sensor, sensor.oscAddress);
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

    html += "<details " + String(tIndex == 0 ? "open" : "") + ">";
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
    html += "<option value='0' " + String(s.type == SENSOR_ANALOG ? "selected" : "") + ">Analog (pot/slider)</option>";
    html += "<option value='4' " + String(s.type == SENSOR_ENCODER ? "selected" : "") + ">Encoder</option>";
    html += "</select><br></div>";
    html += "<div id='s" + idx + "_pin_block'>";
    html += "GPIO Pin: <input name='s" + idx + "_pin' type='number' value='" + String(s.pin) + "'><br>";
    html += "<small>Analog pins allowed: 32,35,36. Digital pins allowed: 5,13,16,17,18,19,21,22,23,25,26,27,32,33,35,36,39</small><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_analog'>";
    html += "Invert: <input name='s" + idx + "_inv' type='checkbox' " + String(s.invert ? "checked" : "") + "><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_digital'>";
    html += "Active high: <input id='s" + idx + "_ah' name='s" + idx + "_ah' type='checkbox' " + String(s.activeHigh ? "checked" : "") + "><br>";
    html += "Use pullup: <input name='s" + idx + "_pu' type='checkbox' " + String(s.pullup ? "checked" : "") + "><br>";
    html += "<small>Button to GND: pullup ON, active high OFF. Button to 3.3V: pullup OFF, active high ON.</small><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_encoder'>";
    html += "<b>Encoder</b><br>";
    html += "Encoder address: <input id='s" + idx + "_enc_addr' name='s" + idx + "_enc_addr' type='text' value='" + s.oscAddress + "'><br>";
    html += "CLK pin: <input name='s" + idx + "_clk' type='number' value='" + String(s.encClkPin) + "'><br>";
    html += "DT pin: <input name='s" + idx + "_dt' type='number' value='" + String(s.encDtPin) + "'><br>";
    html += "SW pin: <input name='s" + idx + "_sw' type='number' value='" + String(s.encSwPin) + "'><br>";
    html += "<small>Encoder sends -1/+1 on each step.</small><br>";
    html += "</div>";

    html += "<div id='s" + idx + "_time_block'>";
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
    html += "Output: <select name='s" + idx + "_ot' id='s" + idx + "_ot'>";
    html += "<option value='0' " + String(s.outTarget == OUT_TARGET_OSC ? "selected" : "") + ">OSC Out</option>";
    html += "<option value='1' " + String(s.outTarget == OUT_TARGET_REBOOT ? "selected" : "") + ">Reboot Device</option>";
    html += "</select><br>";
    html += "<div id='s" + idx + "_osc_out'>";
    html += "OSC device: <select name='s" + idx + "_od' id='s" + idx + "_od'>";
    appendOscDeviceOptions(html, s.outDevice);
    html += "</select><br>";
    html += "<div id='s" + idx + "_addr_block'>";
    html += "OSC address: <input id='s" + idx + "_addr' name='s" + idx + "_addr' type='text' value='" + s.oscAddress + "'><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_button_addr'>";
    html += "<b>Button</b><br>";
    html += "Button address: <input id='s" + idx + "_btn_addr' name='s" + idx + "_btn_addr' type='text' value='" + s.buttonAddress + "'><br>";
    html += "</div>";
    html += "Output type: <select name='s" + idx + "_omode' id='s" + idx + "_omode'>";
    html += "<option value='0' " + String(s.outMode == OUT_INT ? "selected" : "") + ">Int</option>";
    html += "<option value='1' " + String(s.outMode == OUT_FLOAT ? "selected" : "") + ">Float</option>";
    html += "<option value='2' id='s" + idx + "_opt_string' " + String(s.outMode == OUT_STRING ? "selected" : "") + ">String</option>";
    html += "</select><br>";
    html += "<div id='s" + idx + "_range'>";
    html += "Output min: <input name='s" + idx + "_omin' type='number' step='0.001' value='" + String(s.outMin, 3) + "'><br>";
    html += "Output max: <input name='s" + idx + "_omax' type='number' step='0.001' value='" + String(s.outMax, 3) + "'><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_string'>";
    html += "On string: <input name='s" + idx + "_on' type='text' value='" + s.onString + "'><br>";
    html += "Off string: <input name='s" + idx + "_off' type='text' value='" + s.offString + "'><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_cooldown'>";
    html += "Cooldown enabled: <input name='s" + idx + "_cd_en' type='checkbox' " + String(s.cooldownEnabled ? "checked" : "") + "><br>";
    html += "Cooldown ms: <input name='s" + idx + "_cd_ms' type='number' value='" + String(s.cooldownMs) + "'><br>";
    html += "<small>Cooldown blocks repeat presses and only sends the press message.</small><br>";
    html += "</div>";
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
  html += "const om=document.getElementById('s'+i+'_omode');";
  html += "const a=document.getElementById('s'+i+'_analog');";
  html += "const d=document.getElementById('s'+i+'_digital');";
  html += "const s=document.getElementById('s'+i+'_string');";
  html += "const r=document.getElementById('s'+i+'_range');";
  html += "const opt=document.getElementById('s'+i+'_opt_string');";
  html += "const addr=document.getElementById('s'+i+'_addr_block');";
  html += "const addrInput=document.getElementById('s'+i+'_addr');";
  html += "const pin=document.getElementById('s'+i+'_pin_block');";
  html += "const enc=document.getElementById('s'+i+'_encoder');";
  html += "const btnAddrBlock=document.getElementById('s'+i+'_button_addr');";
  html += "const cd=document.getElementById('s'+i+'_cooldown');";
  html += "const encAddr=document.getElementById('s'+i+'_enc_addr');";
  html += "const btnAddr=document.getElementById('s'+i+'_btn_addr');";
  html += "const ot=document.getElementById('s'+i+'_ot');";
  html += "const oscOut=document.getElementById('s'+i+'_osc_out');";
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
  html += "a.style.display=(!isTime && t==='0')?'block':'none';";
  html += "d.style.display=(!isTime && t!=='0'&&!isEnc)?'block':'none';";
  html += "if(t==='0'&&!isTime){";
  html += "opt.disabled=true;";
  html += "if(om.value==='2'){om.value='1';}";
  html += "}else{";
  html += "opt.disabled=false;";
  html += "}";
  html += "s.style.display=(om.value==='2')?'block':'none';";
  html += "r.style.display=(om.value==='2')?'none':'block';";
  html += "addr.style.display=isEnc?'none':'block';";
  html += "pin.style.display=(isEnc||isTime)?'none':'block';";
  html += "enc.style.display=isEnc?'block':'none';";
  html += "btnAddrBlock.style.display=isEnc?'block':'none';";
  html += "if(addrInput){addrInput.disabled=isEnc;}";
  html += "if(encAddr){encAddr.disabled=!isEnc;}";
  html += "if(btnAddr){btnAddr.disabled=!isEnc;}";
  html += "cd.style.display=(!isTime && t==='1')?'block':'none';";
  html += "if(ot){oscOut.style.display=(ot.value==='0')?'block':'none';}";
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
  html += "const om=document.getElementById('s'+i+'_omode');";
  html += "if(om){om.addEventListener('change',()=>updateTrigger(i));}";
  html += "const ot=document.getElementById('s'+i+'_ot');";
  html += "if(ot){ot.addEventListener('change',()=>updateTrigger(i));}";
  html += "updateTrigger(i);";
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
    if (server.hasArg("s" + idx + "_time_daily")) {
      uint8_t h; uint8_t mi; uint8_t se = 0;
      if (parseTimeHMS(server.arg("s" + idx + "_time_daily"), h, mi, se)) {
        s.timeHour = h;
        s.timeMinute = mi;
        s.timeSecond = se;
      }
    }
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
    if (mask != 0) s.weeklyMask = mask;
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
    if (server.hasArg("s" + idx + "_addr")) {
      s.oscAddress = normalizeOscAddress(server.arg("s" + idx + "_addr"));
    }
    if (s.type == SENSOR_ENCODER) {
      if (server.hasArg("s" + idx + "_enc_addr")) {
        s.oscAddress = normalizeOscAddress(server.arg("s" + idx + "_enc_addr"));
      }
      if (server.hasArg("s" + idx + "_btn_addr")) {
        s.buttonAddress = normalizeOscAddress(server.arg("s" + idx + "_btn_addr"));
      }
      if (s.oscAddress.length() == 0) s.oscAddress = "/encoder";
      if (s.buttonAddress.length() == 0) s.buttonAddress = "/button";
    }

    s.invert = server.hasArg("s" + idx + "_inv");
    s.activeHigh = server.hasArg("s" + idx + "_ah");
    s.pullup = server.hasArg("s" + idx + "_pu");
    s.cooldownEnabled = server.hasArg("s" + idx + "_cd_en");
    if (server.hasArg("s" + idx + "_cd_ms")) {
      uint32_t ms = static_cast<uint32_t>(server.arg("s" + idx + "_cd_ms").toInt());
      if (ms > 0) s.cooldownMs = ms;
    }

    if (server.hasArg("s" + idx + "_omin")) {
      s.outMin = server.arg("s" + idx + "_omin").toFloat();
    }
    if (server.hasArg("s" + idx + "_omax")) {
      s.outMax = server.arg("s" + idx + "_omax").toFloat();
    }
    if (server.hasArg("s" + idx + "_omode")) {
      s.outMode = sanitizeOutputMode(server.arg("s" + idx + "_omode").toInt());
    }
    if (server.hasArg("s" + idx + "_ot")) {
      s.outTarget = sanitizeOutputTarget(server.arg("s" + idx + "_ot").toInt());
    }
    if (server.hasArg("s" + idx + "_od")) {
      int dev = server.arg("s" + idx + "_od").toInt();
      if (dev >= 0 && dev < MAX_OSC_DEVICES) s.outDevice = static_cast<uint8_t>(dev);
    }
    if (server.hasArg("s" + idx + "_on")) {
      s.onString = server.arg("s" + idx + "_on");
      if (s.onString.length() == 0) s.onString = DEFAULT_ON_STRING;
    }
    if (server.hasArg("s" + idx + "_off")) {
      s.offString = server.arg("s" + idx + "_off");
      if (s.offString.length() == 0) s.offString = DEFAULT_OFF_STRING;
    }

    if (s.source != SRC_TIME) {
      if (server.hasArg("s" + idx + "_pin")) {
        int pin = server.arg("s" + idx + "_pin").toInt();
        if (pin >= 0 && pin <= 39 && !isReservedPin(pin)) {
          if (s.type != SENSOR_ANALOG || isAllowedAnalogPin(pin)) {
            s.pin = pin;
          }
        }
      }
      if (s.type == SENSOR_ENCODER) {
        int clk = server.arg("s" + idx + "_clk").toInt();
        int dt = server.arg("s" + idx + "_dt").toInt();
        int sw = server.arg("s" + idx + "_sw").toInt();
        if (clk >= 0 && clk <= 39 && !isReservedPin(clk)) s.encClkPin = clk;
        if (dt >= 0 && dt <= 39 && !isReservedPin(dt)) s.encDtPin = dt;
        if (sw >= 0 && sw <= 39 && !isReservedPin(sw)) s.encSwPin = sw;
      }
    }

    if (s.type == SENSOR_ANALOG && s.outMode == OUT_STRING) {
      s.outMode = OUT_FLOAT;
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
  applySensorPinModes();
  resetRuntimeState();

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

  html += "<div class='card'><b>OSC</b><br>";
  html += "<small>Device 1 is the default target.</small><br>";
  html += "<div style='margin-top:10px;'><b>OSC Client Devices</b><br>";

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
  html += "<button type='submit' name='action' value='dev_add'>Add OSC Device</button><br><br>";
  html += "</div>";

  html += "<div style='margin-top:10px;'><b>Test OSC</b><br>";
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
  html += "Debounce ms: <input name='click_db' type='number' value='" + String(config.clickDebounceMs) + "'><br>";
  html += "Multi-click gap ms: <input name='click_gap' type='number' value='" + String(config.multiClickGapMs) + "'><br>";
  html += "Long press ms: <input name='long_ms' type='number' value='" + String(config.longPressMs) + "'><br>";
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

  html += "<div class='card'><b>Updates</b><br>";
  html += "<div>Firmware version: <b>" + String(FIRMWARE_VERSION) + "</b></div>";
  html += "<a href='/update'>Open firmware updater</a><br><br>";
  html += "<button type='button' id='reboot_btn'>Reboot Device</button> ";
  html += "<button type='button' id='factory_reset_btn' class='danger'>Factory Reset</button>";
  html += "</div>";

  html += "<button type='submit' class='primary'>Save Settings</button>";
  html += "</form>";

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
  html += "fetch('/send_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString()});";
  html += "});}";
  html += "const resetBtn=document.getElementById('factory_reset_btn');";
  html += "const rebootBtn=document.getElementById('reboot_btn');";
  html += "if(resetBtn){resetBtn.addEventListener('click',()=>{if(!confirm('Factory reset will erase all settings. Continue?')){return;}fetch('/reset',{method:'POST'});});}";
  html += "if(rebootBtn){rebootBtn.addEventListener('click',()=>{if(!confirm('Reboot device now?')){return;}fetch('/reboot',{method:'POST'});});}";
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

  if (server.hasArg("wifi_ssid")) {
    config.wifiSsid = server.arg("wifi_ssid");
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

  server.sendHeader("Location", "/settings", true);
  server.send(303, "text/plain", "Saved.");
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
  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain", "Sent.");
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
  resetRuntimeState();
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
  server.on("/scan_wifi", HTTP_GET, handleScanNetworks);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/reboot", HTTP_POST, handleReboot);
  server.on("/logs", HTTP_GET, handleLogsPage);
  server.on("/logs.txt", HTTP_GET, handleLogsText);
  server.on("/update", HTTP_GET, handleUpdatePage);
  server.on("/update", HTTP_POST, handleUpdateFinished, handleUpdateUpload);
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
    const String& outAddress = isEncoder ? s.buttonAddress : s.oscAddress;
    int8_t prevLevel = lastLevel[i];
    bool cooldownTriggered = false;

    if (isEncoder) {
      int a = digitalRead(s.encClkPin);
      int b = digitalRead(s.encDtPin);
      int state = (a << 1) | b;
      if (lastEncState[i] < 0) {
        lastEncState[i] = state;
      }
      if (s.oscAddress.length() > 0 && lastEncState[i] >= 0) {
        static const int8_t kEncTable[16] = {
          0, -1, 1, 0,
          1, 0, 0, -1,
          -1, 0, 0, 1,
          0, 1, -1, 0
        };
        int idx = (lastEncState[i] << 2) | state;
        int8_t delta = kEncTable[idx];
        if (delta != 0) {
          encAccum[i] += delta;
          if (encAccum[i] >= 2) {
            sendOscInt(s.oscAddress.c_str(), 1);
            encAccum[i] = 0;
          } else if (encAccum[i] <= -2) {
            sendOscInt(s.oscAddress.c_str(), -1);
            encAccum[i] = 0;
          }
        }
      }
      lastEncState[i] = state;
      lastEncA[i] = a;
      lastEncB[i] = b;

      bool pressed = (digitalRead(s.encSwPin) == LOW);
      norm = pressed ? 1.0f : 0.0f;
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

    if (outAddress.length() == 0) continue;

    if (s.type == SENSOR_ANALOG) {
      norm = readAnalogNormalized(i, s);
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

    if (s.type == SENSOR_BUTTON && s.cooldownEnabled) {
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
      lastBool[i] = -1;
      lastInt[i] = -1;
      lastFloat[i] = NAN;
    }

    bool changed = false;

    if (s.outTarget == OUT_TARGET_REBOOT) {
      if (s.type == SENSOR_ANALOG || s.type == SENSOR_ENCODER) {
        continue;
      }
      bool pressed = (norm >= 0.5f);
      bool rising = pressed && (prevLevel <= 0);
      if (rising) {
        addLog("Reboot triggered by trigger.");
        delay(100);
        ESP.restart();
      }
      continue;
    }

      IPAddress outIp;
      uint16_t outPort;
      if (!getOscTarget(s.outDevice, outIp, outPort)) {
        continue;
      }

    if (s.outMode == OUT_STRING) {
      int8_t state = (norm >= 0.5f) ? 1 : 0;
      changed = (lastBool[i] == -1) || (state != lastBool[i]);
      if (changed) {
        const char* value = (state == 1) ? s.onString.c_str() : s.offString.c_str();
        if (value[0] != '\0') {
          sendOscStringTo(outIp, outPort, outAddress.c_str(), value);
          lastBool[i] = state;
        } else {
          changed = false;
        }
      }
    } else if (s.outMode == OUT_FLOAT) {
      float outputValue = mapOutput(s, norm);
      outputValue = snapOutput(s, outputValue);
      changed = isnan(lastFloat[i]) || fabsf(outputValue - lastFloat[i]) > 0.0005f;
      if (changed) {
        sendOscFloatTo(outIp, outPort, outAddress.c_str(), outputValue);
        lastFloat[i] = outputValue;
      }
    } else {
      float outputValue = mapOutput(s, norm);
      outputValue = snapOutput(s, outputValue);
      int intValue = lroundf(outputValue);
      if (s.type == SENSOR_ANALOG && lastInt[i] >= 0 && abs(intValue - lastInt[i]) <= DEAD_BAND) {
        intValue = lastInt[i];
      }
      changed = (intValue != lastInt[i]);
      if (changed) {
        sendOscIntTo(outIp, outPort, outAddress.c_str(), intValue);
        lastInt[i] = intValue;
      }
    }

    if (changed) {
      Serial.print("Sent ");
      Serial.print(outAddress);
      Serial.print(" = ");
      if (s.outMode == OUT_STRING) {
        Serial.println((norm >= 0.5f) ? s.onString : s.offString);
      } else if (s.outMode == OUT_FLOAT) {
        Serial.println(lastFloat[i], 4);
      } else {
        Serial.println(lastInt[i]);
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
