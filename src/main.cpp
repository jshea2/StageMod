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
#include <WebServer.h>
#include <LittleFS.h>
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

const int MAX_SENSORS = 6;

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

enum OutputMode {
  OUT_INT = 0,
  OUT_FLOAT = 1,
  OUT_STRING = 2
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

// Behavior
const int LOOP_DELAY_MS = 10;                  // read/send cycle
const int SAMPLES       = 8;                   // oversample to calm noise
const float ALPHA       = 0.30f;               // exponential smoothing 0..1
const int DEAD_BAND     = 1;                   // percent hysteresis to stop flicker

const uint32_t DEFAULT_CLICK_DEBOUNCE_MS = 30;   // debounce for buttons
const uint32_t DEFAULT_MULTI_CLICK_GAP_MS = 350; // max gap between clicks
const uint32_t DEFAULT_LONG_PRESS_MS = 700;      // long-press threshold

const uint16_t LOCAL_PORT = 9000;              // Local UDP port
const float SNAP_PERCENT = 1.0f;               // snap edges to min/max

struct SensorConfig {
  bool enabled;
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
  String onString;
  String offString;
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
  bool heartbeatEnabled;
  String heartbeatAddress;
  uint32_t heartbeatMs;
  uint32_t clickDebounceMs;
  uint32_t multiClickGapMs;
  uint32_t longPressMs;
  SensorConfig sensors[MAX_SENSORS];
};

Config config;

float ema[MAX_SENSORS];     // smoothed ADC per sensor
int lastInt[MAX_SENSORS];   // last sent int value per sensor
float lastFloat[MAX_SENSORS]; // last sent float value per sensor
int8_t lastBool[MAX_SENSORS]; // last sent on/off state per sensor
int8_t lastLevel[MAX_SENSORS]; // last raw level per sensor
bool toggleState[MAX_SENSORS]; // toggle state per sensor
int8_t lastEncA[MAX_SENSORS];
int8_t lastEncB[MAX_SENSORS];
int8_t lastEncState[MAX_SENSORS];
int8_t encAccum[MAX_SENSORS];
uint32_t lastButtonTriggerMs[MAX_SENSORS];
uint8_t clickCount[MAX_SENSORS];
bool multiPrevPressed[MAX_SENSORS];
bool multiLongFired[MAX_SENSORS];
unsigned long multiLastChangeMs[MAX_SENSORS];
unsigned long multiPressStartMs[MAX_SENSORS];
unsigned long multiLastReleaseMs[MAX_SENSORS];
unsigned long lastAnalogPrintMs[MAX_SENSORS];
WiFiUDP udp;
WebServer server(80);
Preferences prefs;
bool pendingNetApply = false;
bool pendingMdnsRestart = false;
unsigned long pendingApplyAt = 0;
bool wifiApMode = false;
unsigned long wifiReconnectStarted = 0;
DNSServer dnsServer;
unsigned long resetPressStartMs = 0;
unsigned long lastHeartbeatSentMs = 0;
bool littleFsOk = false;

static String ipToString(const IPAddress& ip);
static bool parseIpString(const String& s, IPAddress& out);
static bool isValidIp(const IPAddress& ip);
static String buildApSsid();
static void startWifiAp();

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
  return static_cast<SensorType>(value);
}

static OutputMode sanitizeOutputMode(int value) {
  if (value < OUT_INT || value > OUT_STRING) return DEFAULT_OUT_MODE;
  return static_cast<OutputMode>(value);
}

static NetMode sanitizeNetworkMode(int value) {
  if (value < NET_ETHERNET || value > NET_WIFI) return DEFAULT_NET_MODE;
  return static_cast<NetMode>(value);
}

static bool isReservedPin(int pin) {
  return pin == RESET_BUTTON_PIN || pin == 0 || pin == 1 || pin == 2 || pin == 3 || pin == 4 || pin == 12 || pin == 14 || pin == 15;
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
  s.onString = DEFAULT_ON_STRING;
  s.offString = DEFAULT_OFF_STRING;
  return s;
}

static String sensorKey(int index, const char* suffix) {
  return String("s") + String(index) + "_" + suffix;
}

static void resetRuntimeState() {
  for (int i = 0; i < MAX_SENSORS; i++) {
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
  }
  lastHeartbeatSentMs = 0;
}

static void applySensorPinModes() {
  for (int i = 0; i < MAX_SENSORS; i++) {
    const SensorConfig& sensor = config.sensors[i];
    if (!sensor.enabled) continue;
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
  for (int i = 0; i < MAX_SENSORS; i++) {
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
        if (pinsA[ia] == pinsB[ib]) return true;
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
  config.heartbeatEnabled = prefs.getBool("hb_en", false);
  config.heartbeatAddress = prefs.getString("hb_addr", "/heartbeat");
  if (config.heartbeatAddress.length() == 0) config.heartbeatAddress = "/heartbeat";
  config.heartbeatMs = prefs.getUInt("hb_ms", 5000);
  if (config.heartbeatMs == 0) config.heartbeatMs = 5000;
  config.clickDebounceMs = prefs.getUInt("click_db", DEFAULT_CLICK_DEBOUNCE_MS);
  if (config.clickDebounceMs == 0) config.clickDebounceMs = DEFAULT_CLICK_DEBOUNCE_MS;
  config.multiClickGapMs = prefs.getUInt("click_gap", DEFAULT_MULTI_CLICK_GAP_MS);
  if (config.multiClickGapMs == 0) config.multiClickGapMs = DEFAULT_MULTI_CLICK_GAP_MS;
  config.longPressMs = prefs.getUInt("long_ms", DEFAULT_LONG_PRESS_MS);
  if (config.longPressMs == 0) config.longPressMs = DEFAULT_LONG_PRESS_MS;
  config.passwordEnabled = prefs.getBool("pwd_en", false);
  config.password = prefs.getString("pwd", "");
  if (!config.passwordEnabled || config.password.length() == 0) {
    config.passwordEnabled = false;
    config.password = "";
  }

  bool hasNew = prefs.isKey(sensorKey(0, "addr").c_str());
  for (int i = 0; i < MAX_SENSORS; i++) {
    config.sensors[i] = defaultSensorConfig(i);
  }

  if (hasNew) {
    for (int i = 0; i < MAX_SENSORS; i++) {
      SensorConfig s = config.sensors[i];
      s.enabled = prefs.getBool(sensorKey(i, "en").c_str(), s.enabled);
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
      s.onString = prefs.getString(sensorKey(i, "on").c_str(), s.onString);
      s.offString = prefs.getString(sensorKey(i, "off").c_str(), s.offString);

      if (s.oscAddress.length() == 0) s.oscAddress = defaultOscAddress(i);
      if (s.buttonAddress.length() == 0) s.buttonAddress = "/button";
      if (s.type == SENSOR_ANALOG && s.outMode == OUT_STRING) s.outMode = OUT_FLOAT;
      config.sensors[i] = s;
    }
  } else {
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
  prefs.putBool("hb_en", config.heartbeatEnabled);
  prefs.putString("hb_addr", config.heartbeatAddress);
  prefs.putUInt("hb_ms", config.heartbeatMs);
  prefs.putUInt("click_db", config.clickDebounceMs);
  prefs.putUInt("click_gap", config.multiClickGapMs);
  prefs.putUInt("long_ms", config.longPressMs);
  prefs.putBool("pwd_en", config.passwordEnabled);
  prefs.putString("pwd", config.password);

  for (int i = 0; i < MAX_SENSORS; i++) {
    const SensorConfig& s = config.sensors[i];
    prefs.putBool(sensorKey(i, "en").c_str(), s.enabled);
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
    prefs.putString(sensorKey(i, "on").c_str(), s.onString);
    prefs.putString(sensorKey(i, "off").c_str(), s.offString);
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

static void startMdns() {
  if (MDNS.begin(config.hostname.c_str())) {
    Serial.print("mDNS ready: http://");
    Serial.print(config.hostname);
    Serial.println(".local/");
  } else {
    Serial.println("mDNS failed to start.");
  }
}

static void sendOscInt(const char* address, int32_t value) {
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

  udp.beginPacket(config.clientIp, config.clientPort);
  udp.write(packet, idx);
  udp.endPacket();
}

static void sendOscFloat(const char* address, float value) {
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

  udp.beginPacket(config.clientIp, config.clientPort);
  udp.write(packet, idx);
  udp.endPacket();
}

static void sendOscString(const char* address, const char* value) {
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

  udp.beginPacket(config.clientIp, config.clientPort);
  udp.write(packet, idx);
  udp.endPacket();
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

static void sendTriggerOutput(const SensorConfig& sensor, const String& addr) {
  if (addr.length() == 0) return;
  if (sensor.outMode == OUT_STRING) {
    const char* value = sensor.onString.c_str();
    if (value[0] == '\0') return;
    sendOscString(addr.c_str(), value);
  } else if (sensor.outMode == OUT_FLOAT) {
    float outputValue = mapOutput(sensor, 1.0f);
    outputValue = snapOutput(sensor, outputValue);
    sendOscFloat(addr.c_str(), outputValue);
  } else {
    float outputValue = mapOutput(sensor, 1.0f);
    outputValue = snapOutput(sensor, outputValue);
    int intValue = lroundf(outputValue);
    sendOscInt(addr.c_str(), intValue);
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

static void handleRoot() {
  if (!ensureAuthenticated()) return;
  String html;
  html.reserve(13000);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>StageMod Settings</title>";
  html += "<style>";
  html += ":root{--bg:#f4f1ea;--ink:#1a1a1a;--muted:#6c6c6c;--card:#ffffff;--accent:#0b6b6f;--danger:#a02a2a;}";
  html += "body{margin:0;font-family:'Avenir Next','Trebuchet MS','Segoe UI',sans-serif;background:linear-gradient(135deg,#f4f1ea 0%,#e7f0ef 100%);color:var(--ink);}";
  html += ".page{max-width:760px;margin:0 auto;padding:20px 16px 40px;}";
  html += ".header{display:flex;flex-wrap:wrap;gap:12px;align-items:center;justify-content:space-between;margin-bottom:8px;}";
  html += ".title{font-size:22px;font-weight:700;letter-spacing:0.5px;}";
  html += ".actions{display:flex;gap:8px;align-items:center;flex-wrap:wrap;}";
  html += ".tag{font-size:12px;color:var(--muted);padding:4px 8px;border:1px solid #ddd;border-radius:999px;background:#fff;}";
  html += ".status{display:flex;flex-wrap:wrap;gap:10px;font-size:13px;color:var(--muted);margin:8px 0 14px;}";
  html += ".alert{background:#fff3d6;border:1px solid #f0d39b;padding:8px 10px;border-radius:8px;margin:8px 0;font-size:13px;}";
  html += ".card{background:var(--card);border:1px solid #e2e2e2;border-radius:12px;padding:12px 12px 6px;margin:10px 0;}";
  html += "fieldset{border:1px solid #e2e2e2;border-radius:12px;padding:12px;margin:10px 0;background:#fff;}";
  html += "legend{padding:0 6px;color:var(--muted);}";
  html += "input,select{width:100%;max-width:320px;padding:6px 8px;margin:2px 0 8px;border:1px solid #cfcfcf;border-radius:6px;background:#fff;}";
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
  html += "<button type='button' id='pin_help_btn'>Board Pin Help</button>";
  html += "<span class='tag'>Olimex ESP32-PoE</span>";
  html += "</div></div>";
  html += "<div id='pin_help_modal' style='display:none;position:fixed;inset:0;background:rgba(0,0,0,0.6);'>";
  html += "<div class='modal'>";
  html += "<h3>Pin Help</h3>";
  html += "<p><b>Analog (ADC1, safe with WiFi/AP):</b> GPIO32, GPIO33, GPIO34, GPIO35, GPIO36, GPIO39.</p>";
  html += "<p><b>Avoid ADC2 for analog when WiFi/AP is on:</b> GPIO0, GPIO2, GPIO4, GPIO12-15, GPIO25-27.</p>";
  html += "<p><b>Input-only pins:</b> GPIO34, GPIO35, GPIO36, GPIO39.</p>";
  html += "<p><b>Boot-strap pins:</b> GPIO0, GPIO2, GPIO12, GPIO15 (avoid pulling these during boot).</p>";
  html += "<p><b>USB serial:</b> GPIO1/GPIO3 used for programming.</p>";
  html += "<p><b>Reserved:</b> GPIO34 used for factory reset button (BUT1).</p>";
  html += "<img src='/pinout.png' alt='ESP32-POE pinout' style='width:100%;height:auto;margin-top:8px;border-radius:8px;border:1px solid #ddd;'>";
  html += "<button type='button' id='pin_help_close'>Close</button>";
  html += "</div></div>";
  if (wifiApMode) {
    html += "<div class='alert'>AP mode: connect to SSID ";
    html += buildApSsid();
    html += " then open http://" + getLocalIp().toString() + "/</div>";
  }
  html += "<div class='status'>";
  html += "<div>IP: " + getLocalIp().toString() + "</div>";
  html += "<div>mDNS: http://" + config.hostname + ".local/</div>";
  html += "</div>";
  html += "<small>Multiple devices on the same network will connect as http://stagemod-x.local</small>";

  bool anyConflict = false;
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (hasPinConflict(i)) {
      anyConflict = true;
      break;
    }
  }
  if (anyConflict) {
    html += "<p style='color:#b00;'>Pin conflict detected: two triggers share the same GPIO.</p>";
  }

  html += "<form method='POST' action='/'>";
  html += "<div class='card'>";
  html += "<b>Network</b><br>";
  html += "Hostname: <input name='hostname' type='text' value='" + config.hostname + "'><br>";
  html += "Client IP: <input name='client_ip' type='text' value='" + ipToString(config.clientIp) + "'><br>";
  html += "Client port: <input name='client_port' type='number' value='" + String(config.clientPort) + "'><br>";
#if USE_ETHERNET
  html += "Network: <select name='net_mode' id='net_mode'>";
  html += "<option value='0' " + String(config.netMode == NET_ETHERNET ? "selected" : "") + ">Ethernet</option>";
  html += "<option value='1' " + String(config.netMode == NET_WIFI ? "selected" : "") + ">WiFi</option>";
  html += "</select><br>";
#else
  html += "<input type='hidden' name='net_mode' id='net_mode' value='1'>";
#endif
  html += "<div id='wifi_fields'>";
  html += "WiFi SSID: <input name='wifi_ssid' type='text' value='" + config.wifiSsid + "'><br>";
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
  html += "<div class='card'>";
  html += "<b>Test OSC</b><br>";
  html += "Address: <input id='test_addr' name='test_addr' type='text' value='/test'><br>";
  html += "Output type: <select id='test_type' name='test_type'>";
  html += "<option value='int'>Int</option>";
  html += "<option value='float'>Float</option>";
  html += "<option value='string'>String</option>";
  html += "</select><br>";
  html += "Output: <input id='test_value' name='test_value' type='text' value='1'><br>";
  html += "<button type='button' id='test_send_btn'>Send</button>";
  html += "</div>";
  html += "<div class='card'>";
  html += "<b>Firmware Update</b><br>";
  html += "Upload a new .bin over your local network.";
  html += " <a href='/update'>Open updater</a><br>";
  html += "</div>";
  html += "<div class='card'>";
  html += "<b>Heartbeat</b><br>";
  html += "Enable: <input name='hb_en' type='checkbox' " + String(config.heartbeatEnabled ? "checked" : "") + "><br>";
  html += "Address: <input name='hb_addr' type='text' value='" + config.heartbeatAddress + "'><br>";
  html += "Interval ms: <input name='hb_ms' type='number' value='" + String(config.heartbeatMs) + "'><br>";
  html += "</div>";
  html += "<div class='card'>";
  html += "<b>Trigger timing</b><br>";
  html += "Debounce ms: <input name='click_db' type='number' value='" + String(config.clickDebounceMs) + "'><br>";
  html += "Multi-click gap ms: <input name='click_gap' type='number' value='" + String(config.multiClickGapMs) + "'><br>";
  html += "Long press ms: <input name='long_ms' type='number' value='" + String(config.longPressMs) + "'><br>";
  html += "</div>";
  html += "<hr>";

  for (int i = 0; i < MAX_SENSORS; i++) {
    const SensorConfig& s = config.sensors[i];
    String idx = String(i);
    html += "<fieldset>";
    html += "<legend><b>Trigger " + String(i + 1) + "</b></legend>";
    html += "Enable: <input name='s" + idx + "_en' type='checkbox' " + String(s.enabled ? "checked" : "") + "><br>";
    html += "Type: <select name='s" + idx + "_type' id='s" + idx + "_type'>";
    html += "<option value='0' " + String(s.type == SENSOR_ANALOG ? "selected" : "") + ">Analog (pot/slider)</option>";
    html += "<option value='1' " + String(s.type == SENSOR_BUTTON ? "selected" : "") + ">Button (momentary)</option>";
    html += "<option value='2' " + String(s.type == SENSOR_TOGGLE ? "selected" : "") + ">Button (toggle)</option>";
    html += "<option value='3' " + String(s.type == SENSOR_DIGITAL ? "selected" : "") + ">Digital (touch/motion)</option>";
    html += "<option value='4' " + String(s.type == SENSOR_ENCODER ? "selected" : "") + ">Encoder</option>";
    html += "<option value='5' " + String(s.type == SENSOR_BUTTON_DOUBLE ? "selected" : "") + ">Button (double click)</option>";
    html += "<option value='6' " + String(s.type == SENSOR_BUTTON_TRIPLE ? "selected" : "") + ">Button (triple click)</option>";
    html += "<option value='7' " + String(s.type == SENSOR_BUTTON_LONG ? "selected" : "") + ">Button (long press)</option>";
    html += "</select><br>";
    html += "<div id='s" + idx + "_pin_block'>";
    html += "GPIO Pin: <input name='s" + idx + "_pin' type='number' value='" + String(s.pin) + "'><br>";
    html += "<small>Analog pins allowed: 32,35,36. Reserved pins: 0,1,2,3,4,12,14,15,34</small><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_addr_block'>";
    html += "OSC address: <input id='s" + idx + "_addr' name='s" + idx + "_addr' type='text' value='" + s.oscAddress + "'><br>";
    html += "</div>";

    html += "<div id='s" + idx + "_analog'>";
    html += "Invert: <input name='s" + idx + "_inv' type='checkbox' " + String(s.invert ? "checked" : "") + "><br>";
    html += "</div>";

    html += "<div id='s" + idx + "_digital'>";
    html += "Active high: <input id='s" + idx + "_ah' name='s" + idx + "_ah' type='checkbox' " + String(s.activeHigh ? "checked" : "") + "><br>";
    html += "Use pullup: <input name='s" + idx + "_pu' type='checkbox' " + String(s.pullup ? "checked" : "") + "><br>";
    html += "</div>";

    html += "<div id='s" + idx + "_encoder'>";
    html += "<b>Encoder</b><br>";
    html += "Encoder address: <input id='s" + idx + "_enc_addr' name='s" + idx + "_enc_addr' type='text' value='" + s.oscAddress + "'><br>";
    html += "CLK pin: <input name='s" + idx + "_clk' type='number' value='" + String(s.encClkPin) + "'><br>";
    html += "DT pin: <input name='s" + idx + "_dt' type='number' value='" + String(s.encDtPin) + "'><br>";
    html += "SW pin: <input name='s" + idx + "_sw' type='number' value='" + String(s.encSwPin) + "'><br>";
    html += "<small>Encoder sends -1/+1 on each step.</small><br>";
    html += "</div>";
    html += "<div id='s" + idx + "_button_addr'>";
    html += "<b>Button</b><br>";
    html += "Button address: <input id='s" + idx + "_btn_addr' name='s" + idx + "_btn_addr' type='text' value='" + s.buttonAddress + "'><br>";
    html += "</div>";
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
    html += "</fieldset>";
  }

  html += "<button type='submit' class='primary'>Save</button>";
  html += "<div class='card'>";
  html += "<b>Security</b><br>";
  html += "Username: <input name='username' type='text' value='" + config.username + "'><br>";
  html += "Enable password: <input name='pwd_enabled' type='checkbox' " + String(config.passwordEnabled ? "checked" : "") + "><br>";
  html += "New password: <input name='pwd_new' type='password' value=''><br>";
  html += "<small>Leave new password blank to keep current.</small><br>";
  html += "</div>";
  html += "<hr>";
  html += "<div style='margin-top:8px;'>";
  html += "<button type='button' id='factory_reset_btn' class='danger'>Factory Reset</button>";
  html += "</div>";
  html += "<script>";
  html += "function updateSensor(i){";
  html += "const tSel=document.getElementById('s'+i+'_type');";
  html += "const t=tSel.value;";
  html += "const om=document.getElementById('s'+i+'_omode');";
  html += "const m=om.value;";
  html += "const a=document.getElementById('s'+i+'_analog');";
  html += "const d=document.getElementById('s'+i+'_digital');";
  html += "const s=document.getElementById('s'+i+'_string');";
  html += "const r=document.getElementById('s'+i+'_range');";
  html += "const opt=document.getElementById('s'+i+'_opt_string');";
  html += "const ah=document.getElementById('s'+i+'_ah');";
  html += "const addr=document.getElementById('s'+i+'_addr_block');";
  html += "const addrInput=document.getElementById('s'+i+'_addr');";
  html += "const pin=document.getElementById('s'+i+'_pin_block');";
  html += "const enc=document.getElementById('s'+i+'_encoder');";
    html += "const btnAddrBlock=document.getElementById('s'+i+'_button_addr');";
  html += "const cd=document.getElementById('s'+i+'_cooldown');";
  html += "const encAddr=document.getElementById('s'+i+'_enc_addr');";
  html += "const btnAddr=document.getElementById('s'+i+'_btn_addr');";
  html += "const isEnc=(t==='4');";
  html += "const prev=tSel.dataset.prev||t;";
  html += "a.style.display=(t==='0')?'block':'none';";
  html += "d.style.display=(t!=='0'&&!isEnc)?'block':'none';";
  html += "if(t==='0'){";
  html += "opt.disabled=true;";
  html += "if(om.value==='2'){om.value='1';}";
  html += "}else{";
  html += "opt.disabled=false;";
  html += "}";
  html += "s.style.display=(om.value==='2')?'block':'none';";
  html += "r.style.display=(om.value==='2')?'none':'block';";
  html += "addr.style.display=isEnc?'none':'block';";
  html += "pin.style.display=isEnc?'none':'block';";
  html += "enc.style.display=isEnc?'block':'none';";
  html += "btnAddrBlock.style.display=isEnc?'block':'none';";
  html += "";
  html += "if(addrInput){addrInput.disabled=isEnc;}";
  html += "if(encAddr){encAddr.disabled=!isEnc;}";
  html += "if(btnAddr){btnAddr.disabled=!isEnc;}";
  html += "cd.style.display=(t==='1'||t==='3')?'block':'none';";
  html += "if(isEnc){";
  html += "if(encAddr&&encAddr.value.trim()===''){encAddr.value='/encoder';}";
  html += "if(btnAddr&&btnAddr.value.trim()===''){btnAddr.value='/button';}";
  html += "}";
  html += "if(t==='3'&&prev!=='3'&&ah){ah.checked=true;}";
  html += "tSel.dataset.prev=t;";
  html += "}";
  html += "const dh=document.getElementById('dhcp_enabled');";
  html += "const sf=document.getElementById('static_fields');";
  html += "const nm=document.getElementById('net_mode');";
  html += "const wf=document.getElementById('wifi_fields');";
  html += "const helpBtn=document.getElementById('pin_help_btn');";
  html += "const helpModal=document.getElementById('pin_help_modal');";
  html += "const helpClose=document.getElementById('pin_help_close');";
  html += "const resetBtn=document.getElementById('factory_reset_btn');";
  html += "const form=document.querySelector('form[action=\"/\"]');";
  html += "const testBtn=document.getElementById('test_send_btn');";
  html += "const testAddr=document.getElementById('test_addr');";
  html += "const testType=document.getElementById('test_type');";
  html += "const testValue=document.getElementById('test_value');";
  html += "const reservedPins=new Set([0,1,2,3,4,12,14,15,34]);";
  html += "const analogPins=new Set([32,35,36]);";
  html += "function pinInputError(){";
  html += "for(let i=0;i<" + String(MAX_SENSORS) + ";i++){";
  html += "const el=document.querySelector('input[name=\"s'+i+'_pin\"]');";
  html += "const typeEl=document.getElementById('s'+i+'_type');";
  html += "if(!el){continue;}";
  html += "const v=parseInt(el.value,10);";
  html += "if(!isNaN(v)&&reservedPins.has(v)){return v;}";
  html += "if(typeEl&&typeEl.value==='0'&&!isNaN(v)&&!analogPins.has(v)){return v;}";
  html += "if(typeEl&&typeEl.value==='4'){";
  html += "const clk=document.querySelector('input[name=\"s'+i+'_clk\"]');";
  html += "const dt=document.querySelector('input[name=\"s'+i+'_dt\"]');";
  html += "const sw=document.querySelector('input[name=\"s'+i+'_sw\"]');";
  html += "const pins=[clk,dt,sw];";
  html += "for(let p=0;p<pins.length;p++){";
  html += "const val=parseInt(pins[p].value,10);";
  html += "if(!isNaN(val)&&reservedPins.has(val)){return val;}";
  html += "}";
  html += "}";
  html += "}";
  html += "return null;";
  html += "}";
  html += "function toggleDhcp(){sf.style.display=dh.checked?'none':'block';}";
  html += "function toggleNet(){";
  html += "if(!nm){wf.style.display='block';return;}";
  html += "wf.style.display=(nm.value==='1')?'block':'none';";
  html += "}";
  html += "helpBtn.addEventListener('click',()=>{helpModal.style.display='block';});";
  html += "helpClose.addEventListener('click',()=>{helpModal.style.display='none';});";
  html += "helpModal.addEventListener('click',(e)=>{if(e.target===helpModal){helpModal.style.display='none';}});";
  html += "resetBtn.addEventListener('click',()=>{";
  html += "if(!confirm('Factory reset will erase all settings. Continue?')){return;}";
  html += "fetch('/reset',{method:'POST'});";
  html += "});";
  html += "if(testBtn&&testAddr){testBtn.addEventListener('click',()=>{";
  html += "const data=new URLSearchParams();";
  html += "data.set('test_addr',testAddr.value||'/test');";
  html += "if(testType){data.set('test_type',testType.value);}";
  html += "if(testValue){data.set('test_value',testValue.value);}";
  html += "fetch('/send_test',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:data.toString()});";
  html += "});}";
  html += "if(form){form.addEventListener('submit',(e)=>{";
  html += "const bad=pinInputError();";
  html += "if(bad!==null){alert('GPIO '+bad+' is not allowed for this selection.');e.preventDefault();}});}";
  html += "dh.addEventListener('change',toggleDhcp);";
  html += "if(nm){nm.addEventListener('change',toggleNet);}";
  html += "toggleDhcp();";
  html += "toggleNet();";
  html += "for(let i=0;i<" + String(MAX_SENSORS) + ";i++){";
  html += "const tSel=document.getElementById('s'+i+'_type');";
  html += "tSel.dataset.prev=tSel.value;";
  html += "tSel.addEventListener('change',()=>updateSensor(i));";
  html += "document.getElementById('s'+i+'_omode').addEventListener('change',()=>updateSensor(i));";
  html += "updateSensor(i);";
  html += "}";
  html += "</script>";
  html += "</form></div></body></html>";
  server.send(200, "text/html", html);
}

static void handleSave() {
  if (!ensureAuthenticated()) return;
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

  if (server.hasArg("client_ip")) {
    IPAddress parsed;
    if (parseIpString(server.arg("client_ip"), parsed)) {
      config.clientIp = parsed;
    }
  }

  if (server.hasArg("client_port")) {
    int port = server.arg("client_port").toInt();
    if (port > 0 && port <= 65535) config.clientPort = static_cast<uint16_t>(port);
  }

#if !USE_ETHERNET
  if (server.hasArg("wifi_ssid")) {
    config.wifiSsid = server.arg("wifi_ssid");
  }
  if (server.hasArg("wifi_pass")) {
    String pass = server.arg("wifi_pass");
    if (pass.length() > 0) config.wifiPass = pass;
  }
#endif

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

  config.heartbeatEnabled = server.hasArg("hb_en");
  if (server.hasArg("hb_addr")) {
    config.heartbeatAddress = normalizeOscAddress(server.arg("hb_addr"));
  }
  if (server.hasArg("hb_ms")) {
    uint32_t ms = static_cast<uint32_t>(server.arg("hb_ms").toInt());
    if (ms > 0) config.heartbeatMs = ms;
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

  for (int i = 0; i < MAX_SENSORS; i++) {
    SensorConfig s = config.sensors[i];
    String idx = String(i);

    s.enabled = server.hasArg("s" + idx + "_en");

    if (server.hasArg("s" + idx + "_type")) {
      s.type = sanitizeSensorType(server.arg("s" + idx + "_type").toInt());
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

    if (server.hasArg("s" + idx + "_on")) {
      s.onString = server.arg("s" + idx + "_on");
      if (s.onString.length() == 0) s.onString = DEFAULT_ON_STRING;
    }

    if (server.hasArg("s" + idx + "_off")) {
      s.offString = server.arg("s" + idx + "_off");
      if (s.offString.length() == 0) s.offString = DEFAULT_OFF_STRING;
    }

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

    if (s.type == SENSOR_ANALOG && s.outMode == OUT_STRING) {
      s.outMode = OUT_FLOAT;
    }

    config.sensors[i] = s;
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
  pendingNetApply = true;
  pendingApplyAt = millis();
  pendingMdnsRestart = (config.hostname != oldHostname);

  applySensorPinModes();
  resetRuntimeState();

  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain", "Saved.");
}

static void handleSendTestOsc() {
  if (!ensureAuthenticated()) return;
  String addr = "/test";
  String type = "int";
  String value = "1";
  if (server.hasArg("test_addr")) {
    addr = normalizeOscAddress(server.arg("test_addr"));
  }
  if (server.hasArg("test_type")) {
    type = server.arg("test_type");
  }
  if (server.hasArg("test_value")) {
    value = server.arg("test_value");
  }
  if (type == "string") {
    sendOscString(addr.c_str(), value.c_str());
  } else if (type == "float") {
    sendOscFloat(addr.c_str(), value.toFloat());
  } else {
    sendOscInt(addr.c_str(), value.toInt());
  }
  server.sendHeader("Location", "/", true);
  server.send(303, "text/plain", "Sent.");
}

static void doFactoryReset(bool respond) {
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
  server.send(200, "text/plain", ok ? "Update successful. Rebooting..." : "Update failed.");
  delay(200);
  if (ok) {
    ESP.restart();
  }
}

static void handleUpdateUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    Update.begin(UPDATE_SIZE_UNKNOWN);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    Update.end(true);
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  loadConfig();
  resetRuntimeState();
  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  Serial.println("Starting network...");
#if USE_ETHERNET
  ETH.begin();
#endif
  applyNetworkConfig();
  if (waitForNetwork(15000)) {
    Serial.print("Network OK. ESP32 IP: ");
    Serial.println(getLocalIp());
    udp.begin(LOCAL_PORT);
    startMdns();
  } else {
    Serial.println("Network failed to connect.");
    startWifiAp();
  }

  littleFsOk = LittleFS.begin();
  if (!littleFsOk) {
    Serial.println("LittleFS mount failed.");
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/", HTTP_POST, handleSave);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/send_test", HTTP_POST, handleSendTestOsc);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/update", HTTP_GET, handleUpdatePage);
  server.on("/update", HTTP_POST, handleUpdateFinished, handleUpdateUpload);
  server.on("/pinout.png", HTTP_GET, []() {
    if (!littleFsOk) {
      server.send(404, "text/plain", "Not found");
      return;
    }
    File file = LittleFS.open("/pinout.png", "r");
    if (!file) {
      server.send(404, "text/plain", "Not found");
      return;
    }
    server.streamFile(file, "image/png");
    file.close();
  });
#if !USE_ETHERNET
  server.on("/generate_204", HTTP_GET, []() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/hotspot-detect.html", HTTP_GET, []() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/fwlink", HTTP_GET, []() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.on("/ncsi.txt", HTTP_GET, []() {
    server.sendHeader("Location", "/", true);
    server.send(200, "text/plain", "OK");
  });
#endif
  server.onNotFound([]() {
    server.sendHeader("Location", "/", true);
    server.send(302, "text/plain", "");
  });
  server.begin();
  Serial.println("Web server started.");

  applySensorPinModes();
}

void loop() {
  if (digitalRead(RESET_BUTTON_PIN) == LOW) {
    if (resetPressStartMs == 0) resetPressStartMs = millis();
    if (millis() - resetPressStartMs >= RESET_HOLD_MS) {
      doFactoryReset(false);
    }
  } else {
    resetPressStartMs = 0;
  }

  for (int i = 0; i < MAX_SENSORS; i++) {
    SensorConfig& s = config.sensors[i];
    if (!s.enabled) continue;
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

    if ((s.type == SENSOR_BUTTON || s.type == SENSOR_DIGITAL) && s.cooldownEnabled) {
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

    if (s.outMode == OUT_STRING) {
      int8_t state = (norm >= 0.5f) ? 1 : 0;
      changed = (lastBool[i] == -1) || (state != lastBool[i]);
      if (changed) {
        const char* value = (state == 1) ? s.onString.c_str() : s.offString.c_str();
        if (value[0] != '\0') {
          sendOscString(outAddress.c_str(), value);
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
        sendOscFloat(outAddress.c_str(), outputValue);
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
        sendOscInt(outAddress.c_str(), intValue);
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
    } else if (!wifiApMode && millis() - wifiReconnectStarted > 10000) {
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

  if (wifiApMode) {
    dnsServer.processNextRequest();
  }

  if (config.heartbeatEnabled && config.heartbeatAddress.length() > 0) {
    unsigned long now = millis();
    if (now - lastHeartbeatSentMs >= config.heartbeatMs) {
      lastHeartbeatSentMs = now;
      sendOscInt(config.heartbeatAddress.c_str(), 1);
    }
  }
}
