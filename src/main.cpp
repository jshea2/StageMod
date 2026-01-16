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

// =========================
// Config (edit these)
// =========================
// Defaults (editable via web UI)
const char* DEFAULT_HOSTNAME = "esp32-osc";
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

const int MAX_SENSORS = 6;

enum NetMode {
  NET_ETHERNET = 0,
  NET_WIFI = 1
};

enum SensorType {
  SENSOR_ANALOG = 0,
  SENSOR_BUTTON = 1,
  SENSOR_TOGGLE = 2,
  SENSOR_DIGITAL = 3
};

enum OutputMode {
  OUT_INT = 0,
  OUT_FLOAT = 1,
  OUT_STRING = 2
};

const int DEFAULT_ANALOG_PIN = 32;
const int DEFAULT_DIGITAL_PIN = 4;
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

const uint16_t LOCAL_PORT = 9000;              // Local UDP port
const float SNAP_PERCENT = 1.0f;               // snap edges to min/max

struct SensorConfig {
  bool enabled;
  SensorType type;
  int pin;
  bool invert;
  bool activeHigh;
  bool pullup;
  String oscAddress;
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
  SensorConfig sensors[MAX_SENSORS];
};

Config config;

float ema[MAX_SENSORS];     // smoothed ADC per sensor
int lastInt[MAX_SENSORS];   // last sent int value per sensor
float lastFloat[MAX_SENSORS]; // last sent float value per sensor
int8_t lastBool[MAX_SENSORS]; // last sent on/off state per sensor
int8_t lastLevel[MAX_SENSORS]; // last raw level per sensor
bool toggleState[MAX_SENSORS]; // toggle state per sensor
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
  snprintf(buf, sizeof(buf), "ESP32osc-Setup-%02X%02X", mac[4], mac[5]);
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
    case 4: return 39;
    default: return 25;
  }
}

static SensorType sanitizeSensorType(int value) {
  if (value < SENSOR_ANALOG || value > SENSOR_DIGITAL) return SENSOR_ANALOG;
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
  return pin == RESET_BUTTON_PIN || pin == 0 || pin == 1 || pin == 2 || pin == 3 || pin == 12 || pin == 15;
}

static SensorConfig defaultSensorConfig(int index) {
  SensorConfig s;
  s.enabled = (index == 0);
  s.type = SENSOR_ANALOG;
  s.pin = defaultSensorPin(index);
  s.invert = DEFAULT_INVERT;
  s.activeHigh = DEFAULT_DIGITAL_ACTIVE_HIGH;
  s.pullup = DEFAULT_DIGITAL_PULLUP;
  s.oscAddress = defaultOscAddress(index);
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
  }
}

static void applySensorPinModes() {
  for (int i = 0; i < MAX_SENSORS; i++) {
    const SensorConfig& sensor = config.sensors[i];
    if (!sensor.enabled) continue;
    if (sensor.type != SENSOR_ANALOG) {
      pinMode(sensor.pin, sensor.pullup ? INPUT_PULLUP : INPUT);
    }
  }
}

static bool hasPinConflict(int index) {
  if (!config.sensors[index].enabled) return false;
  int pin = config.sensors[index].pin;
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (i == index) continue;
    if (config.sensors[i].enabled && config.sensors[i].pin == pin) {
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
      s.invert = prefs.getBool(sensorKey(i, "inv").c_str(), s.invert);
      s.activeHigh = prefs.getBool(sensorKey(i, "ah").c_str(), s.activeHigh);
      s.pullup = prefs.getBool(sensorKey(i, "pu").c_str(), s.pullup);
      s.oscAddress = prefs.getString(sensorKey(i, "addr").c_str(), s.oscAddress);
      s.outMin = prefs.getFloat(sensorKey(i, "omin").c_str(), s.outMin);
      s.outMax = prefs.getFloat(sensorKey(i, "omax").c_str(), s.outMax);
      s.outMode = sanitizeOutputMode(prefs.getInt(sensorKey(i, "omode").c_str(), s.outMode));
      s.onString = prefs.getString(sensorKey(i, "on").c_str(), s.onString);
      s.offString = prefs.getString(sensorKey(i, "off").c_str(), s.offString);

      if (s.oscAddress.length() == 0) s.oscAddress = defaultOscAddress(i);
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

  for (int i = 0; i < MAX_SENSORS; i++) {
    const SensorConfig& s = config.sensors[i];
    prefs.putBool(sensorKey(i, "en").c_str(), s.enabled);
    prefs.putInt(sensorKey(i, "type").c_str(), s.type);
    prefs.putInt(sensorKey(i, "pin").c_str(), s.pin);
    prefs.putBool(sensorKey(i, "inv").c_str(), s.invert);
    prefs.putBool(sensorKey(i, "ah").c_str(), s.activeHigh);
    prefs.putBool(sensorKey(i, "pu").c_str(), s.pullup);
    prefs.putString(sensorKey(i, "addr").c_str(), s.oscAddress);
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

static void handleRoot() {
  String html;
  html.reserve(12000);
  html += "<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>ESP32-POE OSC</title>";
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
  html += "<div class='title'>ESP32-POE OSC</div>";
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

  bool anyConflict = false;
  for (int i = 0; i < MAX_SENSORS; i++) {
    if (hasPinConflict(i)) {
      anyConflict = true;
      break;
    }
  }
  if (anyConflict) {
    html += "<p style='color:#b00;'>Pin conflict detected: two sensors share the same GPIO.</p>";
  }

  html += "<form method='POST' action='/'>";
  html += "<div class='card'>";
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
  html += "<hr>";

  for (int i = 0; i < MAX_SENSORS; i++) {
    const SensorConfig& s = config.sensors[i];
    String idx = String(i);
    html += "<fieldset>";
    html += "<legend>Sensor " + String(i + 1) + "</legend>";
    html += "Enable: <input name='s" + idx + "_en' type='checkbox' " + String(s.enabled ? "checked" : "") + "><br>";
    html += "Type: <select name='s" + idx + "_type' id='s" + idx + "_type'>";
    html += "<option value='0' " + String(s.type == SENSOR_ANALOG ? "selected" : "") + ">Analog (pot/slider)</option>";
    html += "<option value='1' " + String(s.type == SENSOR_BUTTON ? "selected" : "") + ">Button (momentary)</option>";
    html += "<option value='2' " + String(s.type == SENSOR_TOGGLE ? "selected" : "") + ">Button (toggle)</option>";
    html += "<option value='3' " + String(s.type == SENSOR_DIGITAL ? "selected" : "") + ">Digital (touch/motion)</option>";
    html += "</select><br>";
    html += "OSC address: <input name='s" + idx + "_addr' type='text' value='" + s.oscAddress + "'><br>";
    html += "Pin: <input name='s" + idx + "_pin' type='number' value='" + String(s.pin) + "'><br>";
    html += "<small>Reserved pins: 0,1,2,3,12,15,34</small><br>";

    html += "<div id='s" + idx + "_analog'>";
    html += "Invert: <input name='s" + idx + "_inv' type='checkbox' " + String(s.invert ? "checked" : "") + "><br>";
    html += "</div>";

    html += "<div id='s" + idx + "_digital'>";
    html += "Active high: <input name='s" + idx + "_ah' type='checkbox' " + String(s.activeHigh ? "checked" : "") + "><br>";
    html += "Use pullup: <input name='s" + idx + "_pu' type='checkbox' " + String(s.pullup ? "checked" : "") + "><br>";
    html += "</div>";

  html += "<div id='s" + idx + "_range'>";
  html += "Output min: <input name='s" + idx + "_omin' type='number' step='0.001' value='" + String(s.outMin, 3) + "'><br>";
  html += "Output max: <input name='s" + idx + "_omax' type='number' step='0.001' value='" + String(s.outMax, 3) + "'><br>";
  html += "</div>";
  html += "Output type: <select name='s" + idx + "_omode' id='s" + idx + "_omode'>";
  html += "<option value='0' " + String(s.outMode == OUT_INT ? "selected" : "") + ">Int</option>";
  html += "<option value='1' " + String(s.outMode == OUT_FLOAT ? "selected" : "") + ">Float</option>";
  html += "<option value='2' id='s" + idx + "_opt_string' " + String(s.outMode == OUT_STRING ? "selected" : "") + ">String</option>";
  html += "</select><br>";

    html += "<div id='s" + idx + "_string'>";
    html += "On string: <input name='s" + idx + "_on' type='text' value='" + s.onString + "'><br>";
    html += "Off string: <input name='s" + idx + "_off' type='text' value='" + s.offString + "'><br>";
    html += "</div>";

    html += "</fieldset>";
  }

  html += "<button type='submit' class='primary'>Save</button>";
  html += "<hr>";
  html += "<form method='POST' action='/reset' style='margin-top:8px;'>";
  html += "<button type='submit' id='factory_reset_btn' class='danger'>Factory Reset</button>";
  html += "</form>";
  html += "<script>";
  html += "function updateSensor(i){";
  html += "const t=document.getElementById('s'+i+'_type').value;";
  html += "const om=document.getElementById('s'+i+'_omode');";
  html += "const m=om.value;";
  html += "const a=document.getElementById('s'+i+'_analog');";
  html += "const d=document.getElementById('s'+i+'_digital');";
  html += "const s=document.getElementById('s'+i+'_string');";
  html += "const r=document.getElementById('s'+i+'_range');";
  html += "const opt=document.getElementById('s'+i+'_opt_string');";
  html += "a.style.display=(t==='0')?'block':'none';";
  html += "d.style.display=(t!=='0')?'block':'none';";
  html += "if(t==='0'){";
  html += "opt.disabled=true;";
  html += "if(om.value==='2'){om.value='1';}";
  html += "}else{";
  html += "opt.disabled=false;";
  html += "}";
  html += "s.style.display=(om.value==='2')?'block':'none';";
  html += "r.style.display=(om.value==='2')?'none':'block';";
  html += "}";
  html += "const dh=document.getElementById('dhcp_enabled');";
  html += "const sf=document.getElementById('static_fields');";
  html += "const nm=document.getElementById('net_mode');";
  html += "const wf=document.getElementById('wifi_fields');";
  html += "const helpBtn=document.getElementById('pin_help_btn');";
  html += "const helpModal=document.getElementById('pin_help_modal');";
  html += "const helpClose=document.getElementById('pin_help_close');";
  html += "const resetBtn=document.getElementById('factory_reset_btn');";
  html += "function toggleDhcp(){sf.style.display=dh.checked?'none':'block';}";
  html += "function toggleNet(){";
  html += "if(!nm){wf.style.display='block';return;}";
  html += "wf.style.display=(nm.value==='1')?'block':'none';";
  html += "}";
  html += "helpBtn.addEventListener('click',()=>{helpModal.style.display='block';});";
  html += "helpClose.addEventListener('click',()=>{helpModal.style.display='none';});";
  html += "helpModal.addEventListener('click',(e)=>{if(e.target===helpModal){helpModal.style.display='none';}});";
  html += "resetBtn.addEventListener('click',(e)=>{";
  html += "if(!confirm('Factory reset will erase all settings. Continue?')){e.preventDefault();}});";
  html += "dh.addEventListener('change',toggleDhcp);";
  html += "if(nm){nm.addEventListener('change',toggleNet);}";
  html += "toggleDhcp();";
  html += "toggleNet();";
  html += "for(let i=0;i<" + String(MAX_SENSORS) + ";i++){";
  html += "document.getElementById('s'+i+'_type').addEventListener('change',()=>updateSensor(i));";
  html += "document.getElementById('s'+i+'_omode').addEventListener('change',()=>updateSensor(i));";
  html += "updateSensor(i);";
  html += "}";
  html += "</script>";
  html += "</form></div></body></html>";
  server.send(200, "text/html", html);
}

static void handleSave() {
  String oldHostname = config.hostname;

  if (server.hasArg("hostname")) {
    config.hostname = sanitizeHostname(server.arg("hostname"));
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

  for (int i = 0; i < MAX_SENSORS; i++) {
    SensorConfig s = config.sensors[i];
    String idx = String(i);

    s.enabled = server.hasArg("s" + idx + "_en");

    if (server.hasArg("s" + idx + "_type")) {
      s.type = sanitizeSensorType(server.arg("s" + idx + "_type").toInt());
    }

    if (server.hasArg("s" + idx + "_pin")) {
      int pin = server.arg("s" + idx + "_pin").toInt();
      if (pin >= 0 && pin <= 39 && !isReservedPin(pin)) s.pin = pin;
    }

    if (server.hasArg("s" + idx + "_addr")) {
      s.oscAddress = normalizeOscAddress(server.arg("s" + idx + "_addr"));
    }

    s.invert = server.hasArg("s" + idx + "_inv");
    s.activeHigh = server.hasArg("s" + idx + "_ah");
    s.pullup = server.hasArg("s" + idx + "_pu");

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

    if (s.type == SENSOR_ANALOG && s.outMode == OUT_STRING) {
      s.outMode = OUT_FLOAT;
    }

    config.sensors[i] = s;
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
  doFactoryReset(true);
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

  server.on("/", HTTP_GET, handleRoot);
  server.on("/", HTTP_POST, handleSave);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/reset", HTTP_POST, handleReset);
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
    if (s.oscAddress.length() == 0) continue;
    if (hasPinConflict(i)) continue;

    float norm = 0.0f;
    bool active = false;

    if (s.type == SENSOR_ANALOG) {
      norm = readAnalogNormalized(i, s);
    } else {
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

    bool changed = false;

    if (s.outMode == OUT_STRING) {
      int8_t state = (norm >= 0.5f) ? 1 : 0;
      changed = (lastBool[i] == -1) || (state != lastBool[i]);
      if (changed) {
        const char* value = (state == 1) ? s.onString.c_str() : s.offString.c_str();
        if (value[0] != '\0') {
          sendOscString(s.oscAddress.c_str(), value);
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
        sendOscFloat(s.oscAddress.c_str(), outputValue);
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
        sendOscInt(s.oscAddress.c_str(), intValue);
        lastInt[i] = intValue;
      }
    }

    if (changed) {
      Serial.print("Sent ");
      Serial.print(s.oscAddress);
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
}
