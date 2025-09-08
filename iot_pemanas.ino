#include <Adafruit_MAX31865.h>
#include <SPI.h>
#include <math.h>
#include <WiFi.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>

// ================== Hardware Pins & Const ==================

// SCT013 current sensor
const int ACPin = 34;              // ADC1_6 (aman)
#define ACTectionRange 100         // Range sensor SCT-013 (100A model)
#define VREF 3.3                   // ESP32 ADC reference ~3.3V
const float currentThreshold = 0.4;// Ambang arus (A) untuk stirrer_status
const int samples = 5;             // Sampel untuk rata-rata

// MAX31865 (RTD PT100)
#define RTD_CS    21
#define RTD_SCLK  18
#define RTD_MISO  19
#define RTD_MOSI  23
#define RREF      430.0
#define RNOMINAL  100.0

// Seven Segment via 74HC595
#define SDI_PIN   13
#define SCLK_PIN  14
#define LOAD_PIN  15
#define WIFI_LED_PIN 4

// AP sementara untuk konfigurasi
const char* ap_ssid     = "GrainDryer_AP_5";
const char* ap_password = "12345678";

// Panel ID (tetap 5 untuk unit ini)
const int panelId = 5;

// MQTT broker (tetap sesuai proyek Anda)
const char* mqttBroker = "broker.hivemq.com";
const int   mqttPort   = 1883;

// ================== Globals ==================

Adafruit_MAX31865 rtd(RTD_CS); // pakai VSPI global pin di atas

Preferences prefs;
WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Global sensor cache
float globalBurningTemp   = 0;
bool  globalStirrerStatus = false;

// Topik runtime (diisi dari Preferences)
String mqttBase;           // iot/mitra{mitra}/dryer{dryer}
String telemetryTopic;     // {mqttBase}/{panelId}
String resetWifiTopic;     // {mqttBase}/resetwifi/{panelId}
String connectTopic;       // {mqttBase}/{panelId}/connect

// Seven-seg common anode patterns (7 bit + DP)
const byte digitPatterns[11] = {
  B1000000, B1111001, B0100100, B0110000, B0011001,
  B0010010, B0000010, B1111000, B0000000, B0010000, B1111111
};

unsigned long wifiLedPreviousMillis = 0;
const long wifiLedInterval = 500; // blink 500ms
bool wifiLedState = LOW;

// ================== Utils ==================

String getMac() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

void startApForConfig() {
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("AP Started at ");
  Serial.println(WiFi.softAPIP());
}

void stopApIfRunning() {
  // Jika AP aktif, putuskan & matikan
  bool stopped = WiFi.softAPdisconnect(true);
  if (stopped) {
    Serial.println("AP stopped (softAPdisconnect).");
  }
  // Ubah mode supaya tidak broadcast lagi
  if (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) {
    WiFi.mode(WIFI_STA);
    Serial.println("Switched WiFi mode to WIFI_STA.");
  }
}

// ================== HTTP Handlers ==================

void handleRoot() {
  Serial.println("Received GET /");
  server.send(200, "text/html",
              "<!DOCTYPE html><html><body><h2>ESP32 GrainDryer</h2><p>POST /config untuk WiFi</p></body></html>");
}

void handleConfig() {
  if (server.method() != HTTP_POST) {
    Serial.println("Received non-POST /config");
    server.send(405, "text/plain", "Method Not Allowed");
    return;
  }

  Serial.println("Received POST /config");
  String body = server.arg("plain");
  Serial.println("Body: " + body);

  DynamicJsonDocument doc(768);
  DeserializationError err = deserializeJson(doc, body);

  // Samakan kontrak dengan perangkat pertama:
  // Wajib: ssid, pass, dryer_id, mitra_id, location, device_name, mqtt_topic_base
  if (!err &&
      doc.containsKey("ssid") &&
      doc.containsKey("pass") &&
      doc.containsKey("dryer_id") &&
      doc.containsKey("mitra_id") &&
      doc.containsKey("location") &&
      doc.containsKey("device_name") &&
      doc.containsKey("mqtt_topic_base")) {

    String ssid        = doc["ssid"].as<String>();
    String pass        = doc["pass"].as<String>();
    int    dryer_id    = doc["dryer_id"].as<int>();
    int    mitra_id    = doc["mitra_id"].as<int>();
    String location    = doc["location"].as<String>();
    String device_name = doc["device_name"].as<String>();
    String mqtt_base   = doc["mqtt_topic_base"].as<String>();

    // Simpan konfigurasi & flag configured
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.putInt("dryer_id", dryer_id);
    prefs.putInt("mitra_id", mitra_id);
    prefs.putString("location", location);
    prefs.putString("device_name", device_name);
    prefs.putString("mqtt_base", mqtt_base);
    prefs.putBool("configured", true);

    // Build topics & save
    mqttBase       = mqtt_base;
    telemetryTopic = mqttBase + "/" + String(panelId);
    resetWifiTopic = mqttBase + "/resetwifi/" + String(panelId);
    connectTopic   = mqttBase + "/" + String(panelId) + "/connect";

    prefs.putString("telemetry_topic", telemetryTopic);
    prefs.putString("reset_topic",     resetWifiTopic);
    prefs.putString("connect_topic",   connectTopic);

    Serial.println("Saved config to NVS.");
    Serial.println("ssid=" + ssid + ", dryer_id=" + String(dryer_id) + ", mitra_id=" + String(mitra_id));
    Serial.println("mqtt_base=" + mqttBase);
    Serial.println("telemetryTopic=" + telemetryTopic);
    Serial.println("connectTopic=" + connectTopic);
    Serial.println("resetWifiTopic=" + resetWifiTopic);

    server.send(200, "application/json", "{\"status\":\"saved\",\"message\":\"Rebooting...\"}");
    delay(500);
    ESP.restart();
  } else {
    Serial.println("Invalid JSON or missing required fields");
    server.send(400, "application/json", "{\"error\":\"Invalid JSON or missing required fields\"}");
  }
}

void handleNotFound() {
  Serial.println("Received request for unknown endpoint: " + server.uri());
  server.send(404, "text/plain", "File not found");
}

// ================== MQTT ==================

void callback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg;
  for (unsigned int i=0; i<length; i++) msg += (char)payload[i];

  Serial.println("MQTT MSG on " + t + ": " + msg);

  if (t == resetWifiTopic && msg == "reset") {
    Serial.println("Received resetWiFi command. Clearing prefs & topics, shutting down WiFi & MQTT...");

    // Kosongkan runtime topics agar aman
    mqttBase = "";
    telemetryTopic = "";
    resetWifiTopic = "";
    connectTopic = "";

    // Hentikan MQTT jika sedang connect
    if (mqttClient.connected()) {
      mqttClient.disconnect();
    }

    // Bersihkan NVS seluruhnya (termasuk 'configured' & topik)
    prefs.clear();

    // Putus WiFi (hapus konfigurasi STA, matikan AP)
    WiFi.softAPdisconnect(true);
    WiFi.disconnect(true, true); // wifioff=true, erase ap config=true
    WiFi.mode(WIFI_OFF);

    Serial.println("Restarting to pure AP config mode...");
    delay(250);
    ESP.restart();
  }
}

void setupTopicsFromPrefs() {
  Serial.println("setupTopicsFromPrefs()");

  bool configured = prefs.getBool("configured", false);
  if (!configured) {
    // Setelah resetwifi, biarkan kosong — tidak fallback topik default
    mqttBase = "";
    telemetryTopic = "";
    resetWifiTopic = "";
    connectTopic = "";
    Serial.println("NO CONFIG in NVS; topics not set");
    return;
  }

  // Sudah configured → ambil dari NVS
  int dryer_id = prefs.getInt("dryer_id", 1);
  int mitra_id = prefs.getInt("mitra_id", 1);

  String baseFromPrefs = prefs.getString("mqtt_base", "");
  if (baseFromPrefs.length() == 0) {
    // fallback aman jika user pernah configured tapi mqtt_base tidak tersimpan
    baseFromPrefs = "iot/mitra" + String(mitra_id) + "/dryer" + String(dryer_id);
  }

  mqttBase       = baseFromPrefs;
  telemetryTopic = prefs.getString("telemetry_topic", mqttBase + "/" + String(panelId));
  resetWifiTopic = prefs.getString("reset_topic",     mqttBase + "/resetwifi/" + String(panelId));
  connectTopic   = prefs.getString("connect_topic",   mqttBase + "/" + String(panelId) + "/connect");

  Serial.println("mqttBase=" + mqttBase);
  Serial.println("telemetryTopic=" + telemetryTopic);
  Serial.println("connectTopic=" + connectTopic);
  Serial.println("resetWifiTopic=" + resetWifiTopic);
}

bool publishConnectEvent() {
  if (!prefs.getBool("configured", false)) {
    Serial.println("Device not configured; skip CONNECT publish");
    return false;
  }

  DynamicJsonDocument doc(768);
  doc["event"]            = "connect";
  doc["panel_id"]         = panelId;
  doc["dryer_id"]         = prefs.getInt("dryer_id", 1);
  doc["mitra_id"]         = prefs.getInt("mitra_id", 1);
  doc["device_name"]      = prefs.getString("device_name", "GrainDryerPanel");
  doc["location"]         = prefs.getString("location", "");
  doc["ssid"]             = prefs.getString("ssid", "");
  doc["ip"]               = WiFi.localIP().toString();
  doc["mac"]              = getMac();
  doc["mqtt_topic_base"]  = mqttBase;
  doc["telemetry_topic"]  = telemetryTopic;
  doc["firmware"]         = "gd-esp32-1.0.0";
  doc["timestamp"]        = (uint32_t)(millis()/1000);

  size_t n = measureJson(doc);
  std::unique_ptr<char[]> buf(new char[n + 1]);
  serializeJson(doc, buf.get(), n + 1);

  Serial.println("CONNECT payload length = " + String(n));
  Serial.println("CONNECT topic: " + connectTopic);

  if (connectTopic.length() == 0) {
    Serial.println("connectTopic empty! Rebuilding from prefs...");
    setupTopicsFromPrefs();
  }
  if (connectTopic.length() == 0) {
    Serial.println("connectTopic still empty. Skip publish.");
    return false;
  }

  bool ok = mqttClient.publish(connectTopic.c_str(), (uint8_t*)buf.get(), n, true);
  if (ok) {
    Serial.println(String("CONNECT event published: ") + String(buf.get()));
  } else {
    Serial.println("Failed to publish CONNECT event");
  }
  return ok;
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skipping MQTT");
    return;
  }
  if (!prefs.getBool("configured", false)) {
    Serial.println("Device not configured; skip MQTT subscribe/publish");
    return;
  }

  String clientId = "GrainDryer-" + String(panelId) + "-" + String(random(0xffff), HEX);
  Serial.println("Connecting MQTT to " + String(mqttBroker) + ":" + String(mqttPort) + " as " + clientId);

  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("MQTT Connected to " + String(mqttBroker));

    if (resetWifiTopic.length() > 0) {
      mqttClient.subscribe(resetWifiTopic.c_str());
      Serial.println("Subscribed to " + resetWifiTopic);
    } else {
      Serial.println("resetWifiTopic empty; skip subscribe.");
    }

    // publish CONNECT dengan retry ringan
    for (int i=0; i<3; i++) {
      if (publishConnectEvent()) break;
      Serial.print("Retry publish CONNECT (");
      Serial.print(i+1);
      Serial.println(")...");
      delay(300);
    }
  } else {
    Serial.print("MQTT Failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// ================== Setup ==================

void setup() {
  Serial.begin(115200);

  // IO init
  pinMode(ACPin, INPUT);
  pinMode(SDI_PIN, OUTPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(LOAD_PIN, OUTPUT);
  digitalWrite(LOAD_PIN, LOW);
  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW);

  // MAX31865 init
  SPI.begin(RTD_SCLK, RTD_MISO, RTD_MOSI);
  if (!rtd.begin(MAX31865_2WIRE)) {
    Serial.println("Init MAX31865 gagal. Cek VIN (bukan 3V3 modul), wiring, & jumper 2-wire.");
    while (1) delay(10);
  }
  rtd.enable50Hz(true);
  Serial.println("MAX31865 siap (VSPI, CS=GPIO21).");

  // Prefs & topics
  prefs.begin("wifi", false);
  setupTopicsFromPrefs();

  // AP always on for config (akan dimatikan otomatis jika STA connect)
  startApForConfig();

  server.on("/", HTTP_GET, handleRoot);
  server.on("/config", HTTP_POST, handleConfig);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started at http://192.168.4.1");

  // Hanya konek STA jika configured & ada SSID
  bool configured = prefs.getBool("configured", false);
  String ssid = prefs.getString("ssid", "");
  if (configured && ssid.length() > 0) {
    // WiFi STA connect sambil tetap jaga AP sementara
    String pass = prefs.getString("pass", "");
    Serial.printf("Connecting to %s...\n", ssid.c_str());

    WiFi.mode(WIFI_AP_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
      delay(500);
      Serial.print(".");
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());

      // >>> Matikan AP karena STA sudah tersambung <<<
      stopApIfRunning();

      mqttClient.setServer(mqttBroker, mqttPort);
      mqttClient.setCallback(callback);
      mqttClient.setBufferSize(1024);
      mqttClient.setKeepAlive(30);

      connectMQTT();
    } else {
      Serial.println("\nWiFi Failed; AP continues for config");
    }
  }
}

// ================== Sensors & Telemetry ==================

float readACCurrentValue() {
  float ACCurrentValue = 0;
  float peakVoltage = 0;
  float voltageVirtualValue = 0;
  int offset = 0;

  // Offset/noise baseline
  for (int i = 0; i < samples; i++) {
    offset += analogRead(ACPin);
    delay(1);
  }
  offset = offset / samples;

  for (int i = 0; i < samples; i++) {
    peakVoltage += (analogRead(ACPin) - offset);
    delay(1);
  }
  peakVoltage = abs(peakVoltage / samples);
  voltageVirtualValue = peakVoltage * 0.707;

  // ke Volt
  voltageVirtualValue = (voltageVirtualValue / 4095.0f) * VREF;

  // Skala kasar → arus (kalibrasi sesuai kit Anda)
  ACCurrentValue = (voltageVirtualValue / (VREF / 2.0f)) * ACTectionRange;

  return ACCurrentValue;
}

float getBurningTemp() {
  rtd.clearFault();

  rtd.enableBias(true);
  delay(10);
  rtd.autoConvert(false);
  delay(65);

  uint16_t raw = rtd.readRTD();
  (void)raw; // tidak dipakai langsung di sini

  float tC = rtd.temperature(RNOMINAL, RREF);
  rtd.enableBias(false);

  uint8_t f = rtd.readFault();
  if (f) {
    Serial.print("Fault 0x"); Serial.println(f, HEX);
    return 0;
  }

  if (isnan(tC)) return 0;
  return tC;
}

void publishSensorData() {
  float burningTemp   = globalBurningTemp;
  bool  stirrerStatus = globalStirrerStatus;

  // Serial log
  Serial.print("\nSuhu Pembakaran: ");
  Serial.print(burningTemp, 2);
  Serial.println(" C");
  Serial.print("Status Pengaduk: ");
  Serial.println(stirrerStatus ? "True" : "False");
  Serial.println("-----------");

  if (burningTemp != 0 && mqttClient.connected()) {
    DynamicJsonDocument doc(256);
    doc["panel_id"]            = panelId;
    doc["burning_temperature"] = burningTemp;
    doc["stirrer_status"]      = stirrerStatus;

    char buffer[256];
    size_t n = serializeJson(doc, buffer);
    if (telemetryTopic.length() > 0 && mqttClient.publish(telemetryTopic.c_str(), (uint8_t*)buffer, n)) {
      Serial.println("Published to " + telemetryTopic + ": " + String(buffer));
    } else {
      Serial.println("Failed to publish telemetry");
    }
  } else {
    Serial.println("Invalid sensor data or MQTT not connected, skipping publish");
  }
}

// Extract 2 integer digits + 1 decimal for seven-seg
void extractDigits(float value, byte* digits) {
  if (value < 0) value = 0;
  if (value > 99.9) value = 99.9;
  int intPart = (int)value;
  int decPart = (int)((value - intPart) * 10);
  digits[0] = (intPart / 10) % 10;
  digits[1] = intPart % 10;
  digits[2] = decPart;
}

// Display burning temp on 3-digit module
void displayOnSevenSegment() {
  byte digits[3];
  extractDigits(globalBurningTemp, digits);

  byte displayData[3];
  displayData[0] = digitPatterns[digits[0]]; // puluhan
  displayData[1] = digitPatterns[digits[1]]; // satuan (DP on)
  displayData[2] = digitPatterns[digits[2]]; // desimal

  for (int i = 2; i >= 0; i--) {
    byte dataToSend = displayData[i] & B01111111;
    if (i == 1) dataToSend |= B00000000;  // DP on
    else        dataToSend |= B10000000;  // DP off
    shiftOut(SDI_PIN, SCLK_PIN, MSBFIRST, dataToSend);
  }

  digitalWrite(LOAD_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LOAD_PIN, LOW);
}

void updateWifiLed() {
  unsigned long now = millis();

  if (WiFi.status() != WL_CONNECTED) {
    if (now - wifiLedPreviousMillis >= wifiLedInterval) {
      wifiLedState = !wifiLedState;
      digitalWrite(WIFI_LED_PIN, wifiLedState);
      wifiLedPreviousMillis = now;
    }
  } else {
    if (wifiLedState != HIGH) {
      wifiLedState = HIGH;
      digitalWrite(WIFI_LED_PIN, HIGH);
    }
  }
}

// ================== Main Loop ==================

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  server.handleClient();

  // Self-heal: jika STA sudah connect tapi mode masih AP_STA, matikan AP
  if (WiFi.status() == WL_CONNECTED && (WiFi.getMode() == WIFI_AP_STA || WiFi.getMode() == WIFI_AP)) {
    stopApIfRunning();
  }

  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    Serial.println("MQTT disconnected, trying reconnect...");
    connectMQTT();
  }
  mqttClient.loop();

  // Update sensor tiap 1 detik
  if (now - lastUpdate >= 1000) {
    float current = readACCurrentValue();
    globalStirrerStatus = (current > currentThreshold);
    globalBurningTemp   = getBurningTemp();

    Serial.print("\nArus: ");
    Serial.print(current, 2);
    Serial.println(" A");
    Serial.print("Suhu Pembakaran: ");
    Serial.print(globalBurningTemp, 2);
    Serial.println(" C");
    Serial.print("Status Pengaduk: ");
    Serial.println(globalStirrerStatus ? "True" : "False");
    Serial.println("-----------");

    displayOnSevenSegment();
    lastUpdate = now;
  }

  updateWifiLed();

  // Publish telemetry tiap 5 detik
  static unsigned long lastPublish = 0;
  if (mqttClient.connected() && now - lastPublish >= 5000) {
    lastPublish = now;
    publishSensorData();
  }
}
