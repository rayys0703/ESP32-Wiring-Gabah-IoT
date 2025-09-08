#include <WiFi.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>

// ================== Pin ==================
#define lampMerah   5
#define lampKuning 18
#define lampHijau  19
#define SDI_PIN    13
#define SCLK_PIN   14
#define LOAD_PIN   15

// ================== Identitas Panel ==================
const int panelId = 5; // perangkat kedua

// ================== AP sementara untuk konfigurasi ==================
const char* ap_ssid     = "GrainDryer_AP_5";
const char* ap_password = "12345678";

// ================== MQTT broker ==================
const char* mqttBroker = "broker.hivemq.com";
const int   mqttPort   = 1883;

// ================== Global ==================
Preferences prefs;
WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Pola seven segment (common anode)
const byte digitPatterns[11] = {
  B1000000, B1111001, B0100100, B0110000, B0011001,
  B0010010, B0000010, B1111000, B0000000, B0010000, B1111111
};

// Topic runtime
String mqttBase;         // iot/mitra{mitra}/dryer{dryer}
String telemetryTopic;   // {mqttBase}/{panelId}
String resetWifiTopic;   // {mqttBase}/resetwifi/{panelId}
String connectTopic;     // {mqttBase}/{panelId}/connect

// ================== Util ==================
String getMac() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// ================== HTTP Handlers (aktif hanya saat AP) ==================
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

    // Simpan preferensi
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.putInt("dryer_id", dryer_id);
    prefs.putInt("mitra_id", mitra_id);
    prefs.putString("location", location);
    prefs.putString("device_name", device_name);
    prefs.putString("mqtt_base", mqtt_base);

    // Build topics & simpan
    mqttBase       = mqtt_base;
    telemetryTopic = mqttBase + "/" + String(panelId);
    resetWifiTopic = mqttBase + "/resetwifi/" + String(panelId);
    connectTopic   = mqttBase + "/" + String(panelId) + "/connect";

    prefs.putString("telemetry_topic", telemetryTopic);
    prefs.putString("reset_topic", resetWifiTopic);
    prefs.putString("connect_topic", connectTopic);

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
    Serial.println("Invalid JSON or missing fields");
    server.send(400, "application/json", "{\"error\":\"Invalid JSON or missing fields\"}");
  }
}

void handleNotFound() {
  Serial.println("Received request for unknown endpoint: " + server.uri());
  server.send(404, "text/plain", "File not found");
}

// ================== Topics from prefs ==================
void setupTopicsFromPrefs() {
  // fallback aman bila kosong
  int dryer_id = prefs.getInt("dryer_id", 1);
  int mitra_id = prefs.getInt("mitra_id", 1);

  String baseFromPrefs = prefs.getString("mqtt_base", "");
  if (baseFromPrefs.length() == 0) {
    baseFromPrefs = "iot/mitra" + String(mitra_id) + "/dryer" + String(dryer_id);
  }

  mqttBase       = baseFromPrefs;
  telemetryTopic = prefs.getString("telemetry_topic", mqttBase + "/" + String(panelId));
  resetWifiTopic = prefs.getString("reset_topic",   mqttBase + "/resetwifi/" + String(panelId));
  connectTopic   = prefs.getString("connect_topic", mqttBase + "/" + String(panelId) + "/connect");

  Serial.println("setupTopicsFromPrefs()");
  Serial.println("mqttBase=" + mqttBase);
  Serial.println("telemetryTopic=" + telemetryTopic);
  Serial.println("connectTopic=" + connectTopic);
  Serial.println("resetWifiTopic=" + resetWifiTopic);
}

// ================== MQTT ==================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.println("MQTT MSG on " + t + ": " + msg);

  if (t == resetWifiTopic && msg == "reset") {
    Serial.println("Received resetWiFi command. Clearing prefs...");
    // hapus semua prefs, termasuk mqtt_base & topics
    prefs.clear();

    // Putus WiFi & matikan AP
    WiFi.disconnect(true, true);
    WiFi.softAPdisconnect(true);
    WiFi.enableAP(false);

    Serial.println("Restarting to AP-only mode...");
    delay(200);
    ESP.restart();
  }
}

bool publishConnectEvent() {
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
    Serial.println("WiFi not connected, skip MQTT connect");
    return;
  }
  String clientId = "GrainDryer-" + String(panelId) + "-" + String(random(0xffff), HEX);
  Serial.println("Connecting MQTT to " + String(mqttBroker) + ":" + String(mqttPort) + " as " + clientId);

  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("MQTT Connected.");
    mqttClient.subscribe(resetWifiTopic.c_str());
    Serial.println("Subscribed to " + resetWifiTopic);

    // publish CONNECT (dengan retry kecil)
    for (int i = 0; i < 3; i++) {
      if (publishConnectEvent()) break;
      Serial.printf("Retry publish CONNECT (%d)...\n", i+1);
      delay(300);
    }
  } else {
    Serial.print("MQTT Failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// ================== WiFi / AP Control ==================
void startAPMode() {
  Serial.println("Starting AP mode for configuration...");
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.softAP(ap_ssid, ap_password);
  Serial.print("AP Started at ");
  Serial.println(WiFi.softAPIP());

  // start web server ONLY in AP mode
  server.on("/", HTTP_GET, handleRoot);
  server.on("/config", HTTP_POST, handleConfig);
  server.onNotFound(handleNotFound);
  server.begin();
  Serial.println("HTTP server started at http://192.168.4.1");
}

void stopAPModeIfRunning() {
  // hentikan HTTP server lebih dulu supaya /config tidak terbuka di STA
  server.stop();
  WiFi.softAPdisconnect(true);
  WiFi.enableAP(false);
  Serial.println("AP interface disabled.");
}

bool connectWiFiSTA() {
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  if (ssid.length() == 0) {
    Serial.println("No saved SSID. Skipping STA connect.");
    return false;
  }

  Serial.printf("Connecting to SSID: %s ...\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());

    // Pastikan AP benar-benar mati
    stopAPModeIfRunning();
    WiFi.mode(WIFI_STA); // lock to STA

    // MQTT setup
    mqttClient.setServer(mqttBroker, mqttPort);
    mqttClient.setCallback(mqttCallback);
    mqttClient.setBufferSize(1024);
    mqttClient.setKeepAlive(30);

    connectMQTT();
    return true;
  } else {
    Serial.println("\nWiFi connect FAILED.");
    return false;
  }
}

// ================== Dummy data & tampilan ==================
float getBurningTemp() {
  // suhu acak 280..300 dengan 6 desimal
  return random(280000000, 300000001) / 1000000.0;
}

void displayToSevenSegment(float burningTemp) {
  int tempInt = (burningTemp >= 280 && burningTemp <= 300) ? round(burningTemp) : 0;

  byte d1 = (tempInt >= 100) ? (tempInt / 100) : 10;
  byte d2 = (tempInt >= 10)  ? ((tempInt / 10) % 10) : 10;
  byte d3 = (tempInt >= 0)   ? (tempInt % 10) : 10;

  byte displayData[7] = {
    digitPatterns[d1], digitPatterns[d2], digitPatterns[d3],
    digitPatterns[10], digitPatterns[10], digitPatterns[10], digitPatterns[10]
  };

  for (int i = 6; i >= 0; i--) {
    byte dataToSend = displayData[i] & B01111111;
    dataToSend |= B10000000; // DP off
    shiftOut(SDI_PIN, SCLK_PIN, MSBFIRST, dataToSend);
  }
  digitalWrite(LOAD_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LOAD_PIN, LOW);
}

void publishSensorData() {
  float burningTemp = getBurningTemp();
  bool  stirrerStatus = true;

  Serial.print("\nSuhu Pembakaran: ");
  Serial.print(burningTemp, 2);
  Serial.println(" C");
  Serial.println("Stirrer Status: true");
  Serial.println("-----------");

  DynamicJsonDocument doc(256);
  doc["panel_id"] = panelId;
  doc["burning_temperature"] = burningTemp;
  doc["stirrer_status"] = stirrerStatus;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);
  if (mqttClient.connected()) {
    if (mqttClient.publish(telemetryTopic.c_str(), (uint8_t*)buffer, n)) {
      Serial.println("Published to " + telemetryTopic + ": " + String(buffer));
    } else {
      Serial.println("Failed to publish to " + telemetryTopic);
    }
  } else {
    Serial.println("MQTT not connected, skip publish");
  }
}

// ================== Setup / Loop ==================
void setup() {
  Serial.begin(115200);

  pinMode(lampMerah, OUTPUT);
  pinMode(lampKuning, OUTPUT);
  pinMode(lampHijau, OUTPUT);
  pinMode(SDI_PIN, OUTPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(LOAD_PIN, OUTPUT);
  digitalWrite(lampMerah, HIGH);
  digitalWrite(lampKuning, HIGH);
  digitalWrite(lampHijau, HIGH);
  digitalWrite(LOAD_PIN, LOW);

  prefs.begin("wifi", false);
  setupTopicsFromPrefs();

  randomSeed(analogRead(0));

  // === Urutan baru ===
  // 1) Coba STA kalau sudah ada SSID
  bool staOk = connectWiFiSTA();

  // 2) Jika gagal atau belum ada SSID, baru hidupkan AP (server config)
  if (!staOk) {
    startAPMode();
  }
}

void loop() {
  // Hanya handle server saat AP mode aktif (kita cek dengan ip AP)
  if (WiFi.getMode() == WIFI_AP || WiFi.softAPIP() == IPAddress(192,168,4,1)) {
    server.handleClient();
  }

  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    Serial.println("MQTT disconnected, trying reconnect...");
    connectMQTT();
  }
  mqttClient.loop();

  // (opsional) Tampilkan ke seven segment, contoh setiap 1s
  static unsigned long lastDisplay = 0;
  unsigned long now = millis();
  if (now - lastDisplay >= 1000) {
    float t = getBurningTemp();
    displayToSevenSegment(t);
    lastDisplay = now;
  }

  // Publish tiap 5s
  static unsigned long lastPublish = 0;
  if (mqttClient.connected() && now - lastPublish >= 5000) {
    lastPublish = now;
    publishSensorData();
  }

  delay(50);
}
