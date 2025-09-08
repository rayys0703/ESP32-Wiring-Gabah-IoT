#include <DHT.h>
#include <math.h>
#include <WiFi.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>

#define lampMerah 5
#define lampKuning 18
#define lampHijau 19
#define DHTPIN 2
#define DHTTYPE DHT22
#define ANALOG_PIN_GRAIN_TEMP 36
#define ANALOG_PIN_MOIS 39
#define SDI_PIN 13
#define SCLK_PIN 14
#define LOAD_PIN 15
#define WIFI_LED_PIN 4

// ====== Konfigurasi panel/perangkat ======
const int panelId = 1; // ID panel ini (tetap 1 sesuai setup Anda)

// ====== AP sementara untuk konfigurasi ======
const char* ap_ssid = "GrainDryer_AP_1";
const char* ap_password = "12345678";

// ====== MQTT broker (pastikan benar) ======
const char* mqttBroker = "broker.hivemq.com";
const int   mqttPort   = 1883;

Preferences prefs;
WebServer server(80);
WiFiClient espClient;
PubSubClient mqttClient(espClient);
DHT dht(DHTPIN, DHTTYPE);

// Termistor & moisture (placeholder)
const float resistorValue = 10000.0;
const int dryValue = 2910;
const int wetValue = 1465;

// Global sensor cache
float globalMoisture = 0;
float globalGrainTemp = 0;
float globalRoomTemp = 0;

// Topic runtime (diisi dari Preferences)
String mqttBase;           // iot/mitra{mitra}/dryer{dryer}
String telemetryTopic;     // {mqttBase}/{panelId}
String resetWifiTopic;     // {mqttBase}/resetwifi/{panelId}
String connectTopic;       // {mqttBase}/{panelId}/connect

// ====== Seven segment patterns (common anode) ======
const byte digitPatterns[11] = {
  B1000000, B1111001, B0100100, B0110000, B0011001,
  B0010010, B0000010, B1111000, B0000000, B0010000, B1111111
};

unsigned long wifiLedPreviousMillis = 0;
const long wifiLedInterval = 500;
bool wifiLedState = LOW;

// ====== Utils ======
String getMac() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

// ====== HTTP Handlers ======
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

    String ssid          = doc["ssid"].as<String>();
    String pass          = doc["pass"].as<String>();
    int    dryer_id      = doc["dryer_id"].as<int>();
    int    mitra_id      = doc["mitra_id"].as<int>();
    String location      = doc["location"].as<String>();
    String device_name   = doc["device_name"].as<String>();
    String mqtt_base     = doc["mqtt_topic_base"].as<String>();

    // Simpan seluruh konfigurasi + flag configured=true
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.putInt("dryer_id", dryer_id);
    prefs.putInt("mitra_id", mitra_id);
    prefs.putString("location", location);
    prefs.putString("device_name", device_name);
    prefs.putString("mqtt_base", mqtt_base);
    prefs.putBool("configured", true);

    // Rebuild topics & save
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

// ====== MQTT callback ======
void callback(char* topic, byte* payload, unsigned int length) {
  String t = String(topic);
  String msg;
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.println("MQTT MSG on " + t + ": " + msg);

  if (t == resetWifiTopic && msg == "reset") {
    Serial.println("Received resetWiFi command. Clearing prefs...");
    // Hapus seluruh konfigurasi yang tersimpan, termasuk flag 'configured' dan semua topik
    prefs.clear();
    WiFi.disconnect(true);
    Serial.println("Restarting to AP mode...");
    delay(200);
    ESP.restart();
  }
}

// ====== Setup & connect WiFi/MQTT ======
void setupTopicsFromPrefs() {
  Serial.println("setupTopicsFromPrefs()");

  bool configured = prefs.getBool("configured", false);

  if (!configured) {
    // Tidak membangun fallback apa pun; biarkan kosong setelah resetwifi
    mqttBase = "";
    telemetryTopic = "";
    resetWifiTopic = "";
    connectTopic = "";
    Serial.println("NO CONFIG in NVS; topics not set");
    return;
  }

  // Jika sudah configured, ambil dari NVS
  int dryer_id = prefs.getInt("dryer_id", 1);
  int mitra_id = prefs.getInt("mitra_id", 1);

  String baseFromPrefs = prefs.getString("mqtt_base", "");
  if (baseFromPrefs.length() == 0) {
    // Jika user pernah configured tapi tidak menyimpan mqtt_base (kasus langka), barulah fallback
    baseFromPrefs = "iot/mitra" + String(mitra_id) + "/dryer" + String(dryer_id);
  }

  mqttBase       = baseFromPrefs;
  telemetryTopic = prefs.getString("telemetry_topic", mqttBase + "/" + String(panelId));
  resetWifiTopic = prefs.getString("reset_topic",    mqttBase + "/resetwifi/" + String(panelId));
  connectTopic   = prefs.getString("connect_topic",  mqttBase + "/" + String(panelId) + "/connect");

  Serial.println("mqttBase=" + mqttBase);
  Serial.println("telemetryTopic=" + telemetryTopic);
  Serial.println("connectTopic=" + connectTopic);
  Serial.println("resetWifiTopic=" + resetWifiTopic);
}

void connectMQTT() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skip MQTT connect");
    return;
  }

  // Jika belum configured, jangan subscribe/publish
  if (!prefs.getBool("configured", false)) {
    Serial.println("Device not configured; skip MQTT subscribe/publish");
    return;
  }

  String clientId = "GrainDryer-" + String(panelId) + "-" + String(random(0xffff), HEX);
  Serial.println("Connecting MQTT to " + String(mqttBroker) + ":" + String(mqttPort) + " as " + clientId);

  if (mqttClient.connect(clientId.c_str())) {
    Serial.println("MQTT Connected.");

    if (resetWifiTopic.length() > 0) {
      mqttClient.subscribe(resetWifiTopic.c_str());
      Serial.println("Subscribed to " + resetWifiTopic);
    } else {
      Serial.println("resetWifiTopic empty; skip subscribe.");
    }

    // Publish connect event once connected (with small retries)
    for (int i = 0; i < 3; i++) {
      if (publishConnectEvent()) break;
      Serial.print("Retry publish CONNECT (");
      Serial.print(i + 1);
      Serial.println(")...");
      delay(300);
    }
  } else {
    Serial.print("MQTT Failed, rc=");
    Serial.println(mqttClient.state());
  }
}

bool publishConnectEvent() {
  // Safety: jika belum terkonfigurasi jangan publish apa pun
  if (!prefs.getBool("configured", false)) {
    Serial.println("Device not configured; skip CONNECT publish");
    return false;
  }

  // Build connect payload
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

void connectWiFi() {
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  Serial.printf("Connecting to %s...\n", ssid.c_str());

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected: " + WiFi.localIP().toString());

    mqttClient.setServer(mqttBroker, mqttPort);
    mqttClient.setCallback(callback);

    // Perbesar buffer MQTT agar payload JSON besar bisa terkirim
    mqttClient.setBufferSize(1024);

    // KeepAlive supaya broker tidak memutuskan cepat
    mqttClient.setKeepAlive(30);

    connectMQTT();
  } else {
    Serial.println("\nWiFi Failed; Starting AP for config");
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    WiFi.softAP(ap_ssid, ap_password);
    Serial.print("AP Started at ");
    Serial.println(WiFi.softAPIP());

    // Mulai HTTP server hanya saat AP aktif
    server.on("/", HTTP_GET, handleRoot);
    server.on("/config", HTTP_POST, handleConfig);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started at http://192.168.4.1");
  }
}

void setup() {
  Serial.begin(115200);
  dht.begin();

  pinMode(ANALOG_PIN_GRAIN_TEMP, INPUT);
  pinMode(ANALOG_PIN_MOIS, INPUT);
  pinMode(lampMerah, OUTPUT);
  pinMode(lampKuning, OUTPUT);
  pinMode(lampHijau, OUTPUT);
  digitalWrite(lampMerah, HIGH);
  digitalWrite(lampKuning, HIGH);
  digitalWrite(lampHijau, HIGH);

  pinMode(SDI_PIN, OUTPUT);
  pinMode(SCLK_PIN, OUTPUT);
  pinMode(LOAD_PIN, OUTPUT);
  digitalWrite(LOAD_PIN, LOW);

  pinMode(WIFI_LED_PIN, OUTPUT);
  digitalWrite(WIFI_LED_PIN, LOW);

  prefs.begin("wifi", false);
  setupTopicsFromPrefs();

  // === Perilaku baru: AP hanya aktif bila belum configured atau koneksi STA gagal ===
  bool configured = prefs.getBool("configured", false);
  String ssid = prefs.getString("ssid", "");

  if (configured && ssid.length() > 0) {
    // Coba konek STA; jika gagal, connectWiFi() akan mengaktifkan AP & server
    connectWiFi();
  } else {
    // Belum terkonfigurasi → langsung aktifkan AP untuk provisioning
    WiFi.mode(WIFI_AP);
    WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
    WiFi.softAP(ap_ssid, ap_password);
    Serial.print("AP Started at ");
    Serial.println(WiFi.softAPIP());

    server.on("/", HTTP_GET, handleRoot);
    server.on("/config", HTTP_POST, handleConfig);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("HTTP server started at http://192.168.4.1");
  }
}

// ====== Sensor & display ======
float getMoisture() {
  const int numSamples = 10;
  long sum = 0;
  for (int i=0;i<numSamples;i++){ sum += analogRead(ANALOG_PIN_MOIS); delay(10); }
  float sensorValue = (float)sum / numSamples;
  if (sensorValue == 0 || sensorValue > 4095) return 0;

  float moisture = (sensorValue - dryValue) * 100.0 / (wetValue - dryValue);
  if (moisture < 0 || moisture > 100) return 0;
  return moisture;
}

float getGrainTemp() {
  int sensorValue = analogRead(ANALOG_PIN_GRAIN_TEMP);
  if (sensorValue == 0) return 0;

  const float R1 = 10000.0;
  const float Beta = 3950.0;
  const float To = 298.15;
  const float Ro = 10000.0;
  const float adcMax = 4095.0;
  const float Vs = 3.3;

  float Vout = sensorValue * Vs / adcMax;
  float Rt = R1 * Vout / (Vs - Vout);
  float T = 1.0 / (1.0 / To + log(Rt / Ro) / Beta);
  float Tc = T - 273.15;

  if (Tc < 0 || isnan(Tc)) return 0;
  return Tc;
}

float getRoomTemp() {
  float t = dht.readTemperature();
  if (isnan(t)) return 0;
  return t;
}

void publishSensorData() {
  float moisture  = globalMoisture;
  float grainTemp = globalGrainTemp;
  float roomTemp  = globalRoomTemp;

  // Tampilkan di Serial Monitor
  Serial.print("\nKadar Air: ");
  Serial.print(moisture, 2);
  Serial.println(" %");
  Serial.print("Suhu Gabah: ");
  Serial.print(grainTemp, 2);
  Serial.println(" C");
  Serial.print("Suhu Ruangan: ");
  Serial.print(roomTemp, 2);
  Serial.println(" C");
  Serial.println("-----------");

  // Kontrol lampu (tetap sama)
  if (moisture >= 0 && moisture <= 100) {
    if (moisture < 12) {
      digitalWrite(lampMerah, LOW);
      digitalWrite(lampKuning, HIGH);
      digitalWrite(lampHijau, HIGH);
    } else if (moisture > 16) {
      digitalWrite(lampMerah, HIGH);
      digitalWrite(lampKuning, LOW);
      digitalWrite(lampHijau, HIGH);
    } else {
      digitalWrite(lampMerah, HIGH);
      digitalWrite(lampKuning, HIGH);
      digitalWrite(lampHijau, LOW);
    }
  } else {
    digitalWrite(lampMerah, HIGH);
    digitalWrite(lampKuning, HIGH);
    digitalWrite(lampHijau, HIGH);
  }

  if (moisture != 0 && grainTemp != 0 && roomTemp != 0 && mqttClient.connected()) {
    DynamicJsonDocument doc(256);
    doc["panel_id"] = panelId;
    doc["grain_temperature"] = grainTemp;
    doc["grain_moisture"]    = moisture;
    doc["room_temperature"]  = roomTemp;

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

void extractDigits(float value, byte* digits) {
  if (value < 0) value = 0;
  if (value > 99.9) value = 99.9;
  int intPart = (int)value;
  int decPart = (int)((value - intPart) * 10);
  digits[0] = (intPart / 10) % 10;
  digits[1] = intPart % 10;
  digits[2] = decPart;
}

void displayOnSevenSegment() {
  byte digits[3][3];
  extractDigits(globalMoisture,  digits[0]);
  extractDigits(globalGrainTemp, digits[1]);
  extractDigits(globalRoomTemp,  digits[2]);

  byte displayData[9];
  for (int m=0; m<3; m++) {
    displayData[m*3+0] = digitPatterns[digits[m][0]];
    displayData[m*3+1] = digitPatterns[digits[m][1]];
    displayData[m*3+2] = digitPatterns[digits[m][2]];
  }

  for (int i=8; i>=0; i--) {
    byte dataToSend = displayData[i] & B01111111;
    if (i % 3 == 1) dataToSend |= B00000000; // DP on
    else            dataToSend |= B10000000; // DP off
    shiftOut(SDI_PIN, SCLK_PIN, MSBFIRST, dataToSend);
  }

  digitalWrite(LOAD_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(LOAD_PIN, LOW);
}

void updateWifiLed() {
  unsigned long now = millis();
  if ((WiFi.getMode() & WIFI_AP) || WiFi.status() != WL_CONNECTED) {
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

void loop() {
  static unsigned long lastUpdate = 0;
  unsigned long now = millis();

  // Hanya handle HTTP jika AP aktif
  if (WiFi.getMode() & WIFI_AP) {
    server.handleClient();
  }

  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    Serial.println("MQTT disconnected, trying reconnect...");
    connectMQTT();
  }
  mqttClient.loop();

  if (now - lastUpdate >= 1000) {
    globalMoisture  = getMoisture();
    globalGrainTemp = getGrainTemp();
    globalRoomTemp  = getRoomTemp();
    displayOnSevenSegment();
    lastUpdate = now;
  }

  updateWifiLed();

  static unsigned long lastPublish = 0;
  if (mqttClient.connected() && now - lastPublish >= 5000) {
    lastPublish = now;
    publishSensorData();
  }
}
