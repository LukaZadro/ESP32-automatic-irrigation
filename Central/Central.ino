#include <esp_now.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include "ESPAsyncWebServer.h"
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include "LittleFS.h"
#include "Protocol.h"

#define VALVE_PIN 27
#define MAX_NODES 16
#define TEMPERATURE_THRESHOLD 29.0f
#define MOISTURE_THRESHOLD 0.3f
#define LIGHT_THRESHOLD 0.6f 

struct central_node {
    bool manual_mode = false;
    bool valve_state = false;
    struct sensor_data latest_sensor;
    bool sensor_available = false;
};

struct central_node central;
volatile bool data_available = false;

// provisioning / nodes
bool pairingMode = false;
uint8_t nextNodeId = 1;
const char *NODES_PATH = "/nodes.json";
const unsigned long PAIRING_TIMEOUT_MS = 60000; // auto-stop pairing after 60s
const unsigned long NODE_TIMEOUT_MS = 20UL * 60UL * 1000UL; // 20 min
unsigned long pairingStartedAt = 0;
volatile bool nodes_dirty = false;
unsigned long lastNodesSaveAt = 0;
const unsigned long NODES_SAVE_DEBOUNCE_MS = 1000;
volatile bool pair_response_pending = false;
uint8_t pending_pair_mac[6] = {0};
pair_response pending_pair_response = {};

struct NodeMeta {
  uint8_t node_id;
  uint8_t channel;
  uint16_t network_id;
  uint8_t mac[6];
};

struct SensorAverages {
    float soil;
    float light;
    float temp;
    uint8_t count;
};

struct NodeState {
  sensor_data data;
  bool online;
  unsigned long last_seen;
  NodeMeta meta;
  bool paired;
};


NodeState nodes[MAX_NODES];

void saveNodes();
void loadNodes();
void resetNodes();
void printAverages();

bool shouldOpenValve()
{
    if (central.manual_mode)
        return central.valve_state;
    SensorAverages avg = calculateAverages();

    if (avg.count == 0)
        return false;
    return
        avg.soil < MOISTURE_THRESHOLD &&
        avg.light < LIGHT_THRESHOLD &&
        avg.temp < TEMPERATURE_THRESHOLD;
}

void setValve(bool open)
{
    central.valve_state = open;
    digitalWrite(VALVE_PIN, open ? HIGH : LOW);
    Serial.printf("Valve %s\n", open ? "OPEN" : "CLOSED");
}

void printSensorData()
{
    const sensor_data& data = central.latest_sensor;

    Serial.print("Soil moisture: ");
    Serial.println(data.soil_moisture);

    Serial.print("Light intensity: ");
    Serial.println(data.light_intensity);

    Serial.print("Air temperature: ");
    Serial.println(data.air_temp);
}

void processSensorData()
{
    printSensorData();
    printAverages();
    setValve(shouldOpenValve());
}

void OnDataRecv(const esp_now_recv_info_t* esp_now_info, const uint8_t *incoming_data, int len)
{
  if (len <= 0) 
    return;

  uint8_t *mac=esp_now_info->src_addr;
  
  Serial.printf("Packet from MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  uint8_t type = incoming_data[0];
  Serial.printf("Incoming packet type: %d\n", type);

  switch(type) {
  case PAIR_REQUEST:
  {
    if (!pairingMode) {
      Serial.println("Pairing mode not enabled");
      return;
    }

    struct pair_request req;
    if (len < (int)sizeof(req)) {
      Serial.println("Wrong message length");
      return;
    }
    memcpy(&req, incoming_data, sizeof(req));

    // provjeri jel postoji node s istim MAC-om
    int existing = -1;
    for (int i = 1; i < MAX_NODES; i++) {
      if (nodes[i].paired) {
        bool same = true;
        for (int b = 0; b < 6; b++) {
          if (nodes[i].meta.mac[b] != mac[b]) { 
            same = false; 
            break; 
          }
        }
        if (same) { 
          existing = i; 
          break; 
        }
      }
    }

    int assigned = -1;
    if (existing >= 0) {
      assigned = existing;
    } else {
      // find first free slot
      for (int i = 1; i < MAX_NODES; i++) {
        if (!nodes[i].paired) { 
          assigned = i; 
          break; 
        }
      }
    }

    if (assigned < 0) {
      Serial.println("No free node slots");
      break;
    }

    // prepare response using assigned slot
    struct pair_response resp;
    resp.type = PAIR_RESPONSE;
    resp.node_id = (uint8_t)assigned;
    resp.channel = WiFi.channel();
    resp.network_id = NETWORK_ID;
    WiFi.macAddress(resp.central_mac); //postavi centralni MAC u response

    Serial.printf("Provisioned node %u\n", resp.node_id);

    // spremi cvor
    NodeState &slot = nodes[assigned];
    slot.paired = true;
    slot.meta.node_id = resp.node_id;
    slot.meta.channel = resp.channel;
    slot.meta.network_id = resp.network_id;
    memcpy(slot.meta.mac, mac, 6);
    slot.online = false;
    slot.last_seen = 0;
    nodes_dirty = true;

    // queue response
    memcpy(pending_pair_mac, mac, 6);
    pending_pair_response = resp;
    pair_response_pending = true;

    break;
  }
  case SENSOR_DATA:
  {
    if (len != (int)sizeof(sensor_data)) 
      return;

    sensor_data temp;
    memcpy(&temp, incoming_data, sizeof(sensor_data));

    uint8_t nid = temp.node_id;
    if (nid < MAX_NODES && nodes[nid].paired) {
      nodes[nid].data = temp;
      nodes[nid].online = true;
      nodes[nid].last_seen = millis();
      //azuriraj zadnje videni senzor
      central.latest_sensor = temp;
      central.sensor_available = true;
      data_available = true;
    } else {
      // unknown node, ignore or request provisioning
      Serial.printf("Sensor data from unknown node %u\n", nid);
    }
    break;
  }
  default:
    // unknown type
    return;
  }
}

void saveNodes() {
  DynamicJsonDocument doc(4096);
  JsonArray nodesArray = doc.createNestedArray("nodes");

  for (int i = 0; i < MAX_NODES; i++) {
    if (!nodes[i].paired) continue;

    JsonObject node = nodesArray.createNestedObject();
    node["node_id"] = nodes[i].meta.node_id;
    node["channel"] = nodes[i].meta.channel;
    node["network_id"] = nodes[i].meta.network_id;

    char macs[13];
    sprintf(macs, "%02X%02X%02X%02X%02X%02X",
            nodes[i].meta.mac[0], nodes[i].meta.mac[1], nodes[i].meta.mac[2],
            nodes[i].meta.mac[3], nodes[i].meta.mac[4], nodes[i].meta.mac[5]);
    node["mac"] = macs;
    node["paired"] = true;
  }

  File file = LittleFS.open(NODES_PATH, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open nodes.json for writing");
    return;
  }

  serializeJson(doc, file);
  file.close();
}

void loadNodes() {
  if (!LittleFS.exists(NODES_PATH)) return;

  File file = LittleFS.open(NODES_PATH, FILE_READ);
  if (!file) return;

  DynamicJsonDocument doc(4096);
  if (deserializeJson(doc, file)) {
    file.close();
    Serial.println("Failed to parse nodes.json");
    return;
  }

  file.close();

  JsonArray nodesArray = doc["nodes"].as<JsonArray>();
  uint8_t maxId = 0;
  for (JsonObject node : nodesArray) {
    uint8_t nid = node["node_id"] | 0;
    if (nid >= MAX_NODES) continue;

    nodes[nid].paired = true;
    nodes[nid].meta.node_id = nid;
    nodes[nid].meta.channel = node["channel"] | 0;
    nodes[nid].meta.network_id = node["network_id"] | 0;

    const char *macs = node["mac"] | "";
    if (strlen(macs) == 12) {
      for (int i = 0; i < 6; i++) {
        char buf[3] = { macs[i*2], macs[i*2+1], '\0' };
        nodes[nid].meta.mac[i] = (uint8_t) strtoul(buf, NULL, 16);
      }
    }

    nodes[nid].online = false;
    nodes[nid].last_seen = 0;
    if (nid > maxId) maxId = nid;
  }

  nextNodeId = maxId + 1;

  for (int i = 0; i < MAX_NODES; i++) {
    if (!nodes[i].paired) continue;

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, nodes[i].meta.mac, 6);
    peerInfo.channel = nodes[i].meta.channel;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
}

SensorAverages calculateAverages()
{
    SensorAverages avg = {};
    
    float soilSum = 0.0f;
    float lightSum = 0.0f;
    float tempSum = 0.0f;
    
    for (int i = 0; i < MAX_NODES; i++) {
        if (!nodes[i].paired || !nodes[i].online)
            continue;

        soilSum += nodes[i].data.soil_moisture;
        lightSum += nodes[i].data.light_intensity;
        tempSum += nodes[i].data.air_temp;

        avg.count++;
    }

    if (avg.count > 0) {
        avg.soil = soilSum / avg.count;
        avg.light = lightSum / avg.count;
        avg.temp = tempSum / avg.count;
    }

    return avg;
}

void resetNodes() {
  for (int i = 0; i < MAX_NODES; i++) {
    nodes[i] = NodeState{};
  }

  nextNodeId = 1;

  if (LittleFS.exists(NODES_PATH)) {
    LittleFS.remove(NODES_PATH);
  }

  Serial.println("Saved nodes reset");
}

void printAverages()
{
    SensorAverages avg = calculateAverages();

    Serial.println("=== Averages ===");
    Serial.printf("Soil: %.2f\n", avg.soil);
    Serial.printf("Light: %.2f\n", avg.light);
    Serial.printf("Temp: %.2f\n", avg.temp);
}

WiFiManager wm;

AsyncWebServer server(80);

void setup() {
  // inicijalizacija serijskog monitora
  Serial.begin(115200);

  delay(1500);

  //nextNodeId = 1;  //za reset
  resetNodes(); // resetiraj spremljene nodeove za lakse testiranje
  wm.resetSettings();  //svaki put se resetiraju postavke da je lakse testiranje

  WiFi.mode(WIFI_AP_STA);
  
  if(!wm.autoConnect("Lukin esp32")) {
    Serial.println("Failed to connect");
  } else {
    Serial.println("Connected...");
  }

  Serial.print("Channel: ");
  Serial.println(WiFi.channel());

  //
  if (!LittleFS.begin(true)) {
    Serial.println("LittleFS mount failed");
    while(1) {
      delay(1000);
    }
  }

  Serial.println(LittleFS.exists("/index.html") ? "index.html found" : "index.html NOT found");

  //inicijalizacija ESP-NOW protokola
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while(1) {
      delay(1000);
    }
  }

  esp_now_register_recv_cb(OnDataRecv);

  // ucitaj cvorove iz LittleFS
  loadNodes();
  
  pinMode(VALVE_PIN, OUTPUT);
  digitalWrite(VALVE_PIN, 0);

  // postavi mDNS
  if (!MDNS.begin("esp32")) {
    Serial.println("Error setting up MDNS responder!");
    while(1) {
      delay(1000);
    }
  }
  // oglasi uslugu
  MDNS.addService("_http", "_tcp", 80); 

  // rute
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(LittleFS, "/index.html", "text/html");
  });

  server.on("/test", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    request->send(200, "text/plain", "hello");
  });

  // // slanje senzorskih podataka
  // server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request)
  // {
  //   StaticJsonDocument<200> doc;

  //   if (!data_available) {
  //     doc["soil"] = nullptr;
  //     doc["light"] = nullptr;
  //     doc["temp"] = nullptr;
  //   } else {
  //     doc["soil"] = central.latest_sensor.soil_moisture;
  //     doc["light"] = central.latest_sensor.light_intensity;
  //     doc["temp"] = central.latest_sensor.air_temp;
  //   }

  //   String out;
  //   serializeJson(doc, out);
  //   request->send(200, "application/json", out);
  // });

  server.on("/api/saved_nodes", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    if (!LittleFS.exists(NODES_PATH)) {
      request->send(200, "application/json", "{\"nodes\":[]}");
      return;
    }

    File f = LittleFS.open(NODES_PATH, FILE_READ);
    if (!f) {
      request->send(500, "application/json", "{\"error\":\"failed to open nodes file\"}");
      return;
    }

    String content = f.readString();
    f.close();
    request->send(200, "application/json", content);
  });

  // reset cvorova
  server.on("/api/saved_nodes/reset", HTTP_POST, [](AsyncWebServerRequest *request)
  {
    resetNodes();
    request->send(200, "application/json", "{\"ok\":true,\"message\":\"saved nodes reset\"}");
  });

  server.on("/api/nodes", HTTP_GET, [](AsyncWebServerRequest *request)
  {
    DynamicJsonDocument doc(2048);
    JsonArray nodesArray = doc.createNestedArray("nodes");
    JsonObject summary = doc.createNestedObject("summary");

    float soilSum = 0.0f;
    float lightSum = 0.0f;
    float tempSum = 0.0f;
    uint8_t onlineCount = 0;

    for (int i = 0; i < MAX_NODES; i++) {
      if (!nodes[i].paired) continue;

      JsonObject node = nodesArray.createNestedObject();
      node["node_id"] = nodes[i].meta.node_id;
      node["online"] = nodes[i].online;
      node["channel"] = nodes[i].meta.channel;
      node["last_seen_ms"] = nodes[i].last_seen;
      node["network_id"] = nodes[i].meta.network_id;

      char macs[13];
      sprintf(macs, "%02X%02X%02X%02X%02X%02X",
              nodes[i].meta.mac[0], nodes[i].meta.mac[1], nodes[i].meta.mac[2],
              nodes[i].meta.mac[3], nodes[i].meta.mac[4], nodes[i].meta.mac[5]);
      node["mac"] = macs;

      if (nodes[i].online) {
        node["soil"] = nodes[i].data.soil_moisture;
        node["light"] = nodes[i].data.light_intensity;
        node["temp"] = nodes[i].data.air_temp;

        soilSum += nodes[i].data.soil_moisture;
        lightSum += nodes[i].data.light_intensity;
        tempSum += nodes[i].data.air_temp;
        onlineCount++;
      }
    }

    summary["online_count"] = onlineCount;
    summary["soil_avg"] = onlineCount > 0 ? soilSum / onlineCount : NULL;
    summary["light_avg"] = onlineCount > 0 ? lightSum / onlineCount : NULL;
    summary["temp_avg"] = onlineCount > 0 ? tempSum / onlineCount : NULL;

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  // dohvat trenutnog stanja sustava (rucno ili automatsko navodnjavanje)
  server.on("/api/manualMode", HTTP_GET, [](AsyncWebServerRequest *request) 
  {
    request->send(200, "text/plain", central.manual_mode ? "on" : "off");
  });

  // je li ventil otvoren ili zatvoren
  // ukljuci rucni nacin
  server.on("/api/manualMode/on", HTTP_POST, [](AsyncWebServerRequest *request)
  {
    central.manual_mode = true;
    Serial.println("Manual mode: ON");
    request->send(200, "text/plain", "Manual mode enabled");
  });

  // iskljuci rucni nacin
  server.on("/api/manualMode/off", HTTP_POST, [](AsyncWebServerRequest *request)
  {
    central.manual_mode = false;
    Serial.println("Manual mode: OFF");
    request->send(200, "text/plain", "Manual mode disabled");
  });

  // otvori ventil (samo u rucnom nacinu)
  server.on("/api/ventil/on", HTTP_POST, [](AsyncWebServerRequest *request)
  {
    if (!central.manual_mode) {
      request->send(403, "text/plain", "Not in manual mode");
      return;
    }
    setValve(true);
    request->send(200, "text/plain", "Valve ON");
  });

  // zatvori ventil (samo u rucnom nacinu)
  server.on("/api/ventil/off", HTTP_POST, [](AsyncWebServerRequest *request)
  {
    if (!central.manual_mode) {
      request->send(403, "text/plain", "Not in manual mode");
      return;
    }
    setValve(false);
    request->send(200, "text/plain", "Valve OFF");
  });

  server.on("/api/pairing/stop", HTTP_POST, [](AsyncWebServerRequest *request)
  {
    pairingMode = false;
    Serial.println("Pairing mode disabled");
    request->send(200, "text/plain", "Pairing mode disabled");
  });

  server.on("/api/pairing/start", HTTP_POST, [](AsyncWebServerRequest *request)
  {
    pairingMode = true;
    pairingStartedAt = millis();

    Serial.println("Pairing mode enabled");

    request->send(200, "text/plain", "Pairing mode enabled");
  });

  
  //pokretanje servera i postavljanje 
  server.serveStatic("/", LittleFS, "/");
  server.begin();
  Serial.println("Server started");
}

void loop()
{
  if (pair_response_pending) {
    pair_response_pending = false;

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, pending_pair_mac, 6);
    peerInfo.channel = WiFi.channel();
    peerInfo.encrypt = false;
    esp_err_t pe = esp_now_add_peer(&peerInfo);
    Serial.printf("esp_now_add_peer returned %d\n", (int)pe);

    Serial.printf("Sending response to MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                  pending_pair_mac[0], pending_pair_mac[1], pending_pair_mac[2],
                  pending_pair_mac[3], pending_pair_mac[4], pending_pair_mac[5]);
    esp_err_t se = esp_now_send(pending_pair_mac, (uint8_t*)&pending_pair_response, sizeof(pending_pair_response));
    Serial.printf("esp_now_send returned %d\n", (int)se);
  }

  if (data_available) {
    data_available = false;
    processSensorData();
  }

  if (nodes_dirty && (millis() - lastNodesSaveAt > NODES_SAVE_DEBOUNCE_MS)) {
    nodes_dirty = false;
    lastNodesSaveAt = millis();
    saveNodes();
  }

  // auto-stop pairing after timeout
  if (pairingMode && (millis() - pairingStartedAt > PAIRING_TIMEOUT_MS)) {
    pairingMode = false;
    Serial.println("Pairing mode timeout, disabled");
  }

  //provjera koji je node postao offline
  for (int i = 1; i < MAX_NODES; i++) {
    if (!nodes[i].paired)
        continue;

    if (nodes[i].online && millis() - nodes[i].last_seen > NODE_TIMEOUT_MS){
        nodes[i].online = false;
    }
}
  delay(10);
}
