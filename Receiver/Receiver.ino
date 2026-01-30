#include <esp_now.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include "ESPAsyncWebServer.h"
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include "LittleFS.h"

#define VALVE_PIN 27

typedef struct struct_message {
  float light_intensity;
  float soil_moisture;
  float air_temp;
} struct_message;

//inicijalizacija strukure za podatke
struct_message my_data;
//semafor za i bool varijabla za pristup podacima
volatile bool dataAvailable = false;

//false => navodnjavanje po senzorima, true => rucno preko web stranice
bool manual_mode = false;

//po defaultu je ventil zatvoren
bool valve_state = false;

void OnDataRecv(const uint8_t * mac, const uint8_t *incoming_data, int len) {

  struct_message tmp;
  memcpy(&tmp, incoming_data, sizeof(tmp));

  //azuriramo globalne vrijednosti
  my_data = tmp;
  dataAvailable = true;

  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Soil moisture: ");
  Serial.println(my_data.soil_moisture);
  Serial.print("Light intensity: ");
  Serial.println(my_data.light_intensity);
  Serial.print("Air temperature: ");
  Serial.println(my_data.air_temp);

  bool should_open;

  if (manual_mode) {
    //ako je manualni mod onda otvaramo ili ne prema tome sta je odabrano na webu
    should_open = valve_state;
  } else {
    should_open = 
    my_data.soil_moisture < 0.3 &&
    my_data.light_intensity < 0.6 &&
    my_data.air_temp < 29;
  }
  valve_state = should_open:
  digitalWrite(VALVE_PIN, should_open ? HIGH : LOW);

}

WiFiManager wm;

AsyncWebServer server(80);

void setup() {
  // inicijalizacija serijskog monitora
  Serial.begin(115200);

  delay(1500);

  
  //wm.resetSettings();  //svaki put se resetiraju postavke da je lakse testiranje

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

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  
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
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(LittleFS, "/index.html", "text/html");
  });

  // slanje senzorskih podataka
  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
    StaticJsonDocument<200> doc;

    if (!dataAvailable) {
      doc["soil"] = nullptr;
      doc["light"] = nullptr;
      doc["temp"] = nullptr;
    } else {
      doc["soil"] = my_data.soil_moisture;
      doc["light"] = my_data.light_intensity;
      doc["temp"] = my_data.air_temp;
    }

    String out;
    serializeJson(doc, out);
    request->send(200, "application/json", out);
  });

  // dohvat trenutnog stanja sustava (rucno ili automatsko navodnjavanje)
  server.on("/api/manualMode", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", manual_mode ? "on" : "off");
  });

  // je li ventil otvoren ili zatvoren
  // ukljuci rucni nacin
  server.on("/api/manualMode/on", HTTP_POST, [](AsyncWebServerRequest *request){
    manual_mode = true;
    Serial.println("Manual mode: ON");
    request->send(200, "text/plain", "Manual mode enabled");
  });

  // iskljuci rucni nacin
  server.on("/api/manualMode/off", HTTP_POST, [](AsyncWebServerRequest *request){
    manual_mode = false;
    Serial.println("Manual mode: OFF");
    // Immediately revert to sensor-based control
    request->send(200, "text/plain", "Manual mode disabled");
  });

  // otvori ventil (samo u rucnom nacinu)
  server.on("/api/ventil/on", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!manual_mode) {
      request->send(403, "text/plain", "Not in manual mode");
      return;
    }
    valve_state = true;
    digitalWrite(VALVE_PIN, HIGH);
    Serial.println("Valve manually opened");
    request->send(200, "text/plain", "Valve ON");
  });

  // zatvori ventil (samo u rucnom nacinu)
  server.on("/api/ventil/off", HTTP_POST, [](AsyncWebServerRequest *request){
    if (!manual_mode) {
      request->send(403, "text/plain", "Not in manual mode");
      return;
    }
    valve_state = false;
    digitalWrite(VALVE_PIN, LOW);
    Serial.println("Valve manually closed");
    request->send(200, "text/plain", "Valve OFF");
  });

  
  //pokretanje servera i postavljanje 
  server.serveStatic("/", LittleFS, "/");
  server.begin();
  
}

void loop() {

}
