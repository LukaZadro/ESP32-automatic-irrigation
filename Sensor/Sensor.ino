#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_system.h> 
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Preferences.h>
#include "Protocol.h"

#define LIGHT_SENSOR_PIN 35
#define SOIL_MOISTURE_SENSOR_PIN 34
#define WET_VALUE 990
#define DRY_VALUE 3310
#define uS_TO_S_FACTOR 1000000ULL 
#define TIME_TO_SLEEP  600 //10 minuta          

// broadcast address for pairing
uint8_t broadcast_address[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// stored central mac (used after pairing)
uint8_t central_mac[6] = {0};

uint8_t mac[6];

Preferences prefs;
volatile bool is_paired = false;
uint8_t saved_node_id = 0;

esp_now_peer_info_t peerInfo;

sensor_data read_sensors();
void sendData(sensor_data *my_data);


//callback funkcija nakon slanja podataka
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incoming_data, int len) {
  if (len <= 0) {
    Serial.println("Wrong message length");
    return;
  }

  Serial.println("Received a packet");
  uint8_t type = incoming_data[0];
  if (type == PAIR_RESPONSE) {

    Serial.println("Received a response packet");
    if (len < (int)sizeof(pair_response)) 
      return;

    pair_response resp;
    memcpy(&resp, incoming_data, sizeof(resp));
    Serial.printf("PAIR_RESPONSE: node_id=%u channel=%u network_id=%u\n",
                  resp.node_id, resp.channel, resp.network_id);
    

    if (resp.network_id != NETWORK_ID) 
    {
      Serial.printf("Network mismatch: expected %u got %u\n", NETWORK_ID, resp.network_id);
      return;
    }

    esp_wifi_set_channel(resp.channel, WIFI_SECOND_CHAN_NONE); //postavi node na isti kanal kao i centralni esp

    // store config in prefs
    prefs.begin("sensor", false);
    prefs.putBool("paired", true);
    prefs.putBytes("node_id", &resp.node_id, 1);
    prefs.putUChar("channel", resp.channel);
    prefs.putBytes("network_id", &resp.network_id, 2);
    prefs.putBytes("central_mac", resp.central_mac, 6);
    prefs.end();
    memcpy(central_mac, resp.central_mac, 6);
    saved_node_id = resp.node_id;
    is_paired = true;

    // register peer for future unicast
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, central_mac, 6);
    p.channel = resp.channel;
    p.encrypt = false;
    esp_now_add_peer(&p);

    Serial.printf("Paired with central, node_id=%u\n", saved_node_id);
  }
}

Adafruit_BMP280 bmp; // I2C

void setup() {
  Serial.begin(115200);

  delay(1500);

  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); //timer za sleep
  WiFi.mode(WIFI_STA);

  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW!");
    return;
  }

  delay(200);


  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));

  memcpy(peerInfo.peer_addr, broadcast_address, 6);
  peerInfo.encrypt = false;

  prefs.begin("sensor", false);

  // provjeri ako je vec uparen
  is_paired = prefs.getBool("paired", false);

  if (is_paired) {
    uint8_t ch = prefs.getUChar("channel"); 
    prefs.getBytes("central_mac", central_mac, 6);
    prefs.getBytes("node_id", &saved_node_id, 1);
    
    Serial.printf("Paired on channel: %d\n", ch);

    esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);

    // registriraj peera
    memcpy(peerInfo.peer_addr, central_mac, 6);

    peerInfo.channel = ch;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    // posalji podatke sa senzora
    sensor_data sd = read_sensors();
    sd.type = SENSOR_DATA;
    sd.node_id = saved_node_id;
    sendData(&sd);
    //return;
  } else {
    for (uint8_t ch = 1; ch <= 13 && !is_paired; ch++) {
      esp_wifi_set_channel(ch, WIFI_SECOND_CHAN_NONE);
      peerInfo.channel = ch;
      esp_now_del_peer(broadcast_address);
      esp_now_add_peer(&peerInfo);

      pair_request req;
      req.type = PAIR_REQUEST;
      esp_now_send(broadcast_address, (uint8_t*)&req, sizeof(req));

      unsigned long start = millis();
      while (millis() - start < 300) {
        delay(20);
      }
      if (is_paired) {
        prefs.putChar("channel", ch);
        prefs.putBool("paired", true);
      }
    }
  }
  //bmp sensor setup
  unsigned status = bmp.begin(0x76); //I2C adresa
  if (!status) {
    Serial.println("Could not find a valid BMP280 sensor");
  }

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  Serial.println("Not paired yet, trying restarting the device again or wait for sleep to end");
  // attempt pairing and let the response tell us the receiver channeld
  // esp_now_del_peer(broadcast_address);
  // peerInfo.channel = 0;
  // if (esp_now_add_peer(&peerInfo) != ESP_OK) {
  //   Serial.println("Failed to add broadcast peer for pairing");
  // }

  prefs.end();
  esp_deep_sleep_start(); 
}

sensor_data read_sensors() {
  sensor_data sd;
  // inicijalizacija
  sd.light_intensity = 0.0f;
  sd.soil_moisture = 0.0f;
  sd.air_temp = 0.0f;
  sd.type = SENSOR_DATA;
  sd.node_id = saved_node_id;

  int raw_light_val = analogRead(LIGHT_SENSOR_PIN);
  float light_intensity = (float) (4095 - raw_light_val) / 4095.0f;

  int raw_soil_moisture_val = analogRead(SOIL_MOISTURE_SENSOR_PIN);
  int clamped = constrain(raw_soil_moisture_val, WET_VALUE, DRY_VALUE);
  float soil_moisture = (float)(DRY_VALUE - clamped) / (float)(DRY_VALUE - WET_VALUE);

  sd.light_intensity = light_intensity;
  sd.soil_moisture = soil_moisture;
  sd.air_temp = bmp.readTemperature();

  return sd;
}


void sendData(sensor_data *my_data) {
  esp_err_t result = esp_now_send(is_paired ? central_mac : broadcast_address, (uint8_t *) my_data, sizeof(sensor_data));
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  } else {
    Serial.println("Error sending the data");
  }
}

void loop() {

}
