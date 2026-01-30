#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

#define LIGHT_SENSOR_PIN 35
#define SOIL_MOISTURE_SENSOR_PIN 34
#define WET_VALUE 990
#define DRY_VALUE 3310

//MAC adresa drugogo mikrokontrolera
uint8_t broadcast_address[] = {0x00, 0x70, 0x07, 0x1A, 0x48, 0x60};

typedef struct struct_message {
  float light_intensity;
  float soil_moisture;
  float air_temp;
} struct_message;

struct_message my_data;

esp_now_peer_info_t peerInfo;

//callback funkcija nakon slanja podataka
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

Adafruit_BMP280 bmp; // I2C

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW!");
    return;
  }
  esp_now_register_send_cb(esp_now_send_cb_t(OnDataSent));

   // Register peer
  memcpy(peerInfo.peer_addr, broadcast_address, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // add peer      
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
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
}

void loop() {
  int raw_light_val = analogRead(LIGHT_SENSOR_PIN);
  float light_intensity = (float) (4095 - raw_light_val) / 4095;

  int raw_soil_moisture_val = analogRead(SOIL_MOISTURE_SENSOR_PIN);
  int clamped = constrain(raw_soil_moisture_val, WET_VALUE, DRY_VALUE);
  float soil_moisture = (float)(DRY_VALUE - clamped) / (DRY_VALUE - WET_VALUE);

  
  my_data.light_intensity = light_intensity;
  my_data.soil_moisture = soil_moisture;
  my_data.air_temp = bmp.readTemperature();

  //Serial.print("Soil moisture: ");
  //Serial.println(soil_moisture);
  //erial.print("Light intensity: ");
  //Serial.println(light_intensity);
  //Serial.println(light_intensity);

  // slanje poruke
  esp_err_t result = esp_now_send(broadcast_address, (uint8_t *) &my_data, sizeof(my_data));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }

  delay(20000);
}
