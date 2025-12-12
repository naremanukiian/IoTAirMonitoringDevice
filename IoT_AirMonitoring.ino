#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include "SparkFun_SCD30_Arduino_Library.h"
#include "Adafruit_SGP40.h"
#include "VOCGasIndexAlgorithm.h"

// ----------------- Wi-Fi -----------------
const char* ssid = "wifi_name";
const char* password = "wifi_password";

// ----------------- API -------------------
const char* apiEndpoint = "api-key";
const char* deviceId    = "ESP32_01";

// ----------------- Sensors -----------------
Adafruit_BME280 bme;
SCD30 scd30;
Adafruit_SGP40 sgp40;
VOCGasIndexAlgorithm voc_algorithm;

#define I2C_SDA 21
#define I2C_SCL 22

// ----------------- SPS30 -----------------
#define SPS30_ADDR 0x69
#define SPS30_PWR 19

uint8_t sps30_crc8(const uint8_t *buf, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= buf[i];
    for (uint8_t b = 0; b < 8; b++) crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
  }
  return crc;
}

void i2cWrite16(uint16_t cmd) {
  Wire.beginTransmission(SPS30_ADDR);
  Wire.write(cmd >> 8);
  Wire.write(cmd & 0xFF);
  Wire.endTransmission();
}

bool i2cReadBytes(uint8_t *buf, size_t len) {
  Wire.requestFrom(SPS30_ADDR, len);
  size_t i = 0;
  while (Wire.available() && i < len) buf[i++] = Wire.read();
  return i == len;
}

void startSPS30() {
  digitalWrite(SPS30_PWR, HIGH);   // power on
  delay(100);
  Wire.beginTransmission(SPS30_ADDR);
  Wire.write(0x00); Wire.write(0x10);
  Wire.write(0x03); Wire.write(0x00);
  Wire.write(sps30_crc8((uint8_t[]){0x03,0x00},2));
  Wire.endTransmission();
  delay(3000);
}

void stopSPS30() {
  i2cWrite16(0x0104); // stop measurement
  delay(100);
  digitalWrite(SPS30_PWR, LOW);
}

bool spsDataReady() {
  i2cWrite16(0x0202);
  uint8_t buf[3];
  if (!i2cReadBytes(buf,3)) return false;
  return buf[1] == 0x01;
}

float parseFloat(const uint8_t* p) {
  uint8_t raw[4] = {p[0], p[1], p[3], p[4]};
  uint32_t v = (raw[0]<<24)|(raw[1]<<16)|(raw[2]<<8)|raw[3];
  float f; memcpy(&f,&v,4); return f;
}

void readSPS30(float* out, int count) {
  i2cWrite16(0x0300);
  const int packet = 6;
  uint8_t buf[60];
  if (!i2cReadBytes(buf, count*packet)) return;
  for (int i=0;i<count;i++){
    const uint8_t* p = buf + i*packet;
    if (sps30_crc8(p,2)!=p[2] || sps30_crc8(p+3,2)!=p[5])
      out[i] = NAN;
    else
      out[i] = parseFloat(p);
  }
}

// ----------------- Wi-Fi -----------------
void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  Serial.println("Connecting to Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  uint32_t start=millis();
  while (WiFi.status()!=WL_CONNECTED && millis()-start<10000){Serial.print("."); delay(500);}
  if(WiFi.status()==WL_CONNECTED) Serial.println("\nWi-Fi connected!");
  else Serial.println("\nWi-Fi failed!");
}

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA,I2C_SCL);
  Wire.setClock(100000);

  pinMode(SPS30_PWR, OUTPUT);
  digitalWrite(SPS30_PWR, LOW); // off initially

  // BME280
  if(!bme.begin(0x76) && !bme.begin(0x77)) Serial.println("BME280 FAIL");

  // SCD30
  if(scd30.begin()){ scd30.setAutoSelfCalibration(true); scd30.setMeasurementInterval(2); }
  else Serial.println("SCD30 FAIL");

  // SGP40
  int tries=0;
  while(!sgp40.begin() && tries<5){ Serial.println("SGP40 retry..."); delay(2000); tries++; }
  if(!sgp40.begin()) Serial.println("SGP40 FAIL");

  connectWiFi();
}

// ----------------- LOOP -----------------
void loop() {
  // ---- Wake up all sensors 1 min before reading ----
  startSPS30();
  Serial.println("Warming sensors 1 min...");
  delay(60000); // 1 min warm-up

  if(WiFi.status()!=WL_CONNECTED) connectWiFi();

  // ---- Read sensors ----
  float T=bme.readTemperature();
  float H=bme.readHumidity();
  float P=bme.readPressure()/100.0f;
  float A=bme.readAltitude(1013.25);

  float co2ppm = scd30.dataAvailable()?scd30.getCO2():-1;

  float pm[10]={0};
  if(spsDataReady()) readSPS30(pm,10);

  uint16_t sgp_raw=0; int32_t tvoc=0;
  if(sgp40.begin()){ sgp_raw=sgp40.measureRaw(T,H); tvoc=voc_algorithm.process(sgp_raw); }

  // ---- Prepare JSON ----
  String payload="{";
  payload+="\"temperature\":"+String(T,2)+",";
  payload+="\"pressure\":"+String(P,2)+",";
  payload+="\"humidity\":"+String(H,2)+",";
  payload+="\"altitude\":"+String(A,2)+",";
  payload+="\"co2ppm\":"+String(co2ppm,2)+",";
  payload+="\"pm1_0\":"+String(pm[0],2)+",";
  payload+="\"pm2_5\":"+String(pm[1],2)+",";
  payload+="\"pm4_0\":"+String(pm[2],2)+",";
  payload+="\"pm10\":"+String(pm[3],2)+",";
  payload+="\"nc0_5\":"+String(pm[4],2)+",";
  payload+="\"nc1_0\":"+String(pm[5],2)+",";
  payload+="\"nc2_5\":"+String(pm[6],2)+",";
  payload+="\"nc4_0\":"+String(pm[7],2)+",";
  payload+="\"nc10\":"+String(pm[8],2)+",";
  payload+="\"typical_size\":"+String(pm[9],2)+",";
  payload+="\"sgp40_raw\":"+String(sgp_raw)+",";
  payload+="\"tvoc_index\":"+String(tvoc)+",";
  payload+="\"deviceId\":\""+String(deviceId)+"\"}";

  Serial.println(payload);

  // ---- Send HTTP ----
  if(WiFi.status()==WL_CONNECTED){
    HTTPClient http;
    http.begin(apiEndpoint);
    http.addHeader("Content-Type","application/json");
    int code=http.POST(payload);
    Serial.print("HTTP: "); Serial.println(code);
    http.end();
  } else Serial.println("❌ Could not send — Wi-Fi down");

  // ---- Stop all sensors ----
  stopSPS30();
  // BME280, SCD30, SGP40: no hardware off, but ESP will sleep

  Serial.println("All sensors stopped. Deep sleep 9 min...");
  esp_deep_sleep(9*60*1000000ULL); // 9 min deep sleep
}

