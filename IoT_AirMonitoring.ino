// LIBRARIES 

// Library for I2C communication (used by most sensors)
#include <Wire.h>

// Base sensor abstraction used by Adafruit sensors
#include <Adafruit_Sensor.h>

// Library for BME280 sensor (temperature, humidity, pressure)
#include <Adafruit_BME280.h>

// ESP32 Wi-Fi library
#include <WiFi.h>

// Library for sending HTTP requests
#include <HTTPClient.h>

// Library for SCD30 CO2 sensor
#include "SparkFun_SCD30_Arduino_Library.h"

// Library for SGP40 VOC sensor
#include "Adafruit_SGP40.h"

// Algorithm to convert raw VOC data into VOC index
#include "VOCGasIndexAlgorithm.h"


// WI-FI CONFIGURATION 

// Wi-Fi network name
const char* ssid = "wifi_name";

// Wi-Fi password
const char* password = "wifi_password";


// API CONFIGURATION 

// API endpoint where data will be sent
const char* apiEndpoint = "api-key";

// Device ID to identify this ESP32
const char* deviceId = "ESP32_01";


// SENSOR OBJECTS 

// BME280 object (temperature, humidity, pressure)
Adafruit_BME280 bme;

// SCD30 object (CO2 sensor)
SCD30 scd30;

// SGP40 object (VOC sensor)
Adafruit_SGP40 sgp40;

// VOC algorithm object
VOCGasIndexAlgorithm voc_algorithm;


// I2C PIN CONFIGURATION 

// SDA pin for I2C communication
#define I2C_SDA 21

// SCL pin for I2C communication
#define I2C_SCL 22


// SPS30 SENSOR CONFIGURATION 

// I2C address of SPS30 particulate matter sensor
#define SPS30_ADDR 0x69

// GPIO pin used to control power to SPS30
#define SPS30_PWR 19


//  CRC FUNCTION 
// Calculates CRC8 checksum to verify SPS30 data integrity
uint8_t sps30_crc8(const uint8_t *buf, uint8_t len) {

  // Initialize CRC value as required by SPS30 protocol
  uint8_t crc = 0xFF;

  // Loop through each byte in buffer
  for (uint8_t i = 0; i < len; i++) {

    // XOR current byte with CRC
    crc ^= buf[i];

    // Process each bit of the byte
    for (uint8_t b = 0; b < 8; b++) {

      // Check most significant bit and apply polynomial if needed
      crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
    }
  }

  // Return final CRC value
  return crc;
}


//  I2C WRITE FUNCTION 
// Sends a 16-bit command to the SPS30 sensor
void i2cWrite16(uint16_t cmd) {

  // Begin I2C transmission to SPS30 address
  Wire.beginTransmission(SPS30_ADDR);

  // Send high byte of command
  Wire.write(cmd >> 8);

  // Send low byte of command
  Wire.write(cmd & 0xFF);

  // End I2C transmission
  Wire.endTransmission();
}


// I2C READ FUNCTION 
// Reads a specific number of bytes from SPS30
bool i2cReadBytes(uint8_t *buf, size_t len) {

  // Request bytes from SPS30
  Wire.requestFrom(SPS30_ADDR, len);

  // Index for buffer
  size_t i = 0;

  // While data is available and buffer is not full
  while (Wire.available() && i < len) {

    // Read byte and store in buffer
    buf[i++] = Wire.read();
  }

  // Return true if expected number of bytes were read
  return i == len;
}


// START SPS30 
// Powers on SPS30 and starts measurement
void startSPS30() {

  // Turn ON power to SPS30
  digitalWrite(SPS30_PWR, HIGH);

  // Short delay to allow power stabilization
  delay(100);

  // Begin I2C communication
  Wire.beginTransmission(SPS30_ADDR);

  // Send "start measurement" command (high byte, low byte)
  Wire.write(0x00);
  Wire.write(0x10);

  // Send measurement configuration
  Wire.write(0x03);
  Wire.write(0x00);

  // Send CRC checksum for configuration
  Wire.write(sps30_crc8((uint8_t[]){0x03,0x00}, 2));

  // End I2C communication
  Wire.endTransmission();

  // Wait for sensor to initialize and stabilize
  delay(3000);
}


// STOP SPS30 
// Stops measurement and powers off SPS30
void stopSPS30() {

  // Send stop measurement command
  i2cWrite16(0x0104);

  // Small delay for safe shutdown
  delay(100);

  // Turn OFF power to SPS30
  digitalWrite(SPS30_PWR, LOW);
}


// CHECK DATA READY 
// Checks if SPS30 has new data available
bool spsDataReady() {

  // Send "data ready" command
  i2cWrite16(0x0202);

  // Buffer to store response
  uint8_t buf[3];

  // If reading fails, return false
  if (!i2cReadBytes(buf, 3))
    return false;

  // Return true if data-ready flag is set
  return buf[1] == 0x01;
}


//  PARSE FLOAT 
// Converts raw SPS30 bytes into float
float parseFloat(const uint8_t* p) {

  // Extract only data bytes (skip CRC bytes)
  uint8_t raw[4] = {p[0], p[1], p[3], p[4]};

  // Combine bytes into 32-bit integer
  uint32_t v =
    (raw[0] << 24) |
    (raw[1] << 16) |
    (raw[2] << 8)  |
    raw[3];

  // Float variable
  float f;

  // Copy bits into float
  memcpy(&f, &v, 4);

  // Return float value
  return f;
}


// READ SPS30 DATA 
// Reads particulate matter values into array
void readSPS30(float* out, int count) {

  // Send "read measurement" command
  i2cWrite16(0x0300);

  // Each data packet is 6 bytes
  const int packet = 6;

  // Buffer to store incoming data
  uint8_t buf[60];

  // If read fails, exit function
  if (!i2cReadBytes(buf, count * packet))
    return;

  // Loop through all measurement values
  for (int i = 0; i < count; i++) {

    // Pointer to current packet
    const uint8_t* p = buf + i * packet;

    // Check CRC validity
    if (sps30_crc8(p, 2) != p[2] ||
        sps30_crc8(p + 3, 2) != p[5]) {

      // If CRC fails, mark value as invalid
      out[i] = NAN;

    } else {

      // If CRC is correct, convert to float
      out[i] = parseFloat(p);
    }
  }
}


//  WIFI CONNECTION 
// Connects ESP32 to Wi-Fi network
void connectWiFi() {

  // If already connected, do nothing
  if (WiFi.status() == WL_CONNECTED)
    return;

  // Print status
  Serial.println("Connecting to Wi-Fi...");

  // Set Wi-Fi mode to station
  WiFi.mode(WIFI_STA);

  // Start Wi-Fi connection
  WiFi.begin(ssid, password);

  // Store start time
  uint32_t start = millis();

  // Wait until connected or timeout
  while (WiFi.status() != WL_CONNECTED &&
         millis() - start < 10000) {

    Serial.print(".");
    delay(500);
  }

  // Print connection result
  if (WiFi.status() == WL_CONNECTED)
    Serial.println("\nWi-Fi connected!");
  else
    Serial.println("\nWi-Fi failed!");
}


//  SETUP FUNCTION 
// Runs once when ESP32 boots
void setup() {

  // Start serial communication
  Serial.begin(115200);

  // Initialize I2C communication
  Wire.begin(I2C_SDA, I2C_SCL);

  // Set I2C speed
  Wire.setClock(100000);

  // Configure SPS30 power pin as output
  pinMode(SPS30_PWR, OUTPUT);

  // Ensure SPS30 is powered off initially
  digitalWrite(SPS30_PWR, LOW);

  // Initialize BME280 sensor
  if (!bme.begin(0x76) && !bme.begin(0x77))
    Serial.println("BME280 FAIL");

  // Initialize SCD30 sensor
  if (scd30.begin()) {
    scd30.setAutoSelfCalibration(true);
    scd30.setMeasurementInterval(2);
  } else {
    Serial.println("SCD30 FAIL");
  }

  // Initialize SGP40 sensor with retries
  int tries = 0;
  while (!sgp40.begin() && tries < 5) {
    Serial.println("SGP40 retry...");
    delay(2000);
    tries++;
  }

  // Connect to Wi-Fi
  connectWiFi();
}


//  LOOP FUNCTION 
// Main program loop
void loop() {

  // Turn on SPS30 and warm up sensors
  startSPS30();
  delay(60000); // 1-minute warm-up

  // Ensure Wi-Fi connection
  if (WiFi.status() != WL_CONNECTED)
    connectWiFi();

  // Read BME280 sensor data
  float T = bme.readTemperature();
  float H = bme.readHumidity();
  float P = bme.readPressure() / 100.0f;
  float A = bme.readAltitude(1013.25);

  // Read CO2 value if available
  float co2ppm =
    scd30.dataAvailable() ? scd30.getCO2() : -1;

  // Read SPS30 particulate matter data
  float pm[10] = {0};
  if (spsDataReady())
    readSPS30(pm, 10);

  // Read VOC raw value and calculate index
  uint16_t sgp_raw = sgp40.measureRaw(T, H);
  int32_t tvoc = voc_algorithm.process(sgp_raw);

  // Create JSON payload
  String payload = "{";
  payload += "\"temperature\":" + String(T,2) + ",";
  payload += "\"pressure\":" + String(P,2) + ",";
  payload += "\"humidity\":" + String(H,2) + ",";
  payload += "\"altitude\":" + String(A,2) + ",";
  payload += "\"co2ppm\":" + String(co2ppm,2) + ",";
  payload += "\"tvoc_index\":" + String(tvoc) + ",";
  payload += "\"deviceId\":\"" + String(deviceId) + "\"}";

  // Print payload
  Serial.println(payload);

  // Send data to server
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(apiEndpoint);
    http.addHeader("Content-Type", "application/json");
    http.POST(payload);
    http.end();
  }

  // Stop SPS30 sensor
  stopSPS30();

  // Put ESP32 into deep sleep to save power
  esp_deep_sleep(9 * 60 * 1000000ULL);
}
