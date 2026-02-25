#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <FS.h>
#include <Wire.h>
#include <DHT.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>


// ==================== CONFIGURATION ====================
const char *BACKEND_URL = "https://backend-bsfly.vercel.app";

String DEVICE_ID;
String DEVICE_ID_CLEAN;

// ==================== I2C BUS ====================
#define I2C_SDA 21
#define I2C_SCL 22

// ==================== SPI BUS (microSD) ====================
#define SPI_SCK 18
#define SPI_MISO 19
#define SPI_MOSI 23
#define SPI_CS_SD 5

// ==================== CD74HC4067 ANALOG MUX ====================
#define MUX_SIG 35
#define MUX_S0 16
#define MUX_S1 17
#define MUX_S2 25
#define MUX_S3 26

// ==================== DHT22 SENSORS ====================
#define DHT_A_PIN 27
#define DHT_B_PIN 13
#define DHT_C_PIN 33
#define DHT_TYPE DHT22

// ==================== I2C DEVICE ADDRESSES ====================
#define ADS1115_ADDR_1 0x48
#define ADS1115_ADDR_2 0x49
#define MCP23017_ADDR 0x20
#define TCA9548A_ADDR 0x70

// ==================== TCA9548A CHANNELS ====================
#define TCA_CH_OLED1 0
#define TCA_CH_OLED2 1

// ==================== MCP23017 PINS ====================
#define MCP_HUMIDIFIER1 0
#define MCP_HUMIDIFIER2 1
#define MCP_HEATER 2
#define MCP_FAN1 3
#define MCP_FAN2 4
#define MCP_FAN3 5
#define MCP_FAN4 6
#define MCP_HUMIDIFIER3 8
#define MCP_FAN5 9

// ==================== ADS1115 CHANNELS ====================
#define ADS_SOIL1 0
#define ADS_SOIL2 1
#define ADS_SOIL3 2
#define ADS_MQ137 3

// ==================== CD74HC4067 CHANNELS ====================
#define MUX_CH_SOIL1 0
#define MUX_CH_SOIL2 1
#define MUX_CH_SOIL3 2

// ==================== TIMING ====================
#define POLL_INTERVAL 2000
#define SENSOR_INTERVAL 35000
#define HEARTBEAT_INTERVAL 30000
#define SD_SYNC_INTERVAL 60000
#define SD_DATA_FILE "/sensor_data.json"

// ==================== OFFLINE THRESHOLDS ====================
#define TEMP_MIN 25.0
#define TEMP_MAX 35.0
#define TEMP_OPTIMAL_LOW 28.0
#define TEMP_OPTIMAL_HIGH 32.0

#define HUMIDITY_MIN 50.0
#define HUMIDITY_MAX 80.0
#define HUMIDITY_OPTIMAL_LOW 60.0
#define HUMIDITY_OPTIMAL_HIGH 70.0

#define MOISTURE_MIN 40
#define MOISTURE_MAX 70
#define MOISTURE_OPTIMAL_LOW 50
#define MOISTURE_OPTIMAL_HIGH 60

// ==================== GLOBALS ====================
DHT dhtA(DHT_A_PIN, DHT_TYPE);
DHT dhtB(DHT_B_PIN, DHT_TYPE);
DHT dhtC(DHT_C_PIN, DHT_TYPE);

Adafruit_ADS1115 ads1;
Adafruit_ADS1115 ads2;
Adafruit_MCP23X17 mcp;

bool ads1Available = false;
bool ads2Available = false;
bool mcpAvailable = false;
bool sdAvailable = false;

unsigned long lastPollTime = 0;
unsigned long lastSensorTime = 0;
unsigned long lastHeartbeatTime = 0;
unsigned long lastSdSyncTime = 0;

bool lightState = false;

WebServer server(80);

// ==================== FUNCTION PROTOTYPES ====================
void setupWebServer();
void sendHeartbeat();
void pollActuators();
void pollActuator(const char *actuator);
void applyActuatorState(const char *actuator, bool state);
void sendSensorData();
void collectAndProcessDrawer12();
void collectAndProcessDrawer3();
int readSoil1();
int readSoil2();
int readSoil3();
int readMQ137();
void sendOrStoreSensorReading(const char *drawerName, float temperature, float humidity, int moisture, int ammonia);
bool sendSensorReading(const char *drawerName, float temperature, float humidity, int moisture, int ammonia);
void storeSensorToSD(const char *drawerName, float temperature, float humidity, int moisture, int ammonia);
void uploadStoredData();
unsigned long getServerTime();
void autoControlDrawer12(float temperature, float humidity, int moisture);
void autoControlDrawer3(float temperature, float humidity);
void setHumidifier1(bool state);
void setHumidifier2(bool state);
void setHumidifier3(bool state);
void setHeater(bool state);
void setFan1(bool state);
void setFan2(bool state);
void setFan3(bool state);
void setFan4(bool state);
void setFan5(bool state);
void handleRoot();
void handleStatus();
void handleGetSdData();
void handleClearSdData();
void handleSyncSdData();

// ==================== SETUP ====================
void setup()
{
  Serial.begin(115200);
  Serial.println("[setup] Entered setup()");
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);

  pinMode(SPI_CS_SD, OUTPUT);
  digitalWrite(SPI_CS_SD, HIGH);

  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS_SD);

  delay(100);

  sdAvailable = SD.begin(SPI_CS_SD, SPI, 2000000);
  if (sdAvailable)
  {
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE)
    {
      Serial.println("[setup] No SD card attached");
      sdAvailable = false;
    }
    else
    {
      Serial.print("[setup] SD card type: ");
      if (cardType == CARD_MMC)
        Serial.println("MMC");
      else if (cardType == CARD_SD)
        Serial.println("SDSC");
      else if (cardType == CARD_SDHC)
        Serial.println("SDHC");
      else
        Serial.println("UNKNOWN");

      uint64_t cardSize = SD.cardSize() / (1024 * 1024);
      Serial.printf("[setup] SD card size: %lluMB\n", cardSize);
    }
  }
  else
  {
    Serial.println("[setup] SD card mount failed. Check:");
    Serial.println("  - Wiring: SCK=18, MISO=19, MOSI=23, CS=5");
    Serial.println("  - Card formatted as FAT32");
    Serial.println("  - Card inserted properly");
  }

  pinMode(MUX_S0, OUTPUT);
  pinMode(MUX_S1, OUTPUT);
  pinMode(MUX_S2, OUTPUT);
  pinMode(MUX_S3, OUTPUT);

  dhtA.begin();
  dhtB.begin();
  dhtC.begin();

  ads1Available = ads1.begin(ADS1115_ADDR_1);
  if (!ads1Available)
  {
    Serial.println("[setup] ADS1115 #1 not found");
  }

  ads2Available = ads2.begin(ADS1115_ADDR_2);
  if (!ads2Available)
  {
    Serial.println("[setup] ADS1115 #2 not found");
  }

  mcpAvailable = mcp.begin_I2C(MCP23017_ADDR);
  if (mcpAvailable)
  {
    for (int i = 0; i < 16; i++)
    {
      mcp.pinMode(i, OUTPUT);
      mcp.digitalWrite(i, LOW);
    }
  }
  else
  {
    Serial.println("[setup] MCP23017 not found");
  }

  WiFiManager wm;
  wm.setConfigPortalTimeout(180);

  bool res = wm.autoConnect("Connect2Wifi", "esp32bsf");

  if (!res)
  {
    Serial.println("[setup] Failed to connect to WiFi");
    delay(2000);
    ESP.restart();
  }
  else
  {
    Serial.print("[setup] Connected! IP: ");
    Serial.println(WiFi.localIP());

    DEVICE_ID = WiFi.macAddress();
    DEVICE_ID_CLEAN = DEVICE_ID;
    DEVICE_ID_CLEAN.replace(":", "");

    Serial.print("[setup] Device ID: ");
    Serial.println(DEVICE_ID);
    Serial.print("[setup] Device ID (clean): ");
    Serial.println(DEVICE_ID_CLEAN);
  }

  setupWebServer();
  sendHeartbeat();
  Serial.println("[setup] setup() complete");
}

// ==================== MAIN LOOP ====================
void loop()
{
  Serial.println("[loop] Entered loop()");
  unsigned long currentTime = millis();

  server.handleClient();

  if (currentTime - lastPollTime >= POLL_INTERVAL)
  {
    Serial.println("[loop] Polling actuators");
    pollActuators();
    lastPollTime = currentTime;
  }

  if (currentTime - lastSensorTime >= SENSOR_INTERVAL)
  {
    Serial.println("[loop] Sending sensor data");
    sendSensorData();
    lastSensorTime = currentTime;
  }

  if (currentTime - lastHeartbeatTime >= HEARTBEAT_INTERVAL)
  {
    Serial.println("[loop] Sending heartbeat");
    sendHeartbeat();
    lastHeartbeatTime = currentTime;
  }

  if (currentTime - lastSdSyncTime >= SD_SYNC_INTERVAL)
  {
    Serial.println("[loop] Uploading stored data");
    uploadStoredData();
    lastSdSyncTime = currentTime;
  }

  delay(100);
}

// ==================== ACTUATOR POLLING ====================
void pollActuators()
{
  Serial.println("[pollActuators] Polling actuators");
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[pollActuators] WiFi disconnected, skipping poll");
    return;
  }

  HTTPClient http;
  http.setTimeout(5000);

  String lightUrl = String(BACKEND_URL) + "/api/actuators/" + DEVICE_ID + ":light";

  http.begin(lightUrl);
  int httpCode = http.GET();

  if (httpCode == 200)
  {
    String payload = http.getString();

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error && !doc["state"].isNull())
    {
      if (doc["state"].is<bool>())
      {
        bool newState = doc["state"].as<bool>();
        if (newState != lightState)
        {
          lightState = newState;
          Serial.print("[pollActuators] Light: ");
          Serial.println(lightState ? "ON" : "OFF");
        }
      }
      else if (doc["state"].is<JsonObject>())
      {
        int timeSeconds = doc["state"]["time"].as<int>();
        unsigned long startTimeMs = doc["state"]["startTime"].as<unsigned long>();

        bool newState = false;
        if (timeSeconds > 0 && startTimeMs > 0)
        {
          unsigned long serverNow = getServerTime();
          if (serverNow > 0)
          {
            unsigned long endTime = startTimeMs + ((unsigned long)timeSeconds * 1000UL);
            newState = serverNow < endTime;
          }
        }

        if (newState != lightState)
        {
          lightState = newState;
          Serial.print("[pollActuators] Light (timer): ");
          Serial.println(lightState ? "ON" : "OFF");
        }
      }
    }
  }
  else if (httpCode != 404)
  {
    Serial.print("[pollActuators] Poll failed, code: ");
    Serial.println(httpCode);
  }

  http.end();

  pollActuator("humidifier1");
  pollActuator("humidifier2");
  pollActuator("humidifier3");
  pollActuator("heater");
  pollActuator("fan1");
  pollActuator("fan2");
  pollActuator("fan3");
  pollActuator("fan4");
  pollActuator("fan5");
}

void pollActuator(const char *actuator)
{
  Serial.print("[pollActuator] Polling actuator: ");
  Serial.println(actuator);

  HTTPClient http;
  http.setTimeout(5000);

  String url = String(BACKEND_URL) + "/api/actuators/" + DEVICE_ID + ":" + actuator;

  http.begin(url);
  int httpCode = http.GET();

  if (httpCode == 200)
  {
    String payload = http.getString();

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (!error && !doc["state"].isNull())
    {
      bool state = doc["state"].as<bool>();
      applyActuatorState(actuator, state);
      Serial.print("[pollActuator] ");
      Serial.print(actuator);
      Serial.print(" = ");
      Serial.println(state ? "ON" : "OFF");
    }
  }

  http.end();
}

void applyActuatorState(const char *actuator, bool state)
{
  Serial.print("[applyActuatorState] Applying actuator state: ");
  Serial.print(actuator);
  Serial.print(" -> ");
  Serial.println(state ? "ON" : "OFF");

  if (strcmp(actuator, "humidifier1") == 0)
    setHumidifier1(state);
  else if (strcmp(actuator, "humidifier2") == 0)
    setHumidifier2(state);
  else if (strcmp(actuator, "humidifier3") == 0)
    setHumidifier3(state);
  else if (strcmp(actuator, "heater") == 0)
    setHeater(state);
  else if (strcmp(actuator, "fan1") == 0)
    setFan1(state);
  else if (strcmp(actuator, "fan2") == 0)
    setFan2(state);
  else if (strcmp(actuator, "fan3") == 0)
    setFan3(state);
  else if (strcmp(actuator, "fan4") == 0)
    setFan4(state);
  else if (strcmp(actuator, "fan5") == 0)
    setFan5(state);
}

// ==================== SENSOR DATA ====================
void sendSensorData()
{
  Serial.println("[sendSensorData] Sending sensor data");
  collectAndProcessDrawer12();
  collectAndProcessDrawer3();
}

void collectAndProcessDrawer12()
{
  Serial.println("[collectAndProcessDrawer12] Collecting and processing Drawer 1/2 data");
  float humidityA = dhtA.readHumidity();
  float temperatureA = dhtA.readTemperature();
  float humidityB = dhtB.readHumidity();
  float temperatureB = dhtB.readTemperature();

  float humidity = NAN;
  float temperature = NAN;

  if (!isnan(humidityA) && !isnan(humidityB))
  {
    humidity = (humidityA + humidityB) / 2.0;
  }
  else if (!isnan(humidityA))
  {
    humidity = humidityA;
  }
  else if (!isnan(humidityB))
  {
    humidity = humidityB;
  }

  if (!isnan(temperatureA) && !isnan(temperatureB))
  {
    temperature = (temperatureA + temperatureB) / 2.0;
  }
  else if (!isnan(temperatureA))
  {
    temperature = temperatureA;
  }
  else if (!isnan(temperatureB))
  {
    temperature = temperatureB;
  }

  int soil1Raw = readSoil1();
  int soil2Raw = readSoil2();
  int soil3Raw = readSoil3();
  int moisture = (soil1Raw + soil2Raw + soil3Raw) / 3;
  moisture = map(moisture, 0, 26000, 0, 100);
  moisture = constrain(moisture, 0, 100);

  int ammoniaRaw = readMQ137();
  int ammonia = map(ammoniaRaw, 0, 26000, 0, 100);
  ammonia = constrain(ammonia, 0, 100);

  if (!isnan(humidity) && !isnan(temperature))
  {
    sendOrStoreSensorReading("Drawer 1", temperature, humidity, moisture, ammonia);
    sendOrStoreSensorReading("Drawer 2", temperature, humidity, moisture, ammonia);

    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("[collectAndProcessDrawer12] WiFi disconnected, running auto control for Drawer 1/2");
      autoControlDrawer12(temperature, humidity, moisture);
    }
  }
}

void collectAndProcessDrawer3()
{
  Serial.println("[collectAndProcessDrawer3] Collecting and processing Drawer 3 data");
  float humidity = dhtC.readHumidity();
  float temperature = dhtC.readTemperature();

  for (int attempt = 0; attempt < 3 && (isnan(humidity) || isnan(temperature)); attempt++)
  {
    delay(500);
    humidity = dhtC.readHumidity();
    temperature = dhtC.readTemperature();
  }

  if (!isnan(humidity) && !isnan(temperature))
  {
    sendOrStoreSensorReading("Drawer 3", temperature, humidity, -1, -1);

    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("[collectAndProcessDrawer3] WiFi disconnected, running auto control for Drawer 3");
      autoControlDrawer3(temperature, humidity);
    }
  }
}

void sendOrStoreSensorReading(const char *drawerName, float temperature, float humidity, int moisture, int ammonia)
{
  Serial.print("[sendOrStoreSensorReading] Processing reading for ");
  Serial.println(drawerName);

  if (WiFi.status() == WL_CONNECTED)
  {
    bool success = sendSensorReading(drawerName, temperature, humidity, moisture, ammonia);
    if (!success && sdAvailable)
    {
      Serial.println("[sendOrStoreSensorReading] Send failed, storing to SD");
      storeSensorToSD(drawerName, temperature, humidity, moisture, ammonia);
    }
  }
  else if (sdAvailable)
  {
    storeSensorToSD(drawerName, temperature, humidity, moisture, ammonia);
    Serial.print("[sendOrStoreSensorReading] Stored offline: ");
    Serial.println(drawerName);
  }
}

bool sendSensorReading(const char *drawerName, float temperature, float humidity, int moisture, int ammonia)
{
  Serial.print("[sendSensorReading] Sending sensor reading for ");
  Serial.println(drawerName);

  HTTPClient http;
  http.setTimeout(5000);
  String sensorUrl = String(BACKEND_URL) + "/api/sensor";

  http.begin(sensorUrl);
  http.addHeader("Content-Type", "application/json");

  JsonDocument doc;
  doc["macAddress"] = DEVICE_ID;
  doc["drawerName"] = drawerName;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  if (moisture >= 0)
    doc["moisture"] = moisture;
  if (ammonia >= 0)
    doc["ammonia"] = ammonia;

  String payload;
  serializeJson(doc, payload);

  int httpCode = http.POST(payload);
  http.end();

  if (httpCode == 200 || httpCode == 201)
  {
    Serial.print("[sendSensorReading] ");
    Serial.print(drawerName);
    Serial.println(" sensor data sent");
    return true;
  }
  else
  {
    Serial.print("[sendSensorReading] ");
    Serial.print(drawerName);
    Serial.print(" sensor send failed: ");
    Serial.println(httpCode);
    return false;
  }
}

// ==================== HEARTBEAT ====================
void sendHeartbeat()
{
  Serial.println("[sendHeartbeat] Sending heartbeat");
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("[sendHeartbeat] WiFi disconnected, skipping heartbeat");
    return;
  }

  HTTPClient http;
  http.setTimeout(5000);

  String heartbeatUrl = String(BACKEND_URL) + "/api/devices/" + DEVICE_ID + "/heartbeat";

  http.begin(heartbeatUrl);
  http.addHeader("Content-Type", "application/json");

  int httpCode = http.POST("{}");

  if (httpCode == 200)
  {
    Serial.println("[sendHeartbeat] Heartbeat sent - device online");
  }
  else if (httpCode == 404)
  {
    Serial.println("[sendHeartbeat] Device not registered. Register in app Settings.");
  }
  else
  {
    Serial.print("[sendHeartbeat] Heartbeat failed, code: ");
    Serial.println(httpCode);
  }

  http.end();
}

// ==================== HELPER ====================
void setActuatorState(const char *actuatorType, bool state)
{
  Serial.print("[setActuatorState] Setting actuator ");
  Serial.print(actuatorType);
  Serial.print(" to ");
  Serial.println(state ? "ON" : "OFF");

  if (WiFi.status() != WL_CONNECTED)
    return;

  HTTPClient http;
  http.setTimeout(5000);
  String url = String(BACKEND_URL) + "/api/actuators/" + DEVICE_ID + ":" + actuatorType;

  http.begin(url);
  http.addHeader("Content-Type", "application/json");

  JsonDocument doc;
  doc["state"] = state;

  String payload;
  serializeJson(doc, payload);

  int httpCode = http.POST(payload);

  if (httpCode == 200 || httpCode == 201)
  {
    Serial.print("[setActuatorState] Actuator ");
    Serial.print(actuatorType);
    Serial.print(" set to ");
    Serial.println(state ? "ON" : "OFF");
  }

  http.end();
}

// ==================== TIME HELPER ====================
unsigned long getServerTime()
{
  Serial.println("[getServerTime] Getting server time");
  HTTPClient http;
  http.setTimeout(2000);
  http.begin(String(BACKEND_URL) + "/api/time");
  int httpCode = http.GET();
  unsigned long serverTime = 0;

  if (httpCode == 200)
  {
    String payload = http.getString();
    JsonDocument doc;
    if (!deserializeJson(doc, payload))
    {
      serverTime = doc["now"].as<unsigned long>();
    }
  }

  http.end();
  Serial.print("[getServerTime] Server time: ");
  Serial.println(serverTime);
  return serverTime;
}

// ==================== MUX HELPERS ====================
void selectMuxChannel(uint8_t channel)
{
  Serial.print("[selectMuxChannel] Selecting mux channel: ");
  Serial.println(channel);
  digitalWrite(MUX_S0, channel & 0x01);
  digitalWrite(MUX_S1, (channel >> 1) & 0x01);
  digitalWrite(MUX_S2, (channel >> 2) & 0x01);
  digitalWrite(MUX_S3, (channel >> 3) & 0x01);
  delayMicroseconds(100);
}

int readMuxAnalog(uint8_t channel)
{
  Serial.print("[readMuxAnalog] Reading mux analog channel: ");
  Serial.println(channel);
  selectMuxChannel(channel);
  int value = analogRead(MUX_SIG);
  Serial.print("[readMuxAnalog] Value: ");
  Serial.println(value);
  return value;
}

// ==================== TCA9548A HELPER ====================
void selectTcaChannel(uint8_t channel)
{
  Serial.print("[selectTcaChannel] Selecting TCA channel: ");
  Serial.println(channel);
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// ==================== ADS1115 HELPERS ====================
int16_t readAds1Channel(uint8_t channel)
{
  Serial.print("[readAds1Channel] Reading ADS1 channel: ");
  Serial.println(channel);
  if (!ads1Available)
    return 0;
  int16_t value = ads1.readADC_SingleEnded(channel);
  Serial.print("[readAds1Channel] Value: ");
  Serial.println(value);
  return value;
}

int16_t readAds2Channel(uint8_t channel)
{
  Serial.print("[readAds2Channel] Reading ADS2 channel: ");
  Serial.println(channel);
  if (!ads2Available)
    return 0;
  int16_t value = ads2.readADC_SingleEnded(channel);
  Serial.print("[readAds2Channel] Value: ");
  Serial.println(value);
  return value;
}

// ==================== MCP23017 HELPERS ====================
void setMcpActuator(uint8_t pin, bool state)
{
  Serial.print("[setMcpActuator] Setting MCP actuator pin ");
  Serial.print(pin);
  Serial.print(" to ");
  Serial.println(state ? "HIGH" : "LOW");
  if (!mcpAvailable)
    return;
  mcp.digitalWrite(pin, state ? HIGH : LOW);
}

void setHumidifier1(bool state)
{
  Serial.print("[setHumidifier1] ");
  Serial.println(state ? "ON" : "OFF");
  setMcpActuator(MCP_HUMIDIFIER1, state);
}
void setHumidifier2(bool state)
{
  Serial.print("[setHumidifier2] ");
  Serial.println(state ? "ON" : "OFF");
  setMcpActuator(MCP_HUMIDIFIER2, state);
}
void setHumidifier3(bool state)
{
  Serial.print("[setHumidifier3] ");
  Serial.println(state ? "ON" : "OFF");
  setMcpActuator(MCP_HUMIDIFIER3, state);
}
void setHeater(bool state)
{
  Serial.print("[setHeater] ");
  Serial.println(state ? "ON" : "OFF");
  setMcpActuator(MCP_HEATER, state);
}
void setFan1(bool state)
{
  Serial.print("[setFan1] ");
  Serial.println(state ? "ON" : "OFF");
  setMcpActuator(MCP_FAN1, state);
}
void setFan2(bool state)
{
  Serial.print("[setFan2] ");
  Serial.println(state ? "ON" : "OFF");
  setMcpActuator(MCP_FAN2, state);
}
void setFan3(bool state)
{
  Serial.print("[setFan3] ");
  Serial.println(state ? "ON" : "OFF");
  setMcpActuator(MCP_FAN3, state);
}
void setFan4(bool state)
{
  Serial.print("[setFan4] ");
  Serial.println(state ? "ON" : "OFF");
  setMcpActuator(MCP_FAN4, state);
}
void setFan5(bool state)
{
  Serial.print("[setFan5] ");
  Serial.println(state ? "ON" : "OFF");
  setMcpActuator(MCP_FAN5, state);
}

// ==================== SENSOR READERS ====================
int readSoil1()
{
  Serial.println("[readSoil1]");
  return readAds1Channel(ADS_SOIL1);
}
int readSoil2()
{
  Serial.println("[readSoil2]");
  return readAds1Channel(ADS_SOIL2);
}
int readSoil3()
{
  Serial.println("[readSoil3]");
  return readAds1Channel(ADS_SOIL3);
}
int readMQ137()
{
  Serial.println("[readMQ137]");
  return readAds1Channel(ADS_MQ137);
}

// ==================== SD CARD STORAGE ====================
void storeSensorToSD(const char *drawerName, float temperature, float humidity, int moisture, int ammonia)
{
  Serial.print("[storeSensorToSD] Storing sensor data to SD for ");
  Serial.println(drawerName);

  if (!sdAvailable)
    return;

  File file = SD.open(SD_DATA_FILE, FILE_APPEND);
  if (!file)
  {
    Serial.println("[storeSensorToSD] Failed to open SD file for writing");
    return;
  }

  JsonDocument doc;
  doc["macAddress"] = DEVICE_ID;
  doc["drawerName"] = drawerName;
  doc["temperature"] = temperature;
  doc["humidity"] = humidity;
  if (moisture >= 0)
    doc["moisture"] = moisture;
  if (ammonia >= 0)
    doc["ammonia"] = ammonia;
  doc["timestamp"] = millis();

  String line;
  serializeJson(doc, line);
  file.println(line);
  file.close();
  Serial.println("[storeSensorToSD] Data stored");
}

void uploadStoredData()
{
  Serial.println("[uploadStoredData] Uploading stored data from SD");
  if (!sdAvailable || WiFi.status() != WL_CONNECTED)
    return;
  if (!SD.exists(SD_DATA_FILE))
    return;

  File file = SD.open(SD_DATA_FILE, FILE_READ);
  if (!file)
    return;

  String tempPath = "/temp_data.json";
  File tempFile = SD.open(tempPath, FILE_WRITE);

  int uploaded = 0;
  int failed = 0;

  while (file.available())
  {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() == 0)
      continue;

    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, line);
    if (error)
      continue;

    HTTPClient http;
    http.setTimeout(5000);
    http.begin(String(BACKEND_URL) + "/api/sensor");
    http.addHeader("Content-Type", "application/json");

    int httpCode = http.POST(line);
    http.end();

    if (httpCode == 200 || httpCode == 201)
    {
      uploaded++;
    }
    else
    {
      if (tempFile)
        tempFile.println(line);
      failed++;
    }
  }

  file.close();
  if (tempFile)
    tempFile.close();

  SD.remove(SD_DATA_FILE);
  if (failed > 0 && SD.exists(tempPath))
  {
    SD.rename(tempPath, SD_DATA_FILE);
  }
  else
  {
    SD.remove(tempPath);
  }

  if (uploaded > 0)
  {
    Serial.print("[uploadStoredData] Uploaded ");
    Serial.print(uploaded);
    Serial.println(" stored readings");
  }
  if (failed > 0)
  {
    Serial.print("[uploadStoredData] Failed to upload ");
    Serial.print(failed);
    Serial.println(" readings (kept for retry)");
  }
}

int getStoredDataCount()
{
  Serial.println("[getStoredDataCount] Counting stored data on SD");
  if (!sdAvailable || !SD.exists(SD_DATA_FILE))
    return 0;

  File file = SD.open(SD_DATA_FILE, FILE_READ);
  if (!file)
    return 0;

  int count = 0;
  while (file.available())
  {
    String line = file.readStringUntil('\n');
    if (line.length() > 0)
      count++;
  }
  file.close();
  Serial.print("[getStoredDataCount] Count: ");
  Serial.println(count);
  return count;
}

// ==================== OFFLINE AUTO CONTROL ====================
void autoControlDrawer12(float temperature, float humidity, int moisture)
{
  Serial.println("[autoControlDrawer12] Auto controlling Drawer 1/2");
  bool fanOn = false;
  bool heaterOn = false;
  bool humidifierOn = false;

  if (temperature > TEMP_OPTIMAL_HIGH)
  {
    fanOn = true;
    heaterOn = false;
  }
  else if (temperature < TEMP_OPTIMAL_LOW)
  {
    heaterOn = true;
    fanOn = false;
  }

  if (temperature > TEMP_MAX)
  {
    fanOn = true;
    heaterOn = false;
  }
  else if (temperature < TEMP_MIN)
  {
    heaterOn = true;
    fanOn = false;
  }

  if (humidity < HUMIDITY_OPTIMAL_LOW || humidity < HUMIDITY_MIN)
  {
    humidifierOn = true;
  }
  else if (humidity > HUMIDITY_OPTIMAL_HIGH || humidity > HUMIDITY_MAX)
  {
    humidifierOn = false;
    fanOn = true;
  }

  if (moisture < MOISTURE_OPTIMAL_LOW || moisture < MOISTURE_MIN)
  {
    humidifierOn = true;
  }

  setFan1(fanOn);
  setFan2(fanOn);
  setFan3(fanOn);
  setFan4(fanOn);
  setHeater(heaterOn);
  setHumidifier1(humidifierOn);
  setHumidifier2(humidifierOn);

  Serial.println("[autoControlDrawer12] Auto control D1/D2:");
  Serial.print("  Temp=");
  Serial.print(temperature);
  Serial.print(" Hum=");
  Serial.print(humidity);
  Serial.print(" Moist=");
  Serial.println(moisture);
  Serial.print("  Fan=");
  Serial.print(fanOn ? "ON" : "OFF");
  Serial.print(" Heater=");
  Serial.print(heaterOn ? "ON" : "OFF");
  Serial.print(" Humidifier=");
  Serial.println(humidifierOn ? "ON" : "OFF");
}

void autoControlDrawer3(float temperature, float humidity)
{
  Serial.println("[autoControlDrawer3] Auto controlling Drawer 3");
  bool fanOn = false;
  bool humidifierOn = false;

  if (temperature > TEMP_OPTIMAL_HIGH || temperature > TEMP_MAX)
  {
    fanOn = true;
  }

  if (humidity < HUMIDITY_OPTIMAL_LOW || humidity < HUMIDITY_MIN)
  {
    humidifierOn = true;
  }
  else if (humidity > HUMIDITY_OPTIMAL_HIGH || humidity > HUMIDITY_MAX)
  {
    humidifierOn = false;
    fanOn = true;
  }

  setFan5(fanOn);
  setHumidifier3(humidifierOn);

  Serial.println("[autoControlDrawer3] Auto control D3:");
  Serial.print("  Temp=");
  Serial.print(temperature);
  Serial.print(" Hum=");
  Serial.println(humidity);
  Serial.print("  Fan=");
  Serial.print(fanOn ? "ON" : "OFF");
  Serial.print(" Humidifier=");
  Serial.println(humidifierOn ? "ON" : "OFF");
}

// ==================== WEB SERVER ====================
void setupWebServer()
{
  Serial.println("[setupWebServer] Setting up web server");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/sdcard/data", HTTP_GET, handleGetSdData);
  server.on("/sdcard/clear", HTTP_POST, handleClearSdData);
  server.on("/sdcard/sync", HTTP_POST, handleSyncSdData);
  server.enableCORS(true);
  server.begin();
  Serial.print("[setupWebServer] Web server started at http://");
  Serial.println(WiFi.localIP());
}

void handleRoot()
{
  Serial.println("[handleRoot] Handling root web request");
  String html = "<!DOCTYPE html><html><head><title>BSFly IoT</title>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<style>body{font-family:sans-serif;padding:20px;max-width:600px;margin:0 auto}";
  html += "h1{color:#333}.card{background:#f5f5f5;padding:15px;margin:10px 0;border-radius:8px}";
  html += "button{background:#007bff;color:white;border:none;padding:10px 20px;border-radius:5px;margin:5px;cursor:pointer}";
  html += "button:hover{background:#0056b3}.danger{background:#dc3545}.danger:hover{background:#c82333}</style></head>";
  html += "<body><h1>BSFly IoT Device</h1>";
  html += "<div class='card'><strong>Device ID:</strong> " + DEVICE_ID + "</div>";
  html += "<div class='card'><strong>IP Address:</strong> " + WiFi.localIP().toString() + "</div>";
  html += "<div class='card'><strong>SD Card:</strong> " + String(sdAvailable ? "Available" : "Not found") + "</div>";
  html += "<div class='card'><strong>Stored Readings:</strong> " + String(getStoredDataCount()) + "</div>";
  html += "<h2>Actions</h2>";
  html += "<button onclick=\"fetch('/sdcard/sync',{method:'POST'}).then(r=>r.json()).then(d=>alert(d.message))\">Sync to Cloud</button>";
  html += "<button onclick=\"window.location='/sdcard/data'\">Download Data</button>";
  html += "<button class='danger' onclick=\"if(confirm('Clear all stored data?'))fetch('/sdcard/clear',{method:'POST'}).then(r=>r.json()).then(d=>alert(d.message))\">Clear Data</button>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleStatus()
{
  Serial.println("[handleStatus] Handling /status web request");
  JsonDocument doc;
  doc["deviceId"] = DEVICE_ID;
  doc["ip"] = WiFi.localIP().toString();
  doc["sdAvailable"] = sdAvailable;
  doc["storedCount"] = getStoredDataCount();
  doc["wifiConnected"] = WiFi.status() == WL_CONNECTED;
  doc["uptime"] = millis() / 1000;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleGetSdData()
{
  Serial.println("[handleGetSdData] Handling /sdcard/data web request");
  if (!sdAvailable)
  {
    server.send(503, "application/json", "{\"error\":\"SD card not available\"}");
    return;
  }

  if (!SD.exists(SD_DATA_FILE))
  {
    server.send(200, "application/json", "{\"readings\":[]}");
    return;
  }

  File file = SD.open(SD_DATA_FILE, FILE_READ);
  if (!file)
  {
    server.send(500, "application/json", "{\"error\":\"Failed to open file\"}");
    return;
  }

  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "application/json", "");
  server.sendContent("{\"deviceId\":\"" + DEVICE_ID + "\",\"readings\":[");

  bool first = true;
  while (file.available())
  {
    String line = file.readStringUntil('\n');
    line.trim();
    if (line.length() > 0)
    {
      if (!first)
        server.sendContent(",");
      server.sendContent(line);
      first = false;
    }
  }

  server.sendContent("]}");
  file.close();
}

void handleClearSdData()
{
  Serial.println("[handleClearSdData] Handling /sdcard/clear web request");
  if (!sdAvailable)
  {
    server.send(503, "application/json", "{\"error\":\"SD card not available\"}");
    return;
  }

  if (SD.exists(SD_DATA_FILE))
  {
    SD.remove(SD_DATA_FILE);
  }

  server.send(200, "application/json", "{\"message\":\"Data cleared\",\"success\":true}");
}

void handleSyncSdData()
{
  Serial.println("[handleSyncSdData] Handling /sdcard/sync web request");
  if (!sdAvailable)
  {
    server.send(503, "application/json", "{\"error\":\"SD card not available\"}");
    return;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    server.send(503, "application/json", "{\"error\":\"WiFi not connected\"}");
    return;
  }

  int beforeCount = getStoredDataCount();
  uploadStoredData();
  int afterCount = getStoredDataCount();
  int uploaded = beforeCount - afterCount;

  JsonDocument doc;
  doc["message"] = "Sync complete";
  doc["uploaded"] = uploaded;
  doc["remaining"] = afterCount;
  doc["success"] = true;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}
