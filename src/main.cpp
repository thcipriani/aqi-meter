/**
 * AQI Meter
 * =========
 * Copyright 2021 Tyler Cipriani <tyler@tylercipriani.com>
 * License: GPLv3
 *
 * Much of this is work is from:
 *   - BSD License: <https://github.com/SuperHouse/AQS>
 */
#define PROJECT "Tyler's AQI Thingy"
#define VERSION "0.0.1"

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_I2CDevice.h> //Included to make GFX lib happy
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include "PMS.h"

// Stolen from: <https://steve.fi/hardware/d1-pub-sub/>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

/* -------------------- Hardware ----------------- */
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
#define BME_ADDRESS    0x76

#define PMS_RX_PIN  D7
#define PMS_TX_PIN  D8
#define PMS_BAUD  9600

/**
 * In "secrets.h"
 *   - ssid
 *   - password
 *   - mqttBroker
 */
#include "secrets.h"

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BME280 bme;
Adafruit_Sensor *bmeTemp = bme.getTemperatureSensor();
Adafruit_Sensor *bmePressure = bme.getPressureSensor();
Adafruit_Sensor *bmeHumidity = bme.getHumiditySensor();

/* -------------------- WiFi -------------------- */
WiFiClient espClient;
PubSubClient client(espClient);

/* -------------------- PMS5003 ----------------- */
SoftwareSerial pmsSerial(PMS_RX_PIN, PMS_TX_PIN);
PMS pms(pmsSerial);
PMS::DATA pmsData;

enum pmsState { PMS_ASLEEP, PMS_WAKING, PMS_READY };
pmsState pmsCurrentState;
uint64_t pmsTimeLastRead = 0;
uint64_t pmsWarmupTime = 30;      // seconds for warmup
uint64_t pmsReportInterval = 120; // seconds between PMS Reads
bool pmsReadingTaken = false;

uint64_t mqttReportInterval = 120;
uint64_t mqttTimeLastReport = 0;

void connectMQTT() {
  Serial.print("Attempting MQTT connection...");

  char mqttClientId[20];
  sprintf(mqttClientId, "esp8266-%x", ESP.getChipId());

  // Loop until we're reconnected
  while (!client.connected()) {
      if (client.connect(mqttClientId, mqttUsername, mqttPassword)) {
        Serial.println("connected.");

        // Once connected, publish an announcement
        client.publish("meta", "We're connected");
      } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
        // Wait 5 seconds before retrying
        delay(5000);
      }
  }
}

void setupMQTT() {
  client.setServer(mqttBroker, 1883);
  client.setBufferSize(255);
  // Could also use client.setCallback to do something when I publish a message...
  connectMQTT();
}

void setupWifi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Set the hostname
  WiFi.hostname(PROJECT);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}

/**
 * Setup display
 *
 * Used by the setup loop to initialize the SSD1306 OLED display
 */
void setupDisplay() {
  Serial.print(F("Display setup..."));
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  // Clear the buffer.
	display.clearDisplay();

  display.begin();
  display.clearDisplay();
  display.setTextWrap(false);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(PROJECT);
  display.print("\nv: ");
  display.println(VERSION);
  display.println("Connecting...");
  display.display();
  Serial.println(F("DONE!"));
}

/**
 * Used in setup loop to initialize BME280
 *
 * Temp/pressure/humidity sensor
 */
void setupBME() {
  Serial.print(F("Setup BME280..."));

  if (! bme.begin(BME_ADDRESS)) {
    Serial.println("");
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (1) delay(10);
  }

  bmeTemp->printSensorDetails();
  bmePressure->printSensorDetails();
  bmeHumidity->printSensorDetails();
  Serial.println(F("DONE!"));
}

float cToF(float tempC) {
  return (tempC * 1.8) + 32.0;
}

void render(String output) {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextWrap(false);
  display.print(output);
  display.display();
}

void updateMQTT(
  sensors_event_t tempEvent,
  sensors_event_t pressureEvent,
  sensors_event_t humidityEvent,
  PMS::DATA pmsSensorData
  ) {
    if (! pmsReadingTaken) {
      return;
    }
    uint64_t timeNow = millis();
    if (timeNow - mqttTimeLastReport < mqttReportInterval * 1000) {
      return;
    }
    mqttTimeLastReport = timeNow;

    char mqttBMEMessageBuffer[255];
    char mqttBMETopic[50];
    sprintf(mqttBMETopic, "esp8266/%x/BME", ESP.getChipId());
    sprintf(
      mqttBMEMessageBuffer,
      R"BME({
        "Temp": %02.02f,
        "humidity": %02.02f,
        "hPa": %02.02f
      })BME",
      cToF(tempEvent.temperature),           // Temp in F
      humidityEvent.relative_humidity,
      pressureEvent.pressure
    );
    
    Serial.print("MQTT BME message: ");
    Serial.println(mqttBMEMessageBuffer);
    client.publish(mqttBMETopic, mqttBMEMessageBuffer);

    char mqttPMSTopic[50];
    char mqttPMSMessageBuffer[255];
    sprintf(mqttPMSTopic, "esp8266/%x/PMS5003", ESP.getChipId());

    sprintf(
      mqttPMSMessageBuffer,
      R"PMS({
        "CF1": %i,
        "CF2.5": %i,
        "CF10": %i,
        "PM1": %i,
        "PM2.5": %i,
        "PM10": %i,
        "PB0_3": %i,
        "PB0_5": %i,
        "PB1_0": %i,
        "PB2_5": %i
      })PMS",
      pmsSensorData.PM_SP_UG_1_0,            // Standard Particle calibration pm1.0 reading
      pmsSensorData.PM_SP_UG_2_5,            // Standard Particle calibration pm2.5 reading
      pmsSensorData.PM_SP_UG_10_0,           // Standard Particle calibration pm10 reading
      pmsSensorData.PM_AE_UG_1_0,            // Atmospheric Environment pm1.0 reading
      pmsSensorData.PM_AE_UG_2_5,            // Atmospheric Environment pm2.5 reading
      pmsSensorData.PM_AE_UG_10_0,           // Atmospheric Environment pm10.0 reading
      pmsSensorData.PM_TOTALPARTICLES_0_3,   // Particles Per Deciliter pm0.3 reading
      pmsSensorData.PM_TOTALPARTICLES_0_5,   // Particles Per Deciliter pm0.5 reading
      pmsSensorData.PM_TOTALPARTICLES_1_0,   // Particles Per Deciliter pm1.0 reading
      pmsSensorData.PM_TOTALPARTICLES_2_5    // Particles Per Deciliter pm2.5 reading
    );

    Serial.print("MQTT PMS message: ");
    Serial.println(mqttPMSMessageBuffer);
    client.publish(mqttPMSTopic, mqttPMSMessageBuffer);
}

bool pmsTimeForReading(uint64_t timeNow) {
  return timeNow - pmsTimeLastRead >=
      pmsReportInterval * 1000 - pmsWarmupTime * 1000;
}

bool pmsAwake(uint64_t timeNow) {
  return timeNow - pmsTimeLastRead >= pmsWarmupTime * 1000;
}

void readPMS() {
  uint64_t timeNow = millis();
  if (pmsCurrentState == PMS_ASLEEP && pmsTimeForReading(timeNow)) {

    Serial.print("Waking PMS5002...");
    pms.wakeUp();
    pmsCurrentState = PMS_WAKING;
    pmsTimeLastRead = timeNow;

    Serial.print("DONE!");
    Serial.print("(time: ");
    Serial.print(timeNow);
    Serial.println(")");
  }

  if (pmsCurrentState == PMS_WAKING && pmsAwake(timeNow)) {
    Serial.print("PMS5002 is ready...");

    pmsCurrentState = PMS_READY;

    Serial.print("DONE!");
    Serial.print("(time: ");
    Serial.print(timeNow);
    Serial.println(")");
  }

  if (pmsCurrentState == PMS_READY) {
    Serial.print("Reading PMS5002...");

    pms.readUntil(pmsData);
    pms.sleep();
    pmsReadingTaken = true;
    pmsCurrentState = PMS_ASLEEP;
    pmsTimeLastRead = timeNow;

    Serial.print("DONE!");
    Serial.print("(time: ");
    Serial.print(timeNow);
    Serial.println(")");
    Serial.println(pmsData.PM_AE_UG_2_5);
  }
}

String makeBMEOutput(
  sensors_event_t tempEvent,
  sensors_event_t pressureEvent,
  sensors_event_t humidityEvent) {

  char allStr[42], tempStr[15], humStr[12], pressStr[12];
  sprintf(tempStr, "Temp: %02.02f*F", cToF(tempEvent.temperature));
  sprintf(humStr, "Hum: %02.02f%%", humidityEvent.relative_humidity);
  sprintf(pressStr, "hPa: %02.02f", pressureEvent.pressure);
  sprintf(allStr, "%s\n%s\n%s\n", tempStr, humStr, pressStr);

  return allStr;
}

String makePMSOutput() {
  if (! pmsReadingTaken) {
    return "Warming up AQI...";
  }

  char pmsStr[16];
  sprintf(pmsStr, "PM2.5: %i", pmsData.PM_AE_UG_2_5);
  return pmsStr;
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.print("Air Quality Sensor starting up, v");
  Serial.println(VERSION);

  pmsSerial.begin(PMS_BAUD);
  pms.passiveMode();
  delay(100);
  setupDisplay();
  setupBME();
  setupWifi();
  setupMQTT();
}

void loop() {
  String output;

  // Reconnect wifi if needs be
  if (WiFi.status() != WL_CONNECTED) {
    setupWifi();
  }

  // Reconnect mqtt if needs be
  if (! client.connected()) {
    connectMQTT();
  }

  // No fucking idea, copy-pasta
  client.loop();

  // Get Temp, Pressure, Humidity
  sensors_event_t tempEvent, pressureEvent, humidityEvent;
  bmeTemp->getEvent(&tempEvent);
  bmePressure->getEvent(&pressureEvent);
  bmeHumidity->getEvent(&humidityEvent);

  readPMS();

  output = makeBMEOutput(tempEvent, pressureEvent, humidityEvent);
  output += makePMSOutput();

  updateMQTT(tempEvent, pressureEvent, humidityEvent, pmsData);
  render(output);

  delay(1000);
}
