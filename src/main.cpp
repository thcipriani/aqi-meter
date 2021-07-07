/**
 * AQI Meter
 * =========
 * Copyright 2021 Tyler Cipriani <tyler@tylercipriani.com>
 * License: GPLv3
 * 
 * Much of this is work is from:
 *   - BSD License: <https://github.com/SuperHouse/AQS>
 */
#define PROJECT "nodemcu-aqi"
#define VERSION "0.0.1"

/**
 * Only thing in "secrets.h":
 *   - ssid
 *   - password
 *   - mqttBroker
 */
#include "secrets.h"

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

/* -------------------- Hardware Config ----------------- */
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
#define BME_ADDRESS    0x76

#define PMS_RX_PIN  D7
#define PMS_TX_PIN  D8
#define PMS_BAUD  9600

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

SoftwareSerial pmsSerial(PMS_RX_PIN, PMS_TX_PIN);
PMS pms(pmsSerial);
PMS::DATA pmsData;

/* -------------------- WiFi Config ----------------- */
WiFiClient espClient;
PubSubClient client(espClient);

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
  display.println(" Tyler's NEAT MONITOR!!");
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

  bme_temp->printSensorDetails();
  bme_pressure->printSensorDetails();
  bme_humidity->printSensorDetails();
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

String makeBMEOutput(
  sensors_event_t temp_event,
  sensors_event_t pressure_event,
  sensors_event_t humidity_event) {

  char allStr[42], tempStr[15], humStr[12], pressStr[12];
  sprintf(tempStr, "Temp: %02.02f*F", cToF(temp_event.temperature));
  sprintf(humStr, "Hum: %02.02f%%", humidity_event.relative_humidity);
  sprintf(pressStr, "hPa: %02.02f", pressure_event.pressure);
  sprintf(allStr, "%s\n%s\n%s\n", tempStr, humStr, pressStr);
   
  return allStr;
}

bool mqttConnected() {
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
  if (mqttConnected()) {
    return;
  }
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
  sensors_event_t temp_event, pressure_event, humidity_event;
  int loopCount = 0;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  if (loopCount % 10000 == 0) {
    pms.wakeUp();
    pms.readUntil(pmsData);
    pms.sleep();
  }

  Serial.println(pmsData.PM_AE_UG_2_5);
  render(makeBMEOutput(temp_event, pressure_event, humidity_event));
 
  delay(1000);
}