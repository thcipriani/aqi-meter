#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h> //Included to make GFX lib happy
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>

/* -------------------- WiFi Config ----------------- */


/* -------------------- Hardware Config ----------------- */
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C
#define BME_ADDRESS    0x76

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_BME280 bme;
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_pressure = bme.getPressureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

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

void setup() {
  Serial.begin(115200);

  setupDisplay();

  setupBME();
}

void loop() {
  sensors_event_t temp_event, pressure_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_pressure->getEvent(&pressure_event);
  bme_humidity->getEvent(&humidity_event);

  render(makeBMEOutput(temp_event, pressure_event, humidity_event));
  
  delay(1000);
}