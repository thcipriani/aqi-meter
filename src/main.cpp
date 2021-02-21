#include <Arduino.h>
#include <Adafruit_I2CDevice.h> //Included to make GFX lib happy
#include <Adafruit_SSD1306.h>
#include <Wire.h>

/* -------------------- WiFi Config ----------------- */


/* -------------------- Hardware Config ----------------- */
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);

  // Clear the buffer.
	display.clearDisplay();

  display.begin();
  display.clearDisplay();
  display.setTextWrap(false);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.println(" Particulate Matter");
  display.println(" scroll");
  display.println("this screeen");
  display.display();
  display.startscrollright(0x00, 0x00);
}

void loop() {
  delay(1000);
  Serial.println("FUCKKK");
}