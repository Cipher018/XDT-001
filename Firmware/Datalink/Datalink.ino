
// --- Required Library Inclusions ---
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <RF24.h>
#include <SPI.h>
#include <XPT2046_Touchscreen.h>
#include <nRF24L01.h>

// --- Pin Definitions ---

// SPI Bus pins:
// Typically (on ESP32): MOSI: 23, MISO: 19, SCK: 18

// RF Module 1 (Plane)
#define RF1_CE_PIN 16
#define RF1_CSN_PIN 4

// RF Module 2 (Quadcopter)
#define RF2_CE_PIN 5
#define RF2_CSN_PIN 17

// RF Module 3 (Car)
#define RF3_CE_PIN 21
#define RF3_CSN_PIN 22

// TFT Display (ILI9341)
#define TFT_CS_PIN 26
#define TFT_DC_PIN 14
#define TFT_RST_PIN 27

// Touch Screen (XPT2046)
#define TS_CS_PIN 13
#define TS_IRQ_PIN 12

// --- Device Instances ---

// radio
RF24 radio1(RF1_CE_PIN, RF1_CSN_PIN);
RF24 radio2(RF2_CE_PIN, RF2_CSN_PIN);
RF24 radio3(RF3_CE_PIN, RF3_CSN_PIN);

// Touch screen
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS_PIN, TFT_DC_PIN, TFT_RST_PIN);
XPT2046_Touchscreen touch(TS_CS_PIN, TS_IRQ_PIN);

void setup() {
  Serial.begin(115200);

  // 1. Initialize Display
  tft.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(10, 10);
  tft.println("Datalink Multi-RF System");

  // 2. Initialize Touch Screen
  if (!touch.begin()) {
    Serial.println(F("Touch controller error!"));
  } else {
    touch.setRotation(1);
    Serial.println(F("Touch initialized."));
  }

  // 3. Initialize nRF24L01 Radio 1
  if (!radio1.begin()) {
    Serial.println(F("Radio 1 error!"));
    tft.setTextColor(ILI9341_RED);
    tft.println("RF1 Error!");
  } else {
    radio1.setPALevel(RF24_PA_LOW);
    radio1.openWritingPipe(0xF0F0F0F0E1LL);
    radio1.stopListening();
    Serial.println(F("Radio 1 initialized."));
    tft.setTextColor(ILI9341_GREEN);
    tft.println("RF1 Ready.");
  }

  // 4. Initialize nRF24L01 Radio 2
  if (!radio2.begin()) {
    Serial.println(F("Radio 2 error!"));
    tft.setTextColor(ILI9341_RED);
    tft.println("RF2 Error!");
  } else {
    radio2.setPALevel(RF24_PA_LOW);
    radio2.openWritingPipe(0xF0F0F0F0E2LL);
    radio2.stopListening();
    Serial.println(F("Radio 2 initialized."));
    tft.setTextColor(ILI9341_GREEN);
    tft.println("RF2 Ready.");
  }

  // 5. Initialize nRF24L01 Radio 3
  if (!radio3.begin()) {
    Serial.println(F("Radio 3 error!"));
    tft.setTextColor(ILI9341_RED);
    tft.println("RF3 Error!");
  } else {
    radio3.setPALevel(RF24_PA_LOW);
    radio3.openWritingPipe(0xF0F0F0F0E3LL);
    radio3.stopListening();
    Serial.println(F("Radio 3 initialized."));
    tft.setTextColor(ILI9341_GREEN);
    tft.println("RF3 Ready.");
  }
}

void loop() {
  // Handlers for all 3 radios and touch...
}
