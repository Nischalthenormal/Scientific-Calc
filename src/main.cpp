#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>

// ================= PINS =================

#define TFT_BL   3

#define TP_SDA   4
#define TP_SCL   5
#define TP_INT   6
#define TP_RST   7

// CST328 I2C address
#define CST328_ADDR 0x1A

// ================= DISPLAY =================

TFT_eSPI tft = TFT_eSPI();

// ================= BACKLIGHT =================

void setupBacklight()
{
    ledcAttach(TFT_BL, 5000, 8);
    ledcWrite(TFT_BL, 255);
}

// ================= CST328 =================

void touchReset()
{
    pinMode(TP_RST, OUTPUT);

    digitalWrite(TP_RST, LOW);
    delay(10);

    digitalWrite(TP_RST, HIGH);
    delay(100);
}

void setupTouch()
{
    pinMode(TP_INT, INPUT);

    touchReset();

    Wire.begin(TP_SDA, TP_SCL);

    Serial.println("Touch I2C started");

    // Scan I2C bus
    Serial.println("I2C scan:");

    for (uint8_t address = 1; address < 127; address++)
    {
        Wire.beginTransmission(address);

        if (Wire.endTransmission() == 0)
        {
            Serial.printf(
                "  Found device at 0x%02X\n",
                address
            );
        }
    }
}

// ================= DISPLAY TEST =================

void displayTest()
{
    tft.init();

    tft.setRotation(1);

    tft.fillScreen(TFT_BLACK);

    delay(500);

    tft.fillScreen(TFT_RED);
    delay(500);

    tft.fillScreen(TFT_GREEN);
    delay(500);

    tft.fillScreen(TFT_BLUE);
    delay(500);

    tft.fillScreen(TFT_WHITE);
    delay(500);

    tft.fillScreen(TFT_BLACK);

    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    tft.drawString(
        "ESP32-S3 TFT TEST",
        20,
        30,
        4
    );

    tft.drawString(
        "ST7789T3",
        20,
        80,
        4
    );

    tft.drawString(
        "CST328 TOUCH",
        20,
        130,
        4
    );

    tft.drawString(
        "READY",
        20,
        180,
        4
    );
}

// ================= SETUP =================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();
    Serial.println("==============================");
    Serial.println(" ESP32-S3 TFT + TOUCH TEST");
    Serial.println("==============================");

    setupBacklight();

    displayTest();

    setupTouch();

    Serial.println();
    Serial.println("Display initialized.");
    Serial.println("Touch controller scan complete.");
}

// ================= LOOP =================

void loop()
{
    // Touch implementation comes next.
    // For now we're only verifying:
    // 1. TFT
    // 2. Backlight PWM
    // 3. CST328 I2C communication

    delay(100);
}