#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>

// Keep these pins aligned with the existing TFT project configuration.
#define TFT_BL   3
#define TP_SDA   4
#define TP_SCL   5
#define TP_INT   6
#define TP_RST   7

#define CST328_ADDR 0x1A
#define CST328_REG_TOUCH 0xD000
#define CST328_REG_COUNT 0xD005
#define I2C_HZ 400000

static constexpr int DISPLAY_W = 240;
static constexpr int DISPLAY_H = 320;

TFT_eSPI tft;
bool touchReady = false;

void setupBacklight() {
  ledcAttach(TFT_BL, 5000, 8);
  ledcWrite(TFT_BL, 255);
}

void touchReset() {
  pinMode(TP_RST, OUTPUT);
  digitalWrite(TP_RST, HIGH);
  delay(50);
  digitalWrite(TP_RST, LOW);
  delay(5);
  digitalWrite(TP_RST, HIGH);
  delay(50);
}

bool i2cRead(uint8_t address, uint16_t reg, uint8_t *data, uint8_t length) {
  Wire.beginTransmission(address);
  Wire.write(static_cast<uint8_t>(reg >> 8));
  Wire.write(static_cast<uint8_t>(reg & 0xFF));
  if (Wire.endTransmission(true) != 0) return false;

  const uint8_t received = Wire.requestFrom(address, length);
  if (received != length) {
    while (Wire.available()) Wire.read();
    return false;
  }
  for (uint8_t i = 0; i < length; ++i) data[i] = Wire.read();
  return true;
}

// This reproduces the working driver's I2C transaction and packed 12-bit
// point decoding. Reading the report is destructive because the count is
// cleared to acknowledge it, so call this once per loop iteration.
bool readRawTouch(uint16_t &x, uint16_t &y) {
  uint8_t count = 0;
  if (!i2cRead(CST328_ADDR, CST328_REG_COUNT, &count, 1)) return false;
  if ((count & 0x0F) == 0) return false;

  uint8_t report[27] = {};
  if (!i2cRead(CST328_ADDR, CST328_REG_TOUCH, report, sizeof(report))) return false;

  // The report begins at buf[1] in the original working routine, hence
  // its first coordinate bytes are offsets 1, 2, and 3 in this buffer.
  const uint16_t rawX = (static_cast<uint16_t>(report[1]) << 4) | (report[3] >> 4);
  const uint16_t rawY = (static_cast<uint16_t>(report[2]) << 4) | (report[3] & 0x0F);

  // This panel's CST328 axes are swapped; preserve the known-working order.
  x = rawY;
  y = rawX;

  Wire.beginTransmission(CST328_ADDR);
  Wire.write(static_cast<uint8_t>(CST328_REG_COUNT >> 8));
  Wire.write(static_cast<uint8_t>(CST328_REG_COUNT & 0xFF));
  Wire.write(static_cast<uint8_t>(0));
  Wire.endTransmission(true);
  return true;
}

int clampCoordinate(int value, int limit) {
  return value < 0 ? 0 : (value >= limit ? limit - 1 : value);
}

bool readPortraitTouch(int &x, int &y) {
  uint16_t rx, ry;
  if (!readRawTouch(rx, ry)) return false;

  // The old calibration maps into the 320x240 landscape orientation.
  const float landscapeX = 0.973913038f * rx - 0.031239455f * ry + 7.33205278f;
  const float landscapeY = -0.046093319f * rx - 0.903280217f * ry + 235.615557f;
  const int lx = clampCoordinate(static_cast<int>(landscapeX + 0.5f), 320);
  const int ly = clampCoordinate(static_cast<int>(landscapeY + 0.5f), 240);

  // Inverse of TFT_eSPI rotation 1: map the calibrated landscape point
  // into the project's portrait 240x320 coordinate space.
  x = clampCoordinate(ly, DISPLAY_W);
  y = clampCoordinate(DISPLAY_H - 1 - lx, DISPLAY_H);
  return true;
}

void setupTouch() {
  pinMode(TP_INT, INPUT);
  touchReset();
  if (!Wire.begin(TP_SDA, TP_SCL, I2C_HZ)) {
    Serial.println("Touch I2C start failed");
    return;
  }

  Wire.beginTransmission(CST328_ADDR);
  touchReady = (Wire.endTransmission() == 0);
  Serial.println(touchReady ? "CST328 found at 0x1A" : "CST328 not found at 0x1A");
}

void displayTest() {
  tft.init();
  tft.setRotation(0);  // Portrait: 240x320, matching the configured panel size.
  tft.fillScreen(TFT_BLACK);
  for (uint16_t color : {TFT_RED, TFT_GREEN, TFT_BLUE, TFT_WHITE}) {
    tft.fillScreen(color);
    delay(250);
  }

  tft.fillScreen(TFT_BLACK);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.drawString("ESP32-S3 TFT TEST", DISPLAY_W / 2, 45, 2);
  tft.drawString("ST7789", DISPLAY_W / 2, 105, 4);
  tft.drawString("CST328 TOUCH", DISPLAY_W / 2, 175, 2);
  tft.drawString(touchReady ? "TOUCH READY" : "TOUCH NOT FOUND", DISPLAY_W / 2, 215, 2);
  tft.drawString("Tap anywhere", DISPLAY_W / 2, 285, 2);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-S3 portrait TFT + CST328 test");
  setupBacklight();
  setupTouch();
  displayTest();
}

void loop() {
  static int lastX = -1;
  static int lastY = -1;
  int x, y;

  // One read per loop: readRawTouch acknowledges (consumes) each report.
  if (touchReady && readPortraitTouch(x, y)) {
    if (lastX >= 0) tft.fillCircle(lastX, lastY, 5, TFT_BLACK);
    tft.fillCircle(x, y, 5, TFT_CYAN);
    lastX = x;
    lastY = y;
    tft.fillRect(0, 300, DISPLAY_W, 20, TFT_BLACK);
    tft.setTextDatum(MC_DATUM);
    tft.setTextColor(TFT_YELLOW, TFT_BLACK);
    char position[24];
    snprintf(position, sizeof(position), "Touch %d, %d", x, y);
    tft.drawString(position, DISPLAY_W / 2, 310, 2);
    Serial.printf("Touch: x=%d y=%d\n", x, y);
  }
  delay(10);
}
