// =====================================================================
//  CASIO-STYLE TOUCH SCIENTIFIC CALCULATOR  -  ESP32-S3 + ST7789 + CST328
//  Modes: CALC (paged keypad) | GRAPH (multi-function, trace, pan/zoom)
//         TABLE | STAT | EQN (quadratic + 2x2 linear)
//  Answers are shown as exact p/q fractions whenever trustworthy,
//  with the decimal beside it (Casio S<->D style, F/D key toggles).
//  Framebuffer lives in PSRAM. Touch is CST328 code carried over
//  unchanged from the known-working reference project, including its
//  affine calibration + NVS storage (same namespace/keys, so an old
//  calibration still loads). Hold a finger ANYWHERE for 20 s -> the
//  touch-calibration wizard opens.
//  Display: ST7789 320x240 landscape (rotation 1), pins from platformio.ini
//  Touch  : CST328 @0x1A  SDA=4 SCL=5 INT=6 RST=7, 400 kHz, X/Y swapped
// =====================================================================
#include <Arduino.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "CalcEngine.h"
#include "MathTree.h"

// ---------------- hardware (from the working reference project) -------
static const int BL_PIN = 3;
static const int CST_ADDR = 0x1A;
static const int CST_SDA = 4, CST_SCL = 5, CST_INT = 6, CST_RST = 7;
static const uint32_t I2C_HZ = 400000;
static const uint16_t REG_NUM = 0xD005;
static const uint16_t REG_XY  = 0xD000;
static const int SCREEN_W = 320, SCREEN_H = 240;
static const int FB_BYTES = SCREEN_W * SCREEN_H * 2;

#define C565(r,g,b) ((uint16_t)((((r)&0xF8)<<8) | (((g)&0xFC)<<3) | ((b)>>3)))
static const uint16_t C_BG    = C565(246,248,250);
static const uint16_t C_CARD  = C565(255,255,255);
static const uint16_t C_INK   = C565(17,24,39);
static const uint16_t C_MUT   = C565(100,116,139);
static const uint16_t C_LINE  = C565(203,213,225);
static const uint16_t C_ACC   = C565(0,102,204);   // Casio blue
static const uint16_t C_ACCLT = C565(219,234,254);
static const uint16_t C_KEYOP = C565(226,235,245);
static const uint16_t C_DANG  = C565(200,50,45);
static const uint16_t C_GRID  = C565(226,232,240);
static const uint16_t FUNC_COL[3] = { C565(0,102,204), C565(0,150,95), C565(225,110,20) };
static const char *FUNC_NAME[3] = { "f", "g", "h" };

TFT_eSPI tft = TFT_eSPI();
Preferences prefs;
// THE framebuffer: a full-screen sprite whose pixel memory TFT_eSPI
// allocates in PSRAM (sprites default to PSRAM when DMA is off - we
// deliberately never call tft.initDMA(), which would force internal RAM).
// EVERYTHING - shapes and text - is rendered into this one PSRAM canvas,
// then pushed to the panel in a single transfer. (The old code pushed the
// pixel buffer and then painted text straight onto the panel, so every
// redraw visibly erased/repainted the text: that was the flicker.)
static TFT_eSprite canvas(&tft);
static bool canvasOk = false;
static bool canvasInPsram = false;
static bool dirtyPartial = false;           // push only g_pp* region next draw
static int g_ppX = 0, g_ppY = 0, g_ppW = SCREEN_W, g_ppH = SCREEN_H;
static uint32_t lastDrawMs = 0;

// ---------------- small utilities -------------------------------------
static int clampI(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }
static double clampD(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
static void safeCopy(char *dst, size_t n, const char *src) {
  if (!dst || n == 0) return;
  strncpy(dst, src ? src : "", n - 1); dst[n - 1] = '\0';
}

// ---------------- PSRAM framebuffer primitives (all into the sprite) --
static void fbClear(uint16_t c) { if (canvasOk) canvas.fillSprite(c); }
static inline void fbPixel(int x, int y, uint16_t c) { canvas.drawPixel(x, y, c); }
static void fbFillRect(int x, int y, int w, int h, uint16_t c) {
  if (w > 0 && h > 0) canvas.fillRect(x, y, w, h, c);
}
static void fbLine(int x0, int y0, int x1, int y1, uint16_t c) {
  canvas.drawLine(x0, y0, x1, y1, c);
}
static void fbCircle(int cx, int cy, int r, uint16_t c) { canvas.drawCircle(cx, cy, r, c); }
static void fbFillCircle(int cx, int cy, int r, uint16_t c) { canvas.fillCircle(cx, cy, r, c); }
static void fbRoundRect(int x, int y, int w, int h, int r, uint16_t fill, uint16_t border) {
  canvas.fillRoundRect(x, y, w, h, r, border);
  if (w > 2 && h > 2)
    canvas.fillRoundRect(x + 1, y + 1, w - 2, h - 2, r > 1 ? r - 1 : 0, fill);
}

// ---------------- text: drawn INTO the PSRAM canvas, never straight --
// ---------------- to the panel, so a frame push is one atomic image ----
static void addText(const char *s, int x, int y, uint8_t font, uint16_t col, uint16_t bg, uint8_t datum) {
  if (!canvasOk || !s) return;
  canvas.setTextColor(col, bg);
  canvas.setTextDatum(datum);
  canvas.drawString(String(s), x, y, font);
}
static void pushFrame() {
  if (!canvasOk) return;
  if (dirtyPartial) canvas.pushSprite(g_ppX, g_ppY, g_ppX, g_ppY, g_ppW, g_ppH);  // cropped push
  else canvas.pushSprite(0, 0);                                                   // one full frame
}

// ---------------- hit boxes (built while drawing) ----------------------
struct Hit { int x, y, w, h, id, arg; };
static Hit hits[80];
static int hitCount = 0;
static void addHit(int x, int y, int w, int h, int id, int arg) {
  if (hitCount >= 80) return;
  hits[hitCount] = { x, y, w, h, id, arg };
  hitCount++;
}

// Button styles: 0 digit/white, 1 operator, 2 accent, 3 selected, 4 danger, 5 plain
static void drawButton(int x, int y, int w, int h, const char *label,
                       int id, int arg, int style, uint8_t font = 2) {
  uint16_t fill = C_CARD, border = C_LINE, col = C_INK;
  if (style == 1) { fill = C_KEYOP; border = C_KEYOP; }
  else if (style == 2) { fill = C_ACC; border = C_ACC; col = C_CARD; }
  else if (style == 3) { fill = C_ACCLT; border = C_ACC; col = C_ACC; }
  else if (style == 4) { fill = C565(254,226,226); border = C565(254,202,202); col = C_DANG; }
  else if (style == 5) { fill = C_BG; border = C_BG; col = C_MUT; }
  fbRoundRect(x, y, w, h, 5, fill, border);
  if (label && label[0])
    addText(label, x + w/2, y + h/2, font, col, fill, MC_DATUM);
  if (id >= 0) addHit(x, y, w, h, id, arg);
}

// ---------------- app state --------------------------------------------
enum Mode { M_CALC = 0, M_GRAPH, M_TABLE, M_STAT, M_EQN };
enum BtnId {
  B_NONE = -1, B_MODE = 1, B_KEY, B_PAGE, B_SHIFT, B_DEG, B_FD, B_HIST,
  B_HIST_ITEM, B_HIST_CLOSE, B_FIELD, B_NUM, B_FUNCSEL, B_FUNCEYE,
  B_FUNCCLEAR, B_PLOTDONE, B_GRAPHTOOL, B_TABLENAV, B_TABLEEDIT,
  B_STATBTN, B_EQNKIND, B_CALCANCEL
};
enum KeyPage { KP_NUM = 0, KP_SCI, KP_ADV, KP_MORE };

static Mode mode = M_CALC;
static bool dirty = true;
static void markDirty() { dirty = true; dirtyPartial = false; }  // real changes always push a full frame

static bool angleDeg = true;
static bool preferFraction = true;

static String calcExpr;
static int calcCursor = 0;
static bool calcShowResult = false, calcJustEvaluated = false;
static double calcResult = 0, lastAns = 0;
static char calcError[28] = "";

static String funcExpr[3] = { "sin(x)", "x^2/8", "" };
static int funcCursor[3] = { 6, 5, 0 };
static bool funcVisible[3] = { true, true, true };
static int activeFunc = 0;
static bool graphEditing = false;
static Mode graphReturnMode = M_GRAPH;

static int keyPage = KP_NUM;
static bool shiftOn = false;
static bool cursorVisible = true;
static uint32_t lastBlinkMs = 0;

struct HistItem { String expr; double value; };
static HistItem hist[8];
static int histCount = 0;
static bool histOpen = false;

// graph view
static double viewCx = 0, viewCy = 0, viewScale = 26;
static bool traceOn = false;
static double traceX = 0;
static const int PLOT_X = 36, PLOT_Y = 34, PLOT_W = 278, PLOT_H = 148;

// table
static double tblStart = 0, tblEnd = 10, tblStep = 1;
static int tblScroll = 0;
static char tblError[28] = "";

// stat
static double statData[40];
static int statCount = 0;

// eqn
static bool eqnLinear = false;
static double quadA = 1, quadB = 0, quadC = -4;
static double linV[6] = { 1, 1, 3, 1, -1, 1 };  // a1 b1 c1 / a2 b2 c2

// numeric editor overlay
enum FieldId {
  F_TBL_START = 0, F_TBL_END, F_TBL_STEP,
  F_QUAD_A, F_QUAD_B, F_QUAD_C,
  F_LIN_0, F_LIN_1, F_LIN_2, F_LIN_3, F_LIN_4, F_LIN_5,
  F_STAT_ADD = 100
};
static bool numOpen = false;
static int numTarget = F_TBL_START;
static char numBuf[24] = "";
static bool numError = false;

// drag
enum DragMode { DRAG_NONE = 0, DRAG_PLOT, DRAG_TABLE };
static int dragMode = DRAG_NONE;
static int dragLastX = 0, dragLastY = 0, dragAccY = 0;

// ---------------- touch: CST328, verbatim logic from reference --------
static float CAL_A =  0.973913038f, CAL_B = -0.031239455f, CAL_C =   7.33205278f;
static float CAL_D = -0.046093319f, CAL_E = -0.903280217f, CAL_F = 235.615557f;
static float EDGE_GX = 1.04f, EDGE_GY = 1.18f;
static bool calLoaded = false;
static const uint32_t CAL_MAGIC = 0x43414C34;
static const char *NVS_NS = "nexusos";  // same as reference: old cal loads

static bool touchActive = false, touchEvDown = false, touchEvUp = false;
static int touchX = 0, touchY = 0;
static uint32_t lastTouchMs = 0, touchDownAtMs = 0;
static bool rawValid = false;
static uint16_t rawTX = 0, rawTY = 0;

static void CalibrateTouch(float rx, float ry, int &ox, int &oy) {
  float x = CAL_A * rx + CAL_B * ry + CAL_C;
  float y = CAL_D * rx + CAL_E * ry + CAL_F;
  x = (x - SCREEN_W * 0.5f) * EDGE_GX + SCREEN_W * 0.5f;
  y = (y - SCREEN_H * 0.5f) * EDGE_GY + SCREEN_H * 0.5f;
  ox = clampI((int)(x + 0.5f), 0, SCREEN_W - 1);
  oy = clampI((int)(y + 0.5f), 0, SCREEN_H - 1);
}
static bool I2C_Read(uint8_t a, uint16_t r, uint8_t *d, uint32_t n) {
  Wire.beginTransmission(a);
  Wire.write((uint8_t)(r >> 8));
  Wire.write((uint8_t)(r & 0xFF));
  if (Wire.endTransmission(true) != 0) return false;
  uint32_t got = Wire.requestFrom((uint8_t)a, (uint8_t)n);
  if (got != n) return false;
  for (uint32_t i = 0; i < n; i++) d[i] = Wire.read();
  return true;
}
static void CST328_Reset() {
  pinMode(CST_RST, OUTPUT);
  digitalWrite(CST_RST, HIGH); delay(50);
  digitalWrite(CST_RST, LOW);  delay(5);
  digitalWrite(CST_RST, HIGH); delay(50);
}
// Destructive read (acks by writing 0 to REG_NUM): call ONCE per loop,
// always through PollTouch(); results are cached in rawTX/rawTY/rawValid.
static bool ReadRawTouch(uint16_t &x, uint16_t &y) {
  uint8_t buf[28] = {0};
  if (!I2C_Read(CST_ADDR, REG_NUM, buf, 1)) return false;
  if ((buf[0] & 0x0F) == 0) return false;
  if (!I2C_Read(CST_ADDR, REG_XY, &buf[1], 27)) return false;
  uint16_t rx = ((uint16_t)buf[2] << 4) | ((buf[4] & 0xF0) >> 4);
  uint16_t ry = ((uint16_t)buf[3] << 4) | (buf[4] & 0x0F);
  x = ry;  // this panel has X/Y swapped -- do not remove
  y = rx;
  Wire.beginTransmission(CST_ADDR);
  Wire.write((uint8_t)(REG_NUM >> 8));
  Wire.write((uint8_t)(REG_NUM & 0xFF));
  Wire.write((uint8_t)0);
  Wire.endTransmission(true);
  return true;
}
static void PollTouch() {
  touchEvDown = touchEvUp = false;
  rawValid = false;
  uint16_t rx, ry;
  uint32_t now = millis();
  if (ReadRawTouch(rx, ry)) {
    rawTX = rx; rawTY = ry; rawValid = true;
    int cx, cy;
    CalibrateTouch((float)rx, (float)ry, cx, cy);
    touchX = cx; touchY = cy;
    if (!touchActive) {
      touchActive = true; touchEvDown = true; touchDownAtMs = now;
    }
    lastTouchMs = now;
  } else if (touchActive && now - lastTouchMs > 90) {
    touchActive = false; touchEvUp = true;
  }
}
static void SetBrightness(uint8_t v) {
  if (v < 8) v = 8;
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(BL_PIN, v);
#else
  ledcWrite(0, v);
#endif
}

// ---------------- NVS: calibration (reference keys) + settings --------
static void SaveCalibration() {
  prefs.begin(NVS_NS, false);
  prefs.putUInt("magic", CAL_MAGIC);
  prefs.putFloat("A", CAL_A); prefs.putFloat("B", CAL_B); prefs.putFloat("C", CAL_C);
  prefs.putFloat("D", CAL_D); prefs.putFloat("E", CAL_E); prefs.putFloat("F", CAL_F);
  prefs.putFloat("GX", EDGE_GX); prefs.putFloat("GY", EDGE_GY);
  prefs.end();
  calLoaded = true;
}
static void SaveSettings() {
  prefs.begin(NVS_NS, false);
  prefs.putUChar("cAng", angleDeg ? 1 : 0);
  prefs.putUChar("cFrac", preferFraction ? 1 : 0);
  prefs.end();
}
static void LoadSettings() {
  prefs.begin(NVS_NS, true);
  if (prefs.getUInt("magic", 0) == CAL_MAGIC) {
    CAL_A = prefs.getFloat("A", CAL_A); CAL_B = prefs.getFloat("B", CAL_B);
    CAL_C = prefs.getFloat("C", CAL_C); CAL_D = prefs.getFloat("D", CAL_D);
    CAL_E = prefs.getFloat("E", CAL_E); CAL_F = prefs.getFloat("F", CAL_F);
    EDGE_GX = prefs.getFloat("GX", EDGE_GX); EDGE_GY = prefs.getFloat("GY", EDGE_GY);
    calLoaded = true;
  }
  angleDeg = prefs.getUChar("cAng", 1) != 0;
  preferFraction = prefs.getUChar("cFrac", 1) != 0;
  prefs.end();
  Serial.println(calLoaded ? "CAL: loaded from flash" : "CAL: using defaults");
}

// ---------------- calibration wizard (affine least squares) ------------
static bool inCal = false;
static Mode calReturnMode = M_CALC;
static bool calReturnEditing = false;
static const int CAL_PTS = 5;
static const int16_t CAL_TX[CAL_PTS] = { 20, 300, 160, 20, 300 };
static const int16_t CAL_TY[CAL_PTS] = { 18, 18, 120, 222, 222 };
static int calStage = 0, calSamples = 0;
static float calSumX = 0, calSumY = 0;
static float calRawX[CAL_PTS], calRawY[CAL_PTS];
static float calResid = 0;
static bool calFinished = false;
static uint32_t calDoneMs = 0, calHoldStart = 0, calLastSeenMs = 0;
static bool calWaitLift = true;  // ignore the 20 s long-press finger itself

static void startCalibration() {
  calReturnMode = mode;
  calReturnEditing = graphEditing;
  inCal = true;
  calStage = 0; calSamples = 0; calSumX = calSumY = 0;
  calResid = 0; calFinished = false; calLastSeenMs = millis();
  calWaitLift = true;
  dragMode = DRAG_NONE;
  markDirty();
  Serial.println("CAL: wizard started (20 s long-press or boot)");
}
static void exitCalibration() {
  inCal = false;
  mode = calReturnMode;
  graphEditing = calReturnEditing;
  markDirty();
}
static bool solve3x3(float A[3][3], float b[3], float out[3]) {
  float m[3][4];
  for (int i = 0; i < 3; i++) { for (int j = 0; j < 3; j++) m[i][j] = A[i][j]; m[i][3] = b[i]; }
  for (int c = 0; c < 3; c++) {
    int piv = c;
    for (int r = c + 1; r < 3; r++) if (fabsf(m[r][c]) > fabsf(m[piv][c])) piv = r;
    if (fabsf(m[piv][c]) < 1e-9f) return false;
    if (piv != c) for (int j = 0; j < 4; j++) { float t = m[c][j]; m[c][j] = m[piv][j]; m[piv][j] = t; }
    float d = m[c][c];
    for (int j = 0; j < 4; j++) m[c][j] /= d;
    for (int r = 0; r < 3; r++) {
      if (r == c) continue;
      float f = m[r][c];
      for (int j = 0; j < 4; j++) m[r][j] -= f * m[c][j];
    }
  }
  out[0] = m[0][3]; out[1] = m[1][3]; out[2] = m[2][3];
  return true;
}
static void calibSolve() {
  float M[3][3] = {{0,0,0},{0,0,0},{0,0,0}}, bx[3] = {0,0,0}, by[3] = {0,0,0};
  for (int i = 0; i < CAL_PTS; i++) {
    float v[3] = { calRawX[i], calRawY[i], 1.0f };
    for (int r = 0; r < 3; r++) {
      for (int c = 0; c < 3; c++) M[r][c] += v[r] * v[c];
      bx[r] += v[r] * (float)CAL_TX[i];
      by[r] += v[r] * (float)CAL_TY[i];
    }
  }
  float sx[3], sy[3];
  float oA=CAL_A,oB=CAL_B,oC=CAL_C,oD=CAL_D,oE=CAL_E,oF=CAL_F,oGX=EDGE_GX,oGY=EDGE_GY;
  calResid = -1;
  if (solve3x3(M, bx, sx) && solve3x3(M, by, sy)) {
    CAL_A=sx[0]; CAL_B=sx[1]; CAL_C=sx[2];
    CAL_D=sy[0]; CAL_E=sy[1]; CAL_F=sy[2];
    EDGE_GX = 1.0f; EDGE_GY = 1.0f;
    float sum = 0;
    for (int i = 0; i < CAL_PTS; i++) {
      float px = CAL_A*calRawX[i] + CAL_B*calRawY[i] + CAL_C;
      float py = CAL_D*calRawX[i] + CAL_E*calRawY[i] + CAL_F;
      sum += sqrtf((px-CAL_TX[i])*(px-CAL_TX[i]) + (py-CAL_TY[i])*(py-CAL_TY[i]));
    }
    calResid = sum / CAL_PTS;
    bool finite = (CAL_A==CAL_A)&&(CAL_B==CAL_B)&&(CAL_C==CAL_C)&&(CAL_D==CAL_D)&&(CAL_E==CAL_E)&&(CAL_F==CAL_F);
    bool sane = finite && fabsf(CAL_A)<8.0f && fabsf(CAL_E)<8.0f &&
                fabsf(CAL_C)<2000.0f && fabsf(CAL_F)<2000.0f && calResid>=0.0f && calResid<28.0f;
    if (sane) {
      SaveCalibration();
      Serial.printf("CAL mean residual %.2f px -- saved\n", calResid);
    } else {
      CAL_A=oA; CAL_B=oB; CAL_C=oC; CAL_D=oD; CAL_E=oE; CAL_F=oF; EDGE_GX=oGX; EDGE_GY=oGY;
      calResid = -1;
      Serial.println("CAL rejected (bad fit) -- previous values kept");
    }
  }
  calFinished = true;
  calDoneMs = millis();
}
static void updateCalibration() {
  if (calFinished) {
    if (millis() - calDoneMs > 2200) exitCalibration();
    return;
  }
  if (millis() - calLastSeenMs > 25000) {  // walked away: auto-cancel
    Serial.println("CAL: timeout, cancelled");
    exitCalibration();
    return;
  }
  // CANCEL button hit (touch coords from last PollTouch)
  if (touchEvDown) {
    for (int i = 0; i < hitCount; i++) {
      Hit &h = hits[i];
      if (h.id == B_CALCANCEL && touchX >= h.x && touchX < h.x + h.w &&
          touchY >= h.y && touchY < h.y + h.h) {
        Serial.println("CAL: cancelled by user");
        exitCalibration();
        return;
      }
    }
  }
  if (calWaitLift) {
    // The finger that triggered calibration (or finished the last point)
    // must lift before we trust any sample for the next crosshair.
    if (!touchActive) { calWaitLift = false; calLastSeenMs = millis(); }
    markDirty();
    return;
  }
  if (rawValid) {  // consume ONLY the cached sample: never re-read I2C here
    calLastSeenMs = millis();
    if (calSamples == 0) calHoldStart = millis();
    calSumX += rawTX; calSumY += rawTY; calSamples++;
    markDirty();
    if (calSamples >= 14 && millis() - calHoldStart > 300) {
      calRawX[calStage] = calSumX / calSamples;
      calRawY[calStage] = calSumY / calSamples;
      Serial.printf("CAL %d/%d target(%d,%d) raw(%.1f,%.1f)\n",
                    calStage + 1, CAL_PTS, CAL_TX[calStage], CAL_TY[calStage],
                    calRawX[calStage], calRawY[calStage]);
      calStage++; calSamples = 0; calSumX = calSumY = 0;
      calWaitLift = true;  // lift before next crosshair: no sliding samples
      if (calStage >= CAL_PTS) calibSolve();
    }
  } else {
    calSamples = 0; calSumX = calSumY = 0;
  }
}
static void drawCalScreen() {
  fbRoundRect(4, 4, SCREEN_W - 8, SCREEN_H - 8, 8, C_CARD, C_ACC);
  if (!calFinished) {
    addText("TOUCH CALIBRATION", 160, 44, 2, C_ACC, C_CARD, MC_DATUM);
    char buf[40];
    snprintf(buf, sizeof(buf), "POINT %d OF %d", calStage + 1, CAL_PTS);
    addText(buf, 160, 64, 1, C_INK, C_CARD, MC_DATUM);
    addText(calWaitLift ? "Release your finger, then hold the crosshair"
                        : "Hold the crosshair until the ring fills",
            160, 78, 1, C_MUT, C_CARD, MC_DATUM);
    int tx = CAL_TX[calStage], ty = CAL_TY[calStage];
    fbCircle(tx, ty, 14, C_ACC);
    fbLine(tx - 20, ty, tx - 6, ty, C_INK); fbLine(tx + 6, ty, tx + 20, ty, C_INK);
    fbLine(tx, ty - 20, tx, ty - 6, C_INK); fbLine(tx, ty + 6, tx, ty + 20, C_INK);
    fbFillCircle(tx, ty, 3, C_ACC);
    if (calSamples > 0) {  // progress arc
      float fr = calSamples / 14.0f; if (fr > 1) fr = 1;
      int steps = (int)(fr * 40);
      for (int i = 0; i < steps; i++) {
        float a = -1.5708f + 6.28318f * (i / 40.0f);
        fbFillCircle(tx + (int)(cosf(a) * 20), ty + (int)(sinf(a) * 20), 2, C_ACC);
      }
    }
    for (int i = 0; i < CAL_PTS; i++)
      fbFillCircle(160 - (CAL_PTS * 12) / 2 + i * 12 + 6, 100, i < calStage ? 4 : 2,
                   i < calStage ? C_ACC : C_LINE);
    drawButton(120, 196, 80, 24, "CANCEL", B_CALCANCEL, 0, 4, 1);
    addText("Auto-cancels after 25 s without touch", 160, 232, 1, C_MUT, C_CARD, MC_DATUM);
  } else {
    addText("CALIBRATED", 160, 80, 4, C_ACC, C_CARD, MC_DATUM);
    char buf[48];
    if (calResid >= 0) {
      snprintf(buf, sizeof(buf), "Mean error %.1f px  %s", calResid,
               calResid < 4.0f ? "EXCELLENT" : (calResid < 9.0f ? "GOOD" : "POOR - RERUN"));
      addText(buf, 160, 112, 1, C_INK, C_CARD, MC_DATUM);
      addText("Saved to flash", 160, 130, 1, C_MUT, C_CARD, MC_DATUM);
    } else {
      addText("Solve failed - old calibration kept", 160, 112, 1, C_DANG, C_CARD, MC_DATUM);
    }
    addText("Returning...", 160, 160, 1, C_MUT, C_CARD, MC_DATUM);
  }
}

// ---------------- expression editing -----------------------------------
static String &exprRef() { return graphEditing ? funcExpr[activeFunc] : calcExpr; }
static int &cursorRef() { return graphEditing ? funcCursor[activeFunc] : calcCursor; }

static void exprInsert(const char *ins) {
  String &e = exprRef();
  int &c = cursorRef();
  if (!graphEditing && calcJustEvaluated) {
    char f = ins[0];
    bool cont = (f=='+'||f=='-'||f=='*'||f=='/'||f=='^'||f=='%'||f=='!');
    if (cont) { e = "Ans"; c = 3; }
    else { e = ""; c = 0; }
    calcJustEvaluated = false;
    calcShowResult = false;
  }
  if ((int)e.length() + (int)strlen(ins) > 110) {
    if (!graphEditing) safeCopy(calcError, sizeof(calcError), "Too long");
    markDirty();
    return;
  }
  String ne = e.substring(0, c) + ins + e.substring(c);
  e = ne;
  c += (int)strlen(ins);
  if (!graphEditing) calcError[0] = '\0';
  markDirty();
}
static void exprDeleteBack() {
  String &e = exprRef();
  int &c = cursorRef();
  if (c <= 0 || e.length() == 0) return;
  static const char *tokens[] = {
    "asinh(","acosh(","atanh(","floor(","trunc(","hypot(","atan2(",
    "asin(","acos(","atan(","asec(","acsc(","acot(","sinh(","cosh(","tanh(",
    "sqrt(","cbrt(","log2(","frac(","ceil(","nPr(","nCr(","gcd(","lcm(",
    "sin(","cos(","tan(","sec(","csc(","cot(","log(","abs(","exp(","min(","max(","mod(",
    "ln(","Ans","tau","phi","pi"
  };
  String before = e.substring(0, c);
  int del = 1;
  for (size_t i = 0; i < sizeof(tokens)/sizeof(tokens[0]); i++) {
    int tl = (int)strlen(tokens[i]);
    if ((int)before.length() >= tl && before.endsWith(tokens[i])) { del = tl; break; }
  }
  if (del > c) del = c;
  String ne = e.substring(0, c - del) + e.substring(c);
  e = ne;
  c -= del;
  calcJustEvaluated = false;
  markDirty();
}
static void exprClearAll() {
  String &e = exprRef();
  int &c = cursorRef();
  e = ""; c = 0;
  if (!graphEditing) { calcShowResult = false; calcError[0] = '\0'; }
  calcJustEvaluated = false;
  markDirty();
}
static void pushHistory(const String &expr, double value) {
  if (histCount < 8) {
    hist[histCount].expr = expr; hist[histCount].value = value; histCount++;
  } else {
    for (int i = 0; i < 7; i++) hist[i] = hist[i + 1];
    hist[7].expr = expr; hist[7].value = value;
  }
}
static void doEquals() {
  if (graphEditing) {  // "=" on a function means PLOT
    graphEditing = false;
    mode = graphReturnMode;
    markDirty();
    return;
  }
  EvalResult r;
  if (!evaluateExpression(calcExpr.c_str(), 0, lastAns, angleDeg, r)) {
    calcShowResult = false; calcJustEvaluated = false;
    safeCopy(calcError, sizeof(calcError), r.error);
    markDirty();
    return;
  }
  calcResult = r.value;
  lastAns = r.value;
  calcShowResult = true;
  calcJustEvaluated = true;
  calcError[0] = '\0';
  pushHistory(calcExpr, r.value);
  markDirty();
}

// ---------------- paged keypad ------------------------------------------
// Pages keep every key >= 61x26 px (finger-sized). [123] [FX] [MORE]
// swap the whole grid instead of cramming 40 tiny keys on one page.
enum KeyAct { KA_INS = 0, KA_DEL, KA_AC, KA_EQ, KA_LEFT, KA_RIGHT, KA_FD, KA_DEG, KA_HIST };
struct KeyDef { char label[12]; char insert[18]; int act; int style; };
static KeyDef curKeys[20];
static void setKey(int i, const char *label, const char *ins, int act, int style) {
  safeCopy(curKeys[i].label, sizeof(curKeys[i].label), label);
  safeCopy(curKeys[i].insert, sizeof(curKeys[i].insert), ins);
  curKeys[i].act = act; curKeys[i].style = style;
}
static void buildKeys() {
  if (keyPage == KP_NUM) {
    static const char *rows[4] = { "789", "456", "123", "" };
    (void)rows;
    setKey(0,"7","7",KA_INS,0); setKey(1,"8","8",KA_INS,0); setKey(2,"9","9",KA_INS,0);
    setKey(3,"DEL","",KA_DEL,4); setKey(4,"AC","",KA_AC,4);
    setKey(5,"4","4",KA_INS,0); setKey(6,"5","5",KA_INS,0); setKey(7,"6","6",KA_INS,0);
    setKey(8,"*","*",KA_INS,1); setKey(9,"/","/",KA_INS,1);
    setKey(10,"1","1",KA_INS,0); setKey(11,"2","2",KA_INS,0); setKey(12,"3","3",KA_INS,0);
    setKey(13,"+","+",KA_INS,1); setKey(14,"-","-",KA_INS,1);
    if (graphEditing) {
      setKey(15,"x","x",KA_INS,1); setKey(16,"(","(",KA_INS,1); setKey(17,")",")",KA_INS,1);
      setKey(18,".",".",KA_INS,0); setKey(19,"PLOT","",KA_EQ,2);
    } else {
      setKey(15,"(","(",KA_INS,1); setKey(16,")",")",KA_INS,1); setKey(17,"0","0",KA_INS,0);
      setKey(18,".",".",KA_INS,0); setKey(19,"=","",KA_EQ,2);
    }
  } else if (keyPage == KP_SCI) {
    if (!shiftOn) {
      setKey(0,"sin","sin(",KA_INS,1); setKey(1,"cos","cos(",KA_INS,1); setKey(2,"tan","tan(",KA_INS,1);
      setKey(3,"(","(",KA_INS,1); setKey(4,")",")",KA_INS,1);
      setKey(5,"ln","ln(",KA_INS,1); setKey(6,"log","log(",KA_INS,1); setKey(7,"sqrt","sqrt(",KA_INS,1);
      setKey(8,"x^2","^2",KA_INS,1); setKey(9,"^","^",KA_INS,1);
      setKey(10,"pi","pi",KA_INS,1); setKey(11,"e","e",KA_INS,1); setKey(12,"Ans","Ans",KA_INS,1);
      setKey(13,"x","x",KA_INS,1); setKey(14,"!","!",KA_INS,1);
      setKey(15,"abs","abs(",KA_INS,1); setKey(16,"10^x","10^",KA_INS,1); setKey(17,"%","%",KA_INS,1);
      setKey(18,"DEL","",KA_DEL,4); setKey(19,"=","",KA_EQ,2);
    } else {
      setKey(0,"asin","asin(",KA_INS,1); setKey(1,"acos","acos(",KA_INS,1); setKey(2,"atan","atan(",KA_INS,1);
      setKey(3,"sinh","sinh(",KA_INS,1); setKey(4,"cosh","cosh(",KA_INS,1);
      setKey(5,"tanh","tanh(",KA_INS,1); setKey(6,"exp","exp(",KA_INS,1); setKey(7,"log2","log2(",KA_INS,1);
      setKey(8,"cbrt","cbrt(",KA_INS,1); setKey(9,"x^3","^3",KA_INS,1);
      setKey(10,"tau","tau",KA_INS,1); setKey(11,"e^x","e^",KA_INS,1); setKey(12,"1/x","^-1",KA_INS,1);
      setKey(13,"x","x",KA_INS,1); setKey(14,"!","!",KA_INS,1);
      setKey(15,"abs","abs(",KA_INS,1); setKey(16,"sqrt","sqrt(",KA_INS,1); setKey(17,"%","%",KA_INS,1);
      setKey(18,"DEL","",KA_DEL,4); setKey(19,"=","",KA_EQ,2);
    }
    if (graphEditing) setKey(19,"PLOT","",KA_EQ,2);
  } else if (keyPage == KP_ADV) {
    // Advanced symbols page: rounding, combinatorics, 2-arg functions.
    // SHIFT swaps in reciprocal trig + hyperbolic inverses.
    if (!shiftOn) {
      setKey(0,"floor","floor(",KA_INS,1); setKey(1,"ceil","ceil(",KA_INS,1); setKey(2,"round","round(",KA_INS,1);
      setKey(3,"|x|","abs(",KA_INS,1); setKey(4,"sgn","sgn(",KA_INS,1);
      setKey(5,"frac","frac(",KA_INS,1); setKey(6,"trunc","trunc(",KA_INS,1); setKey(7,"min","min(",KA_INS,1);
      setKey(8,"max","max(",KA_INS,1); setKey(9,"mod","mod(",KA_INS,1);
      setKey(10,"nPr","nPr(",KA_INS,1); setKey(11,"nCr","nCr(",KA_INS,1); setKey(12,"gcd","gcd(",KA_INS,1);
      setKey(13,"lcm","lcm(",KA_INS,1); setKey(14,",",",",KA_INS,1);
      setKey(15,"hypot","hypot(",KA_INS,1); setKey(16,"phi","phi",KA_INS,1); setKey(17,"%","%",KA_INS,1);
      setKey(18,"DEL","",KA_DEL,4); setKey(19,"=","",KA_EQ,2);
    } else {
      setKey(0,"sec","sec(",KA_INS,1); setKey(1,"csc","csc(",KA_INS,1); setKey(2,"cot","cot(",KA_INS,1);
      setKey(3,"asec","asec(",KA_INS,1); setKey(4,"acsc","acsc(",KA_INS,1);
      setKey(5,"acot","acot(",KA_INS,1); setKey(6,"asinh","asinh(",KA_INS,1); setKey(7,"acosh","acosh(",KA_INS,1);
      setKey(8,"atanh","atanh(",KA_INS,1); setKey(9,"atan2","atan2(",KA_INS,1);
      setKey(10,"nPr","nPr(",KA_INS,1); setKey(11,"nCr","nCr(",KA_INS,1); setKey(12,"gcd","gcd(",KA_INS,1);
      setKey(13,"lcm","lcm(",KA_INS,1); setKey(14,",",",",KA_INS,1);
      setKey(15,"hypot","hypot(",KA_INS,1); setKey(16,"phi","phi",KA_INS,1); setKey(17,"%","%",KA_INS,1);
      setKey(18,"DEL","",KA_DEL,4); setKey(19,"=","",KA_EQ,2);
    }
    if (graphEditing) setKey(19,"PLOT","",KA_EQ,2);
  } else {  // KP_MORE: cursor, display format, angle, history, constants
    setKey(0,"<","",KA_LEFT,1); setKey(1,">","",KA_RIGHT,1);
    setKey(2,"F/D","",KA_FD,1); setKey(3,"HIST","",KA_HIST,1);
    setKey(4, angleDeg ? "DEG" : "RAD", "", KA_DEG, 3);
    setKey(5,"Ans","Ans",KA_INS,1); setKey(6,"x","x",KA_INS,1); setKey(7,"pi","pi",KA_INS,1);
    setKey(8,"tau","tau",KA_INS,1); setKey(9,"e","e",KA_INS,1);
    setKey(10,"sin","sin(",KA_INS,1); setKey(11,"cos","cos(",KA_INS,1); setKey(12,"tan","tan(",KA_INS,1);
    setKey(13,"(","(",KA_INS,1); setKey(14,")",")",KA_INS,1);
    setKey(15,"0","0",KA_INS,0); setKey(16,".",".",KA_INS,0);
    setKey(17,"DEL","",KA_DEL,4); setKey(18,"AC","",KA_AC,4);
    setKey(19, graphEditing ? "PLOT" : "=", "", KA_EQ, 2);
  }
}
static void handleKey(int idx) {
  if (idx < 0 || idx >= 20) return;
  KeyDef &k = curKeys[idx];
  bool wasShift = shiftOn;
  switch (k.act) {
    case KA_INS: exprInsert(k.insert); break;
    case KA_DEL: exprDeleteBack(); break;
    case KA_AC: exprClearAll(); break;
    case KA_EQ: doEquals(); break;
    case KA_LEFT: { int &c = cursorRef(); if (c > 0) c--; } break;
    case KA_RIGHT: { int &c = cursorRef(); if (c < (int)exprRef().length()) c++; } break;
    case KA_FD: preferFraction = !preferFraction; SaveSettings(); break;
    case KA_DEG: angleDeg = !angleDeg; SaveSettings(); break;
    case KA_HIST: if (!graphEditing) histOpen = true; break;
    default: break;
  }
  if (wasShift && (keyPage == KP_SCI || keyPage == KP_ADV)) shiftOn = false;  // one-shot SHIFT
  markDirty();
}
static void drawKeypad(int topY) {
  // page selector bar
  drawButton(4,   topY, 42, 19, "123",  B_PAGE, KP_NUM,  keyPage == KP_NUM  ? 3 : 5, 1);
  drawButton(48,  topY, 42, 19, "FUNC", B_PAGE, KP_SCI,  keyPage == KP_SCI  ? 3 : 5, 1);
  drawButton(92,  topY, 42, 19, "ADV",  B_PAGE, KP_ADV,  keyPage == KP_ADV  ? 3 : 5, 1);
  drawButton(136, topY, 46, 19, "MORE", B_PAGE, KP_MORE, keyPage == KP_MORE ? 3 : 5, 1);
  drawButton(186, topY, 60, 19, "SHIFT", B_SHIFT, 0, shiftOn ? 3 : 5, 1);
  drawButton(250, topY, 66, 19, angleDeg ? "DEG" : "RAD", B_DEG, 0, 5, 1);
  buildKeys();
  int gridY = topY + 24;
  for (int i = 0; i < 20; i++) {
    int cx = 4 + (i % 5) * 63;
    int cy = gridY + (i / 5) * 29;
    uint8_t fnt = (uint8_t)(strlen(curKeys[i].label) > 4 ? 1 : 2);
    drawButton(cx, cy, 61, 26, curKeys[i].label, B_KEY, i, curKeys[i].style, fnt);
  }
}

// ---------------- shared format helpers --------------------------------
static void fmtVal(double v, char *out, size_t n) {  // fraction-first, compact
  char sec[48];
  formatSmart(v, preferFraction, out, n, sec, sizeof(sec));
  if (strlen(out) > 13) formatDecimal(v, out, n);
}
static void formatAxis(double v, char *out, size_t n) {
  double r = round(v);
  if (fabs(v - r) < 1e-9 && fabs(r) < 1e7) snprintf(out, n, "%d", (int)r);
  else snprintf(out, n, "%.3g", v);
}

// ---------------- fields (table / stat / eqn numeric entry) ------------
static const char *fieldLabel(int id) {
  switch (id) {
    case F_TBL_START: return "Start"; case F_TBL_END: return "End"; case F_TBL_STEP: return "Step";
    case F_QUAD_A: return "a"; case F_QUAD_B: return "b"; case F_QUAD_C: return "c";
    case F_LIN_0: return "a1"; case F_LIN_1: return "b1"; case F_LIN_2: return "c1";
    case F_LIN_3: return "a2"; case F_LIN_4: return "b2"; case F_LIN_5: return "c2";
    case F_STAT_ADD: return "Data value";
    default: return "Value";
  }
}
static bool fieldValue(int id, double &v) {
  switch (id) {
    case F_TBL_START: v = tblStart; return true;
    case F_TBL_END: v = tblEnd; return true;
    case F_TBL_STEP: v = tblStep; return true;
    case F_QUAD_A: v = quadA; return true;
    case F_QUAD_B: v = quadB; return true;
    case F_QUAD_C: v = quadC; return true;
    case F_LIN_0: case F_LIN_1: case F_LIN_2: case F_LIN_3: case F_LIN_4: case F_LIN_5:
      v = linV[id - F_LIN_0]; return true;
    default: return false;
  }
}
static void fieldSet(int id, double v) {
  switch (id) {
    case F_TBL_START: tblStart = v; tblScroll = 0; break;
    case F_TBL_END: tblEnd = v; tblScroll = 0; break;
    case F_TBL_STEP: tblStep = v; tblScroll = 0; break;
    case F_QUAD_A: quadA = v; break;
    case F_QUAD_B: quadB = v; break;
    case F_QUAD_C: quadC = v; break;
    case F_LIN_0: case F_LIN_1: case F_LIN_2: case F_LIN_3: case F_LIN_4: case F_LIN_5:
      linV[id - F_LIN_0] = v; break;
    default: break;
  }
}
static void openNumeric(int target) {
  numTarget = target;
  numOpen = true;
  numError = false;
  numBuf[0] = '\0';
  if (target != F_STAT_ADD) {
    double v;
    if (fieldValue(target, v)) formatDecimal(v, numBuf, sizeof(numBuf));
  }
  markDirty();
}
static void handleNumeric(int idx) {
  static const char *keys[16] = {
    "7","8","9","DEL", "4","5","6","+/-", "1","2","3","CLR", "0",".","ESC","OK"
  };
  if (idx < 0 || idx >= 16) return;
  const char *k = keys[idx];
  int len = (int)strlen(numBuf);
  if (!strcmp(k, "DEL")) { if (len) numBuf[len - 1] = '\0'; }
  else if (!strcmp(k, "CLR")) numBuf[0] = '\0';
  else if (!strcmp(k, "ESC")) { numOpen = false; }
  else if (!strcmp(k, "+/-")) {
    if (len && numBuf[0] == '-') memmove(numBuf, numBuf + 1, strlen(numBuf));
    else if (len < (int)sizeof(numBuf) - 1) { memmove(numBuf + 1, numBuf, strlen(numBuf) + 1); numBuf[0] = '-'; }
  } else if (!strcmp(k, "OK")) {
    if (!numBuf[0] || (numBuf[0] == '-' && !numBuf[1])) { numError = true; markDirty(); return; }
    char *endp = nullptr;
    double v = strtod(numBuf, &endp);
    if (!endp || *endp != '\0' || isnan(v) || isinf(v)) { numError = true; markDirty(); return; }
    if (numTarget == F_STAT_ADD) {
      if (statCount >= 40) { numError = true; markDirty(); return; }
      statData[statCount++] = v;
    } else fieldSet(numTarget, v);
    numOpen = false;
  } else {  // digit or dot
    if (k[0] == '.' && strchr(numBuf, '.')) { markDirty(); return; }
    if (len < (int)sizeof(numBuf) - 1) { numBuf[len] = k[0]; numBuf[len + 1] = '\0'; }
  }
  markDirty();
}
static void drawNumeric() {
  fbRoundRect(8, 8, 304, 224, 8, C_CARD, C_LINE);
  addText("ENTER VALUE", 20, 20, 1, C_MUT, C_CARD, TL_DATUM);
  addText(fieldLabel(numTarget), 20, 34, 2, C_INK, C_CARD, TL_DATUM);
  fbRoundRect(16, 54, 288, 34, 5, C_BG, C_LINE);
  addText(numBuf[0] ? numBuf : "0", 296, 71, 4, numBuf[0] ? C_INK : C_MUT, C_BG, MR_DATUM);
  if (numError) addText("Invalid number", 20, 98, 1, C_DANG, C_CARD, TL_DATUM);
  static const char *keys[16] = {
    "7","8","9","DEL", "4","5","6","+/-", "1","2","3","CLR", "0",".","ESC","OK"
  };
  for (int i = 0; i < 16; i++) {
    int x = 12 + (i % 4) * 76, y = 104 + (i / 4) * 33;
    int style = 0;
    if (!strcmp(keys[i], "OK")) style = 2;
    else if (!strcmp(keys[i], "ESC") || !strcmp(keys[i], "DEL")) style = 4;
    else if (!strcmp(keys[i], "+/-") || !strcmp(keys[i], "CLR")) style = 1;
    drawButton(x, y, 72, 30, keys[i], B_NUM, i, style, strlen(keys[i]) > 3 ? 1 : 2);
  }
}
static void drawField(int x, int y, int w, int h, int id) {
  double v = 0; fieldValue(id, v);
  char buf[32]; formatDecimal(v, buf, sizeof(buf));
  fbRoundRect(x, y, w, h, 5, C_CARD, C_LINE);
  addText(fieldLabel(id), x + 7, y + 5, 1, C_MUT, C_CARD, TL_DATUM);
  addText(buf, x + 7, y + h - 6, 2, C_INK, C_CARD, BL_DATUM);
  addHit(x, y, w, h, B_FIELD, id);
}

// ---------------- top mode tabs ----------------------------------------
static void drawTopTabs() {
  fbFillRect(0, 0, SCREEN_W, 27, C_CARD);
  fbFillRect(0, 26, SCREEN_W, 1, C_LINE);
  static const char *names[5] = { "CALC", "GRAPH", "TABLE", "STAT", "EQN" };
  for (int i = 0; i < 5; i++) {
    int x = i * 64;
    if ((int)mode == i && !graphEditing) fbFillRect(x + 6, 23, 52, 3, C_ACC);
    addText(names[i], x + 32, 12, 1,
            ((int)mode == i) ? C_ACC : C_MUT, C_CARD, MC_DATUM);
    addHit(x, 0, 64, 26, B_MODE, i);
  }
}

// ---------------- expression text with cursor + window ------------------
static void drawExprText(int x, int y, const String &e, int cur) {
  // y = top of font-2 line. Shows a 25-char window that follows the cursor.
  int len = (int)e.length();
  cur = clampI(cur, 0, len);
  int start = 0;
  const int maxCh = 25;
  if (cur > maxCh) start = cur - maxCh;
  if (start + maxCh > len && len > maxCh) start = len - maxCh;
  if (start < 0) start = 0;
  String win = e.substring(start, (start + maxCh < len) ? start + maxCh : len);
  if (win.length() == 0)
    addText("...", x, y, 2, C_MUT, C_CARD, TL_DATUM);
  else
    addText(win.c_str(), x, y, 2, C_INK, C_CARD, TL_DATUM);
  if (cursorVisible) {
    String pre = (cur > start) ? win.substring(0, cur - start) : String("");
    canvas.setTextFont(2);
    int cw = canvas.textWidth(pre);
    int cx = x + cw;
    if (cx < SCREEN_W - 4) fbFillRect(cx, y + 1, 2, 16, C_ACC);
  }
}


// ---------------- natural mathematical typesetting ---------------------
// Renders the MathTree (structure) into the PSRAM canvas: stacked
// fractions, extending radicals, superscripts, |abs| bars, floor/ceil
// brackets, pi/tau/phi glyphs (drawn, since TFT fonts lack them),
// sin^-1 style inverses, log subscripts, nPr/nCr notation.
// Two passes per expression: measure (also locates the editing cursor
// in source-character space), then draw with horizontal scrolling so
// the cursor stays visible. If the natural layout cannot fit the
// allotted height, it degrades to a linear layout, and if the tree
// cannot be built at all, to the plain-text renderer. The underlying
// ASCII expression is never modified by any of this.

static MathTree g_tree;
static const char *g_treeSrc = "";
static bool g_renderLinear = false;
static uint16_t g_exprBg = C_CARD;  // background painted behind glyph boxes
static int g_renderCursor = -1;
static bool g_curFound = false;
static int g_curX = 0, g_curTop = 0, g_curBot = 0;

static int mAsc(int level) { return level <= 0 ? 9 : 5; }
static int mDesc(int level) { return level <= 0 ? 5 : 3; }
static uint8_t mFont(int level) { return (uint8_t)(level <= 0 ? 2 : 1); }
static int mTextW(const char *s, int level) {
  canvas.setTextFont(mFont(level));
  return canvas.textWidth(s);
}
static void mDrawText(const char *s, int x, int baseline, int level, uint16_t col) {
  if (!s || !*s) return;
  canvas.setTextColor(col, g_exprBg);
  canvas.setTextDatum(ML_DATUM);
  canvas.drawString(String(s), x, baseline + (mAsc(level) - mDesc(level)) / 2, mFont(level));
}
static int mGlyphW(int level) { return level <= 0 ? 11 : 6; }  // pi/tau/phi
static void mDrawPi(int x, int baseline, int level, uint16_t col) {
  int top = baseline - mAsc(level), bot = baseline + mDesc(level);
  if (level <= 0) {
    fbFillRect(x + 1, top, 9, 2, col);
    fbFillRect(x + 2, top, 2, bot - top, col);
    fbFillRect(x + 7, top, 2, bot - top, col);
  } else {
    fbFillRect(x, top, 5, 1, col);
    fbFillRect(x + 1, top, 1, bot - top, col);
    fbFillRect(x + 3, top, 1, bot - top, col);
  }
}
static void mDrawTau(int x, int baseline, int level, uint16_t col) {
  int top = baseline - mAsc(level), bot = baseline + mDesc(level);
  if (level <= 0) {
    fbFillRect(x + 1, top, 9, 2, col);
    fbFillRect(x + 5, top, 2, bot - top - 2, col);
    fbFillRect(x + 3, bot - 3, 4, 2, col);
  } else {
    fbFillRect(x, top, 5, 1, col);
    fbFillRect(x + 2, top, 1, bot - top - 1, col);
    fbFillRect(x + 1, bot - 2, 3, 1, col);
  }
}
static void mDrawPhi(int x, int baseline, int level, uint16_t col) {
  int top = baseline - mAsc(level), bot = baseline + mDesc(level);
  int cx = x + mGlyphW(level) / 2, cy = (top + bot) / 2;
  int r = (bot - top) / 2 - 1; if (r < 2) r = 2;
  fbCircle(cx, cy, r, col);
  fbLine(cx, top - 1, cx, bot + 1, col);
}
static void mDrawTimes(int x, int baseline, int level, uint16_t col) {
  int half = level <= 0 ? 4 : 2;
  int cx = x + half + 1, cy = baseline;
  fbLine(cx - half, cy - half, cx + half, cy + half, col);
  fbLine(cx - half, cy + half, cx + half, cy - half, col);
}

struct MMet { int w, asc, desc; };
static MMet mMeasure(int ni, int level);
static void mLayout(int ni, int x, int baseline, int level, bool draw);

static const MNode &mN(int ni) { return g_tree.nodes[ni]; }
static void mSpanText(const MNode &n, char *buf, size_t bufSize) {
  int len = n.end - n.start;
  if (len < 0) len = 0;
  if ((size_t)len >= bufSize) len = (int)bufSize - 1;
  memcpy(buf, g_treeSrc + n.start, len);
  buf[len] = '\0';
}
static int mPrec(const MNode &n) {
  switch (n.type) {
    case MN_ADD: case MN_SUB: return 1;
    case MN_MUL: case MN_DIV: case MN_MOD: return 2;
    case MN_NEG: return 3;
    case MN_POW: return 4;
    case MN_FACT: case MN_PERCENT: return 5;
    default: return 9;
  }
}
static bool mWrapBinary(const MNode &parent, const MNode &child, bool rightSide) {
  int pp = mPrec(parent), cp = mPrec(child);
  if (cp < pp) return true;
  if (rightSide && cp == pp &&
      (parent.type == MN_SUB || parent.type == MN_DIV)) return true;
  return false;
}
static MMet mMeasureWrapped(int child, bool wrap, int level) {
  MMet m = mMeasure(child, level);
  if (wrap) m.w += mTextW("(", level) + mTextW(")", level);
  return m;
}
static void mLayoutWrapped(int child, int x, int baseline, int level, bool draw, bool wrap) {
  if (wrap) {
    if (draw) mDrawText("(", x, baseline, level, C_INK);
    x += mTextW("(", level);
  }
  mLayout(child, x, baseline, level, draw);
  if (wrap) {
    MMet cm = mMeasure(child, level);
    if (draw) mDrawText(")", x + cm.w, baseline, level, C_INK);
  }
}
// function display name: base, optional superscript ("-1"), subscript ("10")
static void mFuncNameParts(const char *name, const char *&base, const char *&sup, const char *&sub) {
  base = name; sup = nullptr; sub = nullptr;
  struct Inv { const char *full, *base; };
  static const Inv inv[] = {
    {"asin","sin"},{"acos","cos"},{"atan","tan"},{"asec","sec"},{"acsc","csc"},
    {"acot","cot"},{"asinh","sinh"},{"acosh","cosh"},{"atanh","tanh"}
  };
  for (size_t i = 0; i < sizeof(inv)/sizeof(inv[0]); i++)
    if (!strcmp(name, inv[i].full)) { base = inv[i].base; sup = "-1"; return; }
  if (!strcmp(name, "log")) { base = "log"; sub = "10"; }
  else if (!strcmp(name, "log2")) { base = "log"; sub = "2"; }
}
static int mFuncNameW(const char *name, int level) {
  const char *base, *sup, *sub;
  mFuncNameParts(name, base, sup, sub);
  int w = mTextW(base, level);
  if (sup) w += mTextW(sup, level + 1);
  if (sub) w += mTextW(sub, level + 1);
  return w;
}
static void mDrawFuncName(const char *name, int x, int baseline, int level, bool draw) {
  const char *base, *sup, *sub;
  mFuncNameParts(name, base, sup, sub);
  if (draw) mDrawText(base, x, baseline, level, C_INK);
  int bx = x + mTextW(base, level);
  if (sup) { if (draw) mDrawText(sup, bx, baseline - (level <= 0 ? 7 : 4), level + 1, C_INK); }
  if (sub) { if (draw) mDrawText(sub, bx, baseline + (level <= 0 ? 5 : 3), level + 1, C_INK); }
}

static MMet mMeasure(int ni, int level) {
  const MNode &n = mN(ni);
  MMet m = { 0, mAsc(level), mDesc(level) };
  char buf[28];
  switch (n.type) {
    case MN_NUM:
      mSpanText(n, buf, sizeof(buf));
      m.w = mTextW(buf, level);
      break;
    case MN_CONST:
      if (!strcmp(n.name, "pi") || !strcmp(n.name, "tau") || !strcmp(n.name, "phi")) m.w = mGlyphW(level);
      else m.w = mTextW(n.name, level);
      break;
    case MN_VAR: case MN_SYMBOL:
      m.w = mTextW(n.name, level);
      break;
    case MN_NEG: {
      MMet c = mMeasureWrapped(n.a, mPrec(mN(n.a)) < 3, level);
      m.w = mTextW("-", level) + c.w; m.asc = c.asc; m.desc = c.desc;
      break;
    }
    case MN_ADD: case MN_SUB: {
      MMet l = mMeasureWrapped(n.a, mWrapBinary(n, mN(n.a), false), level);
      MMet r = mMeasureWrapped(n.b, mWrapBinary(n, mN(n.b), true), level);
      m.w = l.w + mTextW(n.type == MN_ADD ? " + " : " - ", level) + r.w;
      m.asc = l.asc > r.asc ? l.asc : r.asc;
      m.desc = l.desc > r.desc ? l.desc : r.desc;
      break;
    }
    case MN_MUL: {
      MMet l = mMeasureWrapped(n.a, mWrapBinary(n, mN(n.a), false), level);
      MMet r = mMeasureWrapped(n.b, mWrapBinary(n, mN(n.b), true), level);
      int mid = (n.flags & MFLAG_IMPLICIT) ? 1 : (mGlyphW(level) + 2);
      m.w = l.w + mid + r.w;
      m.asc = l.asc > r.asc ? l.asc : r.asc;
      m.desc = l.desc > r.desc ? l.desc : r.desc;
      break;
    }
    case MN_DIV: {
      if (g_renderLinear) {
        MMet l = mMeasureWrapped(n.a, mWrapBinary(n, mN(n.a), false), level);
        MMet r = mMeasureWrapped(n.b, true, level);
        m.w = l.w + mTextW("/", level) + r.w;
        m.asc = l.asc > r.asc ? l.asc : r.asc;
        m.desc = l.desc > r.desc ? l.desc : r.desc;
      } else {
        MMet num = mMeasure(n.a, level + 1);
        MMet den = mMeasure(n.b, level + 1);
        m.w = (num.w > den.w ? num.w : den.w) + 8;
        m.asc = num.asc + num.desc + 5;
        m.desc = den.asc + den.desc + 3;
      }
      break;
    }
    case MN_MOD: {
      MMet l = mMeasureWrapped(n.a, mWrapBinary(n, mN(n.a), false), level);
      MMet r = mMeasureWrapped(n.b, mWrapBinary(n, mN(n.b), true), level);
      m.w = l.w + mTextW(" mod ", level) + r.w;
      m.asc = l.asc > r.asc ? l.asc : r.asc;
      m.desc = l.desc > r.desc ? l.desc : r.desc;
      break;
    }
    case MN_POW: {
      bool wrapB = mPrec(mN(n.a)) < 4 || mN(n.a).type == MN_NEG || mN(n.a).type == MN_DIV;
      MMet bs = mMeasureWrapped(n.a, wrapB, level);
      MMet ex = mMeasure(n.b, level + 1);
      int raise = level <= 0 ? 7 : 4;
      m.w = bs.w + ex.w + 1;
      m.asc = bs.asc > (raise + ex.asc) ? bs.asc : (raise + ex.asc);
      m.desc = bs.desc;
      break;
    }
    case MN_FACT: {
      MMet c = mMeasureWrapped(n.a, mPrec(mN(n.a)) < 5, level);
      m.w = c.w + mTextW("!", level); m.asc = c.asc; m.desc = c.desc;
      break;
    }
    case MN_PERCENT: {
      MMet c = mMeasureWrapped(n.a, mPrec(mN(n.a)) < 5, level);
      m.w = c.w + mTextW("%", level); m.asc = c.asc; m.desc = c.desc;
      break;
    }
    case MN_FUNC: {
      MMet arg = mMeasure(n.a, level + 1);
      if (!strcmp(n.name, "sqrt") || !strcmp(n.name, "cbrt")) {
        int radW = level <= 0 ? 10 : 7;
        m.w = radW + arg.w + 2; m.asc = arg.asc + 3; m.desc = arg.desc;
      } else if (!strcmp(n.name, "abs")) {
        m.w = arg.w + 10; m.asc = arg.asc + 2; m.desc = arg.desc + 2;
      } else if (!strcmp(n.name, "floor") || !strcmp(n.name, "ceil")) {
        m.w = arg.w + 12; m.asc = arg.asc + 2; m.desc = arg.desc + 2;
      } else {
        MMet a2 = mMeasure(n.a, level);
        m.w = mFuncNameW(n.name, level) + mTextW("(", level) + a2.w + mTextW(")", level);
        m.asc = a2.asc > mAsc(level) ? a2.asc : mAsc(level);
        m.desc = a2.desc > mDesc(level) ? a2.desc : mDesc(level);
      }
      break;
    }
    case MN_FUNC2: {
      if (!strcmp(n.name, "nPr") || !strcmp(n.name, "nCr")) {
        MMet a1 = mMeasure(n.a, level);
        MMet a2 = mMeasure(n.b, level + 1);
        m.w = a1.w + mTextW(n.name[1] == 'P' ? "P" : "C", level) + a2.w;
        m.asc = a1.asc;
        int d = a2.desc + (level <= 0 ? 4 : 2);
        m.desc = a1.desc > d ? a1.desc : d;
      } else {
        MMet a1 = mMeasure(n.a, level);
        MMet a2 = mMeasure(n.b, level);
        m.w = mTextW(n.name, level) + mTextW("(", level) + a1.w +
              mTextW(", ", level) + a2.w + mTextW(")", level);
        m.asc = a1.asc > a2.asc ? a1.asc : a2.asc;
        m.desc = a1.desc > a2.desc ? a1.desc : a2.desc;
      }
      break;
    }
    default: break;
  }
  return m;
}

static void mRecordCursor(int x, int baseline, const MMet &m) {
  g_curFound = true;
  g_curX = x;
  g_curTop = baseline - m.asc;
  g_curBot = baseline + m.desc;
}

static void mRecordCursorInToken(int x, int baseline, const MMet &self,
                                 int srcStart, int srcEnd, int drawnW) {
  // cursor strictly inside a token: interpolate across its drawn width
  if (g_renderCursor > srcStart && g_renderCursor < srcEnd && srcEnd > srcStart) {
    int cx = x + drawnW * (g_renderCursor - srcStart) / (srcEnd - srcStart);
    mRecordCursor(cx, baseline, self);
  }
}
static void mLayout(int ni, int x, int baseline, int level, bool draw);

static void mLayout(int ni, int x, int baseline, int level, bool draw) {
  const MNode &n = mN(ni);
  MMet self = mMeasure(ni, level);
  if (g_renderCursor == n.start) mRecordCursor(x, baseline, self);
  char buf[28];
  switch (n.type) {
    case MN_NUM: {
      mSpanText(n, buf, sizeof(buf));
      if (g_renderCursor > n.start && g_renderCursor < n.end) {
        char pre[28];
        int plen = g_renderCursor - n.start;
        if (plen >= (int)sizeof(pre)) plen = (int)sizeof(pre) - 1;
        memcpy(pre, buf, plen); pre[plen] = '\0';
        MMet pm = self; pm.w = mTextW(pre, level);
        mRecordCursor(x + pm.w, baseline, self);
      }
      if (draw) mDrawText(buf, x, baseline, level, C_INK);
      break;
    }
    case MN_CONST: {
      mRecordCursorInToken(x, baseline, self, n.start, n.end, self.w);
      if (!strcmp(n.name, "pi")) { if (draw) mDrawPi(x, baseline, level, C_INK); }
      else if (!strcmp(n.name, "tau")) { if (draw) mDrawTau(x, baseline, level, C_INK); }
      else if (!strcmp(n.name, "phi")) { if (draw) mDrawPhi(x, baseline, level, C_INK); }
      else if (draw) mDrawText(n.name, x, baseline, level, C_INK);
      break;
    }
    case MN_VAR: case MN_SYMBOL:
      if (draw) mDrawText(n.name, x, baseline, level, C_INK);
      break;
    case MN_NEG: {
      bool wrap = mPrec(mN(n.a)) < 3;
      if (draw) mDrawText("-", x, baseline, level, C_INK);
      mLayoutWrapped(n.a, x + mTextW("-", level), baseline, level, draw, wrap);
      break;
    }
    case MN_ADD: case MN_SUB: case MN_MOD: {
      bool wl = mWrapBinary(n, mN(n.a), false);
      bool wr = mWrapBinary(n, mN(n.b), true);
      MMet lm = mMeasureWrapped(n.a, wl, level);
      const char *op = n.type == MN_ADD ? " + " : (n.type == MN_SUB ? " - " : " mod ");
      mLayoutWrapped(n.a, x, baseline, level, draw, wl);
      if (draw) mDrawText(op, x + lm.w, baseline, level, C_INK);
      mLayoutWrapped(n.b, x + lm.w + mTextW(op, level), baseline, level, draw, wr);
      break;
    }
    case MN_MUL: {
      bool wl = mWrapBinary(n, mN(n.a), false);
      bool wr = mWrapBinary(n, mN(n.b), true);
      MMet lm = mMeasureWrapped(n.a, wl, level);
      bool impl = (n.flags & MFLAG_IMPLICIT) != 0;
      int mid = impl ? 1 : (mGlyphW(level) + 2);
      mLayoutWrapped(n.a, x, baseline, level, draw, wl);
      if (!impl && draw) mDrawTimes(x + lm.w, baseline, level, C_INK);
      mLayoutWrapped(n.b, x + lm.w + mid, baseline, level, draw, wr);
      break;
    }
    case MN_DIV: {
      if (g_renderLinear) {
        bool wl = mWrapBinary(n, mN(n.a), false);
        MMet lm = mMeasureWrapped(n.a, wl, level);
        mLayoutWrapped(n.a, x, baseline, level, draw, wl);
        if (draw) mDrawText("/", x + lm.w, baseline, level, C_INK);
        mLayoutWrapped(n.b, x + lm.w + mTextW("/", level), baseline, level, draw, true);
      } else {
        MMet num = mMeasure(n.a, level + 1);
        MMet den = mMeasure(n.b, level + 1);
        int barTop = baseline - 2;
        mLayout(n.a, x + (self.w - num.w) / 2, barTop - 3 - num.desc, level + 1, draw);
        if (draw) fbFillRect(x, barTop, self.w, level <= 0 ? 2 : 1, C_INK);
        mLayout(n.b, x + (self.w - den.w) / 2, barTop + 2 + 3 + den.asc, level + 1, draw);
      }
      break;
    }
    case MN_POW: {
      bool wrapB = mPrec(mN(n.a)) < 4 || mN(n.a).type == MN_NEG || mN(n.a).type == MN_DIV;
      MMet bm = mMeasureWrapped(n.a, wrapB, level);
      mLayoutWrapped(n.a, x, baseline, level, draw, wrapB);
      int raise = level <= 0 ? 7 : 4;
      mLayout(n.b, x + bm.w + 1, baseline - raise, level + 1, draw);
      break;
    }
    case MN_FACT: case MN_PERCENT: {
      bool wrap = mPrec(mN(n.a)) < 5;
      MMet cm = mMeasureWrapped(n.a, wrap, level);
      mLayoutWrapped(n.a, x, baseline, level, draw, wrap);
      if (draw) mDrawText(n.type == MN_FACT ? "!" : "%", x + cm.w, baseline, level, C_INK);
      break;
    }
    case MN_FUNC: {
      MMet arg = mMeasure(n.a, level + 1);
      // source characters of "name(" precede the argument: map the
      // cursor proportionally onto the drawn radical/bracket/name zone
      if (g_renderCursor > n.start && g_renderCursor < mN(n.a).start) {
        int span = mN(n.a).start - n.start;
        int zone = self.w - arg.w;
        if (span > 0 && zone > 0)
          mRecordCursor(x + zone * (g_renderCursor - n.start) / span, baseline, self);
      }
      if (!strcmp(n.name, "sqrt") || !strcmp(n.name, "cbrt")) {
        int radW = level <= 0 ? 10 : 7;
        int argMAsc = arg.asc;
        mLayout(n.a, x + radW, baseline, level + 1, draw);
        if (draw) {
          int barY = baseline - argMAsc - 3;
          fbLine(x + 1, baseline - 1, x + 3, baseline + 2, C_INK);
          fbLine(x + 3, baseline + 2, x + radW - 2, barY, C_INK);
          fbLine(x + radW - 2, barY, x + self.w, barY, C_INK);
          fbLine(x + self.w, barY, x + self.w, barY + 3, C_INK);
          if (!strcmp(n.name, "cbrt")) mDrawText("3", x + 1, barY + 2, level + 1, C_INK);
        }
      } else if (!strcmp(n.name, "abs") || !strcmp(n.name, "floor") || !strcmp(n.name, "ceil")) {
        bool isAbs = !strcmp(n.name, "abs");
        bool isFloor = !strcmp(n.name, "floor");
        mLayout(n.a, x + (isAbs ? 5 : 6), baseline, level + 1, draw);
        if (draw) {
          int top = baseline - self.asc, bot = baseline + self.desc;
          int lx = x + 2, rx = x + self.w - 3;
          fbLine(lx, top, lx, bot, C_INK);
          fbLine(rx, top, rx, bot, C_INK);
          if (!isAbs) {
            if (isFloor) { fbLine(lx, bot, lx + 4, bot, C_INK); fbLine(rx - 4, bot, rx, bot, C_INK); }
            else { fbLine(lx, top, lx + 4, top, C_INK); fbLine(rx - 4, top, rx, top, C_INK); }
          }
        }
      } else {
        MMet a2 = mMeasure(n.a, level);
        int nw = mFuncNameW(n.name, level);
        int nameLen = (int)strlen(n.name);
        mRecordCursorInToken(x, baseline, self, n.start, n.start + nameLen, nw);
        if (g_renderCursor == n.start + nameLen)  // the '(' itself
          mRecordCursor(x + nw, baseline, self);
        if (g_renderCursor == n.end - 1)  // just before ')'
          mRecordCursor(x + self.w - mTextW(")", level), baseline, self);
        mDrawFuncName(n.name, x, baseline, level, draw);
        if (draw) mDrawText("(", x + nw, baseline, level, C_INK);
        mLayout(n.a, x + nw + mTextW("(", level), baseline, level, draw);
        if (draw) mDrawText(")", x + nw + mTextW("(", level) + a2.w, baseline, level, C_INK);
      }
      break;
    }
    case MN_FUNC2: {
      if (!strcmp(n.name, "nPr") || !strcmp(n.name, "nCr")) {
        MMet a1 = mMeasure(n.a, level);
        if (g_renderCursor > n.start && g_renderCursor < mN(n.a).start)
          mRecordCursor(x, baseline, self);  // "nCr(" zone draws before arg1 visually as 5C2
        if (g_renderCursor == mN(n.a).end)  // comma sits between the two args
          mRecordCursor(x + a1.w + mTextW(n.name[1] == 'P' ? "P" : "C", level), baseline, self);
        if (g_renderCursor == n.end - 1)
          mRecordCursor(x + self.w - 1, baseline, self);
        mLayout(n.a, x, baseline, level, draw);
        const char *pc = n.name[1] == 'P' ? "P" : "C";
        if (draw) mDrawText(pc, x + a1.w, baseline, level, C_INK);
        mLayout(n.b, x + a1.w + mTextW(pc, level), baseline + (level <= 0 ? 4 : 2), level + 1, draw);
      } else {
        MMet a1 = mMeasure(n.a, level);
        int nw = mTextW(n.name, level);
        int nameLen = (int)strlen(n.name);
        mRecordCursorInToken(x, baseline, self, n.start, n.start + nameLen, nw);
        if (g_renderCursor == n.start + nameLen)
          mRecordCursor(x + nw, baseline, self);
        if (g_renderCursor == mN(n.a).end)  // comma
          mRecordCursor(x + nw + mTextW("(", level) + a1.w, baseline, self);
        if (g_renderCursor == n.end - 1)
          mRecordCursor(x + self.w - mTextW(")", level), baseline, self);
        if (draw) { mDrawText(n.name, x, baseline, level, C_INK); mDrawText("(", x + nw, baseline, level, C_INK); }
        int ax = x + nw + mTextW("(", level);
        mLayout(n.a, ax, baseline, level, draw);
        if (draw) mDrawText(", ", ax + a1.w, baseline, level, C_INK);
        mLayout(n.b, ax + a1.w + mTextW(", ", level), baseline, level, draw);
        MMet a2 = mMeasure(n.b, level);
        if (draw) mDrawText(")", ax + a1.w + mTextW(", ", level) + a2.w, baseline, level, C_INK);
      }
      break;
    }
    default: break;
  }
  if (g_renderCursor == n.end && n.end > n.start) mRecordCursor(x + self.w, baseline, self);
}

// Plain fallback (unparseable / cannot fit): raw text window, no cursor
// unless one is requested through drawExprText by the caller.
static void renderPlainWindow(const String &e, int x, int y, int w) {
  int maxCh = w / 11; if (maxCh < 1) maxCh = 1;
  String win = (int)e.length() > maxCh ? e.substring(e.length() - maxCh) : e;
  addText(win.length() ? win.c_str() : "...", x, y, 2,
          win.length() ? C_INK : C_MUT, g_exprBg, TL_DATUM);
}

// Typesets 'expr' inside rect (x,y,w,h). cursorPos is a source char
// index, or -1 for no cursor (labels, history, table header).
static void renderExpression(const String &expr, int cursorPos,
                             int x, int y, int w, int h, int baseLevel,
                             uint16_t bg = C_CARD) {
  g_exprBg = bg;
  if ((int)expr.length() == 0) {
    addText("...", x + 2, y + h / 2, 2, C_MUT, C_CARD, ML_DATUM);
    if (cursorPos >= 0 && cursorVisible) fbFillRect(x + 2, y + 4, 2, h - 8, C_ACC);
    return;
  }
  if (!parseMathTree(expr.c_str(), g_tree)) {
    if (cursorPos >= 0) drawExprText(x, y + (h - 18) / 2, expr, cursorPos);
    else renderPlainWindow(expr, x, y + (h - 18) / 2, w);
    return;
  }
  g_treeSrc = expr.c_str();
  g_renderLinear = false;
  MMet rootM = mMeasure(g_tree.root, baseLevel);
  if (rootM.asc + rootM.desc > h) {
    g_renderLinear = true;
    rootM = mMeasure(g_tree.root, baseLevel);
    if (rootM.asc + rootM.desc > h) {  // still too tall: plain text
      if (cursorPos >= 0) drawExprText(x, y + (h - 18) / 2, expr, cursorPos);
      else renderPlainWindow(expr, x, y + (h - 18) / 2, w);
      return;
    }
  }
  int baseline = y + (h - (rootM.asc + rootM.desc)) / 2 + rootM.asc;
  // pass 1 (no draw): locate cursor in layout space
  int scroll = 0;
  g_renderCursor = cursorPos;
  if (cursorPos >= 0) {
    g_curFound = false;
    mLayout(g_tree.root, 0, 0, baseLevel, false);
    if (rootM.w > w && g_curFound)
      scroll = clampI(g_curX - (w * 3) / 4, 0, rootM.w - w + 10);
  }
  // pass 2: draw
  g_curFound = false;
  mLayout(g_tree.root, x - scroll, baseline, baseLevel, true);
  if (cursorPos >= 0 && cursorVisible && g_curFound &&
      g_curX >= x && g_curX < x + w) {
    int top = g_curTop < y ? y : g_curTop;
    int bot = g_curBot > y + h ? y + h : g_curBot;
    if (bot > top) fbFillRect(g_curX, top, 2, bot - top, C_ACC);
  }
  g_renderCursor = -1;
}

// ---------------- CALC screen -------------------------------------------
static void drawCalcScreen() {
  fbRoundRect(4, 30, 312, 66, 6, C_CARD, C_LINE);
  drawButton(8, 34, 48, 17, angleDeg ? "DEG" : "RAD", B_DEG, 0, 5, 1);
  drawButton(60, 34, 44, 17, "F/D", B_FD, 0, preferFraction ? 3 : 5, 1);
  addText(preferFraction ? "p/q" : "dec", 128, 42, 1, C_MUT, C_CARD, ML_DATUM);
  drawButton(264, 34, 48, 17, "HIST", B_HIST, 0, 5, 1);
  renderExpression(calcExpr, calcCursor, 6, 46, 308, 36, 0);
  if (calcError[0]) {
    addText(calcError, 312, 92, 2, C_DANG, C_CARD, BR_DATUM);
  } else if (calcShowResult) {
    char pri[64], sec[64];
    formatSmart(calcResult, preferFraction, pri, sizeof(pri), sec, sizeof(sec));
    addText(pri, 312, 92, (uint8_t)(strlen(pri) <= 11 ? 4 : 2), C_INK, C_CARD, BR_DATUM);
    if (sec[0]) addText(sec, 10, 92, 1, C_MUT, C_CARD, BL_DATUM);
  } else {
    char buf[48], dec[40];
    formatDecimal(lastAns, dec, sizeof(dec));
    snprintf(buf, sizeof(buf), "Ans = %s", dec);
    addText(buf, 10, 92, 1, C_MUT, C_CARD, BL_DATUM);
  }
  drawKeypad(102);
}
static void drawGraphEditor() {
  // function selector row
  for (int i = 0; i < 3; i++) {
    char lab[8]; snprintf(lab, sizeof(lab), "%s(x)", FUNC_NAME[i]);
    fbRoundRect(4 + i * 60, 30, 56, 21, 5, i == activeFunc ? C_ACCLT : C_CARD,
                i == activeFunc ? C_ACC : C_LINE);
    addText(lab, 4 + i * 60 + 28, 40, 1, FUNC_COL[i], i == activeFunc ? C_ACCLT : C_CARD, MC_DATUM);
    addHit(4 + i * 60, 30, 56, 21, B_FUNCSEL, i);
  }
  drawButton(188, 30, 60, 21, funcVisible[activeFunc] ? "SHOWN" : "HIDDEN", B_FUNCEYE, 0,
             funcVisible[activeFunc] ? 3 : 5, 1);
  drawButton(252, 30, 64, 21, "PLOT", B_PLOTDONE, 0, 2, 1);
  // expression card
  fbRoundRect(4, 55, 312, 34, 6, C_CARD, C_LINE);
  {
    char pre[8]; snprintf(pre, sizeof(pre), "%s(x)=", FUNC_NAME[activeFunc]);
    addText(pre, 10, 72, 2, FUNC_COL[activeFunc], C_CARD, ML_DATUM);
  }
  renderExpression(funcExpr[activeFunc], funcCursor[activeFunc], 64, 55, 248, 34, 0);
  addText("123: digits   FUNC: sin/log/sqrt   ADV: floor/nPr/gcd   MORE: cursor/F-D",
          160, 94, 1, C_MUT, C_BG, MC_DATUM);
  drawKeypad(100);
}
static void drawHistory() {
  fbRoundRect(4, 4, 312, 232, 8, C_CARD, C_LINE);
  addText("HISTORY", 16, 14, 2, C_INK, C_CARD, TL_DATUM);
  drawButton(248, 10, 60, 22, "CLOSE", B_HIST_CLOSE, 0, 5, 1);
  if (histCount == 0) {
    addText("No calculations yet.", 16, 60, 1, C_MUT, C_CARD, TL_DATUM);
    return;
  }
  for (int i = 0; i < histCount; i++) {
    int idx = histCount - 1 - i;  // newest first
    int y = 40 + i * 24;
    if (y > 206) break;
    char pri[48], sec[48], res[64];
    formatSmart(hist[idx].value, preferFraction, pri, sizeof(pri), sec, sizeof(sec));
    snprintf(res, sizeof(res), "= %s", pri);
    fbRoundRect(10, y, 300, 21, 4, C_BG, C_LINE);
    renderExpression(hist[idx].expr, -1, 16, y, 210, 21, 1, C_BG);
    addText(res, 232, y + 10, 1, C_MUT, C_BG, ML_DATUM);
    addHit(10, y, 300, 21, B_HIST_ITEM, idx);
  }
  addText("Tap an entry to reuse its expression", 16, 228, 1, C_MUT, C_CARD, TL_DATUM);
}

// ---------------- GRAPH screen (Desmos-style) ---------------------------
static double niceStep(double raw) {
  double mag = pow(10.0, floor(log10(raw)));
  double norm = raw / mag;
  double s = (norm < 1.5) ? 1 : (norm < 3.5) ? 2 : (norm < 7.5) ? 5 : 10;
  return s * mag;
}
static bool evalFunc(int fi, double wx, double &wy) {
  if (funcExpr[fi].length() == 0) return false;
  EvalResult r;
  if (!evaluateExpression(funcExpr[fi].c_str(), wx, lastAns, angleDeg, r)) return false;
  wy = r.value;
  return isfinite(wy);
}
static void drawPlot() {
  fbRoundRect(PLOT_X, PLOT_Y, PLOT_W, PLOT_H, 4, C_CARD, C_LINE);
  double left = viewCx - (PLOT_W * 0.5) / viewScale;
  double right = viewCx + (PLOT_W * 0.5) / viewScale;
  double topW = viewCy + (PLOT_H * 0.5) / viewScale;
  double botW = viewCy - (PLOT_H * 0.5) / viewScale;
  auto sxOf = [&](double wx) { return PLOT_X + (int)((wx - left) / (right - left) * PLOT_W); };
  auto syOf = [&](double wy) { return PLOT_Y + (int)((topW - wy) / (topW - botW) * PLOT_H); };
  double step = niceStep(48.0 / viewScale);
  // grid + labels
  for (double wx = ceil(left / step) * step; wx <= right; wx += step) {
    int sx = sxOf(wx);
    if (sx <= PLOT_X || sx >= PLOT_X + PLOT_W) continue;
    fbFillRect(sx, PLOT_Y + 1, 1, PLOT_H - 2, fabs(wx) < step * 0.01 ? C_MUT : C_GRID);
    if (fabs(wx) > step * 0.01) {
      char b[20]; formatAxis(wx, b, sizeof(b));
      addText(b, sx, PLOT_Y + PLOT_H - 3, 1, C_MUT, C_CARD, BC_DATUM);
    }
  }
  for (double wy = ceil(botW / step) * step; wy <= topW; wy += step) {
    int sy = syOf(wy);
    if (sy <= PLOT_Y || sy >= PLOT_Y + PLOT_H) continue;
    fbFillRect(PLOT_X + 1, sy, PLOT_W - 2, 1, fabs(wy) < step * 0.01 ? C_MUT : C_GRID);
    if (fabs(wy) > step * 0.01) {
      char b[20]; formatAxis(wy, b, sizeof(b));
      addText(b, PLOT_X + 4, sy, 1, C_MUT, C_CARD, ML_DATUM);
    }
  }
  // curves
  for (int fi = 0; fi < 3; fi++) {
    if (!funcVisible[fi] || funcExpr[fi].length() == 0) continue;
    bool havePrev = false;
    int prevX = 0, prevY = 0;
    for (int px = 0; px < PLOT_W; px++) {
      double wx = left + (px + 0.5) / PLOT_W * (right - left);
      double wy;
      if (!evalFunc(fi, wx, wy)) { havePrev = false; continue; }
      int sx = PLOT_X + px;
      int sy = syOf(wy);
      if (havePrev && abs(sy - prevY) <= PLOT_H) {  // break asymptotes
        int y0 = clampI(prevY, PLOT_Y - 300, PLOT_Y + PLOT_H + 300);
        int y1 = clampI(sy, PLOT_Y - 300, PLOT_Y + PLOT_H + 300);
        fbLine(prevX, y0, sx, y1, FUNC_COL[fi]);
        fbLine(prevX, y0 + 1, sx, y1 + 1, FUNC_COL[fi]);
      }
      if (sy >= PLOT_Y && sy < PLOT_Y + PLOT_H) fbPixel(sx, sy, FUNC_COL[fi]);
      prevX = sx; prevY = sy; havePrev = true;
    }
  }
  // trace marker
  if (traceOn) {
    int sx = sxOf(traceX);
    if (sx >= PLOT_X && sx < PLOT_X + PLOT_W) {
      for (int yy = PLOT_Y + 2; yy < PLOT_Y + PLOT_H - 2; yy += 2) fbPixel(sx, yy, C_ACC);
      double wy;
      if (evalFunc(activeFunc, traceX, wy)) {
        int sy = syOf(wy);
        if (sy >= PLOT_Y && sy < PLOT_Y + PLOT_H) {
          fbFillCircle(sx, sy, 5, C_CARD);
          fbCircle(sx, sy, 5, FUNC_COL[activeFunc]);
          fbFillCircle(sx, sy, 2, FUNC_COL[activeFunc]);
        }
        char xb[32], yb[32], line[80];
        fmtVal(traceX, xb, sizeof(xb));
        fmtVal(wy, yb, sizeof(yb));
        snprintf(line, sizeof(line), "x=%s  %s(x)=%s", xb, FUNC_NAME[activeFunc], yb);
        addText(line, PLOT_X + 6, PLOT_Y + 6, 1, C_INK, C_CARD, TL_DATUM);
        // slope f'(x) by central difference
        double h = (right - left) / PLOT_W;
        double ya, ybb;
        if (evalFunc(activeFunc, traceX - h, ya) && evalFunc(activeFunc, traceX + h, ybb)) {
          char sb[32], line2[48];
          fmtVal((ybb - ya) / (2 * h), sb, sizeof(sb));
          snprintf(line2, sizeof(line2), "slope = %s", sb);
          addText(line2, PLOT_X + 6, PLOT_Y + 18, 1, C_MUT, C_CARD, TL_DATUM);
        }
      } else {
        addText("undefined here", PLOT_X + 6, PLOT_Y + 6, 1, C_DANG, C_CARD, TL_DATUM);
      }
    }
  }
  // function chips (tap to select trace/edit target)
  for (int i = 0; i < 3; i++) {
    int x = PLOT_X + PLOT_W - 100 + i * 33;
    uint16_t fill = (i == activeFunc) ? FUNC_COL[i] : C_CARD;
    fbRoundRect(x, PLOT_Y + 4, 29, 17, 4, fill, FUNC_COL[i]);
    addText(FUNC_NAME[i], x + 14, PLOT_Y + 12, 1, (i == activeFunc) ? C_CARD : FUNC_COL[i], fill, MC_DATUM);
    addHit(x, PLOT_Y + 4, 29, 17, B_FUNCSEL, i);
  }
}
static void drawGraphScreen() {
  drawPlot();
  drawButton(4,   188, 58, 24, "EDIT",  B_GRAPHTOOL, 0, 2, 1);
  drawButton(66,  188, 62, 24, "TRACE", B_GRAPHTOOL, 1, traceOn ? 3 : 0, 1);
  drawButton(132, 188, 58, 24, "ZOOM-", B_GRAPHTOOL, 2, 0, 1);
  drawButton(194, 188, 58, 24, "ZOOM+", B_GRAPHTOOL, 3, 0, 1);
  drawButton(256, 188, 60, 24, "RESET", B_GRAPHTOOL, 4, 0, 1);
  if (traceOn)
    addText("Drag across the curve to trace it", 6, 224, 1, C_MUT, C_BG, TL_DATUM);
  else
    addText("Drag to pan - pinch-free zoom above - hold 20 s = calibrate", 6, 224, 1, C_MUT, C_BG, TL_DATUM);
}

// ---------------- TABLE screen ------------------------------------------
static int tableRowCount() {
  if (tblStep == 0) return 0;
  double span = (tblEnd - tblStart) / tblStep;
  if (span < 0) return 0;
  long n = (long)floor(span + 1e-9) + 1;
  if (n > 1000) n = 1000;
  return (int)n;
}
static void drawTableScreen() {
  fbRoundRect(4, 30, 312, 26, 5, C_CARD, C_LINE);
  {
    char pre[8]; snprintf(pre, sizeof(pre), "%s(x)=", FUNC_NAME[activeFunc]);
    addText(pre, 10, 43, 2, FUNC_COL[activeFunc], C_CARD, ML_DATUM);
    renderExpression(funcExpr[activeFunc], -1, 62, 30, 182, 26, 1);
  }
  drawButton(252, 33, 58, 20, "EDIT f", B_TABLEEDIT, 0, 1, 1);
  drawField(4, 60, 100, 36, F_TBL_START);
  drawField(108, 60, 100, 36, F_TBL_END);
  drawField(212, 60, 104, 36, F_TBL_STEP);
  addText("x", 40, 108, 2, C_MUT, C_BG, MC_DATUM);
  {
    char col[16]; snprintf(col, sizeof(col), "%s(x)", FUNC_NAME[activeFunc]);
    addText(col, 160, 108, 2, C_MUT, C_BG, MC_DATUM);
  }
  int rows = tableRowCount();
  if (tblStep == 0) safeCopy(tblError, sizeof(tblError), "Step cannot be 0");
  else if (rows == 0) safeCopy(tblError, sizeof(tblError), "Check Start/End/Step");
  else tblError[0] = '\0';
  if (tblError[0]) {
    addText(tblError, 10, 130, 1, C_DANG, C_BG, TL_DATUM);
  } else {
    if (tblScroll > rows - 1) tblScroll = rows - 1;
    if (tblScroll < 0) tblScroll = 0;
    for (int i = 0; i < 8; i++) {
      int idx = tblScroll + i;
      int y = 118 + i * 14;
      if (idx >= rows) break;
      double xv = tblStart + idx * tblStep;
      char xb[32], yb[32];
      fmtVal(xv, xb, sizeof(xb));
      double wy;
      if (evalFunc(activeFunc, xv, wy)) fmtVal(wy, yb, sizeof(yb));
      else safeCopy(yb, sizeof(yb), "undef");
      if (i % 2 == 0) fbFillRect(4, y - 6, 262, 13, C565(241,245,249));
      addText(xb, 14, y, 1, C_INK, (i % 2 == 0) ? C565(241,245,249) : C_BG, TL_DATUM);
      addText(yb, 130, y, 1, C_INK, (i % 2 == 0) ? C565(241,245,249) : C_BG, TL_DATUM);
    }
    char pg[32]; snprintf(pg, sizeof(pg), "%d rows", rows);
    addText(pg, 10, 234, 1, C_MUT, C_BG, TL_DATUM);
  }
  drawButton(272, 118, 44, 34, "UP", B_TABLENAV, 0, 0, 1);
  drawButton(272, 156, 44, 34, "DOWN", B_TABLENAV, 1, 0, 1);
  drawButton(272, 194, 44, 34, "TOP", B_TABLENAV, 2, 0, 1);
}

// ---------------- STAT screen --------------------------------------------
static void computeStats(double &sum, double &mean, double &med, double &sdS,
                         double &mn, double &mx, double &q1, double &q3) {
  sum = mean = med = sdS = mn = mx = q1 = q3 = 0;
  if (statCount == 0) return;
  double sorted[40];
  for (int i = 0; i < statCount; i++) { sorted[i] = statData[i]; sum += statData[i]; }
  for (int i = 0; i < statCount; i++)
    for (int j = i + 1; j < statCount; j++)
      if (sorted[j] < sorted[i]) { double t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }
  mean = sum / statCount;
  mn = sorted[0]; mx = sorted[statCount - 1];
  auto medianOf = [&](int lo, int hi) -> double {  // [lo,hi)
    int n = hi - lo;
    if (n <= 0) return 0;
    return (n % 2) ? sorted[lo + n / 2] : (sorted[lo + n / 2 - 1] + sorted[lo + n / 2]) * 0.5;
  };
  med = medianOf(0, statCount);
  q1 = medianOf(0, statCount / 2);
  q3 = medianOf(statCount - statCount / 2, statCount);
  if (statCount > 1) {
    double acc = 0;
    for (int i = 0; i < statCount; i++) acc += (statData[i] - mean) * (statData[i] - mean);
    sdS = sqrt(acc / (statCount - 1));
  }
}
static void drawStatScreen() {
  fbRoundRect(4, 30, 312, 62, 6, C_CARD, C_LINE);
  if (statCount == 0) {
    addText("STATISTICS", 10, 40, 1, C_MUT, C_CARD, TL_DATUM);
    addText("No data yet - tap ADD DATA below.", 10, 58, 1, C_INK, C_CARD, TL_DATUM);
  } else {
    double sum, mean, med, sdS, mn, mx, q1, q3;
    computeStats(sum, mean, med, sdS, mn, mx, q1, q3);
    char b1[80], b2[96], p1[40], p2[40], p3[40];
    fmtVal(mean, p1, sizeof(p1)); fmtVal(med, p2, sizeof(p2)); fmtVal(sum, p3, sizeof(p3));
    snprintf(b1, sizeof(b1), "n=%d  sum=%s  mean=%s  med=%s", statCount, p3, p1, p2);
    addText(b1, 10, 40, 1, C_INK, C_CARD, TL_DATUM);
    char s1[28], s2[28], s3[28], s4[28], s5[28];
    fmtVal(mn, s1, sizeof(s1)); fmtVal(q1, s2, sizeof(s2)); fmtVal(med, s3, sizeof(s3));
    fmtVal(q3, s4, sizeof(s4)); fmtVal(mx, s5, sizeof(s5));
    snprintf(b2, sizeof(b2), "min %s  Q1 %s  med %s  Q3 %s  max %s", s1, s2, s3, s4, s5);
    addText(b2, 10, 54, 1, C_INK, C_CARD, TL_DATUM);
    char sd[32], b3[64];
    fmtVal(sdS, sd, sizeof(sd));
    snprintf(b3, sizeof(b3), "sample sd s = %s", sd);
    addText(b3, 10, 68, 1, C_MUT, C_CARD, TL_DATUM);
    addText("Means/medians show as p/q when exact", 10, 82, 1, C_MUT, C_CARD, TL_DATUM);
  }
  fbRoundRect(4, 96, 312, 66, 6, C_CARD, C_LINE);
  addText("DATA", 10, 104, 1, C_MUT, C_CARD, TL_DATUM);
  if (statCount == 0) {
    addText("-", 10, 122, 1, C_MUT, C_CARD, TL_DATUM);
  } else {
    String line = "";
    int ly = 118;
    for (int i = 0; i < statCount; i++) {
      char vb[24]; fmtVal(statData[i], vb, sizeof(vb));
      String item = String(vb) + (i + 1 < statCount ? ", " : "");
      if ((int)(line.length() + item.length()) > 42 && line.length()) {
        addText(line.c_str(), 10, ly, 1, C_INK, C_CARD, TL_DATUM);
        ly += 12; line = "";
        if (ly > 152) { addText("...", 10, ly, 1, C_MUT, C_CARD, TL_DATUM); break; }
      }
      line += item;
    }
    if (line.length() && ly <= 152) addText(line.c_str(), 10, ly, 1, C_INK, C_CARD, TL_DATUM);
  }
  drawButton(4, 170, 102, 28, "ADD DATA", B_STATBTN, 0, 2, 1);
  drawButton(110, 170, 96, 28, "UNDO", B_STATBTN, 1, 0, 1);
  drawButton(210, 170, 106, 28, "CLEAR ALL", B_STATBTN, 2, 4, 1);
  addText("Fractions are used for data/statistics whenever exact.", 6, 216, 1, C_MUT, C_BG, TL_DATUM);
}

// ---------------- EQN screen ----------------------------------------------
static void drawEqnScreen() {
  drawButton(4, 30, 104, 22, "QUADRATIC", B_EQNKIND, 0, !eqnLinear ? 3 : 5, 1);
  drawButton(112, 30, 120, 22, "LINEAR 2x2", B_EQNKIND, 1, eqnLinear ? 3 : 5, 1);
  if (!eqnLinear) {
    {
      canvas.setTextFont(1);
      int hx = 196;
      addText("a x", hx, 41, 1, C_MUT, C_BG, ML_DATUM);
      hx += canvas.textWidth("a x");
      addText("2", hx + 1, 36, 1, C_MUT, C_BG, ML_DATUM);  // raised superscript
      hx += 1 + canvas.textWidth("2");
      addText("+ b x + c = 0", hx + 2, 41, 1, C_MUT, C_BG, ML_DATUM);
    }
    drawField(4, 58, 100, 38, F_QUAD_A);
    drawField(108, 58, 100, 38, F_QUAD_B);
    drawField(212, 58, 104, 38, F_QUAD_C);
    fbRoundRect(4, 104, 312, 96, 6, C_CARD, C_LINE);
    char line[96], v1[40], v2[40];
    if (fabs(quadA) < 1e-15) {
      if (fabs(quadB) < 1e-15) {
        addText(fabs(quadC) < 1e-15 ? "Infinitely many solutions" : "No solution",
                12, 122, 1, C_INK, C_CARD, TL_DATUM);
      } else {
        fmtVal(-quadC / quadB, v1, sizeof(v1));
        snprintf(line, sizeof(line), "Linear:  x = %s", v1);
        addText(line, 12, 122, 1, C_INK, C_CARD, TL_DATUM);
      }
    } else {
      double disc = quadB * quadB - 4 * quadA * quadC;
      fmtVal(disc, v1, sizeof(v1));
      snprintf(line, sizeof(line), "Discriminant D = %s", v1);
      addText(line, 12, 120, 1, C_MUT, C_CARD, TL_DATUM);
      if (disc > 0) {
        double sq = sqrt(disc);
        fmtVal((-quadB + sq) / (2 * quadA), v1, sizeof(v1));
        fmtVal((-quadB - sq) / (2 * quadA), v2, sizeof(v2));
        snprintf(line, sizeof(line), "x1 = %s", v1);
        addText(line, 12, 142, 2, C_INK, C_CARD, TL_DATUM);
        snprintf(line, sizeof(line), "x2 = %s", v2);
        addText(line, 12, 166, 2, C_INK, C_CARD, TL_DATUM);
        addText("Two distinct real roots", 12, 190, 1, C_MUT, C_CARD, TL_DATUM);
      } else if (disc == 0) {
        fmtVal(-quadB / (2 * quadA), v1, sizeof(v1));
        snprintf(line, sizeof(line), "x = %s  (double root)", v1);
        addText(line, 12, 142, 2, C_INK, C_CARD, TL_DATUM);
      } else {
        double re = -quadB / (2 * quadA), im = sqrt(-disc) / fabs(2 * quadA);
        char r1[32], i1[32];
        formatDecimal(re, r1, sizeof(r1)); formatDecimal(im, i1, sizeof(i1));
        snprintf(line, sizeof(line), "x = %s + %si", r1, i1);
        addText(line, 12, 142, 1, C_INK, C_CARD, TL_DATUM);
        snprintf(line, sizeof(line), "x = %s - %si", r1, i1);
        addText(line, 12, 160, 1, C_INK, C_CARD, TL_DATUM);
        addText("Complex roots (shown decimal)", 12, 182, 1, C_MUT, C_CARD, TL_DATUM);
      }
    }
    addText("Tap a, b or c to edit. Rational roots appear as p/q.", 6, 216, 1, C_MUT, C_BG, TL_DATUM);
  } else {
    addText("a1 x + b1 y = c1     a2 x + b2 y = c2", 6, 60, 1, C_MUT, C_BG, TL_DATUM);
    for (int i = 0; i < 6; i++) {
      int x = 4 + (i % 3) * 105;
      int y = (i < 3) ? 66 : 106;
      drawField(x, y, 101, 36, F_LIN_0 + i);
    }
    fbRoundRect(4, 148, 312, 52, 6, C_CARD, C_LINE);
    double det = linV[0] * linV[4] - linV[3] * linV[1];
    char line[96], v1[40], v2[40];
    if (fabs(det) > 1e-15) {
      double xv = (linV[2] * linV[4] - linV[5] * linV[1]) / det;
      double yv = (linV[0] * linV[5] - linV[3] * linV[2]) / det;
      fmtVal(xv, v1, sizeof(v1)); fmtVal(yv, v2, sizeof(v2));
      snprintf(line, sizeof(line), "x = %s     y = %s", v1, v2);
      addText(line, 12, 168, 2, C_INK, C_CARD, TL_DATUM);
      fmtVal(det, v1, sizeof(v1));
      snprintf(line, sizeof(line), "determinant = %s", v1);
      addText(line, 12, 190, 1, C_MUT, C_CARD, TL_DATUM);
    } else {
      double numX = linV[2] * linV[4] - linV[5] * linV[1];
      double numY = linV[0] * linV[5] - linV[3] * linV[2];
      addText((fabs(numX) < 1e-15 && fabs(numY) < 1e-15) ? "Infinitely many solutions" : "No solution",
              12, 168, 1, C_INK, C_CARD, TL_DATUM);
      addText("determinant = 0", 12, 190, 1, C_MUT, C_CARD, TL_DATUM);
    }
    addText("Tap any coefficient to edit it.", 6, 216, 1, C_MUT, C_BG, TL_DATUM);
  }
}

// ---------------- master draw ----------------------------------------------
static void drawScreen() {
  hitCount = 0;
  fbClear(C_BG);
  if (inCal) drawCalScreen();
  else if (numOpen) drawNumeric();
  else if (histOpen) drawHistory();
  else {
    drawTopTabs();
    if (graphEditing) drawGraphEditor();
    else if (mode == M_CALC) drawCalcScreen();
    else if (mode == M_GRAPH) drawGraphScreen();
    else if (mode == M_TABLE) drawTableScreen();
    else if (mode == M_STAT) drawStatScreen();
    else drawEqnScreen();
  }
  pushFrame();
}

static bool overlayShown = false;
// ---------------- touch dispatch --------------------------------------------
static void dispatchHit(int id, int arg) {
  switch (id) {
    case B_MODE:
      mode = (Mode)arg; graphEditing = false; histOpen = false;
      break;
    case B_PAGE: keyPage = arg; break;
    case B_SHIFT: shiftOn = !shiftOn; break;
    case B_DEG: angleDeg = !angleDeg; SaveSettings(); break;
    case B_FD: preferFraction = !preferFraction; SaveSettings(); break;
    case B_HIST: histOpen = true; break;
    case B_HIST_CLOSE: histOpen = false; break;
    case B_HIST_ITEM:
      if (arg >= 0 && arg < histCount) {
        calcExpr = hist[arg].expr;
        calcCursor = (int)calcExpr.length();
        calcShowResult = false; calcJustEvaluated = false; calcError[0] = '\0';
      }
      histOpen = false; mode = M_CALC;
      break;
    case B_KEY: handleKey(arg); break;
    case B_FIELD: openNumeric(arg); break;
    case B_NUM: handleNumeric(arg); break;
    case B_FUNCSEL: activeFunc = clampI(arg, 0, 2); break;
    case B_FUNCEYE: funcVisible[activeFunc] = !funcVisible[activeFunc]; break;
    case B_PLOTDONE: graphEditing = false; mode = graphReturnMode; break;
    case B_GRAPHTOOL:
      if (arg == 0) { graphReturnMode = M_GRAPH; graphEditing = true; }
      else if (arg == 1) traceOn = !traceOn;
      else if (arg == 2) viewScale = clampD(viewScale / 1.3, 2.0, 400.0);
      else if (arg == 3) viewScale = clampD(viewScale * 1.3, 2.0, 400.0);
      else if (arg == 4) { viewCx = 0; viewCy = 0; viewScale = 26; traceX = 0; }
      break;
    case B_TABLENAV:
      if (arg == 0) tblScroll -= 8;
      else if (arg == 1) tblScroll += 8;
      else tblScroll = 0;
      if (tblScroll < 0) tblScroll = 0;
      break;
    case B_TABLEEDIT:
      graphReturnMode = M_TABLE; mode = M_GRAPH; graphEditing = true;
      break;
    case B_STATBTN:
      if (arg == 0) openNumeric(F_STAT_ADD);
      else if (arg == 1) { if (statCount > 0) statCount--; }
      else if (arg == 2) statCount = 0;
      break;
    case B_EQNKIND: eqnLinear = (arg == 1); break;
    default: break;
  }
  markDirty();
}
static void handleTouch() {
  if (touchEvDown) {
    for (int i = hitCount - 1; i >= 0; i--) {
      Hit &h = hits[i];
      if (touchX >= h.x && touchX < h.x + h.w && touchY >= h.y && touchY < h.y + h.h) {
        dispatchHit(h.id, h.arg);
        return;
      }
    }
    if (!numOpen && !histOpen) {
      if (mode == M_GRAPH && !graphEditing &&
          touchX >= PLOT_X && touchX < PLOT_X + PLOT_W &&
          touchY >= PLOT_Y && touchY < PLOT_Y + PLOT_H) {
        dragMode = DRAG_PLOT; dragLastX = touchX; dragLastY = touchY;
        if (traceOn) {
          traceX = viewCx + (touchX - (PLOT_X + PLOT_W / 2)) / viewScale;
          markDirty();
        }
      } else if (mode == M_TABLE && !graphEditing &&
                 touchX >= 4 && touchX < 266 && touchY >= 112 && touchY < 232) {
        dragMode = DRAG_TABLE; dragLastY = touchY; dragAccY = 0;
      }
    }
  }
  if (touchActive && dragMode == DRAG_PLOT) {
    int dx = touchX - dragLastX, dy = touchY - dragLastY;
    if (dx || dy) {
      if (traceOn) {
        traceX = viewCx + (touchX - (PLOT_X + PLOT_W / 2)) / viewScale;
      } else {
        viewCx -= dx / viewScale;
        viewCy += dy / viewScale;
      }
      dragLastX = touchX; dragLastY = touchY;
      markDirty();
    }
  } else if (touchActive && dragMode == DRAG_TABLE) {
    dragAccY += touchY - dragLastY;
    dragLastY = touchY;
    bool changed = false;
    while (dragAccY >= 14) { tblScroll--; dragAccY -= 14; changed = true; }
    while (dragAccY <= -14) { tblScroll++; dragAccY += 14; changed = true; }
    if (tblScroll < 0) tblScroll = 0;
    if (changed) markDirty();
  }
  if (touchEvUp) {
    bool hadDrag = (dragMode != DRAG_NONE);
    dragMode = DRAG_NONE;
    if (hadDrag || overlayShown) markDirty();  // erase long-press overlay
  }
}

// ---------------- 20-second long-press -> calibration -----------------------
static uint32_t holdShownSec = 0xFFFFFFFF;
static void drawHoldOverlay() {
  if (!touchActive || inCal || touchDownAtMs == 0) return;
  uint32_t elapsed = millis() - touchDownAtMs;
  if (elapsed < 250 || elapsed >= 20000) return;
  int w = (int)(elapsed * SCREEN_W / 20000);
  overlayShown = true;
  static int lastOvW = -1;
  if (w != lastOvW) { lastOvW = w; tft.fillRect(0, 0, w, 3, C_ACC); }
  if (elapsed > 1500) {
    uint32_t remain = (20000 - elapsed + 999) / 1000;
    if (remain != holdShownSec) {
      holdShownSec = remain;
      char buf[32];
      snprintf(buf, sizeof(buf), "CALIBRATION IN %lus", (unsigned long)remain);
      tft.fillRoundRect(80, 214, 160, 20, 4, C_CARD);
      tft.drawRoundRect(80, 214, 160, 20, 4, C_LINE);
      tft.setTextColor(C_MUT, C_CARD);
      tft.setTextDatum(MC_DATUM);
      tft.drawString(String(buf), 160, 224, 1);
    }
  }
}

// ---------------- setup / loop ------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  LoadSettings();

  tft.init();
  tft.setRotation(1);        // 320x240 landscape, as in the reference
  tft.fillScreen(TFT_BLACK);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcAttach(BL_PIN, 5000, 8);
#else
  ledcSetup(0, 5000, 8);
  ledcAttachPin(BL_PIN, 0);
#endif
  SetBrightness(220);

  // PSRAM framebuffer sprite. NOTE: no tft.initDMA() anywhere - with DMA
  // enabled TFT_eSPI refuses PSRAM for sprites and falls back to internal
  // RAM. A single blocking pushSprite of one complete frame (text included)
  // is what stops the flicker.
  canvas.setColorDepth(16);
  void *cptr = canvas.createSprite(SCREEN_W, SCREEN_H);
  canvasOk = (cptr != nullptr);
  canvasInPsram = canvasOk && ((uintptr_t)cptr >= 0x3C000000UL && (uintptr_t)cptr < 0x3E000000UL);
  if (!canvasOk) {
    Serial.println("FATAL: PSRAM framebuffer allocation failed");
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("FRAMEBUFFER FAILED - CHECK PSRAM", 10, 110, 2);
    while (true) delay(1000);
  }
  canvas.fillSprite(C_BG);
  Serial.printf("PSRAM framebuffer: %d bytes at %p (%s)\n", FB_BYTES, cptr,
                canvasInPsram ? "PSRAM" : "internal RAM - PSRAM not found!");
  Serial.printf("Free PSRAM: %u\n", (unsigned)ESP.getFreePsram());

  Wire.begin(CST_SDA, CST_SCL, I2C_HZ);
  CST328_Reset();
  pinMode(CST_INT, INPUT);
  Serial.println("Touch: CST328 @ 0x1A ready");
  Serial.println("Calculator ready. Hold anywhere 20 s to calibrate touch.");

  drawScreen();
  dirty = false;
}

void loop() {
  PollTouch();  // the ONLY touch read in the whole loop (destructive read)

  bool justStartedCal = false;
  if (!inCal && touchActive && touchDownAtMs != 0 &&
      millis() - touchDownAtMs >= 20000) {
    startCalibration();
    justStartedCal = true;
  }

  if (inCal) updateCalibration();
  else if (!justStartedCal) handleTouch();

  // Blinking text cursor on editing screens. A blink only changes the
  // expression strip, so it pushes ONLY that strip of the PSRAM canvas -
  // never a full-screen repaint (the old full push every 530 ms read as
  // constant flicker on the CALC screen).
  bool editing = (!inCal && !numOpen && !histOpen && (graphEditing || mode == M_CALC));
  if (editing && millis() - lastBlinkMs > 530) {
    lastBlinkMs = millis();
    cursorVisible = !cursorVisible;
    bool wasDirty = dirty;
    markDirty();  // clears dirtyPartial: a pending real change stays full-screen
    if (!wasDirty) {  // a pure blink: push only the expression strip
      dirtyPartial = true;
      if (graphEditing) { g_ppX = 4; g_ppY = 55; g_ppW = 312; g_ppH = 34; }
      else { g_ppX = 4; g_ppY = 44; g_ppW = 312; g_ppH = 42; }
    }
  }

  // Coalesce redraws to at most ~60 fps: rapid touch/drag events set the
  // dirty flag many times, but the panel only ever receives complete,
  // finished frames from PSRAM - never a half-drawn screen.
  if (dirty && millis() - lastDrawMs >= 16) {
    drawScreen();
    dirty = false;
    dirtyPartial = false;
    lastDrawMs = millis();
  }
  drawHoldOverlay();
  if (!touchActive) { holdShownSec = 0xFFFFFFFF; overlayShown = false; }
  delay(2);
}
