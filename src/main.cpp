/*
  LoLin NodeMCU V3 (ESP8266) Dash
  OLED: 1.51" Transparent OLED (SSD1309) 128x64 via SPI
  BNO055: Lean angle (IMUPLUS) + G-Force (Linear Accel) via I2C
  BH1750 (GY-302): Night Mode 2.0 (contrast soft fade + optional invert)
  Oil temp: analog A0 (NTC Beta)
  Button: D3 -> GND (INPUT_PULLUP)

  Pages (short press cycles): OIL -> LEAN -> G -> OIL ...
  Long press:
   - LEAN: resets all-time max (EEPROM)
   - G:    resets all-time max (EEPROM)

  Boot:
   - Screen shows immediately, sensors flip from "..." to OK/FAIL as they init
   - Then waits 2s; during wait you can hold button to calibrate (CAL icon top-right)
*/

#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#include <BH1750.h>
#include <Adafruit_ADS1X15.h>   // ADC expander for extra analog inputs

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

// ---------------- I2C (ESP32 defaults: SDA=21, SCL=22)
// Adjust these if your wiring differs
#define SDA_PIN 21
#define SCL_PIN 22

// ---------------- OLED (SPI) ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Example pins for ESP32 (change to match your wiring)
// your module labels: VCC GND DIN CLK CS DC RST
// map them to ESP32 pins below
#define OLED_MOSI 23   // DIN -> MOSI
#define OLED_CLK  18   // CLK  -> SCK
#define OLED_DC   16
#define OLED_RST  17
#define OLED_CS   4    // CS connected to GPIO4 on this board

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RST, OLED_CS);


// ---------------- BNO055 ----------------
Adafruit_BNO055 bno(55, 0x28);
bool bnoOk = false;

// ---------------- BH1750 ----------------
BH1750 lightMeter;
bool bhOk = false;

// ---------------- ADS1115 analog expander ----------------
Adafruit_ADS1115 ads;          // default I2C address 0x48

// channels
#define ADS_CH_OIL        0    // A0: oil temperature NTC
//#define ADS_CH_OUTSIDE    1  // not used anymore, replaced by DS18B20
#define ADS_CH_BATT       2    // A2: battery voltage divider

// we will use gain = 1 (±4.096V) which gives 1 LSB = 125µV
#define ADS_GAIN          GAIN_ONE
#define ADS_VREF          4.096f
#define ADS_SCALE         (ADS_VREF / 32768.0f)

// battery divider resistors (adjust to your hardware)
const float BATT_R_TOP = 100000.0f;   // between battery+ and A2
const float BATT_R_BOT = 100000.0f;   // between A2 and GND

// ---------------- one‑wire / outside temp ----------------
#define ONE_WIRE_PIN 2           // choose any free GPIO
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature dsSensors(&oneWire);

// outside temperature cache
float outsideTemp = NAN;
unsigned long lastOutsideMs = 0;
const unsigned long OUTSIDE_INTERVAL_MS = 5000; // sample every 5 seconds

// ---------------- Night Mode 2.0 ----------------
const uint8_t CONTRAST_DAY   = 0xFF;
const uint8_t CONTRAST_NIGHT = 0x35;

const float LUX_NIGHT = 10.0f;
const float LUX_DAY   = 120.0f;
const float CONTRAST_FADE_ALPHA = 0.08f;

const bool  ENABLE_ULTRADARK_INVERT = false;
const float LUX_INVERT_ON  = 3.0f;
const float LUX_INVERT_OFF = 6.0f;

float luxFiltered = 50.0f;
float contrastF   = (float)CONTRAST_DAY;
bool  displayInverted = false;
unsigned long lastContrastPushMs = 0;

// ---------------- Pages ----------------
enum Page : uint8_t { PAGE_OIL = 0, PAGE_LEAN = 1, PAGE_G = 2 };
Page page = PAGE_OIL;

#define BTN_PIN 13
#define DEBOUNCE_MS 15
#define LONGPRESS_MS 800

struct ButtonState {
  bool stableLevel = HIGH;
  bool lastSample  = HIGH;
  unsigned long lastEdgeMs = 0;

  bool pressed = false;
  unsigned long pressStartMs = 0;
  bool longFired = false;
} btn;

unsigned long resetAnimUntilMs = 0;

// ---------------- Oil temp ----------------
// On ESP32 use an ADC-capable GPIO (example: 35 -> ADC1_CH7)
#define OIL_PIN 35

#define R_REF 10000.0
#define R0    10000.0
#define T0C   25.0
#define BETA  3450.0
// ADC on ESP32: 12-bit (0..4095) and Vref ~3.3V (adjust if using calibration)
#define ADC_MAX 4095.0f
#define ADC_VREF 3.3f

const float OIL_BAR_MIN_C = 0.0f;
const float OIL_BAR_MAX_C = 120.0f;
const float OIL_GOOD_MIN  = 80.0f;
const float OIL_GOOD_MAX  = 90.0f;

// ---------------- Lean ----------------
float rollFiltered = 0.0f;
float rollOffsetDeg = 0.0f;      // always 0 unless calibrated this session
const float LEAN_ALPHA = 0.15f;

float rollUi = 0.0f;
const float LEAN_UI_ALPHA = 0.25f;

const float LEAN_DEADZONE_DEG = 2.0f;

// All-time max (EEPROM)
float maxLeanSaved = 0.0f;

// Curve Peak Hold
float cornerPeak = 0.0f;
bool  cornerActive = false;
unsigned long belowExitSinceMs = 0;

float holdLean = 0.0f;
unsigned long holdUntilMs = 0;

const float CORNER_ENTER_DEG = 8.0f;
const float CORNER_EXIT_DEG  = 4.0f;
const uint16_t CORNER_EXIT_MS = 350;
const uint16_t HOLD_AFTER_CORNER_MS = 2000;

// ---------------- G-Force ----------------
const float G0 = 9.80665f;
float gX = 0.0f, gY = 0.0f;
const float G_ALPHA = 0.25f;

float maxGSaved = 0.0f;

// ---------------- EEPROM ----------------
#define EEPROM_SIZE 16
#define EE_MAGIC        0x42
#define EE_MAGIC_ADDR   0
#define EE_MAXLEAN_ADDR 1   // uint16_t deg
#define EE_MAXG_ADDR    3   // uint16_t centi-g

bool maxDirty = false;
unsigned long lastEESaveMs = 0;
bool eepromOk = false;

// =========================================================
// PROTOTYPES
// =========================================================
static float normalizeAngleDeg(float a);
static float clampf(float v, float lo, float hi);
static int   mapf_to_i(float v, float inMin, float inMax, int outMin, int outMax);
static void  oledSetContrast(uint8_t c);

bool  loadMaxValues();
void  saveMaxValuesNow();
void  saveMaxValuesSometimes();

void  buttonUpdate();

float readAdsVoltage(int ch);
float readOilTempOnce();
float readOutsideTempOnce();   // now uses DS18B20
float readBatteryVoltage();

void updateOutsideTemp();
float readOilTempAveraged();

void  updateCurvePeakHold(float leanAbs);
void  updateLean();
void  updateGForce();
void  updateNightMode();

void  drawCenteredBigNumberWithDegree(int value, int16_t baselineY);
void  drawCenteredTitleTiny(const char* text, int16_t baselineY);

void  drawLeanSemiGauge(float rollDeg);

void  drawHatchedRect(int x, int y, int w, int h, int spacing = 3);
void  drawOilBar(float oilC);
void  drawOilPage(float oilC);

void  drawLeanPage();

void  drawGCircle(float gx, float gy, float maxGVal);
void  drawGPage();

void  calibrateRollOffset();
void  showReadyScreen();

// Progressive boot
void  drawCalIconTopRight(bool bnoOK, bool armed);
static void drawSelfTestLineProgress(int y, const char* label, int8_t st);
static void renderBootProgress(int8_t stBno, int8_t stBh, int8_t stEe, bool calArmed, float prog01);
void  bootProgressInitAndMaybeCalibrate();

// =========================================================
// HELPERS
// =========================================================
static float normalizeAngleDeg(float a) {
  while (a > 180.0f) a -= 360.0f;
  while (a < -180.0f) a += 360.0f;
  return a;
}
static float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static int mapf_to_i(float v, float inMin, float inMax, int outMin, int outMax) {
  float t = (v - inMin) / (inMax - inMin);
  t = clampf(t, 0.0f, 1.0f);
  return outMin + (int)lroundf(t * (outMax - outMin));
}
static void oledSetContrast(uint8_t c) {
  display.ssd1306_command(SSD1306_SETCONTRAST);
  display.ssd1306_command(c);
}

// =========================================================
// EEPROM load/save
// =========================================================
bool loadMaxValues() {
  EEPROM.begin(EEPROM_SIZE);

  if (EEPROM.read(EE_MAGIC_ADDR) == EE_MAGIC) {
    uint16_t vLean = 0;
    vLean |= (uint16_t)EEPROM.read(EE_MAXLEAN_ADDR);
    vLean |= (uint16_t)EEPROM.read(EE_MAXLEAN_ADDR + 1) << 8;
    maxLeanSaved = (float)vLean;

    uint16_t vG = 0;
    vG |= (uint16_t)EEPROM.read(EE_MAXG_ADDR);
    vG |= (uint16_t)EEPROM.read(EE_MAXG_ADDR + 1) << 8;
    maxGSaved = ((float)vG) / 100.0f;
  } else {
    maxLeanSaved = 0.0f;
    maxGSaved = 0.0f;
  }

  // EEPROM OK test: write/read/restore one byte
  uint8_t old = EEPROM.read(EE_MAGIC_ADDR);
  uint8_t test = (uint8_t)(old ^ 0x5A);

  EEPROM.write(EE_MAGIC_ADDR, test);
  EEPROM.commit();
  delay(2);

  uint8_t rd = EEPROM.read(EE_MAGIC_ADDR);

  EEPROM.write(EE_MAGIC_ADDR, old);
  EEPROM.commit();

  return (rd == test);
}

void saveMaxValuesNow() {
  uint16_t vLean = (uint16_t)round(maxLeanSaved);
  uint16_t vG = (uint16_t)lroundf(clampf(maxGSaved, 0.0f, 9.99f) * 100.0f);

  EEPROM.write(EE_MAGIC_ADDR, EE_MAGIC);

  EEPROM.write(EE_MAXLEAN_ADDR, (uint8_t)(vLean & 0xFF));
  EEPROM.write(EE_MAXLEAN_ADDR + 1, (uint8_t)((vLean >> 8) & 0xFF));

  EEPROM.write(EE_MAXG_ADDR, (uint8_t)(vG & 0xFF));
  EEPROM.write(EE_MAXG_ADDR + 1, (uint8_t)((vG >> 8) & 0xFF));

  EEPROM.commit();
}

void saveMaxValuesSometimes() {
  if (!maxDirty) return;
  unsigned long now = millis();
  if (now - lastEESaveMs < 5000) return;
  saveMaxValuesNow();
  lastEESaveMs = now;
  maxDirty = false;
}

// =========================================================
// Button
// =========================================================
void buttonUpdate() {
  const unsigned long now = millis();
  const bool sample = digitalRead(BTN_PIN);

  if (sample != btn.lastSample) {
    btn.lastSample = sample;
    btn.lastEdgeMs = now;
  }

  if ((now - btn.lastEdgeMs) >= DEBOUNCE_MS && sample != btn.stableLevel) {
    btn.stableLevel = sample;

    if (btn.stableLevel == LOW) {
      btn.pressed = true;
      btn.pressStartMs = now;
      btn.longFired = false;
    } else {
      if (btn.pressed && !btn.longFired) {
        page = (Page)((page + 1) % 3);
      }
      btn.pressed = false;
    }
  }

  // long press (reset)
  if (btn.pressed && !btn.longFired) {
    if ((now - btn.pressStartMs) >= LONGPRESS_MS) {
      btn.longFired = true;

      if (page == PAGE_LEAN) {
        maxLeanSaved = 0.0f;
        maxDirty = true;
        resetAnimUntilMs = now + 350;
      } else if (page == PAGE_G) {
        maxGSaved = 0.0f;
        maxDirty = true;
        resetAnimUntilMs = now + 350;
      }
    }
  }
}

// =========================================================
// Oil temp
// =========================================================
float readAdsVoltage(int ch) {
  if (ads.begin()) {
    int16_t raw = ads.readADC_SingleEnded(ch);
    if (raw < 0) return NAN;
    return raw * ADS_SCALE;
  }
  return NAN;
}

float readOilTempOnce() {
  float v = readAdsVoltage(ADS_CH_OIL);
  if (isnan(v)) return NAN;
  if (v <= 0.001f) return NAN;
  if (v >= (ADS_VREF - 0.001f)) return NAN;

  float rNtc = R_REF * v / (ADS_VREF - v);
  float tempK = 1.0f / ((1.0f / (T0C + 273.15f)) + (1.0f / BETA) * logf(rNtc / R0));
  return tempK - 273.15f;
}

float readOutsideTempOnce() {
  dsSensors.requestTemperatures();
  if (dsSensors.getDeviceCount() == 0) return NAN;
  float t = dsSensors.getTempCByIndex(0);
  if (t == DEVICE_DISCONNECTED_C) return NAN;
  return t;
}

void updateOutsideTemp() {
  unsigned long now = millis();
  if (now - lastOutsideMs >= OUTSIDE_INTERVAL_MS) {
    lastOutsideMs = now;
    outsideTemp = readOutsideTempOnce();
  }
}

float readBatteryVoltage() {
  float v = readAdsVoltage(ADS_CH_BATT);
  if (isnan(v)) return NAN;
  // correct divider
  return v * (BATT_R_TOP + BATT_R_BOT) / BATT_R_BOT;
}

float readOilTempAveraged() {
  int adc0 = analogRead(OIL_PIN);
  if (adc0 < 10 || adc0 > (int)(ADC_MAX - 10)) return NAN;

  const int N = 80;
  double sum = 0.0;
  int count = 0;

  for (int i = 0; i < N; i++) {
    float t = readOilTempOnce();
    if (!isnan(t)) { sum += t; count++; }

    if ((i % 5) == 0) {
      buttonUpdate();
      yield();
    }
  }

  if (count == 0) return NAN;
  return (float)(sum / count);
}

// =========================================================
// Curve Peak Hold
// =========================================================
void updateCurvePeakHold(float leanAbs) {
  unsigned long now = millis();

  if (!cornerActive) {
    if (leanAbs >= CORNER_ENTER_DEG) {
      cornerActive = true;
      cornerPeak = leanAbs;
      belowExitSinceMs = 0;

      holdUntilMs = 0;
      holdLean = 0.0f;
    }
    return;
  }

  if (leanAbs > cornerPeak) cornerPeak = leanAbs;

  if (leanAbs <= CORNER_EXIT_DEG) {
    if (belowExitSinceMs == 0) belowExitSinceMs = now;

    if (now - belowExitSinceMs >= CORNER_EXIT_MS) {
      cornerActive = false;
      belowExitSinceMs = 0;

      holdLean = cornerPeak;
      holdUntilMs = now + HOLD_AFTER_CORNER_MS;

      cornerPeak = 0.0f;
    }
  } else {
    belowExitSinceMs = 0;
  }
}

// =========================================================
// Sensors update
// =========================================================
void updateLean() {
  if (!bnoOk) return;

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  float roll = normalizeAngleDeg(euler.y() - rollOffsetDeg);

  static bool initDone = false;
  if (!initDone) {
    rollFiltered = roll;
    rollUi = roll;
    initDone = true;
  }

  rollFiltered += LEAN_ALPHA * (roll - rollFiltered);

  float target = rollFiltered;
  if (fabs(target) < LEAN_DEADZONE_DEG) target = 0.0f;

  rollUi += LEAN_UI_ALPHA * (target - rollUi);

  float leanAbs = fabs(rollFiltered);
  if (leanAbs < 0.5f) leanAbs = 0.0f;

  if (leanAbs > maxLeanSaved) {
    maxLeanSaved = leanAbs;
    maxDirty = true;
  }

  updateCurvePeakHold(leanAbs);
}

void updateGForce() {
  if (!bnoOk) return;

  imu::Vector<3> la = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  float gxNow = la.x() / G0;
  float gyNow = la.y() / G0;

  gX += G_ALPHA * (gxNow - gX);
  gY += G_ALPHA * (gyNow - gY);

  float mag = sqrtf(gX * gX + gY * gY);
  if (mag > maxGSaved) {
    maxGSaved = mag;
    maxDirty = true;
  }
}

void updateNightMode() {
  if (!bhOk) return;

  float lux = lightMeter.readLightLevel();
  if (isnan(lux) || lux < 0) return;

  const float LUX_ALPHA = 0.12f;
  luxFiltered += LUX_ALPHA * (lux - luxFiltered);

  float t = (luxFiltered - LUX_NIGHT) / (LUX_DAY - LUX_NIGHT);
  t = clampf(t, 0.0f, 1.0f);

  float targetContrast = (float)CONTRAST_NIGHT + t * ((float)CONTRAST_DAY - (float)CONTRAST_NIGHT);
  contrastF += CONTRAST_FADE_ALPHA * (targetContrast - contrastF);

  unsigned long now = millis();
  if (now - lastContrastPushMs > 60) {
    lastContrastPushMs = now;
    uint8_t c = (uint8_t)clampf(contrastF, 0.0f, 255.0f);
    oledSetContrast(c);
  }

  if (ENABLE_ULTRADARK_INVERT) {
    if (!displayInverted && luxFiltered < LUX_INVERT_ON) {
      display.invertDisplay(true);
      displayInverted = true;
    } else if (displayInverted && luxFiltered > LUX_INVERT_OFF) {
      display.invertDisplay(false);
      displayInverted = false;
    }
  }
}

// =========================================================
// UI helpers
// =========================================================
void drawCenteredBigNumberWithDegree(int value, int16_t baselineY) {
  String s = String(value);

  display.setFont(&FreeSansBold18pt7b);

  int16_t x1, y1;
  uint16_t w, h;

  display.getTextBounds(s, 0, baselineY, &x1, &y1, &w, &h);
  int16_t x = (display.width() - (int16_t)w) / 2;

  display.getTextBounds(s, x, baselineY, &x1, &y1, &w, &h);

  display.setCursor(x, baselineY);
  display.print(s);

  // degree circle
  int16_t topY   = y1;
  int16_t rightX = x1 + (int16_t)w;

  const int r = 3;
  int16_t cx = rightX + r + 4;
  int16_t cy = topY + r - 4;

  if (cx > 127 - r) cx = 127 - r;
  if (cy < r)       cy = r;
  if (cy > 63 - r)  cy = 63 - r;

  display.drawCircle(cx, cy, r, SSD1306_WHITE);
}

void drawCenteredTitleTiny(const char* text, int16_t baselineY) {
  display.setFont();
  display.setTextSize(1);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(text, 0, baselineY, &x1, &y1, &w, &h);
  int16_t x = (display.width() - (int16_t)w) / 2;

  display.setCursor(x, baselineY);
  display.print(text);
}

// =========================================================
// Lean Semi Gauge (>=40° => 3px thick)
// =========================================================
void drawLeanSemiGauge(float rollDeg) {
  const int16_t cx = 64;
  const int16_t cy = 63;
  const int16_t r  = 28;

  const float maxDeg = 60.0f;
  if (fabs(rollDeg) < LEAN_DEADZONE_DEG) rollDeg = 0.0f;
  rollDeg = clampf(rollDeg, -maxDeg, +maxDeg);

  const float dangerDeg = 40.0f;

  for (int a = 0; a <= 180; a += 2) {
    float offset = fabs((float)a - 90.0f);
    float degHere = (offset / 90.0f) * maxDeg;

    float rad = a * 3.1415926f / 180.0f;

    int16_t x1p = cx + (int16_t)roundf(cosf(rad) * r);
    int16_t y1p = cy - (int16_t)roundf(sinf(rad) * r);
    display.drawPixel(x1p, y1p, SSD1306_WHITE);

    if (degHere >= dangerDeg) {
      int16_t x2p = cx + (int16_t)roundf(cosf(rad) * (r - 1));
      int16_t y2p = cy - (int16_t)roundf(sinf(rad) * (r - 1));
      display.drawPixel(x2p, y2p, SSD1306_WHITE);

      int16_t x3p = cx + (int16_t)roundf(cosf(rad) * (r - 2));
      int16_t y3p = cy - (int16_t)roundf(sinf(rad) * (r - 2));
      display.drawPixel(x3p, y3p, SSD1306_WHITE);
    }
  }

  // 0 marker
  {
    int16_t tx = cx;
    int16_t ty = cy - r;
    display.fillTriangle(tx, ty - 2, tx - 3, ty + 3, tx + 3, ty + 3, SSD1306_WHITE);
  }

  // center line
  display.drawLine(cx, cy, cx, cy - r, SSD1306_WHITE);

  // wedge fill + needle
  float rollAbs = fabs(rollDeg);
  float sweep = (rollAbs / maxDeg) * 90.0f;
  float a0 = 90.0f;
  float a1 = (rollDeg >= 0.0f) ? (90.0f - sweep) : (90.0f + sweep);

  if (rollDeg != 0.0f) {
    const float step = 3.0f;

    if (a1 < a0) {
      for (float a = a0; a >= a1; a -= step) {
        float aPrev = a + step;
        float r1 = a * 3.1415926f / 180.0f;
        float r2 = aPrev * 3.1415926f / 180.0f;

        int16_t xA = cx + (int16_t)roundf(cosf(r1) * (r - 3));
        int16_t yA = cy - (int16_t)roundf(sinf(r1) * (r - 3));
        int16_t xB = cx + (int16_t)roundf(cosf(r2) * (r - 3));
        int16_t yB = cy - (int16_t)roundf(sinf(r2) * (r - 3));

        display.fillTriangle(cx, cy, xA, yA, xB, yB, SSD1306_WHITE);
      }
    } else {
      for (float a = a0; a <= a1; a += step) {
        float aPrev = a - step;
        float r1 = a * 3.1415926f / 180.0f;
        float r2 = aPrev * 3.1415926f / 180.0f;

        int16_t xA = cx + (int16_t)roundf(cosf(r1) * (r - 3));
        int16_t yA = cy - (int16_t)roundf(sinf(r1) * (r - 3));
        int16_t xB = cx + (int16_t)roundf(cosf(r2) * (r - 3));
        int16_t yB = cy - (int16_t)roundf(sinf(r2) * (r - 3));

        display.fillTriangle(cx, cy, xA, yA, xB, yB, SSD1306_WHITE);
      }
    }

    float needleAng = a1 * 3.1415926f / 180.0f;
    int16_t nx = cx + (int16_t)roundf(cosf(needleAng) * (r - 1));
    int16_t ny = cy - (int16_t)roundf(sinf(needleAng) * (r - 1));
    display.drawLine(cx, cy, nx, ny, SSD1306_WHITE);
  }
}

// =========================================================
// Oil bar + page
// =========================================================
void drawHatchedRect(int x, int y, int w, int h, int spacing) {
  for (int i = -h; i < w; i += spacing) {
    int x0 = x + i;
    int y0 = y + h - 1;
    int x1 = x0 + h;
    int y1 = y;

    int dx = x1 - x0;
    int dy = y1 - y0;
    int steps = max(abs(dx), abs(dy));
    for (int s = 0; s <= steps; s++) {
      int px = x0 + (dx * s) / steps;
      int py = y0 + (dy * s) / steps;
      if (px >= x && px < x + w && py >= y && py < y + h) {
        display.drawPixel(px, py, SSD1306_WHITE);
      }
    }
  }
}

void drawOilBar(float oilC) {
  const int x = 10;
  const int y = 56;
  const int w = 108;
  const int h = 6;

  display.drawRect(x, y, w, h, SSD1306_WHITE);
  bool valid = !isnan(oilC);

  int gx1 = x + 1 + mapf_to_i(OIL_GOOD_MIN, OIL_BAR_MIN_C, OIL_BAR_MAX_C, 0, w - 3);
  int gx2 = x + 1 + mapf_to_i(OIL_GOOD_MAX, OIL_BAR_MIN_C, OIL_BAR_MAX_C, 0, w - 3);
  if (gx2 < gx1) { int tmp = gx1; gx1 = gx2; gx2 = tmp; }
  int gw = gx2 - gx1 + 1;

  if (valid) {
    float t = clampf(oilC, OIL_BAR_MIN_C, OIL_BAR_MAX_C);
    int fillW = mapf_to_i(t, OIL_BAR_MIN_C, OIL_BAR_MAX_C, 0, w - 2);
    if (fillW > 0) display.fillRect(x + 1, y + 1, fillW, h - 2, SSD1306_WHITE);
  }

  if (gw > 0) drawHatchedRect(gx1, y + 1, gw, h - 2, 3);

  display.drawFastVLine(gx1, y - 2, h + 4, SSD1306_WHITE);
  display.drawFastVLine(gx2, y - 2, h + 4, SSD1306_WHITE);

  if (valid && oilC > OIL_GOOD_MAX) {
    bool on = ((millis() / 140) % 2) == 0;
    if (on) {
      display.fillRect(x, y, w, h, SSD1306_WHITE);
      display.fillRect(x + 1, y + 1, w - 2, h - 2, SSD1306_BLACK);
      display.drawFastVLine(gx1, y - 2, h + 4, SSD1306_WHITE);
      display.drawFastVLine(gx2, y - 2, h + 4, SSD1306_WHITE);
    }
  }
}


// fixed margin from screen edge used for both outside temp and battery
#define SIDE_MARGIN 4     // small gap from edge, adjust as needed

void drawOilPage(float oilC) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // outside temperature at fixed left margin, same vertical as battery
  display.setFont();
  display.setTextSize(1);
  if (!isnan(outsideTemp)) {
    display.setCursor(SIDE_MARGIN, 0);
    display.print(outsideTemp, 1);
    display.print("C");
  } else {
    display.setCursor(SIDE_MARGIN, 0);
    display.print("NaN");
  }

  drawCenteredTitleTiny("Oil", 8);

  int16_t baselineY = 45;
  if (isnan(oilC)) {
    display.setFont(&FreeSansBold18pt7b);
    String s = "--";
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(s, 0, baselineY, &x1, &y1, &w, &h);
    int16_t x = (display.width() - (int16_t)w) / 2;
    display.setCursor(x, baselineY);
    display.print(s);
  } else {
    drawCenteredBigNumberWithDegree((int)round(oilC), baselineY);
  }

  display.setFont();
  display.setTextSize(1);

  // status text next to value
  if (!isnan(oilC)) {
    display.setCursor(2, 46);
    if (oilC >= OIL_GOOD_MIN && oilC <= OIL_GOOD_MAX) display.print("OK");
    else if (oilC > OIL_GOOD_MAX) display.print("HOT");
    else display.print("COLD");
  }

  // battery voltage in corner with same fixed margin from right edge
  float batt = readBatteryVoltage();
  if (!isnan(batt)) {
    display.setFont();
    display.setTextSize(1);
    String bs = String(batt, 1);
    bs += "V";
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(bs.c_str(), 0, 0, &x1, &y1, &w, &h);
    int battX = SCREEN_WIDTH - w - SIDE_MARGIN;
    display.setCursor(battX, 0);
    display.print(bs);
  }

  drawOilBar(oilC);
  display.display();
}

// =========================================================
// Lean page
// =========================================================
void drawLeanPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  float liveLean = fabs(rollUi);
  float peakShown;

  if (millis() < holdUntilMs) peakShown = holdLean;
  else if (cornerActive)      peakShown = cornerPeak;
  else                        peakShown = 0.0f;

  drawCenteredBigNumberWithDegree((int)round(liveLean), 28);

  display.setFont();
  display.setTextSize(1);

  // Max overall (left)
  display.setCursor(2, display.height() - 9);
  display.print("M:");
  display.print(maxLeanSaved, 0);

  // Peak (right, integer)
  String pStr = "P:" + String((int)round(peakShown));
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(pStr, 0, 0, &x1, &y1, &w, &h);
  int16_t xRight = SCREEN_WIDTH - (int16_t)w;

  display.setCursor(xRight, SCREEN_HEIGHT - 9);
  display.print(pStr);

  drawLeanSemiGauge(rollUi);

  // reset flash
  unsigned long now = millis();
  if (now < resetAnimUntilMs) {
    bool on = ((now / 80) % 2) == 0;
    if (on) {
      display.fillRect(36, 10, 56, 16, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(48, 14);
      display.print("RESET");
      display.setTextColor(SSD1306_WHITE);
    }
  }

  display.display();
}

// =========================================================
// G page
// =========================================================
void drawGCircle(float gx, float gy, float maxGVal) {
  const int16_t cx = 64;
  const int16_t cy = 40;
  const int16_t r  = 20;

  const float rangeG = 1.5f; // +/- 1.5g

  display.drawCircle(cx, cy, r, SSD1306_WHITE);
  display.drawLine(cx - r, cy, cx + r, cy, SSD1306_WHITE);
  display.drawLine(cx, cy - r, cx, cy + r, SSD1306_WHITE);
  display.drawCircle(cx, cy, r / 2, SSD1306_WHITE);

  float x = clampf(gx, -rangeG, rangeG);
  float y = clampf(gy, -rangeG, rangeG);

  int16_t px = cx + (int16_t)lroundf((x / rangeG) * (float)(r - 2));
  int16_t py = cy - (int16_t)lroundf((y / rangeG) * (float)(r - 2));

  display.fillCircle(px, py, 2, SSD1306_WHITE);

  display.setFont();
  display.setTextSize(1);

  float mag = sqrtf(gx * gx + gy * gy);
  display.setCursor(1, 10);
  display.print(mag, 2);
  display.print("g");

  display.setCursor(1, display.height() - 9);
  display.print("M:");
  display.print(maxGVal, 2);
  display.print("g");
}

void drawGPage() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  drawCenteredTitleTiny("G-Force", 8);

  drawGCircle(gX, gY, maxGSaved);

  // reset flash
  unsigned long now = millis();
  if (now < resetAnimUntilMs) {
    bool on = ((now / 80) % 2) == 0;
    if (on) {
      display.fillRect(36, 10, 56, 16, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(48, 14);
      display.print("RESET");
      display.setTextColor(SSD1306_WHITE);
    }
  }

  display.display();
}

// =========================================================
// Calibration
// =========================================================
void calibrateRollOffset() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setFont();
  display.setTextSize(1);
  display.setCursor(10, 10);
  display.print("Calibrating...");
  display.setCursor(10, 24);
  display.print("Hold still");
  display.display();

  const unsigned long tStart = millis();
  const unsigned long durMs = 1400;

  double sum = 0.0;
  int n = 0;

  while (millis() - tStart < durMs) {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float roll = normalizeAngleDeg(euler.y());
    sum += roll;
    n++;

    delay(10);
    yield();
  }

  rollOffsetDeg = (n > 0) ? (float)(sum / n) : 0.0f;

  display.clearDisplay();
  display.setCursor(10, 22);
  display.print("Offset set");
  display.display();
  delay(250);
}

// =========================================================
// Progressive Boot UI
// =========================================================
void drawCalIconTopRight(bool bnoOK, bool armed) {
  if (!bnoOK) return;

  const int16_t y = 6;
  const int16_t x0 = 96;
  const int16_t w  = 32;
  const int16_t h  = 10;

  // clear whole area each frame
  display.fillRect(x0, y - 1, w, h, SSD1306_BLACK);

  // button circle
  const int16_t cx = x0 + 6;
  const int16_t cy = y + 2;
  const int16_t r  = 3;
  display.drawCircle(cx, cy, r, SSD1306_WHITE);
  display.fillCircle(cx, cy, 1, SSD1306_WHITE);

  // CAL text
  const int16_t tx = x0 + 12;

  if (!armed) {
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(tx, y);
    display.print("CAL");
  } else {
    display.fillRect(tx - 1, y - 1, 20, 9, SSD1306_WHITE);
    display.setTextColor(SSD1306_BLACK);
    display.setCursor(tx, y);
    display.print("CAL");
    display.setTextColor(SSD1306_WHITE);
  }
}

// status: -1 = pending, 0 = fail, 1 = ok
static void drawSelfTestLineProgress(int y, const char* label, int8_t st) {
  display.setFont();
  display.setTextSize(1);

  display.setCursor(8, y);
  display.print(label);
  display.print(": ");

  if (st < 0) {
    display.print("...");
    return;
  }

  if (st > 0) {
    display.print("OK");
  } else {
    bool on = ((millis() / 160) % 2) == 0;
    if (on) {
      String pre = String(label) + ": ";
      int16_t x1, y1; uint16_t w, h;
      display.getTextBounds(pre, 0, y, &x1, &y1, &w, &h);
      int16_t xFail = 8 + (int16_t)w;

      display.fillRect(xFail - 1, y - 8, 30, 10, SSD1306_WHITE);
      display.setTextColor(SSD1306_BLACK);
      display.setCursor(xFail, y);
      display.print("FAIL");
      display.setTextColor(SSD1306_WHITE);
    }
  }
}

static void renderBootProgress(int8_t stBno, int8_t stBh, int8_t stEe, bool calArmed, float prog01) {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setFont();
  display.setTextSize(1);

  display.setCursor(8, 6);
  display.print("Self-Test");

  drawSelfTestLineProgress(20, "BNO055", stBno);
  drawSelfTestLineProgress(30, "BH1750", stBh);
  drawSelfTestLineProgress(40, "EEPROM", stEe);

  drawCalIconTopRight(stBno > 0, calArmed);

  const int barX = 8, barY = 59, barW = 112, barH = 4;
  display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);
  int fill = (int)((barW - 2) * clampf(prog01, 0.0f, 1.0f));
  if (fill > 0) display.fillRect(barX + 1, barY + 1, fill, barH - 2, SSD1306_WHITE);

  display.display();
}

void bootProgressInitAndMaybeCalibrate() {
  int8_t stBno = -1, stBh = -1, stEe = -1;

  // show instantly
  renderBootProgress(stBno, stBh, stEe, false, 0.05f);

  // EEPROM
  eepromOk = loadMaxValues();
  stEe = eepromOk ? 1 : 0;
  renderBootProgress(stBno, stBh, stEe, false, 0.33f);

  // BH1750
  bhOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  stBh = bhOk ? 1 : 0;
  renderBootProgress(stBno, stBh, stEe, false, 0.66f);

  // BNO055
  bnoOk = bno.begin();
  if (bnoOk) {
    delay(50);
    bno.setMode(OPERATION_MODE_IMUPLUS);
    delay(20);
    bno.setExtCrystalUse(true);
    delay(50);
  }
  stBno = bnoOk ? 1 : 0;
  renderBootProgress(stBno, stBh, stEe, false, 1.0f);

  // wait 2s with hold-to-cal
  const unsigned long waitMs = 2000;
  unsigned long t0 = millis();

  unsigned long holdStart = 0;
  bool calArmed = false;

  while (millis() - t0 < waitMs) {
    unsigned long now = millis();

    if (bnoOk && digitalRead(BTN_PIN) == LOW) {
      if (holdStart == 0) holdStart = now;
      if (!calArmed && (now - holdStart) >= 250) calArmed = true; // latch
    } else {
      holdStart = 0;
    }

    renderBootProgress(stBno, stBh, stEe, calArmed, 1.0f);
    delay(25);
    yield();
  }

  if (bnoOk && calArmed) {
    calibrateRollOffset();
  }
}

void showReadyScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setFont(&FreeSansBold18pt7b);
  display.setCursor(18, 40);
  display.print("READY");
  display.display();
  delay(260);
}

// =========================================================
// Setup / Loop
// =========================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BTN_PIN, INPUT_PULLUP);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // start one‑wire bus for DS18B20
  dsSensors.begin();
  Serial.print("OneWire device count: ");
  Serial.println(dsSensors.getDeviceCount());
  if (dsSensors.getDeviceCount() == 0) {
    Serial.print("Warning: no DS18B20 found on pin ");
    Serial.println(ONE_WIRE_PIN);
  }

  // initialize ADS1115 for analog measurements
  if (!ads.begin()) {
    Serial.println("ADS1115 not found!");
    // we keep going, but analog reads will return NAN
  } else {
    ads.setGain(ADS_GAIN);
    Serial.println("ADS1115 ready");
  }

  // configure hardware SPI pins before display.begin
  // if you're using the default VSPI pins, SPI.begin() with no arguments is fine
  // otherwise specify sck, miso, mosi, ss
  SPI.begin(OLED_CLK, /*MISO*/ -1, OLED_MOSI, OLED_CS);
  pinMode(OLED_CS, OUTPUT);
  digitalWrite(OLED_CS, LOW); // keep chip selected

  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println("OLED init failed!");
    while (true) { delay(100); }
  }
  Serial.println("OLED init ok");

  display.setRotation(0);
  display.setTextColor(SSD1306_WHITE);
  display.clearDisplay();
  display.display();

  oledSetContrast(CONTRAST_DAY);

  rollOffsetDeg = 0.0f; // always start 0 unless calibrated this session

  bootProgressInitAndMaybeCalibrate();
  showReadyScreen();
}

void loop() {
  buttonUpdate();

  updateLean();
  updateGForce();
  updateNightMode();

  saveMaxValuesSometimes();

  static unsigned long lastDraw = 0;
  unsigned long now = millis();

  if (now - lastDraw >= 75) {
    lastDraw = now;

    if (page == PAGE_OIL) {
      float oilC = readOilTempAveraged();
      drawOilPage(oilC);
    } else if (page == PAGE_LEAN) {
      drawLeanPage();
    } else {
      drawGPage();
    }
  }

  delay(1);
  yield();
}