/*
  ESP32-S3 N16R8 Motorrad-Dash
  MCU   : ESP32-S3 (N16R8 – 16 MB Flash QIO, 8 MB PSRAM OPI)

  Hardware & GPIO assignment
  ──────────────────────────
  I2C bus (BNO085 + BH1750 + ADS1115)
    SDA  = GPIO 8
    SCL  = GPIO 9

  OLED 1.51" Transparent (SSD1309 / SSD1306) 128×64 – SPI2
    DIN/MOSI = GPIO 11  (SPI2_MOSI)
    CLK/SCK  = GPIO 12  (SPI2_SCK)
    CS       = GPIO 10
    DC       = GPIO 5
    RST      = GPIO 6

  BNO085 (I2C, address 0x4A)
    → uses shared I2C bus above; no dedicated reset pin

  BH1750 (I2C, address 0x23)
    → uses shared I2C bus above

  ADS1115 (I2C, address 0x48) – 4-channel 16-bit ADC
    AIN0 = Oil temperature NTC
    AIN1 = Blitzer-Warner heartbeat LED line (~2.6 V idle / ~3.5 V pulse)
    AIN2 = Battery voltage divider
    AIN3 = (free)

  DS18B20 – Outside temperature 1-Wirex
    DATA = GPIO 7

  Button (main nav, INPUT_PULLUP → GND)
    GPIO 4

  Blitzer-Warner
    Digital trigger (INPUT_PULLUP → LOW on alert)  = GPIO 14
    Heartbeat LED anode (idle ~2.6 V, pulse ~3.5 V) = ADS1115 AIN1

  RaceBox Mini inputs (all INPUT_PULLUP, HIGH = active)
    GPS fix  = GPIO 15
    BLE conn = GPIO 16
    REC      = GPIO 17

  RaceBox simulated button (NPN transistor, active HIGH)
    GPIO 18

  OIL_PIN (legacy NTC direct-ADC fallback, not actively used)
    GPIO 1   ← oil temp is read via ADS1115 channel 0

  Pages (short press cycles): OIL -> LEAN -> G -> RACEBOX -> OIL ...
  Long press:
   - LEAN : resets all-time max (EEPROM)
   - G    : resets all-time max (EEPROM)

  Boot:
   - Screen shows immediately, sensors flip from "..." to OK/FAIL as they init
   - Then waits 2 s; during wait you can hold button to calibrate (CAL icon top-right)
*/

#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>

#include <BH1750.h>
#include <Adafruit_ADS1X15.h> // ADC expander for extra analog inputs

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

// ---------------- I2C (ESP32-S3: SDA=8, SCL=9)
#define SDA_PIN 8
#define SCL_PIN 9

// ---------------- OLED (SPI2) ----------------
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// ESP32-S3 SPI2 hardware pins
// Module labels: VCC GND DIN CLK CS DC RST
#define OLED_MOSI 11 // DIN  -> GPIO 11 (SPI2_MOSI)
#define OLED_CLK  12 // CLK  -> GPIO 12 (SPI2_SCK)
#define OLED_DC    5 // DC   -> GPIO 5
#define OLED_RST   6 // RST  -> GPIO 6
#define OLED_CS   10 // CS   -> GPIO 10

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RST, OLED_CS);

// ---------------- BNO085 ----------------
Adafruit_BNO08x bno(-1);  // -1 = no reset pin, using I2C
bool bnoOk = false;
// cached sensor values updated every loop
float bno085Roll   = 0.0f;
float bno085LinX   = 0.0f;
float bno085LinY   = 0.0f;

// ---------------- BH1750 ----------------
BH1750 lightMeter;
bool bhOk = false;

// ---------------- ADS1115 analog expander ----------------
Adafruit_ADS1115 ads; // default I2C address 0x48

// channels
#define ADS_CH_OIL            0 // A0: oil temperature NTC
#define ADS_CH_BLITZER_ALIVE  1 // A1: Blitzer-Warner heartbeat LED line
#define ADS_CH_BATT           2 // A2: battery voltage divider

// Voltage threshold: idle=~2.6V, pulse=~3.5V → detect >3.0V as heartbeat
#define BLITZER_ALIVE_V_THRESHOLD 3.0f

// we will use gain = 1 (±4.096V) which gives 1 LSB = 125µV
#define ADS_GAIN GAIN_ONE
#define ADS_VREF 4.096f       // ADS1115 internal reference for GAIN_ONE (for ADC scaling)
#define ADS_SCALE (ADS_VREF / 32768.0f)
#define DIVIDER_VCC 3.3f      // actual supply voltage of the NTC voltage divider

// battery divider resistors (adjust to your hardware)
// 33k/10k: max measurable = 4.096V * (33k+10k)/10k = 17.6V → safe for 12V motorcycle system
const float BATT_R_TOP = 33000.0f; // 33 kΩ between battery+ and A2
const float BATT_R_BOT = 10000.0f; // 10 kΩ between A2 and GND
// calibration: measured_real / measured_displayed (set 1.0 to disable)
// e.g. multimeter shows 12.1V, display shows 12.0V → 12.1/12.0 = 1.00833
const float BATT_CAL = 1.00833f;
float BATT_LOW_V = 10.5f;           // battery low warning threshold (flash text) – runtime, saved in EEPROM

// ---- Test mode: comment out to disable ----
// #define TEST_MODE
// When active: oil temp cycles -5..120°C, lean angle cycles -90..+90°

// ---------------- one‑wire / outside temp ----------------
#define ONE_WIRE_PIN 7  // GPIO7 - safe on ESP32-S3 N16R8 (GPIO35-37 = PSRAM, avoid)
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature dsSensors(&oneWire);

// outside temperature cache
float outsideTemp = NAN;
unsigned long lastOutsideMs = 0;
const unsigned long OUTSIDE_INTERVAL_MS = 5000; // sample every 5 seconds
bool outsideConvRequested = false;  // async DS18B20 flag
DeviceAddress outsideSensorAddr;    // stored sensor address
bool ds18b20Found = false;          // sensor discovered?

// oil temperature cache
float oilTempCached = NAN;
const float OIL_EMA_ALPHA = 0.15f;  // smoothing factor for oil temp

// DS18B20 calibration offset (add to raw reading) – runtime, saved in EEPROM
float DS18B20_OFFSET = -1.2f;

// battery voltage cache
float battVoltageCached = NAN;
const float BATT_EMA_ALPHA = 0.2f;  // smoothing factor for battery

// round-robin ADS1115 scheduling: one read per loop, rotate channels
// slot 0 = oil (ADS ch0), slot 1 = Blitzer alive (ADS ch1), slot 2 = battery (ADS ch2)
unsigned long lastAdsReadMs = 0;
const unsigned long ADS_READ_INTERVAL_MS = 100;  // one ADS read every 100ms
uint8_t adsNextChannel = 0;  // 0 = oil, 1 = blitzer alive, 2 = battery

// ADS1115 init flag
bool adsOk = false;

// ---------------- Night Mode 2.0 ----------------
const uint8_t CONTRAST_DAY = 0xFF;
const uint8_t CONTRAST_NIGHT = 0x35;

const float LUX_NIGHT = 10.0f;
const float LUX_DAY = 120.0f;
const float CONTRAST_FADE_ALPHA = 0.08f;

const bool ENABLE_ULTRADARK_INVERT = false;
const float LUX_INVERT_ON = 3.0f;
const float LUX_INVERT_OFF = 6.0f;

float luxFiltered = 50.0f;
float contrastF = (float)CONTRAST_DAY;
bool displayInverted = false;
unsigned long lastContrastPushMs = 0;

// ---------------- Pages ----------------
enum Page : uint8_t
{
	PAGE_OIL = 0,
	PAGE_LEAN = 1,
	PAGE_G = 2,
	PAGE_RACEBOX = 3
};

// ---------------- RaceBox status inputs ----------------
#define RACEBOX_GPS_PIN 15  // HIGH when GPS fix active  (GPIO26-32 = flash on ESP32-S3, avoid)
#define RACEBOX_BLE_PIN 16  // HIGH when BLE connected
#define RACEBOX_REC_PIN 17  // HIGH when recording active

bool raceboxGps = false;
bool raceboxBle = false;
bool raceboxRec = false;
unsigned long raceboxRecLastActiveMs = 0;
bool raceboxRecEverSeen  = false;
unsigned long raceboxRecLowSinceMs  = 0; // debounce for EverSeen
#define RACEBOX_REC_HOLD_MS 3000
#define RACEBOX_BTN_PIN 18  // OUTPUT: drives NPN transistor to simulate RaceBox button press (active HIGH)
unsigned long raceboxBtnUntilMs = 0;    // when to release the simulated button
unsigned long raceboxBtnCooldownMs = 0; // prevent re-trigger while holding
bool raceboxBtnArmed = true;            // only fire once per button press cycle
#define RACEBOX_LONGPRESS_MS 250        // shorter long press threshold for RaceBox page

// ---------------- Blitzer Warner ----------------
#define BLITZER_PIN 14      // GPIO14 - safe on ESP32-S3 N16R8
unsigned long blitzerActiveUntilMs = 0;
bool blitzerPinLast = true;        // for edge detection (HIGH = idle with PULLUP)
unsigned long blitzerLowSinceMs = 0; // debounce: timestamp when pin first went LOW
#define BLITZER_DEBOUNCE_MS 30       // pin must stay LOW this long before triggering

// Blitzer Warner heartbeat: LED blinks every ~5s to signal device is running
// Idle voltage ~2.6V, pulse voltage ~3.5V -> detected via ADS1115 channel 1
// Wire: LED+ -> AIN1 on ADS1115 board (GND already shared)
#define BLITZER_ALIVE_TIMEOUT_MS 12000  // >12s without pulse = Warner offline
bool blitzerAliveReceived   = false;    // true once the first heartbeat arrived
unsigned long blitzerAliveLastMs = 0;   // timestamp of last heartbeat pulse

// debounce for GPS and BLE (must be stable for 300ms before accepting)
#define RACEBOX_DEBOUNCE_MS 300
bool raceboxGpsRaw = false;
bool raceboxBleRaw = false;
unsigned long raceboxGpsChangeMs = 0;
unsigned long raceboxBleChangeMs = 0;
unsigned long raceboxBleLastActiveMs = 0;
#define RACEBOX_BLE_HOLD_MS 3000
bool raceboxBleEverSeen  = false;
unsigned long raceboxBleLowSinceMs  = 0;
unsigned long raceboxBleHighSinceMs = 0;  // tracks continuous HIGH duration
bool raceboxBlePinOk = false;             // true once pin was HIGH >= 200ms (filters boot blip + backfeed)
unsigned long raceboxGpsLastActiveMs = 0;
#define RACEBOX_GPS_HOLD_MS 3000
bool raceboxGpsEverSeen  = false;
unsigned long raceboxGpsLowSinceMs  = 0;
unsigned long raceboxGpsHighSinceMs = 0;
bool raceboxGpsPinOk = false;
bool raceboxRecPinOk = false;
unsigned long raceboxRecHighSinceMs = 0;
// How long a pin must stay HIGH before LOW triggers are accepted.
// BLE/GPS: 500ms – catches RaceBox boot-up OFF window, filters backfeed (constant LOW, no inter-blink HIGH)
// REC:     2000ms – LED stays HIGH when not recording, so PinOk sets easily; backfeed safely filtered
#define RACEBOX_PIN_VALID_HIGH_BLE_GPS_MS 500
#define RACEBOX_PIN_VALID_HIGH_REC_MS     2000
unsigned long raceboxBleAliveLastMs = 0;   // last time BLE pin was LOW after PinOk (= RaceBox is on)
#define RACEBOX_BLE_ALIVE_TIMEOUT_MS 10000 // show "ON" for 10s after last blink
Page page = PAGE_OIL;

#define BTN_PIN  4          // GPIO4 - safe on ESP32-S3 N16R8 (GPIO13 = SPI2_MISO, avoid)
#define DEBOUNCE_MS 15
#define LONGPRESS_MS 800

struct ButtonState
{
	bool stableLevel = HIGH;
	bool lastSample = HIGH;
	unsigned long lastEdgeMs = 0;

	bool pressed = false;
	unsigned long pressStartMs = 0;
	bool longFired = false;
} btn;

unsigned long resetAnimUntilMs = 0;

// ---------------- Settings ----------------
#define SETTINGS_OPEN_MS   5000   // hold duration to enter settings
#define SETTINGS_TIMEOUT_MS 10000 // auto-close after 10s inactivity
#define SETTINGS_LONGPRESS_MS 600 // long press inside settings = change value

// Items
enum SettingsItem : uint8_t {
	SET_DS18_OFFSET = 0,  // outside temp offset
	SET_G_DEAD,           // G-force deadzone
	SET_BRIGHTNESS,       // brightness mode: 0=Auto, 1=Tag, 2=Nacht
	SET_LEAN_FLIP,        // invert lean angle direction
	SET_RESET_ALL,        // reset all max values
	SET_COUNT
};

// Brightness mode: 0=Auto (BH1750), 1=Tag (always max), 2=Nacht (always min)
uint8_t brightMode = 0;
bool leanFlip = false;  // true = invert roll direction (sensor mounted mirrored)

bool settingsOpen = false;
uint8_t settingsIdx = 0;           // currently selected item
unsigned long settingsLastActMs = 0; // last interaction time (for timeout)
bool settingsLongFired = false;    // long-press-in-settings already fired
unsigned long settingsPressStartMs = 0;

void drawSettingsPage();

// ---------------- Oil temp ----------------
// NOTE: OIL_PIN is a legacy fallback – oil temp is read via ADS1115 ch0.
// GPIO35-37 are used for PSRAM on ESP32-S3 N16R8; remaped to GPIO1 (unused).
#define OIL_PIN 1

#define R_REF 10000.0
#define R0 10000.0
#define T0C 25.0
#define BETA 3450.0
// ADC on ESP32: 12-bit (0..4095) and Vref ~3.3V (adjust if using calibration)
#define ADC_MAX 4095.0f
#define ADC_VREF 3.3f

const float OIL_BAR_MIN_C = 0.0f;
const float OIL_BAR_MAX_C = 120.0f;
const float OIL_GOOD_MIN = 70.0f;
const float OIL_GOOD_MAX = 90.0f;
const float OIL_CRITICAL_C = 110.0f; // above this: full-screen warning overlay

// ---------------- Lean ----------------
float rollFiltered = 0.0f;
float rollOffsetDeg = 0.0f; // always 0 unless calibrated this session
const float LEAN_ALPHA = 0.15f;

float rollUi = 0.0f;
const float LEAN_UI_ALPHA = 0.25f;

const float LEAN_DEADZONE_DEG = 2.0f;

// All-time max (EEPROM)
float maxLeanSaved = 0.0f;
float maxLeanLeft  = 0.0f;  // all-time max lean to the left
float maxLeanRight = 0.0f;  // all-time max lean to the right

// Curve Peak Hold
float cornerPeak = 0.0f;
bool cornerActive = false;
unsigned long belowExitSinceMs = 0;

float holdLean = 0.0f;
unsigned long holdUntilMs = 0;
int8_t lastCornerSign = 0;        // +1 = last corner was right, -1 = left
bool cornerAboveThreshold = false; // true once cornerPeak exceeded 15° in this corner
unsigned long cornerDeclineStartMs = 0; // when lean dropped back below 15° during active corner

const float CORNER_ENTER_DEG = 8.0f;
const float CORNER_EXIT_DEG = 4.0f;
const uint16_t CORNER_EXIT_MS = 350;
const uint16_t HOLD_AFTER_CORNER_MS = 4000;

const float CENTER_PEAK_THRESHOLD = 15.0f;   // above this: show peak
const float CENTER_OPPOSITE_CANCEL = 5.0f;   // opposite-side degrees to cancel hold

// ---------------- G-Force ----------------
const float G0 = 9.80665f;
float gX = 0.0f, gY = 0.0f;           // slow-filtered (display dot)
float gXFast = 0.0f, gYFast = 0.0f;   // fast-filtered (max-G tracking)
const float G_ALPHA_DISPLAY = 0.10f;   // träge → wenig Jitter auf dem Dot
const float G_ALPHA_MAX     = 0.20f;   // schneller → echte Manöver landen im Max
float G_DEADZONE            = 0.04f;   // Motorvibrationen < 0.04g werden ignoriert – runtime, saved in EEPROM

float maxGSaved = 0.0f;

// 4 Quadranten × 2 stärkste Peaks (RAM-only, weg beim Neustart)
// Quadrant: 0=rechts/vorne (+x/+y), 1=links/vorne (-x/+y),
//           2=links/hinten (-x/-y), 3=rechts/hinten (+x/-y)
struct GFPeak { float x; float y; float mag; };
GFPeak gQuadPeaks[4][2];  // [quadrant][slot 0=stärker, 1=schwächer]
bool   gQuadHasSlot[4][2];

static uint8_t gQuadrant(float gx, float gy)
{
	if (gx >= 0.0f && gy >= 0.0f) return 0;
	if (gx <  0.0f && gy >= 0.0f) return 1;
	if (gx <  0.0f && gy <  0.0f) return 2;
	return 3;
}

// ---------------- EEPROM ----------------
#define EEPROM_SIZE 32
#define EE_MAGIC 0x42
#define EE_MAGIC_ADDR 0
#define EE_MAXLEAN_ADDR    1 // uint16_t deg (combined abs max, kept for compat)
#define EE_MAXG_ADDR       3 // uint16_t centi-g
#define EE_MAXLEAN_L_ADDR  5 // uint16_t deg – max left lean
#define EE_MAXLEAN_R_ADDR  7 // uint16_t deg – max right lean
// Settings (stored as signed int8 or uint8)
#define EE_SET_MAGIC     9  // uint8_t magic to detect first-ever settings save
#define EE_SET_MAGIC_VAL 0x5E
#define EE_SET_DS18_OFF  10  // int8_t  tenths of °C  (-30..+30 → -3.0..+3.0°C)
#define EE_SET_BATT_LOW  11  // uint8_t tenths of V   (80..140  → 8.0..14.0V)
#define EE_SET_G_DEAD    12  // uint8_t hundredths g  (0..20    → 0..0.20g)
#define EE_SET_BRIGHT    14  // uint8_t 0=Auto 1=Tag 2=Nacht
#define EE_SET_LEAN_FLIP 13  // uint8_t 0/1

bool maxDirty = false;
unsigned long lastEESaveMs = 0;
bool eepromOk = false;

// =========================================================
// PROTOTYPES
// =========================================================
static float normalizeAngleDeg(float a);
static float clampf(float v, float lo, float hi);
static int mapf_to_i(float v, float inMin, float inMax, int outMin, int outMax);
static void oledSetContrast(uint8_t c);

bool loadMaxValues();
void saveMaxValuesNow();
void saveMaxValuesSometimes();

void buttonUpdate();

float readAdsVoltage(int ch);
float readOilTempOnce();
float readBatteryVoltage();

void updateOutsideTemp();
void updateOilTemp();
void updateAdsReadings();

void updateCurvePeakHold(float leanAbs);
void updateLean();
static void pollBno085();
void updateGForce();
void updateNightMode();

void drawCenteredBigNumberWithDegree(int value, int16_t baselineY);
void drawCenteredTitleTiny(const char *text, int16_t baselineY);

void drawLeanSemiGauge(float rollDeg);

void drawHatchedRect(int x, int y, int w, int h, int spacing = 3);
void drawOilBar(float oilC);
void drawOilPage(float oilC);
void drawBlitzerWarnerAliveIndicator();
static void drawBatteryTopRight();

void drawLeanPage();

void drawGCircle(float gx, float gy, float maxGVal);
void drawGPage();
void drawRaceBoxPage();

void calibrateRollOffset();
void showReadyScreen();

// Progressive boot
void drawCalIconTopRight(bool bnoOK, bool armed);
static void drawSelfTestLineProgress(int y, const char *label, int8_t st);
static void renderBootProgress(int8_t stBno, int8_t stBh, int8_t stEe, int8_t stAds, bool calArmed, float prog01);
void bootProgressInitAndMaybeCalibrate();

// =========================================================
// BNO085 helpers
// =========================================================
// Convert GAME_ROTATION_VECTOR quaternion → lean angle in degrees
// Using pitch (Y-axis rotation) for side-to-side lean
static float bno085QuatToRoll(float qi, float qj, float qk, float qr)
{
	float sinp = 2.0f * (qr * qj - qk * qi);
	sinp = clampf(sinp, -1.0f, 1.0f);
	return asinf(sinp) * (180.0f / M_PI);
}

// Poll all pending BNO085 events and cache roll + linear accel
static void pollBno085()
{
	if (!bnoOk) return;
	sh2_SensorValue_t ev;
	while (bno.getSensorEvent(&ev))
	{
		if (ev.sensorId == SH2_GAME_ROTATION_VECTOR)
		{
			bno085Roll = bno085QuatToRoll(
				ev.un.gameRotationVector.i,
				ev.un.gameRotationVector.j,
				ev.un.gameRotationVector.k,
				ev.un.gameRotationVector.real);
		}
		else if (ev.sensorId == SH2_LINEAR_ACCELERATION)
		{
			bno085LinX = ev.un.linearAcceleration.x;
			bno085LinY = ev.un.linearAcceleration.y;
		}
	}
}

// =========================================================
// HELPERS
// =========================================================
static float normalizeAngleDeg(float a)
{
	while (a > 180.0f)
		a -= 360.0f;
	while (a < -180.0f)
		a += 360.0f;
	return a;
}
// Triangle wave: lo..hi and back, full cycle = periodMs
static float triangleWave(float lo, float hi, unsigned long periodMs)
{
	unsigned long t = millis() % periodMs;
	float half = periodMs / 2.0f;
	float frac = (t < (unsigned long)half)
		? (float)t / half
		: 1.0f - ((float)(t - (unsigned long)half) / half);
	return lo + frac * (hi - lo);
}

static float clampf(float v, float lo, float hi)
{
	if (v < lo)
		return lo;
	if (v > hi)
		return hi;
	return v;
}
static int mapf_to_i(float v, float inMin, float inMax, int outMin, int outMax)
{
	float t = (v - inMin) / (inMax - inMin);
	t = clampf(t, 0.0f, 1.0f);
	return outMin + (int)lroundf(t * (outMax - outMin));
}
static void oledSetContrast(uint8_t c)
{
	display.ssd1306_command(SSD1306_SETCONTRAST);
	display.ssd1306_command(c);
}

// =========================================================
// EEPROM load/save
// =========================================================
bool loadMaxValues()
{
	EEPROM.begin(EEPROM_SIZE);

	if (EEPROM.read(EE_MAGIC_ADDR) == EE_MAGIC)
	{
		uint16_t vLean = 0;
		vLean |= (uint16_t)EEPROM.read(EE_MAXLEAN_ADDR);
		vLean |= (uint16_t)EEPROM.read(EE_MAXLEAN_ADDR + 1) << 8;
		maxLeanSaved = (float)vLean;

		uint16_t vLL = 0;
		vLL |= (uint16_t)EEPROM.read(EE_MAXLEAN_L_ADDR);
		vLL |= (uint16_t)EEPROM.read(EE_MAXLEAN_L_ADDR + 1) << 8;
		maxLeanLeft = (float)vLL;

		uint16_t vLR = 0;
		vLR |= (uint16_t)EEPROM.read(EE_MAXLEAN_R_ADDR);
		vLR |= (uint16_t)EEPROM.read(EE_MAXLEAN_R_ADDR + 1) << 8;
		maxLeanRight = (float)vLR;

		uint16_t vG = 0;
		vG |= (uint16_t)EEPROM.read(EE_MAXG_ADDR);
		vG |= (uint16_t)EEPROM.read(EE_MAXG_ADDR + 1) << 8;
		maxGSaved = ((float)vG) / 100.0f;

		// Settings – only load if their own magic byte is present
		if (EEPROM.read(EE_SET_MAGIC) == EE_SET_MAGIC_VAL)
		{
			int8_t  eeDs18  = (int8_t) EEPROM.read(EE_SET_DS18_OFF);
			uint8_t eeGDead = EEPROM.read(EE_SET_G_DEAD);
			uint8_t eeFlip   = EEPROM.read(EE_SET_LEAN_FLIP);
			uint8_t eeBright = EEPROM.read(EE_SET_BRIGHT);
			DS18B20_OFFSET = clampf((float)eeDs18  / 10.0f, -3.0f,  3.0f);
			G_DEADZONE     = clampf((float)eeGDead / 100.0f, 0.0f,  0.20f);
			brightMode     = (eeBright <= 2) ? eeBright : 0;
			leanFlip       = (eeFlip == 1);
		}
		// else: keep compile-time defaults (DS18B20_OFFSET=-1.2, BATT_LOW_V=10.5, G_DEADZONE=0.04)
	}
	else
	{
		maxLeanSaved = 0.0f;
		maxGSaved = 0.0f;
	}

	// clamp to sane ranges (guard against uninitialized / corrupt EEPROM)
	maxLeanSaved  = clampf(maxLeanSaved,  0.0f, 90.0f);
	maxLeanLeft   = clampf(maxLeanLeft,   0.0f, 90.0f);
	maxLeanRight  = clampf(maxLeanRight,  0.0f, 90.0f);
	maxGSaved     = clampf(maxGSaved,     0.0f, 9.99f);

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

void saveMaxValuesNow()
{
	uint16_t vLean = (uint16_t)round(maxLeanSaved);
	uint16_t vG    = (uint16_t)lroundf(clampf(maxGSaved, 0.0f, 9.99f) * 100.0f);
	uint16_t vLL   = (uint16_t)round(maxLeanLeft);
	uint16_t vLR   = (uint16_t)round(maxLeanRight);

	EEPROM.write(EE_MAGIC_ADDR, EE_MAGIC);

	EEPROM.write(EE_MAXLEAN_ADDR,     (uint8_t)(vLean & 0xFF));
	EEPROM.write(EE_MAXLEAN_ADDR + 1, (uint8_t)((vLean >> 8) & 0xFF));

	EEPROM.write(EE_MAXG_ADDR,     (uint8_t)(vG & 0xFF));
	EEPROM.write(EE_MAXG_ADDR + 1, (uint8_t)((vG >> 8) & 0xFF));

	EEPROM.write(EE_MAXLEAN_L_ADDR,     (uint8_t)(vLL & 0xFF));
	EEPROM.write(EE_MAXLEAN_L_ADDR + 1, (uint8_t)((vLL >> 8) & 0xFF));

	EEPROM.write(EE_MAXLEAN_R_ADDR,     (uint8_t)(vLR & 0xFF));
	EEPROM.write(EE_MAXLEAN_R_ADDR + 1, (uint8_t)((vLR >> 8) & 0xFF));

	// Settings
	EEPROM.write(EE_SET_MAGIC,     EE_SET_MAGIC_VAL);
	EEPROM.write(EE_SET_DS18_OFF,  (uint8_t)(int8_t)lroundf(DS18B20_OFFSET * 10.0f));
	EEPROM.write(EE_SET_G_DEAD,    (uint8_t)lroundf(G_DEADZONE * 100.0f));
	EEPROM.write(EE_SET_BRIGHT,    brightMode);
	EEPROM.write(EE_SET_LEAN_FLIP, leanFlip ? 1 : 0);

	EEPROM.commit();
}

void saveMaxValuesSometimes()
{
	if (!maxDirty)
		return;
	unsigned long now = millis();
	if (now - lastEESaveMs < 5000)
		return;
	saveMaxValuesNow();
	lastEESaveMs = now;
	maxDirty = false;
}

// =========================================================
// Button
// =========================================================
void buttonUpdate()
{
	const unsigned long now = millis();
	const bool sample = digitalRead(BTN_PIN);

	if (sample != btn.lastSample)
	{
		btn.lastSample = sample;
		btn.lastEdgeMs = now;
	}

	if ((now - btn.lastEdgeMs) >= DEBOUNCE_MS && sample != btn.stableLevel)
	{
		btn.stableLevel = sample;

		if (btn.stableLevel == LOW)
		{
			btn.pressed = true;
			btn.pressStartMs = now;
			btn.longFired = false;
			settingsLongFired = false;
			settingsPressStartMs = now;
		}
		else
		{
			// ---- button released ----
			if (settingsOpen)
			{
				if (btn.pressed && !settingsLongFired)
				{
					// short press in settings → next item
					settingsIdx = (settingsIdx + 1) % SET_COUNT;
					settingsLastActMs = now;
				}
			}
			else
			{
				if (btn.pressed && !btn.longFired)
				{
					page = (Page)((page + 1) % (PAGE_RACEBOX + 1));
					resetAnimUntilMs = 0;
				}
				raceboxBtnArmed = true; // re-arm after button fully released
			}
			btn.pressed = false;
		}
	}

	// ---- settings: 10s inactivity timeout ----
	if (settingsOpen && (now - settingsLastActMs) >= SETTINGS_TIMEOUT_MS)
	{
		settingsOpen = false;
		saveMaxValuesNow(); // persist settings
	}

	// ---- long press handling ----
	if (btn.pressed && !btn.longFired)
	{
		if (settingsOpen)
		{
			// long press inside settings → change value
			if (!settingsLongFired && (now - settingsPressStartMs) >= SETTINGS_LONGPRESS_MS)
			{
				settingsLongFired = true;
				btn.longFired = true;
				settingsLastActMs = now;

				switch (settingsIdx)
				{
					case SET_DS18_OFFSET:
					DS18B20_OFFSET += 0.5f;
						if (DS18B20_OFFSET > 3.0f) DS18B20_OFFSET = -3.0f;
						break;
				case SET_G_DEAD:
					{
						// fixed presets: 0.00 → 0.02 → 0.04 → 0.06 → 0.10 → 0.15 → 0.20 → 0.00 ...
						static const float kGDeadPresets[] = { 0.00f, 0.02f, 0.04f, 0.06f, 0.10f, 0.15f, 0.20f };
						static const uint8_t kGDeadN = sizeof(kGDeadPresets) / sizeof(kGDeadPresets[0]);
						// find current index (nearest match)
						uint8_t idx = 0;
						float best = fabsf(G_DEADZONE - kGDeadPresets[0]);
						for (uint8_t p = 1; p < kGDeadN; p++) {
							float d = fabsf(G_DEADZONE - kGDeadPresets[p]);
							if (d < best) { best = d; idx = p; }
						}
						idx = (idx + 1) % kGDeadN;
						G_DEADZONE = kGDeadPresets[idx];
					}
						break;
					case SET_BRIGHTNESS:
						brightMode = (brightMode + 1) % 3;
						// apply immediately
						if (brightMode == 1) oledSetContrast(CONTRAST_DAY);
						else if (brightMode == 2) oledSetContrast(CONTRAST_NIGHT);
						break;
					case SET_LEAN_FLIP:
					leanFlip = !leanFlip;
					break;
					case SET_RESET_ALL:
						maxLeanSaved = 0.0f;
						maxLeanLeft  = 0.0f;
						maxLeanRight = 0.0f;
						maxGSaved    = 0.0f;
						memset(gQuadPeaks,   0, sizeof(gQuadPeaks));
						memset(gQuadHasSlot, 0, sizeof(gQuadHasSlot));
						maxDirty = true;
						break;
					default: break;
				}
			}
		}
		else
		{
			// ---- normal long press ----
			// 5s on the oil page → open settings
			if (page == PAGE_OIL)
			{
				if (!btn.longFired && (now - btn.pressStartMs) >= SETTINGS_OPEN_MS)
				{
					btn.longFired = true;
					settingsOpen  = true;
					settingsIdx   = 0;
					settingsLastActMs = now;
					settingsLongFired = false;
				}
				return; // on oil page, no other long-press action exists
			}

			const unsigned long threshold = (page == PAGE_RACEBOX) ? RACEBOX_LONGPRESS_MS : LONGPRESS_MS;
			if ((now - btn.pressStartMs) >= threshold)
			{
				btn.longFired = true;

				if (page == PAGE_LEAN)
				{
					maxLeanSaved = 0.0f;
					maxLeanLeft  = 0.0f;
					maxLeanRight = 0.0f;
					maxDirty = true;
					resetAnimUntilMs = now + 350;
				}
				else if (page == PAGE_G)
				{
					maxGSaved = 0.0f;
					memset(gQuadPeaks,   0, sizeof(gQuadPeaks));
					memset(gQuadHasSlot, 0, sizeof(gQuadHasSlot));
					maxDirty = true;
					resetAnimUntilMs = now + 350;
				}
				else if (page == PAGE_RACEBOX && raceboxBtnArmed)
				{
					digitalWrite(RACEBOX_BTN_PIN, HIGH);
					raceboxBtnUntilMs = now + 1000;
					raceboxBtnArmed = false;
				}
			}
		}
	}
}

// =========================================================
// Oil temp
// =========================================================
float readAdsVoltage(int ch)
{
	if (!adsOk)
		return NAN;
	int16_t raw = ads.readADC_SingleEnded(ch);
	if (raw < 0)
		return NAN;
	return raw * ADS_SCALE;
}

float readOilTempOnce()
{
	float v = readAdsVoltage(ADS_CH_OIL);
	if (isnan(v))
		return NAN;
	if (v <= 0.001f)
		return NAN;
	if (v >= (DIVIDER_VCC - 0.001f))
		return NAN;

	float rNtc = R_REF * v / (DIVIDER_VCC - v);
	float tempK = 1.0f / ((1.0f / (T0C + 273.15f)) + (1.0f / BETA) * logf(rNtc / R0));
	float tempC = tempK - 273.15f;
	if (tempC < -20.0f || tempC > 160.0f)
		return NAN;
	return tempC;
}

void updateOutsideTemp()
{
	if (!ds18b20Found)
		return;

	unsigned long now = millis();

	// first call: kick off initial conversion
	if (lastOutsideMs == 0 && !outsideConvRequested)
	{
		dsSensors.requestTemperaturesByAddress(outsideSensorAddr);
		outsideConvRequested = true;
		lastOutsideMs = now;
		return;
	}

	if (outsideConvRequested && (now - lastOutsideMs >= 800))
	{
		// 800ms is enough for 12-bit DS18B20 conversion
		outsideConvRequested = false;
		float t = dsSensors.getTempC(outsideSensorAddr);
		if (t != DEVICE_DISCONNECTED_C && t > -50.0f && t < 85.0f)
			outsideTemp = t + DS18B20_OFFSET;
		lastOutsideMs = now;
		return;
	}

	// request new conversion periodically
	if (!outsideConvRequested && (now - lastOutsideMs >= OUTSIDE_INTERVAL_MS))
	{
		dsSensors.requestTemperaturesByAddress(outsideSensorAddr);
		outsideConvRequested = true;
		lastOutsideMs = now;
	}
}

// updateOilTemp: single-read EMA, called from updateAdsReadings
void updateOilTemp()
{
	float t = readOilTempOnce();
	if (isnan(t))
		return;
	if (isnan(oilTempCached))
		oilTempCached = t;  // first valid reading
	else
		oilTempCached += OIL_EMA_ALPHA * (t - oilTempCached);
}

float readBatteryVoltage()
{
	float v = readAdsVoltage(ADS_CH_BATT);
	if (isnan(v))
		return NAN;
	// correct divider + calibration
	return v * (BATT_R_TOP + BATT_R_BOT) / BATT_R_BOT * BATT_CAL;
}

// updateAdsReadings: one ADS1115 read per call, rotating through 3 channels
void updateAdsReadings()
{
	if (!adsOk)
		return;
	unsigned long now = millis();
	if (now - lastAdsReadMs < ADS_READ_INTERVAL_MS)
		return;
	lastAdsReadMs = now;

	if (adsNextChannel == 0)
	{
		// oil temp
		updateOilTemp();
		adsNextChannel = 1;
	}
	else if (adsNextChannel == 1)
	{
		// Blitzer-Warner heartbeat: pulse raises line from ~2.6V to ~3.5V
		float v = readAdsVoltage(ADS_CH_BLITZER_ALIVE);
		if (!isnan(v) && v > BLITZER_ALIVE_V_THRESHOLD)
		{
			blitzerAliveLastMs   = now;
			blitzerAliveReceived = true;
		}
		adsNextChannel = 2;
	}
	else
	{
		// battery voltage
		float v = readBatteryVoltage();
		if (!isnan(v))
		{
			if (isnan(battVoltageCached))
				battVoltageCached = v;
			else
				battVoltageCached += BATT_EMA_ALPHA * (v - battVoltageCached);
		}
		adsNextChannel = 0;
	}
}

// =========================================================
// Curve Peak Hold
// =========================================================
void updateCurvePeakHold(float leanAbs)
{
	unsigned long now = millis();

	if (!cornerActive)
	{
		if (leanAbs >= CORNER_ENTER_DEG)
		{
			cornerActive = true;
			cornerPeak = leanAbs;
			belowExitSinceMs = 0;

			holdUntilMs = 0;
			holdLean = 0.0f;
			cornerAboveThreshold = false;
			cornerDeclineStartMs = 0;
			lastCornerSign = (rollFiltered >= 0.0f) ? 1 : -1;
		}
		else
		{
			// Opposite-side cancel: if still in hold and rider crosses to other side
			if (now < holdUntilMs)
			{
				bool cancel = (lastCornerSign > 0 && rollFiltered < -CENTER_OPPOSITE_CANCEL) ||
				              (lastCornerSign < 0 && rollFiltered >  CENTER_OPPOSITE_CANCEL);
				if (cancel)
				{
					holdUntilMs = 0;
					holdLean    = 0.0f;
				}
			}
		}
		return;
	}

	// Direction reversal during active corner → immediately cancel peak display
	{
		bool reversed = (lastCornerSign > 0 && rollFiltered < -CENTER_OPPOSITE_CANCEL) ||
		                (lastCornerSign < 0 && rollFiltered >  CENTER_OPPOSITE_CANCEL);
		if (reversed)
		{
			cornerActive         = false;
			cornerAboveThreshold = false;
			belowExitSinceMs     = 0;
			holdUntilMs          = 0;
			holdLean             = 0.0f;
			cornerPeak           = 0.0f;
			cornerDeclineStartMs = 0;
			// If already deep enough in the new direction, start fresh corner
			if (leanAbs >= CORNER_ENTER_DEG)
			{
				cornerActive    = true;
				cornerPeak      = leanAbs;
				lastCornerSign  = (rollFiltered >= 0.0f) ? 1 : -1;
			}
			return;
		}
	}

	if (leanAbs > cornerPeak)
		cornerPeak = leanAbs;

	if (cornerPeak >= CENTER_PEAK_THRESHOLD)
		cornerAboveThreshold = true;

	// Track when lean drops back below threshold after peak was reached
	if (cornerAboveThreshold)
	{
		if (leanAbs >= CENTER_PEAK_THRESHOLD)
		{
			cornerDeclineStartMs = 0; // still above threshold, reset timer
		}
		else if (cornerDeclineStartMs == 0)
		{
			cornerDeclineStartMs = now; // just dropped below, start timer
		}
	}

	if (leanAbs <= CORNER_EXIT_DEG)
	{
		if (belowExitSinceMs == 0)
			belowExitSinceMs = now;

		if (now - belowExitSinceMs >= CORNER_EXIT_MS)
		{
			cornerActive = false;
			belowExitSinceMs = 0;

			if (cornerAboveThreshold)
			{
				holdLean    = cornerPeak;
				holdUntilMs = now + HOLD_AFTER_CORNER_MS;
			}
			cornerAboveThreshold = false;
			cornerPeak = 0.0f;
		}
	}
	else
	{
		belowExitSinceMs = 0;
	}
}

// =========================================================
// Sensors update
// =========================================================
void updateLean()
{
	if (!bnoOk)
		return;

	pollBno085();
	float roll = normalizeAngleDeg(bno085Roll - rollOffsetDeg) * (leanFlip ? -1.0f : 1.0f);

	static bool initDone = false;
	if (!initDone)
	{
		rollFiltered = roll;
		rollUi = roll;
		initDone = true;
	}

	rollFiltered += LEAN_ALPHA * (roll - rollFiltered);

	float target = rollFiltered;
	if (fabs(target) < LEAN_DEADZONE_DEG)
		target = 0.0f;

	rollUi += LEAN_UI_ALPHA * (target - rollUi);

	float leanAbs = fabs(rollFiltered);
	if (leanAbs < 0.5f)
		leanAbs = 0.0f;

	if (leanAbs > maxLeanSaved)
	{
		maxLeanSaved = leanAbs;
		maxDirty = true;
	}

	if (rollFiltered < -LEAN_DEADZONE_DEG && leanAbs > maxLeanLeft)
	{
		maxLeanLeft = leanAbs;
		maxDirty = true;
	}
	if (rollFiltered > LEAN_DEADZONE_DEG && leanAbs > maxLeanRight)
	{
		maxLeanRight = leanAbs;
		maxDirty = true;
	}

	updateCurvePeakHold(leanAbs);
}

void updateGForce()
{
	if (!bnoOk)
		return;

	float gxNow = bno085LinX / G0;
	float gyNow = bno085LinY / G0;

	// Deadzone: Motorvibrationen unter Schwellwert auf Null setzen
	if (fabsf(gxNow) < G_DEADZONE) gxNow = 0.0f;
	if (fabsf(gyNow) < G_DEADZONE) gyNow = 0.0f;

	// Langsamer Filter für den Anzeigedot — weniger Vibrations-Jitter
	gX += G_ALPHA_DISPLAY * (gxNow - gX);
	gY += G_ALPHA_DISPLAY * (gyNow - gY);

	// Schnellerer Filter für Max-G-Tracking — echte Manöver gehen nicht verloren
	gXFast += G_ALPHA_MAX * (gxNow - gXFast);
	gYFast += G_ALPHA_MAX * (gyNow - gYFast);

	float mag = sqrtf(gXFast * gXFast + gYFast * gYFast);
	if (mag > maxGSaved)
	{
		maxGSaved = mag;
		maxDirty = true;
	}
	// Quadranten-Peak-Puffer aktualisieren
	{
		uint8_t q = gQuadrant(gXFast, gYFast);
		GFPeak np = { gXFast, gYFast, mag };
		if (!gQuadHasSlot[q][0] || mag > gQuadPeaks[q][0].mag)
		{
			gQuadPeaks[q][1] = gQuadPeaks[q][0];
			gQuadHasSlot[q][1] = gQuadHasSlot[q][0];
			gQuadPeaks[q][0] = np;
			gQuadHasSlot[q][0] = true;
		}
		else if (!gQuadHasSlot[q][1] || mag > gQuadPeaks[q][1].mag)
		{
			gQuadPeaks[q][1] = np;
			gQuadHasSlot[q][1] = true;
		}
	}
}

void updateNightMode()
{
	// Manual override: Tag or Nacht fixed, no sensor needed
	if (brightMode == 1)
	{
		contrastF = (float)CONTRAST_DAY;
		oledSetContrast(CONTRAST_DAY);
		return;
	}
	if (brightMode == 2)
	{
		contrastF = (float)CONTRAST_NIGHT;
		oledSetContrast(CONTRAST_NIGHT);
		return;
	}

	// Auto mode: use BH1750
	if (!bhOk)
		return;

	float lux = lightMeter.readLightLevel();
	if (isnan(lux) || lux < 0 || lux >= 65535.0f)
		return;

	const float LUX_ALPHA = 0.12f;
	luxFiltered += LUX_ALPHA * (lux - luxFiltered);

	float t = (luxFiltered - LUX_NIGHT) / (LUX_DAY - LUX_NIGHT);
	t = clampf(t, 0.0f, 1.0f);

	float targetContrast = (float)CONTRAST_NIGHT + t * ((float)CONTRAST_DAY - (float)CONTRAST_NIGHT);
	contrastF += CONTRAST_FADE_ALPHA * (targetContrast - contrastF);

	unsigned long now = millis();
	if (now - lastContrastPushMs > 60)
	{
		lastContrastPushMs = now;
		uint8_t c = (uint8_t)clampf(contrastF, 0.0f, 255.0f);
		oledSetContrast(c);
	}

	if (ENABLE_ULTRADARK_INVERT)
	{
		if (!displayInverted && luxFiltered < LUX_INVERT_ON)
		{
			display.invertDisplay(true);
			displayInverted = true;
		}
		else if (displayInverted && luxFiltered > LUX_INVERT_OFF)
		{
			display.invertDisplay(false);
			displayInverted = false;
		}
	}
}

// =========================================================
// UI helpers
// =========================================================
void drawCenteredBigNumberWithDegree(int value, int16_t baselineY)
{
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
	int16_t topY = y1;
	int16_t rightX = x1 + (int16_t)w;

	const int r = 3;
	int16_t cx = rightX + r + 4;
	int16_t cy = topY + r - 4;

	if (cx > 127 - r)
		cx = 127 - r;
	if (cy < r)
		cy = r;
	if (cy > 63 - r)
		cy = 63 - r;

	display.drawCircle(cx, cy, r, SSD1306_WHITE);
}

void drawCenteredTitleTiny(const char *text, int16_t baselineY)
{
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
void drawLeanSemiGauge(float rollDeg)
{
	const int16_t cx = 64;
	const int16_t cy = 63;
	const int16_t r = 28;

	const float maxDeg = 60.0f;
	if (fabs(rollDeg) < LEAN_DEADZONE_DEG)
		rollDeg = 0.0f;
	rollDeg = clampf(rollDeg, -maxDeg, +maxDeg);

	const float dangerDeg = 40.0f;

	for (int a = 0; a <= 180; a += 2)
	{
		float offset = fabs((float)a - 90.0f);
		float degHere = (offset / 90.0f) * maxDeg;

		float rad = a * 3.1415926f / 180.0f;

		int16_t x1p = cx + (int16_t)roundf(cosf(rad) * r);
		int16_t y1p = cy - (int16_t)roundf(sinf(rad) * r);
		display.drawPixel(x1p, y1p, SSD1306_WHITE);

		if (degHere >= dangerDeg)
		{
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

	if (rollDeg != 0.0f)
	{
		const float step = 3.0f;

		if (a1 < a0)
		{
			for (float a = a0; a >= a1; a -= step)
			{
				float aPrev = a + step;
				float r1 = a * 3.1415926f / 180.0f;
				float r2 = aPrev * 3.1415926f / 180.0f;

				int16_t xA = cx + (int16_t)roundf(cosf(r1) * (r - 3));
				int16_t yA = cy - (int16_t)roundf(sinf(r1) * (r - 3));
				int16_t xB = cx + (int16_t)roundf(cosf(r2) * (r - 3));
				int16_t yB = cy - (int16_t)roundf(sinf(r2) * (r - 3));

				display.fillTriangle(cx, cy, xA, yA, xB, yB, SSD1306_WHITE);
			}
		}
		else
		{
			for (float a = a0; a <= a1; a += step)
			{
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
void drawHatchedRect(int x, int y, int w, int h, int spacing)
{
	for (int i = -h; i < w; i += spacing)
	{
		int x0 = x + i;
		int y0 = y + h - 1;
		int x1 = x0 + h;
		int y1 = y;

		int dx = x1 - x0;
		int dy = y1 - y0;
		int steps = max(abs(dx), abs(dy));
		for (int s = 0; s <= steps; s++)
		{
			int px = x0 + (dx * s) / steps;
			int py = y0 + (dy * s) / steps;
			if (px >= x && px < x + w && py >= y && py < y + h)
			{
				display.drawPixel(px, py, SSD1306_WHITE);
			}
		}
	}
}

// Small heartbeat indicator: filled circle = alive, blinking = offline, nothing = initialising
// Drawn at top-left corner (radius 2, center (2,4)) – 4px wide, sits just left of outside-temp text.
void drawBlitzerWarnerAliveIndicator()
{
	// Nur anzeigen wenn Warner schon mal da war, aber jetzt offline ist
	if (!blitzerAliveReceived)
		return;

	bool alive = (millis() - blitzerAliveLastMs) < BLITZER_ALIVE_TIMEOUT_MS;
	if (alive)
		return; // läuft normal → kein Punkt

	// Warner offline → blinkender Kreis als Warnung
	if (((millis() / 400) % 2) == 0)
		display.drawCircle(2, 4, 2, SSD1306_WHITE);
}

void drawOilBar(float oilC)
{
	const int x = 10;
	const int y = 56;
	const int w = 108;
	const int h = 6;

	display.drawRect(x, y, w, h, SSD1306_WHITE);
	bool valid = !isnan(oilC);

	int gx1 = x + 1 + mapf_to_i(OIL_GOOD_MIN, OIL_BAR_MIN_C, OIL_BAR_MAX_C, 0, w - 3);
	int gx2 = x + 1 + mapf_to_i(OIL_GOOD_MAX, OIL_BAR_MIN_C, OIL_BAR_MAX_C, 0, w - 3);
	if (gx2 < gx1)
	{
		int tmp = gx1;
		gx1 = gx2;
		gx2 = tmp;
	}
	int gw = gx2 - gx1 + 1;

	int fillRight = x + 1; // right edge of the solid fill
	if (valid)
	{
		float t = clampf(oilC, OIL_BAR_MIN_C, OIL_BAR_MAX_C);
		int fillW = mapf_to_i(t, OIL_BAR_MIN_C, OIL_BAR_MAX_C, 0, w - 2);
		if (fillW > 0)
		{
			display.fillRect(x + 1, y + 1, fillW, h - 2, SSD1306_WHITE);
			fillRight = x + 1 + fillW;
		}
	}

	display.drawFastVLine(gx1, y - 2, h + 4, SSD1306_WHITE);
	display.drawFastVLine(gx2, y - 2, h + 4, SSD1306_WHITE);

	if (valid && oilC > 100.0f)
	{
		bool on = ((millis() / 140) % 2) == 0;
		if (on)
		{
			display.fillRect(x, y, w, h, SSD1306_WHITE);
			display.fillRect(x + 1, y + 1, w - 2, h - 2, SSD1306_BLACK);
			display.drawFastVLine(gx1, y - 2, h + 4, SSD1306_WHITE);
			display.drawFastVLine(gx2, y - 2, h + 4, SSD1306_WHITE);
		}
	}
}

// fixed margin from screen edge used for both outside temp and battery
#define SIDE_MARGIN 4 // small gap from edge, adjust as needed

// Snowflake warning: drawn near outside temp when <= 0°C (ice risk)
static void drawSnowflakeWarning(int16_t cx, int16_t cy)
{
	// 3 lines through center (horizontal, vertical, diagonal)
	const int r = 4;
	display.drawFastHLine(cx - r, cy, 2 * r + 1, SSD1306_WHITE);
	display.drawFastVLine(cx, cy - r, 2 * r + 1, SSD1306_WHITE);
	display.drawLine(cx - r + 1, cy - r + 1, cx + r - 1, cy + r - 1, SSD1306_WHITE);
	display.drawLine(cx + r - 1, cy - r + 1, cx - r + 1, cy + r - 1, SSD1306_WHITE);
	// center dot
	display.drawPixel(cx, cy, SSD1306_WHITE);
}

static void drawBatteryTopRight()
{
	float batt = battVoltageCached;
	if (isnan(batt))
		return;
	bool lowBatt = (batt < BATT_LOW_V);
	if (lowBatt && ((millis() / 400) % 2) == 0)
		return; // blink off every other 400ms interval
	display.setFont();
	display.setTextSize(1);
	char buf[8];
	snprintf(buf, sizeof(buf), "%.1fV", batt);
	int16_t x1, y1;
	uint16_t w, h;
	display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
	display.setCursor(SCREEN_WIDTH - (int16_t)w - SIDE_MARGIN, 2);
	display.print(buf);
}

void drawOilPage(float oilC)
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);

	// outside temperature at fixed left margin, same vertical as battery
	display.setFont();
	display.setTextSize(1);
	if (!isnan(outsideTemp))
	{
		display.setCursor(SIDE_MARGIN, 2);
		display.print(outsideTemp, 1);
		display.print("C");
		// ice warning: snowflake when at or below 0°C
		if (outsideTemp <= 0.0f)
			drawSnowflakeWarning(SIDE_MARGIN + 34, 5);
	}
	else
	{
		display.setCursor(SIDE_MARGIN, 2);
		display.print("--");
	}

	int16_t baselineY = 45;
	if (isnan(oilC))
	{
		display.setFont(&FreeSansBold18pt7b);
		String s = "--";
		int16_t x1, y1;
		uint16_t w, h;
		display.getTextBounds(s, 0, baselineY, &x1, &y1, &w, &h);
		int16_t x = (display.width() - (int16_t)w) / 2;
		display.setCursor(x, baselineY);
		display.print(s);
	}
	else
	{
		drawCenteredBigNumberWithDegree((int)round(oilC), baselineY);
	}

	display.setFont();
	display.setTextSize(1);

	// status text: COLD below range, HOT above 95°C, nothing in OK range
	if (!isnan(oilC))
	{
		display.setCursor(2, 46);
		if (oilC < 60.0f)
			display.print("COLD");
		else if (oilC > 95.0f)
			display.print("HOT");
	}

	drawBatteryTopRight();

	drawOilBar(oilC);
	drawBlitzerWarnerAliveIndicator();

	// hold-progress bar: grows left→right while button held, shows 5s entry progress
	if (btn.pressed)
	{
		unsigned long held = millis() - btn.pressStartMs;
		int barW = (int)((float)held / (float)SETTINGS_OPEN_MS * 126.0f);
		if (barW > 126) barW = 126;
		if (barW > 0)
			display.fillRect(1, 63, barW, 1, SSD1306_WHITE);
	}

	display.display();
}

// =========================================================
// Lean page
// =========================================================
void drawLeanPage()
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);

	float liveLean = fabs(rollUi);

	// Hybrid center value:
	//  corner active, peak >= 15°, and decline timer not expired → show running corner peak
	//  in 4-sec hold after corner → show final corner peak
	//  otherwise → show live lean
	unsigned long nowMs = millis();
	float centerValue;
	bool declineExpired = cornerDeclineStartMs != 0 &&
	                      (nowMs - cornerDeclineStartMs >= HOLD_AFTER_CORNER_MS);
	// Only show live lean once the bike has crossed 2° to the opposite side of the peak
	bool crossedOpposite = (lastCornerSign > 0 && rollFiltered < -5.0f) ||
	                       (lastCornerSign < 0 && rollFiltered >  5.0f);
	if (!crossedOpposite && cornerActive && cornerAboveThreshold && !declineExpired)
	{
		centerValue = cornerPeak;
	}
	else if (!crossedOpposite && nowMs < holdUntilMs)
	{
		centerValue = holdLean;
	}
	else
	{
		centerValue = liveLean;
	}

	drawCenteredBigNumberWithDegree((int)round(centerValue), 28);

	display.setFont();
	display.setTextSize(1);

	// Max lean left (bottom-left)
	display.setCursor(2, display.height() - 9);
	display.print("L:");
	display.print(maxLeanLeft, 0);

	// Max lean right (bottom-right)
	String rStr = "R:" + String((int)round(maxLeanRight));
	int16_t x1, y1;
	uint16_t w, h;
	display.getTextBounds(rStr, 0, 0, &x1, &y1, &w, &h);
	int16_t xRight = SCREEN_WIDTH - (int16_t)w;

	display.setCursor(xRight, SCREEN_HEIGHT - 9);
	display.print(rStr);

	drawLeanSemiGauge(rollUi);

	// reset flash
	unsigned long now = millis();
	if (now < resetAnimUntilMs)
	{
		bool on = ((now / 80) % 2) == 0;
		if (on)
		{
			display.fillRect(36, 10, 56, 16, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			display.setCursor(48, 14);
			display.print("RESET");
			display.setTextColor(SSD1306_WHITE);
		}
	}

	drawBlitzerWarnerAliveIndicator();
	display.display();
}

// =========================================================
// G page
// =========================================================
void drawGCircle(float gx, float gy, float maxGVal)
{
	const int16_t cx = 64;
	const int16_t cy = 32;
	const int16_t r  = 26;
	const float rangeG = 1.5f;

	// Fadenkreuz (volle Achslinien)
	display.drawLine(cx - r, cy, cx + r, cy, SSD1306_WHITE);
	display.drawLine(cx, cy - r, cx, cy + r, SSD1306_WHITE);
	// äußerer Ring: 1.5g
	display.drawCircle(cx, cy, r, SSD1306_WHITE);
	// innerer Ring: 0.75g
	const int16_t rInner = (int16_t)lroundf((float)r * 0.5f);
	display.drawCircle(cx, cy, rInner, SSD1306_WHITE);

	// Ring-Legende: nur am äußeren Ring
	display.setFont();
	display.setTextSize(1);
	display.setCursor(cx + r + 2, cy - 4);
	display.print("1.5g");

	// Peak-Marker: Punkte nach Winkel sortieren → sauberes Polygon
	{
		int16_t ptx[8], pty[8];
		float   pta[8]; // Winkel für Sortierung
		uint8_t ptCount = 0;
		for (uint8_t q = 0; q < 4; q++)
		{
			for (uint8_t s = 0; s < 2; s++)
			{
				if (!gQuadHasSlot[q][s]) continue;
				float px_f = clampf(gQuadPeaks[q][s].x, -rangeG, rangeG);
				float py_f = clampf(gQuadPeaks[q][s].y, -rangeG, rangeG);
				ptx[ptCount] = cx + (int16_t)lroundf((px_f / rangeG) * (float)(r - 2));
				pty[ptCount] = cy - (int16_t)lroundf((py_f / rangeG) * (float)(r - 2));
				pta[ptCount] = atan2f(py_f, px_f); // Winkel für Sortierung
				ptCount++;
			}
		}
		// Insertion-Sort nach Winkel (max 8 Elemente → kein Overhead)
		for (uint8_t i = 1; i < ptCount; i++)
		{
			int16_t kx = ptx[i], ky = pty[i]; float ka = pta[i];
			int8_t j = (int8_t)i - 1;
			while (j >= 0 && pta[j] > ka)
			{
				ptx[j+1] = ptx[j]; pty[j+1] = pty[j]; pta[j+1] = pta[j];
				j--;
			}
			ptx[j+1] = kx; pty[j+1] = ky; pta[j+1] = ka;
		}
		// Polygon zeichnen
		for (uint8_t i = 0; i < ptCount; i++)
		{
			uint8_t j = (i + 1) % ptCount;
			display.drawLine(ptx[i], pty[i], ptx[j], pty[j], SSD1306_WHITE);
		}
		// Punkte obendrauf
		for (uint8_t i = 0; i < ptCount; i++)
			display.drawCircle(ptx[i], pty[i], 2, SSD1306_WHITE);
	}

	// Aktuellen Punkt pushen + zeichnen
	float x = clampf(gx, -rangeG, rangeG);
	float y = clampf(gy, -rangeG, rangeG);
	int16_t dx = cx + (int16_t)lroundf((x / rangeG) * (float)(r - 2));
	int16_t dy = cy - (int16_t)lroundf((y / rangeG) * (float)(r - 2));
	display.fillCircle(dx, dy, 3, SSD1306_WHITE);

	// Texte
	display.setFont();
	display.setTextSize(1);

	float mag = sqrtf(gx * gx + gy * gy);
	display.setCursor(1, 0);
	display.print(mag, 1);
	display.print("g");

	display.setCursor(1, display.height() - 9);
	display.print("M:");
	display.print(maxGVal, 1);
	display.print("g");
}

void drawGPage()
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);

	drawGCircle(gX, gY, maxGSaved);

	// reset flash
	unsigned long now = millis();
	if (now < resetAnimUntilMs)
	{
		bool on = ((now / 80) % 2) == 0;
		if (on)
		{
			display.fillRect(36, 10, 56, 16, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			display.setCursor(48, 14);
			display.print("RESET");
			display.setTextColor(SSD1306_WHITE);
		}
	}

	drawBlitzerWarnerAliveIndicator();
	display.display();
}

void drawRaceBoxPage()
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);
	display.setFont();
	display.setTextSize(1);

	// Hilfsfunktion: Text horizontal im Badge zentrieren
	auto printCentered = [&](int bx, int bw, int ty, const char* text) {
		int16_t x1, y1; uint16_t tw, th;
		display.getTextBounds(text, 0, ty, &x1, &y1, &tw, &th);
		display.setCursor(bx + ((int16_t)bw - (int16_t)tw) / 2, ty);
		display.print(text);
	};

	// Titel zentriert
	{
		const char* title = "RaceBox";
		int16_t x1, y1; uint16_t tw, th;
		display.getTextBounds(title, 0, 0, &x1, &y1, &tw, &th);
		display.setCursor((128 - (int16_t)tw) / 2, 0);
		display.print(title);
	}
	display.drawLine(0, 9, 127, 9, SSD1306_WHITE);

	const int BX = 50, BW = 32; // Badge x und Breite

	// BLE  (row 1)
	display.setCursor(8, 12);
	display.print("BLE");
	{
		bool bleAlive = raceboxBleAliveLastMs > 0 && (millis() - raceboxBleAliveLastMs) < RACEBOX_BLE_ALIVE_TIMEOUT_MS;
		if (raceboxBle) {
			display.fillRoundRect(BX, 11, BW, 10, 3, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			printCentered(BX, BW, 12, "CONN");
			display.setTextColor(SSD1306_WHITE);
		} else if (bleAlive) {
			// RaceBox ist an, aber kein Phone verbunden
			display.drawRoundRect(BX, 11, BW, 10, 3, SSD1306_WHITE);
			printCentered(BX, BW, 12, "ON");
		} else {
			display.drawRoundRect(BX, 11, BW, 10, 3, SSD1306_WHITE);
			printCentered(BX, BW, 12, "---");
		}
	}

	// REC  (row 2)
	display.setCursor(8, 25);
	display.print("REC");
	if (raceboxRec) {
		display.fillRoundRect(BX, 24, BW, 10, 3, SSD1306_WHITE);
		display.setTextColor(SSD1306_BLACK);
		printCentered(BX, BW, 25, "REC");
		display.setTextColor(SSD1306_WHITE);
	} else {
		display.drawRoundRect(BX, 24, BW, 10, 3, SSD1306_WHITE);
		printCentered(BX, BW, 25, "---");
	}

	// GPS  (row 3)
	display.setCursor(8, 38);
	display.print("GPS");
	if (raceboxGps) {
		display.fillRoundRect(BX, 37, BW, 10, 3, SSD1306_WHITE);
		display.setTextColor(SSD1306_BLACK);
		printCentered(BX, BW, 38, "FIX");
		display.setTextColor(SSD1306_WHITE);
	} else {
		display.drawRoundRect(BX, 37, BW, 10, 3, SSD1306_WHITE);
		printCentered(BX, BW, 38, "---");
	}

	// Blitzer Warner alive  (row 4)
	display.setCursor(8, 51);
	display.print("BLT");
	{
		bool alive = blitzerAliveReceived && (millis() - blitzerAliveLastMs) < BLITZER_ALIVE_TIMEOUT_MS;
		if (blitzerAliveReceived && alive) {
			display.fillRoundRect(BX, 50, BW, 10, 3, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			printCentered(BX, BW, 51, "ON");
			display.setTextColor(SSD1306_WHITE);
		} else if (blitzerAliveReceived && !alive) {
			if (((millis() / 400) % 2) == 0)
				display.drawRoundRect(BX, 50, BW, 10, 3, SSD1306_WHITE);
			printCentered(BX, BW, 51, "OFF");
		} else {
			display.drawRoundRect(BX, 50, BW, 10, 3, SSD1306_WHITE);
			printCentered(BX, BW, 51, "???");
		}
	}

	// Schaltflächen-Indikator oben rechts
	if (raceboxBtnUntilMs > 0 && millis() < raceboxBtnUntilMs) {
		display.fillRoundRect(95, 0, 33, 9, 2, SSD1306_WHITE);
		display.setTextColor(SSD1306_BLACK);
		printCentered(95, 33, 1, "REC");
		display.setTextColor(SSD1306_WHITE);
	}

	drawBlitzerWarnerAliveIndicator();
	display.display();
}

// =========================================================
// Settings UI helpers
// =========================================================
void drawSettingsPage()
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);
	display.setFont();
	display.setTextSize(1);

	// Title bar
	display.fillRect(0, 0, 128, 10, SSD1306_WHITE);
	display.setTextColor(SSD1306_BLACK);
	display.setCursor(3, 2);
	display.print("EINSTELLUNGEN");
	display.setTextColor(SSD1306_WHITE);

	// Items – 5 rows of 10px each starting at y=13
	const char* labels[SET_COUNT] = {
		"AuTemp Offset",
		"G-Deadzone",
		"Helligkeit",
		"Lean flip",
		"Alles loeschen"
	};

	char valBuf[14];
	for (uint8_t i = 0; i < SET_COUNT; i++)
	{
		int16_t y = 13 + i * 10;

		if (i == settingsIdx)
			display.fillRect(0, y - 1, 128, 10, SSD1306_WHITE);

		display.setTextColor(i == settingsIdx ? SSD1306_BLACK : SSD1306_WHITE);
		display.setCursor(2, y);
		display.print(labels[i]);

		switch (i)
		{
			case SET_DS18_OFFSET:
				snprintf(valBuf, sizeof(valBuf), "%+.1fC", DS18B20_OFFSET);
				break;
			case SET_G_DEAD:
				snprintf(valBuf, sizeof(valBuf), "%.2fg", G_DEADZONE);
				break;
			case SET_BRIGHTNESS:
			{
				const char* bNames[] = { "Auto", "Tag", "Nacht" };
				snprintf(valBuf, sizeof(valBuf), "%s", bNames[brightMode]);
				break;
			}
			case SET_LEAN_FLIP:
				snprintf(valBuf, sizeof(valBuf), "%s", leanFlip ? "AN" : "AUS");
				break;
			case SET_RESET_ALL:
				snprintf(valBuf, sizeof(valBuf), "HOLD");
				break;
			default:
				valBuf[0] = 0;
				break;
		}

		int16_t vx = 128 - (int16_t)(strlen(valBuf) * 6) - 2;
		display.setCursor(vx, y);
		display.print(valBuf);
	}
	display.setTextColor(SSD1306_WHITE);

	// hold-progress bar at bottom while pressing
	unsigned long now = millis();
	if (btn.pressed)
	{
		unsigned long held = now - settingsPressStartMs;
		int barW = (int)((float)held / (float)SETTINGS_LONGPRESS_MS * 128.0f);
		if (barW > 128) barW = 128;
		if (barW > 0)
			display.fillRect(0, 63, barW, 1, SSD1306_WHITE);
	}

	// timeout countdown bar (bottom line shrinks as timeout approaches)
	{
		unsigned long elapsed = now - settingsLastActMs;
		int barW = 128 - (int)((float)elapsed / (float)SETTINGS_TIMEOUT_MS * 128.0f);
		if (barW < 0) barW = 0;
		display.drawFastHLine(128 - barW, 63, barW, SSD1306_WHITE);
	}

	display.display();
}

// =========================================================
void calibrateRollOffset()
{
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

	while (millis() - tStart < durMs)
	{
		pollBno085();
		sum += bno085Roll;
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
void drawCalIconTopRight(bool bnoOK, bool armed)
{
	if (!bnoOK)
		return;

	const int16_t y = 6;
	const int16_t x0 = 96;
	const int16_t w = 32; 
	const int16_t h = 10;

	// clear whole area each frame
	display.fillRect(x0, y - 1, w, h, SSD1306_BLACK);

	// button circle
	const int16_t cx = x0 + 6;
	const int16_t cy = y + 2;
	const int16_t r = 3;
	display.drawCircle(cx, cy, r, SSD1306_WHITE);
	display.fillCircle(cx, cy, 1, SSD1306_WHITE);

	// CAL text
	const int16_t tx = x0 + 12;

	if (!armed)
	{
		display.setTextColor(SSD1306_WHITE);
		display.setCursor(tx, y);
		display.print("CAL");
	}
	else
	{
		display.fillRect(tx - 1, y - 1, 20, 9, SSD1306_WHITE);
		display.setTextColor(SSD1306_BLACK);
		display.setCursor(tx, y);
		display.print("CAL");
		display.setTextColor(SSD1306_WHITE);
	}
}

// status: -1 = pending, 0 = fail, 1 = ok
static void drawSelfTestLineProgress(int y, const char *label, int8_t st)
{
	display.setFont();
	display.setTextSize(1);

	// label links
	display.setCursor(8, y);
	display.print(label);

	// status immer bei fester X-Position (60) → alle "OK" auf gleicher Höhe
	const int16_t statusX = 60;
	display.setCursor(statusX, y);

	if (st < 0)
	{
		display.print("...");
		return;
	}

	if (st > 0)
	{
		display.print("OK");
	}
	else
	{
		bool on = ((millis() / 160) % 2) == 0;
		if (on)
		{
			display.fillRect(statusX - 1, y - 1, 30, 10, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			display.setCursor(statusX, y);
			display.print("FAIL");
			display.setTextColor(SSD1306_WHITE);
		}
	}
}

static void renderBootProgress(int8_t stBno, int8_t stBh, int8_t stEe, int8_t stAds, bool calArmed, float prog01)
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);
	display.setFont();
	display.setTextSize(1);

	display.setCursor(8, 6);
	display.print("Self-Test");

	drawSelfTestLineProgress(20, "BNO085",  stBno);
	drawSelfTestLineProgress(30, "BH1750",  stBh);
	drawSelfTestLineProgress(40, "ADS1115", stAds);
	drawSelfTestLineProgress(50, "EEPROM",  stEe);

	drawCalIconTopRight(stBno > 0, calArmed);

	const int barX = 8, barY = 59, barW = 112, barH = 4;
	display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);
	int fill = (int)((barW - 2) * clampf(prog01, 0.0f, 1.0f));
	if (fill > 0)
		display.fillRect(barX + 1, barY + 1, fill, barH - 2, SSD1306_WHITE);

	display.display();
}

void bootProgressInitAndMaybeCalibrate()
{
	int8_t stBno = -1, stBh = -1, stEe = -1, stAds = -1;

	// show instantly
	renderBootProgress(stBno, stBh, stEe, stAds, false, 0.0f);

	// EEPROM
	eepromOk = loadMaxValues();
	stEe = eepromOk ? 1 : 0;
	renderBootProgress(stBno, stBh, stEe, stAds, false, 0.15f);

	// BH1750
	bhOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
	stBh = bhOk ? 1 : 0;
	renderBootProgress(stBno, stBh, stEe, stAds, false, 0.3f);

	// ADS1115
	adsOk = ads.begin();
	stAds = adsOk ? 1 : 0;
	if (adsOk) {
		ads.setGain(ADS_GAIN);
		delay(5);  // let ADS settle after gain change
		// pre-fill oil temp: retry up to 5× so first frame always shows a reading
		for (int attempt = 0; attempt < 5 && isnan(oilTempCached); attempt++) {
			float t = readOilTempOnce();
			if (!isnan(t)) oilTempCached = t;
			else delay(20);
		}
	}
	renderBootProgress(stBno, stBh, stEe, stAds, false, 0.45f);

	// BNO085
	bnoOk = bno.begin_I2C();
	if (bnoOk)
	{
		// GAME_ROTATION_VECTOR: fused gyro+accel, NO magnetometer → no drift from vibration
		bno.enableReport(SH2_GAME_ROTATION_VECTOR, 10000);  // 10ms = 100Hz
		bno.enableReport(SH2_LINEAR_ACCELERATION,  10000);  // 10ms = 100Hz
		delay(100);
	}
	rollOffsetDeg = -3.00f;  // hardcoded mount offset (measured)
	stBno = bnoOk ? 1 : 0;
	renderBootProgress(stBno, stBh, stEe, stAds, false, 0.6f);

	// kick off DS18B20 conversion now — will be ready after 2s wait
	if (ds18b20Found)
	{
		dsSensors.requestTemperaturesByAddress(outsideSensorAddr);
	}

	// wait 2s with hold-to-cal — bar fills from 0.6 → 1.0 during this time
	const unsigned long waitMs = 2000;
	unsigned long t0 = millis();

	unsigned long holdStart = 0;
	bool calArmed = false;

	while (millis() - t0 < waitMs)
	{
		unsigned long now = millis();
		float prog = 0.6f + 0.4f * clampf((float)(now - t0) / (float)waitMs, 0.0f, 1.0f);

		if (bnoOk && digitalRead(BTN_PIN) == LOW)
		{
			if (holdStart == 0)
				holdStart = now;
			if (!calArmed && (now - holdStart) >= 250)
				calArmed = true; // latch
		}
		else
		{
			holdStart = 0;
		}

		updateAdsReadings();  // keep pumping ADS during splash so oil temp is ready
		renderBootProgress(stBno, stBh, stEe, stAds, calArmed, prog);
		delay(25);
		yield();
	}

	if (bnoOk && calArmed)
	{
		calibrateRollOffset();
	}

	// DS18B20 conversion is done after 2s wait — read result now
	if (ds18b20Found)
	{
		float t = dsSensors.getTempC(outsideSensorAddr);
		if (t != DEVICE_DISCONNECTED_C && t > -50.0f && t < 85.0f)
			outsideTemp = t + DS18B20_OFFSET;
		outsideConvRequested = false;
		lastOutsideMs = millis();
	}
}

void showReadyScreen()
{
	const char *txt = "READY";

	// measure text with 18pt font
	display.setFont(&FreeSansBold18pt7b);
	display.setTextSize(1);
	int16_t x1, y1;
	uint16_t tw, th;
	display.getTextBounds(txt, 0, 0, &x1, &y1, &tw, &th);
	int16_t cx = (SCREEN_WIDTH  - (int16_t)tw) / 2 - x1;
	int16_t cy = (SCREEN_HEIGHT - (int16_t)th) / 2 - y1;

	// left-to-right reveal: draw full text, mask right portion with black rect shrinking each frame
	const unsigned long dur = 380;
	unsigned long t0 = millis();

	while (true)
	{
		unsigned long elapsed = millis() - t0;
		float prog = clampf((float)elapsed / (float)dur, 0.0f, 1.0f);
		int revealX = (int)(prog * (float)(x1 + (int16_t)tw + cx)); // right edge of revealed area

		display.clearDisplay();
		display.setTextColor(SSD1306_WHITE);
		display.setFont(&FreeSansBold18pt7b);
		display.setCursor(cx, cy);
		display.print(txt);

		// black mask covers everything to the right of revealX
		if (revealX < SCREEN_WIDTH)
			display.fillRect(revealX, 0, SCREEN_WIDTH - revealX, SCREEN_HEIGHT, SSD1306_BLACK);

		display.display();

		if (prog >= 1.0f) break;
		delay(16);
	}
	delay(180); // brief hold after fully revealed
}

// =========================================================
// Setup / Loop
// =========================================================
void setup()
{
	Serial.begin(115200);
	delay(200);

	pinMode(BTN_PIN, INPUT_PULLUP);
	pinMode(RACEBOX_BTN_PIN, OUTPUT);
	digitalWrite(RACEBOX_BTN_PIN, LOW); // transistor off = button open
	pinMode(RACEBOX_GPS_PIN, INPUT_PULLUP); // cathode: LOW = LED on
	pinMode(RACEBOX_BLE_PIN, INPUT_PULLUP); // cathode: LOW = LED on
	pinMode(RACEBOX_REC_PIN, INPUT_PULLUP); // cathode: LOW = LED on
	pinMode(BLITZER_PIN, INPUT_PULLUP); // LOW = blitzer detected (active low)
	// Blitzer-Warner heartbeat LED is measured via ADS1115 AIN1 (no GPIO needed)

	Wire.begin(SDA_PIN, SCL_PIN);
	Wire.setClock(100000);

	// start one‑wire bus for DS18B20
	pinMode(ONE_WIRE_PIN, INPUT_PULLUP);  // internal pull-up as substitute for 4.7kΩ resistor
	delay(10);
	dsSensors.begin();
	dsSensors.setWaitForConversion(false);  // non-blocking mode!

	if (dsSensors.getDeviceCount() > 0 && dsSensors.getAddress(outsideSensorAddr, 0))
	{
		ds18b20Found = true;
		dsSensors.setResolution(outsideSensorAddr, 12);
	}
	else
	{
		ds18b20Found = false;
		Serial.print("Warning: no DS18B20 found on GPIO ");
		Serial.println(ONE_WIRE_PIN);
	}

	// ADS1115 is initialized in bootProgressInitAndMaybeCalibrate()

	// configure hardware SPI pins before display.begin
	// if you're using the default VSPI pins, SPI.begin() with no arguments is fine
	// otherwise specify sck, miso, mosi, ss
	SPI.begin(OLED_CLK, /*MISO*/ -1, OLED_MOSI, OLED_CS);

	if (!display.begin(SSD1306_SWITCHCAPVCC))
	{
		while (true) { delay(100); }
	}
	
	display.setRotation(0);
	display.setTextColor(SSD1306_WHITE);
	display.clearDisplay();
	display.display();

	oledSetContrast(CONTRAST_DAY);

	rollOffsetDeg = -3.00f;  // hardcoded mount offset

	bootProgressInitAndMaybeCalibrate();

	// pre-fill oil + battery so first frame never shows NaN
	for (int i = 0; i < 4; i++)
	{
		float t = readOilTempOnce();
		if (!isnan(t)) { oilTempCached = t; break; }
		delay(20);
	}
	battVoltageCached = readBatteryVoltage();

	// outside temp was already read during boot wait — no extra request needed

	showReadyScreen();
}

void loop()
{
	buttonUpdate();

	updateLean();
	updateGForce();
	updateNightMode();
	updateOutsideTemp();
	updateAdsReadings();  // one ADS read per loop: oil or battery alternating

	saveMaxValuesSometimes();

	// release relay after 1s
	if (raceboxBtnUntilMs > 0 && millis() >= raceboxBtnUntilMs)
	{
		digitalWrite(RACEBOX_BTN_PIN, LOW); // transistor off = button open
		raceboxBtnUntilMs = 0;
	}

	static unsigned long lastDraw = 0;
	unsigned long now = millis();

	if (now - lastDraw >= 75)
	{
		lastDraw = now;

		// read RaceBox status pins every frame
		// Pins müssen >= 2s kontinuierlich HIGH sein bevor LOW zählt.
		// Verhindert false-trigger durch Boot-Blip (~ms) oder Backfeed wenn Gerät aus ist (dauerhaft LOW).
		// GPS
		{
			bool isLow = digitalRead(RACEBOX_GPS_PIN) == LOW;
			if (!isLow) {
				if (raceboxGpsHighSinceMs == 0) raceboxGpsHighSinceMs = millis();
				raceboxGpsLowSinceMs = 0;
				if (!raceboxGpsPinOk && (millis() - raceboxGpsHighSinceMs) >= RACEBOX_PIN_VALID_HIGH_BLE_GPS_MS)
					raceboxGpsPinOk = true;
			} else {
				raceboxGpsHighSinceMs = 0;
				if (raceboxGpsPinOk) {
					if (raceboxGpsLowSinceMs == 0) raceboxGpsLowSinceMs = millis();
					if ((millis() - raceboxGpsLowSinceMs) >= 6000) // 6s: filters boot flash, real fix stays on minutes
						{ raceboxGpsLastActiveMs = millis(); raceboxGpsEverSeen = true; }
				}
			}
			if (raceboxGpsEverSeen && (millis() - raceboxGpsLastActiveMs) >= RACEBOX_GPS_HOLD_MS)
				raceboxGpsEverSeen = false;
			raceboxGps = raceboxGpsEverSeen;
		}
		// BLE – 1000ms Debounce: Suchen = kurze Blinks (<1s) werden ignoriert für CONN;
		// jeder Blink (>0ms nach PinOk) aktualisiert raceboxBleAliveLastMs → zeigt "ON"
		{
			bool isLow = digitalRead(RACEBOX_BLE_PIN) == LOW;
			if (!isLow) {
				if (raceboxBleHighSinceMs == 0) raceboxBleHighSinceMs = millis();
				raceboxBleLowSinceMs = 0;
				if (!raceboxBlePinOk && (millis() - raceboxBleHighSinceMs) >= RACEBOX_PIN_VALID_HIGH_BLE_GPS_MS)
					raceboxBlePinOk = true;
			} else {
				raceboxBleHighSinceMs = 0;
				if (raceboxBlePinOk) {
					raceboxBleAliveLastMs = millis(); // jeder LOW-Moment nach PinOk = Gerät ist an
					if (raceboxBleLowSinceMs == 0) raceboxBleLowSinceMs = millis();
					if ((millis() - raceboxBleLowSinceMs) >= 1000)
						{ raceboxBleLastActiveMs = millis(); raceboxBleEverSeen = true; }
				}
			}
			if (raceboxBleEverSeen && (millis() - raceboxBleLastActiveMs) >= RACEBOX_BLE_HOLD_MS)
				raceboxBleEverSeen = false;
			raceboxBle = raceboxBleEverSeen;
		}
		// REC – 50ms Debounce reicht gegen Rauschen
		{
			bool isLow = digitalRead(RACEBOX_REC_PIN) == LOW;
			if (!isLow) {
				if (raceboxRecHighSinceMs == 0) raceboxRecHighSinceMs = millis();
				raceboxRecLowSinceMs = 0;
				if (!raceboxRecPinOk && (millis() - raceboxRecHighSinceMs) >= RACEBOX_PIN_VALID_HIGH_REC_MS)
					raceboxRecPinOk = true;
			} else {
				raceboxRecHighSinceMs = 0;
				if (raceboxRecPinOk) {
					if (raceboxRecLowSinceMs == 0) raceboxRecLowSinceMs = millis();
					if ((millis() - raceboxRecLowSinceMs) >= 50)
						{ raceboxRecLastActiveMs = millis(); raceboxRecEverSeen = true; }
				}
			}
			if (raceboxRecEverSeen && (millis() - raceboxRecLastActiveMs) >= RACEBOX_REC_HOLD_MS)
				raceboxRecEverSeen = false;
			raceboxRec = raceboxRecEverSeen;
		}

		// detect blitzer pulse: pin must stay LOW for BLITZER_DEBOUNCE_MS to avoid noise
		// ignore first 10s after boot to let blitzer warner initialize
		bool blitzerNow = digitalRead(BLITZER_PIN);
		if (blitzerNow == false) // pin is LOW
		{
			if (blitzerPinLast == true) // falling edge → start debounce timer
				blitzerLowSinceMs = millis();
			else if (millis() > 10000 &&
			         (millis() - blitzerLowSinceMs) >= BLITZER_DEBOUNCE_MS &&
			         blitzerActiveUntilMs <= millis()) // only trigger once per pulse
				blitzerActiveUntilMs = millis() + 5000;
		}
		blitzerPinLast = blitzerNow;

		// Blitzer-Warner heartbeat is sampled in updateAdsReadings() via AIN1

		// --- Test mode overrides ---
		#ifdef TEST_MODE
		{
			oilTempCached    = triangleWave(-5.0f,  120.0f, 30000UL);
			battVoltageCached = triangleWave(0.0f,   15.0f,  12000UL);
			outsideTemp      = triangleWave(-10.0f,  40.0f,  14000UL);
			float simLean = triangleWave(-90.0f, 90.0f, 8000UL);
			rollUi        = simLean;
			rollFiltered  = simLean;
		}
		#endif

		// settings overlay takes priority over everything except blitzer
		if (settingsOpen)
		{
			drawSettingsPage();
		}
		// blitzer overlay takes over entire display
		else if (millis() < blitzerActiveUntilMs)
		{
			bool flashOn = ((millis() / 200) % 2) == 0;
			display.clearDisplay();
			if (flashOn)
				display.fillRect(0, 0, 128, 64, SSD1306_WHITE);
			display.setTextColor(flashOn ? SSD1306_BLACK : SSD1306_WHITE);
			display.setFont(&FreeSansBold18pt7b);
			display.setCursor(4, 44);
			display.print("BLITZ!");
			display.setFont();
			display.setTextColor(SSD1306_WHITE);
			display.display();
		}
		else if (!isnan(oilTempCached) && oilTempCached >= OIL_CRITICAL_C)
		{
			bool flashOn = ((millis() / 300) % 2) == 0;
			display.clearDisplay();
			if (flashOn)
				display.fillRect(0, 0, 128, 64, SSD1306_WHITE);
			display.setTextColor(flashOn ? SSD1306_BLACK : SSD1306_WHITE);
			display.setFont(&FreeSansBold18pt7b);
			{
				const char* hotTxt = "HOT";
				int16_t tx1, ty1; uint16_t tw, th;
				display.getTextBounds(hotTxt, 0, 0, &tx1, &ty1, &tw, &th);
				display.setCursor((SCREEN_WIDTH - (int16_t)tw) / 2 - tx1, 46);
				display.print(hotTxt);
			}
			display.setFont();
			display.setTextSize(1);
			char tbuf[8];
			snprintf(tbuf, sizeof(tbuf), "%.0fC", oilTempCached);
			display.setCursor(90, 0);
			display.print(tbuf);
			display.setTextColor(SSD1306_WHITE);
			display.display();
		}
		else if (page == PAGE_OIL)
		{
			drawOilPage(oilTempCached);
		}
		else if (page == PAGE_LEAN)
		{
			drawLeanPage();
		}
		else if (page == PAGE_G)
		{
			drawGPage();
		}
		else
		{
			drawRaceBoxPage();
		}
	}

	delay(1);
	yield();
}