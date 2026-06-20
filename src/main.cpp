/*
  ESP32-S3 N16R8 Motorrad-Dash
  MCU   : ESP32-S3 (N16R8 – 16 MB Flash QIO, 8 MB PSRAM OPI)

  Hardware & GPIO assignment
  ──────────────────────────
  I2C bus (BNO085 + BH1750 + ADS1115)
    SDA  = GPIO 8
    SCL  = GPIO 9

  TFT 1.69" ST7789 (240×280, SPI2, landscape via rotation 1 → 280×240)
    DIN/MOSI = GPIO 11  (SPI2_MOSI)
    CLK/SCK  = GPIO 12  (SPI2_SCK)
    CS       = GPIO 10
    DC       = GPIO 5
    RST      = GPIO 6
    BLK      = 3.3 V (backlight always on)

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
    REC      = GPIO 2   (moved from 17 to free GPIO for CAN bus)

  RaceBox simulated button (NPN transistor, active HIGH)
    GPIO 3   (moved from 18 to free GPIO for CAN bus)

  CAN bus (TWAI – OBD2 Kühlwassertemperatur PID 0x05)
    TX = GPIO 18
    RX = GPIO 17

  OIL_PIN (legacy NTC direct-ADC fallback, not actively used)
    GPIO 1   ← oil temp is read via ADS1115 channel 0

  BLE HID Media Keyboard (NimBLE backend)
    Device name : "Moto-Dash"
    Backend     : NimBLE (-DUSE_NIMBLE) → stable MAC address, no iOS duplicate-pairing
    Sends       : KEY_MEDIA_VOLUME_UP / KEY_MEDIA_VOLUME_DOWN
    Note        : iOS blocks BLE HID when screen is locked (OS restriction)

  Pages:
  Primary group (only MAIN):
   - Double-tap (<250ms)  : enter secondary group (starts at LEAN)
   - Long press MAIN 2s   : enter VOLUME page
  Secondary group (short press cycles LEAN → G → ENGINE → RACEBOX):
   - Short press          : cycle LEAN → G → ENGINE → RACEBOX → LEAN
   - Double-tap (<250ms)  : back to primary group (MAIN)
   - Long press LEAN 2s   : enter settings (auto-exit after 5s → back to LEAN)
   - Long press ENGINE 800ms: arm/cancel 0–100 km/h sprint timer
   - Long press RACEBOX 800ms: simulate RaceBox button press
  VOLUME page:
   - Short press : Vol- (leiser), bleibt auf Seite, Timer reset
   - Long press  : Vol+ (lauter)
   - Auto-exit   : nach 5 s Inaktivität → zurück zu MAIN
   - Beim BLE-Connect (wenn AN): 16× Vol- dann N× Vol+ (non-blocking state machine)

  Boot:
   - Screen shows immediately, sensors flip from "..." to OK/FAIL as they init
   - Then waits 2 s; during wait you can hold button to calibrate (CAL icon top-right)
*/

#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include <EEPROM.h>

#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
// ST7789: map old SSD1306 color constants to 16-bit RGB565
#define SSD1306_WHITE  ST77XX_WHITE
#define SSD1306_BLACK  ST77XX_BLACK
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO08x.h>

#include <BH1750.h>
#include <Adafruit_ADS1X15.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <U8g2_for_Adafruit_GFX.h>

#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSans9pt7b.h>

#include "driver/twai.h"
#include <BleKeyboard.h>

// ---------------- CAN bus (TWAI) ----------------
#define CAN_TX_PIN GPIO_NUM_18
#define CAN_RX_PIN GPIO_NUM_17

// ---------------- I2C (ESP32-S3: SDA=8, SCL=9)
#define SDA_PIN 8
#define SCL_PIN 9

// ---------------- TFT (SPI2) ----------------
#define SCREEN_WIDTH  280   // ST7789 1.69" landscape (setRotation 1)
#define SCREEN_HEIGHT 240
// ST7789 1.69" has ~14px physical corner radius in landscape.
// All UI elements stay inside this safe zone.
#define CORNER_R      20

// ESP32-S3 SPI2 hardware pins
// Module labels: VCC GND DIN CLK CS DC RST
#define OLED_MOSI 11 // DIN  -> GPIO 11 (SPI2_MOSI)
#define OLED_CLK  12 // CLK  -> GPIO 12 (SPI2_SCK)
#define OLED_DC    5 // DC   -> GPIO 5
#define OLED_RST   6 // RST  -> GPIO 6
#define OLED_CS   10 // CS   -> GPIO 10

// ---- Physical TFT hardware driver (SPI device) ----
static Adafruit_ST7789 _tft_hw(&SPI, OLED_CS, OLED_DC, OLED_RST);

// ---- GFXcanvas16-backed display: ALL drawing goes into a RAM framebuffer.
// display() pushes the complete buffer to the TFT in one SPI burst → zero flicker.
class TFT_Display : public GFXcanvas16 {
public:
    TFT_Display() : GFXcanvas16(SCREEN_WIDTH, SCREEN_HEIGHT) {}
    void clearDisplay()            { fillScreen(0x0000); }
    void display() {
        _tft_hw.startWrite();
        _tft_hw.setAddrWindow(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT);
        _tft_hw.writePixels(getBuffer(), (uint32_t)SCREEN_WIDTH * SCREEN_HEIGHT);
        _tft_hw.endWrite();
    }
    void ssd1306_command(uint8_t c) {
        if      (c == 0xAF) _tft_hw.enableDisplay(true);
        else if (c == 0xAE) _tft_hw.enableDisplay(false);
    }
    void invertDisplay(bool inv)   { _tft_hw.invertDisplay(inv); }
};
TFT_Display display;
U8G2_FOR_ADAFRUIT_GFX u8g2fonts;

// 8×8 PROGMEM status icons (MSB = leftmost pixel per row)
// ---- 12×12 status icons (2 bytes per row, MSB left, bottom 4 bits of 2nd byte unused) ----
// Bluetooth ᛒ  — vertical spine with upper+lower diagonal branches right
static const uint8_t PROGMEM icon_bt[24] = {
    0x08,0x00, // ....#...
    0x0C,0x00, // ....##..
    0x4A,0x00, // .#..#.#.
    0x2C,0x00, // ..#.##..
    0x18,0x00, // ...##...
    0x18,0x00, // ...##...
    0x2C,0x00, // ..#.##..
    0x4A,0x00, // .#..#.#.
    0x0C,0x00, // ....##..
    0x08,0x00, // ....#...
    0x00,0x00,
    0x00,0x00,
};
// GPS location pin — solid outer ring, hollow circle, narrow stem + tip
static const uint8_t PROGMEM icon_gps[24] = {
    0x3C,0x00, // ..XXXX..  outer arc top
    0x7E,0x00, // .XXXXXX.  solid
    0x42,0x00, // .X....X.  ring (4px hollow)
    0x42,0x00, // .X....X.
    0x42,0x00, // .X....X.
    0x7E,0x00, // .XXXXXX.  close ring
    0x3C,0x00, // ..XXXX..  outer arc bottom
    0x1C,0x00, // ...XXX..  stem
    0x18,0x00, // ...XX...
    0x08,0x00, // ....X...  tip
    0x00,0x00,
    0x00,0x00,
};
// REC — solid filled circle in center
static const uint8_t PROGMEM icon_rec[24] = {
    0x00,0x00,
    0x1C,0x00, // ...###..
    0x3E,0x00, // ..#####.
    0x7F,0x00, // .#######
    0x7F,0x00,
    0x7F,0x00,
    0x7F,0x00,
    0x3E,0x00, // ..#####.
    0x1C,0x00, // ...###..
    0x00,0x00,
    0x00,0x00,
    0x00,0x00,
};
// Lightning bolt ⚡ — user pixel design
static const uint8_t PROGMEM icon_blitz[24] = {
    0x04,0x00, // .....#......
    0x0C,0x00, // ....##......
    0x1C,0x00, // ...###......
    0x3C,0x00, // ..####......
    0x7C,0x00, // .#####......
    0x0F,0x80, // ....#####...  ← kink
    0x0F,0x00, // ....####....
    0x0E,0x00, // ....###.....
    0x0C,0x00, // ....##......
    0x08,0x00, // ....#.......
    0x00,0x00,
    0x00,0x00,
};

// ---------------- BNO085 ----------------
Adafruit_BNO08x bno(-1);
bool bnoOk = false;
float bno085Roll   = 0.0f;
float bno085Pitch  = 0.0f;  // forward/backward tilt of sensor in degrees
float bno085LinX   = 0.0f;
float bno085LinY   = 0.0f;
float bno085LinZ   = 0.0f;
float bno085GyroX  = 0.0f;  // calibrated gyro in rad/s (sensor frame)
float bno085GyroY  = 0.0f;
float bno085GyroZ  = 0.0f;
float bno085Qi     = 0.0f;  // last ARVR_STABILIZED_GRV quaternion components
float bno085Qj     = 0.0f;  // needed to compute earth-frame yaw rate
float bno085Qk     = 0.0f;
float bno085Qr     = 1.0f;

// ---------------- BH1750 ----------------
BH1750 lightMeter;
bool bhOk = false;

// ---------------- ADS1115 analog expander ----------------
Adafruit_ADS1115 ads;

#define ADS_CH_OIL            0 // A0: oil temperature NTC
#define ADS_CH_BLITZER_ALIVE  1 // A1: Blitzer-Warner heartbeat LED line
#define ADS_CH_BATT           2 // A2: battery voltage divider

// Voltage threshold: idle=~2.6V, pulse=~3.5V → detect >3.0V as heartbeat
#define BLITZER_ALIVE_V_THRESHOLD 3.0f

// we will use gain = 1 (±4.096V) which gives 1 LSB = 125µV
#define ADS_GAIN GAIN_ONE
#define ADS_VREF 4.096f       // ADS1115 internal reference for GAIN_ONE (for ADC scaling)
#define ADS_SCALE (ADS_VREF / 32768.0f)
#define DIVIDER_VCC 3.3f      // actual supply voltage of the NTC voltage divider (10k to 3.3V)

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
// When active: sensors cycle through normal range, warnings NOT triggered
// Use TEST_MODE_WARNINGS to also test warning screens
// #define TEST_MODE_WARNINGS
// Main demo: fixed oil/batt/temp, lean oscillates 0→41→0 (pause 8s) 0→-51→0 (pause 8s)
// #define TEST_MODE_MAIN

// ---------------- one‑wire / outside temp ----------------
#define ONE_WIRE_PIN 7
OneWire oneWire(ONE_WIRE_PIN);
DallasTemperature dsSensors(&oneWire);

float outsideTemp = NAN;
unsigned long lastOutsideMs = 0;
const unsigned long OUTSIDE_INTERVAL_MS = 5000;
bool outsideConvRequested = false;
DeviceAddress outsideSensorAddr;
bool ds18b20Found = false;

float oilTempCached = NAN;
// alpha=0.05 → Zeitkonstante ~6s (Kanal wird alle 300ms aktualisiert: 300ms/0.05=6s)
// Öl ändert sich physikalisch in Sekunden kaum → starke Dämpfung sinnvoll
const float OIL_EMA_ALPHA = 0.05f;
float dbgOilVoltage = NAN;  // last raw ADS voltage (debug)

float coolantTempCached = NAN; // OBD2 PID 0x05 – Kühlwassertemperatur via CAN
float engineRpmCached   = NAN; // OBD2 PID 0x0C – Motordrehzahl
float engineLoadCached  = NAN; // OBD2 PID 0x04 – Motorlast %
float throttlePosCached = NAN; // OBD2 PID 0x11 – Drosselklappenstellung %
float vehicleSpeedCached = NAN; // OBD2 PID 0x0D – Geschwindigkeit km/h

// 0-100 km/h timer
enum Sprint100State : uint8_t { S100_IDLE, S100_ARMED, S100_RUNNING, S100_DONE };
Sprint100State sprint100State = S100_IDLE;
unsigned long sprint100StartMs = 0;
float sprint100Result = NAN;

// ---- set to 1 to show raw ADS voltage on oil page ----
#define OIL_DEBUG 0

// ---- set to 1 to scan and print all supported OBD2 PIDs on boot (Serial 115200) ----
#define CAN_PID_SCAN 0

// DS18B20 calibration offset (add to raw reading) – runtime, saved in EEPROM
float DS18B20_OFFSET = -1.2f;

float battVoltageCached = NAN;
const float BATT_EMA_ALPHA = 0.3f;
const float BATT_SPIKE_REJECT_V = 0.25f; // ignore single-sample spikes >0.25V
uint8_t cpuLoadPct = 0; // main-loop CPU load estimate (0–100%)

// round-robin ADS1115 scheduling: one read per loop, rotate channels
// slot 0 = oil (ADS ch0), slot 1 = Blitzer alive (ADS ch1), slot 2 = battery (ADS ch2)
unsigned long lastAdsReadMs = 0;
const unsigned long ADS_READ_INTERVAL_MS = 100;
uint8_t adsNextChannel = 0;
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
	PAGE_MAIN = 0,        // main/oil page (big number + oil bar)
	PAGE_LEAN = 1,
	PAGE_G = 2,
	PAGE_ENGINE = 3,
	PAGE_RACEBOX = 4,
	PAGE_VOLUME = 5       // BLE media volume control
};

// ---------------- BLE Keyboard (Volume control) ----------------
BleKeyboard bleKeyboard("Moto-Dash", "ESP32", 100);
unsigned long volLastInteractMs = 0;
#define VOL_PAGE_TIMEOUT_MS 5000
unsigned long volFeedbackUntilMs = 0;  // show +/- on screen until this time
bool volFeedbackUp = true;             // true=vol+, false=vol-
bool volFeedbackWasConn = false;       // was BLE connected when key was sent

// ---------------- RaceBox status inputs ----------------
#define RACEBOX_GPS_PIN 15  // HIGH when GPS fix active  (GPIO26-32 = flash on ESP32-S3, avoid)
#define RACEBOX_BLE_PIN 16  // HIGH when BLE connected
#define RACEBOX_REC_PIN 2   // HIGH when recording active (moved from 17 for CAN bus)

bool raceboxGps = false;
bool raceboxBle = false;
bool raceboxRec = false;
unsigned long raceboxRecLastActiveMs = 0;
bool raceboxRecEverSeen  = false;
unsigned long raceboxRecLowSinceMs  = 0;
#define RACEBOX_REC_HOLD_MS 3000
#define RACEBOX_BTN_PIN 3   // OUTPUT: drives NPN transistor to simulate RaceBox button press (moved from 18 for CAN bus)
unsigned long raceboxBtnUntilMs = 0;
unsigned long raceboxBtnCooldownMs = 0; // prevent re-trigger while holding
bool raceboxBtnArmed = true;            // only fire once per button press cycle

// ---------------- Blitzer Warner ----------------
#define BLITZER_PIN 14
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
Page page = PAGE_MAIN;

// Two-group navigation: primary (OIL/LEAN) and secondary (G/ENGINE/RACEBOX)
bool primaryGroup       = true;       // true = in primary group
Page lastPrimaryPage    = PAGE_MAIN; // remembered when leaving primary
Page lastSecondaryPage  = PAGE_LEAN;  // remembered when leaving secondary

#define BTN_PIN  4
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
#define MAIN_VOL_LONGPRESS_MS  2000   // hold MAIN this long → enter VOLUME page
#define LEAN_SETTINGS_OPEN_MS 2000   // hold LEAN this long → enter settings
#define SETTINGS_TIMEOUT_MS   5000   // auto-close settings after 5s inactivity → back to LEAN
#define SETTINGS_LONGPRESS_MS 600    // long press inside settings = change value
#define SLEEP_COUNTDOWN_MS    3000   // delay before display goes dark after activating sleep

enum SettingsItem : uint8_t {
	SET_BRIGHTNESS = 0,   // brightness mode: 0=Auto, 1=Tag, 2=Nacht
	SET_NIGHT_SLEEP,      // turn display off (button wakes)
	SET_LEAN_FLIP,        // invert lean angle direction
	SET_LEAN_OFFSET,      // capture current roll as lean zero offset
	SET_PITCH_OFFSET,     // sensor pitch correction for G-force
	SET_RESET_ALL,        // reset lean max L/R AND G-force peaks
	SET_DS18_OFFSET,      // DS18B20 temperature calibration offset
	SET_BATT_LOW,         // battery low warning threshold
	SET_G_DEADZONE,       // G-force vibration deadzone
	SET_COUNT
};

// Brightness mode: 0=Auto (BH1750), 1=Tag (always max), 2=Nacht (always min), 3=Sonne (forced invert)
uint8_t brightMode = 0;
bool leanFlip = false;  // true = invert roll direction (sensor mounted mirrored)
bool settingsOpen = false;
uint8_t settingsIdx = 0;
unsigned long settingsLastActMs = 0; // last interaction time (for timeout)
bool settingsLongFired = false;    // long-press-in-settings already fired
unsigned long settingsPressStartMs = 0;
bool settingsEnterReleasePending = false; // ignore first release after entering settings

bool displaySleeping = false;           // display off, wakes on button press
bool sleepCountdownActive = false;      // countdown before display turns off
unsigned long sleepCountdownStartMs = 0;

void drawSettingsPage();

// ---------------- Oil temp ----------------
// NOTE: OIL_PIN is a legacy fallback – oil temp is read via ADS1115 ch0.
// GPIO35-37 are used for PSRAM on ESP32-S3 N16R8; remaped to GPIO1 (unused).
#define OIL_PIN 1

#define R_REF 10000.0
#define R0 10000.0
#define T0C 25.0
#define BETA 3450.0
#define ADC_MAX 4095.0f
#define ADC_VREF 3.3f

const float OIL_BAR_MIN_C = 0.0f;
const float OIL_BAR_MAX_C = 135.0f;
const float OIL_GOOD_MIN = 80.0f;
const float OIL_GOOD_MAX = 100.0f;
const float OIL_CRITICAL_C = 125.0f; // above this: full-screen warning overlay

// ---------------- Lean ----------------
float rollFiltered = 0.0f;
const float ROLL_MOUNT_OFFSET_DEG = -3.0f; // fixed production mount error
float rollOffsetDeg = 0.0f; // user lean offset from settings (EEPROM), default 0
// Time constants in seconds (replaces sample-based alphas so behaviour is
// independent of BNO085 event rate / I2C load)
const float LEAN_TAU_S    = 0.06f;  // sensor filter τ  (~60 ms, same feel as old α=0.15 at 100Hz)
const float LEAN_UI_TAU_S = 0.08f;  // display filter τ  (~80 ms, same feel as old α=0.25 at 100Hz)

float rollUi = 0.0f;

const float LEAN_DEADZONE_DEG = 2.0f;

// All-time max (EEPROM)
float maxLeanSaved = 0.0f;
float maxLeanLeft  = 0.0f;
float maxLeanRight = 0.0f;

// Session-only max (reset on power-off, not persisted)
float sessionMaxLeanLeft  = 0.0f;
float sessionMaxLeanRight = 0.0f;

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

const float CENTER_PEAK_THRESHOLD = 30.0f;   // above this: show peak
const float CENTER_OPPOSITE_CANCEL = 5.0f;   // opposite-side degrees to cancel hold

// ---------------- G-Force ----------------
const float G0 = 9.80665f;
float gX = 0.0f, gY = 0.0f;           // slow-filtered (display dot)
float gXFast = 0.0f, gYFast = 0.0f;   // fast-filtered (max-G tracking)
const float G_ALPHA_DISPLAY = 0.10f;   // träge → wenig Jitter auf dem Dot
const float G_ALPHA_MAX     = 0.20f;   // schneller → echte Manöver landen im Max
float G_DEADZONE            = 0.04f;   // Motorvibrationen < 0.04g werden ignoriert – runtime, saved in EEPROM
float pitchOffsetDeg        = 0.0f;    // sensor pitch correction in degrees (saved in EEPROM)

float maxGSaved = 0.0f;
float maxGBrake = 0.0f;   // session peak braking G (positive value)
float maxGAccel = 0.0f;   // session peak acceleration G (positive value)

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
#define EE_SET_MAGIC_VAL 0x5F
#define EE_SET_DS18_OFF  10  // int8_t  tenths of °C  (-30..+30 → -3.0..+3.0°C)
#define EE_SET_BATT_LOW  11  // uint8_t tenths of V   (80..140  → 8.0..14.0V)
#define EE_SET_G_DEAD    12  // uint8_t hundredths g  (0..20    → 0..0.20g)
#define EE_SET_BRIGHT    14  // uint8_t 0=Auto 1=Tag 2=Nacht
#define EE_SET_LEAN_FLIP 13  // uint8_t 0/1
#define EE_SET_PITCH_OFF 15  // int8_t  degrees -20..+20
#define EE_SET_ROLL_OFF  16  // int8_t  degrees -20..+20
#define EE_SET_MAIN     17  // uint8_t 0/1 – Main Page 2 mode

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
void updateCan();
void scanObdPids();
void updateSprint100();

void updateCurvePeakHold(float leanAbs);
void updateLean();
static void pollBno085();
void updateGForce();
void updateNightMode();

void drawCenteredBigNumberWithDegree(int value, int16_t baselineY);
void drawCenteredBigNumber(int value, int16_t baselineY);
void drawCenteredTitleTiny(const char *text, int16_t baselineY);

void drawLeanSemiGauge(float rollDeg);

void drawHatchedRect(int x, int y, int w, int h, int spacing = 3);
void drawOilBar(float oilC, int barX = 20, int barW = 240);
void drawMainPage();
void drawBlitzerWarnerAliveIndicator();
void drawRpmRedlineBorder();
static void drawBatteryTopRight();

void drawLeanPage();

void drawGPage();
void drawEnginePage();
void drawRaceBoxPage();
void drawVolumePage();

void calibrateRollOffset();
void showReadyScreen();

// Progressive boot
static void drawSelfTestLineProgress(int y, const char *label, int8_t st);
static void renderBootProgress(int8_t stBno, int8_t stBh, int8_t stAds, int8_t stEe, bool calArmed, float prog01);
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
// Convert GAME_ROTATION_VECTOR quaternion → forward/backward tilt in degrees (X-axis rotation)
static float bno085QuatToPitch(float qi, float qj, float qk, float qr)
{
	float sinr = 2.0f * (qr * qi + qj * qk);
	float cosr = 1.0f - 2.0f * (qi * qi + qj * qj);
	return atan2f(sinr, cosr) * (180.0f / M_PI);
}

static void pollBno085()
{
	if (!bnoOk) return;
	sh2_SensorValue_t ev;
	while (bno.getSensorEvent(&ev))
	{
		if (ev.sensorId == SH2_ARVR_STABILIZED_GRV)
		{
			bno085Qi = ev.un.arvrStabilizedGRV.i;
			bno085Qj = ev.un.arvrStabilizedGRV.j;
			bno085Qk = ev.un.arvrStabilizedGRV.k;
			bno085Qr = ev.un.arvrStabilizedGRV.real;
			bno085Roll  = bno085QuatToRoll (bno085Qi, bno085Qj, bno085Qk, bno085Qr);
			bno085Pitch = bno085QuatToPitch(bno085Qi, bno085Qj, bno085Qk, bno085Qr);
		}
		else if (ev.sensorId == SH2_LINEAR_ACCELERATION)
		{
			bno085LinX = ev.un.linearAcceleration.x;
			bno085LinY = ev.un.linearAcceleration.y;
			bno085LinZ = ev.un.linearAcceleration.z;
		}
		else if (ev.sensorId == SH2_GYROSCOPE_CALIBRATED)
		{
			bno085GyroX = ev.un.gyroscope.x;
			bno085GyroY = ev.un.gyroscope.y;
			bno085GyroZ = ev.un.gyroscope.z;
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

// Smooth lean demo: 0→41→0 (6s), pause 8s, 0→−51→0 (6s), pause 8s
static float leanTestWave()
{
	const unsigned long sweepMs = 6000UL;
	const unsigned long waitMs  = 8000UL;
	const unsigned long period  = sweepMs * 2UL + waitMs * 2UL; // 28s
	unsigned long t = millis() % period;
	if (t < sweepMs)
		return 41.0f * sinf((float)t / sweepMs * (float)M_PI);
	else if (t < sweepMs + waitMs)
		return 0.0f;
	else if (t < sweepMs * 2UL + waitMs)
		return -51.0f * sinf((float)(t - sweepMs - waitMs) / sweepMs * (float)M_PI);
	else
		return 0.0f;
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
	// ST7789 has no direct contrast control – stub for API compatibility
	(void)c;
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
			uint8_t eeFlip   = EEPROM.read(EE_SET_LEAN_FLIP);
			uint8_t eeBright = EEPROM.read(EE_SET_BRIGHT);
			int8_t  eePitch  = (int8_t)EEPROM.read(EE_SET_PITCH_OFF);
			int8_t  eeRoll   = (int8_t)EEPROM.read(EE_SET_ROLL_OFF);
			brightMode     = (eeBright <= 3) ? eeBright : 0;
			leanFlip       = (eeFlip == 1);
			pitchOffsetDeg = clampf((float)eePitch, -20.0f, 20.0f);
			rollOffsetDeg  = clampf((float)eeRoll,  -20.0f, 20.0f);
			// DS18B20 offset: stored as int8 tenths of °C
			int8_t eeDs18  = (int8_t)EEPROM.read(EE_SET_DS18_OFF);
			DS18B20_OFFSET = clampf((float)eeDs18 * 0.1f, -3.0f, 3.0f);
			// Battery low: stored as uint8 tenths of V
			uint8_t eeBatt = EEPROM.read(EE_SET_BATT_LOW);
			BATT_LOW_V     = clampf((float)eeBatt * 0.1f, 8.0f, 14.0f);
			// G deadzone: stored as uint8 hundredths of g
			uint8_t eeGD   = EEPROM.read(EE_SET_G_DEAD);
			G_DEADZONE     = clampf((float)eeGD   * 0.01f, 0.0f, 0.20f);
		}
		// else: keep compile-time defaults (DS18B20_OFFSET=-1.2, BATT_LOW_V=10.5, G_DEADZONE=0.04)
	}
	else
	{
		maxLeanSaved = 0.0f;
		maxGSaved = 0.0f;
		maxGBrake = 0.0f;
		maxGAccel = 0.0f;
	}

	// clamp to sane ranges (guard against uninitialized / corrupt EEPROM)
	maxLeanSaved  = clampf(maxLeanSaved,  0.0f, 90.0f);
	maxLeanLeft   = clampf(maxLeanLeft,   0.0f, 90.0f);
	maxLeanRight  = clampf(maxLeanRight,  0.0f, 90.0f);
	maxGSaved     = clampf(maxGSaved,     0.0f, 9.99f);

	// EEPROM OK test: write/read/restore to scratch byte at addr 31 (unused end of block).
	// Using a dedicated scratch address avoids corrupting EE_MAGIC_ADDR on an unexpected
	// power-loss between the test-write and the restore-write.
	const uint8_t EE_SCRATCH_ADDR = 31;
	uint8_t old = EEPROM.read(EE_SCRATCH_ADDR);
	uint8_t test = (uint8_t)(old ^ 0x5A);

	EEPROM.write(EE_SCRATCH_ADDR, test);
	EEPROM.commit();
	delay(2);

	uint8_t rd = EEPROM.read(EE_SCRATCH_ADDR);

	EEPROM.write(EE_SCRATCH_ADDR, old);
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
	EEPROM.write(EE_SET_BRIGHT,    brightMode);
	EEPROM.write(EE_SET_LEAN_FLIP, leanFlip ? 1 : 0);
	EEPROM.write(EE_SET_PITCH_OFF, (uint8_t)(int8_t)lroundf(pitchOffsetDeg));
	EEPROM.write(EE_SET_ROLL_OFF,  (uint8_t)(int8_t)lroundf(rollOffsetDeg));
	EEPROM.write(EE_SET_DS18_OFF,  (uint8_t)(int8_t)lroundf(DS18B20_OFFSET * 10.0f));
	EEPROM.write(EE_SET_BATT_LOW,  (uint8_t)lroundf(clampf(BATT_LOW_V, 8.0f, 14.0f) * 10.0f));
	EEPROM.write(EE_SET_G_DEAD,    (uint8_t)lroundf(clampf(G_DEADZONE, 0.0f, 0.20f) * 100.0f));

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
			// Wake display if sleeping – consume this press
			if (displaySleeping)
			{
				display.ssd1306_command(0xAF); // SSD1306_DISPLAYON
				displaySleeping = false;
				sleepCountdownActive = false;
				btn.longFired = true; // suppress any page change on release
				return;
			}
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
				if (settingsEnterReleasePending)
				{
					// ignore this release – it's the release of the hold that opened settings
					settingsEnterReleasePending = false;
				}
				else if (btn.pressed && !settingsLongFired)
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
					if (page == PAGE_VOLUME)
					{
						// Short press on VOLUME = leiser, Timer reset (bleibt auf Seite)
						if (bleKeyboard.isConnected())
							bleKeyboard.write(KEY_MEDIA_VOLUME_DOWN);
						volLastInteractMs = now;
					}
					else
					{
						// Double-tap detection (250ms window) - works in both groups
						static unsigned long lastSecPressMs = 0;
						bool doubleTap = (now - lastSecPressMs) < 250;
						lastSecPressMs = now;

						if (doubleTap)
						{
							if (primaryGroup)
							{
								// Double-tap in primary → enter secondary group at LEAN
								lastPrimaryPage   = PAGE_MAIN;
								primaryGroup      = false;
								page              = PAGE_LEAN;
								lastSecondaryPage = PAGE_LEAN;
								resetAnimUntilMs  = 0;
							}
							else
							{
								// Double-tap in secondary → back to primary (MAIN)
								lastSecondaryPage = page;
								primaryGroup      = true;
								page              = PAGE_MAIN;
								resetAnimUntilMs  = 0;
							}
						}
						else
						{
							// Single press: cycle within current group
							if (primaryGroup)
							{
								// Only MAIN in primary group - nothing to cycle
							}
							else
							{
								// Cycle: LEAN → G → ENGINE → RACEBOX → LEAN
								if (page == PAGE_LEAN)         page = PAGE_G;
								else if (page == PAGE_G)       page = PAGE_ENGINE;
								else if (page == PAGE_ENGINE)  page = PAGE_RACEBOX;
								else                           page = PAGE_LEAN;
							}
							resetAnimUntilMs = 0;
						}
					}
				}

				raceboxBtnArmed = true; // re-arm after button fully released
			}
			btn.pressed = false;
		}
	}

	// ---- settings: 5s inactivity timeout → back to LEAN ----
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
					case SET_BRIGHTNESS:
						brightMode = (brightMode + 1) % 4;
						// apply immediately
						if (brightMode == 1) { oledSetContrast(CONTRAST_DAY);   display.invertDisplay(false); }
						else if (brightMode == 2) { oledSetContrast(CONTRAST_NIGHT); display.invertDisplay(false); }
						else if (brightMode == 3) { oledSetContrast(CONTRAST_DAY);   display.invertDisplay(true);  }
						else                      { display.invertDisplay(false); } // Auto
						break;
					case SET_PITCH_OFFSET:
						// capture current sensor pitch as offset
						pitchOffsetDeg = clampf(bno085Pitch, -20.0f, 20.0f);
						break;
					case SET_LEAN_FLIP:
						leanFlip = !leanFlip;
						break;
					case SET_NIGHT_SLEEP:
						// close settings and start sleep countdown
						settingsOpen = false;
						saveMaxValuesNow();
						sleepCountdownActive = true;
						sleepCountdownStartMs = now;
						return;
					case SET_LEAN_OFFSET:
						// capture roll correction ON TOP of fixed -3° mount offset
						rollOffsetDeg = clampf(bno085Roll - ROLL_MOUNT_OFFSET_DEG, -20.0f, 20.0f);
						maxDirty = true;
						break;
					case SET_RESET_ALL:
						maxLeanSaved = 0.0f;
						maxLeanLeft  = 0.0f;
						maxLeanRight = 0.0f;
						maxGBrake    = 0.0f;
						maxGAccel    = 0.0f;
						maxDirty = true;
						resetAnimUntilMs = now + 350;
						break;
					case SET_DS18_OFFSET:
						// step +0.5°C, wrap -3.0 → +3.0
						DS18B20_OFFSET += 0.5f;
						if (DS18B20_OFFSET > 3.0f + 0.01f) DS18B20_OFFSET = -3.0f;
						break;
					case SET_BATT_LOW:
						// step +0.5V, wrap 8.0 → 14.0
						BATT_LOW_V += 0.5f;
						if (BATT_LOW_V > 14.0f + 0.01f) BATT_LOW_V = 8.0f;
						break;
					case SET_G_DEADZONE:
						// step +0.01g, wrap 0.00 → 0.20
						G_DEADZONE += 0.01f;
						if (G_DEADZONE > 0.20f + 0.001f) G_DEADZONE = 0.0f;
						break;
					default: break;
				}
			}
		}
		else
		{
			// ---- normal long press ----
			// MAIN: 5s → VOLUME page
			if (page == PAGE_MAIN)
			{
				if (!btn.longFired && (now - btn.pressStartMs) >= MAIN_VOL_LONGPRESS_MS)
				{
					btn.longFired     = true;
					page              = PAGE_VOLUME;
					volLastInteractMs = now;
				}
				return;
			}

			// LEAN (secondary): 3s → settings
			if (page == PAGE_LEAN)
			{
				if (!btn.longFired && (now - btn.pressStartMs) >= LEAN_SETTINGS_OPEN_MS)
				{
					btn.longFired               = true;
					settingsOpen                = true;
					settingsIdx                 = 0;
					settingsLastActMs           = now;
					settingsLongFired           = false;
					settingsEnterReleasePending = true;
				}
				return;
			}

			if ((now - btn.pressStartMs) >= LONGPRESS_MS)
			{
				btn.longFired = true;

				if (page == PAGE_ENGINE)
				{
					if (sprint100State == S100_IDLE)
					{
						// Arm the 0-100 timer
						sprint100State = S100_ARMED;
					}
					else
					{
						// Cancel sprint timer
						sprint100State   = S100_IDLE;
						resetAnimUntilMs = 0;
					}
				}
				else if (page == PAGE_RACEBOX)
				{
					// RACEBOX: simulate RaceBox button press via NPN transistor
					if (raceboxBtnArmed)
					{
						raceboxBtnArmed   = false;
						digitalWrite(RACEBOX_BTN_PIN, HIGH);
						raceboxBtnUntilMs = now + 250;
					}
				}
				else if (page == PAGE_VOLUME)
				{
					if (bleKeyboard.isConnected())
						bleKeyboard.write(KEY_MEDIA_VOLUME_UP);
					volLastInteractMs = now;
				}
			}
		}
	}
}

// =========================================================
// Oil temp
// =========================================================

// Median-of-3: one bad I2C read out of three is automatically rejected
static int16_t medianOf3(int16_t a, int16_t b, int16_t c)
{
	if (a > b) { int16_t t = a; a = b; b = t; }
	if (b > c) { int16_t t = b; b = c; c = t; }
	if (a > b) { int16_t t = a; a = b; b = t; }
	return b;
}

float readAdsVoltage(int ch)
{
	if (!adsOk)
		return NAN;
	int16_t r0 = ads.readADC_SingleEnded(ch);
	int16_t r1 = ads.readADC_SingleEnded(ch);
	int16_t r2 = ads.readADC_SingleEnded(ch);
	int16_t raw = medianOf3(r0, r1, r2);
	if (raw < 0)
		return NAN;
	return raw * ADS_SCALE;
}

float readOilTempOnce()
{
	float v = readAdsVoltage(ADS_CH_OIL);
	dbgOilVoltage = v;  // store for debug display
	if (isnan(v))
		return NAN;
	if (v <= 0.001f)
		return NAN;
	if (v >= (DIVIDER_VCC - 0.1f))  // within 100mV of VCC = no sensor connected
		return NAN;

	// 10kΩ at top (5V→10kΩ→AIN0), NTC at bottom (AIN0→NTC→GND)
	// v = VCC * R_NTC / (R_NTC + R_REF)  →  R_NTC = R_REF * v / (VCC - v)
	float rNtc = R_REF * v / (DIVIDER_VCC - v);
	float tempK = 1.0f / ((1.0f / (T0C + 273.15f)) + (1.0f / BETA) * logf(rNtc / R0));
	float tempC = tempK - 273.15f;
	return tempC;
}

// =========================================================
// CAN bus (TWAI) – OBD2 Kühlwassertemperatur
// =========================================================
void updateCan()
{
	// Poll rates:
	//   RPM   (0x0C): 100ms / 10Hz – redline detection needs fast updates
	//   Speed (0x0D): 100ms / 10Hz – lean physics model braucht frische Geschwindigkeit
	//   Rest  (0x05 coolant, 0x04 load, 0x11 throttle): 333ms round-robin → ~1s each
	static const uint8_t pidList[] = { 0x05, 0x04, 0x11 };
	static uint8_t pidIdx = 0;
	static unsigned long lastCanRequestMs  = 0;
	static unsigned long lastRpmRequestMs  = 0;
	static unsigned long lastSpeedRequestMs = 0;
	static unsigned long lastCanRxMs = 0; // timestamp of last valid OBD2 frame
	unsigned long now = millis();

	// Staleness timeout: if no valid frame received for 3 s, reset all cached values
	// to NAN so the lean drift-cancel physics model doesn't use stale speed data.
	if (lastCanRxMs != 0 && (now - lastCanRxMs) > 3000)
	{
		coolantTempCached  = NAN;
		engineRpmCached    = NAN;
		engineLoadCached   = NAN;
		throttlePosCached  = NAN;
		vehicleSpeedCached = NAN;
		lastCanRxMs = 0; // prevent repeated clears
	}

	// Fast RPM poll: 100ms = 10Hz so brief redline spikes are reliably caught
	if (now - lastRpmRequestMs >= 100)
	{
		lastRpmRequestMs = now;
		twai_message_t req;
		req.identifier        = 0x18DB33F1;
		req.extd              = 1;
		req.rtr               = 0;
		req.data_length_code  = 8;
		req.data[0] = 0x02;
		req.data[1] = 0x01;
		req.data[2] = 0x0C; // RPM
		req.data[3] = req.data[4] = req.data[5] = req.data[6] = req.data[7] = 0x00;
		twai_transmit(&req, pdMS_TO_TICKS(10));
	}

	// Speed poll: 100ms = 10Hz, gleich schnell wie RPM
	if (now - lastSpeedRequestMs >= 100)
	{
		lastSpeedRequestMs = now;
		twai_message_t req;
		req.identifier        = 0x18DB33F1;
		req.extd              = 1;
		req.rtr               = 0;
		req.data_length_code  = 8;
		req.data[0] = 0x02;
		req.data[1] = 0x01;
		req.data[2] = 0x0D; // Speed
		req.data[3] = req.data[4] = req.data[5] = req.data[6] = req.data[7] = 0x00;
		twai_transmit(&req, pdMS_TO_TICKS(10));
	}

	// Slow round-robin for coolant / load / throttle at 333ms each (~1s cycle)
	if (now - lastCanRequestMs >= 333)
	{
		lastCanRequestMs = now;
		twai_message_t req;
		req.identifier        = 0x18DB33F1;
		req.extd              = 1;
		req.rtr               = 0;
		req.data_length_code  = 8;
		req.data[0] = 0x02;
		req.data[1] = 0x01;
		req.data[2] = pidList[pidIdx];
		req.data[3] = req.data[4] = req.data[5] = req.data[6] = req.data[7] = 0x00;
		twai_transmit(&req, pdMS_TO_TICKS(10));
		pidIdx = (pidIdx + 1) % 3;
	}

	// non-blocking receive – drain all queued frames each call
	twai_message_t resp;
	while (twai_receive(&resp, 0) == ESP_OK)
	{
		if (!resp.extd || resp.data_length_code < 4 || resp.data[1] != 0x41) continue;
		lastCanRxMs = now; // refresh staleness timer on any valid frame
		switch (resp.data[2])
		{
			case 0x05: coolantTempCached  = (float)(resp.data[3]) - 40.0f; break;
			case 0x0C: engineRpmCached    = (float)(((uint16_t)resp.data[3] << 8) | resp.data[4]) / 4.0f; break;
			case 0x04: engineLoadCached   = (float)(resp.data[3]) * 100.0f / 255.0f; break;
			case 0x11: throttlePosCached  = (float)(resp.data[3]) * 100.0f / 255.0f; break;
			case 0x0D: vehicleSpeedCached = (float)(resp.data[3]); break;
		}
	}
}

// =========================================================
// OBD2 PID-Support-Scan (einmalig beim Boot, Serial output)
// =========================================================
void scanObdPids()
{
#if CAN_PID_SCAN
	// Supported PIDs query groups: 0x00 (PIDs 01-20), 0x20 (21-40), 0x40 (41-60)
	const uint8_t groups[] = {0x00, 0x20, 0x40};
	Serial.println("=== OBD2 PID Scan ===");

	for (uint8_t gi = 0; gi < 3; gi++)
	{
		uint8_t groupPid = groups[gi];

		// send request
		twai_message_t req;
		req.identifier       = 0x18DB33F1;
		req.extd             = 1;
		req.rtr              = 0;
		req.data_length_code = 8;
		req.data[0] = 0x02;
		req.data[1] = 0x01;
		req.data[2] = groupPid;
		req.data[3] = req.data[4] = req.data[5] = req.data[6] = req.data[7] = 0x00;
		twai_transmit(&req, pdMS_TO_TICKS(50));

		// wait up to 500 ms for response
		twai_message_t resp;
		bool got = false;
		unsigned long t0 = millis();
		while (millis() - t0 < 500)
		{
			if (twai_receive(&resp, pdMS_TO_TICKS(10)) == ESP_OK)
			{
				if (resp.extd && resp.data_length_code >= 6 &&
				    resp.data[1] == 0x41 && resp.data[2] == groupPid)
				{
					got = true;
					break;
				}
			}
		}

		if (!got)
		{
			Serial.print("Gruppe 0x");
			Serial.print(groupPid, HEX);
			Serial.println(": keine Antwort");
			continue;
		}

		// decode bitmask: bytes A=data[3], B=data[4], C=data[5], D=data[6]
		// bit 7 of A = PID groupPid+1, ... bit 0 of D = PID groupPid+32
		uint32_t bitmask = ((uint32_t)resp.data[3] << 24) |
		                   ((uint32_t)resp.data[4] << 16) |
		                   ((uint32_t)resp.data[5] <<  8) |
		                    (uint32_t)resp.data[6];

		Serial.print("Gruppe 0x");
		Serial.print(groupPid, HEX);
		Serial.print(" → unterstützte PIDs: ");
		bool any = false;
		for (uint8_t bit = 0; bit < 32; bit++)
		{
			if (bitmask & (0x80000000UL >> bit))
			{
				Serial.print("0x");
				uint8_t pid = groupPid + bit + 1;
				if (pid < 0x10) Serial.print('0');
				Serial.print(pid, HEX);
				Serial.print(' ');
				any = true;
			}
		}
		if (!any) Serial.print("(keine)");
		Serial.println();

		delay(200); // kurz warten bevor nächste Gruppe
	}
	Serial.println("=== Scan fertig ===");
#endif
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
			outsideTemp = roundf((t + DS18B20_OFFSET) * 2.0f) / 2.0f;
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
	static unsigned long firstNanMs = 0;
	float t = readOilTempOnce();
	if (isnan(t))
	{
		// clear cache after 3s of consecutive NAN readings → shows "--" again
		if (firstNanMs == 0) firstNanMs = millis();
		if (millis() - firstNanMs > 3000) oilTempCached = NAN;
		return;
	}
	firstNanMs = 0;  // reset timer on valid reading
	if (isnan(oilTempCached))
	{
		// warm-up: average 8 quick samples to avoid ADS1115 settling error
		float sum = t;
		int count = 1;
		for (int i = 0; i < 7; i++)
		{
			float s = readOilTempOnce();
			if (!isnan(s)) { sum += s; count++; }
		}
		oilTempCached = sum / count;
	}
	else
		oilTempCached += OIL_EMA_ALPHA * (t - oilTempCached);
	// note: rounding happens at display time only (drawOilPage uses (int)round)
}

float readBatteryVoltage()
{
	float v = readAdsVoltage(ADS_CH_BATT);
	if (isnan(v))
		return NAN;
	// correct divider + calibration
	float batt = v * (BATT_R_TOP + BATT_R_BOT) / BATT_R_BOT * BATT_CAL;
	if (batt < 5.0f)   // below 5V → floating/disconnected, treat as no source
		return NAN;
	return batt;
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

	// During BLITZ the OLED draws full-white → 3.3V rail dips → NTC divider reads wrong.
	// Freeze oil + battery updates for the duration; channel 1 (blitzer alive) still runs.
	bool blitzerDisplayActive = (now < blitzerActiveUntilMs);

	if (adsNextChannel == 0)
	{
		if (!blitzerDisplayActive) updateOilTemp();
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
		static unsigned long battNanSinceMs = 0;
		float v = readBatteryVoltage();
		if (!isnan(v))
		{
			battNanSinceMs = 0;
			if (isnan(battVoltageCached))
				battVoltageCached = v;
			else if (!blitzerDisplayActive && fabsf(v - battVoltageCached) < BATT_SPIKE_REJECT_V)
				battVoltageCached += BATT_EMA_ALPHA * (v - battVoltageCached);
		}
		else
		{
			// source disconnected/floating → clear after 3s of NAN readings
			if (battNanSinceMs == 0) battNanSinceMs = now;
			if (now - battNanSinceMs > 3000) battVoltageCached = NAN;
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
	{
		// Only re-enable peak display if the decline timer hasn't already expired.
		// Without this guard, returning above 30° after 4+ seconds at lower angle
		// would re-activate cornerAboveThreshold and show the old peak (e.g. 26°)
		// instead of the current live lean angle.
		bool alreadyExpired = cornerDeclineStartMs != 0 &&
		                      (now - cornerDeclineStartMs >= HOLD_AFTER_CORNER_MS);
		if (!alreadyExpired)
			cornerAboveThreshold = true;
	}

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

	// declineExpired: display already switched to live → prevent hold from firing on corner exit
	if (cornerAboveThreshold && cornerDeclineStartMs != 0 &&
	    (now - cornerDeclineStartMs >= HOLD_AFTER_CORNER_MS))
	{
		cornerAboveThreshold = false;
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

	// ── OEM-style drift correction ────────────────────────────────────────────────
	// Bosch/Continental moto-IMUs solve the centripetal-drift problem with a
	// three-mode gravity-reference correction:
	//
	//  MODE 1 – SPEED < 10 km/h (stopped / traffic light)
	//    Bike must be upright. Correct aggressively (τ = 1 s).
	//
	//  MODE 2 – GOING STRAIGHT (|ωbody| < 0.08 rad/s)
	//    Centripetal force ≈ 0, gravity reference is reliable. Correct τ = 4 s.
	//
	//  MODE 3 – CORNERING + OBD2 SPEED AVAILABLE  ← the OEM trick
	//    Steady-state lean satisfies:  θ_expected = atan(v · ψ̇_earth / g)
	//    where ψ̇_earth = earth-frame yaw rate (from quaternion × body-gyro).
	//    We know the true lean from physics → update driftCancel continuously (τ = 3 s)
	//    so drift never has a chance to accumulate, even through repeated S-bends.
	//
	//  MODE 4 – CORNERING, NO SPEED (OBD2 not available)
	//    No physics reference → freeze driftCancel (same as before).
	//
	// Earth-frame yaw rate: project body angular velocity onto world Z-axis.
	//   ω_z_earth = R[2][0]*ωx + R[2][1]*ωy + R[2][2]*ωz
	//   R from Hamilton quaternion q=(Qr,Qi,Qj,Qk):
	//     R[2][0] = 2(Qi·Qk - Qr·Qj)
	//     R[2][1] = 2(Qj·Qk + Qr·Qi)
	//     R[2][2] = 1 - 2(Qi² + Qj²)
	static float driftCancel = 0.0f;
	static bool  initDone    = false;
	static unsigned long lastLeanMs = 0;

	unsigned long leanNowMs = millis();
	float leanDt = (lastLeanMs == 0) ? 0.0f : (float)(leanNowMs - lastLeanMs) * 0.001f;
	if (leanDt > 0.5f) leanDt = 0.0f;
	lastLeanMs = leanNowMs;

	// Subtract zero-offsets, then correct for sensor pitch mount offset.
	// A pitch-mounted sensor reports atan(sin(φ)/(cos(φ)·cos(p))) instead of φ.
	// Inverse: φ = atan(tan(measured) · cos(p))  — error is ~1.5° at 30°, ~1.8° at 45°.
	float rawRollDeg  = bno085Roll - ROLL_MOUNT_OFFSET_DEG - rollOffsetDeg;
	float rawRoll;
	if (pitchOffsetDeg != 0.0f)
	{
		float rrRad = rawRollDeg * (float)(M_PI / 180.0);
		float pRad  = pitchOffsetDeg * (float)(M_PI / 180.0);
		rawRoll = normalizeAngleDeg(atanf(tanf(rrRad) * cosf(pRad)) * (float)(180.0 / M_PI));
	}
	else
	{
		rawRoll = normalizeAngleDeg(rawRollDeg);
	}
	if (!initDone)
	{
		driftCancel  = rawRoll;
		rollFiltered = 0.0f;
		rollUi       = 0.0f;
		initDone = true;
	}

	if (leanDt > 0.0f)
	{
		// Earth-frame yaw rate (valid at any lean angle, no mounting-orientation assumption)
		float yawRateEarth =
			 2.0f*(bno085Qi*bno085Qk - bno085Qr*bno085Qj)*bno085GyroX
			+2.0f*(bno085Qj*bno085Qk + bno085Qr*bno085Qi)*bno085GyroY
			+(1.0f - 2.0f*(bno085Qi*bno085Qi + bno085Qj*bno085Qj))*bno085GyroZ;

		float omegaSq = bno085GyroX*bno085GyroX
		              + bno085GyroY*bno085GyroY
		              + bno085GyroZ*bno085GyroZ;

		// Vibration filter: EMA-smooth the gyro magnitude so that high-frequency
		// frame vibrations (< ~100 ms bursts) don't falsely flag "cornering".
		// τ = 0.25 s → a real corner onset (~0.5 s) passes through cleanly.
		static float omegaSqFiltered = 0.0f;
		const float OMEGA_VIB_TAU = 0.25f;
		float omegaAlpha = clampf(leanDt / OMEGA_VIB_TAU, 0.0f, 1.0f);
		omegaSqFiltered += omegaAlpha * (omegaSq - omegaSqFiltered);

		const float OMEGA_CORNER  = 0.08f; // rad/s threshold: below = going straight
		bool isCornering   = (omegaSqFiltered > OMEGA_CORNER * OMEGA_CORNER);
		bool hasSpeed      = (!isnan(vehicleSpeedCached) && vehicleSpeedCached >= 10.0f);
		bool canAlive      = !isnan(vehicleSpeedCached); // CAN läuft = Speed kommt an

		if (!isCornering && canAlive && vehicleSpeedCached > 0.5f)
		{
			// MODE 2: Geradeausfahrt, CAN bestätigt Bewegung – τ = 2.5 s
			// (verkürzt von 4 s: schnellere Nachkorrektur nach Vibrations-bedingtem Drift)
			// Stillstand (Speed=0) oder kein CAN → einfrieren
			driftCancel += (leanDt / 2.5f) * (rawRoll - driftCancel);
		}
		else if (hasSpeed)
		{
			// MODE 3: OEM physics model
			// Expected lean from vehicle dynamics: θ = atan(v · ψ̇_earth / g)
			float v_m_s = vehicleSpeedCached / 3.6f;
			float leanExpected_deg = atan2f(v_m_s * yawRateEarth, G0) * (180.0f / M_PI);
			// Solve for what driftCancel must be so that (rawRoll - driftCancel) = leanExpected
			float target = rawRoll - leanExpected_deg;
			driftCancel += (leanDt / 3.0f) * (target - driftCancel);
		}
		// MODE 4: cornering + no speed → freeze (safe fallback)
	}

	float roll = (rawRoll - driftCancel) * (leanFlip ? -1.0f : 1.0f);

	// Time-based EMA: α = dt/τ so filter speed is independent of loop/event rate
	float leanAlpha   = clampf(leanDt / LEAN_TAU_S,    0.0f, 1.0f);
	float leanUiAlpha = clampf(leanDt / LEAN_UI_TAU_S, 0.0f, 1.0f);
	rollFiltered += leanAlpha * (roll - rollFiltered);

	// Soft deadzone: instead of a hard snap to 0, smoothly suppress small angles
	// using a cubic curve:  output = input * (|input|/deadzone)^2  inside the zone.
	// Outside the zone the value passes through unchanged.
	float target;
	if (fabsf(rollFiltered) < LEAN_DEADZONE_DEG)
	{
		float t = rollFiltered / LEAN_DEADZONE_DEG;  // -1..+1
		target = rollFiltered * (t * t);             // cubic → smooth 0 near centre
	}
	else
	{
		target = rollFiltered;
	}

	rollUi += leanUiAlpha * (target - rollUi);

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

	// Session-only tracking (not persisted)
	if (rollFiltered < -LEAN_DEADZONE_DEG && leanAbs > sessionMaxLeanLeft)
		sessionMaxLeanLeft = leanAbs;
	if (rollFiltered > LEAN_DEADZONE_DEG && leanAbs > sessionMaxLeanRight)
		sessionMaxLeanRight = leanAbs;

	updateCurvePeakHold(leanAbs);
}

void updateGForce()
{
	if (!bnoOk)
		return;

	float gxNow = bno085LinX / G0;
	float gyNow = bno085LinY / G0;

	// Pitch correction: rotate sensor Y/Z to vehicle frame if sensor is mounted tilted
	if (pitchOffsetDeg != 0.0f)
	{
		float p = pitchOffsetDeg * (float)(M_PI / 180.0);
		gyNow = (bno085LinY * cosf(p) + bno085LinZ * sinf(p)) / G0;
	}
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
	// Brake / accel peak (longitudinal axis only)
	if (gYFast < 0.0f && -gYFast > maxGBrake) maxGBrake = -gYFast;
	if (gYFast > 0.0f &&  gYFast > maxGAccel) maxGAccel =  gYFast;
}

void updateSprint100()
{
	if (sprint100State == S100_RUNNING)
	{
		if (!isnan(vehicleSpeedCached) && vehicleSpeedCached >= 100.0f)
		{
			sprint100Result = (float)(millis() - sprint100StartMs) / 1000.0f;
			sprint100State  = S100_DONE;
		}
	}
	else if (sprint100State == S100_ARMED)
	{
		// auto-start as soon as speed leaves 0
		if (!isnan(vehicleSpeedCached) && vehicleSpeedCached > 3.0f)
		{
			sprint100StartMs = millis();
			sprint100State   = S100_RUNNING;
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
	if (brightMode == 3)
	{
		// Sonne: forced invert, max contrast – nothing to auto-adjust
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

	// Sonne mode (brightMode==3) inverts the display; this is handled in the
	// settings handler when the mode is changed. Auto mode only adjusts contrast.
}

// =========================================================
// UI helpers
// =========================================================
void drawCenteredBigNumberWithDegree(int value, int16_t baselineY)
{
	char s[8];
	snprintf(s, sizeof(s), "%d", value);

	// Bigger display-optimized numeric font
	u8g2fonts.setFont(u8g2_font_logisoso58_tn);
	int16_t w = (int16_t)u8g2fonts.getUTF8Width(s);
	int16_t x = (display.width() - w) / 2;
	u8g2fonts.setCursor(x, baselineY);
	u8g2fonts.print(s);

	// degree circle – scaled for larger font
	int16_t topY   = baselineY - 58;
	int16_t rightX = x + w;

	const int r = 6;
	int16_t cx = rightX + r + 5;
	int16_t cy = topY  + r - 2;

	cx = constrain(cx, CORNER_R + r, 279 - r);
	cy = constrain(cy, CORNER_R + r, 239 - r);

	display.drawCircle(cx, cy, r, SSD1306_WHITE);
}

void drawCenteredBigNumber(int value, int16_t baselineY)
{
	char s[8];
	snprintf(s, sizeof(s), "%d", value);
	u8g2fonts.setFont(u8g2_font_logisoso58_tn);
	int16_t w = (int16_t)u8g2fonts.getUTF8Width(s);
	int16_t x = (display.width() - w) / 2;
	u8g2fonts.setCursor(x, baselineY);
	u8g2fonts.print(s);
}

void drawCenteredTitleTiny(const char *text, int16_t baselineY)
{
	u8g2fonts.setFont(u8g2_font_logisoso16_tf);
	int16_t tw = (int16_t)u8g2fonts.getUTF8Width(text);
	u8g2fonts.setCursor((SCREEN_WIDTH - tw) / 2, baselineY);
	u8g2fonts.print(text);
}

static int16_t uiTextWidth16(const char *text)
{
	u8g2fonts.setFont(u8g2_font_logisoso16_tf);
	return (int16_t)u8g2fonts.getUTF8Width(text);
}

static void uiText16(int16_t x, int16_t baselineY, const char *text)
{
	u8g2fonts.setFont(u8g2_font_logisoso16_tf);
	u8g2fonts.setCursor(x, baselineY);
	u8g2fonts.print(text);
}

static int16_t uiTextWidth24(const char *text)
{
	u8g2fonts.setFont(u8g2_font_logisoso24_tf);
	return (int16_t)u8g2fonts.getUTF8Width(text);
}

static void uiText24(int16_t x, int16_t baselineY, const char *text)
{
	u8g2fonts.setFont(u8g2_font_logisoso24_tf);
	u8g2fonts.setCursor(x, baselineY);
	u8g2fonts.print(text);
}

static void uiTextCenter16(int16_t x, int16_t w, int16_t baselineY, const char *text)
{
	int16_t tw = uiTextWidth16(text);
	uiText16(x + (w - tw) / 2, baselineY, text);
}

static void uiTextCenter24(int16_t x, int16_t w, int16_t baselineY, const char *text)
{
	int16_t tw = uiTextWidth24(text);
	uiText24(x + (w - tw) / 2, baselineY, text);
}

// =========================================================
// Lean Semi Gauge (>=40° => 3px thick)
// =========================================================
void drawLeanSemiGauge(float rollDeg)
{
	const int16_t cx = 139;  // exact center for 0..279 pixel range
	const int16_t cy = 239;  // bottom edge
	const int16_t r  = 95;   // radius

	const float maxDeg = 60.0f;
	if (fabs(rollDeg) < LEAN_DEADZONE_DEG)
		rollDeg = 0.0f;
	rollDeg = clampf(rollDeg, -maxDeg, +maxDeg);

	// Draw as connected 1-degree polylines for a clean, continuous arc edge.
	auto drawArcStroke = [&](int16_t rr, int aStart, int aEnd) {
		float rad0 = aStart * (float)(M_PI / 180.0f);
		int16_t px = cx + (int16_t)roundf(cosf(rad0) * rr);
		int16_t py = cy - (int16_t)roundf(sinf(rad0) * rr);
		for (int a = aStart + 1; a <= aEnd; ++a)
		{
			float rad = a * (float)(M_PI / 180.0f);
			int16_t x = cx + (int16_t)roundf(cosf(rad) * rr);
			int16_t y = cy - (int16_t)roundf(sinf(rad) * rr);
			display.drawLine(px, py, x, y, SSD1306_WHITE);
			px = x;
			py = y;
		}
	};

	// Base semicircle.
	drawArcStroke(r, 0, 180);
	// Danger zones (outer left/right 20deg each) thicker.
	drawArcStroke(r - 1, 0, 30);
	drawArcStroke(r - 1, 150, 180);
	drawArcStroke(r - 2, 0, 30);
	drawArcStroke(r - 2, 150, 180);

	// 0 marker
	{
		int16_t tx = cx;
		int16_t ty = cy - r;
		display.fillTriangle(tx, ty - 4, tx - 6, ty + 5, tx + 6, ty + 5, SSD1306_WHITE);
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
		float aPrev = a0;
		if (a1 < a0)
		{
			for (float a = a0 - step; a >= a1; a -= step)
			{
				float r1 = aPrev * (float)(M_PI / 180.0f);
				float r2 = a * (float)(M_PI / 180.0f);

				int16_t xA = cx + (int16_t)roundf(cosf(r1) * (r - 3));
				int16_t yA = cy - (int16_t)roundf(sinf(r1) * (r - 3));
				int16_t xB = cx + (int16_t)roundf(cosf(r2) * (r - 3));
				int16_t yB = cy - (int16_t)roundf(sinf(r2) * (r - 3));
				display.fillTriangle(cx, cy, xA, yA, xB, yB, SSD1306_WHITE);
				aPrev = a;
			}
		}
		else
		{
			for (float a = a0 + step; a <= a1; a += step)
			{
				float r1 = aPrev * (float)(M_PI / 180.0f);
				float r2 = a * (float)(M_PI / 180.0f);

				int16_t xA = cx + (int16_t)roundf(cosf(r1) * (r - 3));
				int16_t yA = cy - (int16_t)roundf(sinf(r1) * (r - 3));
				int16_t xB = cx + (int16_t)roundf(cosf(r2) * (r - 3));
				int16_t yB = cy - (int16_t)roundf(sinf(r2) * (r - 3));
				display.fillTriangle(cx, cy, xA, yA, xB, yB, SSD1306_WHITE);
				aPrev = a;
			}
		}

		// Ensure the very last sliver reaches the exact target angle.
		if (aPrev != a1)
		{
			float r1 = aPrev * (float)(M_PI / 180.0f);
			float r2 = a1 * (float)(M_PI / 180.0f);
			int16_t xA = cx + (int16_t)roundf(cosf(r1) * (r - 3));
			int16_t yA = cy - (int16_t)roundf(sinf(r1) * (r - 3));
			int16_t xB = cx + (int16_t)roundf(cosf(r2) * (r - 3));
			int16_t yB = cy - (int16_t)roundf(sinf(r2) * (r - 3));
			display.fillTriangle(cx, cy, xA, yA, xB, yB, SSD1306_WHITE);
		}

		float needleAng = a1 * (float)(M_PI / 180.0);
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

// Redline border flash at >= 9500 rpm – drawn on every page before display.display()
void drawRpmRedlineBorder()
{
	if (!isnan(engineRpmCached) && engineRpmCached >= 9500.0f)
	{
		if (((millis() / 80) % 2) == 0)
		{
			// Rounded border matches physical display corners
			display.drawRoundRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, CORNER_R, SSD1306_WHITE);
			display.drawRoundRect(2, 2, SCREEN_WIDTH - 4, SCREEN_HEIGHT - 4, CORNER_R - 2, SSD1306_WHITE);
		}
	}
}

// Small heartbeat indicator: filled circle = alive, blinking = offline, nothing = initialising
// Drawn at top-left corner (radius 2, center (2,4)) – 4px wide, sits just left of outside-temp text.
void drawBlitzerWarnerAliveIndicator()
{
	bool alive = blitzerAliveReceived &&
	             (millis() - blitzerAliveLastMs) < BLITZER_ALIVE_TIMEOUT_MS;
	if (alive)
		return; // läuft normal → kein Indikator

	// Warner offline oder noch nie verbunden → blinkende kleine Striche links und rechts
	if (((millis() / 400) % 2) == 0)
	{
		display.fillRect(0,   97, 6, 45, SSD1306_WHITE);
		display.fillRect(274, 97, 6, 45, SSD1306_WHITE);
	}
}

void drawOilBar(float oilC, int barX, int barW)
{
	const int x = barX;
	const int y = 214;
	const int w = barW;
	const int h = 18;

	// Flat-UI palette (RGB565)
	//  cold  #3498DB steel-blue  |  good #2ECC71 emerald
	//  hot   #E67E22 orange      |  crit #E74C3C flat-red
	static constexpr uint16_t COL_COLD   = 0x34DB;
	static constexpr uint16_t COL_GOOD   = 0x2E6E;
	static constexpr uint16_t COL_ORANGE = 0xE3E4;
	static constexpr uint16_t COL_RED    = 0xE267;

	bool valid = !isnan(oilC);
	uint16_t fillColor = COL_COLD;
	if (valid)
	{
		if (oilC >= OIL_CRITICAL_C)
			fillColor = COL_RED;
		else if (oilC > OIL_GOOD_MAX)
		{
			// smooth lerp orange → red
			float t = clampf((oilC - OIL_GOOD_MAX) / (OIL_CRITICAL_C - OIL_GOOD_MAX), 0.0f, 1.0f);
			uint8_t r5 = 28;
			uint8_t g6 = (uint8_t)(31.0f - t * 12.0f);
			uint8_t b5 = (uint8_t)(4.0f  + t * 3.0f);
			fillColor = (uint16_t)((r5 << 11) | (g6 << 5) | b5);
		}
		else if (oilC >= OIL_GOOD_MIN)
			fillColor = COL_GOOD;
	}

	// dim-grey border so fill colour pops
	display.drawRect(x, y, w, h, 0x7BEF);

	int gx1 = x + 1 + mapf_to_i(OIL_GOOD_MIN, OIL_BAR_MIN_C, OIL_BAR_MAX_C, 0, w - 3);
	int gx2 = x + 1 + mapf_to_i(OIL_GOOD_MAX, OIL_BAR_MIN_C, OIL_BAR_MAX_C, 0, w - 3);
	if (gx2 < gx1) { int tmp = gx1; gx1 = gx2; gx2 = tmp; }

	if (valid)
	{
		float t = clampf(oilC, OIL_BAR_MIN_C, OIL_BAR_MAX_C);
		int fillW = mapf_to_i(t, OIL_BAR_MIN_C, OIL_BAR_MAX_C, 0, w - 2);
		if (fillW > 0)
			display.fillRect(x + 1, y + 1, fillW, h - 2, fillColor);
	}

	// zone tick marks in neutral colour (clean look)
	display.drawFastVLine(gx1, y - 6, h + 12, 0xBDF7);
	display.drawFastVLine(gx2, y - 6, h + 12, 0xBDF7);

	// critical: flash red double-border
	if (valid && oilC >= OIL_CRITICAL_C)
	{
		if (((millis() / 140) % 2) == 0)
		{
			display.drawRect(x,     y,     w,     h,     COL_RED);
			display.drawRect(x - 1, y - 1, w + 2, h + 2, COL_RED);
		}
	}
}

// fixed margin from screen edge used for both outside temp and battery
#define SIDE_MARGIN   (CORNER_R + 2)
#define BOTTOM_SAFE_Y (SCREEN_HEIGHT - CORNER_R + 2)

// Snowflake warning: drawn near outside temp when <= 0°C (ice risk)
static void drawSnowflakeWarning(int16_t cx, int16_t cy)
{
	// 3 lines through center (horizontal, vertical, diagonal)
	const int r = 8;  // doubled for new screen
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
	if (isnan(batt) || batt < 1.0f)
		return;
	bool lowBatt = (batt < BATT_LOW_V);
	if (lowBatt && ((millis() / 400) % 2) == 0)
		return; // blink off every other 400ms interval
	char num[8];
	snprintf(num, sizeof(num), "%.1f", batt);
	u8g2fonts.setFont(u8g2_font_logisoso24_tn);
	int16_t numW = (int16_t)u8g2fonts.getUTF8Width(num);
	int16_t x = SCREEN_WIDTH - numW - SIDE_MARGIN - 10; // leave room for unit
	int16_t baseline = 28;
	u8g2fonts.setCursor(x, baseline);
	u8g2fonts.print(num);
	uiText16(x + numW + 2, 17, "V");
}

// =========================================================
// Main Page 2  (oil page; center number auto-switches to lean at >= 25°)
// =========================================================
void drawMainPage()
{
	static bool mainLeanActive   = false;
	static unsigned long mainLeanBelowMs = 0;
	static float mainLeanPeak    = 0.0f;  // highest lean reached since threshold crossed

	const float MAIN_LEAN_THRESHOLD = 25.0f;
	const unsigned long MAIN_LEAN_HOLD_MS = 4000;

	float leanAbs = fabsf(rollFiltered); // use unsmoothed value so peak is never missed

	if (leanAbs >= MAIN_LEAN_THRESHOLD)
	{
		mainLeanActive  = true;
		mainLeanBelowMs = 0; // reset hold timer while still above threshold
		if (leanAbs > mainLeanPeak)
			mainLeanPeak = leanAbs;
	}
	else if (mainLeanActive)
	{
		if (mainLeanBelowMs == 0)
			mainLeanBelowMs = millis();
		if ((millis() - mainLeanBelowMs) >= MAIN_LEAN_HOLD_MS)
		{
			mainLeanActive  = false;
			mainLeanBelowMs = 0;
			mainLeanPeak    = 0.0f;
		}
	}
	else
	{
		mainLeanPeak = 0.0f;
	}

	float oilC = oilTempCached;

	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);

	// outside temperature at fixed left margin, same vertical as battery
	if (!isnan(outsideTemp))
	{
		char otBuf[8];
		snprintf(otBuf, sizeof(otBuf), "%.1f", outsideTemp);
		u8g2fonts.setFont(u8g2_font_logisoso24_tn);
		int16_t otW = (int16_t)u8g2fonts.getUTF8Width(otBuf);
		int16_t otX = SIDE_MARGIN;
		int16_t otBaseline = 28;
		u8g2fonts.setCursor(otX, otBaseline);
		u8g2fonts.print(otBuf);
		int16_t otDegX = otX + otW;
		display.drawCircle(otDegX + 4, 9, 2, SSD1306_WHITE);
		if (outsideTemp <= 0.0f)
			drawSnowflakeWarning(otDegX + 18, 12);
	}

	// coolant temperature (OBD2 PID 0x05) – bottom right, above oil bar
	if (!isnan(coolantTempCached))
	{
		char cwBuf[8];
		snprintf(cwBuf, sizeof(cwBuf), "%d", (int)round(coolantTempCached));
		u8g2fonts.setFont(u8g2_font_logisoso24_tn);
		int16_t cw = (int16_t)u8g2fonts.getUTF8Width(cwBuf);
		int16_t x = SCREEN_WIDTH - cw - 18 - SIDE_MARGIN;
		int16_t baseline = 199;
		u8g2fonts.setCursor(x, baseline);
		u8g2fonts.print(cwBuf);
		display.drawCircle(x + cw + 4, baseline - 18, 2, SSD1306_WHITE);
	}

	int16_t baselineY = 154; // vertically centered for 58px number font
	if (mainLeanActive)
	{
		// Always show peak (never live) – only goes up, held 4s after returning below 30°
		drawCenteredBigNumber((int)round(mainLeanPeak), baselineY);
	}
	else if (isnan(oilC))
	{
		u8g2fonts.setFont(u8g2_font_logisoso58_tn);
		int16_t w = (int16_t)u8g2fonts.getUTF8Width("--");
		u8g2fonts.setCursor((SCREEN_WIDTH - w) / 2, baselineY);
		u8g2fonts.print("--");
	}
	else
	{
		static int shownOilInt2 = INT_MIN;
		if (shownOilInt2 == INT_MIN)
			shownOilInt2 = (int)round(oilC);
		else if (oilC > (float)shownOilInt2 + 0.6f)
			shownOilInt2++;
		else if (oilC < (float)shownOilInt2 - 0.6f)
			shownOilInt2--;
		drawCenteredBigNumberWithDegree(shownOilInt2, baselineY);

		if (oilC < 60.0f)
			uiText16(6, 190, "COLD");
	}

	drawBatteryTopRight();

	drawOilBar(oilC);

	drawBlitzerWarnerAliveIndicator();

	// lean-active border: rounded to match display corners
	if (mainLeanActive)
	{
		display.drawRoundRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, CORNER_R, SSD1306_WHITE);
		display.drawRoundRect(2, 2, SCREEN_WIDTH - 4, SCREEN_HEIGHT - 4, CORNER_R - 2, SSD1306_WHITE);
	}

	// hold-progress bar – CORNER_R inset to clear rounded corners
	if (btn.pressed && !mainLeanActive)
	{
		unsigned long held = millis() - btn.pressStartMs;
		int maxW = SCREEN_WIDTH - 2 * CORNER_R;
		int barW = (int)((float)held / (float)MAIN_VOL_LONGPRESS_MS * (float)maxW);
		if (barW > maxW) barW = maxW;
		if (barW > 0)
			display.fillRect(CORNER_R, SCREEN_HEIGHT - 4, barW, 3, SSD1306_WHITE);
	}

	// CAN offline indicator – unter Batterie wenn sichtbar, sonst an deren Stelle
	if (isnan(vehicleSpeedCached))
	{
		bool battVisible = !isnan(battVoltageCached) && battVoltageCached >= 1.0f &&
		                   !(battVoltageCached < BATT_LOW_V && ((millis() / 400) % 2) == 0);
		const int16_t bx = SCREEN_WIDTH - SIDE_MARGIN - 46;
		const int16_t by = battVisible ? 28 : 4;
		display.drawRect(bx, by, 46, 20, SSD1306_WHITE);
		uiText16(bx + 2, by + 16, "CAN");
	}

	drawRpmRedlineBorder();
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

	drawCenteredBigNumber((int)round(centerValue), 154); // vertically centered

	// Max lean left/right: same numeric font family as main value
	char lBuf[8], rBuf[8];
	snprintf(lBuf, sizeof(lBuf), "%d", (int)round(maxLeanLeft));
	snprintf(rBuf, sizeof(rBuf), "%d", (int)round(maxLeanRight));
	u8g2fonts.setFont(u8g2_font_logisoso24_tn);
	int16_t lW = (int16_t)u8g2fonts.getUTF8Width(lBuf);
	int16_t rW = (int16_t)u8g2fonts.getUTF8Width(rBuf);
	(void)lW;

	uiText16(SIDE_MARGIN - 6, BOTTOM_SAFE_Y, "L:");
	u8g2fonts.setCursor(SIDE_MARGIN + 16, BOTTOM_SAFE_Y);
	u8g2fonts.print(lBuf);

	int16_t rx = SCREEN_WIDTH - SIDE_MARGIN - rW;
	uiText16(rx - 20, BOTTOM_SAFE_Y, "R:");
	u8g2fonts.setCursor(rx, BOTTOM_SAFE_Y);
	u8g2fonts.print(rBuf);

	drawLeanSemiGauge(rollUi);

	// reset flash – centred, scaled for 280×240
	unsigned long now = millis();
	if (now < resetAnimUntilMs)
	{
		bool on = ((now / 80) % 2) == 0;
		if (on)
		{
			display.fillRect(79, 38, 122, 45, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			u8g2fonts.setForegroundColor(SSD1306_BLACK);
			uiTextCenter24(79, 122, 67, "RESET");
			u8g2fonts.setForegroundColor(SSD1306_WHITE);
			display.setTextColor(SSD1306_WHITE);
		}
	}

	drawBlitzerWarnerAliveIndicator();
	drawRpmRedlineBorder();

	// hold-progress bar – CORNER_R inset so it clears the rounded corners
	if (btn.pressed)
	{
		unsigned long held = millis() - btn.pressStartMs;
		int maxW = SCREEN_WIDTH - 2 * CORNER_R;
		int barW = (int)((float)held / (float)LEAN_SETTINGS_OPEN_MS * (float)maxW);
		if (barW > maxW) barW = maxW;
		if (barW > 0)
			display.fillRect(CORNER_R, SCREEN_HEIGHT - 4, barW, 3, SSD1306_WHITE);
	}

	// CAN offline indicator
	if (isnan(vehicleSpeedCached))
	{
		const int16_t bx = SCREEN_WIDTH - CORNER_R - 50;
		const int16_t by = CORNER_R;
		display.drawRect(bx, by, 46, 20, SSD1306_WHITE);
		uiText16(bx + 2, by + 16, "CAN");
	}

	display.display();
}

// =========================================================
// G page
// =========================================================
void drawGPage()
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);

	// ---- Header labels ----
	uiText16(4, 20, "< BREMSEN");
	uiText16(196, 20, "GAS >");

	// ---- Large current G value ----
	char gBuf[8];
	snprintf(gBuf, sizeof(gBuf), "%.2f", fabsf(gY));
	u8g2fonts.setFont(u8g2_font_logisoso32_tn);
	int16_t gW = (int16_t)u8g2fonts.getUTF8Width(gBuf);
	u8g2fonts.setCursor((SCREEN_WIDTH - gW) / 2, 138);
	u8g2fonts.print(gBuf);

	// ---- Horizontal bar (y=158, h=30) ----
	// range ±1.5g, centre at x=140, usable half-width = 127px
	const int16_t BAR_Y  = 158;
	const int16_t BAR_H  = 30;
	const int16_t BAR_CX = 140;
	const float   BAR_G  = 1.5f;
	const int16_t BAR_HW = 127; // half-width in pixels
	display.drawRect(BAR_CX - BAR_HW, BAR_Y, BAR_HW * 2, BAR_H, SSD1306_WHITE);
	display.drawFastVLine(BAR_CX, BAR_Y, BAR_H, SSD1306_WHITE); // centre tick
	// fill (gY positiv = Gas, gY negativ = Bremsen)
	if (fabsf(gY) > 0.01f)
	{
		int16_t fillW = (int16_t)((fabsf(gY) / BAR_G) * (float)BAR_HW);
		if (fillW > BAR_HW) fillW = BAR_HW;
		if (fillW > 0)
		{
			if (gY < 0.0f) // Bremsen: rechts
				display.fillRect(BAR_CX + 1,     BAR_Y + 1, fillW, BAR_H - 2, SSD1306_WHITE);
			else            // Gas: links
				display.fillRect(BAR_CX - fillW, BAR_Y + 1, fillW, BAR_H - 2, SSD1306_WHITE);
		}
	}

	// ---- Peak tick marks on the bar ----
	if (maxGBrake > 0.01f)
	{
		int16_t bx = BAR_CX + (int16_t)((maxGBrake / BAR_G) * (float)BAR_HW);
		if (bx > BAR_CX + BAR_HW - 1) bx = BAR_CX + BAR_HW - 1;
		display.drawFastVLine(bx, BAR_Y - 5, BAR_H + 10, SSD1306_WHITE);
	}
	if (maxGAccel > 0.01f)
	{
		int16_t ax = BAR_CX - (int16_t)((maxGAccel / BAR_G) * (float)BAR_HW);
		if (ax < BAR_CX - BAR_HW + 1) ax = BAR_CX - BAR_HW + 1;
		display.drawFastVLine(ax, BAR_Y - 5, BAR_H + 10, SSD1306_WHITE);
	}

	// ---- Peak values ----
	char pbuf[10];
	snprintf(pbuf, sizeof(pbuf), "M:%.2fg", maxGBrake);
	uiText16(4, 216, pbuf);
	snprintf(pbuf, sizeof(pbuf), "M:%.2fg", maxGAccel);
	uiText16(SCREEN_WIDTH - uiTextWidth16(pbuf) - 4, 216, pbuf);

	// ---- reset flash ----
	unsigned long now = millis();
	if (now < resetAnimUntilMs)
	{
		bool on = ((now / 80) % 2) == 0;
		if (on)
		{
			display.fillRect(79, 38, 122, 45, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			u8g2fonts.setForegroundColor(SSD1306_BLACK);
			uiTextCenter24(79, 122, 67, "RESET");
			u8g2fonts.setForegroundColor(SSD1306_WHITE);
			display.setTextColor(SSD1306_WHITE);
		}
	}

	drawBlitzerWarnerAliveIndicator();
	drawRpmRedlineBorder();
	display.display();
}

// =========================================================
// Engine page
// =========================================================
void drawEnginePage()
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);

	// ---- km/h – big number top center ----
	{
		char kmhBuf[8];
		if (!isnan(vehicleSpeedCached))
			snprintf(kmhBuf, sizeof(kmhBuf), "%d", (int)round(vehicleSpeedCached));
		else
			snprintf(kmhBuf, sizeof(kmhBuf), "--");
		u8g2fonts.setFont(u8g2_font_logisoso42_tn);
		int16_t kmhW = (int16_t)u8g2fonts.getUTF8Width(kmhBuf);
		u8g2fonts.setCursor((SCREEN_WIDTH - kmhW) / 2, 120);
		u8g2fonts.print(kmhBuf);
		uiText16((SCREEN_WIDTH + kmhW) / 2 + 8, 92, "km/h");
	}

	// ---- Load % – bottom left ----
	{
		char num[8];
		if (!isnan(engineLoadCached))
			snprintf(num, sizeof(num), "%d", (int)round(engineLoadCached));
		else
			snprintf(num, sizeof(num), "--");
		uiText16(2, 146, "Ld:");
		u8g2fonts.setFont(u8g2_font_logisoso24_tn);
		u8g2fonts.setCursor(36, 150);
		u8g2fonts.print(num);
		uiText16(36 + (int16_t)u8g2fonts.getUTF8Width(num) + 2, 146, "%");
	}

	// ---- RPM – bottom right ----
	{
		char num[8];
		if (!isnan(engineRpmCached))
			snprintf(num, sizeof(num), "%d", (int)round(engineRpmCached));
		else
			snprintf(num, sizeof(num), "--");
		u8g2fonts.setFont(u8g2_font_logisoso24_tn);
		int16_t nw = (int16_t)u8g2fonts.getUTF8Width(num);
		int16_t nx = SCREEN_WIDTH - nw - 28;
		u8g2fonts.setCursor(nx, 150);
		u8g2fonts.print(num);
		uiText16(nx + nw + 2, 146, "rpm");
	}

	// ---- Throttle bar ----
	{
		const int bx = 0, by = 156, bw = 280, bh = 30;
		display.drawRect(bx, by, bw, bh, SSD1306_WHITE);
		if (!isnan(throttlePosCached))
		{
			int fillW = (int)(throttlePosCached / 100.0f * (bw - 2));
			if (fillW > 0)
				display.fillRect(bx + 1, by + 1, fillW, bh - 2, SSD1306_WHITE);
		}
		// percentage centered in bar
		if (!isnan(throttlePosCached))
		{
			char tbuf[6];
			snprintf(tbuf, sizeof(tbuf), "%d%%", (int)round(throttlePosCached));
			u8g2fonts.setFont(u8g2_font_logisoso16_tf);
			int16_t w = (int16_t)u8g2fonts.getUTF8Width(tbuf);
			bool fillHigh = (!isnan(throttlePosCached) && throttlePosCached >= 50.0f);
			u8g2fonts.setForegroundColor(fillHigh ? SSD1306_BLACK : SSD1306_WHITE);
			u8g2fonts.setCursor((SCREEN_WIDTH - w) / 2, by + 22);
			u8g2fonts.print(tbuf);
			u8g2fonts.setForegroundColor(SSD1306_WHITE);
		}
	}

	// ---- 0-100 timer – bottom row ----
	{
		switch (sprint100State)
		{
			case S100_IDLE:
				uiText16(66, 228, "0-100: hold to arm");
				break;
			case S100_ARMED:
				uiText16(66, 228, "0-100: READY - gas!");
				break;
			case S100_RUNNING:
			{
				float elapsed = (float)(millis() - sprint100StartMs) / 1000.0f;
				char tbuf[16];
				snprintf(tbuf, sizeof(tbuf), "0-100: %.1fs...", elapsed);
				uiText16(0, 228, tbuf);
				break;
			}
			case S100_DONE:
			{
				char tbuf[16];
				snprintf(tbuf, sizeof(tbuf), "0-100: %.2fs", sprint100Result);
				uiTextCenter16(0, SCREEN_WIDTH, 228, tbuf);
				break;
			}
		}
	}

	drawBlitzerWarnerAliveIndicator();
	drawRpmRedlineBorder();
	display.display();
}

void drawRaceBoxPage()
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);

	// Hilfsfunktion: Text horizontal im Badge zentrieren
	auto printCentered = [&](int bx, int bw, int baseline, const char* text) {
		uiTextCenter16(bx, bw, baseline, text);
	};

	// Titel zentriert
	uiTextCenter16(0, SCREEN_WIDTH, 18, "RaceBox");
	display.drawLine(0, 22, SCREEN_WIDTH - 1, 22, SSD1306_WHITE);

	const int BX = 109, BW = 70; // Badge x und Breite

	bool bleAlive = raceboxBleAliveLastMs > 0 && (millis() - raceboxBleAliveLastMs) < RACEBOX_BLE_ALIVE_TIMEOUT_MS;
	bool raceboxOn = raceboxBle || bleAlive;

	// BLT-Status Hilfslambda (gemeinsam für beide Zustände)
	auto drawBlt = [&](int iconY, int textY, int badgeY, int badgeFillY) {
		display.drawBitmap(0, iconY, icon_blitz, 12, 12, SSD1306_WHITE);
		uiText16(16, textY + 14, "BLT");
		bool alive = blitzerAliveReceived && (millis() - blitzerAliveLastMs) < BLITZER_ALIVE_TIMEOUT_MS;
		if (blitzerAliveReceived && alive) {
			display.fillRoundRect(BX, badgeY, BW, 20, 4, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			u8g2fonts.setForegroundColor(SSD1306_BLACK);
			printCentered(BX, BW, badgeFillY, "ON");
			u8g2fonts.setForegroundColor(SSD1306_WHITE);
			display.setTextColor(SSD1306_WHITE);
		} else if (blitzerAliveReceived && !alive) {
			if (((millis() / 400) % 2) == 0)
				display.drawRoundRect(BX, badgeY, BW, 20, 4, SSD1306_WHITE);
			printCentered(BX, BW, badgeFillY, "OFF");
		} else {
			display.drawRoundRect(BX, badgeY, BW, 20, 4, SSD1306_WHITE);
			printCentered(BX, BW, badgeFillY, "OFF");
		}
	};

	if (!raceboxOn) {
		// RaceBox ist aus: RaceBox + Co-Driver Status
		const int OBX = 197, OBW = 70;

		// Zeile 1: RaceBox
		uiText16(8, 74, "RaceBox");
		display.drawRoundRect(OBX, 56, OBW, 20, 4, SSD1306_WHITE);
		printCentered(OBX, OBW, 59, "OFF");

		// Zeile 2: Co-Driver
		uiText16(8, 124, "Co-Driver");
		{
			bool alive = blitzerAliveReceived && (millis() - blitzerAliveLastMs) < BLITZER_ALIVE_TIMEOUT_MS;
			if (blitzerAliveReceived && alive) {
				display.fillRoundRect(OBX, 106, OBW, 20, 4, SSD1306_WHITE);
				display.setTextColor(SSD1306_BLACK);
				u8g2fonts.setForegroundColor(SSD1306_BLACK);
				printCentered(OBX, OBW, 109, "ON");
				u8g2fonts.setForegroundColor(SSD1306_WHITE);
				display.setTextColor(SSD1306_WHITE);
			} else if (blitzerAliveReceived && !alive) {
				if (((millis() / 400) % 2) == 0)
					display.drawRoundRect(OBX, 106, OBW, 20, 4, SSD1306_WHITE);
				printCentered(OBX, OBW, 109, "OFF");
			} else {
				display.drawRoundRect(OBX, 106, OBW, 20, 4, SSD1306_WHITE);
				printCentered(OBX, OBW, 109, "OFF");
			}
		}

	} else {
		// RaceBox ist an: BLE, REC, GPS und BLT anzeigen

		// BLE  (row 1)
		display.drawBitmap(2, 30, icon_bt, 12, 12, SSD1306_WHITE);
		uiText16(18, 44, "BLE");
		if (raceboxBle) {
			display.fillRoundRect(BX, 28, BW, 18, 4, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			u8g2fonts.setForegroundColor(SSD1306_BLACK);
			printCentered(BX, BW, 30, "CONN");
			u8g2fonts.setForegroundColor(SSD1306_WHITE);
			display.setTextColor(SSD1306_WHITE);
		} else {
			display.drawRoundRect(BX, 28, BW, 18, 4, SSD1306_WHITE);
			printCentered(BX, BW, 30, "ON");
		}

		// REC  (row 2)
		display.drawBitmap(2, 60, icon_rec, 12, 12, SSD1306_WHITE);
		uiText16(18, 74, "REC");
		if (raceboxRec) {
			display.fillRoundRect(BX, 58, BW, 18, 4, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			u8g2fonts.setForegroundColor(SSD1306_BLACK);
			printCentered(BX, BW, 60, "REC");
			u8g2fonts.setForegroundColor(SSD1306_WHITE);
			display.setTextColor(SSD1306_WHITE);
		} else {
			display.drawRoundRect(BX, 58, BW, 18, 4, SSD1306_WHITE);
			printCentered(BX, BW, 60, "---");
		}

		// GPS  (row 3)
		display.drawBitmap(2, 90, icon_gps, 12, 12, SSD1306_WHITE);
		uiText16(18, 104, "GPS");
		if (raceboxGps) {
			display.fillRoundRect(BX, 88, BW, 18, 4, SSD1306_WHITE);
			display.setTextColor(SSD1306_BLACK);
			u8g2fonts.setForegroundColor(SSD1306_BLACK);
			printCentered(BX, BW, 90, "FIX");
			u8g2fonts.setForegroundColor(SSD1306_WHITE);
			display.setTextColor(SSD1306_WHITE);
		} else {
			display.drawRoundRect(BX, 88, BW, 18, 4, SSD1306_WHITE);
			printCentered(BX, BW, 90, "---");
		}

		// BLT  (row 4)
		drawBlt(120, 120, 118, 120);
	}

	// Schaltflächen-Indikator oben rechts
	if (raceboxBtnUntilMs > 0 && millis() < raceboxBtnUntilMs) {
		display.fillRoundRect(SCREEN_WIDTH - 80, 2, 72, 18, 4, SSD1306_WHITE);
		display.setTextColor(SSD1306_BLACK);
		u8g2fonts.setForegroundColor(SSD1306_BLACK);
		printCentered(SCREEN_WIDTH - 80, 72, 4, "REC");
		u8g2fonts.setForegroundColor(SSD1306_WHITE);
		display.setTextColor(SSD1306_WHITE);
	}

	drawBlitzerWarnerAliveIndicator();
	drawRpmRedlineBorder();
	display.display();
}

// =========================================================
// Volume control page (BLE HID media keys)
// =========================================================
void drawVolumePage()
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);

	// Title centred
	uiTextCenter16(0, SCREEN_WIDTH, 18, "VOLUME");
	display.drawLine(0, 24, SCREEN_WIDTH - 1, 24, SSD1306_WHITE);

	// BT icon + connection status
	display.drawBitmap(2, 30, icon_bt, 12, 12, SSD1306_WHITE);
	bool btConn = bleKeyboard.isConnected();
	if (btConn)
	{
		uiText16(20, 44, "Verbunden");
	}
	else
	{
		if (((millis() / 500) % 2) == 0)
			uiText16(20, 44, "Suche...");
	}

	// Vol+ row
	uiText24(4, 84, "+");
	uiText16(40, 87, "Lauter  (halten)");

	// Vol- row
	uiText24(8, 134, "-");
	uiText16(40, 137, "Leiser  (kurz)");

	// Auto-close countdown bar at bottom
	unsigned long elapsed = millis() - volLastInteractMs;
	float frac = 1.0f - clampf((float)elapsed / (float)VOL_PAGE_TIMEOUT_MS, 0.0f, 1.0f);
	int barW = (int)(258.0f * frac);
	display.drawRect(11, 214, 258, 10, SSD1306_WHITE);
	if (barW > 0)
		display.fillRect(11, 214, barW, 10, SSD1306_WHITE);

	drawRpmRedlineBorder();
	display.display();
}

// =========================================================
// Fonts showcase page
// =========================================================
// =========================================================
// Settings UI helpers
// =========================================================
void drawSettingsPage()
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);

	// Title bar
	display.fillRect(0, 0, 280, 22, SSD1306_WHITE);
	display.setTextColor(SSD1306_BLACK);
	u8g2fonts.setForegroundColor(SSD1306_BLACK);
	// Left: ESP temperature
	char hdrBuf[8];
	snprintf(hdrBuf, sizeof(hdrBuf), "%.0fC", (double)temperatureRead());
	uiText16(4, 18, hdrBuf);
	// Centre: title
	uiTextCenter16(0, SCREEN_WIDTH, 18, "SETTINGS");
	// Right: RAM used%
	uint8_t ramUsedPct = (uint8_t)((ESP.getHeapSize() - ESP.getFreeHeap()) * 100 / ESP.getHeapSize());
	snprintf(hdrBuf, sizeof(hdrBuf), "%u%%", ramUsedPct);
	int16_t rx = 280 - uiTextWidth16(hdrBuf) - 4;
	uiText16(rx, 18, hdrBuf);
	u8g2fonts.setForegroundColor(SSD1306_WHITE);
	display.setTextColor(SSD1306_WHITE);

	// Items – 20px per row, 5 visible at a time
	const int16_t ITEM_H      = 20;
	const int16_t VISIBLE     = 5;
	const int16_t LIST_TOP    = 24;

	const char* labels[SET_COUNT] = {
		"Brightness",
		"Sleep Mode",
		"Lean Flip",
		"Lean Offset",
		"Pitch Offset",
		"Reset All",
		"Temp Offset",
		"Batt Warnung",
		"G Deadzone"
	};

	// Scroll offset: keep selected item in view
	int8_t scrollTop = (int8_t)settingsIdx - (VISIBLE - 1);
	if (scrollTop < 0) scrollTop = 0;
	if (scrollTop > (int8_t)(SET_COUNT - VISIBLE)) scrollTop = (int8_t)(SET_COUNT - VISIBLE);

	char valBuf[14];
	for (int8_t i = scrollTop; i < scrollTop + VISIBLE && i < (int8_t)SET_COUNT; i++)
	{
		int16_t y = LIST_TOP + (i - scrollTop) * ITEM_H;

		if (i == (int8_t)settingsIdx)
			display.fillRect(0, y, 280, ITEM_H - 1, SSD1306_WHITE);

		display.setTextColor(i == (int8_t)settingsIdx ? SSD1306_BLACK : SSD1306_WHITE);
		u8g2fonts.setForegroundColor(i == (int8_t)settingsIdx ? SSD1306_BLACK : SSD1306_WHITE);
		uiText16(4, y + 16, labels[i]);

		switch (i)
		{
			case SET_BRIGHTNESS:
			{
				const char* bNames[] = { "Auto", "Tag", "Nacht", "Sonne" };
				snprintf(valBuf, sizeof(valBuf), "%s", bNames[brightMode]);
				break;
			}
			case SET_PITCH_OFFSET:
				snprintf(valBuf, sizeof(valBuf), "%+.0f Grad", pitchOffsetDeg);
				break;
			case SET_LEAN_FLIP:
				snprintf(valBuf, sizeof(valBuf), "%s", leanFlip ? "AN" : "AUS");
				break;
			case SET_NIGHT_SLEEP:
				snprintf(valBuf, sizeof(valBuf), "HOLD");
				break;
			case SET_LEAN_OFFSET:
				snprintf(valBuf, sizeof(valBuf), "%+.0f Grad", rollOffsetDeg);
				break;
			case SET_RESET_ALL:
				snprintf(valBuf, sizeof(valBuf), "HOLD");
				break;
			case SET_DS18_OFFSET:
				snprintf(valBuf, sizeof(valBuf), "%+.1f C", DS18B20_OFFSET);
				break;
			case SET_BATT_LOW:
				snprintf(valBuf, sizeof(valBuf), "%.1f V", BATT_LOW_V);
				break;
			case SET_G_DEADZONE:
				snprintf(valBuf, sizeof(valBuf), "%.2f g", G_DEADZONE);
				break;
			default:
				valBuf[0] = 0;
				break;
		}

		int16_t vx = 280 - uiTextWidth16(valBuf) - 4;
		uiText16(vx, y + 16, valBuf);
	}
	u8g2fonts.setForegroundColor(SSD1306_WHITE);
	display.setTextColor(SSD1306_WHITE);

	// hold-progress bar at bottom while pressing
	unsigned long now = millis();
	if (btn.pressed)
	{
		unsigned long held = now - settingsPressStartMs;
		int barW = (int)((float)held / (float)SETTINGS_LONGPRESS_MS * 280.0f);
		if (barW > 280) barW = 280;
		if (barW > 0)
			display.fillRect(0, 236, barW, 4, SSD1306_WHITE);
	}

	// timeout countdown bar (bottom line shrinks as timeout approaches)
	{
		unsigned long elapsed = now - settingsLastActMs;
		int barW = 280 - (int)((float)elapsed / (float)SETTINGS_TIMEOUT_MS * 280.0f);
		if (barW < 0) barW = 0;
		display.drawFastHLine(280 - barW, 237, barW, SSD1306_WHITE);
	}

	display.display();
}

// =========================================================
void calibrateRollOffset()
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);
	uiText16(20, 94, "Calibrating...");
	uiText16(20, 124, "Hold still");
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
	uiText16(20, 114, "Offset set");
	display.display();
	delay(250);
}

// =========================================================
// Progressive Boot UI
// =========================================================
// status: -1 = pending, 0 = fail, 1 = ok
static void drawSelfTestLineProgress(int y, const char *label, int8_t st)
{
	// label links
	uiText16(18, y + 14, label);

	// status immer bei fester X-Position
	const int16_t statusX = 131;

	if (st < 0)
	{
		uiText16(statusX, y + 14, "...");
		return;
	}

	if (st > 0)
	{
		uiText16(statusX, y + 14, "OK");
	}
	else
	{
		bool on = ((millis() / 160) % 2) == 0;
		if (on)
			display.fillRect(statusX - 2, y - 2, 60, 20, ST77XX_RED);
		else
			display.drawRect(statusX - 2, y - 2, 60, 20, ST77XX_RED);
		display.setTextColor(on ? SSD1306_WHITE : ST77XX_RED);
		u8g2fonts.setForegroundColor(on ? SSD1306_WHITE : ST77XX_RED);
		uiText16(statusX, y + 14, "FAIL");
		u8g2fonts.setForegroundColor(SSD1306_WHITE);
		display.setTextColor(SSD1306_WHITE);
	}
}

static void renderBootProgress(int8_t stBno, int8_t stBh, int8_t stAds, int8_t stEe, bool calArmed, float prog01)
{
	display.clearDisplay();
	display.setTextColor(SSD1306_WHITE);
	uiText16(18, 34, "Self-Test");

	drawSelfTestLineProgress(55,  "BNO085",  stBno);
	drawSelfTestLineProgress(80,  "BH1750",  stBh);
	drawSelfTestLineProgress(105, "ADS1115", stAds);
	drawSelfTestLineProgress(130, "EEPROM",  stEe);

	const int barX = 18, barY = 214, barW = 245, barH = 12;
	display.drawRect(barX, barY, barW, barH, SSD1306_WHITE);
	int fill = (int)((barW - 2) * clampf(prog01, 0.0f, 1.0f));
	if (fill > 0)
		display.fillRect(barX + 1, barY + 1, fill, barH - 2, SSD1306_WHITE);

	display.display();
}

void bootProgressInitAndMaybeCalibrate()
{
	int8_t stBno = -1, stBh = -1, stAds = -1, stEe = -1;

	// show instantly
	renderBootProgress(stBno, stBh, stAds, stEe, false, 0.0f);

	eepromOk = loadMaxValues();
	stEe = eepromOk ? 1 : 0;
	renderBootProgress(stBno, stBh, stAds, stEe, false, 0.15f);

	bhOk = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
	stBh = bhOk ? 1 : 0;
	renderBootProgress(stBno, stBh, stAds, stEe, false, 0.3f);

	adsOk = ads.begin();
	stAds = adsOk ? 1 : 0;
	if (adsOk) {
		ads.setGain(ADS_GAIN);
		delay(10);  // let ADS settle after gain change
		// oilTempCached stays NAN → updateOilTemp() handles 8-sample warm-up on first call
	}
	renderBootProgress(stBno, stBh, stAds, stEe, false, 0.45f);

	bnoOk = bno.begin_I2C();
	if (bnoOk)
	{
		// ARVR_STABILIZED_GRV: gyro+accel fusion, NO magnetometer, stabilized for sustained tilt
		// → reduces (but doesn't fully eliminate) centripetal-acceleration drift during cornering
		bno.enableReport(SH2_ARVR_STABILIZED_GRV,      10000);  // 10ms = 100Hz
		bno.enableReport(SH2_LINEAR_ACCELERATION,       10000);  // 10ms = 100Hz
		// Calibrated gyro: used in software to detect active cornering (non-zero angular rate)
		// so we only apply drift correction when the bike is actually going straight.
		bno.enableReport(SH2_GYROSCOPE_CALIBRATED,     10000);  // 10ms = 100Hz
		delay(100);
	}
	stBno = bnoOk ? 1 : 0;
	renderBootProgress(stBno, stBh, stAds, stEe, false, 0.6f);

	// kick off DS18B20 conversion now — will be ready after 2s wait
	if (ds18b20Found)
	{
		dsSensors.requestTemperaturesByAddress(outsideSensorAddr);
	}

	// wait 2s — bar fills from 0.6 → 1.0
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
				calArmed = true;
		}
		else
		{
			holdStart = 0;
		}

		updateAdsReadings();  // keep pumping ADS during splash so oil temp is ready
		renderBootProgress(stBno, stBh, stAds, stEe, calArmed, prog);
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
			outsideTemp = roundf((t + DS18B20_OFFSET) * 2.0f) / 2.0f;
		outsideConvRequested = false;
		lastOutsideMs = millis();
	}
}

void showReadyScreen()
{
	const char *txt = "READY";
	u8g2fonts.setFont(u8g2_font_logisoso42_tn);
	int16_t tw = (int16_t)u8g2fonts.getUTF8Width(txt);
	int16_t cx = (SCREEN_WIDTH - tw) / 2;
	int16_t baseline = (SCREEN_HEIGHT / 2) + 14;

	// left-to-right reveal: draw full text, mask right portion with black rect shrinking each frame
	const unsigned long dur = 380;
	unsigned long t0 = millis();

	while (true)
	{
		unsigned long elapsed = millis() - t0;
		float prog = clampf((float)elapsed / (float)dur, 0.0f, 1.0f);
		int revealX = (int)(prog * (float)(cx + tw)); // right edge of revealed area

		display.clearDisplay();
		display.setTextColor(SSD1306_WHITE);
		u8g2fonts.setFont(u8g2_font_logisoso42_tn);
		u8g2fonts.setCursor(cx, baseline);
		u8g2fonts.print(txt);

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

	neopixelWrite(48, 0, 0, 0); // board RGB LED off

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

	// CAN bus (TWAI) – OBD2 Kühlwassertemperatur
	{
		twai_general_config_t g_cfg = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
		twai_timing_config_t  t_cfg = TWAI_TIMING_CONFIG_500KBITS();
		twai_filter_config_t  f_cfg = TWAI_FILTER_CONFIG_ACCEPT_ALL();
		twai_driver_install(&g_cfg, &t_cfg, &f_cfg);
		twai_start();
	}
#if CAN_PID_SCAN
	delay(500); // kurz warten bis CAN-Bus stabil ist
	scanObdPids();
#endif

	// configure hardware SPI pins before display.begin
	// if you're using the default VSPI pins, SPI.begin() with no arguments is fine
	// otherwise specify sck, miso, mosi, ss
	SPI.begin(OLED_CLK, /*MISO*/ -1, OLED_MOSI, OLED_CS);

	// Initialise the physical ST7789 hardware
	_tft_hw.init(240, 280);         // physical pixels: 240 × 280
	_tft_hw.setRotation(1);         // landscape: 280 × 240
	_tft_hw.setSPISpeed(40000000);  // 40 MHz – fast enough for 40+ fps frame push
	_tft_hw.fillScreen(ST77XX_BLACK);

	// Initialise the canvas (framebuffer)
	display.setTextColor(ST77XX_WHITE);
	display.clearDisplay();
	u8g2fonts.begin(display);
	u8g2fonts.setForegroundColor(ST77XX_WHITE);
	u8g2fonts.setBackgroundColor(ST77XX_BLACK);
	u8g2fonts.setFontMode(1); // transparent glyph background

	// oledSetContrast is a no-op for ST7789

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

	// BLE HID keyboard for media volume control (NimBLE backend = stable address)
	bleKeyboard.begin();
}

void loop()
{
	// CPU-Auslastung messen: Dauer dieser Iteration vs. Interval seit letztem Aufruf
	static uint64_t prevStartUs  = 0;
	static uint64_t prevDurUs    = 0;
	uint64_t loopStartUs = (uint64_t)esp_timer_get_time();
	if (prevStartUs > 0)
	{
		uint64_t interval = loopStartUs - prevStartUs;
		if (interval > 0)
			cpuLoadPct = (uint8_t)min((uint64_t)99, prevDurUs * 100 / interval);
	}
	prevStartUs = loopStartUs;

	buttonUpdate();

	// PAGE_VOLUME: auto-exit to PAGE_MAIN after 5s of no interaction
	if (page == PAGE_VOLUME && volLastInteractMs > 0 &&
	    (millis() - volLastInteractMs) >= VOL_PAGE_TIMEOUT_MS)
	{
		primaryGroup      = true;
		page              = PAGE_MAIN;
		volLastInteractMs = 0;
	}

	updateLean();
	updateGForce();
	updateNightMode();
	updateOutsideTemp();
	updateAdsReadings();  // one ADS read per loop: oil or battery alternating
	updateCan();          // CAN OBD2 request/receive for coolant temperature
	updateSprint100();    // 0-100 km/h timer state machine

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
		#ifdef TEST_MODE_WARNINGS
		{
			oilTempCached      = triangleWave(-30.0f, 130.0f, 30000UL);
			battVoltageCached  = triangleWave(0.0f,   15.0f,  12000UL);
			outsideTemp        = triangleWave(-10.0f,  40.0f,  14000UL);
			coolantTempCached  = triangleWave(20.0f,  125.0f, 25000UL);
			vehicleSpeedCached = triangleWave(0.0f,   120.0f, 20000UL);
			engineRpmCached    = triangleWave(800.0f, 12000.0f, 15000UL);
			engineLoadCached   = triangleWave(0.0f,   100.0f, 18000UL);
			throttlePosCached  = triangleWave(0.0f,   100.0f, 12000UL);
			float simLean = triangleWave(-90.0f, 90.0f, 8000UL);
			rollUi        = simLean;
			rollFiltered  = simLean;
		}
		#elif defined(TEST_MODE)
		{
			oilTempCached      = triangleWave(-30.0f, 120.0f, 30000UL); // bleibt unter 125°C (kein OIL-Screen)
			battVoltageCached  = triangleWave(0.0f,   15.0f,  12000UL);
			outsideTemp        = triangleWave(-10.0f,  40.0f,  14000UL);
			coolantTempCached  = triangleWave(20.0f,  115.0f, 25000UL); // bleibt unter 120°C (kein WATER-Screen)
			vehicleSpeedCached = triangleWave(0.0f,   120.0f, 20000UL);
			engineRpmCached    = triangleWave(800.0f, 10000.0f, 15000UL);
			engineLoadCached   = triangleWave(0.0f,   100.0f, 18000UL);
			throttlePosCached  = triangleWave(0.0f,   100.0f, 12000UL);
			float simLean = triangleWave(-90.0f, 90.0f, 8000UL);
			rollUi        = simLean;
			rollFiltered  = simLean;
		}
		#elif defined(TEST_MODE_MAIN)
		{
			oilTempCached     = 82.0f;
			battVoltageCached = 12.5f;
			outsideTemp       = 21.0f;
			float simLean     = leanTestWave();
			rollUi            = simLean;
			rollFiltered      = simLean;
		}
		#endif

		// Sleep countdown: turn display off after SLEEP_COUNTDOWN_MS
		if (sleepCountdownActive && (now - sleepCountdownStartMs) >= (unsigned long)SLEEP_COUNTDOWN_MS)
		{
			sleepCountdownActive = false;
			displaySleeping = true;
			display.ssd1306_command(0xAE); // SSD1306_DISPLAYOFF
		}

		// settings overlay takes priority over everything except blitzer
		if (displaySleeping)
		{
			// display is off – nothing to draw
		}
		else if (settingsOpen)
		{
			drawSettingsPage();
		}
		else if (millis() < blitzerActiveUntilMs)
		{
			bool flashOn = ((millis() / 200) % 2) == 0;
			display.clearDisplay();
			if (flashOn)
				display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
			display.setTextColor(flashOn ? SSD1306_BLACK : SSD1306_WHITE);
			u8g2fonts.setForegroundColor(flashOn ? SSD1306_BLACK : SSD1306_WHITE);
			uiTextCenter24(0, SCREEN_WIDTH, SCREEN_HEIGHT / 2, "BLITZ!");
			u8g2fonts.setForegroundColor(SSD1306_WHITE);
			display.setTextColor(SSD1306_WHITE);
			display.display();
		}
		else if (!isnan(oilTempCached) && oilTempCached >= OIL_CRITICAL_C)
		{
			bool flashOn = ((millis() / 300) % 2) == 0;
			display.clearDisplay();
			if (flashOn)
				display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
			display.setTextColor(flashOn ? SSD1306_BLACK : SSD1306_WHITE);
			u8g2fonts.setForegroundColor(flashOn ? SSD1306_BLACK : SSD1306_WHITE);
			uiTextCenter24(0, SCREEN_WIDTH, SCREEN_HEIGHT / 2, "OIL");
			char tbuf[8];
			snprintf(tbuf, sizeof(tbuf), "%.0fC", oilTempCached);
			uiTextCenter16(0, SCREEN_WIDTH, 18, tbuf);
			u8g2fonts.setForegroundColor(SSD1306_WHITE);
			display.setTextColor(SSD1306_WHITE);
			display.display();
		}
		else if (!isnan(coolantTempCached) && coolantTempCached >= 120.0f)
		{
			bool flashOn = ((millis() / 300) % 2) == 0;
			display.clearDisplay();
			if (flashOn)
				display.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
			display.setTextColor(flashOn ? SSD1306_BLACK : SSD1306_WHITE);
			u8g2fonts.setForegroundColor(flashOn ? SSD1306_BLACK : SSD1306_WHITE);
			uiTextCenter24(0, SCREEN_WIDTH, SCREEN_HEIGHT / 2, "WATER");
			char tbuf[8];
			snprintf(tbuf, sizeof(tbuf), "%.0fC", coolantTempCached);
			uiTextCenter16(0, SCREEN_WIDTH, 18, tbuf);
			u8g2fonts.setForegroundColor(SSD1306_WHITE);
			display.setTextColor(SSD1306_WHITE);
			display.display();
		}
		else if (page == PAGE_MAIN)
		{
			drawMainPage();
		}
		else if (page == PAGE_LEAN)
		{
			drawLeanPage();
		}
		else if (page == PAGE_G)
		{
			drawGPage();
		}
		else if (page == PAGE_ENGINE)
		{
			drawEnginePage();
		}
		else if (page == PAGE_RACEBOX)
		{
			drawRaceBoxPage();
		}
		else
		{
			drawVolumePage();
		}
	}

	// Dauer VOR dem Delay messen – delay() ist Idle, keine CPU-Last
	prevDurUs = (uint64_t)esp_timer_get_time() - loopStartUs;

	delay(1);
	yield();
}