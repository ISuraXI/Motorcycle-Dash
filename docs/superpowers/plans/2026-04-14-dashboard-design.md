# Dashboard Design Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Rework all 5 OLED draw functions in `src/main.cpp` to match the approved Minimal/Clean design spec.

**Architecture:** All changes are self-contained within the five `draw*Page()` functions and the `drawGCircle()` helper in `src/main.cpp`. No new files or data structures needed. The Python simulator (`simulator/oled_simulator.py`) is used to verify each page visually before committing.

**Tech Stack:** C++/Arduino, Adafruit SSD1306 + GFX, **FreeSans18pt7b** font (not bold — cleaner strokes on OLED), PlatformIO, Python/pygame simulator.

**Font decision:** Use `FreeSans18pt7b` (regular weight) instead of `FreeSansBold18pt7b` for all large numbers. Thinner strokes reduce visible pixel aliasing on the 128×64 monochrome display. The include at the top of `main.cpp` must be updated accordingly, and the helper `drawCenteredBigNumberWithDegree` must use `FreeSans18pt7b`.

---

## File Map

| Function | Lines (approx) | Change |
|---|---|---|
| `drawOilPage()` | 1720–1819 | Major rewrite: remove oil bar, add 3-col bottom layout |
| `drawLeanPage()` | 1824–1895 | Minor: add "LEAN" label + top-right max summary |
| `drawGCircle()` | 1900–1983 | Shift circle left (cx 64→48), remove inline text |
| `drawGPage()` | 1985–2010 | Add right-side panel: AKTUELL + ALL-TIME values |
| `drawEnginePage()` | 2015–2129 | Major rewrite: km/h left + 0-100 right, RPM bar, 2-col bottom |
| `drawRaceBoxPage()` | 2131–2235 | Major rewrite: circle-icon column + right summary panel |

---

## Task 1: OIL page — 3-column bottom layout

**Files:**
- Modify: `src/main.cpp:1720–1819` (`drawOilPage`)

**What changes:** Remove the oil bar (`drawOilBar`), the top-left outside temp, the top-right battery call, and the COLD label. Replace with "OIL" page label, big oil temp centered, horizontal divider at y=50, three columns below (Kühlwasser | Außen | Batterie).

- [ ] **Step 1: Open simulator and confirm current OIL page baseline**

```bash
cd /Users/tobiasjaeger/Documents/PlatformIO/Projects/Motorrad-Dash
python3 simulator/oled_simulator.py
```

Navigate to OIL page with LEFT/RIGHT arrows. Note current layout. Close simulator (Q).

- [ ] **Step 2: Replace `drawOilPage` body**

Replace the full body of `drawOilPage` (everything between the opening `{` and closing `}`) with:

```cpp
void drawOilPage(float oilC)
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setFont();
    display.setTextSize(1);

    // "OIL" page label top-left
    display.setCursor(2, 0);
    display.print("OIL");

    // Big oil temp centered, baseline y=44
    if (isnan(oilC)) {
        display.setFont(&FreeSansBold18pt7b);
        const char* s = "--";
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(s, 0, 44, &x1, &y1, &w, &h);
        display.setCursor((SCREEN_WIDTH - (int16_t)w) / 2, 44);
        display.print(s);
    } else {
        drawCenteredBigNumberWithDegree((int)round(oilC), 44);
    }

    display.setFont();
    display.setTextSize(1);

    // Horizontal divider + column separators
    display.drawFastHLine(0, 50, SCREEN_WIDTH, SSD1306_WHITE);
    display.drawFastVLine(42, 50, 14, SSD1306_WHITE);
    display.drawFastVLine(85, 50, 14, SSD1306_WHITE);

    // Col 1: Kühlwassertemperatur (OBD2)
    display.setCursor(2, 52);
    display.print("KWT");
    display.setCursor(2, 60);
    if (!isnan(coolantTempCached)) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", (int)round(coolantTempCached));
        display.print(buf);
        int16_t dx = display.getCursorX();
        display.drawCircle(dx + 2, 59, 1, SSD1306_WHITE);
        display.setCursor(dx + 5, 60);
        display.print("C");
    } else {
        display.print("--");
    }

    // Col 2: Außentemperatur (DS18B20)
    display.setCursor(46, 52);
    display.print("AUSSEN");
    display.setCursor(46, 60);
    if (!isnan(outsideTemp)) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%.0f", outsideTemp);
        display.print(buf);
        int16_t dx = display.getCursorX();
        display.drawCircle(dx + 2, 59, 1, SSD1306_WHITE);
        display.setCursor(dx + 5, 60);
        display.print("C");
        if (outsideTemp <= 0.0f)
            drawSnowflakeWarning(dx + 14, 62);
    } else {
        display.print("--");
    }

    // Col 3: Batteriespannung
    display.setCursor(88, 52);
    display.print("BATT");
    display.setCursor(88, 60);
    if (!isnan(battVoltageCached) && battVoltageCached >= 1.0f) {
        bool lowBatt = battVoltageCached < BATT_LOW_V;
        if (!lowBatt || ((millis() / 400) % 2) == 0) {
            char buf[8];
            snprintf(buf, sizeof(buf), "%.1fV", battVoltageCached);
            display.print(buf);
        }
    } else {
        display.print("--");
    }

    // Settings hold-progress bar (grows while button held)
    if (btn.pressed) {
        unsigned long held = millis() - btn.pressStartMs;
        int barW = (int)((float)held / (float)SETTINGS_OPEN_MS * 126.0f);
        if (barW > 126) barW = 126;
        if (barW > 0)
            display.fillRect(1, 63, barW, 1, SSD1306_WHITE);
    }

    drawBlitzerWarnerAliveIndicator();
    drawRpmRedlineBorder();
    display.display();
}
```

- [ ] **Step 3: Update simulator to match**

In `simulator/oled_simulator.py`, find the `draw_oil_page()` function and update it to mirror the new layout (OIL label top-left, big temp, divider, 3-col bottom). Run simulator and confirm the OIL page looks like the approved mockup.

```bash
python3 simulator/oled_simulator.py
```

Expected: OIL label top-left, big temperature centered, horizontal line at ~50, three columns (KWT | AUSSEN | BATT) below.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp simulator/oled_simulator.py
git commit -m "redesign OIL page: 3-column bottom layout, remove oil bar"
```

---

## Task 2: LEAN page — add page label + top-right max summary

**Files:**
- Modify: `src/main.cpp:1824–1895` (`drawLeanPage`)

**What changes:** Add "LEAN" label top-left and "L xx° | R xx°" compact max summary top-right. The existing gauge, angle value, and bottom max display stay intact.

- [ ] **Step 1: Add labels at the top of `drawLeanPage`**

Directly after `display.clearDisplay(); display.setTextColor(SSD1306_WHITE);`, insert:

```cpp
    // Page label top-left
    display.setFont();
    display.setTextSize(1);
    display.setCursor(2, 0);
    display.print("LEAN");

    // Max summary top-right: "L xx | R xx"
    {
        char buf[18];
        snprintf(buf, sizeof(buf), "L%d|R%d",
                 (int)round(maxLeanLeft), (int)round(maxLeanRight));
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
        display.setCursor(SCREEN_WIDTH - (int16_t)w - 2, 0);
        display.print(buf);
    }
```

- [ ] **Step 2: Verify in simulator**

```bash
python3 simulator/oled_simulator.py
```

Navigate to LEAN page. Expected: "LEAN" top-left, "L42|R38" style top-right, existing gauge and values unchanged.

- [ ] **Step 3: Commit**

```bash
git add src/main.cpp
git commit -m "LEAN page: add page label and max summary top-right"
```

---

## Task 3: G-FORCE page — shift circle left, add right value panel

**Files:**
- Modify: `src/main.cpp:1900–1983` (`drawGCircle`)
- Modify: `src/main.cpp:1985–2010` (`drawGPage`)

**What changes:** Shift the G circle from center (cx=64) to left-of-center (cx=48). Remove the inline `g` and `M:` text that currently sit top-left and bottom-left. Add a right-side panel in `drawGPage` with "AKTUELL" + current g-value and "ALL-TIME" + max g-value.

- [ ] **Step 1: Update `drawGCircle` — shift circle left, remove text labels**

Change the constants at the top of `drawGCircle`:
```cpp
// OLD:
const int16_t cx = 64;
const int16_t cy = 32;

// NEW:
const int16_t cx = 48;
const int16_t cy = 32;
```

Remove the "1.5g" ring legend block (currently `display.setCursor(cx + r + 2, cy - 4); display.print("1.5g");`) — the right panel takes over.

Remove the two text blocks at the bottom of `drawGCircle`:
```cpp
// DELETE these lines:
float mag = sqrtf(gx * gx + gy * gy);
display.setCursor(1, 0);
display.print(mag, 1);
display.print("g");

display.setCursor(1, display.height() - 9);
display.print("M:");
display.print(maxGVal, 1);
display.print("g");
```

- [ ] **Step 2: Add right panel in `drawGPage`**

After `drawGCircle(gX, gY, maxGSaved);`, add:

```cpp
    display.setFont();
    display.setTextSize(1);

    // Vertical divider between circle and panel
    display.drawFastVLine(80, 0, SCREEN_HEIGHT, SSD1306_WHITE);

    // Page label top-left (above circle)
    display.setCursor(2, 0);
    display.print("G");

    // Right panel: current G
    display.setCursor(83, 4);
    display.print("AKTUELL");
    {
        float mag = sqrtf(gX * gX + gY * gY);
        char buf[8];
        snprintf(buf, sizeof(buf), "%.1fg", mag);
        display.setTextSize(2);
        display.setCursor(83, 14);
        display.print(buf);
        display.setTextSize(1);
    }

    // Right panel: all-time max
    display.setCursor(83, 36);
    display.print("ALL-TIME");
    {
        char buf[8];
        snprintf(buf, sizeof(buf), "%.1fg", maxGSaved);
        display.setTextSize(2);
        display.setCursor(83, 46);
        display.print(buf);
        display.setTextSize(1);
    }
```

- [ ] **Step 3: Verify in simulator**

```bash
python3 simulator/oled_simulator.py
```

Navigate to G-Force page. Expected: G-circle shifted left (center at x=48), vertical divider at x=80, AKTUELL + value top-right, ALL-TIME + value bottom-right.

- [ ] **Step 4: Commit**

```bash
git add src/main.cpp
git commit -m "G-FORCE page: shift circle left, add right-side value panel"
```

---

## Task 4: ENGINE page — km/h left, 0-100 right, RPM bar, 2-col bottom

**Files:**
- Modify: `src/main.cpp:2015–2129` (`drawEnginePage`)

**What changes:** Major layout restructure. km/h big left (not centered). 0-100 timer top-right. RPM bar full-width below. Drossel + Last as 2-column at bottom.

- [ ] **Step 1: Replace `drawEnginePage` body**

Replace the full body of `drawEnginePage` (everything between `{` and `}`) with:

```cpp
void drawEnginePage()
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setFont();
    display.setTextSize(1);

    // ---- km/h — big, left-aligned ----
    {
        display.setFont(&FreeSansBold18pt7b);
        char kmhBuf[8];
        if (!isnan(vehicleSpeedCached))
            snprintf(kmhBuf, sizeof(kmhBuf), "%d", (int)round(vehicleSpeedCached));
        else
            snprintf(kmhBuf, sizeof(kmhBuf), "--");
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(kmhBuf, 0, 36, &x1, &y1, &w, &h);
        display.setCursor(-x1, 36);
        display.print(kmhBuf);
        display.setFont();
        display.setTextSize(1);
        display.setCursor(-x1 + (int16_t)w + 2, 11);
        display.print("km/h");
    }

    // Vertical divider between km/h and 0-100 section
    display.drawFastVLine(76, 0, 41, SSD1306_WHITE);

    // ---- 0-100 timer — top right ----
    display.setFont();
    display.setTextSize(1);
    display.setCursor(79, 2);
    display.print("0-100");
    {
        display.setTextSize(2);
        switch (sprint100State) {
            case S100_IDLE:
                display.setTextSize(1);
                display.setCursor(79, 14);
                display.print("bereit");
                break;
            case S100_ARMED:
                display.setTextSize(1);
                display.setCursor(79, 14);
                display.print("READY!");
                display.setCursor(79, 24);
                display.print("los!");
                break;
            case S100_RUNNING: {
                float elapsed = (float)(millis() - sprint100StartMs) / 1000.0f;
                char tbuf[8];
                snprintf(tbuf, sizeof(tbuf), "%.1fs", elapsed);
                display.setCursor(79, 14);
                display.print(tbuf);
                break;
            }
            case S100_DONE: {
                char tbuf[8];
                snprintf(tbuf, sizeof(tbuf), "%.2fs", sprint100Result);
                display.setCursor(79, 14);
                display.print(tbuf);
                break;
            }
        }
        display.setTextSize(1);
    }

    // ---- Horizontal divider below top zone ----
    display.drawFastHLine(0, 41, SCREEN_WIDTH, SSD1306_WHITE);

    // ---- RPM bar ----
    {
        const int bx = 1, by = 44, bw = 126, bh = 5;
        display.drawRect(bx, by, bw, bh, SSD1306_WHITE);
        const float RPM_MAX = 12000.0f;
        if (!isnan(engineRpmCached) && engineRpmCached > 0.0f) {
            int fillW = (int)(clampf(engineRpmCached, 0, RPM_MAX) / RPM_MAX * (float)(bw - 2));
            if (fillW > 0)
                display.fillRect(bx + 1, by + 1, fillW, bh - 2, SSD1306_WHITE);
        }
        // Redline marker at 10500 RPM
        int redlineX = bx + 1 + (int)(10500.0f / RPM_MAX * (float)(bw - 2));
        display.drawFastVLine(redlineX, by - 1, bh + 2, SSD1306_WHITE);

        // RPM value as small text to right of bar
        if (!isnan(engineRpmCached)) {
            char buf[10];
            snprintf(buf, sizeof(buf), "%drpm", (int)round(engineRpmCached));
            int16_t x1, y1; uint16_t w, h;
            display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
            display.setCursor(SCREEN_WIDTH - (int16_t)w - 1, by - 8);
            display.print(buf);
        }
    }

    // ---- Horizontal divider above bottom row ----
    display.drawFastHLine(0, 52, SCREEN_WIDTH, SSD1306_WHITE);

    // ---- 2-col bottom: Drossel | Last ----
    display.drawFastVLine(63, 52, 12, SSD1306_WHITE);

    // Left: Drosselklappenstellung
    display.setCursor(2, 54);
    display.print("DROSSEL");
    display.setCursor(2, 54);
    if (!isnan(throttlePosCached)) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", (int)round(throttlePosCached));
        // right-align within left column
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
        display.setCursor(60 - (int16_t)w, 54);
        display.print(buf);
    }

    // Right: Motorlast
    display.setCursor(66, 54);
    display.print("LAST");
    if (!isnan(engineLoadCached)) {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d%%", (int)round(engineLoadCached));
        int16_t x1, y1; uint16_t w, h;
        display.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
        display.setCursor(125 - (int16_t)w, 54);
        display.print(buf);
    }

    drawBlitzerWarnerAliveIndicator();
    drawRpmRedlineBorder();
    display.display();
}
```

- [ ] **Step 2: Verify in simulator**

```bash
python3 simulator/oled_simulator.py
```

Navigate to Engine page. Use UP/DOWN arrows to change speed value. Expected: km/h big left, 0-100 top-right, RPM bar with redline marker, Drossel/Last bottom.

- [ ] **Step 3: Commit**

```bash
git add src/main.cpp
git commit -m "ENGINE page: km/h left, 0-100 right, RPM bar, 2-col bottom"
```

---

## Task 5: RACEBOX page — icon column + right summary, Blitzer status

**Files:**
- Modify: `src/main.cpp:2131–2235` (`drawRaceBoxPage`)

**What changes:** Replace centered badge layout with a left column of circle icons (GPS/BLE/REC/BLZ) and a right summary panel (RaceBox status + Blitzer status). The page already has Blitzer (`blitzerAliveReceived`) — just reposition it. Remove centered title.

- [ ] **Step 1: Replace `drawRaceBoxPage` body**

Replace the full body of `drawRaceBoxPage` with:

```cpp
void drawRaceBoxPage()
{
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setFont();
    display.setTextSize(1);

    // Page label top-left
    display.setCursor(2, 0);
    display.print("RACEBOX");

    // Vertical divider between columns
    display.drawFastVLine(82, 0, SCREEN_HEIGHT, SSD1306_WHITE);

    // ── Left column: status icons ──
    // Each row: filled circle = active, empty circle = inactive
    // Row spacing: 16px. Circles at x=8, text at x=18.
    // Row 1 (y=14): GPS
    {
        bool fix = raceboxGps;
        if (fix) display.fillCircle(8, 18, 6, SSD1306_WHITE);
        else      display.drawCircle(8, 18, 6, SSD1306_WHITE);
        display.setTextColor(fix ? SSD1306_BLACK : SSD1306_WHITE);
        display.setCursor(5, 15);
        display.print("GPS");
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(18, 15);
        display.print(fix ? "FIX" : "---");
    }

    // Row 2 (y=30): BLE
    {
        bool bleAlive = raceboxBleAliveLastMs > 0 &&
                        (millis() - raceboxBleAliveLastMs) < RACEBOX_BLE_ALIVE_TIMEOUT_MS;
        bool active = raceboxBle || bleAlive;
        if (active) display.fillCircle(8, 34, 6, SSD1306_WHITE);
        else         display.drawCircle(8, 34, 6, SSD1306_WHITE);
        display.setTextColor(active ? SSD1306_BLACK : SSD1306_WHITE);
        display.setCursor(5, 31);
        display.print("BLE");
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(18, 31);
        display.print(raceboxBle ? "CONN" : (bleAlive ? "ON" : "---"));
    }

    // Row 3 (y=46): REC
    {
        bool rec = raceboxRec;
        if (rec) display.fillCircle(8, 50, 6, SSD1306_WHITE);
        else      display.drawCircle(8, 50, 6, SSD1306_WHITE);
        display.setTextColor(rec ? SSD1306_BLACK : SSD1306_WHITE);
        display.setCursor(5, 47);
        display.print("REC");
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(18, 47);
        display.print(rec ? "REC" : "---");
    }

    // Row 4 (y=62): Blitzer
    {
        bool alive = blitzerAliveReceived &&
                     (millis() - blitzerAliveLastMs) < BLITZER_ALIVE_TIMEOUT_MS;
        if (alive) display.fillCircle(8, 60, 4, SSD1306_WHITE);
        else        display.drawCircle(8, 60, 4, SSD1306_WHITE);
        display.setTextColor(alive ? SSD1306_BLACK : SSD1306_WHITE);
        // "BLZ" doesn't fit at r=4 with black text — keep white label beside
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(16, 57);
        display.print(alive ? "BLZ ON" : "BLZ --");
    }

    // ── Right panel ──
    // RaceBox status
    display.setCursor(85, 4);
    display.print("RACEBOX");
    {
        bool bleAlive = raceboxBleAliveLastMs > 0 &&
                        (millis() - raceboxBleAliveLastMs) < RACEBOX_BLE_ALIVE_TIMEOUT_MS;
        display.setCursor(85, 14);
        if (raceboxRec)
            display.print("AUFNAHME");
        else if (raceboxBle)
            display.print("BEREIT");
        else if (bleAlive)
            display.print("ON");
        else
            display.print("---");
    }

    display.drawFastHLine(82, 30, 46, SSD1306_WHITE);

    // Blitzer status
    display.setCursor(85, 33);
    display.print("BLITZER");
    {
        bool alive = blitzerAliveReceived &&
                     (millis() - blitzerAliveLastMs) < BLITZER_ALIVE_TIMEOUT_MS;
        display.setCursor(85, 43);
        if (!blitzerAliveReceived) {
            display.print("???");
        } else if (alive) {
            display.print("ONLINE");
        } else {
            // blink offline warning
            if (((millis() / 400) % 2) == 0)
                display.print("OFFLINE");
        }
    }

    // REC button trigger indicator (top-right flash when triggering)
    auto printCentered = [&](int bx, int bw, int ty, const char* text) {
        int16_t x1, y1; uint16_t tw, th;
        display.getTextBounds(text, 0, ty, &x1, &y1, &tw, &th);
        display.setCursor(bx + ((int16_t)bw - (int16_t)tw) / 2, ty);
        display.print(text);
    };
    if (raceboxBtnUntilMs > 0 && millis() < raceboxBtnUntilMs) {
        display.fillRoundRect(95, 0, 33, 9, 2, SSD1306_WHITE);
        display.setTextColor(SSD1306_BLACK);
        printCentered(95, 33, 1, "REC");
        display.setTextColor(SSD1306_WHITE);
    }

    // Bottom hint
    display.setCursor(85, 55);
    display.print("lang=REC");

    drawRpmRedlineBorder();
    display.display();
}
```

- [ ] **Step 2: Verify in simulator**

```bash
python3 simulator/oled_simulator.py
```

Navigate to RaceBox page. Toggle states with keyboard. Expected: 4 status icons left column (GPS/BLE/REC/BLZ), divider at x=82, RACEBOX status + BLITZER status right panel.

- [ ] **Step 3: Commit**

```bash
git add src/main.cpp
git commit -m "RACEBOX page: icon column + right summary, Blitzer status"
```

---

## Task 6: Build and smoke-test on device (optional but recommended)

**Files:** none

- [ ] **Step 1: Build firmware**

```bash
cd /Users/tobiasjaeger/Documents/PlatformIO/Projects/Motorrad-Dash
pio run
```

Expected: build succeeds with 0 errors. Warnings about unused variables are OK.

- [ ] **Step 2: Upload and test**

```bash
pio run --target upload
pio device monitor
```

Cycle through all pages with the button and confirm each matches the simulator output.

- [ ] **Step 3: Final commit if any last-minute fixes were needed**

```bash
git add src/main.cpp
git commit -m "fix: post-hardware-test tweaks to display layout"
```

---

## Self-Review Checklist

- [x] **OIL**: oil bar removed, 3-col layout (KWT/AUSSEN/BATT), OIL label, big temp — ✓
- [x] **LEAN**: LEAN label + L/R max top-right — ✓ (existing gauge/values untouched)
- [x] **G-FORCE**: circle at cx=48, right panel AKTUELL + ALL-TIME — ✓
- [x] **ENGINE**: km/h big left, 0-100 right, RPM bar+redline, DROSSEL+LAST bottom — ✓
- [x] **RACEBOX**: GPS/BLE/REC/BLZ icon column, right summary, no 0-100 timer — ✓
- [x] `blitzerAliveReceived` / `blitzerAliveLastMs` / `BLITZER_ALIVE_TIMEOUT_MS` used consistently in both RACEBOX and old indicator — ✓
- [x] `sprint100State`, `sprint100StartMs`, `sprint100Result` all referenced from existing globals — ✓
- [x] `drawBlitzerWarnerAliveIndicator()` removed from RACEBOX page (it's a side indicator not needed when Blitzer has its own row) — ✓
- [x] `drawRpmRedlineBorder()` kept on all pages — ✓
- [x] **Font:** `FreeSansBold18pt7b` replaced by `FreeSans18pt7b` everywhere — update `#include`, `drawCenteredBigNumberWithDegree`, and ENGINE page km/h block — ✓
