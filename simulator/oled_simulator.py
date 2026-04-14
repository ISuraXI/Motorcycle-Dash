#!/usr/bin/env python3
"""
Motorrad-Dash OLED Simulator
Renders all 5 display pages on the PC – no ESP, no hardware needed.

Starten:
  cd /Users/tobiasjaeger/Documents/PlatformIO/Projects/Motorrad-Dash
  python3 simulator/oled_simulator.py

Controls:
  RIGHT / LEFT arrows  →  next / previous page
  UP / DOWN arrows     →  change the main value on the current page
  B                    →  toggle Blitzer warning
  O                    →  toggle oil-critical warning
  W                    →  toggle water-critical warning
  Q / Escape           →  quit

Requires:  pip install pygame
"""

import pygame
import sys
import math
import time

# ── Display dimensions (real OLED: 128×64, shown 4× bigger) ──────────────────
OLED_W, OLED_H = 128, 64
SCALE = 6          # pixel scale factor  (128*6 = 768, 64*6 = 384)
WIN_W = OLED_W * SCALE + 260   # extra space for live-editing sliders
WIN_H = OLED_H * SCALE + 60

WHITE = (255, 255, 255)
BLACK = (0,   0,   0)
BG    = (20,  20,  20)
PANEL = (35,  35,  35)

# ── Pages ──────────────────────────────────────────────────────────────────────
PAGE_OIL    = 0
PAGE_LEAN   = 1
PAGE_G      = 2
PAGE_ENGINE = 3
PAGE_RACEBOX = 4
PAGE_NAMES  = ["OIL", "LEAN", "G-Force", "Engine", "RaceBox"]
NUM_PAGES   = 5

# ── Fonts (pygame.font) ───────────────────────────────────────────────────────
pygame.init()
# ── Adafruit GFX default 5×7 bitmap font (exact OLED pixel match) ────────────
# Each character = 5 bytes (columns); bit 0 = top row, bit 7 = bottom row.
# ASCII 0x20 (space) through 0x7E (~)
_GFX_FONT = (
    b'\x00\x00\x00\x00\x00'  # 0x20 space
    b'\x00\x00_\x00\x00'    # 0x21 !
    b'\x00\x07\x00\x07\x00' # 0x22 "
    b'\x14\x7f\x14\x7f\x14' # 0x23 #
    b'$*\x7f*\x12'           # 0x24 $
    b'#\x13\x08db'           # 0x25 %
    b'6IV P'                 # 0x26 &
    b'\x00\x08\x07\x03\x00' # 0x27 \'
    b'\x00\x1c"A\x00'       # 0x28 (
    b'\x00A"\x1c\x00'       # 0x29 )
    b'*\x1c\x7f\x1c*'       # 0x2a *
    b'\x08\x08>\x08\x08'    # 0x2b +
    b'\x00\x80p0\x00'       # 0x2c ,
    b'\x08\x08\x08\x08\x08' # 0x2d -
    b'\x00\x00``\x00'       # 0x2e .
    b' \x10\x08\x04\x02'    # 0x2f /
    b'>QIE>'                 # 0x30 0
    b'\x00B\x7f@\x00'       # 0x31 1
    b'rIIIF'                 # 0x32 2
    b'!AIM3'                 # 0x33 3
    b'\x18\x14\x12\x7f\x10' # 0x34 4
    b"'EEE9"                 # 0x35 5
    b'<JII1'                 # 0x36 6
    b'A!\x11\t\x07'         # 0x37 7
    b'6III6'                 # 0x38 8
    b'FII)\x1e'              # 0x39 9
    b'\x00\x00\x14\x00\x00' # 0x3a :
    b'\x00@4\x00\x00'       # 0x3b ;
    b'\x00\x08\x14"A'       # 0x3c <
    b'\x14\x14\x14\x14\x14' # 0x3d =
    b'\x00A"\x14\x08'       # 0x3e >
    b'\x02\x01Y\t\x06'      # 0x3f ?
    b'>A]YN'                 # 0x40 @
    b'|\x12\x11\x12|'       # 0x41 A
    b'\x7fIII6'              # 0x42 B
    b'>AAA"'                 # 0x43 C
    b'\x7fAAA>'              # 0x44 D
    b'\x7fIIIA'              # 0x45 E
    b'\x7f\t\t\t\x01'       # 0x46 F
    b'>AAQs'                 # 0x47 G
    b'\x7f\x08\x08\x08\x7f' # 0x48 H
    b'\x00A\x7fA\x00'       # 0x49 I
    b' @A?\x01'              # 0x4a J
    b'\x7f\x08\x14"A'       # 0x4b K
    b'\x7f@@@@'              # 0x4c L
    b'\x7f\x02\x1c\x02\x7f' # 0x4d M
    b'\x7f\x04\x08\x10\x7f' # 0x4e N
    b'>AAA>'                 # 0x4f O
    b'\x7f\t\t\t\x06'       # 0x50 P
    b'>AQ!^'                 # 0x51 Q
    b'\x7f\t\x19)F'         # 0x52 R
    b'&III2'                 # 0x53 S
    b'\x03\x01\x7f\x01\x03' # 0x54 T
    b'?@@@?'                 # 0x55 U
    b'\x1f @ \x1f'          # 0x56 V
    b'?@8@?'                 # 0x57 W
    b'c\x14\x08\x14c'       # 0x58 X
    b'\x03\x04x\x04\x03'    # 0x59 Y
    b'aYIMC'                 # 0x5a Z
    b'\x00\x7fAA\x00'       # 0x5b [
    b'\x02\x04\x08\x10 '    # 0x5c \
    b'\x00AA\x7f\x00'       # 0x5d ]
    b'\x04\x02\x01\x02\x04' # 0x5e ^
    b'@@@@@'                 # 0x5f _
    b'\x00\x03\x07\x08\x00' # 0x60 `
    b' TTx@'                 # 0x61 a
    b'\x7f(DD8'              # 0x62 b
    b'8DDD('                 # 0x63 c
    b'8DD(\x7f'              # 0x64 d
    b'8TTT\x18'              # 0x65 e
    b'\x00\x08~\t\x02'      # 0x66 f
    b'\x18\xa4\xa4\x9cx'    # 0x67 g
    b'\x7f\x08\x04\x04x'    # 0x68 h
    b'\x00D}@\x00'          # 0x69 i
    b' @@=\x00'              # 0x6a j
    b'\x7f\x10(D\x00'       # 0x6b k
    b'\x00A\x7f@\x00'       # 0x6c l
    b'|\x04x\x04x'          # 0x6d m
    b'|\x08\x04\x04x'       # 0x6e n
    b'8DDD8'                 # 0x6f o
    b'\xfc\x18$$\x18'       # 0x70 p
    b'\x18$$\x18\xfc'       # 0x71 q
    b'|\x08\x04\x04\x08'   # 0x72 r
    b'HTTT$'                 # 0x73 s
    b'\x04\x04?D$'          # 0x74 t
    b'<@@ |'                 # 0x75 u
    b'\x1c @ \x1c'          # 0x76 v
    b'<@0@<'                 # 0x77 w
    b'D(\x10(D'              # 0x78 x
    b'L\x90\x90\x90|'       # 0x79 y
    b'DdTLD'                 # 0x7a z
    b'\x00\x086A\x00'       # 0x7b {
    b'\x00\x00w\x00\x00'    # 0x7c |
    b'\x00A6\x08\x00'       # 0x7d }
    b'\x02\x01\x02\x04\x02' # 0x7e ~
)

_GFX_W = 6   # 5 data columns + 1 pixel gap
_GFX_H = 8   # rows per character


_FONT_BIG   = None   # loaded lazily after display init
_FONT_MED   = None


def _fonts():
    global _FONT_BIG, _FONT_MED
    if _FONT_BIG is None:
        # Sizes are in OLED pixels (the buffer is 128×64, 1 px = 1 OLED dot)
        _FONT_BIG = pygame.font.SysFont("Arial", 20, bold=True)   # big numbers ~20px tall
        _FONT_MED = pygame.font.SysFont("Arial", 11, bold=False)  # medium labels
    return _FONT_BIG, _FONT_MED


# ── OLED surface renderer ─────────────────────────────────────────────────────
class OledSurface:
    """Thin wrapper that emulates the Adafruit_SSD1306 drawing API.
       All pixel coordinates are in OLED space (128×64).
       Internally we render onto a 128×64 bool-array, then scale up.
    """

    def __init__(self):
        # 1-bit framebuffer: 0=dark, 1=lit
        self._buf  = bytearray(OLED_W * OLED_H)
        # pygame surface at native OLED scale – rebuilt each frame
        self._surf = pygame.Surface((OLED_W * SCALE, OLED_H * SCALE))

    # ── Primitives ────────────────────────────────────────────────────────────

    def clear(self):
        for i in range(len(self._buf)):
            self._buf[i] = 0

    def _put(self, x, y, on=True):
        x, y = int(x), int(y)
        if 0 <= x < OLED_W and 0 <= y < OLED_H:
            self._buf[y * OLED_W + x] = 1 if on else 0

    def pixel(self, x, y, on=True):
        self._put(int(x), int(y), on)

    def hline(self, x, y, w, on=True):
        for i in range(int(w)):
            self._put(x + i, y, on)

    def vline(self, x, y, h, on=True):
        for i in range(int(h)):
            self._put(x, y + i, on)

    def line(self, x0, y0, x1, y1, on=True):
        x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
        dx = abs(x1 - x0); dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        while True:
            self._put(x0, y0, on)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy; x0 += sx
            if e2 < dx:
                err += dx; y0 += sy

    def rect(self, x, y, w, h, on=True):
        self.hline(x, y, w, on)
        self.hline(x, y + h - 1, w, on)
        self.vline(x, y, h, on)
        self.vline(x + w - 1, y, h, on)

    def fill_rect(self, x, y, w, h, on=True):
        for dy in range(int(h)):
            for dx in range(int(w)):
                self._put(x + dx, y + dy, on)

    def circle(self, cx, cy, r, on=True):
        cx, cy, r = int(cx), int(cy), int(r)
        f = 1 - r; ddF_x = 1; ddF_y = -2 * r; x = 0; y = r
        self._put(cx, cy + r, on); self._put(cx, cy - r, on)
        self._put(cx + r, cy, on); self._put(cx - r, cy, on)
        while x < y:
            if f >= 0:
                y -= 1; ddF_y += 2; f += ddF_y
            x += 1; ddF_x += 2; f += ddF_x
            for px, py in [(cx+x,cy+y),(cx-x,cy+y),(cx+x,cy-y),(cx-x,cy-y),
                           (cx+y,cy+x),(cx-y,cy+x),(cx+y,cy-x),(cx-y,cy-x)]:
                self._put(px, py, on)

    def fill_circle(self, cx, cy, r, on=True):
        for dy in range(-r, r + 1):
            length = int(math.sqrt(max(0, r * r - dy * dy)))
            self.hline(cx - length, cy + dy, 2 * length + 1, on)

    def fill_triangle(self, x0, y0, x1, y1, x2, y2, on=True):
        """Filled triangle via scanline."""
        pts = sorted([(int(y0), int(x0)), (int(y1), int(x1)), (int(y2), int(x2))])
        (ay, ax), (by, bx), (cy, cx) = pts
        def interp(ya, xa, yb, xb, y):
            if yb == ya:
                return xa
            return xa + (xb - xa) * (y - ya) / (yb - ya)
        for y in range(ay, cy + 1):
            if y <= by:
                xl = interp(ay, ax, by, bx, y)
                xr = interp(ay, ax, cy, cx, y)
            else:
                xl = interp(by, bx, cy, cx, y)
                xr = interp(ay, ax, cy, cx, y)
            xl, xr = int(min(xl, xr)), int(max(xl, xr))
            self.hline(xl, y, xr - xl + 1, on)

    def round_rect(self, x, y, w, h, r, on=True):
        self.hline(x + r, y, w - 2*r, on)
        self.hline(x + r, y + h - 1, w - 2*r, on)
        self.vline(x, y + r, h - 2*r, on)
        self.vline(x + w - 1, y + r, h - 2*r, on)
        for i in range(r + 1):
            j = int(math.sqrt(max(0, r*r - i*i)))
            self._put(x + r - i,         y + r - j,         on)
            self._put(x + w - 1 - r + i, y + r - j,         on)
            self._put(x + r - i,         y + h - 1 - r + j, on)
            self._put(x + w - 1 - r + i, y + h - 1 - r + j, on)

    def fill_round_rect(self, x, y, w, h, r, on=True):
        self.fill_rect(x + r, y, w - 2*r, h, on)
        for i in range(r + 1):
            j = int(math.sqrt(max(0, r*r - i*i)))
            self.vline(x + r - i,         y + r - j, h - 2*(r - j), on)
            self.vline(x + w - 1 - r + i, y + r - j, h - 2*(r - j), on)

    # ── Text ─────────────────────────────────────────────────────────────────

    def _stamp(self, font, text, x, y, on=True):
        """Render text into the OLED bool buffer at OLED coordinates."""
        surf = font.render(str(text), True, WHITE, BLACK)
        sw, sh = surf.get_size()
        for oy in range(sh):
            for ox in range(sw):
                r, *_ = surf.get_at((ox, oy))
                if r > 80:
                    self._put(x + ox, y + oy, on)
                elif not on:
                    self._put(x + ox, y + oy, True)  # invert: clear pixel for black-on-white

    def text_small(self, text, x, y, on=True):
        """Draw small text using the exact Adafruit GFX 5×7 bitmap font."""
        cx = x
        for ch in str(text):
            code = ord(ch)
            if 0x20 <= code <= 0x7E:
                base = (code - 0x20) * 5
                for col in range(5):
                    byte = _GFX_FONT[base + col]
                    for row in range(8):
                        if byte & (1 << row):
                            self._put(cx + col, y + row, on)
                        elif not on:
                            self._put(cx + col, y + row, True)
            cx += _GFX_W

    def text_big(self, text, x, y, on=True):
        """Draw big (~20px) bold text into the OLED buffer."""
        bfont, _ = _fonts()
        self._stamp(bfont, text, x, y, on)

    def text_med(self, text, x, y, on=True):
        _, mfont = _fonts()
        self._stamp(mfont, text, x, y, on)

    def text_width_small(self, text):
        return len(str(text)) * _GFX_W

    def text_width_big(self, text):
        bfont, _ = _fonts()
        return bfont.size(str(text))[0]

    def text_height_big(self):
        bfont, _ = _fonts()
        return bfont.get_height()

    # ── Blit to screen ────────────────────────────────────────────────────────

    def blit_to(self, screen, ox, oy):
        """Upscale the 128×64 buf and draw at (ox, oy)."""
        self._surf.fill(BLACK)
        for y in range(OLED_H):
            row = y * OLED_W
            for x in range(OLED_W):
                if self._buf[row + x]:
                    pygame.draw.rect(
                        self._surf, WHITE,
                        (x * SCALE, y * SCALE, SCALE - 1, SCALE - 1)
                    )
        screen.blit(self._surf, (ox, oy))


# ── State / simulated sensor values ──────────────────────────────────────────
class State:
    def __init__(self):
        self.page = PAGE_OIL

        # sensor values
        self.oil_temp      = 85.0    # °C
        self.outside_temp  = 18.5    # °C
        self.batt_v        = 12.4    # V
        self.coolant_temp  = 92.0    # °C
        self.rpm           = 4200.0
        self.speed         = 67.0    # km/h
        self.engine_load   = 45.0    # %
        self.throttle      = 32.0    # %
        self.lean          = 28.0    # degrees (positive = right)
        self.max_lean_l    = 34.0
        self.max_lean_r    = 34.0
        self.gx            = 0.18    # lateral g
        self.gy            = 0.42    # longitudinal g
        self.max_g         = 0.88

        # flags
        self.blitzer_warn  = False
        self.oil_critical  = False
        self.water_critical = False
        self.blitzer_alarm_until = 0.0  # time.time()

        # RaceBox
        self.rb_ble  = True
        self.rb_gps  = True
        self.rb_rec  = False
        self.rb_blt  = True

        self._t0 = time.time()

    def elapsed(self):
        return time.time() - self._t0

    def next_page(self):
        self.page = (self.page + 1) % NUM_PAGES

    def prev_page(self):
        self.page = (self.page - 1) % NUM_PAGES

    def adjust_main(self, delta):
        """UP/DOWN on the active page changes the primary value."""
        if self.page == PAGE_OIL:
            self.oil_temp = max(-30.0, min(140.0, self.oil_temp + delta))
        elif self.page == PAGE_LEAN:
            self.lean     = max(-90.0, min(90.0, self.lean + delta))
            if self.lean > 0:
                self.max_lean_r = max(self.max_lean_r, self.lean)
            else:
                self.max_lean_l = max(self.max_lean_l, -self.lean)
        elif self.page == PAGE_G:
            self.gy = max(-1.5, min(1.5, self.gy + delta * 0.1))
        elif self.page == PAGE_ENGINE:
            self.speed = max(0.0, min(250.0, self.speed + delta * 5))
            self.rpm   = max(0.0, min(14000.0, self.rpm + delta * 200))


# ── Draw helpers ──────────────────────────────────────────────────────────────

def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _mapf(v, in_min, in_max, out_min, out_max):
    t = _clamp((v - in_min) / (in_max - in_min), 0.0, 1.0)
    return int(round(out_min + t * (out_max - out_min)))


def _millis_flash(period_ms=400):
    return (int(time.time() * 1000) // period_ms) % 2 == 0


# ── Page draws ────────────────────────────────────────────────────────────────

def draw_battery_top_right(d: OledSurface, batt_v):
    if batt_v < 1:
        return
    low = batt_v < 10.5
    if low and not _millis_flash(400):
        return
    text = f"{batt_v:.1f}V"
    w = d.text_width_small(text)
    d.text_small(text, OLED_W - w - 4, 2)


def draw_oil_bar(d: OledSurface, oil_c):
    x, y, w, h = 10, 56, 108, 6
    d.rect(x, y, w, h)
    OIL_MIN, OIL_MAX = 0.0, 135.0
    GOOD_MIN, GOOD_MAX = 80.0, 100.0

    gx1 = x + 1 + _mapf(GOOD_MIN, OIL_MIN, OIL_MAX, 0, w - 3)
    gx2 = x + 1 + _mapf(GOOD_MAX, OIL_MIN, OIL_MAX, 0, w - 3)
    d.vline(gx1, y - 2, h + 4)
    d.vline(gx2, y - 2, h + 4)

    t = _clamp(oil_c, OIL_MIN, OIL_MAX)
    fill_w = _mapf(t, OIL_MIN, OIL_MAX, 0, w - 2)
    if fill_w > 0:
        d.fill_rect(x + 1, y + 1, fill_w, h - 2)

    if oil_c >= 115.0 and _millis_flash(140):
        d.fill_rect(x, y, w, h)
        d.fill_rect(x + 1, y + 1, w - 2, h - 2, on=False)
        d.vline(gx1, y - 2, h + 4)
        d.vline(gx2, y - 2, h + 4)


def draw_blitzer_indicator(d: OledSurface):
    # always-on offline indicator (simplified: blinking bars when offline)
    # in simulator we skip this unless explicitly disabled
    pass


def draw_centered_big(d: OledSurface, text, baseline_y):
    w = d.text_width_big(text)
    x = (OLED_W - w) // 2
    d.text_big(text, x, baseline_y - d.text_height_big() + 2)
    # degree circle
    rx = x + w + 5
    ry = baseline_y - d.text_height_big()
    d.circle(rx, ry, 3)


def draw_oil_page(d: OledSurface, st: State):
    d.clear()

    # "OIL" page label top-left
    d.text_small("OIL", 2, 0)

    # Big oil temp centered, baseline y=44
    if st.oil_temp < -99:
        draw_centered_big(d, "--", 44)
    else:
        draw_centered_big(d, str(int(round(st.oil_temp))), 44)

    # Horizontal divider + column separators
    d.hline(0, 50, OLED_W)
    d.vline(42, 50, 14)
    d.vline(85, 50, 14)

    # Col 1: KWT (Kühlwassertemperatur)
    d.text_small("KWT", 2, 52)
    if st.coolant_temp > -99:
        kwt_text = str(int(round(st.coolant_temp)))
        d.text_small(kwt_text, 2, 60)
        dx = 2 + d.text_width_small(kwt_text)
        d.circle(dx + 2, 59, 1)
        d.text_small("C", dx + 5, 60)
    else:
        d.text_small("--", 2, 60)

    # Col 2: AUSSEN (Außentemperatur)
    d.text_small("AUSSEN", 46, 52)
    if st.outside_temp > -99:
        at_text = f"{st.outside_temp:.0f}"
        d.text_small(at_text, 46, 60)
        dx = 46 + d.text_width_small(at_text)
        d.circle(dx + 2, 59, 1)
        d.text_small("C", dx + 5, 60)
        if st.outside_temp <= 0.0:
            sx, sy = dx + 14, 62
            d.hline(sx - 4, sy, 9)
            d.vline(sx, sy - 4, 9)
            d.line(sx - 3, sy - 3, sx + 3, sy + 3)
            d.line(sx + 3, sy - 3, sx - 3, sy + 3)
    else:
        d.text_small("--", 46, 60)

    # Col 3: BATT (Batteriespannung)
    d.text_small("BATT", 88, 52)
    if st.batt_v >= 1.0:
        d.text_small(f"{st.batt_v:.1f}V", 88, 60)
    else:
        d.text_small("--", 88, 60)


def draw_lean_semi_gauge(d: OledSurface, roll_deg):
    cx, cy, r = 64, 63, 28
    MAX_DEG = 60.0
    DANGER_DEG = 40.0
    DEADZONE = 2.0

    if abs(roll_deg) < DEADZONE:
        roll_deg = 0.0
    roll_deg = _clamp(roll_deg, -MAX_DEG, MAX_DEG)

    # arc
    for a in range(0, 181, 2):
        offset = abs(a - 90.0)
        deg_here = (offset / 90.0) * MAX_DEG
        rad = math.radians(a)
        xp = cx + round(math.cos(rad) * r)
        yp = cy - round(math.sin(rad) * r)
        d.pixel(xp, yp)
        if deg_here >= DANGER_DEG:
            d.pixel(cx + round(math.cos(rad) * (r-1)), cy - round(math.sin(rad) * (r-1)))
            d.pixel(cx + round(math.cos(rad) * (r-2)), cy - round(math.sin(rad) * (r-2)))

    # 0-marker triangle
    d.fill_triangle(cx, cy - r - 2, cx - 3, cy - r + 3, cx + 3, cy - r + 3)
    # center line
    d.line(cx, cy, cx, cy - r)

    if roll_deg == 0.0:
        return

    roll_abs = abs(roll_deg)
    sweep = (roll_abs / MAX_DEG) * 90.0
    a0 = 90.0
    a1 = (90.0 - sweep) if roll_deg >= 0 else (90.0 + sweep)

    step = 3.0
    a = a0
    while True:
        if roll_deg >= 0 and a < a1:
            break
        if roll_deg < 0 and a > a1:
            break
        a_prev = a + step if roll_deg >= 0 else a - step
        # clamp
        a_prev = _clamp(a_prev, min(a0, a1) - step, max(a0, a1) + step)
        r1 = math.radians(a)
        r2 = math.radians(a_prev)
        xA = cx + round(math.cos(r1) * (r - 3))
        yA = cy - round(math.sin(r1) * (r - 3))
        xB = cx + round(math.cos(r2) * (r - 3))
        yB = cy - round(math.sin(r2) * (r - 3))
        d.fill_triangle(cx, cy, xA, yA, xB, yB)
        if roll_deg >= 0:
            a -= step
        else:
            a += step

    # needle
    r1 = math.radians(a1)
    nx = cx + round(math.cos(r1) * (r - 1))
    ny = cy - round(math.sin(r1) * (r - 1))
    d.line(cx, cy, nx, ny)


def draw_lean_page(d: OledSurface, st: State):
    d.clear()
    live = abs(st.lean)
    draw_centered_big(d, str(int(round(live))), 28)

    # bottom labels
    d.text_small(f"L:{int(round(st.max_lean_l))}", 2, OLED_H - 9)
    rstr = f"R:{int(round(st.max_lean_r))}"
    rw = d.text_width_small(rstr)
    d.text_small(rstr, OLED_W - rw, OLED_H - 9)

    draw_lean_semi_gauge(d, st.lean)


def draw_g_circle(d: OledSurface, gx, gy, max_g):
    cx, cy, r = 64, 32, 26
    RANGE_G = 1.5

    d.line(cx - r, cy, cx + r, cy)
    d.line(cx, cy - r, cx, cy + r)
    d.circle(cx, cy, r)
    r_inner = round(r * 0.5)
    d.circle(cx, cy, r_inner)

    d.text_small("1.5g", cx + r + 2, cy - 4)

    # current dot
    dx = cx + round(_clamp(gx, -RANGE_G, RANGE_G) / RANGE_G * (r - 2))
    dy = cy - round(_clamp(gy, -RANGE_G, RANGE_G) / RANGE_G * (r - 2))
    d.fill_circle(dx, dy, 3)

    mag = math.sqrt(gx*gx + gy*gy)
    d.text_small(f"{mag:.1f}g", 1, 0)
    d.text_small(f"M:{max_g:.1f}g", 1, OLED_H - 9)


def draw_g_page(d: OledSurface, st: State):
    d.clear()
    draw_g_circle(d, st.gx, st.gy, st.max_g)


def draw_engine_page(d: OledSurface, st: State):
    d.clear()
    # speed big
    spd_text = str(int(round(st.speed)))
    sw = d.text_width_big(spd_text)
    d.text_big(spd_text, (OLED_W - sw) // 2, 4)
    d.text_small("km/h", (OLED_W + sw) // 2 + 2, 15)

    # load bottom-left
    d.text_small(f"Ld:{int(round(st.engine_load))}%", 0, 34)

    # rpm bottom-right
    rpm_text = f"{int(round(st.rpm))}rpm"
    rw = d.text_width_small(rpm_text)
    d.text_small(rpm_text, OLED_W - rw - 1, 34)

    # throttle bar
    bx, by, bw, bh = 0, 43, 128, 9
    d.rect(bx, by, bw, bh)
    fill_w = int(st.throttle / 100.0 * (bw - 2))
    if fill_w > 0:
        d.fill_rect(bx + 1, by + 1, fill_w, bh - 2)
    thr_text = f"{int(round(st.throttle))}%"
    tw = d.text_width_small(thr_text)
    fill_high = st.throttle >= 50.0
    d.text_small(thr_text, (OLED_W - tw) // 2, by + 1, on=not fill_high)

    d.text_small("0-100: hold to arm", 0, 57)


def draw_racebox_page(d: OledSurface, st: State):
    d.clear()
    title = "RaceBox"
    tw = d.text_width_small(title)
    d.text_small(title, (OLED_W - tw) // 2, 0)
    d.hline(0, 9, 127)

    BX, BW = 50, 32

    def badge_print(bx, bw, ty, text, on=True):
        tw2 = d.text_width_small(text)
        d.text_small(text, bx + (bw - tw2) // 2, ty, on=on)

    rows = [
        ("BLE",  12, 11, st.rb_ble,  "CONN", "ON", "---"),
        ("REC",  25, 24, st.rb_rec,  "REC",  None, "---"),
        ("GPS",  38, 37, st.rb_gps,  "FIX",  None, "---"),
        ("BLT",  51, 50, st.rb_blt,  "ON",   None, "OFF"),
    ]
    for label, ty, by, active, active_label, semi_label, off_label in rows:
        d.text_small(label, 8, ty)
        if active:
            d.fill_round_rect(BX, by, BW, 10, 3)
            badge_print(BX, BW, ty, active_label, on=False)
        else:
            d.round_rect(BX, by, BW, 10, 3)
            badge_print(BX, BW, ty, off_label)


def draw_blitzer_overlay(d: OledSurface):
    flash_on = _millis_flash(200)
    d.clear()
    if flash_on:
        d.fill_rect(0, 0, 128, 64)
    d.text_big("BLITZ!", 4, 22, on=not flash_on)


def draw_oil_critical(d: OledSurface, oil_c):
    flash_on = _millis_flash(300)
    d.clear()
    if flash_on:
        d.fill_rect(0, 0, 128, 64)
    d.text_big("OIL", 36, 24, on=not flash_on)
    d.text_small(f"{int(oil_c)}C", 90, 0, on=not flash_on)


def draw_water_critical(d: OledSurface, water_c):
    flash_on = _millis_flash(300)
    d.clear()
    if flash_on:
        d.fill_rect(0, 0, 128, 64)
    d.text_big("WATER", 10, 24, on=not flash_on)
    d.text_small(f"{int(water_c)}C", 90, 0, on=not flash_on)


# ── Side panel (controls + info) ─────────────────────────────────────────────

_PAN_TITLE = None
_PAN_MED   = None
_PAN_SMALL = None


def _pan_fonts():
    global _PAN_TITLE, _PAN_MED, _PAN_SMALL
    if _PAN_TITLE is None:
        _PAN_TITLE = pygame.font.SysFont("Arial", 15, bold=True)
        _PAN_MED   = pygame.font.SysFont("Arial", 13, bold=False)
        _PAN_SMALL = pygame.font.SysFont("Arial", 11, bold=False)
    return _PAN_TITLE, _PAN_MED, _PAN_SMALL


def draw_panel(screen, st: State):
    px = OLED_W * SCALE + 12
    ptitle, pmed, psmall = _pan_fonts()

    def bl(font, text, x, y, color=WHITE):
        screen.blit(font.render(str(text), True, color), (x, y))

    screen.fill(PANEL, (OLED_W * SCALE, 0, WIN_W - OLED_W * SCALE, WIN_H))
    bl(ptitle, "Motorrad-Dash Simulator", px, 10, (255, 200, 60))

    bw = WIN_W - OLED_W * SCALE - 24
    rows = [
        ("Page",    PAGE_NAMES[st.page],                 None,  None),
        ("Oil",     f"{st.oil_temp:.1f} °C",           -30,   140),
        ("Outside", f"{st.outside_temp:.1f} °C",        -20,    45),
        ("Battery", f"{st.batt_v:.1f} V",                8.0,  16.0),
        ("Coolant", f"{st.coolant_temp:.0f} °C",         20,   130),
        ("Lean",    f"{st.lean:.1f} °",                 -90,    90),
        ("Speed",   f"{st.speed:.0f} km/h",               0,   250),
        ("RPM",     f"{st.rpm:.0f}",                      0, 14000),
        ("Thr",     f"{st.throttle:.0f} %",               0,   100),
        ("Gx",      f"{st.gx:.2f} g",                 -1.5,   1.5),
        ("Gy",      f"{st.gy:.2f} g",                 -1.5,   1.5),
    ]
    y = 38
    for name, val, lo, hi in rows:
        bl(psmall, name, px,      y,     (160, 160, 160))
        bl(pmed,   val,  px + 70, y - 1, (180, 220, 255))
        if lo is not None:
            try:
                t = _clamp((float(val.split()[0]) - lo) / (hi - lo + 1e-9), 0, 1)
            except ValueError:
                t = 0
            pygame.draw.rect(screen, (50, 50, 50),   (px, y + 14, bw, 5))
            pygame.draw.rect(screen, (60, 140, 255), (px, y + 14, int(t * bw), 5))
            y += 10
        y += 20

    hints = [
        ("← →", "Page"),
        ("↑ ↓", "Adjust value"),
        ("B",   "Blitzer 5 s"),
        ("O",   "Oil critical"),
        ("W",   "Water critical"),
        ("Q",   "Quit"),
    ]
    hy = WIN_H - 14 - len(hints) * 17
    for key, desc in hints:
        bl(psmall, f"[{key}]  {desc}", px, hy, (120, 120, 120))
        hy += 17


# ── Main loop ─────────────────────────────────────────────────────────────────


def main():
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    pygame.display.set_caption("Motorrad-Dash OLED Simulator")
    clock = pygame.time.Clock()
    d  = OledSurface()
    st = State()
    _fonts(); _pan_fonts()   # pre-init after display is created

    while True:
        clock.tick(30)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit(); sys.exit()
            elif event.type == pygame.KEYDOWN:
                k = event.key
                if k in (pygame.K_q, pygame.K_ESCAPE):
                    pygame.quit(); sys.exit()
                elif k == pygame.K_RIGHT:  st.next_page()
                elif k == pygame.K_LEFT:   st.prev_page()
                elif k == pygame.K_UP:     st.adjust_main(+1)
                elif k == pygame.K_DOWN:   st.adjust_main(-1)
                elif k == pygame.K_b:
                    st.blitzer_warn = True
                    st.blitzer_alarm_until = time.time() + 5.0
                elif k == pygame.K_o:
                    st.oil_critical = not st.oil_critical
                    st.oil_temp = 128.0 if st.oil_critical else 85.0
                elif k == pygame.K_w:
                    st.water_critical = not st.water_critical
                    st.coolant_temp = 125.0 if st.water_critical else 92.0

        if st.blitzer_warn and time.time() > st.blitzer_alarm_until:
            st.blitzer_warn = False

        screen.fill(BG)

        if   st.blitzer_warn:                              draw_blitzer_overlay(d)
        elif st.oil_critical and st.oil_temp >= 125.0:    draw_oil_critical(d, st.oil_temp)
        elif st.water_critical and st.coolant_temp >= 120: draw_water_critical(d, st.coolant_temp)
        elif st.page == PAGE_OIL:     draw_oil_page(d, st)
        elif st.page == PAGE_LEAN:    draw_lean_page(d, st)
        elif st.page == PAGE_G:       draw_g_page(d, st)
        elif st.page == PAGE_ENGINE:  draw_engine_page(d, st)
        else:                         draw_racebox_page(d, st)

        oled_x, oled_y = 8, 8
        pygame.draw.rect(screen, (60, 60, 60),
                         (oled_x-3, oled_y-3, OLED_W*SCALE+6, OLED_H*SCALE+6), 2)
        d.blit_to(screen, oled_x, oled_y)

        _, _, psmall = _pan_fonts()  # returns (title, med, small)
        info = psmall.render(
            f"Page {st.page+1}/{NUM_PAGES}: {PAGE_NAMES[st.page]}   "
            f"[←→] switch   [↑↓] adjust",
            True, (150, 150, 150))
        screen.blit(info, (oled_x, oled_y + OLED_H*SCALE + 12))

        draw_panel(screen, st)
        pygame.display.flip()


if __name__ == "__main__":
    main()
