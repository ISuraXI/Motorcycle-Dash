# Motorrad-Dash — Display Design Spec

**Datum:** 2026-04-14  
**Display:** 128×64 OLED (SSD1309), monochrom, transparent  
**Design-Stil:** Minimal/Clean — großer Hauptwert, dezente Hilfselemente, klare Hierarchie

---

## Allgemeine Designprinzipien

- **Schriftart:** Monospace (Adafruit GFX default 5×7 + FreeSans-Fonts für große Zahlen)
- **Hauptwert:** Immer groß und zentriert (oder links) — auf einen Blick lesbar
- **Label:** Klein, gedimmt (opacity ~70 %), über dem Wert
- **Trennlinien:** Dünne horizontale oder vertikale Linien zur Strukturierung
- **Seiten-Label:** Oben links, klein (z. B. `OIL`, `LEAN`)
- **Batteriespannung:** Oben rechts auf der OIL-Seite; auf anderen Seiten ggf. Statusinfo

---

## Seiten-Übersicht

Kurzdrucken wechselt: **OIL → LEAN → G → ENGINE → RACEBOX → OIL …**

---

## Seite 1 — OIL

```
OIL                          12.4V
 
           98°
 
─────────────────────────────────
KÜHLWASSER  │  AUSSEN  │  BATTERIE
   82°C     │   18°C   │   12.4V
```

**Layout:**
- Oben links: Label `OIL`
- Oben rechts: Batterispannung (Kurzanzeige)
- Mitte: Öltemperatur in °C, sehr groß zentriert (FreeSansBold18pt oder äquivalent)
- Horizontale Trennlinie ~2/3 der Höhe
- Untere Zone 3-spaltig mit vertikalen Trennlinien:
  - Links: Kühlwassertemperatur (OBD2)
  - Mitte: Außentemperatur (DS18B20)
  - Rechts: Batterispannung

**Warnungen (bestehend, unverändert):**
- Öltemp kritisch (>125 °C): Vollbild-Overlay
- Batterie niedrig: blinkender Text

---

## Seite 2 — LEAN

```
LEAN                   L 42° | R 38°

        /  35°  \
       /    ↑    \
      /_____|_____\

MAX-L  42°  │  MAX-R  38°
```

**Layout:**
- Oben links: Label `LEAN`
- Oben rechts: All-time-Max kurz (L xx° | R xx°)
- Mitte: Halbkreis-Gauge (Bogen ~180°), Nadel zeigt aktuellen Winkel
- Aktueller Winkelwert groß, über dem Bogenmittelpunkt
- Horizontale Trennlinie unten
- Untere Zone 2-spaltig: MAX-L links, MAX-R rechts

---

## Seite 3 — G-FORCE

```
G-FORCE                    MAX 1.8g

  ┌─────┐    AKTUELL
  │  ·  │      0.8g
  │  ○  │
  └─────┘    ALL-TIME
               1.8g
```

**Layout:**
- Oben links: Label `G-FORCE`
- Oben rechts: All-time-Max kurz
- Links (ca. 58 % Breite): Kreis-Plot mit 2 konzentrischen Kreisen + Fadenkreuz; weißer Punkt = aktueller G-Vektor, kleiner Kreis = All-time-Max-Position
- Vertikale Trennlinie
- Rechts: `AKTUELL` (Label) + aktueller g-Wert groß, darunter `ALL-TIME` + gespeicherter Maximalwert

---

## Seite 4 — ENGINE

```
ENGINE                   5400 RPM

  87          │  0–100 km/h
 km/h         │    4.2s
              │  (letzter Run)
──────────────────────────────────
[═══════════════════|···········]  ← RPM-Balken (| = Redline)
──────────────────────────────────
DROSSEL  43%  │  LAST  61%
```

**Layout:**
- Oben links: Label `ENGINE`
- Oben rechts: RPM-Zahl klein (z. B. `5400 RPM`)
- Obere Zone 2-spaltig:
  - Links: Geschwindigkeit in km/h, sehr groß
  - Rechts: 0–100-km/h-Timer (letztes Ergebnis in Sekunden)
- Horizontale Trennlinie
- RPM-Balken (volle Breite), mit Redline-Marker als vertikaler Strich
- Horizontale Trennlinie
- Untere Zone 2-spaltig: Drosselklappenstellung % | Motorlast %

**0–100-Timer-Logik (bestehend, unverändert):**
- Startet automatisch bei Fahrtbeginn, zeigt Ergebnis nach Erreichen von 100 km/h

---

## Seite 5 — RACEBOX

```
RACEBOX

[GPS] FIX ✓        │  RACEBOX
[BLE] VERBUNDEN    │  BEREIT
[REC] BEREIT       │  ─────────
[BLZ] ON ✓         │  BLITZER
                   │  ONLINE ✓
──────────────────────────────────
        Lang drücken → REC starten
```

**Layout:**
- Oben links: Label `RACEBOX`
- Linke Spalte (Status-Icons als gefüllte/leere Kreise + Label):
  - `GPS` — FIX ✓ / KEIN FIX
  - `BLE` — VERBUNDEN / GETRENNT
  - `REC` — AKTIV / BEREIT
  - `BLZ` — ON ✓ / OFF (Blitzer-Warner)
- Vertikale Trennlinie
- Rechte Spalte:
  - RaceBox-Gesamtstatus (BEREIT / AUFNAHME)
  - Blitzer-Warner-Status (ONLINE / OFFLINE)
- Untere Linie + Hinweis: `Lang drücken → REC starten`

**Kein 0–100-Timer auf dieser Seite** (liegt auf ENGINE-Seite).

---

## Nicht geändert

- Boot-Sequenz (Sensor-Init mit OK/FAIL)
- Settings-Seite
- Warn-Overlays (Öl kritisch, Batterie niedrig, Blitzer-Alarm)
- Kalibrierungslogik
- Night-Mode / Auto-Brightness
- Alle Sensor-Logiken und EEPROM-Handling
