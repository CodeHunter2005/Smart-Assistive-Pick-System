# Smart Pick Assistant — ESP32-S3 Pick-to-Light System

An **offline** pick-to-light assembly guidance system running entirely on an ESP32-S3. The ESP32 creates its own WiFi access point, serves a React dashboard from flash, lights the correct bin with LEDs, and verifies every pick with per-bin load cells (HX711).

No internet, no cloud, no external server required.

---

## Features

- **Guided assembly** — step-by-step sequences with part, bin, quantity and notes
- **LED guidance** — addressable LED strips light the bin to pick from
- **Weight verification** — one HX711 load cell per bin; picked quantity = weight removed ÷ unit weight
- **Employee login** — ID + PIN roster stored on the device; operator shown in the header
- **Assembly timing & log** — every session logged (Assembly ID, operator, model, start/end, duration, status)
- **Status tab** — searchable assembly log with Excel export
- **Low-stock alerts** — per-bin full capacity; alerts when remaining stock drops below 20 %
- **On-demand calibration** — per-channel calibration stored in NVS, run only when needed (drift check available)
- **Optional laptop sync** — `log_sync.py` keeps a running `assembly_log.xlsx` on a laptop

## LED colour meaning

| Colour | Meaning |
|---|---|
| 🟡 Yellow | Pick from this bin now |
| 🟢 Green | Correct quantity picked |
| 🟠 Orange | Correct bin, wrong quantity |
| 🔴 Red | Wrong bin picked |
| ⚫ Off | Inactive bin |

---

## Hardware

| Part | Notes |
|---|---|
| ESP32-S3 (N16R8) | 16 MB flash, 8 MB Octal PSRAM |
| 6 × HX711 + load cells | one per bin, each with its own DT/SCK pair |
| 2 × WS2812B strips | 34 LEDs each, 9 LEDs per bin |
| 5 V supply | size for ESP32 + 6 HX711 + LEDs |

### Pin map (firmware v6.1)

| Signal | GPIO |
|---|---|
| LED strip A (bins 1–3) | 4 |
| LED strip B (bins 4–6) | 5 |
| Bin 1 DT / SCK | 8 / 9 |
| Bin 2 DT / SCK | 6 / 7 |
| Bin 3 DT / SCK | 15 / 16 |
| Bin 4 DT / SCK | 17 / 18 |
| Bin 5 DT / SCK | 1 / 2 |
| Bin 6 DT / SCK | 21 / 22 |

> **N16R8 note:** GPIO 26–37 are wired to the Octal PSRAM/flash and must not be used. Avoid strapping pins (0, 3, 45, 46) and USB/UART pins (19, 20, 43, 44).

---

## Architecture

```
Browser (phone / laptop)
        │  WiFi: PickToLight-Edge
        ▼
ESP32-S3  ── serves React build from LittleFS
   │      ── REST API (/api/*)
   ├── 6 × HX711 load cells   (weight verification)
   ├── 2 × WS2812B strips     (bin guidance)
   ├── LittleFS               (dashboard, sequences, roster, assembly log)
   └── NVS                    (calibration factors)
```

## Repository layout

```
.
├── firmware/
│   └── PickToLight_v6_1/
│       ├── PickToLight_v6_1.ino
│       └── data/                  ← uploaded to LittleFS
│           ├── index.html         (from dashboard/dist)
│           ├── assets/            (from dashboard/dist/assets)
│           ├── sequences.json
│           ├── employees.json
│           └── Images/
├── dashboard/                     ← React + Vite source
│   ├── src/
│   ├── package.json
│   └── vite.config.js
├── tools/
│   └── log_sync.py                ← optional laptop-side Excel sync
├── README.md
├── LICENSE
└── .gitignore
```

---

## Getting started

### 1. Build the dashboard

```bash
cd dashboard
npm install
npm run build
```

This produces `dist/` (≈ 635 KB total: main JS, CSS, and a lazily-loaded `xlsx` chunk).

### 2. Prepare the LittleFS `data/` folder

Copy **everything** from `dist/` into the sketch's `data/` folder, plus your JSON files and part images:

```bash
rm -rf ../firmware/PickToLight_v6_1/data/assets
cp dist/index.html ../firmware/PickToLight_v6_1/data/
cp -r dist/assets  ../firmware/PickToLight_v6_1/data/
```

Asset filenames are hashed and change on every build — always re-copy the whole `assets/` folder. The `xlsx-*.js` chunk is not referenced in `index.html` but **must** be uploaded or Excel export will 404.

### 3. Configure data files

**`sequences.json`**

```json
{
  "binCapacities": [500, 500, 300, 200, 150, 150],
  "models": [
    {
      "id": 0,
      "name": "Engine A",
      "image": "/Images/engine_assembly_A.jpg",
      "steps": [
        { "step": 1, "part": "Bolt M6", "bin": 1, "qty": 4, "unitWeight": 2.0, "note": "Torque to 12 Nm" }
      ]
    }
  ]
}
```

`binCapacities[i]` is the full-bin weight (g) of bin *i+1*, used for low-stock alerts. `0` or omitted disables the check for that bin.

**`employees.json`**

```json
{
  "employees": [
    { "id": "EMP001", "name": "Alice Rao", "pin": "1234" }
  ]
}
```

> PINs are stored in plaintext. This is shop-floor identification, **not** security.

### 4. Flash the firmware

1. Open `PickToLight_v6_1.ino` in Arduino IDE (ESP32 board package, **ESP32S3 Dev Module**, PSRAM: OPI).
2. Required libraries: `ArduinoJson`, `FastLED`, `HX711` (Rob Tillaart).
3. Upload the sketch.
4. Upload the `data/` folder to LittleFS (separate step from sketch upload).

### 5. Connect

- WiFi: **`PickToLight-Edge`**
- Password: `assembly123` (change in firmware)
- Open **http://192.168.4.1**

---

## Calibration

Calibration is on demand, per bin, over the Serial Monitor (115200 baud):

| Command | Action |
|---|---|
| `cal <bin>` | start calibration wizard for a bin (e.g. `cal 3`) |
| `check <bin>` | drift check against the stored reference weight |
| `t` | tare the bin being calibrated |
| `y` / `n` / `r` | save / cancel / redo at the confirm step |
| `p` | capture one-part weight (final step) |
| `skip` | skip part-weight capture |

Factors are saved in NVS and survive reboots. Calibrate each load cell individually.

---

## API overview

| Method | Endpoint | Purpose |
|---|---|---|
| POST | `/api/login`, `/api/logout` | operator sign-in / out |
| GET | `/api/models`, `/api/sequence?modelId=` | models and steps |
| POST | `/api/session/start`, `/confirm`, `/reset` | assembly session control |
| GET | `/api/weight`, `/api/status` | live weight and device status |
| GET | `/api/lowstock` | bins below the low-stock threshold |
| GET | `/api/logs` | assembly log |
| GET/POST | `/api/calibrate/*` | calibration status / control |

`/api/session/confirm` accepts an optional `binPicked` field to enable wrong-bin (red) detection.

---

## Optional: automatic Excel sync

```bash
pip install requests openpyxl
python tools/log_sync.py
```

Polls `/api/logs` every 15 s and upserts rows into `assembly_log.xlsx` (columns: S.No, Assembly ID, Emp ID, Name, Model, Started, Completed, Duration, Status). Rows are updated in place by Assembly ID, so in-progress sessions never duplicate.

The on-device log is a ring buffer capped at 200 entries; keep the sync script running if you need a complete archive.

---

## Notes & limitations

- **Timestamps are boot-relative** (`boot +04m 32s`). The device is an isolated AP with no RTC/NTP.
- Weight verification only works for bins with a responding HX711; other bins fall back to button-only confirm.
- Wrong-bin detection depends on the dashboard sending `binPicked`.
- Keep `NUM_BINS`, `HX_PINS`, and `BIN_MAP` in the firmware in sync with your wiring.

## Troubleshooting

| Symptom | Check |
|---|---|
| WiFi network not visible | Serial Monitor for AP start; power supply stability; antenna/board variant |
| Page won't load | Connected to `PickToLight-Edge`, using `http://` (not https), LittleFS upload done |
| Blank page | All files from `dist/assets/` uploaded; hashed names match `index.html` |
| Export to Excel fails | `xlsx-*.js` chunk missing from LittleFS |
| Compile error `'LogEntry' does not name a type` | Move `struct`/`enum` definitions above the forward-declaration block (Arduino auto-prototype issue) |

## License

Released under the [MIT License](LICENSE).
