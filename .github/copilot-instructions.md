# Copilot Instructions for D-StarBeacon

## Project Overview

D-Star transceiver firmware for the TTGO T-Beam ESP32 + SX1278 board, built with PlatformIO/Arduino. Implements D-Star DV (Digital Voice) protocol: RF header encoding/decoding, slow/fast data, DPRS position reporting, Bluetooth serial output, and a web configuration interface.

## Build & Flash Commands

```bash
# Build firmware
pio run

# Flash firmware
pio run -t upload

# Flash SPIFFS filesystem (config/web UI files)
pio run -t uploadfs

# Serial monitor
pio device monitor
```

There is no automated test suite. Hardware is required for functional testing.

## Architecture

All application logic lives in `src/main.cpp` (one large file). Local libraries under `lib/` handle the D-Star protocol layers:

| Library | Role |
|---|---|
| `lib/DSTAR` | RF header codec — convolution, Viterbi decode, de/interleave, CRC, pseudo-random scrambling (from F4GOH) |
| `lib/DStarDV` | DV frame management — slow data (5 bytes/packet), fast data (20 bytes/packet), DPRS encoding, message buffering via FreeRTOS queue |
| `lib/BitSlicer` | Bit-level RX framing — accumulates raw bits after sync, separates header bits from even/odd/sync AMBE frames |
| `lib/BitMatcher` | Shift-register pattern matcher used by BitSlicer to detect the end-of-transmission frame |
| `lib/Scrambler` | Applies/reverses the D-Star scrambler polynomial on byte buffers |
| `lib/NoOut` | Null `Stream` implementation (discard output) |

### Data flow

**TX:** `prepareHeader()` → DSTAR (CRC → convolution → interleave → pseudo-random) → ISR `txBit()` shifts out: preamble → RF header bits → DV payload bits (from `txQueue`) → stopping frame.

**RX:** SX1278 DIO0 fires `receivedSyncWord()` on preamble/sync detect → DIO1 ISR `rxBit()` calls `BitSlicer::appendBit()` bit-by-bit → `decodeHeader()` runs Viterbi → `DStarDV::receiveData()` reassembles slow/fast data → output forwarded to Bluetooth serial and `Serial`.

### LittleFS filesystem (`src/data/`)

- `config.txt` — `Key=Value` per line, loaded into `BeaconConfig` struct at boot.
- `networks.txt` — alternating `ssid` / `password` lines; first entry used as fallback AP.
- `index.html` — web config UI served by ESPAsyncWebServer.

## Key Conventions

### ISR functions
All interrupt handlers are marked `IRAM_ATTR` and must stay short. `txBit()` and `rxBit()` run on every SX1278 clock pulse (DIO1); avoid heap allocations or blocking calls inside them.

### FreeRTOS tasks
`gpsTask` and `bluetoothTask` are pinned to core 1 (`xTaskCreatePinnedToCore(..., 1)`). The main Arduino `loop()` runs on core 1 as well; `txBit`/`rxBit` ISRs fire on whatever core triggers the interrupt.

### TX queue
`txQueue` (capacity 40, element = one `uint8_t`) decouples DV frame preparation in `loop()` from bit-by-bit consumption in the `txBit` ISR. Check `DStarDV::hasSpaceInBuffer()` before pushing new frames.

### Radio mode switching
The SX1278 is used in **FSK Continuous Direct** mode (not LoRa). TX/RX are mutually exclusive; `startTX()` clears DIO0, `startRX()` clears DIO1 and re-arms DIO0 for sync detection.

### D-Star RF header layout (42 bytes raw)
`flags(3) | destination(8) | repeater(8) | companion(8) | callsign(8) | suffix(1) | ...` — fields are space-padded to their fixed widths.

### Config parsing
`setConfig(const char*)` parses one `Key=Value` line at a time. `readConfig()` reads `/config.txt` from SPIFFS line by line and calls `setConfig`. To add a new setting, add a field to `BeaconConfig`, handle it in `setConfig`, write it back in the web server POST handler, and add a default in `src/data/config.txt`.

### Board variant pin differences
Old T-Beam (manual switch): `GPS_TX=12, GPS_RX=15`. New T-Beam (AXP20X): `GPS_TX=34, GPS_RX=12`. Controlled by a `#if 0` block near the top of `main.cpp`. When targeting `ttgo-lora32-v21`, `LORA_IO1`/`LORA_IO2` pin aliases also differ — see the `#ifndef` guards.

### Debug output
Uses the `Streaming` library (`Serial << value`). Verbose decode info (header bytes, CRC result, Viterbi error count) is always printed to Serial at 115200 baud.
