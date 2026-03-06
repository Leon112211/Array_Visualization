# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Running the Project

**Processing sketch (`Array_Visualization.pde`):**
- Open in Processing 4.x IDE and press the Run button (or Ctrl+R).
- No build step needed; Processing compiles and runs directly.
- The Serial library is built-in — no additional installation required.

**Arduino firmware (`Arduino_arrey1212perfect/Arduino_arrey1212perfect.ino`):**
- Upload via Arduino IDE to a **Teensy 4.1** at **115200 baud**.
- Required libraries: `Adafruit_MCP23X17` (install via Arduino Library Manager).

## Architecture

This is a two-part embedded + visualization system for a **12×12 piezoresistive pressure sensor array**.

### Data Flow

```
Teensy 4.1 (Arduino .ino)
  → scans 144 channels via hardware mux
  → outputs serial frames at 115200 baud
  → format: "=== 12x12 矩阵 ..." (frame start)
            "R01\t val\t val\t..." (12 rows of 12 tab-separated voltages)
            "========================" (frame end)

Processing (.pde)
  → reads serial frames in draw() loop via processSerialData()
  → double-buffers: bufferData[] accumulates rows until frame end, then commits to pressureData[]
  → renders heatmap: pressureData → pressureToColor() → HSB color → rect()
```

### Hardware (Arduino side)

- **AD5754R** (3× daisy-chained via SPI): 12-channel DAC driving rows at 1.0 V
- **MCP23017** (I2C on Wire2, pins 24/25): GPIO expander selecting which row is active (active-LOW)
- **ADG1206** (16:1 mux): column selection via 4-bit address on pins 3–6
- **ADG1419**: single-pole switch on the ADC input path
- **Teensy ADC pin 26**: reads voltage at selected row/column intersection

Physical→logical channel mapping is non-trivial — see `physicalToLogicalRow[]` and `physicalToLogicalColumn[]` arrays. The scan order (`rowScanOrder_Physical[]`, `columnScanOrder_Physical[]`) differs from the logical order.

### Visualization (Processing side)

- `settings()`: sets window size and calls `noSmooth()` (required for pixel-perfect 1px grid lines)
- `drawHeatmap()`: three-pass rendering — fill cells (no stroke), draw grid lines separately, then draw text labels
- `pressureToColor()`: maps pressure 0–30 → HSB color (green=0, yellow=mid, red=30); switches `colorMode` temporarily
- Voltage→pressure mapping is linear: `VOLTAGE_MIN/MAX` → `PRESSURE_MIN/MAX` (default 0–3.3V → 0–30)

### Key Configuration Constants (Processing)

| Constant | Default | Purpose |
|---|---|---|
| `CELL_SIZE` | 50 px | Grid cell size |
| `PADDING` | 60 px | Canvas margin |
| `VOLTAGE_MIN/MAX` | 0 / 3.3 | Arduino output range |
| `PRESSURE_MIN/MAX` | 0 / 30 | Pressure display range |
| `SERIAL_PORT_NAME` | `""` | Leave empty to auto-select last port |

### Keyboard Shortcuts (Processing)

| Key | Action |
|---|---|
| D | Fill demo data (Gaussian distribution, no serial needed) |
| R | Reset all data to NaN |
| S | Save screenshot as `heatmap_####.png` |
