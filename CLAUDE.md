# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Quick Start

**To run the visualization:**
1. Open `Array_Visualization.pde` in Processing 4.x IDE
2. Press **Run** (Ctrl+R) — no build step needed
3. If a Teensy is connected at 115200 baud, it automatically detects the serial port and starts streaming data
4. Press **D** to toggle demo modes (no hardware required)

**To upload firmware:**
1. Open `Arduino_arrey1212perfect/Arduino_arrey1212perfect.ino` in Arduino IDE
2. Install `Adafruit_MCP23X17` via **Sketch → Include Library → Manage Libraries**
3. Select **Tools → Board → Teensy → Teensy 4.1** and **Tools → Port → COM#**
4. Click **Upload** (Ctrl+U) at 115200 baud

## System Architecture

This is a two-part **embedded + visualization system** for a 12×12 piezoresistive pressure sensor array.

### High-Level Data Flow

```
┌─────────────────────────────────────────────────────────────┐
│ Teensy 4.1 (Arduino)                                        │
│  • Hardware: AD5754R DAC + MCP23017 GPIO + ADG1206 mux      │
│  • Scans 144 channels sequentially                          │
│  • Serial @ 115200 baud                                     │
└────────────────────┬────────────────────────────────────────┘
                     │
                     │ Frame: "===\nR01\t...\nR02\t...\n==="
                     ↓
┌─────────────────────────────────────────────────────────────┐
│ Processing (Visualization)                                  │
│  • Parser: frames → bufferData[] → pressureData[]           │
│  • Double buffering: displayPressureData[] (lerp animation) │
│  • Renderer: P3D mesh + HSB colors + lighting               │
│  • Interactive: mouse/keyboard shortcuts                    │
└─────────────────────────────────────────────────────────────┘
```

### Arduino Hardware Layer (Teensy 4.1)

**Key Components:**
- **AD5754R** (3× daisy-chained SPI DACs): outputs 1.0V row select signals
- **MCP23017** (I2C GPIO, pins 24/25): controls row multiplexer (active-LOW, pins 0–11)
- **ADG1206** (16:1 analog mux): column selection via 4-bit address (pins 3–6)
- **ADG1419**: input gating switch
- **Teensy ADC 26**: reads voltage at [row, column]

**Non-Obvious Details:**
- **Channel mapping:** Hardware physical indices ≠ logical 1-indexed grid. The `physicalToLogicalRow[]` (12 elements) and `physicalToLogicalColumn[]` (16 elements) arrays remap all 144 readings.
- **Scan order:** `rowScanOrder_Physical[]` and `columnScanOrder_Physical[]` define scan sequence. Both arrays must be reordered together if you change scanning pattern.
- **MCP23017 pinout:** Row control on PortA pins 0–11; pins 7, 8, 9 unused. Column physical indices {1,2,3,4,5,6,10,11,12,13,14,15} map to logical columns 7–12, 1–6.

**Serial Protocol (ASCII):**
```
=== 12x12 矩阵 START
R01	<voltage>	<voltage>	... (12 columns, tab-separated)
R02	<voltage>	<voltage>	...
...
R12	<voltage>	<voltage>	...
======================== END
```

### Processing Visualization Layer

**Key Functions:**
- `processSerialData()`: line-by-line parser; accumulates rows in `bufferData[]` until frame delimiter; validates row count
- `commitFrame()`: copies `bufferData[]` → `pressureData[]` when complete
- `updateDisplayData()`: smooth interpolation `displayPressureData[]` ← `pressureData[]` using `lerp()` for animation
- `drawHeatmap3D()`: renders 13×13 mesh vertices (corners of 12×12 cells) with dual-triangle faces; applies per-cell colors
- `pressureToColor()`: voltage → pressure (via `VOLTAGE_MIN/MAX`) → HSB color (0=green, 15=yellow, 30=red)

**Rendering Pipeline:**
1. Create 13×13 vertices by averaging neighboring cells
2. Connect vertices with triangles (each 12×12 cell = 2 triangles)
3. Color each triangle per its source cell's pressure
4. Apply directional light (120°) + ambient (20%) for 3D depth

**Data Decoupling:**
- `pressureData[]` updated once per complete serial frame
- `displayPressureData[]` updated every draw cycle (60 Hz typical) with `lerp()` to smooth animation
- Separates serial bandwidth from rendering frame rate

### Configuration Touchpoints

**[Array_Visualization.pde:25-41](Array_Visualization.pde#L25-L41) — Voltage & Layout:**
- `PRESSURE_MIN/MAX`: 0–3.3V → 0–30 (default); adjust if Teensy outputs different range
- `CELL_SIZE`: mesh cell size (px)
- `PADDING`, `LEGEND_WIDTH`: layout spacing
- `SERIAL_PORT_NAME`: leave `""` for auto-detect; set to `"COM3"` etc. to pin to a specific port

**[Arduino_arrey1212perfect.ino:47-77](Arduino_arrey1212perfect/Arduino_arrey1212perfect.ino#L47-L77) — Hardware Mapping:**
- `physicalToLogicalRow[]` / `physicalToLogicalColumn[]`: remap hardware pins to logical grid (1-indexed, 1–12)
- `rowScanOrder_Physical[]` / `columnScanOrder_Physical[]`: scan sequence (reorder both together)
- `VOLTAGE_MAX` (line ~101): DAC output voltage (1.0V default on rows)

## Common Development Tasks

### Testing Without Hardware

Press **D** during execution to cycle demo modes:
1. **Gaussian** — bell curve centered at [6,6]
2. **Ring** — pressure concentrated in a circle
3. **Off** — returns to live serial input

Useful for:
- Validating UI before connecting Teensy
- Testing responsiveness
- Debugging rendering pipeline

### Debugging Serial Communication

To inspect incoming frames, add to `processSerialData()`:
```processing
if (line.startsWith("R")) {
  println("Raw: " + line.substring(0, 20) + "...");
}
```
Open **Tools → Serial Monitor** in Processing to verify frame structure and detect parsing errors.

### Modifying Scan Order

If you change how the Teensy scans (e.g., rows in different sequence):
1. Update **both** `rowScanOrder_Physical[]` and `columnScanOrder_Physical[]` in Arduino code
2. Verify new sequence appears in serial output (must still print R01–R12 in that order)
3. Do **not** modify `physicalToLogicalRow/Column[]` unless hardware wiring changes

### Changing Voltage Range

If your Teensy outputs different voltage or the sensors give 0–30 directly:
- Adjust `VOLTAGE_MIN`, `VOLTAGE_MAX` in Processing (line 30-31)
- Formula: `pressure = map(voltage, VOLTAGE_MIN, VOLTAGE_MAX, PRESSURE_MIN, PRESSURE_MAX)`
- Example: for 0–30 raw pressure, set `VOLTAGE_MIN=0, VOLTAGE_MAX=30, PRESSURE_MIN=0, PRESSURE_MAX=30`

### Interactive Controls

During execution:

| Key/Mouse | Function |
|-----------|----------|
| **D** | Cycle demo mode |
| **R** | Reset view & data |
| **S** | Save screenshot |
| **Drag** | Rotate 3D (X/Y) |
| **Scroll** | Zoom ±10% |
| **Arrows** | Pan (±50 px) |
