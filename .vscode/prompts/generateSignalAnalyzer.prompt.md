---
name: generateSignalAnalyzer
description: Generate firmware to analyze PWM/servo signals on an embedded board.
argument-hint: Provide target board, framework (PlatformIO/Arduino), input pin, I2C pins, display type/resolution, and upload/monitor ports.
---

You are an expert embedded systems developer. Given a PlatformIO (or similar) embedded project, implement a compact, production-ready RC/servo PWM signal analyzer firmware that measures pulse width and repetition rate and displays results on a small I2C OLED.

Requirements (generalized):
- Target: the selected project and target board (e.g., ESP32-C3) using the specified framework (PlatformIO + Arduino by default).
- Input pin: measure an external PWM/servo signal on `the input pin` (placeholder).
- Timing: use a high-resolution timer (e.g., `esp_timer_get_time()` or `micros()` on non-ESP) inside an ISR to record rising/falling timestamps.
- Measurements: compute pulse width (microseconds), period (microseconds), and instantaneous frequency (Hz).
- History: maintain a time-windowed sample buffer (deque or ring buffer) of measured periods for the last N seconds (configurable, default 60s) to compute Min/Max period and a short-window average frequency (default 10s).
- Concurrency: protect shared ISR/main data with appropriate primitives (critical sections / portMUX / disabling interrupts) to avoid races on volatile variables.
- Display: support small I2C OLEDs via a lightweight library (prefer `U8g2`); allow using a user-provided constructor (placeholder `DISPLAY_CONSTRUCTOR`) or choose a sensible default for a given controller/resolution.
- I2C: initialize `Wire` with user-specified SDA and SCL pins (placeholders) when needed.
- Serial: produce a single-line status string periodically with consistent formatting; support zero-padded pulse width (4 digits) as required.
- Limits: clamp values before rendering to prevent overflow or unreadable output on small displays.
- Configuration: update `platformio.ini` (or dependency manifest) to add required `lib_deps` (e.g., `U8g2`) and set serial monitor speed.
- Build & deploy: compile the project, upload to the specified serial port, and open the serial monitor to verify runtime output.
- Safety & style: keep changes minimal, prefer root-cause fixes, avoid changing unrelated files, and follow existing coding style.

Deliverables:
- A complete, single-file firmware implementation (or minimal set of files) that is buildable in the project.
- `platformio.ini` updates (library deps and monitor/upload settings) if necessary.
- Short README instructions describing wiring, pins, and how to build/upload/monitor.
- A brief verification plan: commands to build, upload, and open the serial monitor.

Placeholders to replace when invoking this prompt:
- `the selected project` — path or project root to modify.
- `the input pin` — GPIO number for PWM measurement.
- `SDA` and `SCL` — I2C pins to use for the display.
- `DISPLAY_CONSTRUCTOR` — optional exact U8g2 constructor string if known (e.g., `U8G2_SH1106_72X40_WISE_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);`).
- `UPLOAD_PORT` / `MONITOR_PORT` — serial ports for upload and monitor.
- `60s` / `10s` — history and short-window durations (configurable).

Instructions for the assistant executing this prompt:
1. Inspect the project folder (`the selected project`) and open `platformio.ini` and `src/main.cpp` (or equivalent).
2. Add or update `lib_deps` to include `U8g2` (or chosen display lib) and set `monitor_speed` to `115200`.
3. Replace or create the firmware source implementing the ISR-based measurement, sample buffer, min/max/avg computations, display rendering via `DISPLAY_CONSTRUCTOR`, and serial printing with zero-padded pulse width.
4. Protect shared variables with correct primitives for the target architecture.
5. Add clamping/limiting before display to keep numbers readable on small screens.
6. Build the project; if build errors occur, fix root causes (duplicate symbols, constructor mismatches, missing includes) with minimal edits.
7. Upload to `UPLOAD_PORT` and open the serial monitor on `MONITOR_PORT` to verify output; print one example line of serial output and confirm display update.
8. Document wiring and run steps in a short `README.md`.

Output expectations when run:
- Summarize the changes (files edited and why) in a short checklist.
- Show the key snippets you added/changed (ISR, sample buffer, display init, serial line) — keep snippets concise.
- Report build and upload results and a sample serial output line.

Be conservative: if a required hardware-specific constructor or controller is unknown, ask the user for the `DISPLAY_CONSTRUCTOR` or provide a small list of likely constructors to try.
