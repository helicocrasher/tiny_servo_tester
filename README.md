# Tiny Servo Signal Tester (ESP32-C3)

This project measures RC servo PWM signals (pulse width and repetition rate) on an ESP32-C3 dev board and shows values on an SSD1306 OLED.

Wiring
- Connect the servo signal line (PWM) to `GPIO5` (change `MEASURE_PIN` in `src/main.cpp` if needed).
- Connect the SSD1306 I2C pins: `SDA` -> `SDA` pin, `SCL` -> `SCL` pin (typical ESP32-C3: IO8/IO9 or board-specific pins). OLED address assumed `0x3C`.
- Connect the servo signal line (PWM) to `GPIO4` (change `MEASURE_PIN` in `src/main.cpp` if needed).
- Connect the SSD1306 I2C pins: `SDA` -> `GPIO5`, `SCL` -> `GPIO6` (OLED address assumed `0x3C`).
- Share ground between signal source and the ESP32-C3.

Build & Upload

1. Install PlatformIO and open this project.
2. Connect your `esp32-c3-devkitm-1` board.
3. Build & Upload from PlatformIO or run:

```bash
pio run -t upload -e esp32-c3-devkitm-1
```

Notes
- The code uses `attachInterrupt()` and `esp_timer_get_time()` (high-resolution timer) to measure pulse width and period.
- If your OLED uses a different I2C address or pins, update `OLED_ADDR` or `Wire.begin()` accordingly.
