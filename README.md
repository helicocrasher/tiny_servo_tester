# Tiny Servo Signal Tester (ESP32-C3 / Abrobot)

A comprehensive servo testing and signal analysis tool for the Abrobot ESP32-C3 board with built-in OLED display. This project measures incoming RC servo PWM signals, generates servo PWM output with adjustable modes, and monitors supply voltage.

## Features

- **PWM Input Measurement**: Analyzes incoming servo signals on GPIO4
  - Measures pulse width (1000-2399µs)
  - Measures period and frequency
  - Tracks min/max period over 60 seconds
  - 10-second moving average frequency
  - Signal presence detection

- **Servo PWM Output**: Generates servo control signals on GPIO7
  - 50 Hz PWM frequency
  - **Mode 1**: Fixed 1500µs pulse (center position)
  - **Mode 2**: Sweep from 1000µs to 2000µs in 2 seconds
  - Mode switching via boot button (GPIO9)

- **Voltage Measurement**: Monitors supply voltage on GPIO A2
  - Divide-by-3.6 voltage divider
  - 4 Hz update rate
  - Displayed with 2 decimal places (format: %3.2f)

- **Real-time Display**: U8G2 OLED display (72×40)
  - PWM signal status (YES/NO)
  - Pulse width, period, and frequency
  - Voltage reading and servo mode/output

- **Serial Output**: Complete monitoring via serial terminal (115200 baud)
  - Input signal analysis
  - Voltage measurement
  - Servo output mode and pulse width

## Pin Configuration

| Function | GPIO Pin | Type | Notes |
|----------|----------|------|-------|
| PWM Input | GPIO4 | Input | Interrupt-based measurement |
| Servo Output | GPIO7 | Output | 50Hz PWM, 10-bit resolution |
| Boot Button | GPIO9 | Input | Mode switching (pull-up) |
| Voltage Input | A2 | Analog Input | 3.6V divider ratio |
| I2C SDA | GPIO5 | I2C | OLED display |
| I2C SCL | GPIO6 | I2C | OLED display |

## Wiring

- **PWM Input Signal**: Connect to GPIO4 (share ground)
- **Servo Output**: Connect to GPIO7 for servo control (share ground)
- **Voltage Measurement**: Connect to GPIO A2 through a 3.6:1 voltage divider
- **I2C Display**: SDA → GPIO5, SCL → GPIO6, OLED address 0x3C
- **All signals**: Share common ground with ESP32-C3

## Build & Upload

### Target Hardware Selection

This project supports two hardware configurations:

#### 1. **Abrobot ESP32-C3 with U8G2 OLED** (Default)
- Board: Abrobot ESP32-C3 with 72×40 SH1106 display
- Display Library: U8g2 (olikraus/U8g2)
- Pin Configuration:
  - I2C SDA: GPIO5
  - I2C SCL: GPIO6
  - PWM Input: GPIO4
  - Servo Output: GPIO7
  - Boot Button: GPIO9

#### 2. **Seeed Xiao ESP32-C3 with Adafruit SSD1306**
- Board: Seeed Xiao ESP32-C3
- Display Library: Adafruit GFX + Adafruit SSD1306
- Pin Configuration:
  - I2C SDA: GPIO9
  - I2C SCL: GPIO10
  - PWM Input: GPIO4
  - Servo Output: GPIO7
  - Boot Button: GPIO9

### Build Configuration

The hardware selection is controlled by build flags in `platformio.ini`:

```ini
[env:esp32-c3-Abrobot-OLED]
build_flags = -DAbrobot          # Compiles for Abrobot hardware

[env:esp32-c3-Seeed_Xiao]
build_flags = -DSeeedXiao        # Compiles for Seeed Xiao hardware
```

### Conditional Compilation in Code

The source code uses `#ifdef` preprocessor directives to select the correct hardware configuration:

```cpp
#ifdef SeeedXiao
  // Seeed Xiao specific includes and pin definitions
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define SDA 9
  #define SCL 10
#endif

#ifdef Abrobot
  // Abrobot specific includes and pin definitions
  #include <U8g2lib.h>
  #define SDA 5
  #define SCL 6
#endif
```

### Build Instructions

1. Install PlatformIO and open this project

2. **Select target hardware** by editing `platformio.ini`:
   - For Abrobot: Uncomment `esp32-c3-Abrobot-OLED` in `default_envs`
   - For Seeed Xiao: Uncomment `esp32-c3-Seeed_Xiao` in `default_envs`

3. **Build and upload**:

```bash
# Build for default environment
pio run -t upload

# Or specify environment explicitly
pio run -e esp32-c3-Abrobot-OLED -t upload
pio run -e esp32-c3-Seeed_Xiao -t upload
```

4. Monitor serial output:

```bash
pio device monitor -b 115200
```

## Operation

### Servo Mode Control
- Press the **boot button** (GPIO9) to toggle between modes
- Mode 1: Servo held at center position (1500µs)
- Mode 2: Servo sweeps continuously between limits (1000-2000µs in 2 seconds)

### Serial Output Format
```
Pulse=XXXX µs Period=XXXXXX µs Freq=XX.XX Hz Avg10s=XX.XX Hz Min60s=XXXXXX µs Max60s=XXXXXX µs Voltage=XX.XX V | Servo: Mode=X PulseOut=XXXX µs
```

## Technical Details

- **Interrupt-based PWM measurement**: Uses edge detection on GPIO4 with critical sections for thread-safe access
- **High-resolution timing**: `esp_timer_get_time()` for microsecond precision
- **Debounced button input**: 100ms debounce interval for boot button
- **Fixed center pulse**: 1500µs (standard servo neutral position)
- **Sweep parameters**: 1000-2000µs range, 2-second full cycle (1s up, 1s down)
- **Display update rate**: 250ms (4 Hz) to prevent display flicker

## Notes

- The code uses `LEDC` PWM with 10-bit resolution (0-1023) for servo output at 50Hz
- Input measurement is interrupt-driven for accurate timing
- 60-second sample window tracks period statistics
- Voltage divider must be sized for your input voltage (currently 3.6:1 for 0-12V range)

