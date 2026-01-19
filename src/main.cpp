#include <Arduino.h>
#include <Wire.h>
#include "esp_timer.h"
#include <deque>

// Board used: Abrobot ESP32-C3 with tiny OLED https://www.espboards.dev/esp32/esp32-c3-oled-042/
//             or Seeed Xiao ESP32 C3 with Adafruit_SSD1306 display
//
// Define the board in platformio.ini to select the correct display library and pin definitions
// [e.g., env:esp32-c3-Abrobot-OLED or env:esp32-c3-Seeed_Xiao]

#define SERVO_PWM_FREQ 50 // 50 Hz servo frequency
#define SERVO_PERIOD_US 20000 // 20ms period for 50Hz (1000000/50)
#define SERVO_MIN_PULSE 1000 // minimum pulse width in microseconds
#define SERVO_CENTER_PULSE 1500 // center position in microseconds
#define SERVO_MAX_PULSE 2000 // maximum pulse width in microseconds
#define SERVO_SWEEP_TIME_MS 2000 // 2 seconds for full sweep
#define BOOT_BUTTON_PIN 9 // ESP32-C3 boot button pin
//#define SCREEN_WIDTH 72
//#define SCREEN_HEIGHT 40
#define OLED_ADDR 0x3C
#define VOLTAGE_MEASURE_RATE_HZ 4 // 4 Hz measurement rate
#define VOLTAGE_DIVIDER_RATIO 3.6f // voltage divider ratio for A2 adjust this based on your resistor values and measurement



#ifdef SeeedXiao // Seeed Xiao ESP32 C3 with Adafruit_SSD1306 display
  #include <Adafruit_GFX.h>
  #include <Adafruit_SSD1306.h>
  #define WIRE Wire
  #define SDA 9
  #define SCL 10 
  #define SCREEN_WIDTH 128 // OLED display width, in pixels
  #define SCREEN_HEIGHT 64 // OLED display height, in pixels
  #define PWM_Input_PIN 4 // default input pin for servo signal (change as needed)
  #define VOLTAGE_MEASURE_PIN A2 // analog input for voltage measurement
  #define SERVO_OUTPUT_PIN 7 // PWM output pin for servo control
  // Derived class to add drawStr clearbuffer sendBuffer member functions
  class MyDisplay : public Adafruit_SSD1306 {
  public:
    MyDisplay(uint16_t w, uint16_t h) : Adafruit_SSD1306(w, h) {}
      void drawStr(uint8_t hor, uint8_t ver, const char * str) {
      setCursor(hor, ver);
      print(str);
    }
    void clearBuffer(void) {
      clearDisplay();
    }
    void sendBuffer(void) {
      display();
    }
  };
  MyDisplay display = MyDisplay(128, 64);
#endif 


#ifdef Abrobot
  #include <U8g2lib.h>
  #define SDA 5
  #define SCL 6 
  #define SCREEN_WIDTH 72  // OLED display width, in pixels
  #define SCREEN_HEIGHT 40 // OLED display height, in pixels
  #define PWM_Input_PIN 4 // default input pin for servo signal (change as needed)
  #define VOLTAGE_MEASURE_PIN A2 // analog input for voltage measurement
  #define SERVO_OUTPUT_PIN 7 // PWM output pin for servo control
// Use this constructor for Abrobot Display
  U8G2_SH1106_72X40_WISE_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);
#endif


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile int64_t lastRising = 0;     // timestamp of last rising edge (us)
volatile int64_t lastPeriod = 0;     // period between last two rising edges (us)
volatile int64_t lastPulseWidth = 0; // last measured pulse width (us)
volatile int64_t sawSignalLast = 0;   // timestamp of last sawSignal (us)

// Samples of (timestamp_us, period_us) for last 60 seconds
std::deque<std::pair<int64_t,int64_t>> periodSamples;
int64_t lastLoggedPeriod = 0;

// Voltage measurement variables
volatile float lastMeasuredVoltage = 0.0f;
int64_t lastVoltageMeasureTime = 0;
int64_t voltageMeasureInterval = 1000000LL / VOLTAGE_MEASURE_RATE_HZ; // microseconds between measurements

// Servo output variables
int servoMode = 1; // 1 = fixed center, 2 = sweep
volatile int servoPulseWidth = SERVO_CENTER_PULSE; // current servo pulse width in microseconds
int64_t lastBootButtonCheck = 0;
int64_t bootButtonCheckInterval = 100000LL; // 100ms debounce interval
bool lastBootButtonState = HIGH;
int64_t servoSweepStartTime = 0;

// Print buffer
#define printBufSize 200
char printBuf[printBufSize ]="";

void IRAM_ATTR signal_isr() {   // interrupt service routine to measure PWM period and PWM Pulse high width
  int level = digitalRead(PWM_Input_PIN);
  int64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&mux);
  if (level == HIGH) {
    if (lastRising != 0) {
      lastPeriod = now - lastRising;    // lastPeriod is the time between last two rising edges = result(period)
    }
    lastRising = now;
    sawSignalLast = now;
  } else {
    // falling edge -> pulse width
    if (lastRising != 0) {
      lastPulseWidth = now - lastRising; // lastPulseWidth is the time between rising and falling edge = result(pulse width)
      sawSignalLast = now;
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

float measureVoltage() {
  // Read ADC value from pin A2 (0-4095 for ESP32)
  int rawValue = analogRead(VOLTAGE_MEASURE_PIN);
  // ESP32 uses 12-bit ADC, full scale is typically 3.3V (or higher with attenuation)
  // Convert to voltage: rawValue / 4095 * 3.3V * voltage_divider_ratio
  float voltage = (rawValue / 4095.0f) * 3.3f * VOLTAGE_DIVIDER_RATIO;
  return voltage;
}

void updateServoOutput() {
  // Calculate servo pulse width based on mode
  int64_t now = esp_timer_get_time();
  
  if (servoMode == 1) {
    // Mode 1: Fixed center position (1500us)
    servoPulseWidth = SERVO_CENTER_PULSE;
  } else if (servoMode == 2) {
    // Mode 2: Sweep from 1000us to 2000us and back in 2 seconds
    int64_t elapsedMs = (now - servoSweepStartTime) / 1000LL;
    int64_t sweepCycleMs = SERVO_SWEEP_TIME_MS * 2; // up and down = 2 cycles
    int64_t positionInCycle = elapsedMs % sweepCycleMs;
    
    if (positionInCycle < SERVO_SWEEP_TIME_MS) {
      // Sweep up from MIN to MAX
      float progress = (float)positionInCycle / (float)SERVO_SWEEP_TIME_MS;
      servoPulseWidth = SERVO_MIN_PULSE + (int)(progress * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
    } else {
      // Sweep down from MAX to MIN
      float progress = (float)(positionInCycle - SERVO_SWEEP_TIME_MS) / (float)SERVO_SWEEP_TIME_MS;
      servoPulseWidth = SERVO_MAX_PULSE - (int)(progress * (SERVO_MAX_PULSE - SERVO_MIN_PULSE));
    }
  }
}

void checkBootButtonMode() {
  // Check boot button for mode switching (debounced)
  int64_t now = esp_timer_get_time();
  if (now - lastBootButtonCheck >= bootButtonCheckInterval) {
    lastBootButtonCheck = now;
    bool buttonState = digitalRead(BOOT_BUTTON_PIN);
    
    // Detect falling edge (button pressed)
    if (buttonState == LOW && lastBootButtonState == HIGH) {
      // Mode switch
      if (servoMode == 1) {
        servoMode = 2;
        servoSweepStartTime = now;
        Serial.println("Servo Mode switched to 2: SWEEP");
      } else {
        servoMode = 1;
        Serial.println("Servo Mode switched to 1: CENTER");
      }
    }
    lastBootButtonState = buttonState;
  }
}

void setup() {

  Serial.begin(115200);
  delay(100);
  Serial.println("Starting setup() in Servo Signal Analyzer");
  pinMode(PWM_Input_PIN, INPUT);
  pinMode(VOLTAGE_MEASURE_PIN, INPUT);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SERVO_OUTPUT_PIN, OUTPUT);
  
  // Configure PWM for servo output on pin 7
  // Using ledcSetup and ledcAttachPin for ESP32 PWM
  // Use 10-bit resolution (0-1023) for better frequency compatibility at 50Hz
  ledcSetup(0, SERVO_PWM_FREQ, 10); // channel 0, 50Hz, 10-bit resolution
  ledcAttachPin(SERVO_OUTPUT_PIN, 0);
  ledcWrite(0, 75); // Initial 1500us pulse = 1500/20000 * 1024 = 76.8
  // Set initial servo position
  servoPulseWidth = SERVO_CENTER_PULSE;
  servoSweepStartTime = esp_timer_get_time();
  
  attachInterrupt(digitalPinToInterrupt(PWM_Input_PIN), signal_isr, CHANGE); // activate measurement ISR
  Serial.println("Measurement Interrupt attached");
  Serial.println("Servo output configured on pin 7 at 50Hz");
  delay(200);
#ifdef SeeedXiao
  WIRE.begin(SDA,SCL);
  delay(100);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  display.display();
  delay(100);
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.clearDisplay();
  display.drawStr(0, 0, "Servo Signal Tester");
  display.display();
#endif
#ifdef Abrobot
  display.begin();
  display.setContrast(255);
  display.setFont(u8g2_font_6x10_tr);
  display.firstPage();
  display.drawStr(0, 0, "Servo Signal Tester");
#endif
  delay(1000);

  Serial.println("Display setup & complete setup() done");
}
uint32_t i=0;
int64_t lastDisplayUpdateTime = 0;

void loop() {
  int64_t period, pulse,last_PWM_Edge;

  bool PWM_present=false;

  uint64_t now = esp_timer_get_time();

  i++;
  // Check boot button for mode switching
  checkBootButtonMode();
  
  // Update servo output
  updateServoOutput();
  
  // Write servo PWM pulse using ledcWrite
  // PWM duty = (pulse_width / period) * max_value
  // For 10-bit resolution (max = 1023), and 20ms period:
  // duty = (servoPulseWidth / 20000) * 1024
  uint16_t dutyCycle = (uint16_t)((servoPulseWidth * 1024) / SERVO_PERIOD_US);
  ledcWrite(0, dutyCycle);

  // Voltage measurement at 4 Hz rate
  if (now - lastVoltageMeasureTime >= voltageMeasureInterval) {
    lastMeasuredVoltage = measureVoltage();
    lastVoltageMeasureTime = now;
  }

  portENTER_CRITICAL(&mux); // start critical section to safely read volatile variables - block measurement ISR
  period = lastPeriod;
  pulse = lastPulseWidth;
  last_PWM_Edge = sawSignalLast;
  portEXIT_CRITICAL(&mux); // end critical section -  unblock measurement ISR

  now = esp_timer_get_time();
  PWM_present = (now - last_PWM_Edge < 2000000LL); // signal PWM_present if seen within last 2 second
 

  float freqHz = 0.0;
  if (period > 0) freqHz = 1000000.0f / (float)period;

  uint64_t window10Start = now - 10000000LL; // 10s window
  uint64_t sumPeriod10 = 0;
  size_t count10 = 0;
  int64_t minPeriod = 0, maxPeriod = 0;
  float avgFreq10 = 0.0f;
/**/
  if (period > 0 && period != lastLoggedPeriod) {
    periodSamples.emplace_back(now, period);
    lastLoggedPeriod = period;
  }
  int64_t windowStart = now - 60000000LL; // 60s window
  while (!periodSamples.empty() && periodSamples.front().first < windowStart) periodSamples.pop_front(); // remove old samples


  if (!periodSamples.empty()) {
    minPeriod = periodSamples.front().second;
    maxPeriod = periodSamples.front().second;
    for (const auto &p : periodSamples) {
      if (p.second < minPeriod) minPeriod = p.second;
      if (p.second > maxPeriod) maxPeriod = p.second;
    }
  }


  for (const auto &p : periodSamples) if (p.first >= window10Start) { sumPeriod10 += p.second; ++count10; } //
 
  if (count10 > 0) {
    uint64_t avgPeriod10 = sumPeriod10 / (int64_t)count10;
    if (avgPeriod10 > 0) avgFreq10 = 1000000.0f / (float)avgPeriod10;
  }
/**/
  if (now - lastDisplayUpdateTime > 250000LL) {
    // limit display update rate to 4 Hz
    lastDisplayUpdateTime = now;
    delay(1); // allow time for Serial to flush
;;    snprintf(printBuf, printBufSize, "i=%5d  %8X", i, now);     
    snprintf(printBuf, printBufSize, "i=%8d", i); 
    Serial.print(printBuf);


// Limit the values to prevent display overrun & set default values when no PWM_present
/**/
  if (pulse > 2399) pulse=2399;
  else if ((pulse < 199)||!PWM_present) pulse=199;
  if ((period > 29999)||!PWM_present) period=29999;
  else if (period < 999) period=999;
  if (freqHz > 999) freqHz=999;
  else if ((freqHz < 9)||!PWM_present) freqHz=9;  
  if (avgFreq10 > 999) avgFreq10=999;
  else if ((avgFreq10 < 9)||!PWM_present) avgFreq10=9;
  if ((minPeriod > 29999)||!PWM_present) minPeriod=29999;
  else if (minPeriod < 999) minPeriod=999;
  if ((maxPeriod > 29999)||!PWM_present) maxPeriod=29999;
  else if (maxPeriod < 999) maxPeriod=999;
/**/
  if (PWM_present) {
    snprintf(printBuf, printBufSize, "   Pulse=%5dus Period=%6dus Freq=%6.2fHz Avg10s=%6.2fHz Min60s=%6dus Max60s=%6dus Voltage=%3.2fV | Servo: Mode=%d PulseOut=%dus",
            (int) pulse, (int)period, freqHz, avgFreq10, (int)minPeriod, (int)maxPeriod, lastMeasuredVoltage, servoMode, servoPulseWidth);
  } else {
    snprintf(printBuf, printBufSize, "NO Pulse=%5dus Period=%6dus Freq=%6.2fHz Avg10s=%6.2fHz Min60s=%6dus Max60s=%6dus Voltage=%3.2fV | Servo: Mode=%d PulseOut=%dus",
            (int) pulse, (int)period, freqHz, avgFreq10, (int)minPeriod, (int)maxPeriod, lastMeasuredVoltage, servoMode, servoPulseWidth);
  }
  Serial.println(printBuf);
  display.clearBuffer();

  snprintf(printBuf, printBufSize, "PWM:%s", PWM_present?"YES":"NO");
  display.drawStr(0, 8, printBuf);
  snprintf(printBuf, printBufSize, "Pulse:%dus", (int)pulse);
  display.drawStr(0, 18, printBuf);
  snprintf(printBuf, printBufSize, "Prd:%dus Freq:%.2f", (int)period, freqHz);
  display.drawStr(0, 28, printBuf);
  snprintf(printBuf, printBufSize, "Avg10s:%.2fHz", avgFreq10);
  display.drawStr(0, 38, printBuf);
  snprintf(printBuf, printBufSize, "Volt:%3.2fV M%d:%dus", lastMeasuredVoltage, servoMode, servoPulseWidth);
  display.drawStr(0, 48, printBuf);
  display.sendBuffer();
}
//  delay(20); // 4 Hz update rate = 250ms between updates
//  delay(1000); // 4 Hz update rate = 250ms between updates
}

