#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "esp_timer.h"
#include <deque>

// Board used: Abrobot ESP32-C3 with tiny OLED https://www.espboards.dev/esp32/esp32-c3-oled-042/

#define PWM_Input_PIN 4 // default input pin for servo signal (change as needed)
#define SCREEN_WIDTH 72
#define SCREEN_HEIGHT 40
#define OLED_ADDR 0x3C
#define SDA 5
#define SCL 6 

// Use this constructor (matches user's working sample)
U8G2_SH1106_72X40_WISE_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, SCL, SDA);


portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

volatile int64_t lastRising = 0;     // timestamp of last rising edge (us)
volatile int64_t lastPeriod = 0;     // period between last two rising edges (us)
volatile int64_t lastPulseWidth = 0; // last measured pulse width (us)
volatile int64_t sawSignalLast = 0;   // timestamp of last sawSignal (us)

// Samples of (timestamp_us, period_us) for last 60 seconds
std::deque<std::pair<int64_t,int64_t>> periodSamples;
int64_t lastLoggedPeriod = 0;

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

void setup() {

  Serial.begin(115200);
  delay(100);
  Serial.println("Starting setup() in Servo Signal Analyzer");
  pinMode(PWM_Input_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PWM_Input_PIN), signal_isr, CHANGE); // activate measurement ISR
  Serial.println("Measurement Interrupt attached");
  delay(200);

  u8g2.begin();
  u8g2.setContrast(255);
  u8g2.setFont(u8g2_font_6x10_tr);
  u8g2.firstPage();
  u8g2.drawStr(0, 0, "Servo Signal Tester");
  delay(1000);

  Serial.println("Display setup & complete setup() done");
}

void loop() {
  int64_t period, pulse,last_PWM_Edge;
  bool PWM_present=false;
  char printBuf[128]="";
  int64_t now = esp_timer_get_time();

  portENTER_CRITICAL(&mux); // start critical section to safely read volatile variables - block measurement ISR
  period = lastPeriod;
  pulse = lastPulseWidth;
  last_PWM_Edge = sawSignalLast;
  portEXIT_CRITICAL(&mux); // end critical section -  unblock measurement ISR

  now = esp_timer_get_time();
  PWM_present = (now - last_PWM_Edge < 2000000LL); // signal PWM_present if seen within last 2 second
 

  float freqHz = 0.0;
  if (period > 0) freqHz = 1000000.0f / (float)period;

  if (period > 0 && period != lastLoggedPeriod) {
    periodSamples.emplace_back(now, period);
    lastLoggedPeriod = period;
  }
  int64_t windowStart = now - 60000000LL; // 60s window
  while (!periodSamples.empty() && periodSamples.front().first < windowStart) periodSamples.pop_front();

  int64_t minPeriod = 0, maxPeriod = 0;
  if (!periodSamples.empty()) {
    minPeriod = periodSamples.front().second;
    maxPeriod = periodSamples.front().second;
    for (const auto &p : periodSamples) {
      if (p.second < minPeriod) minPeriod = p.second;
      if (p.second > maxPeriod) maxPeriod = p.second;
    }
  }

  int64_t window10Start = now - 10000000LL; // 10s window
  int64_t sumPeriod10 = 0;
  size_t count10 = 0;
  for (const auto &p : periodSamples) if (p.first >= window10Start) { sumPeriod10 += p.second; ++count10; }
  float avgFreq10 = 0.0f;
  if (count10 > 0) {
    int64_t avgPeriod10 = sumPeriod10 / (int64_t)count10;
    if (avgPeriod10 > 0) avgFreq10 = 1000000.0f / (float)avgPeriod10;
  }


// Limit the values to prevent display overrun & set default values when no PWM_present
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

  if (PWM_present) {
    sprintf(printBuf, "   Pulse=%5dus Period=%6dus Freq=%6.2fHz Avg10s=%6.2fHz Min60s=%6dus Max60s=%6dus",
            (int) pulse, (int)period, freqHz, avgFreq10, (int)minPeriod, (int)maxPeriod);
  } else {
    sprintf(printBuf, "NO Pulse=%5dus Period=%6dus Freq=%6.2fHz Avg10s=%6.2fHz Min60s=%6dus Max60s=%6dus",
            (int) pulse, (int)period, freqHz, avgFreq10, (int)minPeriod, (int)maxPeriod);
  }
  Serial.println(printBuf);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
 
//  snprintf(printBuf, sizeof(printBuf), "Pin:%d %s", PWM_Input_PIN, PWM_present?"YES":"NO");
  snprintf(printBuf, sizeof(printBuf), "PWM:%s", PWM_present?"YES":"NO");
  u8g2.drawStr(0, 8, printBuf);
  snprintf(printBuf, sizeof(printBuf), "Pulse:%dus", (int)pulse);
  u8g2.drawStr(0, 18, printBuf);
  snprintf(printBuf, sizeof(printBuf), "Prd:%dus Freq:%.2f", (int)period, freqHz);
  u8g2.drawStr(0, 28, printBuf);
  snprintf(printBuf, sizeof(printBuf), "Avg10s:%.2fHz", avgFreq10);
  u8g2.drawStr(0, 38, printBuf);
  u8g2.sendBuffer();

  delay(150);
}

