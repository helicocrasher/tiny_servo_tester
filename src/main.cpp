#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "esp_timer.h"
#include <deque>
//#include <cstring>

#define MEASURE_PIN 4 // default input pin for servo signal (change as needed)
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
volatile bool sawSignal = false;

// Samples of (timestamp_us, period_us) for last 60 seconds
std::deque<std::pair<int64_t,int64_t>> periodSamples;
int64_t lastLoggedPeriod = 0;

void IRAM_ATTR signal_isr() {
  int level = digitalRead(MEASURE_PIN);
  int64_t now = esp_timer_get_time();
  portENTER_CRITICAL_ISR(&mux);
  if (level == HIGH) {
    if (lastRising != 0) {
      lastPeriod = now - lastRising;
    }
    lastRising = now;
    sawSignal = true;
  } else {
    // falling edge -> pulse width
    if (lastRising != 0) {
      lastPulseWidth = now - lastRising;
      sawSignal = true;
    }
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  // initialize I2C on SDA=GPIO5, SCL=GPIO6
  //Wire.begin(SDA, SCL ); is obsolte and conflicting with OLED
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting setup() in Servo Signal Analyzer");
  pinMode(MEASURE_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(MEASURE_PIN), signal_isr, CHANGE);
  Serial.println("Measurement Interrupt attached");
  delay(200);

  u8g2.begin();
  u8g2.setContrast(255);
  //delay(1000);
  //u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  //u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.firstPage();
  u8g2.drawStr(0, 10, "Servo Signal Analyzer");
  //u8g2.sendBuffer();
  delay(100);

//  u8g2.nextPage();
  Serial.println("Display setup & complete setup() done");
}

void loop() {
  int64_t period, pulse;
  bool present;
  portENTER_CRITICAL(&mux);
  period = lastPeriod;
  pulse = lastPulseWidth;
  present = sawSignal;
  portEXIT_CRITICAL(&mux);

  float freqHz = 0.0;
  if (period > 0) freqHz = 1000000.0f / (float)period;

  int64_t now = esp_timer_get_time();
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
/*
  char pulseBuf[16];
  if (pulse > 0) {
    if (pulse < 10000) sprintf(pulseBuf, "%04lld", (long long)pulse);
    else sprintf(pulseBuf, "%lld", (long long)pulse);
  } else {
    strncpy(pulseBuf, "--", sizeof(pulseBuf));
    pulseBuf[sizeof(pulseBuf)-1] = '\0';
  }
*/

// Limit the values to prevent display overrun
  if (pulse > 2399) pulse=2399;
  else if (pulse < 199) pulse=199;
  if (period > 29999) period=29999;
  else if (period < 999) period=999;
  if (freqHz > 999) freqHz=999;
  else if (freqHz < 9) freqHz=9;  
  if (avgFreq10 > 999) avgFreq10=999;
  else if (avgFreq10 < 9) avgFreq10=99;
  if (minPeriod > 29999) minPeriod=29999;
  else if (minPeriod < 999) minPeriod=999;
  if (maxPeriod > 29999) maxPeriod=29999;
  else if (maxPeriod < 999) maxPeriod=999;
  


  char out[128];
  sprintf(out, "Pulse=%5dus Period=%6dus Freq=%6.2fHz Avg10s=%6.2fHz Min60s=%6dus Max60s=%6dus",
          (int) pulse, (int)period, freqHz, avgFreq10, (int)minPeriod, (int)maxPeriod);
  Serial.println(out);

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x10_tr);
  char buf[64];
  snprintf(buf, sizeof(buf), "Pin:%d %s", MEASURE_PIN, present?"YES":"NO");
  u8g2.drawStr(0, 8, buf);
  snprintf(buf, sizeof(buf), "Pulse:%dus", (int)pulse);
  u8g2.drawStr(0, 18, buf);
  snprintf(buf, sizeof(buf), "Prd:%dus Freq:%.2f", (int)period, freqHz);
  u8g2.drawStr(0, 28, buf);
  snprintf(buf, sizeof(buf), "Avg10s:%.2fHz", avgFreq10);
  u8g2.drawStr(0, 38, buf);
  u8g2.sendBuffer();

  delay(150);
}

