#include <Arduino.h>

// base class GxEPD2_GFX can be used to pass references or pointers to the
// display instance as parameter, uses ~1.2k more code enable or disable
// GxEPD2_GFX base class
#define ENABLE_GxEPD2_GFX 0

// uncomment next line to use class GFX of library GFX_Root instead of
// Adafruit_GFX
// #include <GFX.h>
// Note: if you use this with ENABLE_GxEPD2_GFX 1:
//       uncomment it in GxEPD2_GFX.h too, or add #include <GFX.h> before any
//       #include <GxEPD2_GFX.h>

// #include "Micro_Wx_Icons.h" // Weather Icons
#include "epaper_fonts.h"
#include "forecast_record.h"
#include "owm_credentials.h" // See 'owm_credentials' tab and enter your OWM API key and set the Wifi SSID and PASSWORD
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <ArduinoJson.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_BW.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>

// BATTERY
// To protect the battery upon LOW_BATTERY_VOLTAGE, the display will cease to
// update until battery is charged again. The ESP32 will deep-sleep (consuming
// < 11μA), waking briefly check the voltage at the corresponding interval (in
// minutes). Once the battery voltage has fallen to CRIT_LOW_BATTERY_VOLTAGE,
// the esp32 will hibernate and a manual press of the reset (RST) button to
// begin operating again.
const uint32_t WARN_BATTERY_VOLTAGE = 3535;                // (millivolts) ~20%
const uint32_t LOW_BATTERY_VOLTAGE = 3462;                 // (millivolts) ~10%
const uint32_t VERY_LOW_BATTERY_VOLTAGE = 3442;            // (millivolts)  ~8%
const uint32_t CRIT_LOW_BATTERY_VOLTAGE = 3404;            // (millivolts)  ~5%
const unsigned long LOW_BATTERY_SLEEP_INTERVAL = 30;       // (minutes)
const unsigned long VERY_LOW_BATTERY_SLEEP_INTERVAL = 120; // (minutes)
// Battery voltage calculations are based on a typical 3.7v LiPo.
const uint32_t MAX_BATTERY_VOLTAGE = 4200; // (millivolts)
const uint32_t MIN_BATTERY_VOLTAGE = 3000; // (millivolts)

// voltage divider pin
#define PIN_BAT_ADC 3

RTC_DATA_ATTR int lowbat = 0;

#define SCREEN_WIDTH 200
#define SCREEN_HEIGHT 200

enum alignment { LEFT, RIGHT, CENTER };

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp; // I2C
// Sensors I2C pin definition
#define S_SDA 1
#define S_SCL 0

Adafruit_Sensor *aht_humidity, *aht_temp;
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// Display pin definition
#define RST_PIN 8
#define DC_PIN 9
#define CS_PIN 10
#define BUSY_PIN 7
// SS = 5;
// MOSI = 4; - display DIN pin
// MISO = 3;
// SCK = 2; - display CLK pin

// adapt the constructor parameters to your wiring
GxEPD2_3C<GxEPD2_154_Z90c, GxEPD2_154_Z90c::HEIGHT> display(GxEPD2_154_Z90c(
    /*CS=*/CS_PIN, /*DC=*/DC_PIN, /*RST=*/RST_PIN,
    /*BUSY=*/BUSY_PIN)); // GDEH0154Z90 200x200, SSD1681, (HINK-E154A07-A1)

// ################ VARIABLES ###########################

bool LargeIcon = true, SmallIcon = false, RxWeather = false, RxForecast = false;
#define Large 10
#define Small 4
String Time_str, Date_str,
    rxtext; // strings to hold time and received weather data;
int StartTime, CurrentHour = 0, CurrentMin = 0, CurrentSec = 0;

// ################ PROGRAM VARIABLES and OBJECTS ################

#define max_readings 4

Forecast_record_type WxConditions[1];
Forecast_record_type WxForecast[max_readings];

#include "common.h"

#define autoscale_on true
#define autoscale_off false
#define barchart_on true
#define barchart_off false

float pressure_readings[max_readings] = {0};
float temperature_readings[max_readings] = {0};
float rain_readings[max_readings] = {0};

long SleepDuration =
    30; // Sleep time in minutes, aligned to minute boundary, so if 30 will
        // always update at 00 or 30 past the hour
int WakeupTime = 7; // Don't wakeup until after 07:00 to save battery power
int SleepTime = 23; // Sleep after (23+1) 00:00 to save battery power

// ################ FUNCTIONS ################

void InitialiseDisplay() {
  display.init(115200, true, 2, false);
  //// display.init(); for older Waveshare HAT's
  SPI.end();
  SPI.begin(SCK, MISO, MOSI, CS_PIN);
  display.setRotation(3);
  display.setTextSize(0);
  display.setFont(&DejaVu_Sans_Bold_11);
  display.setTextColor(GxEPD_BLACK);
  display.fillScreen(GxEPD_WHITE);
  display.setFullWindow();
}

void drawString(int x, int y, String text, alignment align) {
  int16_t x1,
      y1; // the bounds of x,y and w and h of the variable 'text' in pixels.
  uint16_t w, h;
  display.setTextWrap(false);
  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  if (align == RIGHT)
    x = x - w;
  if (align == CENTER)
    x = x - w / 2;
  display.setCursor(x, y + h);
  display.print(text);
}

/* Returns battery percentage, rounded to the nearest integer.
 * Takes a voltage in millivolts and uses a sigmoidal approximation to find an
 * approximation of the battery life percentage remaining.
 *
 * This function contains LGPLv3 code from
 * <https://github.com/rlogiacco/BatterySense>.
 *
 * Symmetric sigmoidal approximation
 * <https://www.desmos.com/calculator/7m9lu26vpy>
 *
 * c - c / (1 + k*x/v)^3
 */
uint32_t calcBatPercent(uint32_t v, uint32_t minv, uint32_t maxv) {
  // slow
  // uint32_t p = 110 - (110 / (1 + pow(1.468 * (v - minv)/(maxv - minv),
  // 6)));

  // steep
  // uint32_t p = 102 - (102 / (1 + pow(1.621 * (v - minv)/(maxv -
  // minv), 8.1)));

  // normal
  uint32_t p = 105 - (105 / (1 + pow(1.724 * (v - minv) / (maxv - minv), 5.5)));
  return p >= 100 ? 100 : p;
} // end calcBatPercent

void DisplayHeadingSection(uint32_t batVoltage) {
  drawString(2, 2, Time_str, LEFT);
  drawString(SCREEN_WIDTH - 2, 0, Date_str, RIGHT);
  uint32_t batPercent =
      calcBatPercent(batVoltage, MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE);
  if (batPercent < 20) {
    drawString(80, 0, "!" + String(batPercent) + "%", CENTER);
  } else {
    drawString(80, 0, String(batPercent) + "%", CENTER);
  }
  display.drawLine(0, 12, SCREEN_WIDTH, 12, GxEPD_RED);
}

void DisplayTempHumiSection(int x, int y) {
  display.drawRect(x, y, 100, 97, GxEPD_RED);

  sensors_event_t humidity, temp;
  aht.getEvent(&humidity,
               &temp); // populate temp and humidity objects with fresh data

  // display.drawRect(0, 0, 99, 99, GxEPD_RED);
  // display.setFont(&DejaVu_Sans_Bold_11);
  // display.setTextSize(1);
  // drawString(50, 15, "AHT20", CENTER);
  // drawString(50, 30, String(temp.temperature, 1) + "*C", CENTER);
  // drawString(50, 45, String(humidity.relative_humidity, 1) + "% RH", CENTER);

  /* Default settings from datasheet */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling
                                                     */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);

  // display.drawRect(99, 0, 199, 99, GxEPD_RED);
  // display.setFont(&DejaVu_Sans_Bold_11);
  // display.setTextSize(1);
  // drawString(150, 15, "BMP280", CENTER);
  // drawString(150, 30, String(temp_event.temperature, 1) + "*C", CENTER);
  // drawString(150, 45, String(pressure_event.pressure, 1) + "hPa", CENTER);

  display.setFont(&DSEG7_Classic_Bold_21);
  display.setTextSize(2);
  drawString(x + 5, y + 10, String(temp.temperature, 0) + "'",
             LEFT); // Show current Temperature from AHT
  display.setTextSize(1);
  drawString(x + 78, y + 30, (Units == "M" ? "C" : "F"),
             LEFT); // Add-in smaller Temperature unit
  drawString(x + 185, y + 10, String(WxConditions[0].Temperature, 0) + "'",
             RIGHT);
  drawString(x + 143, y + 38,
             String(WxConditions[0].High, 0) + "'/" +
                 String(WxConditions[0].Low, 0) + "'",
             CENTER); // Show forecast high and Low, in the font ' is a °
  display.setTextSize(2);
  display.setFont(&DejaVu_Sans_Bold_11);
  drawString(x + 148, y + 64, String(WxConditions[0].Humidity, 0) + "%RH",
             CENTER); // Show Humidity outdoor
  drawString(x + 48, y + 64, String(humidity.relative_humidity, 0) + "%RH",
             CENTER); // Show Humidity from AHT
  display.setTextSize(1);
  drawString(x + 48, y + 83, String(pressure_event.pressure, 1) + "hPa",
             CENTER); // Show Pressure from BMP
  // drawString(x + 155, y + 83, String(WxConditions[0].Humidity, 0) + "%RH",
  // CENTER); // Show Humidity outdoor
  drawString(x + 148, y + 83, String(WxConditions[0].Pressure, 1) + "hPa",
             CENTER); // Show pressure outdoor
}

void BeginSleep() {
  display.powerOff();
  long SleepTimer =
      (SleepDuration * 60 -
       ((CurrentMin % SleepDuration) * 60 +
        CurrentSec)); // Some ESP32 are too fast to maintain accurate time
  esp_sleep_enable_timer_wakeup((SleepTimer + 20) *
                                1000000LL); // Added +20 seconnds to cover ESP32
                                            // RTC timer source inaccuracies
#ifdef BUILTIN_LED
  pinMode(BUILTIN_LED,
          INPUT); // If it's On, turn it off and some boards use GPIO-5 for
                  // SPI-SS, which remains low after screen use
  digitalWrite(BUILTIN_LED, HIGH);
#endif

  esp_deep_sleep_start(); // Sleep for e.g. 30 minutes
}

uint8_t StartWiFi() {
  // Serial.print(F("\r\nConnecting to: "));
  // Serial.println(String(ssid));
  IPAddress dns(8, 8, 8, 8); // Google DNS
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long start = millis();
  uint8_t connectionStatus;
  bool AttemptConnection = true;
  while (AttemptConnection) {
    connectionStatus = WiFi.status();
    if (millis() > start + 15000) { // Wait 15-secs maximum
      AttemptConnection = false;
    }
    if (connectionStatus == WL_CONNECTED ||
        connectionStatus == WL_CONNECT_FAILED) {
      AttemptConnection = false;
    }
    delay(100);
  }
  // if (connectionStatus == WL_CONNECTED) {
  // Serial.println("WiFi connected at: " + WiFi.localIP().toString());
  //} else
  // Serial.println("WiFi connection *** FAILED ***");
  return connectionStatus;
}

void StopWiFi() {
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}

boolean UpdateLocalTime() {
  struct tm timeinfo;
  char output[30], day_output[30];
  while (!getLocalTime(&timeinfo,
                       5000)) { // Wait for 5-sec for time to synchronise
    // Serial.println(F("Failed to obtain time"));
    return false;
  }
  strftime(output, 30, "%H", &timeinfo);
  CurrentHour = timeinfo.tm_hour;
  CurrentMin = timeinfo.tm_min;
  CurrentSec = timeinfo.tm_sec;
  // See http://www.cplusplus.com/reference/ctime/strftime/
  // Serial.println(&timeinfo, "%a %b %d %Y   %H:%M:%S"); // Displays: Saturday,
  // June 24 2017 14:05:49
  // Serial.println(&timeinfo, "%H:%M:%S"); // Displays: 14:05:49
  if (Units == "M") {
    strftime(day_output, 30, "%d-%b-%y", &timeinfo); // Displays: 24-Jun-17
    strftime(output, 30, "%H:%M", &timeinfo);        // Creates: '14:05'
  } else {
    strftime(day_output, 30, "%b-%d-%y", &timeinfo); // Creates: Jun-24-17
    strftime(output, 30, "%I:%M%p", &timeinfo);      // Creates: '2:05pm'
  }
  Date_str = day_output;
  Time_str = output;
  return true;
}

boolean SetupTime() {
  configTime(0, 0, "0.ua.pool.ntp.org", "time.nist.gov");
  setenv("TZ", Timezone, 1);
  tzset(); // Set the TZ environment variable
  delay(100);
  bool TimeStatus = UpdateLocalTime();
  return TimeStatus;
}

// #########################################################################################
//  Symbols are drawn on a relative 10x10grid and 1 scale unit = 1 drawing
//  unit
void addcloud(int x, int y, int scale, int linesize) {
  // Draw cloud outer
  display.fillCircle(x - scale * 3, y, scale,
                     GxEPD_BLACK); // Left most circle
  display.fillCircle(x + scale * 3, y, scale,
                     GxEPD_BLACK); // Right most circle
  display.fillCircle(x - scale, y - scale, scale * 1.4,
                     GxEPD_BLACK); // left middle upper circle
  display.fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75,
                     GxEPD_BLACK); // Right middle upper circle
  display.fillRect(x - scale * 3 - 1, y - scale, scale * 6, scale * 2 + 1,
                   GxEPD_BLACK); // Upper and lower lines
  // Clear cloud inner
  display.fillCircle(x - scale * 3, y, scale - linesize,
                     GxEPD_WHITE); // Clear left most circle
  display.fillCircle(x + scale * 3, y, scale - linesize,
                     GxEPD_WHITE); // Clear right most circle
  display.fillCircle(x - scale, y - scale, scale * 1.4 - linesize,
                     GxEPD_WHITE); // left middle upper circle
  display.fillCircle(x + scale * 1.5, y - scale * 1.3, scale * 1.75 - linesize,
                     GxEPD_WHITE); // Right middle upper circle
  display.fillRect(x - scale * 3 + 2, y - scale + linesize - 1, scale * 5.9,
                   scale * 2 - linesize * 2 + 2,
                   GxEPD_WHITE); // Upper and lower lines
}
// #########################################################################################
void addrain(int x, int y, int scale) {
  y = y + scale / 2;
  for (int i = 0; i < 6; i++) {
    display.drawLine(x - scale * 4 + scale * i * 1.3 + 0, y + scale * 1.9,
                     x - scale * 3.5 + scale * i * 1.3 + 0, y + scale,
                     GxEPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 4 + scale * i * 1.3 + 1, y + scale * 1.9,
                       x - scale * 3.5 + scale * i * 1.3 + 1, y + scale,
                       GxEPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.3 + 2, y + scale * 1.9,
                       x - scale * 3.5 + scale * i * 1.3 + 2, y + scale,
                       GxEPD_BLACK);
    }
  }
}
// #########################################################################################
void addsnow(int x, int y, int scale) {
  int dxo, dyo, dxi, dyi;
  for (int flakes = 0; flakes < 5; flakes++) {
    for (int i = 0; i < 360; i = i + 45) {
      dxo = 0.5 * scale * cos((i - 90) * 3.14 / 180);
      dxi = dxo * 0.1;
      dyo = 0.5 * scale * sin((i - 90) * 3.14 / 180);
      dyi = dyo * 0.1;
      display.drawLine(dxo + x + 0 + flakes * 1.5 * scale - scale * 3,
                       dyo + y + scale * 2,
                       dxi + x + 0 + flakes * 1.5 * scale - scale * 3,
                       dyi + y + scale * 2, GxEPD_BLACK);
    }
  }
}
// #########################################################################################
void addtstorm(int x, int y, int scale) {
  y = y + scale / 2;
  for (int i = 0; i < 5; i++) {
    display.drawLine(x - scale * 4 + scale * i * 1.5 + 0, y + scale * 1.5,
                     x - scale * 3.5 + scale * i * 1.5 + 0, y + scale,
                     GxEPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 4 + scale * i * 1.5 + 1, y + scale * 1.5,
                       x - scale * 3.5 + scale * i * 1.5 + 1, y + scale,
                       GxEPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.5 + 2, y + scale * 1.5,
                       x - scale * 3.5 + scale * i * 1.5 + 2, y + scale,
                       GxEPD_BLACK);
    }
    display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 0,
                     x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 0,
                     GxEPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 1,
                       x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 1,
                       GxEPD_BLACK);
      display.drawLine(x - scale * 4 + scale * i * 1.5, y + scale * 1.5 + 2,
                       x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5 + 2,
                       GxEPD_BLACK);
    }
    display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 0, y + scale * 2.5,
                     x - scale * 3 + scale * i * 1.5 + 0, y + scale * 1.5,
                     GxEPD_BLACK);
    if (scale != Small) {
      display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 1, y + scale * 2.5,
                       x - scale * 3 + scale * i * 1.5 + 1, y + scale * 1.5,
                       GxEPD_BLACK);
      display.drawLine(x - scale * 3.5 + scale * i * 1.4 + 2, y + scale * 2.5,
                       x - scale * 3 + scale * i * 1.5 + 2, y + scale * 1.5,
                       GxEPD_BLACK);
    }
  }
}
// #########################################################################################
void addsun(int x, int y, int scale) {
  int linesize = 1;
  int dxo, dyo, dxi, dyi;
  display.fillCircle(x, y, scale, GxEPD_BLACK);
  display.fillCircle(x, y, scale - linesize, GxEPD_WHITE);
  for (float i = 0; i < 360; i = i + 45) {
    dxo = 2.2 * scale * cos((i - 90) * 3.14 / 180);
    dxi = dxo * 0.6;
    dyo = 2.2 * scale * sin((i - 90) * 3.14 / 180);
    dyi = dyo * 0.6;
    if (i == 0 || i == 180) {
      display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y, GxEPD_BLACK);
      if (scale != Small) {
        display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y,
                         GxEPD_BLACK);
        display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y,
                         GxEPD_BLACK);
      }
    }
    if (i == 90 || i == 270) {
      display.drawLine(dxo + x, dyo + y - 1, dxi + x, dyi + y - 1, GxEPD_BLACK);
      if (scale != Small) {
        display.drawLine(dxo + x, dyo + y + 0, dxi + x, dyi + y + 0,
                         GxEPD_BLACK);
        display.drawLine(dxo + x, dyo + y + 1, dxi + x, dyi + y + 1,
                         GxEPD_BLACK);
      }
    }
    if (i == 45 || i == 135 || i == 225 || i == 315) {
      display.drawLine(dxo + x - 1, dyo + y, dxi + x - 1, dyi + y, GxEPD_BLACK);
      if (scale != Small) {
        display.drawLine(dxo + x + 0, dyo + y, dxi + x + 0, dyi + y,
                         GxEPD_BLACK);
        display.drawLine(dxo + x + 1, dyo + y, dxi + x + 1, dyi + y,
                         GxEPD_BLACK);
      }
    }
  }
}
// #########################################################################################
void addmoon(int x, int y, int scale) {
  display.fillCircle(x - 20, y - 15, scale, GxEPD_BLACK);
  display.fillCircle(x - 15, y - 15, scale * 1.6, GxEPD_WHITE);
}
// #########################################################################################
void addfog(int x, int y, int scale, int linesize) {
  y -= 10;
  linesize = 1;
  for (int i = 0; i < 6; i++) {
    display.fillRect(x - scale * 3, y + scale * 1.5, scale * 6, linesize,
                     GxEPD_BLACK);
    display.fillRect(x - scale * 3, y + scale * 2.0, scale * 6, linesize,
                     GxEPD_BLACK);
    display.fillRect(x - scale * 3, y + scale * 2.7, scale * 6, linesize,
                     GxEPD_BLACK);
  }
}
// #########################################################################################
void MostlyCloudy(int x, int y, bool IconSize, String IconName) {
  int scale = Large, linesize = 3;
  if (IconSize == SmallIcon) {
    scale = Small;
    linesize = 1;
  }
  if (IconName.endsWith("n"))
    addmoon(x, y, scale);
  addcloud(x, y, scale, linesize);
  addsun(x - scale * 1.8, y - scale * 1.8, scale);
  addcloud(x, y, scale, linesize);
}

void MostlySunny(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n"))
    addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addsun(x - scale * 1.8, y - scale * 1.8 + offset, scale);
}
// #########################################################################################
void Rain(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  if (LargeSize) {
    scale = Large;
    offset = 12;
  }
  int linesize = 3;
  if (scale == Small)
    linesize = 1;
  if (IconName.endsWith("n"))
    addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addrain(x, y + offset, scale);
}
// #########################################################################################
void Cloudy(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n"))
    addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
}
// #########################################################################################
void Sunny(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  if (IconName.endsWith("n"))
    addmoon(x, y + offset, scale);
  scale = scale * 1.5;
  addsun(x, y + offset, scale);
}
// #########################################################################################
void ExpectRain(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n"))
    addmoon(x, y + offset, scale);
  addsun(x - scale * 1.8, y - scale * 1.8 + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addrain(x, y + offset, scale);
}
// #########################################################################################
void ChanceRain(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n"))
    addmoon(x, y + offset, scale);
  addsun(x - scale * 1.8, y - scale * 1.8 + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addrain(x, y + offset, scale);
}
// #########################################################################################
void Tstorms(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n"))
    addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addtstorm(x, y + offset, scale);
}
// #########################################################################################
void Snow(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n"))
    addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addsnow(x, y + offset, scale);
}
// #########################################################################################
void Fog(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n"))
    addmoon(x, y + offset, scale);
  addcloud(x, y + offset, scale, linesize);
  addfog(x, y + offset, scale, linesize);
}
// #########################################################################################
void Haze(int x, int y, bool LargeSize, String IconName) {
  int scale = Small, offset = 0;
  int linesize = 1;
  if (IconName.endsWith("n"))
    addmoon(x, y + offset, scale);
  addsun(x, y + offset, scale * 1.4);
  addfog(x, y + offset, scale * 1.4, linesize);
}
// #########################################################################################
void Nodata(int x, int y, bool LargeSize) {
  int scale = Small, offset = 0;
  if (LargeSize) {
    scale = Large;
    offset = 7;
  }
  if (scale == Large)
    display.setFont(&FreeMonoBold12pt7b);
  else
    display.setFont(&DejaVu_Sans_Bold_11);
  drawString(x - 20, y - 10 + offset, "N/A", LEFT);
}

void DisplayWxIcon(int x, int y, String IconName, bool LargeSize) {
  Serial.println(IconName);
  if (IconName == "01d" || IconName == "01n")
    Sunny(x, y, LargeSize, IconName);
  else if (IconName == "02d" || IconName == "02n")
    MostlySunny(x, y, LargeSize, IconName);
  else if (IconName == "03d" || IconName == "03n")
    Cloudy(x, y, LargeSize, IconName);
  else if (IconName == "04d" || IconName == "04n")
    MostlyCloudy(x, y, LargeSize, IconName);
  else if (IconName == "09d" || IconName == "09n")
    ChanceRain(x, y, LargeSize, IconName);
  else if (IconName == "10d" || IconName == "10n")
    Rain(x, y, LargeSize, IconName);
  else if (IconName == "11d" || IconName == "11n")
    Tstorms(x, y, LargeSize, IconName);
  else if (IconName == "13d" || IconName == "13n")
    Snow(x, y, LargeSize, IconName);
  else if (IconName == "50d")
    Haze(x, y, LargeSize, IconName);
  else if (IconName == "50n")
    Fog(x, y, LargeSize, IconName);
  else
    Nodata(x, y, LargeSize);
}

void DisplayForecastWeather(int x, int y, int offset, int index) {
  display.drawRect(x, y, offset, 65, GxEPD_RED);
  display.drawLine(x, y + 13, x + offset, y + 13, GxEPD_RED);
  DisplayWxIcon(x + offset / 2 + 1, y + 35, WxForecast[index].Icon, SmallIcon);
  drawString(
      x + offset / 2, y + 3,
      String(ConvertUnixTime(WxForecast[index].Dt + WxConditions[0].Timezone)
                 .substring(0, 5)),
      CENTER);
  drawString(x + offset / 2, y + 50,
             String(WxForecast[index].High, 0) + "/" +
                 String(WxForecast[index].Low, 0),
             CENTER);
}

String WindDegToDirection(float winddirection) {
  int dir = int((winddirection / 22.5) + 0.5);
  String Ord_direction[16] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                              "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  return Ord_direction[(dir % 16)];
}

void drawStringMaxWidth(int x, int y, int text_width, String text,
                        alignment align) {
  int16_t x1,
      y1; // the bounds of x,y and w and h of the variable 'text' in pixels.
  uint16_t w, h;
  if (text.length() > text_width * 2)
    text = text.substring(0, text_width *
                                 2); // Truncate if too long for 2 rows of text
  display.getTextBounds(text, x, y, &x1, &y1, &w, &h);
  if (align == RIGHT)
    x = x - w;
  if (align == CENTER)
    x = x - w / 2;
  display.setCursor(x, y + h);
  display.println(text.substring(0, text_width));
  if (text.length() > text_width) {
    display.setCursor(x, y + h * 2);
    display.println(text.substring(text_width));
  }
}

void DisplayMainWeatherSection(int x, int y) {
  display.drawRect(x, y - 4, SCREEN_WIDTH, 28, GxEPD_RED);
  String Wx_Description1 = WxConditions[0].Forecast0;
  display.setFont(&DejaVu_Sans_Bold_11);
  String Wx_Description2 = WindDegToDirection(WxConditions[0].Winddir) +
                           " wind, " + String(WxConditions[0].Windspeed, 1) +
                           (Units == "M" ? "m/s" : "mph");
  drawStringMaxWidth(x + 2, y - 2, 27, TitleCase(Wx_Description1), LEFT);
  drawStringMaxWidth(x + 2, y + 10, 27, TitleCase(Wx_Description2), LEFT);
}

void DisplayForecastSection(int x, int y) {
  int offset = 50;
  DisplayForecastWeather(x + offset * 0, y, offset, 0);
  DisplayForecastWeather(x + offset * 1, y, offset, 1);
  DisplayForecastWeather(x + offset * 2, y, offset, 2);
  DisplayForecastWeather(x + offset * 3, y, offset, 3);
  int r = 0;
  do {
    if (Units == "I")
      pressure_readings[r] = WxForecast[r].Pressure * 0.02953;
    else
      pressure_readings[r] = WxForecast[r].Pressure;
    temperature_readings[r] = WxForecast[r].Temperature;
    if (Units == "I")
      rain_readings[r] = WxForecast[r].Rainfall * 0.0393701;
    else
      rain_readings[r] = WxForecast[r].Rainfall;
    r++;
  } while (r < max_readings);
}

/* Returns battery voltage in millivolts (mv).*/
uint32_t readBatteryVoltage() {
  esp_adc_cal_characteristics_t adc_chars;
  // __attribute__((unused)) disables compiler warnings about this variable
  // being unused (Clang, GCC) which is the case when DEBUG_LEVEL == 0.
  esp_adc_cal_value_t val_type __attribute__((unused));
  adc_power_acquire();
  uint16_t adc_val = analogRead(PIN_BAT_ADC);
  adc_power_release();

  // We will use the eFuse ADC calibration bits, to get accurate voltage
  // readings. The DFRobot FireBeetle Esp32-E V1.0's ADC is 12 bit, and uses
  // 11db attenuation, which gives it a measurable input voltage range of
  // 150mV to 2450mV.
  val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_11db,
                                      ADC_WIDTH_BIT_12, 1100, &adc_chars);

  // debug on
  // display.setTextSize(1);
  // display.setFont(&DejaVu_Sans_Bold_11);
  // if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
  //  drawString(100, 50, "ADC Cal eFuse Vref", CENTER);
  //} else if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
  //  drawString(100, 50, "ADC Cal Two Point", CENTER);
  //} else {
  //  drawString(100, 50, "ADC Cal Default", CENTER);
  //}
  // debug off

  uint32_t batteryVoltage = esp_adc_cal_raw_to_voltage(adc_val, &adc_chars);
  // DFRobot FireBeetle Esp32-E V1.0 voltage divider (1M+1M), so readings are
  // multiplied by 2.
  batteryVoltage *= 2;

  // debug on
  // drawString(100, 70, String(batteryVoltage) + " mV", CENTER);
  // uint32_t batPercent =
  //    calcBatPercent(batteryVoltage, MIN_BATTERY_VOLTAGE,
  //    MAX_BATTERY_VOLTAGE);
  // drawString(100, 90, String(batPercent) + "%", CENTER);
  // display.display(false); // Full screen update mode
  // esp_sleep_enable_timer_wakeup(1 * 60ULL * 1000000ULL);
  // esp_deep_sleep_start();
  // debug off

  return batteryVoltage;
} // end readBatteryVoltage

// ================================= main code
// ==================================

void setup() {
  // put your setup code here, to run once:
  StartTime = millis();

  // Read battery
  uint32_t batteryVoltage = readBatteryVoltage();
  if (batteryVoltage <= LOW_BATTERY_VOLTAGE) {
    if (lowbat == 0) { // battery is now low for the first time
      lowbat = 1;
      InitialiseDisplay();
      display.setTextSize(2);
      display.setFont(&DejaVu_Sans_Bold_11);
      drawString(100, 55, "!", CENTER);
      drawString(100, 75, "Low battery", CENTER);
      drawString(100, 110, "Please charge", CENTER);
      display.display(false); // Full screen update mode
    }
    if (batteryVoltage <= CRIT_LOW_BATTERY_VOLTAGE) { // critically low battery
      // don't set esp_sleep_enable_timer_wakeup();
      // We won't wake up again until someone manually presses the RST button.
    } else if (batteryVoltage <= VERY_LOW_BATTERY_VOLTAGE) { // very low battery
      esp_sleep_enable_timer_wakeup(VERY_LOW_BATTERY_SLEEP_INTERVAL * 60ULL *
                                    1000000ULL);
    } else { // low battery
      esp_sleep_enable_timer_wakeup(LOW_BATTERY_SLEEP_INTERVAL * 60ULL *
                                    1000000ULL);
    }
    esp_deep_sleep_start();
  }

  lowbat = 0;
  Wire.begin(S_SDA, S_SCL);

  if (!aht.begin()) {
    display.setTextSize(2);
    display.setFont(&DejaVu_Sans_Bold_11);
    drawString(100, 100, "AHT not found", CENTER);
    drawString(100, 120, "Check wirig", CENTER);
    esp_deep_sleep_start();
  }

  if (!bmp.begin()) {
    display.setTextSize(2);
    display.setFont(&DejaVu_Sans_Bold_11);
    drawString(100, 100, "BMP not found", CENTER);
    drawString(100, 120, "Check wirig", CENTER);
    esp_deep_sleep_start();
  }

  if (StartWiFi() == WL_CONNECTED && SetupTime() == true) {
    if ((CurrentHour >= WakeupTime && CurrentHour <= SleepTime)) {
      InitialiseDisplay();
      byte Attempts = 1;
      WiFiClient client; // wifi client object
      while ((RxWeather == false || RxForecast == false) &&
             Attempts <= 2) { // Try up-to twice for Weather and Forecast data
        if (RxWeather == false)
          RxWeather = obtain_wx_data(client, "weather");
        if (RxForecast == false)
          RxForecast = obtain_wx_data(client, "forecast");
        Attempts++;
      }

      if (RxWeather ||
          RxForecast) { // If received either Weather or Forecast data then
                        // proceed, report later if either failed
        StopWiFi();     // Reduces power consumption
        DisplayHeadingSection(batteryVoltage); // Top line of the display
        DisplayTempHumiSection(0, 12); // Current temperature with Max/Min
        display.drawRect(99, 12, 101, 97, GxEPD_RED);
        DisplayWxIcon(126, 34, WxConditions[0].Icon, SmallIcon);
        DisplayMainWeatherSection(0, 112); // Weather forecast text
        DisplayForecastSection(0, 135);    // 3hr interval forecast boxes
        display.display(false);            // Full screen update mode
      }
    }
    BeginSleep();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
