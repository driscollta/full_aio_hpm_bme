
#include "hpma115S0.h"
#include "AQINowCast.h"

#include "config.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// store the boot count in the ESP32 RTC slow memory to retain it during deep sleep
RTC_DATA_ATTR int bootCount = 0;
//index to the next place to store HPM data in the ESP32 RTC slow memory
RTC_DATA_ATTR int current_pm_array_index = 0;
//array to store last 12 hours of pm2.5 HPM data in the ESP32 RTC slow memory
RTC_DATA_ATTR unsigned int pm25_array[12];
//array to store last 12 hours of pm10 HPM data in the ESP32 RTC slow memory
RTC_DATA_ATTR unsigned int pm10_array[12];

// for timing-out Adafruit IO connection
const unsigned long AIO_CONNECT_TIMEOUT = 5000;
const bool ARE_DEBUGGING = false;

// how long to sleep (in micro-seconds)
// maximum is 4294 seconds (71.5 minutes) from max value of unsigned long.
const unsigned long SLEEP_LENGTH = SIX_MINUTES;
// calculated particle measurement values
unsigned int PM25, PM10;
// 12-hour averaged pm2.5 and pm10
unsigned int AvgPM25, AvgPM10;
// we'll take num_measurements particle measurements from the HPM and average
int num_measurements = 5;
unsigned int PM25measurements[5], PM10measurements[5];

void setup() {
  Serial.begin(9600);
  Serial1.begin(9600, SERIAL_8N1, RXD2, TXD2);
  // set the digital pin as output:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SHDNPin, OUTPUT);
  pinMode(BMEPin, OUTPUT);
  prnt_debug_text("");
  prnt_debug_text("Waking...");
  // set the LED, SHDN and BME pins
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(SHDNPin, HIGH);
  digitalWrite(BMEPin, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  prnt_debug_text("Starting...");
  //Increment boot number and print it every reboot
  ++bootCount;
  prnt_debug_text("Boot number: " + String(bootCount));
  bool hpm_status = false;
  // take hpm measurements every fifth time we turn-on the board
  if (bootCount % 5 == 0) {
    hpm_status = setupHPM();
  }
  //turn-off the Pololu boost
  digitalWrite(SHDNPin, LOW);
  // start the BME280 Pressure, temperature, humidity sensor and take a measurement
  bool bme_status = setupBME();
  // connect us to Adafruit IO
  // only wait AIO_CONNECT_TIMEOUT milliseconds to connect to Adafruit IO. Usually 2 tries are enough to connect.
  // It seems that sometimes every-other attempt fails
  bool AIO_is_connected = connect_AIO();
  if (! AIO_is_connected) {
    AIO_is_connected = connect_AIO();
  }
  if (! AIO_is_connected) {
    AIO_is_connected = connect_AIO();
  }
  // grab the 2.5 um particle measurement feed
  AdafruitIO_Feed *pm2_5 = io.feed("PM2_5");
  // grab the 10 um particle measurement feed
  AdafruitIO_Feed *pm10 = io.feed("PM10");
  // grab the temperature feed
  AdafruitIO_Feed *temperature = io.feed("temperature");
  // grab the humidity feed
  AdafruitIO_Feed *humidity = io.feed("humidity");
  // grab the pressure feed
  AdafruitIO_Feed *pressure = io.feed("barometer");
  // grab the battery feed
  AdafruitIO_Feed *battery = io.feed("battery");
  // grab the now cast AQI 2.5 um feed
  AdafruitIO_Feed *aqi2_5 = io.feed("AQI_pm2_5");
  // grab the now cast AQI 10 um feed
  AdafruitIO_Feed *aqi10 = io.feed("AQI_pm10");
  // send battery level to AIO
  if (AIO_is_connected) {
    battery->save(battery_level());
    io.run();
  }
  // if BME280 started-up, read t, h, p and send to Adafruit IO service
  if (bme_status && AIO_is_connected) {
    // send temperature to AIO
    float t = get_temp();
    if (t > MIN_TEMPERATURE && t < MAX_TEMPERATURE) {
      temperature->save(t);
      io.run();
    }
    // send humidity level to AIO
    humidity->save(get_humidity());
    io.run();
    // send pressure level to AIO
    float p = get_pressure();
    if (p > MIN_PRESSURE && p < MAX_PRESSURE) {
      pressure->save(p);
      io.run();
    }
  }
  if (hpm_status && AIO_is_connected) {
    pm2_5->save(PM25);
    io.run();
    pm10->save(PM10);
    io.run();
    // calculate AQI, send to AIO
    //todo save in ESP32 RTC memory
    aqi2_5->save(getNowCast25());
    io.run();
    aqi10->save(getNowCast10());
    io.run();
  }
  // turn-off the BME280 power
  digitalWrite(BMEPin, LOW);

  // Configure the wake up source timer with delay in uSeconds
  esp_sleep_enable_timer_wakeup(SLEEP_LENGTH * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
}

// noop
void loop() {}

void doAverages() {
  PM25 = 0;
  PM10 = 0;
  byte _array_size = (sizeof(PM25measurements) / sizeof(PM25measurements[0]));
  for (byte i = 0; i < _array_size; i++) {
    PM25 += PM25measurements[i];
    PM10 += PM10measurements[i];
  }
  PM25 /= _array_size;
  PM10 /= _array_size;
}

void do12HourAverages(){
  // find range
  // find scaled rate of change
  // find weight factor {0.5 -> 1}
  // weighted sum of past 12 hour concentrations 
  // (multiply by power of weight factor)
  // divide by weighted weight factor
  float _max25 = 1.;
  float _min25 = 65535.;
  float _max10 = 1.;
  float _min10 = 65535.;
  byte _array_size = (sizeof(pm25_array) / sizeof(pm25_array[0]));

  for (byte i = 0; i < _array_size; i++) {
    // document the current 12-hour pm readings
    Serial.print("  pm25_["); Serial.print(i); Serial.print("] "); 	Serial.print(pm25_array[i]);
    Serial.print("  pm10_["); Serial.print(i); Serial.print("] "); 	Serial.println(pm10_array[i]);

    if (pm25_array[i] > _max25){
      _max25 = pm25_array[i];
    }
    if (pm25_array[i] < _min25){
      _min25 = pm25_array[i];
    }
    if (pm10_array[i] > _max10){
      _max10 = pm10_array[i];
    }
    if (pm10_array[i] < _min10){
      _min10 = pm10_array[i];
    }
  }
  float _weight25 = 1 - (_max25 - _min25) / _max25;
  if (_weight25 < 0.5) {
    _weight25 = 0.5;
  }
  float _weight10 = 1 - (_max10 - _min10) / _max10;
  if (_weight10 < 0.5) {
    _weight10 = 0.5;
  }
  Serial.print("weight10: "); Serial.print(_weight10); Serial.print(" weight25: "); Serial.println(_weight25);
  float _multiplier25 = 1.0;
  float _multiplier10 = 1.0;
  float _numerator25 = 0.0;
  float _numerator10 = 0.0;
  float _divisor25 = 0.0;
  float _divisor10 = 0.0;
  byte _real_array_index = current_pm_array_index;
  for (byte i = 0; i < _array_size; i++){
    // document the current 12-hour pm readings
    // don't include "missing" measurements where pm values are 0
    if (pm25_array[_real_array_index] > 0) {
      _divisor25 += _multiplier25;
    }
    if (pm10_array[_real_array_index] > 0) {
      _divisor10 += _multiplier10;
    }
    Serial.print("real_array_index: "); Serial.println(_real_array_index);
    _numerator25 += pm25_array[_real_array_index] * _multiplier25;
    _numerator10 += pm10_array[_real_array_index] * _multiplier10;
    // we haven't yet incremented current_pm_array_index, so this is the most current reading
    _real_array_index--;
    // roll-over _real_array_index
    if (_real_array_index == 255) {
      _real_array_index = _array_size - 1;
    }
    _multiplier25 *= _weight25;
    _multiplier10 *= _weight10;
    Serial.print("multiplier10: "); Serial.print(_multiplier10); Serial.print(" multiplier25: "); Serial.println(_multiplier25);
    Serial.print("numerator10: "); Serial.print(_numerator10); Serial.print(" numerator25: "); Serial.println(_numerator25);
    Serial.print("divisor10: "); Serial.print(_divisor10); Serial.print(" divisor25: "); Serial.println(_divisor25);
  }
  // truncate according to airnow formula: 0.1 μg/m3 for PM2.5, 1 μg/m3 for PM10
  AvgPM25 = floorf((_numerator25 / _divisor25) * 10.) / 10.;
  AvgPM10 = floorf(_numerator10 / _divisor10);
  if (Serial && ARE_DEBUGGING) {
    String text = "12 hr average pm2.5 " + String(AvgPM25) + " ugm/m^3";
    Serial.println(" "); prnt_debug_text(text);
    text = "12 hr average pm10 " + String(AvgPM10) + " ugm/m^3";
    Serial.println(" "); prnt_debug_text(text);
  }
}

int getNowCast25() {
  int nowCastAQI = aqi.calcNowCastAQI(aqi.AQI_TYPE_PM2_5, AvgPM25);
  if (nowCastAQI > 500) {
    nowCastAQI = 500;
  }
  if (Serial && ARE_DEBUGGING) {
    String text = "12 hr nowCastAQI 2.5 " + String(nowCastAQI);
    Serial.println(" "); prnt_debug_text(text);
  }
  return nowCastAQI;
}

int getNowCast10(){
  int nowCastAQI = aqi.calcNowCastAQI(aqi.AQI_TYPE_PM10, AvgPM10);
  if (nowCastAQI > 500) {
    nowCastAQI = 500;
  }
  if (Serial && ARE_DEBUGGING) {
    String text = "12 hr nowCastAQI 10 " + String(nowCastAQI);
    Serial.println(" "); prnt_debug_text(text);
  }
  return nowCastAQI;
}

void logPMMeasurement() {
  if (Serial && ARE_DEBUGGING) {
    String text = "pm2.5 " + String(PM25) + " ugm/m^3";
    Serial.println(" "); prnt_debug_text(text);
    text = "pm10 " + String(PM10) + " ugm/m^3";
    Serial.println(" "); prnt_debug_text(text);
  }
}

bool setupBME() {
  bool bme_status = bme.begin();
  if (bme_status) {
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, //temperature
                    Adafruit_BME280::SAMPLING_X1, //pressure
                    Adafruit_BME280::SAMPLING_X1, //humidity
                    Adafruit_BME280::FILTER_OFF );
    delay(100);
    bme.takeForcedMeasurement();
  }
  return bme_status;
}
bool setupHPM() {
      // talk to the HPM over Serial1 UART and read multiple measurements
    hpma115S0.Init();
    delay(1000);
    hpma115S0.StartParticleMeasurement();
    // have to wait after starting particle measurement before reading!!! At least 0.5 seconds!!!
    delay(HPM_DELAY);
    int _num_valid_pms = 0;
    for (int _index = 0; _index < num_measurements; _index++) {
      if (hpma115S0.ReadParticleMeasurement(&PM25, &PM10)) {
        _num_valid_pms++;
        PM25measurements[_index] = PM25;
        PM10measurements[_index] = PM10;
      }
      // wait HPM_DELAY milliseconds until the next measurement
      if (_index < num_measurements - 1) {
        delay(HPM_DELAY);
      }
    }
    // save HPM data in RTC memory
    if (_num_valid_pms > 0) {
      doAverages();
      pm25_array[current_pm_array_index] = PM25;
      pm10_array[current_pm_array_index] = PM10;
    } else {
      pm25_array[current_pm_array_index] = 0;
      pm10_array[current_pm_array_index] = 0;
    }
    do12HourAverages();
    // increment index, but keep it < 12. If it is 10 it will be 11. If it is 11 it will be 0
    if (current_pm_array_index < 11) {
      current_pm_array_index++;
    } else {
      current_pm_array_index = 0;
    }
    delay(50);
    hpma115S0.StopParticleMeasurement();
    // doesn't matter whether we got PM readings, just say we tried and upload to AIO
    return true;
}
float battery_level() {
  // read the battery level from the ESP32 analog in pin.
  // when we know the adc range for the highest and lowest battery voltage, convert to percent
  float level = analogRead(A13) * BATTERY_CONVERSION;
  // constrain level to 0 - 100% range
  //level = constrain(level, 3.1, 4.2);
  // convert battery level to percent
  //level = map(level, minBattADC, maxBattADC, 0, 100);
  if (Serial && ARE_DEBUGGING) {
    Serial.print(F("Battery level: ")); Serial.println(level);
  }
  return level;
}

float get_temp() {
  // already took forced measurements, just read
  float t = convertC2F(bme.readTemperature());
  if (Serial && ARE_DEBUGGING) {
    Serial.print(F("Temperature: ")); Serial.print(t); Serial.println("*F");
  }
  return t;
}

float get_pressure() {
  // already took forced measurements, just read
  float p = convertP2inHg(bme.readPressure());
  if (Serial && ARE_DEBUGGING) {
    Serial.print(F("Pressure: ")); Serial.print(p); Serial.println("in-Hg");
  }
  return p;
}

float get_humidity() {
  // already took forced measurements, just read
  float h = bme.readHumidity();
  String text = "Humidity: " + String(h) + "%";
  prnt_debug_text(text);
  return h;
}

float convertC2F(float temp) {
  return temp * 1.8 + 32;
}

float convertP2inHg(float pressure) {
  return (pressure / 3386.39F);
}

bool is_timed_out(unsigned long startTime_ms, unsigned long time_out_time) {
  unsigned long delta_time = millis() - startTime_ms;
  if (delta_time < 0) {
    // rolled-over since we started! startTime_ms is really big, millis() is very small.
    // delta_time is negative, have to add the maximum unsigned long
    delta_time += MAX_UNSIGNED_LONG;
  }
  return (delta_time > time_out_time);
}

void prnt_debug_text(String text) {
  if (Serial && ARE_DEBUGGING) {
    Serial.println(text);
  }
}

bool connect_AIO() {
  prnt_debug_text("Connecting to Adafruit IO...");
  unsigned long startTime_ms = millis();
  // io is the AdafruitIO_WiFi
  io.connect();
  // wait for a connection; have a time-out if this doesn't connect
  while (io.status() < AIO_CONNECTED && !is_timed_out(startTime_ms, AIO_CONNECT_TIMEOUT)) {
    if (Serial && ARE_DEBUGGING) {
      Serial.print(" .");
    }
    delay(200);
  }
  // we are connected or timed-out
  if (Serial && ARE_DEBUGGING) {
    Serial.println(); Serial.println(io.statusText());
  }
  return (io.status() == AIO_CONNECTED);
}
