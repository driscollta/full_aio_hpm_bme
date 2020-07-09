/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#include <AdafruitIO_WiFi.h>
#include <Adafruit_BME280.h>

#define IO_USERNAME  "xxxxxx"
#define IO_KEY       "xxxxxx"

/******************************* WIFI **************************************/

// the AdafruitIO_WiFi client will work with the following boards:
//   - HUZZAH ESP8266 Breakout -> https://www.adafruit.com/products/2471
//   - Feather HUZZAH ESP8266 -> https://www.adafruit.com/products/2821
//   - Feather M0 WiFi -> https://www.adafruit.com/products/3010
//   - Feather WICED -> https://www.adafruit.com/products/3056

#define WIFI_SSID       "xxxxxx"
#define WIFI_PASS       "xxxxxx"

#define uS_TO_S_FACTOR 1000000ULL
#define MAX_UNSIGNED_LONG 4294967295ULL
#define SIX_MINUTES 360
#define ONE_MINUTE 60
#define HPM_READ_RESP 0
#define HPM_START_STOP_RESP 1
#define MIN_PRESSURE 28.0
#define MAX_PRESSURE 32.0
#define MIN_TEMPERATURE 10.0
#define MAX_TEMPERATURE 132.0
#define BATTERY_CONVERSION .001718F
// the pins for the UART on the ESP32 Huzzah feather board
#define RXD2 16
#define TXD2 17
// delay for next HPM read (min is <6000 from data sheet)
#define HPM_DELAY 6000

#define  SHDNPin  21 //use pin 21 to control the BME power. This pin has a pull-down resistor built into it
#define  BMEPin  4 //use pin 4 to control the Pololu U1V11F5 5 v step-up. This pin has a pull-down resistor built into it



AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);
AQINowCast aqi;
Adafruit_BME280 bme; // I2C
//Create an instance of the hpma115S0 library
HPMA115S0 hpma115S0(Serial1);

void doAverages();
void do12HourAverages();
int getNowCast25();
int getNowCast10();
void logPMMeasurement();
bool setupBME();
bool setupHPM();
float battery_level();
float get_temp();
float get_pressure();
float get_humidity();
float convertC2F(float temp);
float convertP2inHg(float pressure);
bool is_timed_out(unsigned long startTime_ms, unsigned long time_out_time);
void prnt_debug_text(String text);
bool connect_AIO();
