// WEMOS D1 mini: Arduino Board Lolin/Wemos D1 R2 & mini
// uncomment for Serial debugging statements
#define DEBUG_SERIAL

#ifdef DEBUG_SERIAL
#define DEBUG_BEGIN Serial.begin(115200)
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x) 
#define DEBUG_BEGIN
#endif

#define USE_BME280 0x76
#define USE_BME680 0x77
#define USE_IAQCOREC
#define USE_MHZ19B

#include <Adafruit_Sensor.h>              // Adafruit unified sensor library
#include <Adafruit_BME280.h>              // Adafruit BME280 library
#include <Adafruit_BME680.h>              // Adafruit BME680 library
//#include <bme680.h>
//#include <bme680_defs.h>
#include <EEPROM.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include <DNSServer.h> 
#include "iAQcore.h"                      // iAQ-Core: https://github.com/maarten-pennings/iAQcore
#include <MHZ.h>                          // MH-Z19b: https://github.com/tobiasschuerg/MH-Z-CO2-Sensors
#include <SoftwareSerial.h>               // Software Serial, used by MH-Z19b
#include <WiFiManager.h>                  // WifiMagaer: https://github.com/tzapu/WiFiManager
#include <WiFiUdp.h>                      // UDP Class
#include <Wire.h>

// Multicast declarations
WiFiUDP Udp;
IPAddress ipMulti(239, 255, 255, 250);    // site-local
unsigned int portMulti = 2085;            // port
char packetIn[255];                       // UDP in-buffer
char packetOut[512];                      // UPD out-buffer

// TODO: clear defaults
struct { 
  float t_offset = -.5;                   // offset temperature sensor
  float h_offset = 1.5;                   // offset humitidy sensor
  uint32_t vocBaseR = 108000;             // base value for VOC resistance clean air, abc 
  uint32_t vocBaseC = 0;                  // base value for VOC resistance clean air, abc  
  float vocHum = 0;                       // reserved, abc
  uint32_t signature = 0x49415143;        // 'IAQC'
} preload, param;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BME 280 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_BME280
Adafruit_BME280 bme280; // BME280 sensor object
unsigned long prevBme280Millis  = millis(); // counter main loop for BME 280
unsigned long intervalBme280    = 10000;    // 10 sec update interval default
char bme280Msg[64];   // payload

void getBme280Readings() {
  //
  float t_offset = 0.0F;
  float h_offset = 7.0F;
  //
  float t = bme280.readTemperature();
  float h = bme280.readHumidity() + h_offset;
  float a = absHum(t, h);
  float d = dewPoint(t, h);
  float p = bme280.readPressure() / 100.0F;
  //
  char str_temp[6];
  char str_hum[6];
  char str_absHum[6];
  char str_dewPoint[6];
  char str_pressure[16];
  //
#ifdef USE_DISPLAY  
  display.setTempHum(t, h);
#endif
  dtostrf(t, 4, 2, str_temp);
  dtostrf(h, 4, 2, str_hum);
  dtostrf(a, 4, 2, str_absHum);
  dtostrf(d, 4, 2, str_dewPoint);
  dtostrf(p, 3, 1, str_pressure);
  snprintf(bme280Msg, sizeof(bme280Msg), "F:THP;T:%s;H:%s;AH:%s;D:%s;P:%s;", str_temp, str_hum, str_absHum, str_dewPoint, str_pressure);
  //DEBUG_PRINT(bme280Msg);
};
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BME 680 
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_BME680
Adafruit_BME680 bme680; // BME680 sensor object
unsigned long prevBme680Millis  = millis(); // counter main loop for BME 680
unsigned long intervalBme680    = 10000;    // 10 sec update interval default
float resFiltered;                          // low pass
bool bme680VocValid = false;                // true if filter is initialized, ramp-up after start
char bme680Msg[128];                        // payload

void getBme680Readings() {
  if (! bme680.performReading()) {
    DEBUG_PRINT("Failed to perform reading :(");
    return;
  };
  
  float t = bme680.temperature + param.t_offset;
  float h = bme680.humidity + param.h_offset;
  float a = absHum(t, h);
  float d = dewPoint(t, h);
  float p = bme680.pressure /100.0F;
  uint32_t r = bme680.gas_resistance; // raw R VOC
  if (r == 0) return;                 // first reading is invalid
  uint32_t base = bme680Abc(r, a);       // update base resistance 
  if (!bme680VocValid && (millis() > 300000)) { // allow 300 sec warm-up for stable voc resistance (300000ms)
    resFiltered = r;        // preload low pass filter
    bme680VocValid = true;
  };
  if (!bme680VocValid ) return;
  resFiltered += 0.1 * (r - resFiltered);
  float ratio = (float)base / (resFiltered * a * 7.0F);
  float tVoc = (1250 * log(ratio)) + 125; // approximation
  char str_temp[6];
  char str_hum[6];
  char str_absHum[6];
  char str_dewPoint[6];
  char str_pressure[16];
  char str_tVoc[8];
#ifdef USE_DISPLAY      
  display.setTempHum(t, h);
#endif
  dtostrf(t, 4, 2, str_temp);
  dtostrf(h, 4, 2, str_hum);
  dtostrf(a, 4, 2, str_absHum);
  dtostrf(d, 4, 2, str_dewPoint);
  dtostrf(p, 3, 1, str_pressure);
  dtostrf(tVoc, 1, 0, str_tVoc);
  // DEBUG
  char str_filtered[16];
  dtostrf(resFiltered, 4, 3, str_filtered);
  char str_ratio[16];
  dtostrf(ratio, 4, 4, str_ratio);
  //
  snprintf(bme680Msg, sizeof(bme680Msg), "F:THPV;T:%s;H:%s;AH:%s;D:%s;P:%s;V:%s;R:%lu;DB:%lu;DF:%s;DR:%s;", 
    str_temp, str_hum, str_absHum, str_dewPoint, str_pressure, str_tVoc, r, base, str_filtered, str_ratio);
  //DEBUG_PRINT(bme680Msg);
};

// automatic baseline correction
uint32_t bme680_baseC = 0;                  // highest r (lowest voc) in current time period
float bme680_baseH = 0;                     // abs hum (g/m3)
unsigned long prevBme680AbcMillis = 0;      // ts of last save to nv 
unsigned long intervalBme680NV = 604800000; // 7 days of ms

uint32_t bme680Abc(uint32_t r, float a) {   // automatic baseline correction
  uint32_t b = r * a * 7.0F;
  if (b > bme680_baseC) {
    bme680_baseC = b;
    bme680_baseH = a;
  };

  // store baseline to nv
  unsigned long currentMillis = millis();
  if (currentMillis - prevBme680AbcMillis > intervalBme680NV) {
//    prevBme680AbcMillis = currentMillis;    
//    //store baseline
//    param.vocBase = bme680CurrentHigh;
//    bme680CurrentHigh = 0;
  };
  return (param.vocBaseC > bme680_baseC)?param.vocBaseC:bme680_baseC;
};
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// iaq core
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_IAQCOREC
iAQcore iaqcore;
unsigned long prevIaqMillis     = millis(); // counter main loop for iaq
unsigned long intervalIaq       = 10000;    // default

char iaqMsg[64];

void getIaqReadings() {
  uint16_t eco2;
  uint16_t stat;
  uint32_t resist;
  uint16_t etvoc;
  
  iaqcore.read(&eco2,&stat,&resist,&etvoc);
  stat = 0;
  if( stat & IAQCORE_STAT_I2CERR ) {
    DEBUG_PRINT("iAQcore: I2C error");
  } else if( stat & IAQCORE_STAT_ERROR ) {
    DEBUG_PRINT("iAQcore: chip broken");
  } else if( stat & IAQCORE_STAT_BUSY ) {
    DEBUG_PRINT("iAQcore: chip busy");
    prevIaqMillis = millis() - intervalIaq + 500;  // retry in 500msec
  } else {
    //display.setCo2VOC(eco2, etvoc);
    snprintf(iaqMsg, sizeof(iaqMsg), "F:IAQ;C:%u;V:%u;R:%u;", eco2, etvoc, resist);
    //DEBUG_PRINT(iaqMsg);
  };
};
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MH Z19B
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef USE_MHZ19B
#define MH_Z19_RX D7  // D7
#define MH_Z19_TX D8  // D6
#define CO2_IN 0      // pin for PWM reading, unused
MHZ co2(MH_Z19_RX, MH_Z19_TX, CO2_IN, MHZ19B);
unsigned long prevMhz19Millis   = millis(); // counter main loop for MH-Z19
unsigned long intervalMhz19     = 10000;    // default
char mhz19Msg[32];

void getMhz19Readings() {
  if (co2.isPreHeating()) return;
  int ppm_uart = co2.readCO2UART();
  if (ppm_uart > 0) {
    //display.setCo2(ppm_uart);
    snprintf(mhz19Msg, sizeof(mhz19Msg), "F:CO2;C:%u;", ppm_uart);
    //DEBUG_PRINT(mhz19Msg);
  };
};
#endif

float absHum(float temp, float hum) {
  double sdd, dd;
  sdd=6.1078 * pow(10,(7.5*temp)/(237.3+temp));
  dd=hum/100.0*sdd;
  return (float)216.687*dd/(273.15+temp);
};

float dewPoint(float temp, float hum) {
  double A = (hum/100) * pow(10, (7.5*temp / (237+temp)));
  return (float) 237*log10(A)/(7.5-log10(A));
};

void sendMessage(char* msg) {
  if (WiFi.status() == WL_CONNECTED && strlen(msg) != 0) {
    snprintf(packetOut, sizeof(packetOut), "T:IAQC;FW:1.0;ID:%06X;IP:%s;R:%ld;%s", ESP.getChipId(), WiFi.localIP().toString().c_str(), WiFi.RSSI(), msg);
    DEBUG_PRINT(packetOut);
    Udp.beginPacketMulticast( ipMulti, portMulti, WiFi.localIP() );
    Udp.println(packetOut);
    Udp.endPacket();
    strcpy(msg, "");
  };
};

void setup() {
  
  DEBUG_BEGIN; //for terminal debugging
  DEBUG_PRINT();

  // Enable I2C for Wemos D1 mini SDA:D2 / SCL:D1 
  Wire.begin(D2, D1);
  //Wire.setClock(10000L);

  // fetches ssid and pass from eeprom and tries to connect
  // opens an AP (ESP+ChipID), enable WiFi setup at 192.168.4.1
  // reboots if credentials are set
  WiFiManager wifiManager;
  // uncomment and run it once, if you want to erase all the stored information
  // wifiManager.resetSettings();
  wifiManager.setTimeout(90);
  wifiManager.autoConnect();

  // restart watchdog as a safety measure. 
  // its unknown how much time we spend in connect and the wdt triggers after 3.5 sec
  ESP.wdtFeed();
  ESP.wdtEnable(0);

  Udp.begin(portMulti); 
  DEBUG_PRINT("connected. device ip: " + WiFi.localIP());

  // read nv ram, param
  EEPROM.begin(sizeof(param));
  EEPROM.get(0, param);
  // validate param
  if (param.signature != 0x49415143) {
    DEBUG_PRINT("load default param");
    param = preload;
  };

#ifdef USE_BME280
  if (! bme280.begin(USE_BME280)) {
    DEBUG_PRINT("Could not find a valid BME280 sensor, check wiring and address!");
  };
#endif

#ifdef USE_BME680
  if (! bme680.begin(USE_BME680)) {
    DEBUG_PRINT("Could not find a valid BME680 sensor, check wiring and address!");
  };
#endif

#ifdef USE_IAQCOREC
  // initialize IAQ after other i2c periphals because Wire.begin will be called ie from within BME280 constructor
  Wire.setClockStretchLimit(1000); // Default is 230us, see line78 of https://github.com/esp8266/Arduino/blob/master/cores/esp8266/core_esp8266_si2c.c
  if (! iaqcore.begin()) {
    DEBUG_PRINT("Could not find a valid AMS IAQ CORE C sensor, check wiring!");
  };
  iaqcore.begin();
};
#endif

void loop() {

  unsigned long currentMillis = millis();

#ifdef USE_BME280
  if (currentMillis - prevBme280Millis > intervalBme280) {
    prevBme280Millis = currentMillis;
    getBme280Readings();
    sendMessage(bme280Msg);
  };
#endif

#ifdef USE_BME680
  if (currentMillis - prevBme680Millis > intervalBme680) {
    prevBme680Millis = currentMillis;
    getBme680Readings();
    sendMessage(bme680Msg);
  };
#endif

#ifdef USE_IAQCOREC  
  if (currentMillis - prevIaqMillis > intervalIaq) {
    prevIaqMillis = currentMillis;
    getIaqReadings();
    sendMessage(iaqMsg);
  };
#endif

#ifdef USE_MHZ19B
  if (currentMillis - prevMhz19Millis > intervalMhz19) {
    prevMhz19Millis = currentMillis;
    getMhz19Readings();
    sendMessage(mhz19Msg);
  };
#endif
  
};
