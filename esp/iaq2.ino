// uncomment for Serial debugging statements
// #define DEBUG_SERIAL 

#ifdef DEBUG_SERIAL
#define DEBUG_BEGIN Serial.begin(115200)
#define DEBUG_PRINT(x) Serial.println(x)
#else
#define DEBUG_PRINT(x) 
#define DEBUG_BEGIN
#endif

// includes
#include <Arduino.h>
#include <BearSSLHelpers.h>
#include <CertStoreBearSSL.h>
#include <DHT.h>
#include <DHT_U.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFi.h>
#include <ESP8266WiFiAP.h>
#include <ESP8266WiFiGeneric.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266WiFiScan.h>
#include <ESP8266WiFiSTA.h>
#include <ESP8266WiFiType.h>
#include "iAQcore.h" // iAQ-Core driver
#include <MHZ.h>
#include <SSD1306Wire.h>
#include <Ticker.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
#include <WiFiClientSecureAxTLS.h>
#include <WiFiClientSecureBearSSL.h>
#include <WiFiManager.h>
#include <WiFiServer.h>
#include <WiFiServerSecure.h>
#include <WiFiServerSecureAxTLS.h>
#include <WiFiServerSecureBearSSL.h>
#include <WiFiUdp.h>
#include <Wire.h>

//#include <SSD1306Wire.h>

// Include custom images
#include "images.h"
#include "RobotoML_Plain.h"

// LED
#define LED_ESP_BUILTIN D4
#define LED_ESP_NODEMCU D0

// Multicast declarations
WiFiUDP Udp;
IPAddress ipMulti(239, 255, 255, 250);    // site-local
unsigned int portMulti = 2085;            // port
// UDP in-buffer
char incomingPacket[255];
// udp message to send
char message[512];

// timer vars
// Signal LED
unsigned long prevLedMillis   = millis(); // counter main loop for signaling led
unsigned long prevDhtMillis   = millis(); // counter main loop for dht
unsigned long prevIaqMillis   = millis(); // counter main loop for iaq
unsigned long prevMhz19Millis = millis(); // counter main loop for MH-Z19
unsigned long prevLdrMillis   = millis(); // counter main loop for LDR
unsigned long sigLedOn        = 50;       // ms for led on
unsigned long sigLedOff       = 450;      // ms for led off
unsigned long intervalDht     = 10000;    // default
unsigned long intervalIaq     = 10000;    // default
unsigned long intervalMhz19   = 10000;    // default
unsigned long intervalLdr     = 1000;    // default
unsigned long intervalLed     = 0;        // 

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// derived display class
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class AIQDisplay: public SSD1306Wire { 
  private:
    uint8_t _hour;
    uint8_t _min;
    float _temp = 20;
    float _hum = 40;
    uint16_t _eco2 = 450;
    uint16_t _etvoc = 125;
    uint16_t _co2 = 400;
    uint16_t _lux = 0;
    bool _update = true;
    
  public:
    AIQDisplay(uint8_t _address, uint8_t _sda, uint8_t _scl, OLEDDISPLAY_GEOMETRY g = GEOMETRY_128_64)
      :SSD1306Wire(_address, _sda, _scl, g) {};

    bool connect() {
      return true;
    };

    void drawStr(int16_t xMove, int16_t yMove, char* text) {
      uint16_t textLength = strlen(text);
      drawStringInternal(xMove, yMove, text, textLength, getStringWidth(text, textLength));
    };

    void setTempHum(float temp, float hum) {
      _temp = temp;
      _hum = hum;
      _update = true;
    };

    void setCo2VOC(uint16_t eco2, uint16_t etvoc) {
      _eco2 = eco2;
      _etvoc = etvoc;
      _update = true;
    };

    void setCo2(uint16_t co2val) {
      _co2 = co2val;
      _update = true;
    };

    void setLux(uint16_t lux) {
      _lux = lux;
      _update = true;
    };

    void showSummary() {
      if (!_update) return;
      _update = false;
      char line[32];
      char str_temp[6];
      char str_hum[6];
      delay(10); // required i2c 
      clear();
      setFont(Roboto_Mono_Light_12);
      setTextAlignment(TEXT_ALIGN_CENTER);
      // temp
      dtostrf(_temp, 3, 1, str_temp);
      sprintf(line, "Temperatur: %s%s", str_temp, "\xb0");
      drawStr(64, 0, line);
      // hum
      dtostrf(_hum, 3, 1, str_hum);
      sprintf(line, "Luftfeuchte: %s%s", str_hum, "%");
      drawStr(64, 12, line);
      //  display();
      //  return;
      // eCO2
      sprintf(line, "CO2: %d%s", _co2, "ppm");
      drawStr(64, 26, line);
      // TVOC
      sprintf(line, "VOC: %d%s", _etvoc, "ppm");
      drawStr(64, 38, line);
      //sprintf(line, "BR: %d%s", _lux, "lux");
      //drawStr(64, 38, line);
      display();
      DEBUG_PRINT("display update");
    };
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// display
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SDA, SCL
  AIQDisplay display(0x3c, D2, D1);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// service
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
char serviceMsg[512];
Ticker serviceMessageTimer;           // cyclic service message (alive)

void serviceAlive() {
  sprintf(serviceMsg, "F:SERVICE;UP:%ld;", millis());
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// dht22
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DHTPIN D5                     // NodeMCU D5
#define DHTTYPE DHT22 
// !!! https://github.com/adafruit/DHT-sensor-library/issues/94
char dhtMsg[512];
DHT dht(DHTPIN, DHTTYPE);
char str_temp[6];
char str_hum[6];

void getDhtReadings() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(t) || isnan(h)) {
    prevDhtMillis = millis() - intervalDht + 2000;  // retry in 2 sec
  } else {
    display.setTempHum(t, h);
    dtostrf(h, 3, 1, str_hum);
    dtostrf(t, 3, 1, str_temp);
    sprintf(dhtMsg, "F:TH_A;T:%s;H:%s;", str_temp, str_hum);
  };
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// iaq core
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
char iaqMsg[512];
iAQcore iaqcore;

void getIaqReadings() {
  uint16_t eco2;
  uint16_t stat;
  uint32_t resist;
  uint16_t etvoc;
  
  iaqcore.read(&eco2,&stat,&resist,&etvoc);
  
  if( stat & IAQCORE_STAT_I2CERR ) {
    DEBUG_PRINT("iAQcore: I2C error");
  } else if( stat & IAQCORE_STAT_ERROR ) {
    DEBUG_PRINT("iAQcore: chip broken");
  } else if( stat & IAQCORE_STAT_BUSY ) {
    DEBUG_PRINT("iAQcore: chip busy");
    // prevIaqMillis = millis() - intervalIaq + 500;  // retry in 500msec
  } else {
    display.setCo2VOC(eco2, etvoc);
    sprintf(iaqMsg, "F:IAQ;C:%u;V:%u;R:%u;", eco2, etvoc, resist);
  };
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// MH Z19B
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

char mhz19Msg[32];

// pin for PWM reading, unused
#define CO2_IN 10

// pin for uart reading
#define MH_Z19_RX D7  // D7
#define MH_Z19_TX D8  // D6

MHZ co2(MH_Z19_RX, MH_Z19_TX, CO2_IN, MHZ19B);

void getMhz19Readings() {
  if (co2.isPreHeating()) return;
  int ppm_uart = co2.readCO2UART();
  if (ppm_uart > 0) {
    display.setCo2(ppm_uart);
    sprintf(mhz19Msg, "F:CO2;C:%u;", ppm_uart);
  };
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// LDR (NodeMCU A0)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

char ldrMsg[32];

void getLdrReadings() {
  uint16_t lux = analogRead (A0);
  display.setLux(lux);
  sprintf(mhz19Msg, "F:BR;B:%u;", lux);
};
////////////////////////////////////////////////

void prepareMessage(char* payload ) {
  //sprintf(message, "T:IAQC;FW:1.0;ID:%06X;IP:%s;R:%ld;%s", ESP.getChipId(), WiFi.localIP().toString().c_str(), WiFi.RSSI(), payload);
  Serial.println(message);
};

void setup() {
  // disable led on esp modul
  pinMode(LED_ESP_BUILTIN, OUTPUT);
  digitalWrite(LED_ESP_BUILTIN, HIGH);   
  pinMode(LED_ESP_NODEMCU, OUTPUT);
  digitalWrite(LED_ESP_NODEMCU, HIGH);  
  
  DEBUG_BEGIN; //for terminal debugging
  DEBUG_PRINT();

  // Enable I2C for ESP8266 NodeMCU boards [VDD to 3V3, GND to GND, SDA to D2, SCL to D1]
  Wire.begin(SDA,SCL); 
  Wire.setClockStretchLimit(1000); // Default is 230us, see line78 of https://github.com/esp8266/Arduino/blob/master/cores/esp8266/core_esp8266_si2c.c
  
  // display
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawStringMaxWidth(64, 52, 128, "AP:ESP.../192.168.4.1");
  display.drawXbm(34, 14, WiFi_Logo_width, WiFi_Logo_height, WiFi_Logo_bits);
  display.display();
  
  // fetches ssid and pass from eeprom and tries to connect
  // opens an AP (ESP+ChipID), enable WiFi setup at 192.168.4.1
  // reboots if credentials are set
  WiFiManager wifiManager;
  // uncomment and run it once, if you want to erase all the stored information
  // wifiManager.resetSettings();
  wifiManager.setTimeout(1); // TODO !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  wifiManager.autoConnect();

  // restart watchdog as a safety measure. 
  // its unknown how much time we spend in connect - but the wdt triggers after 3.5 sec
  ESP.wdtFeed();
  ESP.wdtEnable(0);

  Udp.begin(portMulti); 
  DEBUG_PRINT("connected. device ip: " + WiFi.localIP());
  
  sprintf(serviceMsg, "F:SERVICE;START:%s;", ESP.getResetReason().c_str());
  serviceMessageTimer.attach(60, serviceAlive);

  iaqcore.begin();
  prevIaqMillis = millis() - intervalIaq + 500;  // first Iaq reading 500 msec after startup
  
  // temp/hum
  dht.begin();
  prevDhtMillis = millis() - intervalDht + 1000;  // first Dht reading 1 sec after startup
  // LDR
  prevLdrMillis = millis() - intervalLdr + 200;  // first LDR reading 200 msec after startup
};

void loop() {

  unsigned long currentMillis = millis();
   
  // led signal
  if (currentMillis - prevLedMillis > intervalLed) {
    uint8_t state = digitalRead(LED_ESP_NODEMCU);
    if (state == HIGH) {
      DEBUG_PRINT("ping");
      //digitalWrite(LED_ESP_NODEMCU, LOW);
      intervalLed = sigLedOn;
      prevLedMillis = currentMillis;
    } else {
      //digitalWrite(LED_ESP_NODEMCU, HIGH);
      intervalLed = sigLedOff;
      prevLedMillis = currentMillis;      
    };
  };

  if (currentMillis - prevDhtMillis > intervalDht) {
    prevDhtMillis = currentMillis;
    getDhtReadings();
  };

  if (currentMillis - prevIaqMillis > intervalIaq) {
    prevIaqMillis = currentMillis;
    getIaqReadings();
  };

  if (currentMillis - prevMhz19Millis > intervalMhz19) {
    prevMhz19Millis = currentMillis;
    getMhz19Readings();
  };

  if (currentMillis - prevLdrMillis > intervalLdr) {
    prevLdrMillis = currentMillis;
    getLdrReadings();
  };
  
  // incoming udp msg
  int packetSize = Udp.parsePacket();
  if (packetSize && (packetSize < 0x100)) { // prevent overflow
    //Serial.printf("Received %d bytes from %s, port %d\n", packetSize, Udp.remoteIP().toString().c_str(), Udp.remotePort());
    int len = Udp.read(incomingPacket, 255);
    //Serial.printf( "UDP packet contents: %s\n", incomingPacket );
    if (len > 0) {
      incomingPacket[len] = 0;
      char* command = strtok(incomingPacket, ":;");
      //Serial.printf( "command: %s\n", command );
      char* value = strtok(NULL, ";");
      //Serial.printf( "value: %s\n", value );
      if (command != 0) {
//        if ((strcmp(command, "S0_A_Meter") == 0) && (value != 0)) {
//          //Serial.printf( "value: %s\n", value );
//          char* meter = strtok(value, ",");
//          S0_A_Meter = strtoul(meter, NULL, 0);
//          char* factor = strtok(NULL, "");
//          //Serial.printf( "factor: %s\n", factor );
//          S0_A_Factor = atoi(factor);
//          S0_A_inSync = true;
//          sprintf(S0_A_MeterMsg, "F:S0_A;IX:%d;C:%d,%d;S:%d;", S0_A_Counter, S0_A_Meter, S0_A_Factor, S0_A_inSync); // sort of ack
//        };
//        if ((strcmp(command, "S0_B_Meter") == 0) && (value != 0)) {
//          char* meter = strtok(value, ",");
//          S0_B_Meter = strtoul(meter, NULL, 0);
//          char* factor = strtok(NULL, "");
//          S0_B_Factor = atoi(factor);
//          S0_B_inSync = true;
//          sprintf(S0_B_MeterMsg, "F:S0_B;IX:%d;C:%d,%d;S:%d;", S0_B_Counter, S0_B_Meter, S0_B_Factor, S0_B_inSync); // sort of ack
//        };
      };
    };
  };
  
  // send and clear service msg if any
  if (WiFi.status() == WL_CONNECTED && strlen(serviceMsg) != 0) {
    prepareMessage( serviceMsg );
    DEBUG_PRINT("service message:");
    DEBUG_PRINT(message);
    Udp.beginPacketMulticast(ipMulti, portMulti, WiFi.localIP());
    Udp.println(message);
    Udp.endPacket();
    strcpy(serviceMsg,"");
  };
    
  // send and clear Temp/Hum  msg if any
  if (WiFi.status() == WL_CONNECTED && strlen(dhtMsg) != 0) {
    prepareMessage(dhtMsg);
    DEBUG_PRINT("Temp Hum msg:");
    DEBUG_PRINT(message);
    Udp.beginPacketMulticast( ipMulti, portMulti, WiFi.localIP() );
    Udp.println(message);
    Udp.endPacket();
    strcpy(dhtMsg, "");
  };
  
  // send and clear IAQ msg if any
  if (WiFi.status() == WL_CONNECTED && strlen(iaqMsg) != 0) {
    prepareMessage(iaqMsg);
    DEBUG_PRINT("IAQ msg:");
    DEBUG_PRINT(message);
    Udp.beginPacketMulticast( ipMulti, portMulti, WiFi.localIP() );
    Udp.println(message);
    Udp.endPacket();
    strcpy(iaqMsg, "");
  };
  
  // send and clear MH-Z19 msg if any
  if (WiFi.status() == WL_CONNECTED && strlen(mhz19Msg) != 0) {
    prepareMessage(mhz19Msg);
    DEBUG_PRINT("MH-Z19 msg:");
    DEBUG_PRINT(message);
    Udp.beginPacketMulticast( ipMulti, portMulti, WiFi.localIP() );
    Udp.println(message);
    Udp.endPacket();
    strcpy(mhz19Msg, "");
  };

  display.showSummary();
//  display.clear();
//  display.setTextAlignment(TEXT_ALIGN_LEFT);
//  display.setFont(ArialMT_Plain_24);
//  clockScreen();
//  //display.clockScreen();
//  display.display();
  
};
