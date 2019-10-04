#if !(defined(ESP_NAME))
  #define ESP_NAME "trigger" 
#endif

#if !(defined(DISPLAY_128X32) || defined(DISPLAY_128X64))
  #define DISPLAY_128X64 1 
#endif

#if !(defined(MAX_RANGE))
  #define MAX_RANGE 1200 
#endif

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY

#include <Arduino.h>

#include <ESP8266WiFi.h> // WIFI support
#include <ESP8266mDNS.h> // For network discovery
#include <WiFiUdp.h> // OSC over UDP
#include <ArduinoOTA.h> // Updates over the air

// WiFi Manager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h> 

// OSC
#include <OSCMessage.h> // for sending OSC messages

// I2C
#include <SPI.h>
#include <Wire.h>

// Display (SSD1306)
#include <SSD1306Ascii.h>
#include <SSD1306AsciiWire.h>

// Sensor
#include <VL53L0X.h>

// Median for Sensor
#include <RunningMedian.h>

/* Constants */
const unsigned long RUN_INTERVAL = 100;

/* Variables */
bool tripped = false;
int maxRange = 0;
int minRange = 0;

unsigned int count = 0;
unsigned long nextRun = 0;
unsigned long displayTimeout = 0;

/* Display */
SSD1306AsciiWire oled;

/* WIFI */
const unsigned int OSC_PORT = 53000;
char hostname[32] = {0};

/* VL53L0X */
VL53L0X sensor;
RunningMedian samples = RunningMedian(9);

/* OSC */
WiFiUDP Udp;

String qLabHostname;
IPAddress qLabIp;
unsigned int qLabPort;
const char* qLabMessage = QLAB_MESSAGE;

void sendQLabOSCMessage(const char* address) {
  OSCMessage msg(address);
  Udp.beginPacket(qLabIp, qLabPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();

  // Send message three times to ensure delivery.  Need to come up with a better approach.
  Udp.beginPacket(qLabIp, qLabPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();

  Udp.beginPacket(qLabIp, qLabPort);
  msg.send(Udp);
  Udp.endPacket();
  msg.empty();
}

void configModeCallback (WiFiManager *myWiFiManager) {
  oled.println(F("Config Mode"));
  oled.println(WiFi.softAPIP());
  oled.println(myWiFiManager->getConfigPortalSSID());
}

void setup()
{
  Serial.begin(9600);
  Wire.begin(D2, D1);

  delay(1000);

  /* Display */
#if defined DISPLAY_128X64
  oled.begin(&Adafruit128x64, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x64)
#else
  oled.begin(&Adafruit128x32, 0x3C);
#endif
  oled.setFont(System5x7);
  oled.setScrollMode(SCROLL_MODE_AUTO);
  oled.clear();

  /* WiFi */
  sprintf(hostname, "%s-%06X", ESP_NAME, ESP.getChipId());
  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);
  if(!wifiManager.autoConnect(hostname)) {
    oled.println("WiFi Connect Failed");
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(1000);
  } 

  /* UDP */
  Udp.begin(OSC_PORT);

  oled.println(hostname);
  oled.print(F("  "));
  oled.print(WiFi.localIP());
  oled.print(F(":"));
  oled.println(Udp.localPort());
#if defined DISPLAY_128X64
  oled.print(F("  "));
  oled.println(WiFi.macAddress());
#endif

  /* OTA */
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    oled.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    oled.println(F("\nEnd"));
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    oled.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    oled.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) { oled.println(F("Auth Failed")); }
    else if (error == OTA_BEGIN_ERROR) { oled.println(F("Begin Failed")); }
    else if (error == OTA_CONNECT_ERROR) { oled.println(F("Connect Failed")); }
    else if (error == OTA_RECEIVE_ERROR) { oled.println(F("Receive Failed")); } 
    else if (error == OTA_END_ERROR) { oled.println(F("End Failed")); }
  });
  ArduinoOTA.begin();

  /* mDNS */
  // Initialization happens inside ArduinoOTA;
  MDNS.addService(ESP_NAME, "udp", OSC_PORT);

  // Discover qLab
  int queryCount = 0;
  while (MDNS.queryService("qlab", "udp") == 0) {
    oled.printf("find qlab: %u\r", queryCount);
    ArduinoOTA.handle();
    delay(1000);
    queryCount++;
  }
  qLabHostname = MDNS.hostname(0);
  qLabIp = MDNS.IP(0);
  qLabPort = MDNS.port(0);

#if defined DISPLAY_128X64
  oled.println(qLabHostname);
  oled.print(F("  "));
  oled.print(qLabIp);
  oled.print(F(":"));
  oled.println(qLabPort);
  oled.print(F("  "));
  oled.println(QLAB_MESSAGE);
#endif

  /* Sensor */
  sensor.init();
  sensor.setTimeout(500);
#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  sensor.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  sensor.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  sensor.setMeasurementTimingBudget(200000);
#endif
  sensor.startContinuous();

  // Calibrate
  oled.print(F("Calibrating...\r"));
  delay(1000);
  int calibrateCount = 0;
  int range = 0;
  while((range = sensor.readRangeContinuousMillimeters()) > 1200) {
    oled.printf("Calibrate: %u\r", calibrateCount);
    ArduinoOTA.handle();
    delay(100);
    calibrateCount++;
  }
  oled.println(F("---------------------"));
  maxRange = range;
  minRange = range;

  /* LED */
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
  //oled.setContrast(0);
}

void loop()
{
  ArduinoOTA.handle();
  
  if (millis() < nextRun) {
    return;
  }
  nextRun = millis() + RUN_INTERVAL;

  int range = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) { 
    return;
  }

  // Filter Noise
  //samples.add(range);
  //range = samples.getMedian();

  // Detect if tripped
  if ((range < (maxRange - 100)) && !tripped) {
    sendQLabOSCMessage(QLAB_MESSAGE);
    digitalWrite(LED_BUILTIN, LOW);
    tripped = true;
    oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
    minRange = range; // Reset minimum range
    count++;
  } 
  else if (range > (minRange + 100) && tripped) {
    digitalWrite(LED_BUILTIN, HIGH);
    tripped = false;
    oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
    maxRange = range; // Reset max range
  }
  else if (range > maxRange) {
    maxRange = range;
  } else if (range < minRange) {
    minRange = range;
  }

  // Ensure max range does not go above maximum allowed
  if (maxRange > MAX_RANGE) {
    maxRange = MAX_RANGE;
  }

  oled.printf("%4d %4d %4d %6d\r", minRange, range, maxRange, count);
  //oled.invertDisplay(tripped);
}

