/**
   LedClock.ino

   Displays Clock using LEDs

    Created on: 12.3.2018

*/

//Pin defintions
#define I2CSDA 4  //D2 gruen
#define I2CSCL 5  //D1 gelb
#define PIRINPUT 12 //D6
#define NEOPIXEL 14 //D5


#define NROFLEDS 60
#define DEBUG 1
#define DS3231 0x68
#define BME280ADDR 0x76
// #define SI7021
#define MPR121 0x5a
#define TSL2561

// 0x29 TSL45315 (Light)
// 0x38 VEML6070 (Light)
// 0x39 TSL2561
// 0x40 SI7021
// 0x48 4*AD converter
// 0x4a GY49 or MAX44009 Light Sensor
// 0x50 PCF8583P
// 0x57 ATMEL732
// 0x5a MPR121 12-Touchpanel
// 0x68 DS3231 Clock
// 0x76 BME280
// 0x77 BME680 (also BMP180)



#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP8266)
#include <ESP8266WiFi.h>
#else
#include <WiFi.h>
#endif

#include <Wire.h>
#include <PubSubClient.h>
#include <FS.h>

// to get reset reason
#if defined(ARDUINO_ARCH_ESP32)
#include <rom/rtc.h>
#include <SPIFFS.h>
#endif

#include <SparkFunBME280.h>
// #include <ESP8266HTTPClient.h>

#if defined(MPR121)
#include "Adafruit_MPR121.h"
Adafruit_MPR121 cap = Adafruit_MPR121();
uint16_t lasttouched = 0;
uint16_t currtouched = 0;

// define 12 touch buttons
#define BUTTON_TEMPERATURE 0x0001 // show temperature
#define BUTTON_HUMIDITY    0x0002 // show humidity
#define BUTTON_HOURPLUS    0x0004 // increase hour by one
#define BUTTON_HOURMINUS   0x0008
#define BUTTON_MINUTEPLUS  0x0010
#define BUTTON_MINUTEMINUS 0x0020
#define BUTTON_ENTER       0x0040
#define BUTTON_CANCEL      0x0080

// Left side from top to bottom
// show temperature
// show humidity

// set time
// set countdown
// set waketime

// hour+
// hour-
// minute+
// minute-

// right side from top to bottom
// enter
// cancel
// countdown minunte+ (hold for fast increase)
// countdown minute-


#endif

// define operational modes
#define MODE_OFF 0
#define MODE_CLOCK 1
#define MODE_TIMER 2
#define MODE_SETTIMER 3
static int runMode = MODE_CLOCK;

// DS3231 real time clock
#if defined(DS3231)
#include "RTClib.h"
RTC_DS3231 rtc;
static byte rtc_initialized = 0;

// Task Scheduler 
#include <TaskScheduler.h>
Scheduler runner;
void showTime();
void showSensor();
void publishSensor();
void showTimer();
Task clock_task(500,TASK_FOREVER,&showTime);
Task showSensor_task(1000*3*60,TASK_FOREVER,&showSensor);
Task publishSensor_task(1000*30,TASK_FOREVER,&publishSensor);
Task timer_task(500,20,&showTimer);

DateTime timerTarget;
TimeSpan setupTimer;

// Brightness
static double Brightness = 10;


// print current time
void printCurrentTime(void) {
  if (rtc_initialized) {
    DateTime now = rtc.now();
    Serial.println("Now: " + String(now.year()) + "-" + String(now.month()) + "-" + String(now.day()) + " " + String(now.hour()) + ":" + String(now.minute()));
  } else {
    Serial.println("Error: RTC not initialized");
  }
}

void rtcSetup() {
  if (rtc.begin() && !rtc.lostPower()) {
    rtc_initialized = 1;
#ifdef DEBUG  
    printCurrentTime();
#endif
    } else {
    rtc_initialized = 0;
  }
}
#endif



// Temp + Humidity Sensor Si7021
#if defined(SI7021)
#include "Adafruit_Si7021.h"
Adafruit_Si7021 si7021 = Adafruit_Si7021();
#endif


// Light sensor TSL2561
#ifdef TSL2561
#include <SparkFunTSL2561.h>
SFE_TSL2561 lightTSL2561;
unsigned int msTSL2561;
boolean gainTSL2561;
#endif


#include <Adafruit_NeoPixel.h>
Adafruit_NeoPixel led = Adafruit_NeoPixel(NROFLEDS, NEOPIXEL, NEO_GRB + NEO_KHZ800);

const char* fn_ssid = "/ssid";
const char *fn_pass = "/pass";
const char *fn_myname = "/myname";
const char *fn_mqttserver = "/mqttserver";
const char *fn_mqttpass = "/mqttpass";
const char *fn_mqttuser = "/mqttuser";
const char *fn_mqttport = "/mqttport";
const char *fn_site = "/site";
const char *fn_location = "/location";

String Smyname, Spass, Sssid, Smqttserver, Ssite, Slocation, Smqttuser, Smqttpass, Smqttport;

// Web for status and config
// WiFiServer server(80);
WiFiClient espClient, webClient;
PubSubClient client;
// experimental
// WiFiEventHandler disconnectedEventHandler;

long lastMsg = 0;
char msg[50];
char topic[50];
int value = 0;


int pirInput = PIRINPUT;
int pirState = LOW;


// read data from file and return it
String readFromFile(const char *filename, String s) {
  s = "";
  if (SPIFFS.exists(filename)) {
    File f = SPIFFS.open(filename, "r");
    if (!f) {
#ifdef DEBUG
      Serial.print("Cannot read from file ");
      Serial.println(filename);
#endif
      s = "";
    } else {
      s = f.readString();
      f.close();
    }
  } else {
#ifdef DEBUG
    Serial.print("File ");
    Serial.print(filename);
    Serial.println("does not exist");
#endif
  }
  // remove trailing cr
  s.trim();
  return s;
}

// write into file
String writeFile(const char *filename, String s) {
  File f = SPIFFS.open(filename, "w");
  f.println(s);
  f.close();
  return s;
}


uint32_t makeColor(int r, int g, int b) {
#ifdef NROFLEDS
  return led.Color(constrain(r,0,255),constrain(g,0,255),constrain(b,0,255));
#endif
}

// dims all colors equally by factor dimBy
uint32_t dimColor(uint32_t color, int dimBy) {
  uint8_t 
      r = (uint8_t)(color >> 16),
      g = (uint8_t)(color >>  8),
      b = (uint8_t)color;

  // r /= dimBy;
  // g /= dimBy;
  // b /= dimBy;

  // lowest color 
  r = (r == 0) ? 0 : dimBy;
  g = (g == 0) ? 0 : dimBy;
  b = (b == 0) ? 0 : dimBy;
  return makeColor(r,g,b);
}

// dim color to Brightness variable
uint32_t dimColor(uint32_t color) {
  // mapping of Brighness
  uint8_t 
      r = (uint8_t)(color >> 16),
      g = (uint8_t)(color >>  8),
      b = (uint8_t)color;

  uint8_t smallest = min(r,b);
  if (smallest == 0)
    smallest = max(r,b);

  uint8_t sm2 = min(smallest,g);
  if (sm2 == 0)
    sm2 = max(smallest,g);

  if (sm2 == 0)
    return 0;
  
  uint8_t mr = r / sm2;
  uint8_t mg = g / sm2;
  uint8_t mb = b / sm2;
  
  if (Brightness < 1) {
    return makeColor(mr,mg,mb);
  } else if (Brightness <= 20) {
    return makeColor(mr*Brightness,mg*Brightness,mb*Brightness);
  } else {
    return color;
  }
}

// round-robin map
uint32_t rrmap(int in) {
  in +=30;
#ifdef NROFLEDS
  if (in < 0) {
    return NROFLEDS+in;
  } else if (in >= NROFLEDS) {
    return in-NROFLEDS;
  } else
    return in;
#endif
}

uint32_t addColor(int i, uint32_t color) {
#ifdef NROFLEDS
  if (i < 0 || i >= NROFLEDS) 
    return 0;

  uint8_t r,g,b;
  uint32_t oldcolor = led.getPixelColor(i);
  r = (uint8_t)(oldcolor >> 16);
  g = (uint8_t)(oldcolor >>  8);
  b = (uint8_t)oldcolor;
  
  uint8_t 
      rx = (uint8_t)(color >> 16),
      gx = (uint8_t)(color >>  8),
      bx = (uint8_t)color;

  uint32_t result = makeColor(r+rx,g+gx,b+bx);
  led.setPixelColor(i,result);
  return result;
#endif
}

// displays a number of led with the center one brightest
void centerLight(int center, unsigned int width, uint32_t color) {
#ifdef NROFLEDS
  led.setPixelColor(rrmap(center),color);
  for (int i=1; i<=width; i++) {
    led.setPixelColor(rrmap(center+i),dimColor(color,width+1-i));
    led.setPixelColor(rrmap(center-i),dimColor(color,width+1-i));
  }
#endif
}

// additive version - adds Colors together
void centerLight(int center, unsigned int width, uint32_t color, bool additive) {
#ifdef NROFLEDS
  if (!additive)
    return centerLight(center,width,color);

  
  addColor(rrmap(center),color);
  for (int i=1; i<=width; i++) {
    // led.setPixelColor(rrmap(center+i),dimColor(color,width+1-i));
    // led.setPixelColor(rrmap(center-i),dimColor(color,width+1-i));
    addColor(rrmap(center+i),dimColor(color,width+1-i));
    addColor(rrmap(center-i),dimColor(color,width+1-i));
  }
#endif
}



void showTime(DateTime when) {
  int hour12;
  if (when.hour() >= 12) {
    hour12 = when.hour()-12;
  } else {
    hour12 = when.hour();
  }

  for (int i=0; i < NROFLEDS; i++)
    led.setPixelColor(i,0,0,0);

  int hourStart = map(hour12,0,12,0,NROFLEDS);
  int hourWidth = map(2,0,12,0,NROFLEDS-1) - map(1,0,12,0,NROFLEDS-1);
  float hourFraction= when.minute()/60.0;
  int hourEnd = hourStart + int((float)hourWidth * hourFraction);

  // Hour light width 5 (2*2+1)
  centerLight(hourEnd,2,dimColor(led.Color(120,0,0)),false);
  // Minute light width 3 (2*1+1)
  centerLight(map(when.minute(),0,59,0,NROFLEDS-1),1,dimColor(led.Color(120,120,0)),true);
  // second light, dim to Brightnes
  led.setPixelColor(rrmap(map(when.second(),0,59,0,NROFLEDS-1)),dimColor(makeColor(120,120,120))); 
  led.show();
}

void showTime() {
#if defined(DS3231)
  if (timer_task.isEnabled()) {
    timer_task.disable();
    timer_task.enableDelayed(3*1000);
  }
  showTime(rtc.now());
#endif
}

// show the temperature on a scale from minTemp to maxTemp, negative values are blue, positive values are red
void showTemperature(int minTemp, int maxTemp, int thisTemp) {
#ifdef NROFLEDS
#ifdef DEBUG
  Serial.println("showTemperature(" + String(minTemp) + "," + String(maxTemp) + "," + String(thisTemp) + ")");
#endif

  for (int i=0; i < NROFLEDS; i++)
    led.setPixelColor(i,0,0,0);

  led.show();

  int lastLed = 0;
  for (int i=minTemp; i<=0 && i <= thisTemp;i++) {
    if (i == thisTemp) {
      led.setPixelColor(rrmap(map(i,minTemp,maxTemp,0,NROFLEDS-1)),dimColor(makeColor(0,0,254)));
    } else {
      int thisLed = map(i,minTemp,maxTemp,0,NROFLEDS-1);
      led.setPixelColor(rrmap(thisLed),0,0,1);
      if (thisLed-1 > lastLed)
        led.setPixelColor(rrmap(thisLed-1),0,0,1);
      lastLed = thisLed;
    }
    led.show();
    delay(10);
  }
  for (int i=1;i<=maxTemp && i <= thisTemp;i++) {
    if (i == thisTemp) {
      led.setPixelColor(rrmap(map(i,minTemp,maxTemp,0,NROFLEDS-1)),dimColor(makeColor(254,0,0)));
    } else {
      int thisLed = map(i,minTemp,maxTemp,0,NROFLEDS-1);
      led.setPixelColor(rrmap(thisLed),1,0,0);
      if (thisLed-1 > lastLed)
        led.setPixelColor(rrmap(thisLed-1),1,0,0);
      lastLed = thisLed;
    }
    led.show();
    delay(10);
  }
  
#endif  
}

void showTemperature() {
  clock_task.disable();
  timer_task.disable();
  showTemperature(-10,40,getLocalTemperature());
  delay(5000);
  for (int i=NROFLEDS-1; i >=0; i--) {
    int wait = led.getPixelColor(rrmap(i));
    led.setPixelColor(rrmap(i),0,0,0);
    led.show();
    if (wait) {
      delay(10);
    }
  }
  showTime();
  clock_task.enable();
  timer_task.enable();
}


// show the humidity on a scale of 0 to 100
void showHumidity(int thisHum) {
#ifdef NROFLEDS
  for (int i=0; i < NROFLEDS; i++)
    led.setPixelColor(i,0,0,0);

  
  led.show();
  
  for (int i=0; i <= thisHum; i++) {
    if (i == thisHum) {
      led.setPixelColor(rrmap(map(i,0,100,0,NROFLEDS-1)),dimColor(makeColor(0,100,100)));
    } else if (i < thisHum) {
      led.setPixelColor(rrmap(map(i,0,100,0,NROFLEDS-1)),0,1,1);
    }
    led.show();
    delay(7);
  }
#endif
}

void showHumidity() {
  clock_task.disable();
  timer_task.disable();
  showHumidity(getLocalHumidity());
  delay(5000);
  for (int i=NROFLEDS-1; i >=0; i--) {
    int wait = led.getPixelColor(rrmap(i));
    led.setPixelColor(rrmap(i),0,0,0);
    led.show();
    if (wait) {
      delay(10);
    }
  }
  showTime();
  clock_task.enable();
  timer_task.enable();
}

void showSensor() {
  clock_task.disable();
  showTemperature(-10,40,getLocalTemperature());
  delay(5000);
  showHumidity(getLocalHumidity());
  delay(5000);
  for (int i=NROFLEDS-1; i >=0; i--) {
    int wait = led.getPixelColor(rrmap(i));
    led.setPixelColor(rrmap(i),0,0,0);
    led.show();
    if (wait) {
      delay(10);
    }
  }
  showTime();
  clock_task.enable();
}


// set all LEDs to the same color
void monochrome(byte r, byte g, byte b) {
#ifdef NROFLEDS
  for (int i=0; i < NROFLEDS; i++) {
    led.setPixelColor(i,r,g,b);
  }
  led.show();
#endif
}

void monochrome (uint32_t c) {
  #ifdef NROFLEDS
  for (int i=0; i < NROFLEDS; i++) {
    led.setPixelColor(i,c);
  }
  led.show();
#endif
}

// set LEDs to a variant of the same color (randomized)
void monochrome(byte r, byte g, byte b, byte variant) {
#ifdef NROFLEDS
  for (int i=0; i < NROFLEDS; i++) {
    led.setPixelColor(i,makeColor(r+random(-variant,variant+1),g+random(-variant,variant+1),b+random(-variant,variant+1)));
  }
  led.show();
#endif
}


// led funtion. in case of not indoor it does nothing
void setled(byte r, byte g, byte b) {
#ifdef NROFLEDS
  led.setPixelColor(0, r, g, b);
  led.show();
#endif
}

void setled(byte n, byte r, byte g, byte b) {
#ifdef NROFLEDS
  led.setPixelColor(n, r, g, b);
  led.show();
#endif
}

void setled(byte n, byte r, byte g, byte b, byte show) {
#ifdef NROFLEDS
  led.setPixelColor(n, r, g, b);
  if (show) {
    led.show();
  }
#endif
}

void setled(byte show) {
#ifdef NROFLEDS
  if (!show) {
    int i;
    for (i = 0; i < NROFLEDS; i++) {
      setled(i, 0, 0, 0, 0);
    }
  }
  led.show();
#endif
}

void flashLeds(int howoften, uint32_t c) {
  for (int i = 0; i < howoften; i++ ) {
    monochrome(c);
    delay(50);
    led.clear();
    led.show();
    delay(50);
  }
}

void dumpT(TimeSpan t) {
#ifdef DEBUG
  Serial.print("TimeSpan hours=");
  Serial.print(t.hours());
  Serial.print(" minutes=");
  Serial.print(t.minutes());
  Serial.print(" seconds=");
  Serial.println(t.seconds());
#endif
}

void dumpD(uint32_t x) {
#ifdef DEBUG
  DateTime t = DateTime(x);
  Serial.print("DateTime hour=");
  Serial.print(t.hour());
  Serial.print(" minute=");
  Serial.print(t.minute());
  Serial.print(" second=");
  Serial.println(t.second());
#endif
}

// Functions for count down timer
void setTimer(int minutes, int seconds) {
  // sets the timer to number of minutes and seconds
#ifdef DEBUG
  Serial.print("setTimer(");
  Serial.print(minutes);
  Serial.println(")");
#endif

  TimeSpan when = TimeSpan(0,0,minutes,0);
  dumpT(when);
  timerTarget = DateTime(rtc.now() + when); 
  dumpD(rtc.now().unixtime());
  dumpD(timerTarget.unixtime());
}

void setTimer(int minutes) {
  setTimer(minutes,0);
}

void setTimer(TimeSpan t) {
  timerTarget = DateTime(rtc.now() + t);
}

void adjustTimer(int minutes) {
  TimeSpan adjustment = TimeSpan(0,0,minutes,0);
  dumpD(timerTarget.unixtime());
  timerTarget = DateTime(timerTarget + adjustment);
  dumpD(timerTarget.unixtime());
}


void showTimer(TimeSpan timeLeft) {

    led.clear();
    
    for (int i = 0; i <= timeLeft.minutes(); i++) {
      led.setPixelColor(rrmap(i),dimColor(makeColor(0,100,0)));
    }
    if (timeLeft.hours() >= 1) {
      for (int i = 0; i < timeLeft.hours(); i++) {
        led.setPixelColor(rrmap(i),dimColor(makeColor(100,20,0)));
      }
    }
    if (runMode != MODE_SETTIMER) 
      led.setPixelColor(rrmap(timeLeft.seconds()),dimColor(makeColor(100,100,100)));
    led.show();
  
}

void showTimer(void) {
  TimeSpan timeLeft = timerTarget - rtc.now();

  // dumpT(timeLeft);

  if (runMode == MODE_TIMER && timeLeft.totalseconds() <= 0) {
    // time is up
    flashLeds(10,makeColor(0,100,0));
    setMode(MODE_CLOCK);
  } 
  else 
    showTimer(timeLeft);

}

void printConfig() {
#ifdef DEBUG
  String s;
  Serial.println("Config data in memory / in file:");
  Serial.print(fn_myname);
  Serial.print("=");
  Serial.print(Smyname);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_myname, s));

  Serial.print(fn_ssid);
  Serial.print("=");
  Serial.print(Sssid);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_ssid, s));

  Serial.print(fn_pass);
  Serial.print("=");
  Serial.print(Spass);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_pass, s));

  Serial.print(fn_site);
  Serial.print("=");
  Serial.print(Ssite);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_site, s));

  Serial.print(fn_location);
  Serial.print("=");
  Serial.print(Slocation);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_location, s));

  Serial.print(fn_mqttserver);
  Serial.print("=");
  Serial.println(Smqttserver);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_mqttserver, s));

  Serial.print(fn_mqttport);
  Serial.print("=");
  Serial.println(Smqttport);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_mqttport, s));

  Serial.print(fn_mqttuser);
  Serial.print("=");
  Serial.println(Smqttuser);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_mqttuser, s));

Serial.print(fn_mqttpass);
  Serial.print("=");
  Serial.println(Smqttpass);
  Serial.print(" / ");
  Serial.println(readFromFile(fn_mqttpass, s));


#endif
}

#if defined(ARDUINO_ARCH_ESP32)
void verbose_print_reset_reason(RESET_REASON reason)
{
  switch ( reason)
  {
    case 1  : Serial.println ("Vbat power on reset");break;
    case 3  : Serial.println ("Software reset digital core");break;
    case 4  : Serial.println ("Legacy watch dog reset digital core");break;
    case 5  : Serial.println ("Deep Sleep reset digital core");break;
    case 6  : Serial.println ("Reset by SLC module, reset digital core");break;
    case 7  : Serial.println ("Timer Group0 Watch dog reset digital core");break;
    case 8  : Serial.println ("Timer Group1 Watch dog reset digital core");break;
    case 9  : Serial.println ("RTC Watch dog Reset digital core");break;
    case 10 : Serial.println ("Instrusion tested to reset CPU");break;
    case 11 : Serial.println ("Time Group reset CPU");break;
    case 12 : Serial.println ("Software reset CPU");break;
    case 13 : Serial.println ("RTC Watch dog Reset CPU");break;
    case 14 : Serial.println ("for APP CPU, reseted by PRO CPU");break;
    case 15 : Serial.println ("Reset when the vdd voltage is not stable");break;
    case 16 : Serial.println ("RTC Watch dog reset digital core and rtc module");break;
    default : Serial.println ("NO_MEAN");
  }
}
#endif

void setup() {

#ifdef DEBUG
  Serial.begin(115200);
  // USE_SERIAL.setDebugOutput(true);
  Serial.println("\nStarting");
#if defined(ARDUINO_ARCH_ESP8266)
  Serial.print("Sleep mode:");
  Serial.println(WiFi.getSleepMode());
  Serial.print("Phy mode:");
  Serial.println(WiFi.getPhyMode());
  Serial.print("Wifi mode:");
  Serial.println(WiFi.getMode());
  Serial.print("Reset reason:");
  Serial.println(ESP.getResetReason());
  Serial.print("Reset info:");
  Serial.println(ESP.getResetInfo());
#endif
#if defined(ARDUINO_ARCH_ESP32)
  Serial.print("CPU0 reset reason: ");
  verbose_print_reset_reason(rtc_get_reset_reason(0));
  Serial.print("CPU1 reset reason: ");
  verbose_print_reset_reason(rtc_get_reset_reason(1));
  
#endif

#endif


  led.begin();
  led.show();

  setled(255, 0, 0);
  // setup filesystem
  SPIFFS.begin();

  // read configs from files
  Smyname = readFromFile(fn_myname, Smyname);
  Sssid = readFromFile(fn_ssid, Sssid);
  Spass = readFromFile(fn_pass, Spass);
  Smqttserver = readFromFile(fn_mqttserver, Smqttserver);
  Smqttuser = readFromFile(fn_mqttuser, Smqttuser);
  Smqttpass = readFromFile(fn_mqttpass, Smqttpass);
  Smqttport = readFromFile(fn_mqttport, Smqttport);
  Ssite = readFromFile(fn_site, Ssite);
  Slocation = readFromFile(fn_location, Slocation);

#ifdef DEBUG
  printConfig();
#endif

  WiFi.persistent(false);
  WiFi.disconnect();
  delay(100);
  WiFi.mode(WIFI_STA);
#if defined(ARDUINO_ARCH_ESP8266)
  WiFi.hostname(Smyname);
#endif
  WiFi.begin(Sssid.c_str(), Spass.c_str());

#ifdef DEBUG
  Serial.print("Connecting to ");
  Serial.println(Sssid);
#endif

  setled(255, 128, 0);
  int retries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    retries++;
#ifdef DEBUG
    Serial.print("Wifi.status() = ");
    Serial.println(WiFi.status());
    if ((retries % 20) == 0) {
      WiFi.printDiag(Serial);
      Serial.setDebugOutput(true);
      WiFi.waitForConnectResult();
    }
#endif
  }

  setled(0, 255, 0);
  delay(100);


#ifdef DEBUG
  Serial.println("");
  Serial.println("Wifi connected");
  Serial.print("IP address: ");
  Serial.print(WiFi.localIP());
  Serial.print("/");
  Serial.println(WiFi.subnetMask());
#endif



  // setup i2c
  Wire.begin(I2CSDA, I2CSCL);

bme_setup();

//Setup real time clock
#if defined(DS3231)
  rtcSetup();
#endif

  // Setup Si7021
#if defined(SI7021)
  si7021.begin();
#endif

#ifdef TSL2561
  int isOK = 0;
  while (!isOK) { 
  lightTSL2561.begin();
#ifdef DEBUG
  unsigned char ID;
  if (lightTSL2561.getID(ID)) {
    Serial.print("Got TSL2561 ID: 0x");
    Serial.print(ID, HEX);
    Serial.println(", should be 0x5x");
    isOK = 1;
  } else {
    byte error = lightTSL2561.getError();
    Serial.print("Got TSL2561 Error: ");
    Serial.println(error);
  }
  }
#endif
  lightTSL2561.setTiming(gainTSL2561, 2, msTSL2561);
  lightTSL2561.setPowerUp();
#endif

// GY49 Light Sensor
#ifdef GY49
  Wire.beginTransmission(GY49);
  Wire.write(0x02);
  Wire.write(0x40);
  Wire.endTransmission();
#endif



  // setup the mqtt client
  client.setClient(espClient);
  client.setServer(Smqttserver.c_str(), atoi(Smqttport.c_str()));
  client.setCallback(callback);

#if defined(MPR121)
  cap.begin(MPR121);
#endif

  // Task Scheduler
  runner.init();
  runner.addTask(showSensor_task);
  runner.addTask(clock_task);
  runner.addTask(publishSensor_task);
  runner.addTask(timer_task);
  
  setled(0, 0, 0);
  publishSensor_task.enableDelayed();

  setMode(MODE_CLOCK);

  }

/* Verbal description of run modes:
 *  
 *  Clock:
 *    - clock update is called evera 0.5 seconds
 *    - every 3 minutes temperature and humidity is shown
 *    
 *  Timer:
 *    - timer is updated every 0.5 seconds
 *    - every 10 seconds time is shown for 3 seconds
 *    - every 3 minutes temperature and humidity is shown
 *    
 *  Timer Set:
 *    - clock is not shown
 *    - timer is set to 1 second and shown constantly
 *    - hour +/- and minute +/- adjust timer
 *    - enter starts the timer
 *    - cancel sets mode to ClockMode
 *    - after no action for one minute mode also changes to ClockMode
 *  
 */

void setMode (int mode) {
  if (mode == MODE_CLOCK) {
    //show clock, every 
    clock_task.setInterval(500);
    clock_task.setIterations(TASK_FOREVER);
    showSensor_task.setInterval(1000*3*60);
    timer_task.disable();
    showSensor_task.enableDelayed();
    clock_task.enable();
  } else if (mode == MODE_OFF) {
    clock_task.disable();
    showSensor_task.disable();
    timer_task.disable();
  } else if (mode == MODE_TIMER) {
    clock_task.disable();
    timer_task.setIterations(20);
    timer_task.setInterval(500);
    timer_task.enable();
    showSensor_task.enableDelayed();
  } else if (mode == MODE_SETTIMER) {
    clock_task.disable();
    timer_task.disable();
    showSensor_task.disable();
    setupTimer = TimeSpan(1);
    showTimer(setupTimer);
  }
  runMode = mode;
}

void callback(char* topic, byte* payload, unsigned int length)  {
#ifdef DEBUG
  Serial.print("Message arrived[");
  Serial.print(topic);
  Serial.print("]: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
#endif

  String in;
  for (int i = 0; i < length; i++) {
    in += String((char)payload[i]);
  }

  if (in.equals(String("reboot"))) {
    ESP.restart();
  } else if (in.startsWith("led ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int r = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int g = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int b = in.substring(position).toInt();
    setled(r, g, b);
  }

  // LEDs off
  else if (in.startsWith("ledsoff")) {
    setled(0);
  }

  // LED code in binary
  else if (in.startsWith("ledb ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    byte thisLED = 0;
    position++;
    while (position + 2 < length) {
      setled(thisLED, payload[position + 0], payload[position + 1], payload[position + 2], 0);
      position += 3;
      thisLED++;
    }
    setled(1);
  }
  else if (in.startsWith("ledn ")) {
    int position = 0;

    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int n = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int r = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int g = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int b = in.substring(position).toInt();
    setled(n, r, g, b);

  } else if (in.startsWith("monochrome ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int r = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int g = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int b = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int v = in.substring(position).toInt();
    monochrome(r, g, b, v);

  } else if (in.startsWith("flash ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int r = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int g = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int b = in.substring(position).toInt();
    position++;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    int count = in.substring(position).toInt();
    flashLeds(count,makeColor(r,g,b));
    // Location - note that change becomes active after reboot only
  } else if (in.startsWith("location ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_location, in.substring(position + 1));
    printConfig();
  }

  // Mqttserver
  else if (in.startsWith("mqttserver ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_mqttserver, in.substring(position + 1));
    printConfig();
  }

  // Mqttport
  else if (in.startsWith("mqttport ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_mqttport, in.substring(position + 1));
    printConfig();
  }

// Mqttuser
  else if (in.startsWith("mqttuser ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_mqttuser, in.substring(position + 1));
    printConfig();
  }

// Mqttserver
  else if (in.startsWith("mqttpass ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_mqttpass, in.substring(position + 1));
    printConfig();
  }


  // Site
  else if (in.startsWith("site ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    writeFile(fn_site, in.substring(position + 1));
    printConfig();
  }

  // Unix time if RTC is defined
  else if (in.startsWith("unixtime ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    uint32_t newnow = in.substring(position).toInt();
    printCurrentTime();
    rtc.adjust(DateTime(newnow));
    rtcSetup();
    printCurrentTime();
  }

  else if (in.startsWith("show temperature")) {
    int position = 15;
    
    while (position < in.length() && in.substring(position, position + 1) != " ") {
      position++;
    }
    if (position == 15) {
      showTemperature(-10,40,(int)getLocalTemperature());
    }
    else
      showTemperature(-10,40,in.substring(position).toInt());

  }

  else if (in.startsWith("show humidity")) {
    int position = 12;
    
    while (position < in.length() && in.substring(position, position + 1) != " ") {
      position++;
    }
    if (position == 12) {
      showHumidity((int)getLocalHumidity());
    }
    else
      showHumidity(in.substring(position).toInt());

  }

  else if (in.startsWith("set timer ")) {
    int position = 9;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    uint32_t interval = in.substring(position).toInt();
    setTimer(interval);
    setMode(MODE_TIMER);
  }

  else if (in.startsWith("adjust timer ")) {
    int position = 11;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    uint32_t interval = in.substring(position).toInt();
    adjustTimer(interval);
  }

  else if (in.startsWith("mode ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    uint32_t newmode = in.substring(position).toInt();
    setMode(newmode);
  }

  else if (in.startsWith("key ")) {
    int position = 0;
    while (in.substring(position, position + 1) != " " && position < in.length()) {
      position++;
    }
    uint32_t thiskey = in.substring(position).toInt();
    processKey(thiskey);
  }

  
  else if (in.startsWith("now")) {
    printCurrentTime();
  }

  else if (in.startsWith("clock")) {
    showTime();
  }
  else if (in.startsWith("startclock")) {
    clock_task.enable();
  }
  else if (in.startsWith("stopclock")) {
    clock_task.disable();
  }
  else if (in.startsWith("second0")) {
    DateTime thisnow = rtc.now();
    rtc.adjust(thisnow - TimeSpan(thisnow.second()));
    printCurrentTime();
  }
  else if (in.startsWith("minute-")) {
    rtc.adjust(DateTime(rtc.now()-TimeSpan(60)));
    printCurrentTime();
  }
  else if (in.startsWith("minute+")) {
    rtc.adjust(DateTime(rtc.now()+TimeSpan(60)));
    printCurrentTime();
  }
  else if (in.startsWith("hour-")) {
    rtc.adjust(DateTime(rtc.now()-TimeSpan(60*60)));
    printCurrentTime();
  }
  else if (in.startsWith("hour+")) {
    rtc.adjust(DateTime(rtc.now()+TimeSpan(60*60)));
    printCurrentTime();
  }
  else if (in.startsWith("hour")) {
    int position = 3;
    
    while (position < in.length() && in.substring(position, position + 1) != " ") {
      position++;
    }
    int hour = in.substring(position).toInt();

    position++;
    while (position < in.length() && in.substring(position, position + 1) != " ") {
      position++;
    }
    int minute = in.substring(position).toInt();

    DateTime t(2018,3,15,hour,minute,0);
    
    showTime(t);
  }
    
  else {
#ifdef DEBUG
    Serial.println("unknown command received: " + in);
#endif
  }
}


boolean reconnect() {
  // Loop until we're reconnected
  char mytopic[50];
  snprintf(mytopic, 50, "/%s/%s/status", Ssite.c_str(), Smyname.c_str());



#ifdef DEBUG
  Serial.print("Attempting MQTT connection...");
  Serial.print(client.state());
  Serial.print("...");
#endif
  // Attempt to connect
  if (client.connect(Smyname.c_str(),Smqttuser.c_str(),Smqttpass.c_str(),mytopic,0,0,"stopped")) {
  // if (client.connect(Smyname.c_str())) {
#ifdef DEBUG
    Serial.println("connected");
#endif
    // Once connected, publish an announcement...

    client.publish(mytopic, "started");
    delay(10);
    // ... and resubscribe to my name
    client.subscribe(Smyname.c_str());
    delay(10);
  } else {
#ifdef DEBUG
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" try again in 5 seconds");
#endif
  }
  return client.connected();
}

#ifdef LSSENSOR
/******************************************** Lichtsensor */
#define LS_I2C_ADDR     0x29
#define LS_REG_CONTROL  0x00
#define LS_REG_CONFIG   0x01
#define LS_REG_DATALOW  0x04
#define LS_REG_DATAHIGH 0x05
#define LS_REG_ID       0x0A

void ls_setup() {
#ifdef DEBUG
  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80 | LS_REG_ID);
  Wire.endTransmission();

  Serial.print("LS ID: ");
  Wire.requestFrom(LS_I2C_ADDR, 1); // request 1 byte
  while (Wire.available()) {
    unsigned char c = Wire.read();
    Serial.print( c & 0xF0, HEX);
  }
  Serial.println();
#endif

  delay(100);
  // Power on
  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80 | LS_REG_CONTROL);
  Wire.write(0x03); //power on
  Wire.endTransmission();
  delay(100);

  // Config
  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80 | LS_REG_CONFIG);

  Wire.write(LS_FACTOR_M - 1);
  // Wire.write(0x00); //M=1 T=400ms
  // Wire.write(0x01); //M=2 T=200ms
  // Wire.write(0x02); //M=4 T=100ms
  Wire.endTransmission();
}

void ls_shutdown() {
  // Power off
  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80 | LS_REG_CONTROL);
  Wire.write(0x00); //power on
  Wire.endTransmission();
}

uint32_t ls_read() {
  uint16_t l, h;
  uint32_t lux;

  Wire.beginTransmission(LS_I2C_ADDR);
  Wire.write(0x80 | LS_REG_DATALOW);
  Wire.endTransmission();
  Wire.requestFrom(LS_I2C_ADDR, 2); //request 2 bytes
  l = Wire.read();
  h = Wire.read();
  while (Wire.available()) {
    Wire.read();  //received more bytes?
  }
  lux  = (h << 8) | (l << 0);
  lux *= LS_FACTOR_M;
  // lux *= 1; //M=1
  // lux *= 2; //M=2
  // lux *= 4; //M=4
#ifdef DEBUG
  Serial.print("Lux: ");
  Serial.println(lux, DEC);
#endif

  return lux;
}
#endif

/******************************************** Wettersensor */
BME280 wetterSensor;
void bme_setup() {
#ifdef BME280ADDR
  wetterSensor.settings.commInterface = I2C_MODE;
  wetterSensor.settings.I2CAddress = BME280ADDR;
  wetterSensor.settings.runMode = 3;
  wetterSensor.settings.tStandby = 0;
  wetterSensor.settings.filter = 0;
  wetterSensor.settings.tempOverSample = 1;
  wetterSensor.settings.pressOverSample = 1;
  wetterSensor.settings.humidOverSample = 1;

  delay(10);
#ifdef DEBUG
  Serial.print("Starting BME280... result of .begin(): 0x");
  Serial.println(wetterSensor.begin(), HEX);
#else
  wetterSensor.begin();
#endif
#endif
}


unsigned long lastReconnectAttempt = 0;
void myPublish(char *topic, char *msg) {
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  client.loop();
#ifdef DEBUG
  Serial.print("Publish message: ");
  Serial.print(topic);
  Serial.print(" ");
  Serial.println(msg);
#endif
  client.publish(topic, msg);
}

// if any sensor for temperature is defined, return temperature
float getLocalTemperature() {
#if defined(BME280ADDR)
  return wetterSensor.readTempC();
#elif defined(SI7021)
  return si7021.readTemperature();
#else
  return 0;
#endif
}

float getLocalHumidity() {
#if defined(SI7021)
  return si7021.readHumidity();
#elif defined(BME280ADDR)
  return wetterSensor.readFloatHumidity();
#else
  return 0;
#endif
  
}

double getBrightness() {
#ifdef TSL2561
    unsigned int tsldata0, tsldata1;
    if (lightTSL2561.getData(tsldata0, tsldata1)) {
      double lux;
      boolean good;
      good = lightTSL2561.getLux(gainTSL2561, msTSL2561, tsldata0, tsldata1, lux);
      if (good) {
        return lux;
      } 
    }
#endif
  return -1;
}

void publishSensor() {
  #if defined(BME280ADDR) || defined(BME680ADDR) || defined(SI7021)
    snprintf(topic, 50, "/%s/%s/temperature", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%s", String(getLocalTemperature(), 2).c_str()); 
    if (value > 2) {
      myPublish(topic, msg);
    }

    snprintf(topic, 50, "/%s/%s/humidity", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%s", String(getLocalHumidity(), 2).c_str());
    if (value > 2) {
      myPublish(topic, msg);
    }
#endif

#ifdef BME280ADDR
    snprintf(topic, 50, "/%s/%s/airpressure", Ssite.c_str(), Slocation.c_str());
    snprintf(msg, 50, "%s", String(wetterSensor.readFloatPressure() / 100, 2).c_str());
    if (value > 2) {
      myPublish(topic, msg);
    }

#endif

    Brightness = getBrightness();
    if (Brightness >= 0) {
      snprintf(topic, 50, "/%s/%s/light", Ssite.c_str(), Slocation.c_str());
      snprintf(msg, 50, "%u", (unsigned int) Brightness);
      if (value > 2) {
        myPublish(topic, msg);
      }
    }


  value++;
}

void processKey(uint32_t button) {
  if (button & BUTTON_TEMPERATURE) {
    showTemperature();
  }

  if (button & BUTTON_HUMIDITY) {
    showHumidity();
  }

  if (runMode == MODE_SETTIMER) {
    if (button & BUTTON_HOURPLUS) {
      setupTimer = setupTimer + TimeSpan(0,1,0,0);
    }
    if (button & BUTTON_HOURMINUS) {
      setupTimer = setupTimer - TimeSpan(0,1,0,0);
    }
    if (button & BUTTON_MINUTEPLUS) {
      setupTimer = setupTimer + TimeSpan(0,0,1,0);
    }
    if (button & BUTTON_MINUTEMINUS) {
      setupTimer = setupTimer - TimeSpan(0,0,1,0);
    }
    if (button & BUTTON_CANCEL) {
      setMode(MODE_CLOCK);
    }
    else if (button & BUTTON_ENTER) {
      setTimer(setupTimer);
      setMode(MODE_TIMER);
    }
    else 
      showTimer(setupTimer);
  }
}


void processKey(void) {
#if defined(MPR121)
  currtouched = cap.touched();
#ifdef DEBUG
  if (currtouched != lasttouched) {
    Serial.print("curtouched = ");
    Serial.println(currtouched);
  }
#endif

  for (uint32_t thisbutton = 1; thisbutton <= (1<<12); thisbutton <<= 1) {
    if ((currtouched & thisbutton) && !(lasttouched & thisbutton)) {
      processKey(thisbutton);
    }
  }

  lasttouched = currtouched;
#endif
}  


unsigned long int loopDelay = 2000;
unsigned long now;
int lastMotion = 0, thisMotion = 0;

void loop() {
  runner.execute();

  if (runMode == MODE_TIMER && timer_task.isEnabled() && timer_task.getIterations() < 2) {
    clock_task.setIterations(8);
    clock_task.enableDelayed(500);
  }
  if (runMode == MODE_TIMER && clock_task.isEnabled() && clock_task.getIterations() < 2) {
    timer_task.setIterations(20);
    timer_task.enableDelayed(500);
  }


  processKey();
  Brightness = getBrightness();

  if (!client.connected()) {
    now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  }
  client.loop();

}



