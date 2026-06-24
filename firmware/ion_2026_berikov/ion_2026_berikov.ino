//
//  The MIT License (MIT)
//
// Copyright (c) 2026 Frank Peri for Cubes in Space
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// Update History
// 11/26/2023:
// Fork that combines ION_Circularbuffertest.ino + ION_FLIGHT_2024.ino
// 1: refactored to use FSM library - significanly simplifies logic
// 2: first attempt to push Rocket and Balloon codes together - #defines used to determine if LORA and GPS enabled
// 3: similarly, combining M0, nRF52 and ESP32 elements into one codesets - complete - established #if defined's to block code (LED vs. NEOPIXEL)
// 4: settled on original IMU from balloon 2023 mission (LSM6D+LIS3), rather than using BNOxx - added Adafruit AHRS to get RPY
// 5: using RTC from balloon - added logic for deciding if RTC or GPS will be used for timing if both RTC and GPS installed
// 6: refined buffer from ION_Circularbuffertest.ino to be more efficient - moving pointers rather than using memcopy
// 7: added functions to simulate ax, deltaH variations for rocket
//
// WORK AHEAD:
// 1 - decide how to add FLOAT state to balloon, in lieu of DEPLOY for rocket - are they the same? - CLOSED: removed deploy state from balloon
// 2 - close various ACTIONs throughout - CLOSED
// 3 - simple and elegant way to pull data off FLASH for analysis - currently can list to Serial, need to capture to a file - CLOSED: added a web interface qith ability to download files from FLASH - CLOSED, using web
//
// 09/18/2024: Updates - FSM_demo_copy_20240918123502_copy_20240918123559
// 1: evaluate triggers for FSM that transition from Standby to Launch based upon experience handling Rocket at FF 2024 (too easy to trogger launch during routine handling)
//
// 10/08/24: Update - event_demo <- FSM_demo_copy_20240918123502
// refactoring into event-driven logic rather than states - triggers for transitions are the same, simply using the already existing flightStatus register to check and set events
// FSM worked fine, but was overkill. Events include: launch, deploy, land
// System boots into Command Mode (flightStatus = 0x80) which disables any event checking. CmdMode allows file operations (download and delete)
// Must command into Standby (flightStatus = 0x00) to enable event checking
// Command Mode is used Modes include: command, standby, missionend. Note I have not implemented groundmode nor SIM in this update.
//
// 6/2025 updates "event_demo_copy.ino"
// a) updated trigger logic
// b) removed Ambient Sensor measurement (all references to steinhart commented out - simply not practical to incluide external thermistors on ION
// c) Integrated Calibration code into this codeset (it was integrated in earlier versions, but pulled to reduce code size). Will examine code size before
// deciding to include Calibration here. Pulling code from ION_Calibration_FINAL_Rev1. Calibration is assessible via the serial processor (enter Y within the first 8 seconds of
// poweron). Had to reorganize setup() to init flash and cal before calling calibration during Serial poll sequence - thereby nulling some Serial outputs
// d) added SerialhandleFileList routine to setup since this seems to fix the random flash errors in prior versions
// e) using elapsedmillis() for getdata routine - seems to avoid data overrun errors - future work: incorporate elapsedmillis for other timers (writedata and LoRA)
// f) Minor updates to indicator light notifications
//
// Summary of indicator notifications:
// err(2, RED) - calibration routine error - rare - CONSIDER DELETING THIS NOTIFICATION 6-3-2025
// err(2, RED) - Failed to sync time error
// err(3, RED) - file open error
// err(4, RED) - flash file system error
// err(5, RED) - Wire interface error - rare - CONSIDER DELETING THIS NOTIFICATION 6-3-2025
// err(2, YELLOW) - IMU, BMP or RTC error - likely a wiring error
// err(3, YELLOW) - LoRA error
//
// Summary of status indicator:
// DKGREEN Steady - in STANDBY mode
// MAGENTA Steady - in CMD mode
// ORANGE Steady - launched
// PINK Steady - deployed (float)
// WHITE Steady - landed
// YELLOW 1 blink repeat - post boot waiting on input to Serial (times out after 8 sec)
// BLUE Steady - GPS acquired time signal during setup()
//
// 7/24/25 updates: "event_demo_copy_copy_20250724232525.ino"
// Addresses two uncertainties 1) the 2025 mission has more ION's than battery capacity for full duration of the mission, therefore
// likely to run out of power before the mission completes, and 2) event triggers are not perfect, especially deploy (aka float) and
// land, therefore continue to collect and sync data to datafile and do not close file
//
// 1) added functionality to periodically save data to memory to ensure no data lost due to power outage - I did this for the SD card
//    version - don't know why I didn't consider this earlier
// 2) improved smoothing and derivative functions - using SavLay library to replace median filters
// 3) improved EVENT triggers
// 4) commented out closefile command in closeFile()
//
//  BASELINE FOR 2025 CiS BALLOONN MISSION and copied to ION_Balloon_Flight_2025-as-flown.ino
//
// Future work:
// 1) review EVENT triggers for Rocket - battery is less likely a problem for Rocket, so may uncomment closefile command
// 2) review dataMessage tht is transmitted to Groundstation - likely update to Groundstation required to decode dataMessage
// 3) update to determine 3D velocity
//
// 02/16/2026 update to simplify for 2026
// Starting with baseline: "event_demo_copy_copy_20250724232525_copy_20250803194217.ino"
// Motivation:
// - Use deep sleep function of esp32 to save battery and storage
// - simplify operations
// - eliminate EVENT triggers based upon flight regime changes (launch, float, land)
//
// Concept:
// Use esp32 deep sleep feature to save battery: sleep cycle: 1 min on, 10 min off
// 1) Boot
// 2) Start RTC, read uniqueID, prep memory, init sensors, etc
// 3) Start serial
// 4) Check serial
// 5a) if no serial input
// 5a1) open file for append
// 5a2) take data for 1 min, store in file, close file - NOTE 1 min TBR depending on measured power consumption
// 5a3) go to deep sleep for 14 min - NOTE 14 min TBR depending on measured power consumption
// 5a4) waking from deep-sleep returns to 2)
// 5b) if serial input (Y)
// 5b1) parse input: C for calibrate, W for webserver
// 5b2) After choosing C, start calibration then goto standby mode
// 5b1) After choosing W, infinte while loop around webserver (add a esp.reset to web interface)
// 5b2) process serial commmand as in prior version

// in serial mode can take data but not save data to memory
// all web functions as in prior version - allows for downloading data and reviewing data collection in RT

// 3/1/2026: changes to SdFat code broke webserver.h. This is documented at:
// https://github.com/greiman/SdFat/issues/337#issuecomment-949610330
// Beginning at about line 90 the following changes must be made to ArduinoFiles.h:
// ----- begin changes (note lines 90-93 are commented out and new definiotn for getName replaced)
// FROM:
// #ifndef DOXYGEN_SHOULD_SKIP_THIS
//   char* __attribute__((error("use getName(name, size)"))) name();
// #endif  // DOXYGEN_SHOULD_SKIP_THIS
//
// TO:
// //#ifndef DOXYGEN_SHOULD_SKIP_THIS
// //  char* __attribute__((error("use getName(name, size)"))) name();
// //#endif  // DOXYGEN_SHOULD_SKIP_THIS
//  char *name()
//  {
//    BaseFile::getName(m_name, sizeof(m_name));
//    return m_name;
//  }
//  char m_name[256];
// ----- end changes
//
// FUTURE WORK:
// 3/4/2026:
// 1) consider an error register. Baseline behavior is to terminate operation if an error arises (typically hw)
//    in some cases this is overkill - for example, a pmon error might be ok to proceed, noting that voltage measurements
//    will be inaccurate. Others may include calibration sw routine error, timesync error. Could look like:
//      uint8_t errorStatus = 0x00;
//  b     b      b         b      b      b      b      b
//  pmon  calsw  timesync  other  other  other  other  other
//  of course logging errors does no good if we can't read them, so the error register must be added to the datastream, so not a trivial change
//
//
// --------------------------- README ---------------------------
//
// In 2026, CiS offfered students the ability to change sensors for the balloon mission. Comments are added in code to indicate where changes to code must be made.
// Comments will be noted by: //@SENSOR
// We are not allowing changes to datastream so no sensors can be added, only swapped. This has an impact to mode changes if imu is delected (which is another good
// reason to eliminate that operational mode for balloon variant)
//

#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3)
//#include <driver/adc.h>  // for ESP32
#endif

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
#include <avr/dtostrf.h>                                                                                      // for M0
#include <Fadinglight.h>                                                                                      // LED version
#endif

#include <SPI.h>
#include <SdFat.h>
#include <BMP388_DEV.h>                   //@SENSOR
#include <Adafruit_MCP9808.h>             //@SENSOR //@BERIKOV under-regolith temperature sensor (replaces IMU)
#include <Adafruit_Sensor.h>             //@BERIKOV provides sensors_event_t (was pulled in via IMU/AHRS headers)
//@BERIKOV IMU/magnetometer/AHRS not used in our configuration
//#include <Adafruit_LSM6DSOX.h>            //@SENSOR
//#include <Adafruit_LIS3MDL.h>             //@SENSOR
//#include <Adafruit_AHRS.h>                //@SENSOR
#include <Adafruit_Sensor_Calibration.h>  //@SENSOR
#include "Adafruit_LC709203F.h"
#include "Adafruit_MAX1704X.h"
#include <NeoFader.h>
#include <TimeLib.h>
#include <RTClib.h>
#include <ArduinoUniqueID.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS - so many GPS libraries, I like this the best
#include <LoRa.h>
#include <Adafruit_SPIFlash.h>
#include <WiFi.h>
#include <WebServer.h>
#include <elapsedMillis.h>
#include <SavLayFilter.h>
#include <esp_wifi.h>

// necessary to address esp3.0.1 library changes - ugh
#define FILE_READ O_RDONLY
#define FILE_WRITE (O_RDWR | O_CREAT | O_AT_END)

//
// HARDWARE CONFIGURATION DEFINITIONS
//
#define BALLOON         // comment out for rocket version
//#define LORA_ENABLED  // uncomment for LoRa version only for Rocket
//#define GPS_ENABLED   // uncomment for GPS version only for Rocket

//
// status register definitions
//
#define LAUNCH 0      // bit 0 of flightStatus register (Rightmost bit) - indicates Launch detected
#define DEPLOY 1      // bit 1 of flightStatus register - indicates Deploy (or hard G) detected
#define LAND 2        // bit 2 of flightStatus register - indicates Landing detected
#define GPSFIX 3      // bit 3 of flightStatus register - indicates GPS fixed - informational only, not used for any event checking
#define MISSIONEND 4  // bit 4 of flightStatus register - indicates mission ended
#define UNUSED 5      // bit 5 of flightStatus register - unused
#define WRITEFLASH 6  // bit 6 of flightStatus register - 1 = write to FLASH storage: 0 = write to circular buffer
#define CMDMODE 7     // bit 7 of flightStatus register - indicates cmd mode
//  b         b          b        b             b         b         b           b
//  CmdMode   WriteFlash GndTest  missionEnd    GPSFix    landFlag  deployFlag  launchFlag

//
// HARDWARE PIN DEFINITIONS
//
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3)
#define LORA_CS A1
#define LORA_IRQ SDA
#define LORA_RESET SCL
#define VBATPIN A2  // analog pin for external battery voltage measurement on QT PY

// The first 2 here are boards w/radio BUILT-IN (Note the RP2040 board does not work with the BNO085 IMU)
// Boards using FeatherWing follow those. Note only nRF52840 express and adafruit_feather_m0 have been tested
#elif defined(__AVR_ATmega32U4__)  // Feather 32u4 w/Radio
#define LORA_CS 8
#define LORA_IRQ 7
#define LORA_RESET 4

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
#define LORA_CS 8
#define LORA_IRQ 3  // or is it 7?
#define LORA_RESET 4
#define VBATPIN A7  // internal voltage divider at this pin
#define VREF 3.3

#elif defined(__AVR_ATmega328P__)  // Feather 328P w/wing
#define LORA_CS 4                  //
#define LORA_IRQ 3                 //
#define LORA_RESET 2               // "A"

#elif defined(ESP8266)  // ESP8266 feather w/wing
#define LORA_CS 2       // "E"
#define LORA_IRQ 15     // "B"
#define LORA_RESET 16   // "D"

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
#define LORA_CS 10     // "B"
#define LORA_IRQ 6     // "D"
#define LORA_RESET 11  // "A"
#define VBATPIN A6     // internal voltage divider at this pin for nRF52840, ESP32 use LC device
#define VREF 3.6

#elif defined(ESP32)  // ESP32 feather w/wing
#define LORA_CS 33    // "B"
#define LORA_IRQ 27   // "A"
#define LORA_RESET 13

#elif defined(ARDUINO_NRF52832_FEATHER)  // nRF52832 feather w/wing
#define LORA_CS 11                       // "B"
#define LORA_IRQ 31                      // "C"
#define LORA_RESET 7                     // "A"
#endif                                   // HW PIN DEFINITIONS
// end pin definitions

// deep sleep definitions
#define mS_TO_S_FACTOR 1000ULL     // Conversion factor for microseconds to seconds (ULL makes is an uint64_t)
#define uS_TO_S_FACTOR 1000000ULL  // Conversion factor for microseconds to seconds (ULL makes is an uint64_t)
#define SLEEP_TIME 60            // 600 sec (10 min) for sleep time while testing
#define AWAKE_TIME 60              // 60 sec to stay awake

// hardware structures
//@BERIKOV IMU/magnetometer not installed in our configuration
//Adafruit_LSM6DSOX lsm;  // IMU data structure           //@SENSOR
//Adafruit_LIS3MDL lis;   // magentometer data structure  //@SENSOR
Adafruit_MCP9808 mcp9808;          // under-regolith temperature sensor  //@SENSOR //@BERIKOV
float mcpTempUnderRegolith = 0.0;  // MCP9808 reading, routed into the ax datastream field  //@BERIKOV
Adafruit_LC709203F lc;  // only for ESP32-S3
Adafruit_MAX17048 maxlipo;
bool addr0x36 = true;  // MAX17048 i2c address

// sw structures
// QT PY uses Wire1 for STEMMA connector, so redefine Wire as Wire1 so sensor init statements don't need to change
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3)
#define WIRE Wire1
BMP388_DEV bmp388(SDA1, SCL1, WIRE);  // ESP QT PY uses Wire1 interface
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM)
#define WIRE Wire
BMP388_DEV bmp388(SDA, SCL, WIRE);
#endif

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
Adafruit_Sensor_Calibration_EEPROM cal;
#else
Adafruit_Sensor_Calibration_SDFat cal;
#endif

// love this timer library
elapsedMillis getDataTimer;
elapsedMillis flashWriteTimer;
elapsedMillis writeDataTimer;
elapsedMillis LoRaXmitTimer;

// data structures for sensor events (an Adafruit concept) when data is available) //@SENSOR
sensors_event_t accel_event, gyro_event, temp_event, mag_event;

// choose IMU filter: slower = better quality output
//@BERIKOV AHRS filter not used without IMU
//Adafruit_NXPSensorFusion filter;  // slowest - RPY seem more stable, doesn't seem "slow"  //@SENSOR
//Adafruit_Madgwick filter;  // faster than NXP
//Adafruit_Mahony filter;  // fastest/smalleset - RPY seem to drift with this filter

WebServer webserver(80);

#if defined(GPS_ENABLED)
SFE_UBLOX_GNSS myGNSS;
#endif

// various flash memory contructs
Adafruit_FlashTransport_ESP32 flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatVolume fatfs;
File32 dataFile;
File32 root;
File32 file;

// real-time clock constructs
RTC_DS3231 rtc;
time_t getRtcTime();
time_t getGPSTime();
time_t launchTime;
tmElements_t tm;

// Offset hours from gps time (UTC)
//const int offset = 1;   // Central European Time
//const int offset = -5;  // Eastern Standard Time (USA)
const int offset = -4;  // Eastern Daylight Time (USA)
//const int offset = -8;  // Pacific Standard Time (USA)
//const int offset = -7;  // Pacific Daylight Time (USA)
//const char zone[] = "EST";  // "EDT"

// constant definitions
const uint16_t TIME_SYNC_INTERVAL = 300;  // 300 = 5 min, testing drift with long sync interval
const uint16_t SAMPLE_INTERVAL = 10;      // 10msec data sampling rate
const uint16_t FILTER_UPDATE_RATE_HZ = 100;
const uint16_t WRITE_BUFFER_LENGTH = 5;
const uint16_t MESSAGE_LENGTH = 270;
const uint16_t LAND_INTERVAL = 5000;

// various constants used by event logic
//#ifdef BALLOON - this approach superceded by implementing low power mode. No longer have to slow down write interval to save ram
//const uint16_t WRITE_INTERVAL = 10000;  // for balloon = 10 sec - remember, always sampling data at 100 Hz (10msec)
//const int8_t ALTITUDE_THRESHOLD = 100;  // 100 // start saving data above 100m for balloon
//const uint32_t SAVE_INTERVAL = 10000L;  // sync dataFile every 10 seconds
//#else
const uint16_t WRITE_INTERVAL = 100;  // for rocket launch = 0.1 sec - remember, always sampling data at 100 Hz (10msec)
const int8_t ALTITUDE_THRESHOLD = 1;  // start saving data above 1m for rocket
const uint32_t SAVE_INTERVAL = 500;   // sync dataFile every 500 msec
//#endif
const int8_t LAUNCH_THRESHOLD = 14;  // accel at rocket launch
const int8_t DEPLOY_THRESHOLD = 15;  // accel at rocket ejection charge
const int8_t ALTITUDE_ERROR = 1;     // deadband for measuring FLOAT event
const int8_t LAND_THRESHOLD = 0;     // delta altitude, not moving, therefore landed

#if defined(LORA_ENABLED)
const uint8_t localAddress = 0xDD;  // address of this device
const uint8_t destination = 0xBB;   // destination to send to
const uint32_t XMTRATE = 500;       // 500 msec
#endif

// variable definitions
uint32_t vBattery;                                      // external battery voltage

// sensor variable definitions                          //@SENSOR
float gx, gy, gz, roll, pitch, yaw;                     // 3-axis accels and rotations
float gravX, gravY, gravZ;                              // gravity vector = 1 if axis is pointing opposite gravity, -1 if axis is pointing toward gravity
float temperature, pressure, altitude, altitudeApogee;  // Create the temperature, pressure and altitude variables
float sealevelPressure;                                 // will be determined after boot
float daltitude;                                        // used for computed derivative
float padElevation;                                     // altitude at pad (prior to launch)
double doublealtitude;
double smoothaltitude;
double doublebattery;
double smoothbattery;
SavLayFilter altitudeSmoother(&doublealtitude, 0, 5);    // 5-point smoother
SavLayFilter altitudeDerivative(&smoothaltitude, 1, 7);  // Computes the first derivative of the altitude
SavLayFilter batterySmoother(&doublebattery, 0, 5);      // 5-point smoother

const byte numChars = 35;
char filename[numChars] = { 0 };
char endMarker = '\n';

char writeBuffer[WRITE_BUFFER_LENGTH][MESSAGE_LENGTH];  // circular buffer to store data prior to launch

uint8_t flightStatus = 0x00;                            // start in Standby
//  b         b          b        b             b         b         b           b
//  CmdMode   WriteFlash GndTest  missionEnd    GPSFix    landFlag  deployFlag  launchFlag

bool RTCOk = false;
bool GPSOk = false;
bool dumpFile = false;
bool listFiles = false;
#ifdef BALLOON
bool copiedBuffer = true;  // don't need circular buffer for BALLOON
#else
bool copiedBuffer = false;
#endif

RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool baselinePressSet = false;   // save this in RTC memory to we don't recalc baseline pressure after waking from sleep
RTC_DATA_ATTR char datafilename[33] = "ION-";  // typ: ION-7CDFA1-20240129-182154.log

char actionfile[33];
char action[9];
char UniqueIDString[10];
char dateStr[16];  // YYYYMMDD-HHMMSS+terminator
char buff[22];     // temp array for rx/tx cmd strings

// indicator macros for different platforms
#if defined(ESP32) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM)
NeoFader neo;  // for ESP32
#else
Fadinglight led(LED_BUILTIN);                          // create instance of led for error codes for M0 that doesn't have NeoPixel
#endif

// HTML for webpage
String SendHTML() {
  String ptr = "<!DOCTYPE HTML>";
  ptr += "<html>";
  ptr += "<head>";
  ptr += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">";
  ptr += "<title>ION Data</title>";
  ptr += "<style>";
  ptr += "html{font-family: 'Open Sans', sans-serif; display: block; margin: 0px auto; text-align: center;color: #444444;}";
  ptr += "body{margin-top: 0px;} h1 {margin: 5px auto 5px;}";
  ptr += ".side-by-side{display: table-cell;vertical-align: right;position: relative;}";
  ptr += ".text{font-weight: 400;font-size: 16px;width: 150px;}";
  ptr += ".reading{font-weight: 300;font-size: 20px;padding-right: 15px;}";
  ptr += ".altitude .reading{color: #955BA5;}";
  ptr += ".pressure .reading{color: #955BA5;}";
  ptr += ".temperature .reading{color: #955BA5;}";
  ptr += ".ax .reading{color: #0000ff;}";
  ptr += ".ay .reading{color: #0000ff;}";
  ptr += ".az .reading{color: #0000ff;}";
  ptr += ".roll .reading{color: #0000ff;}";
  ptr += ".pitch .reading{color: #0000ff;}";
  ptr += ".yaw .reading{color: #0000ff;}";
  ptr += ".vbat .reading{color: #008800;}";
  ptr += ".superscript{font-size: 17px;font-weight: 400;position: absolute;top: 0px;}";
  ptr += ".data{padding: 2px;}";
  ptr += ".inputBtn{width: 25%;background-color: #008800;border: none;color: white;padding: 10px 10px;text-decoration: none;font-size: 14px;margin: 5px;cursor: pointer;border-radius: 4px; box-shadow: 4px 2px 12px 1px rgba(140,140,140,.5);}";
  ptr += ".container{display: table;margin: 0 auto;}";
  ptr += ".button {display: inline; width: 50%;background-color: #3498db;border: none;color: white;padding: 10px 10px;text-decoration: none;font-size: 14px;margin: 5px;cursor: pointer;border-radius: 4px; box-shadow: 4px 2px 12px 1px rgba(140,140,140,.5);}";
  ptr += ".button-on {background-color: #008800;}";
  ptr += ".button-off {background-color: #880000;}";
  ptr += ".status {margin: 8px 5px 8px 5px}";
  ptr += ".status-on {background-color: #00ff00;}";
  ptr += ".status-off {background-color: #ff0000;}";
  ptr += "</style>";

  if (!bitRead(flightStatus, CMDMODE)) {  // only show realtime data before launch OR if not in CMDMODE
    ptr += "<script>";
    ptr += "setInterval(loadDoc,250);";  // update webpage content 250msec
    ptr += "function loadDoc() {";
    ptr += "var xhttp = new XMLHttpRequest();";
    ptr += "xhttp.onload = function() {";
    ptr += "if (this.readyState == 4 && this.status == 200) {";
    ptr += "document.body.innerHTML = this.responseText}";
    ptr += "};";
    ptr += "xhttp.open(\"GET\", \"/\", true);";
    ptr += "xhttp.send();";
    ptr += "}";
    ptr += "</script>";
  }

  ptr += "</head>";
  ptr += "<body>";
  ptr += "<h1>ION Data</h1>";
  ptr += "<div class='container'>";
  ptr += "<div class='data altitude'>";
  ptr += "<div class='side-by-side text'> Altitude</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += smoothaltitude;  //@SENSOR
  ptr += "<span class='superscript'>m</span></div>";
  ptr += "</div>";
  ptr += "<div class='data altitude'>";
  ptr += "<div class='side-by-side text'> Alt Change</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += daltitude;  //@SENSOR
  ptr += "<span class='superscript'>m/sec</span></div>";
  ptr += "</div>";
  ptr += "<div class='data pressure'>";
  ptr += "<div class='side-by-side text'> Pressure</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += pressure;  //@SENSOR
  ptr += "<span class='superscript'>hPa</span></div>";
  ptr += "</div>";
  ptr += "<div class='data temperature'>";
  ptr += "<div class='side-by-side text'> Board Temp</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += temperature;  //@SENSOR
  ptr += "<span class='superscript'>C</span></div>";
  ptr += "</div>";
  ptr += "<div class='data ax'>";
  ptr += "<div class='side-by-side text'> Accel X</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += accel_event.acceleration.x;  //@SENSOR
  ptr += "<span class='superscript'>m/s^2</span></div>";
  ptr += "</div>";
  ptr += "<div class='data ay'>";
  ptr += "<div class='side-by-side text'> Accel Y</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += accel_event.acceleration.y;  //@SENSOR
  ptr += "<span class='superscript'>m/s^2</span></div>";
  ptr += "</div>";
  ptr += "<div class='data az'>";
  ptr += "<div class='side-by-side text'> Accel Z</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += accel_event.acceleration.z;  //@SENSOR
  ptr += "<span class='superscript'>m/s^2</span></div>";
  ptr += "</div>";
  ptr += "<div class='data roll'>";
  ptr += "<div class='side-by-side text'> Roll</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += roll;  //@SENSOR
  ptr += "<span class='superscript'>&deg</span></div>";
  ptr += "</div>";
  ptr += "<div class='data pitch'>";
  ptr += "<div class='side-by-side text'> Pitch</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += pitch;  //@SENSOR
  ptr += "<span class='superscript'>&deg</span></div>";
  ptr += "</div>";
  ptr += "<div class='data yaw'>";
  ptr += "<div class='side-by-side text'> Yaw</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += yaw;  //@SENSOR
  ptr += "<span class='superscript'>&deg</span></div>";
  ptr += "</div>";
  ptr += "<div class='data vbat'>";
  ptr += "<div class='side-by-side text'> Vbat</div>";
  ptr += "<div class='side-by-side reading'>";
  ptr += smoothbattery;
  ptr += "<span class='superscript'>V</span></div>";
  ptr += "&nbsp&nbsp";
  ptr += "</div>";
  ptr += "</div>";

  ptr += "<div>";
  if (bitRead(flightStatus, GPSFIX))
    ptr += "<span class=\"status status-on\">GPSFix</span>";
  else
    ptr += "<span class=\"status status-off\">GPSFix</span>";

  if (bitRead(flightStatus, CMDMODE))
    ptr += "<span class=\"status status-off\">Realtime</span>";
  else
    ptr += "<span class=\"status status-on\">Standby</span>";

  if (bitRead(flightStatus, LAUNCH))
    ptr += "<span class=\"status status-on\">Launch</span>";
  else
    ptr += "<span class=\"status status-off\">Launch</span>";

  if (bitRead(flightStatus, DEPLOY))
    ptr += "<span class='status status-on'>Deploy</span>";
  else
    ptr += "<span class='status status-off'>Deploy</span>";

  if (bitRead(flightStatus, LAND))
    ptr += "<span class=\"status status-on\">Land</span>";
  else
    ptr += "<span class=\"status status-off\">Land</span>";
  ptr += "</div>";
  ptr += "<br>";

  if (!bitRead(flightStatus, CMDMODE)) {  // if cmdmode is not set
    ptr += "<a class=\"button button-on\" href=\"/cmdmode\">Enter Command Mode</a>";
  } else {
    ptr += "<a class=\"button button-off\" href=\"/standby\">Return to Standby</a>";
  }
  ptr += "<a class=\"button button-on\" href=\"/restart\">Restart ION</a>";
  ptr += "<h3>File list</h3>";
  char temp[33];
  long dataUsed = 0;
  root.rewind();
  root.open("/");
  while (file.openNext(&root, O_RDONLY)) {
    dataUsed += file.size();
    ptr += file.size();
    ptr += "&nbsp&nbsp";
    file.getName(temp, sizeof(temp));
    ptr += temp;
    ptr += "<br>";

    //  ptr += "<div class='listing size'>";
    //  ptr += file.size();
    //  ptr += "</div>";
    //  ptr += "<class='listing name'>";
    //  ptr += temp;
    //  ptr += "</div>";

    file.close();
  }
  ptr += "<br>Used: ";
  ptr += dataUsed;
  ptr += "&nbsp&nbspFree: ";
  ptr += (flash.size() - dataUsed);
  ptr += "<br><br>";

  if (bitRead(flightStatus, CMDMODE)) {
    ptr += "<form action=\"/get\">";
    ptr += "<label for=\"fname\">Filename:</label>";
    ptr += "<input type=\"text\" id=\"fname\" name=\"fname\"><br><br>";
    ptr += "<input class=\"button button-on\" type=\"submit\" name=\"action\" value=\"Download\">";
    ptr += "<input class=\"button button-on\" type=\"submit\" name=\"action\" value=\"Delete\">";
    ptr += "</form>";
  }
  ptr += "</body>";
  ptr += "</html>";
  return ptr;
}
const char *PARAM_INPUT_1 = "actionfile";
//
// routine to collect data from sensors
//
void getData() {
  char dataMessage[MESSAGE_LENGTH] = "";
  static uint32_t landTime;

  if (getDataTimer >= SAMPLE_INTERVAL) {  // if time is right read GPS and sensors
#if defined(GPS_ENABLED)
    myGNSS.getPVT();
    if (myGNSS.getSIV())
      bitSet(flightStatus, GPSFIX);  // set GPSFIX register bit
    else
      bitClear(flightStatus, GPSFIX);
#endif
    //@SENSOR
    //@BERIKOV IMU/magnetometer/AHRS removed (no IMU). Read MCP9808 under-regolith temperature instead.
    //         All IMU-derived variables (gx/gy/gz, roll/pitch/yaw, accel_event/mag_event) stay 0.
    mcpTempUnderRegolith = mcp9808.readTempC();  // under-regolith temperature (degC)  //@SENSOR //@BERIKOV
    //lsm.getEvent(&accel_event, &gyro_event, &temp_event);  // get data from IMU - currently not using this temperature
    //lis.getEvent(&mag_event);                              // get data from magentometer

    //cal.calibrate(gyro_event);  // apply cal factors
    //cal.calibrate(accel_event);
    //cal.calibrate(mag_event);

    // calculate quaternions and RPY
    //gx = gyro_event.gyro.x * SENSORS_RADS_TO_DPS;
    //gy = gyro_event.gyro.y * SENSORS_RADS_TO_DPS;
    //gz = gyro_event.gyro.z * SENSORS_RADS_TO_DPS;

    // Update the SensorFusion filter
    //filter.update(gx, gy, gz,
    //              accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
    //              mag_event.magnetic.x, mag_event.magnetic.y, mag_event.magnetic.z);

    //roll = filter.getRoll();
    //pitch = filter.getPitch();
    //yaw = filter.getYaw();

    //filter.getGravityVector(&gravX, &gravY, &gravZ);  // gravX = +1 when x-axis pointed up, -1 when pointed down. Not using these atm, but for future

    //
    // test calculations for measuring velocity will go here
    //

    // get environmental sensor data
    if (bmp388.getMeasurements(temperature, pressure, altitude)) {  // Check if the measurement is complete
      if (!baselinePressSet) {                                      // take baseline measurement once after boot, this variable survives deep-sleep
        Serial.println(F("Taking baseline pressure measurements"));
        sealevelPressure = pressure;
        baselinePressSet = true;
        Serial.print(F("Baseline pressure = "));
        Serial.println(sealevelPressure);
        bmp388.setSeaLevelPressure(sealevelPressure);  // Set the sea level pressure value
        for (int i = 0; i < 4; i++) {                  // take several samples to settle noise
          bmp388.getAltitude(padElevation);
          delay(50);
          Serial.print(F("Raw altitude = "));
          Serial.println(padElevation);
        }
        Serial.print(F("Pad Elevation = "));
        Serial.println(padElevation);
      }
    }

    // smooth altitude data and calculate derivative
    doublealtitude = (double)altitude;
    smoothaltitude = altitudeSmoother.Compute();
    daltitude = altitudeDerivative.Compute();

    // calculate if apogee reached
    altitudeApogee = (altitudeApogee > smoothaltitude) ? altitudeApogee : smoothaltitude;
    //@SENSOR

    // meaure battery voltage - feathers have on board pmon, qt py and M0 have raw voltage on gpio pin
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3)
    vBattery = 2 * analogReadMilliVolts(VBATPIN) - 700;                                                         // integer mV - voltage divided by 2 by internal voltage divider, so multiply back, then subtract off voltage drop from Schotkey diode
#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
    vBattery = 2 * VREF * analogRead(VBATPIN) / 1024;  // voltage divided by 2 by internal voltage divider, so multiply back
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM)
    if (addr0x36 == true) {  // if max17048
      vBattery = maxlipo.cellVoltage();
    } else {  // if lc709203f
      vBattery = lc.cellVoltage();
    }
#endif

    doublebattery = (double)vBattery / 1024;    // QT PY returns mV - ACTION: review M0 vs QT PY methods
    smoothbattery = batterySmoother.Compute();  // apply SavLay filter to doublebattery

#ifndef BALLOON
    //
    // EVENT LOGIC - ACTION: simplify and/or reduce number of modes for ROCKET. No longer used for BALLOON, see 2026 comments at beginning of file
    //
    if (!bitRead(flightStatus, LAUNCH) && ((accel_event.acceleration.x >= LAUNCH_THRESHOLD) || ((smoothaltitude - padElevation) >= ALTITUDE_THRESHOLD))) {  // accel used for rocket, altitude used for balloon
      onPosAx();
      Serial.print(F("ax = "));
      Serial.println(accel_event.acceleration.x);
      Serial.print(F("alt = "));
      Serial.println(smoothaltitude);
    }
    //if (!bitRead(flightStatus, DEPLOY) && bitRead(flightStatus, LAUNCH) && accel_event.acceleration.x >= DEPLOY_THRESHOLD) {  // for Balloon this is FLOAT, for rocket this is DEPLOY - uncomment line 1245
    if (!bitRead(flightStatus, DEPLOY) && bitRead(flightStatus, LAUNCH) && (abs(smoothaltitude - altitudeApogee) <= ALTITUDE_ERROR) || accel_event.acceleration.x >= DEPLOY_THRESHOLD) {  // for Balloon this is FLOAT, for rocket this is DEPLOY
      onNegAx();
      Serial.print(F("ax = "));
      Serial.println(accel_event.acceleration.x);
      Serial.print(F("alt = "));
      Serial.println(smoothaltitude);
      Serial.print(F("apogee = "));
      Serial.println(altitudeApogee);
      Serial.print(F("abs(smoothaltitude - altitudeApogee) = "));
      Serial.println(abs(smoothaltitude - altitudeApogee));
    }
    if (!bitRead(flightStatus, LAND) && bitRead(flightStatus, DEPLOY) && accel_event.acceleration.x <= 0) {  // is using accel.x sufficient? system may land in any orientation - will gravity vector work? THIS WORKS 7/27/25 9pm
      onNoAltChange();
      Serial.print(F("ax = "));
      Serial.println(accel_event.acceleration.x);
      Serial.print(F("dalt = "));
      Serial.println(daltitude);
      landTime = millis();
    }
    if (!bitRead(flightStatus, MISSIONEND) && bitRead(flightStatus, LAND) && (millis() - landTime) >= LAND_INTERVAL) {
      bitSet(flightStatus, MISSIONEND);
      Serial.println(F("Mission Ended"));
      closeFile();
      onStandby();
    }
#endif

    // build dataMessage
    // IONId, flightStatus, lat, lon, altGPS, T, P, altP, sealevelP, ax, ay, az, gx, gy, gx, roll, pitch, yaw, vbat, groundspeed, SIV, FixType, launchTime
    // 270 char with all data (ROCKET)
    // 128 char with subset of data (BALLOON)
    // An argument can be made to have a consistent data stream across BALLOON and ROCKET and all HW platforms - simplifies post processing
    // For the BALLOON missions, we would simply set all the ROCKET data (GPS, LoRA) to NAN (not 0 as that is a valid value)
    // However, for long durarion balloon missions, data storage must be managed and every byte counts, therefore shorten the stream using #ifdef's
    strcpy(buff, "");
    strcat(dataMessage, UniqueIDString);
    strcat(dataMessage, ",");
    utoa(flightStatus, buff, 10);
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");

#if defined(GPS_ENABLED)
    dtostrf(myGNSS.getLatitude() * 1e-7, 8, 4, buff);
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(myGNSS.getLongitude() * 1e-7, 8, 4, buff);
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(myGNSS.getAltitude() / 1000, 8, 2, buff);
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
#endif  // GPS_ENABLED

    dtostrf(temperature, 8, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(pressure, 8, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(smoothaltitude, 8, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(sealevelPressure, 8, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(mcpTempUnderRegolith, 6, 2, buff);  //@SENSOR //@BERIKOV under-regolith temp (degC) in the former 'ax' field
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(accel_event.acceleration.y, 6, 2, buff);  //@SENSOR //@BERIKOV unused IMU field = 0
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(accel_event.acceleration.z, 6, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(gyro_event.gyro.x, 6, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(gyro_event.gyro.y, 6, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(gyro_event.gyro.z, 6, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(roll, 7, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(pitch, 7, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(yaw, 7, 2, buff);  //@SENSOR
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    dtostrf(smoothbattery, 4, 3, buff);  // for ESP32-S3 with LC or max battery mon (float) and QT PY
    //ultoa(smoothbattery, buff, 10);   // for M0 with analogread (int) - ACTION: review if necessary
    //buff[4] = '\0';  // shorten to 4 char
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");

#if defined(GPS_ENABLED)
    dtostrf(myGNSS.getGroundSpeed() / 1000, 7, 2, buff);  // not used in groundstation, consider removing
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    ultoa(myGNSS.getSIV(), buff, 10);  // Returns number of sats used in fix
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
    ultoa(myGNSS.getFixType(), buff, 10);  // The fix type is as follows:
    // 0 = no fix
    // 1 = dead reckoning (requires external sensors)
    // 2 = 2D (not quite enough satellites in view)
    // 3 = 3D (the standard fix)
    // 4 = GNSS + dead reckoning (requires external sensors)
    // 5 = Time fix only
    strcat(dataMessage, buff);
    strcat(dataMessage, ",");
#endif

    ultoa(launchTime, buff, 10);
    strcat(dataMessage, buff);
    strcat(dataMessage, "\0");  // terminate char array

    // now that dataMessage created, write dataMessage to either buffer or flash
    if (flashWriteTimer >= WRITE_INTERVAL) {
      if (bitRead(flightStatus, WRITEFLASH) == 0) {                 // WRITEFLASH cleared = write to circular buffer, set = write to FLASH
        strcpy(writeBuffer[0], dataMessage);                        // if launch not detected, push data into buffer
        for (uint8_t j = (WRITE_BUFFER_LENGTH - 1); j > 0; j--) {
          strcpy(writeBuffer[j], writeBuffer[j - 1]);               // shift the prior data element down one
        }
      } else {
        if (!copiedBuffer) {  // set to false on boot so that we only copy from buffer to data file once (immediately after launch)
          for (uint8_t k = 0; k < WRITE_BUFFER_LENGTH; ++k) {
            dataFile.println(writeBuffer[WRITE_BUFFER_LENGTH - k - 1]);  // data stored most recent on top, so write out backwards, FILO
          }
          //Serial.println(F("wrote writeBuffer to dataFile"));
          copiedBuffer = true;  // only need to copy to buffer once, so set the flag to disable copy on next data acq cycle
        }
        dataFile.println(dataMessage);

        if (writeDataTimer >= SAVE_INTERVAL) {  // 7/24/25 code to periodically (every min) sync file - this is important in the event power is lost before EOM
          dataFile.sync();
          writeDataTimer = writeDataTimer - SAVE_INTERVAL;
          //Serial.println(F("syncing datafile"));
        }
      }
      flashWriteTimer = flashWriteTimer - WRITE_INTERVAL;
    }
#if defined(LORA_ENABLED)
    // ACTION - decide whether or not to turn off LoRa after rocket recovered
    sendMessage(dataMessage);
//receiveMessage(LoRa.parsePacket());  // always check for commands - ACTION: verify if I'm still sending commands from groundstation
// 6/8/24 revisit - I shouldn't be looking for commands in the same period as I'm sending out data - that seems too frequent. Revisit in 2026 after RB-11 mission
#endif

    getDataTimer = getDataTimer - SAMPLE_INTERVAL;
  }
}

time_t getRtcTime() {
  DateTime dt = rtc.now();
  setTime(dt.unixtime());
  //adjustTime(offset * SECS_PER_HOUR);
  return (dt.unixtime());
}

#if defined(GPS_ENABLED)
time_t getGPSTime() {
  tm.Second = myGNSS.getSecond();
  tm.Minute = myGNSS.getMinute();
  tm.Hour = myGNSS.getHour();
  tm.Day = myGNSS.getDay();
  tm.Month = myGNSS.getMonth();
  tm.Year = CalendarYrToTm(myGNSS.getYear());
  setTime(myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), myGNSS.getDay(), myGNSS.getMonth(), myGNSS.getYear());
  adjustTime(offset * SECS_PER_HOUR);
  return (makeTime(tm));
}

void configGPS() {
  myGNSS.setI2COutput(COM_TYPE_UBX);  //Set the I2C port to output UBX only (turn off NMEA)

  if (myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE4g) == false) {  // Set the dynamic model to ARIBORNE4g
    Serial.println(F("*** Warning: setDynamicModel failed ***"));
  } else {
    Serial.println(F("Dynamic platform model changed successfully!"));
  }

  uint8_t newDynamicModel = myGNSS.getDynamicModel();  // Let's read the new dynamic model to see if it worked
  if (newDynamicModel == DYN_MODEL_UNKNOWN) {
    Serial.println(F("Warning: getDynamicModel failed"));
  } else {
    Serial.print(F("The new dynamic model is: "));
    Serial.println(newDynamicModel);
  }
  myGNSS.setNavigationFrequency(10);              // produce ten solutions per second (was 2)
  myGNSS.setAutoPVT(true, true, defaultMaxWait);  // was (true, false); //Tell the GNSS to "send" each solution

  Serial.println(F("Done with GPS configure"));
}
#endif  // GPS_ENABLED

#if defined(LORA_ENABLED)
void sendMessage(char *outgoing) {
  if (LoRaXmitTimer >= XMTRATE) {
    LoRa.beginPacket();        // start packet
    LoRa.write(destination);   // add destination address
    LoRa.write(localAddress);  // add sender address
    LoRa.print(outgoing);      // add payload - using .print since it expects a char *
    LoRa.endPacket(true);      // finish packet and send it: true = async / non-blocking mode
    LoRaXmitTimer = LoRaXmitTimer - XMTRATE;
  }
}

void receiveMessage(int packetSize) {
  const char *cmdDelimiter = "=!";
  char *cmddata[2] = { 0 };  // added = {0}

  if (packetSize == 0) return;  // if there's no packet, return

  uint8_t recipient = LoRa.read();  // recipient address
  uint8_t sender = LoRa.read();     // sender address

  // if the recipient isn't this device or broadcast,
  if (recipient != localAddress && recipient != 0xFF) {
    Serial.println(F("This message is not for me."));
    return;  // skip rest of function
  }

  char rxbuf[packetSize];
  int i = 0;
  while (LoRa.available()) {
    snprintf(&rxbuf[i++], packetSize, "%c", (char *)LoRa.read());
  }

  Serial.print(F("Received command: "));
  Serial.println(rxbuf);

  i = 0;
  char *r = strtok(rxbuf, cmdDelimiter);
  while (r != NULL) {
    cmddata[i++] = r;
    r = strtok(NULL, cmdDelimiter);
  }

  if (!strcmp(cmddata[0], "LIST")) {  // cmd for listing file directory - no longer necessary since using web interface
    if (!strcmp(cmddata[1], "1"))
      listFiles = true;
  }

  if (!strcmp(cmddata[0], "DUMP")) {  // cmd for dumping contents of dataFile to Serial - no longer necessary since using web interface
    if (!strcmp(cmddata[1], "1"))
      dumpFile = true;
  }
}
#endif  // LORA_ENABLED

// callback function for setting file date/time
void dateTime(uint16_t *date, uint16_t *time) {
  time_t filetime = now();
  *date = FS_DATE(year(filetime), month(filetime), day(filetime));
  *time = FS_TIME(hour(filetime), minute(filetime), second(filetime));
}

// function to flash led or neo and freeze
// general logic: lower flashes indicate hw related errors, e.g wiring. more flashes indicate system errors, e.g. internal flash. YELLOW associated with wiring, RED potential HW failure
void err(int errnum, uint32_t color) {
#if defined(ESP32) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM)
  neo.setColor(color);
  neo.setBlinks(errnum);
  while (1) {
    neo.update();
  }
#else
  led.pattern(errnum);                                 // neo not available on M0
  while (1)
    ;
#endif
}

void serialDeleteFile() {
  static byte ndx = 0;
  char rc;
  if (bitRead(flightStatus, LAND)) {  // only allow file delete after landed and files closed - ACTION: revisit this rationale since can delete from webserver without restriction
    Serial.println(F("Type filename to delete: (remember to set Serial Monitor to New Line)"));

    while (Serial.available()) {
      rc = Serial.read();
      if (rc != endMarker) {
        filename[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        filename[ndx] = '\0';  // terminate the string
        ndx = 0;
        Serial.printf("Deleting file: %s\n", filename);
        if (fatfs.remove(filename))
          Serial.printf("File %s deleted\n", filename);
        else
          Serial.println("File not found");
      }
    }
  } else {
    Serial.println(F("Must be Landed to delete files"));
  }
}

void serialReadFile() {
  static byte ndx = 0;
  char rc;
  Serial.println(F("Type filename to read: (remember to set Serial Monitor to New Line)"));

  while (Serial.available()) {
    rc = Serial.read();
    if (rc != endMarker) {
      filename[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      filename[ndx] = '\0';  // terminate the string
      ndx = 0;
      Serial.printf("Reading file: %s\n", filename);
      File32 dataFile = fatfs.open(filename, FILE_READ);
      if (dataFile) {
        Serial.println("Opened file, printing contents below:");
        while (dataFile.available()) {
          char c = dataFile.read();
          Serial.print(c);
        }
      } else
        Serial.printf("Failed to open file %s\n", filename);
    }
  }
}

// handy "ls" function
void SerialhandleFileList() {
  String listing;
  char temp[33];
  long dataUsed = 0;
  root.rewind();
  Serial.println("\nListing files");
  listing = "Listing files";
  listing += '\n';
  root.open("/");
  Serial.println();
  while (file.openNext(&root, O_RDONLY)) {
    dataUsed += file.size();
    file.printFileSize(&Serial);
    Serial.write(' ');
    listing += file.size();
    listing += '\t';
    file.printModifyDateTime(&Serial);
    Serial.write(' ');
    file.printName(&Serial);
    file.getName(temp, sizeof(temp));
    listing += temp;
    listing += '\n';
    if (file.isDir()) {
      Serial.write('/');
    }
    Serial.println();
    file.close();
  }

  if (root.getError()) {
    Serial.println("directory listing failed");
  } else {
    Serial.print(F("Used "));
    Serial.println(dataUsed);
    Serial.print("Free ");
    Serial.println(flash.size() - dataUsed);
    listing += "\nUsed: ";
    listing += dataUsed;
    listing += "\nFree: ";
    listing += (flash.size() - dataUsed);
    listing += "\nDone!\n";
    Serial.println("Done!");
  }
  webserver.send(200, "text/plain", listing);
}
// calibration function - called from serial command
// cal values are stored in esp32 non-volatile memory. If different sensors attached, must recalibrate
//@SENSOR
#if 0  //@BERIKOV calibration routine is IMU/magnetometer-specific; disabled (no IMU in this configuration)
void performCalibration() {
  sensors_event_t accel_event, gyro_event, temp_event, mag_event;  // data structures for events (when data is available)
  const uint16_t NumberCalSamples = 500;                           // used by gyro cal routine
  float gx, gy, gz, mx, my, mz;
  float amax_x, amax_y, amax_z;  // for accel cal
  float gmin_x, gmax_x, gmid_x;  // for gyro cal
  float gmin_y, gmax_y, gmid_y;
  float gmin_z, gmax_z, gmid_z;
  float magmin_x, magmax_x, magmid_x;  // for mag cal
  float magmin_y, magmax_y, magmid_y;
  float magmin_z, magmax_z, magmid_z;
  float magdelta_x, magdelta_y, magdelta_z, magavg_delta;
  float magscale_x, magscale_y, magscale_z;

  Serial.println(F("Entering Calibration routine"));

  lsm.getEvent(&accel_event, &gyro_event, &temp_event);

  gmin_x = gmax_x = gyro_event.gyro.x;
  gmin_y = gmax_y = gyro_event.gyro.y;
  gmin_z = gmax_z = gyro_event.gyro.z;
  delay(10);

  Serial.println(F("Place ION on a flat, stable surface"));
  Serial.println(F("Type key when ready to begin gyro calibration. Hold the ION steady."));
  while (!Serial.available()) {}  // wait for a character
  Serial.read();                  // clear serial buffer

  for (uint16_t sample = 0; sample < NumberCalSamples; sample++) {  //perform gyro calibration
    lsm.getEvent(&accel_event, &gyro_event, &temp_event);
    gx = gyro_event.gyro.x;
    gy = gyro_event.gyro.y;
    gz = gyro_event.gyro.z;
    Serial.print(F("Gyro: ("));
    Serial.print(gx);
    Serial.print(F(", "));
    Serial.print(gy);
    Serial.print(F(", "));
    Serial.print(gz);
    Serial.print(F(")"));

    gmin_x = min(gmin_x, gx);
    gmin_y = min(gmin_y, gy);
    gmin_z = min(gmin_z, gz);

    gmax_x = max(gmax_x, gx);
    gmax_y = max(gmax_y, gy);
    gmax_z = max(gmax_z, gz);

    gmid_x = (gmax_x + gmin_x) / 2;
    gmid_y = (gmax_y + gmin_y) / 2;
    gmid_z = (gmax_z + gmin_z) / 2;

    Serial.print(F(" Zero rate offset: ("));
    Serial.print(gmid_x, 4);
    Serial.print(F(", "));
    Serial.print(gmid_y, 4);
    Serial.print(F(", "));
    Serial.print(gmid_z, 4);
    Serial.print(F(")"));

    Serial.print(F(" rad/s   Noise: ("));
    Serial.print(gmax_x - gmin_x, 3);
    Serial.print(F(", "));
    Serial.print(gmax_y - gmin_y, 3);
    Serial.print(F(", "));
    Serial.print(gmax_z - gmin_z, 3);
    Serial.println(F(")"));

    delay(10);
  }
  Serial.println(F("\nFinal zero rate offset in radians/s: "));
  Serial.print(gmid_x, 4);
  Serial.print(F(", "));
  Serial.print(gmid_y, 4);
  Serial.print(F(", "));
  Serial.println(gmid_z, 4);

  Serial.println(F("Gyro calibration complete."));
  Serial.println(F(""));

  Serial.println(F("Prepare ION for accelerometer calibration."));  // accelerometer calibration routine
  Serial.println(F("There are 3 steps. First you will place ION +x up, press a key, position +y up, press a key, then position +z up"));

  for (uint8_t sample = 0; sample < 3; sample++) {
    Serial.print(F("Position ION and type key when ready to begin. Step: "));
    Serial.print(sample + 1);
    Serial.println(F(" of 3"));
    while (!Serial.available()) {}  // wait for a character

    lsm.getEvent(&accel_event, &gyro_event, &temp_event);

    if (accel_event.acceleration.x > amax_x) amax_x = accel_event.acceleration.x;
    if (accel_event.acceleration.y > amax_y) amax_y = accel_event.acceleration.y;
    if (accel_event.acceleration.z > amax_z) amax_z = accel_event.acceleration.z;

    Serial.print(F("Accel Maximums: "));
    Serial.print(amax_x);
    Serial.print("  ");
    Serial.print(amax_y);
    Serial.print("  ");
    Serial.print(amax_z);
    Serial.println();

    while (Serial.available()) {  // wait for a character
      Serial.read();              // clear the input buffer
    }
  }
  amax_x -= SENSORS_GRAVITY_EARTH;  // single point cal - subtract gravity from max readings gathered
  amax_y -= SENSORS_GRAVITY_EARTH;
  amax_z -= SENSORS_GRAVITY_EARTH;

  Serial.print(F("Accel Offsets: "));
  Serial.print(amax_x);
  Serial.print("  ");
  Serial.print(amax_y);
  Serial.print("  ");
  Serial.print(amax_z);
  Serial.println();
  Serial.println(F("Accelerometer calibration complete."));

  // magnetometer calibration routine
  Serial.println(F("Prepare ION for magnetometer calibration. Remember to continually rotate ION aoround all 3-axes."));
  Serial.println(F("Once you are satisfied, press any key to stop calibration"));
  while (!Serial.available()) {}  // wait for a character
  Serial.read();                  // clear the input buffer

  lis.getEvent(&mag_event);
  magmin_x = magmax_x = mag_event.magnetic.x;
  magmin_y = magmax_y = mag_event.magnetic.y;
  magmin_z = magmax_z = mag_event.magnetic.z;

  while (!Serial.available()) {  // wait for a character
    lis.getEvent(&mag_event);
    mx = mag_event.magnetic.x;
    my = mag_event.magnetic.y;
    mz = mag_event.magnetic.z;

    Serial.print(F("Mag: ("));
    Serial.print(mx);
    Serial.print(F(", "));
    Serial.print(my);
    Serial.print(F(", "));
    Serial.print(mz);
    Serial.print(F(")"));

    magmin_x = min(magmin_x, mx);  // record min values
    magmin_y = min(magmin_y, my);
    magmin_z = min(magmin_z, mz);

    magmax_x = max(magmax_x, mx);  // record max values
    magmax_y = max(magmax_y, my);
    magmax_z = max(magmax_z, mz);

    magmid_x = (magmax_x + magmin_x) / 2;  // hard iron offsets are avg of min and max
    magmid_y = (magmax_y + magmin_y) / 2;
    magmid_z = (magmax_z + magmin_z) / 2;
    Serial.print(F(" Hard iron offset: ("));
    Serial.print(magmid_x);
    Serial.print(F(", "));
    Serial.print(magmid_y);
    Serial.print(F(", "));
    Serial.print(magmid_z);
    Serial.print(F(")"));

    magdelta_x = (magmax_x - magmin_x) / 2;  // simplified calculation for determining soft iron factors ref: https://www.appelsiini.net/2018/calibrate-magnetometer/
    magdelta_y = (magmax_y - magmin_y) / 2;
    magdelta_z = (magmax_z - magmin_z) / 2;
    magavg_delta = (magdelta_x + magdelta_y + magdelta_z) / 3;

    magscale_x = magavg_delta / magdelta_x;  // simplified soft iron factors is a scaled identity matrix (diagonols)
    magscale_y = magavg_delta / magdelta_y;
    magscale_z = magavg_delta / magdelta_z;
    Serial.print(F(" Magscale: ("));
    Serial.print(magscale_x);
    Serial.print(F(", "));
    Serial.print(magscale_y);
    Serial.print(F(", "));
    Serial.print(magscale_z);
    Serial.print(F(")"));

    Serial.print(F(" Field: ("));
    Serial.print(magdelta_x);
    Serial.print(F(", "));
    Serial.print(magdelta_y);
    Serial.print(F(", "));
    Serial.print(magdelta_z);
    Serial.println(F(")"));
  }
  Serial.println(F("Magnetometer calibration complete"));

  cal.accel_zerog[0] = amax_x;
  cal.accel_zerog[1] = amax_y;
  cal.accel_zerog[2] = amax_z;
  cal.gyro_zerorate[0] = gmid_x;
  cal.gyro_zerorate[1] = gmid_y;
  cal.gyro_zerorate[2] = gmid_z;
  cal.mag_hardiron[0] = magmid_x;
  cal.mag_hardiron[1] = magmid_y;
  cal.mag_hardiron[2] = magmid_z;
  cal.mag_softiron[0] = magscale_x;
  cal.mag_softiron[1] = 0;
  cal.mag_softiron[2] = 0;
  cal.mag_softiron[3] = 0;
  cal.mag_softiron[4] = magscale_y;
  cal.mag_softiron[5] = 0;
  cal.mag_softiron[6] = 0;
  cal.mag_softiron[7] = 0;
  cal.mag_softiron[8] = magscale_z;
  cal.mag_field = (magmax_x - magmin_x) / 2;

  if (!cal.saveCalibration())  // save cal data to file
    Serial.println(F("Could not save calibration"));
  else
    Serial.println(F("Wrote calibration"));

  cal.printSavedCalibration();
  Serial.println("Calibrations found: ");
  Serial.print("\tMagnetic Hard Offset: ");
  for (int i = 0; i < 3; i++) {
    Serial.print(cal.mag_hardiron[i]);
    if (i != 2) Serial.print(", ");
  }
  Serial.println();

  Serial.print("\tMagnetic Soft Offset: ");
  for (int i = 0; i < 9; i++) {
    Serial.print(cal.mag_softiron[i]);
    if (i != 8) Serial.print(", ");
  }
  Serial.println();

  Serial.print("\tMagnetic Field Magnitude: ");
  Serial.println(cal.mag_field);

  Serial.print("\tGyro Zero Rate Offset (rad/sec): ");
  for (int i = 0; i < 3; i++) {
    Serial.print(cal.gyro_zerorate[i]);
    if (i != 2) Serial.print(", ");
  }
  Serial.println();

  Serial.print("\tAccel Zero G Offset (m/sec^2): ");
  for (int i = 0; i < 3; i++) {
    Serial.print(cal.accel_zerog[i]);
    if (i != 2) Serial.print(", ");
  }
  Serial.println();
}
#endif  //@BERIKOV end disabled performCalibration
//@SENSOR

//
// define event functions
//
void onStandby() {
  Serial.println(F("In Standby"));
  neo.setColor(DKGREEN);
  neo.on();
  bitClear(flightStatus, WRITEFLASH);  // clear bit means save to circular buffer
}

void onPosAx() {
  Serial.println("\nLaunch triggered");
  bitSet(flightStatus, LAUNCH);  // set launch register bit
#if defined(BALLOON)
//bitSet(flightStatus, DEPLOY);      // set deploy register bit - testing on 07/26/2025 to skip DEPLOY event
#endif
  bitSet(flightStatus, WRITEFLASH);  // set bit to save data to storage
  webserver.stop();                  // stop webserver once launch detected
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  //neo.setColor(ORANGE);
  //neo.on();
  Serial.println(F("webserver stopped and wifi sleeping"));
  launchTime = now();  // record realtime flag was set
}

void onNegAx() {
  Serial.println("\nDeploy triggered");
  bitSet(flightStatus, DEPLOY);
  neo.setColor(PINK);
  neo.on();
}

void onNoAltChange() {
  Serial.println("\nLand triggered");
  bitSet(flightStatus, LAND);
  neo.setColor(WHITE);
  neo.on();
}

void closeFile() {
  bitClear(flightStatus, WRITEFLASH);  // continue writing to circular buffer but stop writing to Flash
  //dataFile.close();                          // commented out for CiS 2025 balloon flight - likely add back for Rocket
  //Serial.println(F("closed data file"));     // commented out for CiS 2025 balloon flight - likely add back for Rocket
  bitSet(flightStatus, CMDMODE);  // once landed, do not resume checking for flight trigger events
  startServer();                  // once files are saved, can restart WiFi and webserver - WARNING in this version, files are not closed - perform file actions with caution
}

void onCmdEnabled() {
  Serial.println(F("entering cmd mode"));
  bitSet(flightStatus, CMDMODE);
  neo.setColor(MAGENTA);
  neo.on();
}

//
// Serial console - used for calibration and to enable webserver, also allows file download and delete
// get here by pressing 'Y' within 8 seconds of ION boot (neopixel will blink yellow)
//
void serialProcessCmd() {
  char cmd;
  SerialhandleFileList();

  while (Serial.available()) {
    Serial.read();
  }
  Serial.println(F("FLASH UTIL - Press 'D filename' to delete, 'R filename' to read, C to calibrate, W to start webserver, or E to restart ION"));

  while (!Serial.available())
    ;

  cmd = toupper(Serial.read());

  if (cmd == 'D') {
    serialDeleteFile();
  }
  if (cmd == 'R') {
    serialReadFile();
  }
  if (cmd == 'C') {  //@BERIKOV calibration disabled (no IMU)
    //performCalibration();
    Serial.println(F("Calibration not available in this configuration (no IMU)"));
    bitClear(flightStatus, CMDMODE);
    Serial.println(F("exiting cmd mode and restarting data acq"));
    onStandby();
  }
  if (cmd == 'W') {  // prior to sleep change, this was "Resume", to restart data collection
    bitClear(flightStatus, CMDMODE);
    Serial.println(F("WiFi starting..."));
    startServer();  // start web server
    Serial.println(F("exiting cmd mode and restarting data collection"));
    while (1) {  // simulates "loop()" - added since balloon mission never gets to loop() - only way out is to press reest or choose reset on webpage
      getData();
      webserver.handleClient();
      neo.update();
    }
  }
  if (cmd == 'E') {
    ESP.restart();
  }
}

// handy commands - not used atm
bool isCmdSet() {
  return !bitRead(flightStatus, CMDMODE);
}

bool hasNotLanded() {
  return !bitRead(flightStatus, LAND);
}

bool hasNotLaunched() {
  return !bitRead(flightStatus, LAUNCH);
}

// start webserver
// note this requires lots of power - only use when on ground when calibrating pre-flight or retreiving data post-flight
void startServer() {
  if (!WiFi.softAP(UniqueIDString)) {
    Serial.println(F("Soft AP creation failed."));
    while (1)
      ;
  }
  Serial.print(F("Connect to SSID: "));
  Serial.println(UniqueIDString);
  Serial.printf("Then point your web browser to <http://%s>\n\n", WiFi.softAPIP().toString().c_str());
  webserver.begin();
}

// This function is called when the sysInfo service is requested - handy reference data on esp32
void handleSysInfo() {
  String result;
  result += "{\n";
  result += "  \"Chip Model\": " + String(ESP.getChipModel()) + ",\n";
  result += "  \"Chip Cores\": " + String(ESP.getChipCores()) + ",\n";
  result += "  \"Chip Revision\": " + String(ESP.getChipRevision()) + ",\n";
  result += "  \"flashSize\": " + String(ESP.getFlashChipSize()) + ",\n";
  result += "  \"freeHeap\": " + String(ESP.getFreeHeap()) + ",\n";
  result += "  \"fsTotalBytes\": " + String(flash.size()) + ",\n";
  result += "}";

  webserver.sendHeader("Cache-Control", "no-cache");
  webserver.send(200, "text/javascript; charset=utf-8", result);
}

//
// setup
//
// 2026 update
// for balloon missions, put ION to sleep after completing setup. Also put radios in lowest power setting during all phases of balloon
// mission and only turn on when serial command "w" is issued (to download data). Option to keep ION awake for calibration or data
// retrieval exists via serial command. Before we prompt for serial, we must init all functions. Therefore no serial.print commands are
// avaible during init (I've left the legacy print commands there). We must rely on neopixel flashes to decode any errors.
//
void setup() {
  static uint32_t waittimer = millis();
  static const uint32_t SERIAL_WAIT = 8000;  // timeoout Serial 8 sec after boot

#ifdef BALLOON
  ++bootCount;                                                    // increment boot counter (stored in non-volatile RTC memory)
  esp_sleep_enable_timer_wakeup(SLEEP_TIME * uS_TO_S_FACTOR);  // time is in usec and is 64 bit unsigned integer
  // Enable modem sleep (power save mode), availble modes:
  // WIFI_PS_NONE: No power save
  // WIFI_PS_MIN_MODEM: Minimum modem power saving
  // WIFI_PS_MAX_MODEM: Maximum modem power saving
  WiFi.setSleep(WIFI_PS_MIN_MODEM);  // calls esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  WiFi.mode(WIFI_OFF);               // does this achieve the same effect?
#endif

  neo.begin();  // initialize the NeoPixel
  neo.setBrightness(40); // was 60, lower brightness to save a bit of power
  neo.setColor(YELLOW);
  neo.setBlinks(1);
  neo.update();

  //
  // disable other devices on SPI (LoRa) before we init flash
  //
#if defined(LORA_ENABLED)
  digitalWrite(LORA_CS, HIGH);
  pinMode(LORA_CS, OUTPUT);
#endif

  //
  // init flash
  //
  if (!flash.begin()) {
    //Serial.println(F("Error, failed to initialize flash chip!")); // can't print since Serial isn't started yet
    err(4, RED);  // blink neo and wait here forever if flash failure
  }
  // First call begin to mount the filesystem.  Check that it returns true to make sure the filesystem was mounted.
  if (!fatfs.begin(&flash)) {
    //Serial.println(F("Error, failed to mount newly formatted filesystem!"));
    //Serial.println(F("Was the flash chip formatted with the fatfs_format example?"));
    err(4, RED);  // blink neo and wait here forever if flash failure
  }

  //
  // init Wire interface
  //
#if defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3)
  if (Wire1.begin(SDA1, SCL1, 400000)) {  // for QT PY use second Wire interface on STEMMA plug
    //Serial.print("Wire successful\n");
    //Serial.printf("I2C clock = %u\n", Wire1.getClock());
  } else {
    //Serial.println(F("Failed to initialize Wire1, check for hardware errors and press reset to try again"));
    err(5, RED);  // general logic: lower flashes indicate hw related errors, e.g wiring. more flashed indicate system errors, e.g. internal flash
  }
#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM)
  Wire.begin();
#endif

  //
  // use esp32's unique ID to build filename and ID
  //
  UniqueIDString[(UniqueIDsize) + 1];  // XXXXXX+terminator

  byte index = 0;
  for (size_t i = UniqueIDsize / 2; i < UniqueIDsize; i++) {         // use the last 3 bytes of the 6 byte sequence since those are unique
    UniqueIDString[index++] = "0123456789ABCDEF"[UniqueID[i] >> 4];  // clever bit mapping I found on github
    UniqueIDString[index++] = "0123456789ABCDEF"[UniqueID[i] & 0x0F];
  }
  UniqueIDString[index++] = 0;  // Null Terminator

  //
  // initialize RTC
  //
  //@BERIKOV RTC (DS3231) not installed; skip RTC init and its halting err()
  RTCOk = false;
  /*  //@BERIKOV original RTC init disabled
  if (rtc.begin(&WIRE)) {
    RTCOk = true;
    //Serial.println(F("RTC started"));
    if (rtc.lostPower()) {
      //Serial.println(F("RTC lost power, setting time"));
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
  } else {  //  don't want to call err here if GPS could be available
#if defined(GPS_ENABLED)
    //Serial.println(F("RTC not found - will check for GPS"));
#else
    //Serial.println(F("GPS not enabled and could not find RTC"));
    err(2, YELLOW);
#endif
  }
  */

  //
  // init GPS - ION doesn't use GPS so not worried about reducing power here (serial.prints)
  //
#if defined(GPS_ENABLED)
  neo.setColor(BLUE);
  neo.on();
  neo.update();

  if (myGNSS.begin(WIRE)) {
    GPSOk = true;  // flag used to determine if GPS time is used
    Serial.println(F("GPS ready"));
    Serial.println(F("Waiting for valid date"));
    // wait for valid PVT to get date/time for filename
    do {
      myGNSS.getPVT();
      Serial.print(F("."));
    } while (!myGNSS.getDateValid());

    Serial.print(myGNSS.getYear());
    Serial.print("-");
    Serial.print(myGNSS.getMonth());
    Serial.print("-");
    Serial.print(myGNSS.getDay());
    Serial.print(" ");
    if (myGNSS.getHour() < 10) Serial.print(F("0"));
    Serial.print(myGNSS.getHour());
    Serial.print(":");
    if (myGNSS.getMinute() < 10) Serial.print(F("0"));
    Serial.print(myGNSS.getMinute());
    Serial.print(":");
    if (myGNSS.getSecond() < 10) Serial.print(F("0"));
    Serial.print(myGNSS.getSecond());

    Serial.print("  Time is ");
    if (myGNSS.getTimeValid() == false) {
      Serial.print("not ");
    }
    Serial.print("valid  Date is ");
    if (myGNSS.getDateValid() == false) {
      Serial.print("not ");
    }
    Serial.println("valid");

    setTime(myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond(), myGNSS.getDay(), myGNSS.getMonth(), myGNSS.getYear());
    adjustTime(offset * SECS_PER_HOUR);
    configGPS();
  } else {
    Serial.println(F("Error setting up GPS"));
  }
#endif

#if defined(LORA_ENABLED)
  //
  // init LoRa - similar to GPS, ION does not use LoRa, code not designed for low power
  //
  LoRa.setPins(LORA_CS, LORA_RESET, LORA_IRQ);  // set CS, reset, IRQ pin

  if (!LoRa.begin(915E6)) {  // initialize ratio at 915 MHz
    Serial.println("LoRa init failed. Check your connections.");
    err(2, YELLOW);  // if failed, report error
  }

  //LoRa.onReceive(receiveMessage);  // TESTING 10/5/23: register the receive callback - greatly improved responsivity to cmds
  // line below commented out on 6/9/24
  //LoRa.receive();                    // 6/8/2024 code review - this isn't necessary since never looking for a packet, always sending
  // besides only necessary when using onReceive callback - which was commented out in this version
  Serial.println("Radio ready");
#endif

  //
  // init battery mon - NA to ION since QT PY doesn't have onboard pmon
  //
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3) || defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_NOPSRAM)
  if (!maxlipo.begin()) {  // check for max17048
    Serial.println(F("Couldnt find MAX17048, looking for LC709203F.."));
    if (!lc.begin()) {                                                                                                   // check for lc709203f
      Serial.println(F("Couldnt find MAX17048 or LC709203F, continuing but voltage measurements will be inaccurate."));  // ACTION: might consider adding an error register to log this error - 2027 future work?
    } else {                                                                                                             // found lc709203f
      addr0x36 = false;
      Serial.println(F("Found LC709203F"));
      Serial.print("Version: 0x");
      Serial.println(lc.getICversion(), HEX);
      lc.setThermistorB(3950);
      Serial.print("Thermistor B = ");
      Serial.println(lc.getThermistorB());
      lc.setPackSize(LC709203F_APA_3000MAH);
      lc.setAlarmVoltage(3.8);
    }
  } else {  // found max17048
    addr0x36 = true;
    Serial.print(F("Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x"));
    Serial.println(maxlipo.getChipID(), HEX);
  }
#endif

  if (RTCOk) {
    DateTime dt = rtc.now();
    setTime(dt.unixtime());
    //adjustTime(offset * SECS_PER_HOUR);
  }
  //@BERIKOV no RTC -> constant dateStr. Each power-on appends to the same datafile; keep notes of test runs.
  //snprintf(dateStr, sizeof(dateStr), "%2d%02d%02d-%02d%02d%02d", year(), month(), day(), hour(), minute(), second());  // save date-time in a pretty string
  strcpy(dateStr, "00000000-000000");  //@BERIKOV constant reference timestamp (no RTC)

  // Note that having a valid date/time does NOT mean we have a valid position - ACTION - decide what to do while waiting for a positional fix, and how to indicate via neo

  //
  // do not want to create datafile name using date/time for balloon. Always want to use the same name so that upon
  // wake from sleep (equivalent to restart), data is written (concatenated) to the same file. Rocket continues to use
  // a date-time stamped filename. Note I am not altering the byte-size of the datefilename variable, depending on balloon or rocket variant
  //
  // now that timesync is started, can create datafile
  //
  // only create file on first boot (relevant for balloon version)
  if (bootCount == 1) {
    strcat(datafilename, UniqueIDString);

#ifndef BALLOON
    strcat(datafilename, "-");
    strcat(datafilename, dateStr);
#endif

    strcat(datafilename, ".log");
  }
  //Serial.print(F("Log filename:\t"));
  //Serial.println(datafilename);

  SdFile::dateTimeCallback(dateTime);  // sets the file creation date and time - see dateTime function

  // 6/22/2025 test to mitigate random file not created error (noting serial not started so no printout)
  SerialhandleFileList();

  dataFile = fatfs.open(datafilename, FILE_WRITE);
  if (dataFile) {
    dataFile.println(dateStr);  // print date/time at beginning of file
  } else {                      // set an error if the file didn't open
    //Serial.println(F("Could not open file, reboot and use serial cmd to perform directory listing"));
  }

  //
  // set time sync provider
  //
  if (GPSOk) {
#if defined(GPS_ENABLED)
    setSyncProvider(getGPSTime);
#endif
  } else if (RTCOk) {
    setSyncProvider(getRtcTime);
  } else {
    //@BERIKOV no RTC/GPS time source; do not halt. Time is reconstructed from the fixed 100 ms write interval.
    //Serial.println(F("Failed to set internal clock, check for hardware errors and press reset to try again"));
    //err(2, RED);
  }
  setSyncInterval(TIME_SYNC_INTERVAL);  // experimenting with long interval (1800) - originally 300 (5 min)
  //
  // ## INPUT SENSOR CHANGES HERE ##
  //
  // init sensors
  //
  //@BERIKOV init MCP9808 (under-regolith temperature) instead of the IMU
  //
  if (!mcp9808.begin(0x18, &WIRE)) {  // initialize MCP9808 on the STEMMA QT (Wire1) bus
    //Serial.println(F("Could not find MCP9808, check wiring/address"));
    err(2, YELLOW);  // blink neo and wait here forever
  }
  mcp9808.setResolution(3);  //@BERIKOV 0.0625 degC resolution (highest)
  /*  //@BERIKOV original IMU init disabled (no IMU)
  if (!lsm.begin_I2C(0x6a, &WIRE) || !lis.begin_I2C(0x1c, &WIRE)) {  // inititalize IMU and magnetometer sensors
    //Serial.println(F("Could not find a valid IMU, unplug USB and/or remove battery and check wiring"));
    err(2, YELLOW);  // blink neo and wait here forever
  }
  */
  //
  // init cal data structures
  //
  if (!cal.begin()) {  // init cal routines and prints JEDEC info
    //Serial.println(F("Failed to initialize calibration helper, quitting"));
    //Serial.println(F("Press reset and try again"));
    //Serial.println(F("If error repeats, likely HW failure"));
    err(2, RED);  // blink neo and wait here forever - this is a rare error
  }
  //
  // init pressure sensor
  //
  if (!bmp388.begin(NORMAL_MODE, OVERSAMPLING_X8, OVERSAMPLING_SKIP, IIR_FILTER_2, TIME_STANDBY_20MS)) {  // 160MS consider OVERSAMPLING_X16 for pressure (second  parameter for ROCKET
    //Serial.println(F("Could not find the BMP, unplug USB and/or remove battery and check wiring"));
    err(2, YELLOW);  // blink neo and wait here forever
  }

  /*  //@BERIKOV IMU/magnetometer configuration disabled (no IMU)
  //Serial.println(F("Accelerometer settings"));
#if defined(BALLOON)
  lsm.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
#else
  lsm.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);  // higher G range for Rocket
#endif
  lsm.setAccelDataRate(LSM6DS_RATE_104_HZ);

  //Serial.println(F("Gyro settings"));
  lsm.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);
  lsm.setGyroDataRate(LSM6DS_RATE_104_HZ);

  //Serial.println(F("Magnetometer settings"));
  lis.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  lis.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  lis.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis.setDataRate(LIS3MDL_DATARATE_1000_HZ);
  */

  if (!cal.loadCalibration()) {
    //Serial.println(F("No calibration loaded/found... will start with defaults"));
  } else {
    //Serial.println(F("Loaded existing calibration"));
  }

  //filter.begin(FILTER_UPDATE_RATE_HZ);  // start IMU filter  //@BERIKOV no IMU/AHRS

  analogReadResolution(12);  // when using ESP32-S3 or S2, for ext thermistor, set the A/D resolution to 12-bit (0..4095) - can be 8, 10, 12 or 14

  // web handlers Serial.prints are ok here since these are called after Serial is started
  webserver.on("", []() {
    webserver.sendHeader("Location", "/", true);
    webserver.sendHeader("Cache-Control", "no-store");
    webserver.send(200, "text/html", SendHTML());
  });

  webserver.on("/", []() {
    webserver.sendHeader("Location", "/", true);
    webserver.sendHeader("Cache-Control", "no-store");
    webserver.send(200, "text/html", SendHTML());
  });

  webserver.on("/download", []() {
    File32 sendFile = fatfs.open(actionfile, FILE_READ);  // open the local file
    webserver.sendHeader("Content-Type", "text/plain; charset=utf-8", true);
    String content = "attachment; filename=" + String(actionfile);
    webserver.sendHeader("Content-Disposition", content, true);  // this sets up where to save data on the client
    webserver.streamFile(sendFile, "text/plain", 200);           // this streams the contents of the local file into the client file named above
    webserver.sendHeader("Location", "/", true);
    webserver.sendHeader("Cache-Control", "no-cache");
    webserver.send(302, "text/html", SendHTML());
    sendFile.close();
    actionfile[0] = 0;  // testing delete filename after download cmd
  });

  webserver.on("/delete", []() {   // note deleting from webserver DOES NOT require ION to be in LAND mode and therefore files can be deleted without confirmation - be careful!
    if (fatfs.remove(actionfile))  // ACTION: for BALLOON, consider adding LAND mode check - which then requires setting LAND at some point in logic
      Serial.printf("File %s deleted\n", actionfile);
    else
      Serial.println("File not found");
    webserver.sendHeader("Location", "/", true);
    webserver.sendHeader("Cache-Control", "no-cache");
    webserver.send(302, "text/html", SendHTML());
    actionfile[0] = 0;  // testing delete filename after delete cmd
  });

  webserver.on("/cmdmode", []() {
    onCmdEnabled();
    webserver.sendHeader("Location", "/", true);
    webserver.sendHeader("Cache-Control", "no-cache");
    webserver.send(302, "text/html", SendHTML());
  });

  webserver.on("/standby", []() {
    bitClear(flightStatus, CMDMODE);  // clear cmd mode register bit
    onStandby();                      // ACTION reopen files that were closed when entering CMDMODE? was qKeyPressed
    webserver.sendHeader("Location", "/", true);
    webserver.sendHeader("Cache-Control", "no-cache");
    webserver.send(302, "text/html", SendHTML());
  });

  webserver.on("/restart", []() {
    ESP.restart();
  });

  webserver.on("/get", []() {
    String inputMessage1 = webserver.arg(0);
    String inputMessage2 = webserver.arg(1);

    inputMessage1.toCharArray(actionfile, inputMessage1.length() + 1);
    Serial.print(F("File selected: "));
    Serial.println(actionfile);

    inputMessage2.toCharArray(action, inputMessage2.length() + 1);
    Serial.print(F("Action chosen: "));
    Serial.println(action);

    if (!strcmp(action, "Download")) {
      webserver.sendHeader("Location", "/download", true);
      webserver.send(302);
    } else if (!strcmp(action, "Delete")) {
      webserver.sendHeader("Location", "/delete", true);
      webserver.send(302);
    }
  });

  // the following commands are only availble from browser command line
  webserver.on("/serial", []() {
    serialProcessCmd();
    webserver.sendHeader("Location", "/", true);
    webserver.sendHeader("Cache-Control", "no-cache");
    webserver.send(302, "text/html", SendHTML());
  });

  webserver.on("/sysinfo", HTTP_GET, handleSysInfo);
  webserver.on("/list", HTTP_GET, SerialhandleFileList);

  //
  // now we can start serial
  //
  // ACTION: give some thought to avoiding starting Serial (and the 8 sec delay) after the first boot
  // - motivation: save power
  // - challenge: how to stop sleep/wake process, maybe using hw reset button is ok?
  // - remember datafile is synced every 500msec so random resets will loose at most 5 samples
  //
  Serial.begin(115200);
  while (!Serial.available()) {  // wait for a character
    neo.update();
    if ((millis() - waittimer) >= SERIAL_WAIT) {  // if Serial hasn't awoken within 8 sec, start anyway (important when not using USB - so we don't hang waiting for Serial)
      break;                                      // stop waiting and continue
    }
  }
  if (toupper(Serial.read()) == 'Y') {  // check if it's 'Y' and if so run serial cmd processor
    serialProcessCmd();
  }
  //
  // print a bunch of status/dianostic info before starting
  //
  if (bootCount == 1) {
    Serial.println("");
    Serial.print(F("Source code:\t"));
    Serial.println(__FILE__);  // print name of the .ino file for reference to Serial & write to datafile (later)
    Serial.printf("Flash Size = %u KB\n", flash.size() / 1024);
    Serial.print(F("This device's UniqueID = "));
    Serial.println(UniqueIDString);
    Serial.printf("System Date/Time: %02d/%02d/%02d at %02d:%02d:%02d\n", year(), month(), day(), hour(), minute(), second());
    Serial.println("ION will sleep for " + String(SLEEP_TIME) + " seconds, then wake to take data for " + String(AWAKE_TIME) + " seconds");
    Serial.println(F("Completing setup and starting data collection"));
  }

//
// for BALLOON start data collection/save until ready to sleep - for ROCKET, will go directly to loop() and not sleep
// web requests are handled in serialProcessCmd() which is called from pressing W once in Serial mode (pressing Y within 8 sec of reboot)
#ifdef BALLOON
  Serial.println("");
  Serial.println("Boot number: " + String(bootCount));
  onPosAx();  // force LAUNCH status (which starts data saving and turns off wifi)
  static uint32_t awakeTimer = millis();
  while (millis() < (awakeTimer + (AWAKE_TIME * mS_TO_S_FACTOR))) {
    getData();        // would love to flash the neopixel here, but gotta save power
  }
  Serial.println(F("Collected data and now going to sleep"));
  esp_deep_sleep_start();
#endif
}  // end of setup


//
// main - not alot to say here, get data, handle web requests and update the indicator
//
void loop() {
  getData();
  webserver.handleClient();
  neo.update();
}