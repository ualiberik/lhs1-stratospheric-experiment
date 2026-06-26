// ion_no-regolith_c6.ino
// Bench-test firmware for ESP32-C6 SuperMini, no-regolith configuration.
// Sensors: BMP388 (temperature + pressure), MCP9808 (temperature, no regolith shield).
// No battery monitor. No LoRa. No GPS. No NeoPixel.
//
// Deep sleep cycle: AWAKE_TIME s collecting → SLEEP_TIME s sleeping → repeat.
// Data format: column positions match ion_2026_berikov so cooling_analysis.py works unchanged.
//   col 0: deviceID    col 1: status(0)   col 2: BMP temp   col 3: pressure
//   col 4: altitude    col 5: sealevel    col 6: MCP temp
//
// Serial command mode: press 'Y' within 8 s of boot.
//   L = list files   D = dump data file   X = delete data file   E = exit/restart
//
// Required libraries: BMP388_DEV, Adafruit_MCP9808, LittleFS (built into ESP32 Arduino core)

#include <Wire.h>
#include <BMP388_DEV.h>
#include <Adafruit_MCP9808.h>
#include <LittleFS.h>

// ── Timing ────────────────────────────────────────────────────────────────────
#define uS_TO_S_FACTOR 1000000ULL
#define mS_TO_S_FACTOR 1000UL
#define SLEEP_TIME     60     // seconds (flight value: 600)
#define AWAKE_TIME     60     // seconds
#define WRITE_INTERVAL 100    // ms between samples → 600 samples per burst
#define SERIAL_WAIT    8000   // ms to wait for Serial 'Y' on boot

// ── Pin definitions ───────────────────────────────────────────────────────────
// Adjust these to match your ESP32-C6 SuperMini board wiring.
#define LED_PIN 8    // built-in LED on many ESP32-C6 SuperMini boards
#define SDA_PIN 6    // I2C SDA
#define SCL_PIN 7    // I2C SCL

// ── Separator ─────────────────────────────────────────────────────────────────
// Must match SEPARATOR in cooling_analysis.py. Written at the start of each wakeup.
#define SEPARATOR "00000000-000000"

// ── Persistent state (survives deep sleep, cleared on power-off) ──────────────
RTC_DATA_ATTR int bootCount = 0;
RTC_DATA_ATTR bool baselinePressSet = false;
RTC_DATA_ATTR float savedSealevel = 1013.25f;

// ── Globals ───────────────────────────────────────────────────────────────────
BMP388_DEV bmp388;
Adafruit_MCP9808 mcp9808;

float temperature, pressure, altitude;
char deviceID[13];    // 12 hex chars from MAC + null
char datafilename[28];  // "/ION-XXXXXXXXXXXX.log"

// ── Helpers ───────────────────────────────────────────────────────────────────
void blinkLED(int n, int ms = 150) {
  for (int i = 0; i < n; i++) {
    digitalWrite(LED_PIN, HIGH); delay(ms);
    digitalWrite(LED_PIN, LOW);  delay(ms);
  }
}

void buildDeviceID() {
  uint64_t mac = ESP.getEfuseMac();
  snprintf(deviceID, sizeof(deviceID), "%04X%08X",
           (uint16_t)(mac >> 32), (uint32_t)mac);
}

// ── Serial command mode ───────────────────────────────────────────────────────
void serialCmdMode() {
  Serial.println(F("\n=== ION-C6 Command Mode ==="));
  Serial.printf("Data file: %s\n", datafilename);
  Serial.println(F("L = list files | D = dump data | X = delete data | E = exit/restart"));

  while (true) {
    while (!Serial.available()) {}
    char cmd = toupper(Serial.read());

    if (cmd == 'L') {
      File root = LittleFS.open("/");
      File f = root.openNextFile();
      while (f) {
        Serial.printf("  %-24s  %lu bytes\n", f.name(), (unsigned long)f.size());
        f = root.openNextFile();
      }
      root.close();

    } else if (cmd == 'D') {
      Serial.printf(">>> Dumping %s\n", datafilename);
      File f = LittleFS.open(datafilename, "r");
      if (f) {
        while (f.available()) Serial.write(f.read());
        f.close();
        Serial.println(F(">>> End of file"));
      } else {
        Serial.println(F("File not found."));
      }

    } else if (cmd == 'X') {
      LittleFS.remove(datafilename);
      Serial.printf("Deleted %s\n", datafilename);

    } else if (cmd == 'E') {
      Serial.println(F("Restarting..."));
      delay(100);
      ESP.restart();
    }
  }
}

// ── setup ─────────────────────────────────────────────────────────────────────
void setup() {
  ++bootCount;
  pinMode(LED_PIN, OUTPUT);
  blinkLED(1);  // signal: awake

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!LittleFS.begin(true)) {
    blinkLED(5, 300);  // filesystem error
  }

  buildDeviceID();
  snprintf(datafilename, sizeof(datafilename), "/ION-%s.log", deviceID);

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_TIME * uS_TO_S_FACTOR);

  // Init sensors
  if (!bmp388.begin(NORMAL_MODE, OVERSAMPLING_X8, OVERSAMPLING_SKIP, IIR_FILTER_2, TIME_STANDBY_20MS)) {
    blinkLED(3, 300);  // BMP388 not found — data collection will produce no BMP rows
  }
  if (!mcp9808.begin(0x18, &Wire)) {
    blinkLED(2, 300);  // MCP9808 not found
  }
  mcp9808.setResolution(3);  // 0.0625 °C resolution

  // Wait for Serial 'Y'
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial.available() && (millis() - t0) < SERIAL_WAIT) {}
  if (Serial.available() && toupper(Serial.peek()) == 'Y') {
    Serial.read();
    serialCmdMode();  // never returns — ends with ESP.restart()
  }

  // Baseline pressure: set once after first power-on, kept across deep-sleep cycles
  if (!baselinePressSet) {
    delay(200);
    if (bmp388.getMeasurements(temperature, pressure, altitude)) {
      savedSealevel = pressure;
      bmp388.setSeaLevelPressure(savedSealevel);
      baselinePressSet = true;
    }
  } else {
    bmp388.setSeaLevelPressure(savedSealevel);
  }

  // Open file and write separator (marks start of this wakeup burst)
  File dataFile = LittleFS.open(datafilename, "a");
  if (dataFile) {
    dataFile.println(SEPARATOR);
  }

  // Collect data for AWAKE_TIME seconds
  uint32_t awakeStart = millis();
  uint32_t lastWrite  = 0;

  while ((millis() - awakeStart) < (uint32_t)AWAKE_TIME * mS_TO_S_FACTOR) {
    uint32_t now = millis();
    if (now - lastWrite >= WRITE_INTERVAL) {
      lastWrite = now;

      float mcpTemp = mcp9808.readTempC();
      if (bmp388.getMeasurements(temperature, pressure, altitude)) {
        char line[96];
        // Columns: deviceID, status, bmp_temp, pressure, altitude, sealevel, mcp_temp
        snprintf(line, sizeof(line),
                 "%s,%u,%8.2f,%8.2f,%8.2f,%8.2f,%6.2f",
                 deviceID, 0u,
                 temperature, pressure, altitude, savedSealevel, mcpTemp);
        if (dataFile) dataFile.println(line);
      }
    }
  }

  if (dataFile) {
    dataFile.flush();
    dataFile.close();
  }

  blinkLED(2, 100);  // signal: going to sleep
  esp_deep_sleep_start();
}

void loop() {
  // Never reached — firmware sleeps in setup()
}
