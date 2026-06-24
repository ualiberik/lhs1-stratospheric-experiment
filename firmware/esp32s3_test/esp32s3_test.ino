// ESP32-S3 self-test sketch
// Выводит в Serial информацию о чипе и периодические тестовые сообщения.
// Board: ESP32-S3 Dev Module. Baud: 115200.
// Совместимо с ядром ESP32 Arduino 2.x и 3.x (только Arduino API, без IDF-заголовков).

#include <Wire.h>
#include <Adafruit_BMP3XX.h>

// На Adafruit QT Py ESP32-S3 две шины I2C:
//   - Wire  (пады):        SDA=GPIO7,  SCL=GPIO6   — сюда припаян MCP9808
//   - Wire1 (STEMMA QT):   SDA=GPIO41, SCL=GPIO40  — сюда подключён BMP388
// Пины обеих шин заданы явно, чтобы всё работало независимо от выбранной в IDE платы.
static const uint8_t PRIMARY_SDA   = 7;
static const uint8_t PRIMARY_SCL   = 6;
static const uint8_t SECONDARY_SDA = 41;  // STEMMA QT
static const uint8_t SECONDARY_SCL = 40;  // STEMMA QT

// MCP9808 — I2C-датчик температуры. Чтение на «голом» Wire, без внешних библиотек.
// Адрес по умолчанию 0x18 (0x18..0x1F при замкнутых A0..A2).
static const uint8_t MCP9808_ADDR     = 0x18;
static const uint8_t MCP9808_REG_TEMP = 0x05;  // ambient temperature
static const uint8_t MCP9808_REG_MFG  = 0x06;  // manufacturer ID == 0x0054
static const uint8_t MCP9808_REG_DEV  = 0x07;  // device ID, старший байт == 0x04

// BMP388 — датчик давления/температуры (Adafruit STEMMA QT), библиотека Adafruit_BMP3XX.
// Адрес по умолчанию 0x77 (0x76 при замкнутой перемычке SDO->GND).
#define SEALEVEL_HPA 1013.25f  // опорное давление на уровне моря для расчёта высоты

static uint32_t counter = 0;
static TwoWire* mcpBus = nullptr;  // шина, на которой найден MCP9808 (nullptr — нет)

static Adafruit_BMP3XX bmp;
static bool        bmpFound   = false;
static const char* bmpBusName = "";
static uint8_t     bmpAddr    = 0x00;

static bool mcp9808ReadReg16(TwoWire& bus, uint8_t reg, uint16_t &out) {
  bus.beginTransmission(MCP9808_ADDR);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom((int)MCP9808_ADDR, 2) != 2) return false;
  out = ((uint16_t)bus.read() << 8) | bus.read();
  return true;
}

static bool mcp9808Detect(TwoWire& bus) {
  uint16_t mfg, dev;
  if (!mcp9808ReadReg16(bus, MCP9808_REG_MFG, mfg)) return false;
  if (!mcp9808ReadReg16(bus, MCP9808_REG_DEV, dev)) return false;
  return (mfg == 0x0054) && ((dev >> 8) == 0x04);
}

// Возвращает температуру в °C, либо NAN при ошибке чтения.
static float mcp9808ReadTempC(TwoWire& bus) {
  uint16_t raw;
  if (!mcp9808ReadReg16(bus, MCP9808_REG_TEMP, raw)) return NAN;
  float temp = (raw & 0x0FFF) / 16.0f;   // 12 бит дробной части (шаг 0.0625 °C)
  if (raw & 0x1000) temp -= 256.0f;       // бит знака
  return temp;
}

// Пробует инициализировать BMP388 на указанной шине (адреса 0x77, затем 0x76).
static bool bmpTryBegin(TwoWire& bus, const char* name) {
  const uint8_t addrs[] = {0x77, 0x76};
  for (uint8_t i = 0; i < 2; i++) {
    if (bmp.begin_I2C(addrs[i], &bus)) {
      bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
      bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
      bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
      bmp.setOutputDataRate(BMP3_ODR_50_HZ);
      bmpBusName = name;
      bmpAddr    = addrs[i];
      return true;
    }
  }
  return false;
}

// Проверка состояния линий БЕЗ подтяжек МК: на исправной шине с внешними
// pull-up обе линии в покое читаются как HIGH. LOW = обрыв/КЗ на GND/неконтакт.
static void i2cLineCheck(uint8_t sdaPin, uint8_t sclPin, const char* name) {
  pinMode(sdaPin, INPUT);  // без INPUT_PULLUP — проверяем именно внешнюю подтяжку
  pinMode(sclPin, INPUT);
  delay(2);
  int sda = digitalRead(sdaPin);
  int scl = digitalRead(sclPin);
  Serial.printf("Line check [%s]: SDA(GPIO%u)=%s  SCL(GPIO%u)=%s%s\n",
                name, sdaPin, sda ? "HIGH" : "LOW",
                sclPin, scl ? "HIGH" : "LOW",
                (sda && scl) ? "  OK" : "  <-- ПРОБЛЕМА (ожидается HIGH)");
}

// Сканирует шину и печатает все найденные I2C-адреса.
static void i2cScan(TwoWire& bus, const char* name) {
  Serial.printf("I2C scan [%s]: ", name);
  uint8_t found = 0;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      Serial.printf("0x%02X ", addr);
      found++;
    }
  }
  if (found == 0) Serial.print("(пусто)");
  Serial.println();
}

void printChipInfo() {
  Serial.println(F("=================================="));
  Serial.println(F(" ESP32-S3 SELF TEST"));
  Serial.println(F("=================================="));

  Serial.printf("Chip model     : %s\n", ESP.getChipModel());
  Serial.printf("Chip cores     : %d\n", ESP.getChipCores());
  Serial.printf("Chip revision  : %d\n", ESP.getChipRevision());
  Serial.printf("CPU frequency  : %u MHz\n", getCpuFrequencyMhz());

  Serial.printf("Flash size     : %u bytes\n", ESP.getFlashChipSize());
  Serial.printf("PSRAM size     : %u bytes\n", ESP.getPsramSize());
  Serial.printf("Free heap      : %u bytes\n", ESP.getFreeHeap());
  Serial.printf("Free PSRAM     : %u bytes\n", ESP.getFreePsram());

  uint64_t mac = ESP.getEfuseMac();
  uint8_t* m = (uint8_t*)&mac;
  Serial.printf("Base MAC       : %02X:%02X:%02X:%02X:%02X:%02X\n",
                m[0], m[1], m[2], m[3], m[4], m[5]);

  Serial.printf("SDK version    : %s\n", ESP.getSdkVersion());
  Serial.println(F("=================================="));
}

void setup() {
  Serial.begin(115200);

  // Ждём подключения монитора порта (актуально для нативного USB CDC у S3)
  uint32_t start = millis();
  while (!Serial && (millis() - start) < 3000) {
    delay(10);
  }

  delay(500);
  Serial.println();
  Serial.println(F("[BOOT] Serial OK"));

  printChipInfo();

  // Сначала диагностика линий (внешние подтяжки должны держать обе линии в HIGH).
  i2cLineCheck(PRIMARY_SDA, PRIMARY_SCL, "Wire/pads");
  i2cLineCheck(SECONDARY_SDA, SECONDARY_SCL, "Wire1/QT");

  // ВАЖНО: на QT Py ESP32-S3 НЕТ подтягивающих резисторов на I2C (ни пады, ни QT).
  // Включаем внутренние pull-up МК как подстраховку (на коротких проводах хватает;
  // для надёжности всё равно поставьте внешние 4.7 кОм SDA->3V3 и SCL->3V3).
  pinMode(PRIMARY_SDA, INPUT_PULLUP);
  pinMode(PRIMARY_SCL, INPUT_PULLUP);
  pinMode(SECONDARY_SDA, INPUT_PULLUP);
  pinMode(SECONDARY_SCL, INPUT_PULLUP);

  // Обе шины с явными пинами — не зависят от выбранной в IDE платы.
  Wire.begin(PRIMARY_SDA, PRIMARY_SCL);
  Wire.setClock(100000);  // 100 кГц — устойчивее при слабой подтяжке/длинных проводах
  i2cScan(Wire, "Wire/pads");

  Wire1.begin(SECONDARY_SDA, SECONDARY_SCL);
  Wire1.setClock(100000);
  i2cScan(Wire1, "Wire1/QT");

  // MCP9808 ищем на обеих шинах (QT, затем пады).
  if (mcp9808Detect(Wire1)) mcpBus = &Wire1;
  if (!mcpBus && mcp9808Detect(Wire)) mcpBus = &Wire;
  if (mcpBus) {
    Serial.printf("MCP9808        : найден на %s (0x%02X)\n",
                  (mcpBus == &Wire) ? "Wire/pads" : "Wire1/QT", MCP9808_ADDR);
  } else {
    Serial.printf("MCP9808        : НЕ найден (0x%02X) — проверьте подключение/адрес\n", MCP9808_ADDR);
  }

  // BMP388 (STEMMA QT) ищем на QT-шине, затем на падах.
  bmpFound = bmpTryBegin(Wire1, "Wire1/QT") || bmpTryBegin(Wire, "Wire/pads");
  if (bmpFound) {
    Serial.printf("BMP388         : найден на %s (0x%02X)\n", bmpBusName, bmpAddr);
  } else {
    Serial.println(F("BMP388         : НЕ найден (0x77/0x76) — проверьте подключение/адрес"));
  }

  Serial.println(F("[TEST] Запуск периодических тестовых сообщений..."));
}

void loop() {
  counter++;

  // При временном сбое чтения пытаемся заново найти датчик на обеих шинах.
  if (!mcpBus) {
#ifdef WIRE1_PIN_DEFINED
    if (mcp9808Detect(Wire1)) mcpBus = &Wire1;
#endif
    if (!mcpBus && mcp9808Detect(Wire)) mcpBus = &Wire;
  }
  float tempC = mcpBus ? mcp9808ReadTempC(*mcpBus) : NAN;

  Serial.printf("[TEST #%u] uptime=%u ms, free_heap=%u bytes",
                counter,
                millis(),
                ESP.getFreeHeap());
  if (isnan(tempC)) {
    Serial.print(", MCP9808=n/a");
    mcpBus = nullptr;
  } else {
    Serial.printf(", MCP9808=%.4f C", tempC);
  }

  if (bmpFound && bmp.performReading()) {
    Serial.printf(", BMP388: T=%.2f C P=%.2f hPa Alt=%.2f m",
                  bmp.temperature,
                  bmp.pressure / 100.0,            // Па -> гПа
                  bmp.readAltitude(SEALEVEL_HPA));
  } else {
    Serial.print(", BMP388=n/a");
  }
  Serial.println();

  delay(1000);
}
