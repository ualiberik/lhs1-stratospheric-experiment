// ion_flash_dumper.ino
// -----------------------------------------------------------------------------
// Standalone, READ-ONLY dumper for the ION flight log.
// It does NOT modify ion_2026 in any way. It mounts the SAME FAT partition on the
// ESP32-S3 internal flash, reads ION-5863FC.log, and prints it over USB Serial
// between explicit markers. Block reads + yield() => never trips the watchdog,
// so the full multi-MB file dumps in one pass without the board resetting.
//
// SAFETY / IMPORTANT before uploading (otherwise the log can be lost):
//   * Board:           Adafruit QT Py ESP32-S3  (the same board as ion_2026)
//   * Partition Scheme: MUST be identical to what ion_2026 was compiled with.
//                       (If unsure, do not change Tools settings between the two.)
//   * Tools -> "Erase All Flash Before Sketch Upload" = DISABLED.
//   * This sketch only READS. It never formats and never writes to flash.
//
// After you have the data, re-flash ion_2026 (with SLEEP_TIME = 600) for flight.
// -----------------------------------------------------------------------------

#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

// Same flash/filesystem objects and init as ion_2026 (lines 328-330, 1559-1564).
Adafruit_FlashTransport_ESP32 flashTransport;
Adafruit_SPIFlash flash(&flashTransport);
FatVolume fatfs;

// Log filename = "ION-" + UniqueID + ".log"  (this board's UniqueID is 5863FC).
const char *LOGNAME = "ION-5863FC.log";

void listFiles() {
  File32 root, file;
  char nm[64];
  if (!root.open("/")) {
    Serial.println("ERR: cannot open root directory");
    return;
  }
  Serial.println("Files on flash (name  size):");
  while (file.openNext(&root, O_RDONLY)) {
    file.getName(nm, sizeof(nm));
    Serial.print("  ");
    Serial.print(nm);
    Serial.print("  ");
    Serial.println((uint32_t)file.size());
    file.close();
  }
  root.close();
}

void dumpLog() {
  File32 f = fatfs.open(LOGNAME, FILE_READ);
  if (!f) {
    Serial.print("ERR: cannot open ");
    Serial.println(LOGNAME);
    listFiles();
    return;
  }

  uint32_t sz = (uint32_t)f.size();
  Serial.print("<<<DUMP-START name=");
  Serial.print(LOGNAME);
  Serial.print(" size=");
  Serial.print(sz);
  Serial.println(">>>");

  uint8_t buf[512];
  uint32_t total = 0;
  while (f.available()) {
    int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;
    Serial.write(buf, n);
    total += n;
    yield();  // feed the task watchdog / let the USB task run -> no reset
  }
  f.close();
  Serial.flush();

  Serial.print("\n<<<DUMP-END bytes=");
  Serial.print(total);
  Serial.println(">>>");
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 4000)) {
    delay(10);  // wait briefly for the USB-CDC host (DTR) to connect
  }
  delay(300);

  if (!flash.begin()) {
    Serial.println("ERR: flash.begin() failed");
    return;
  }
  if (!fatfs.begin(&flash)) {
    Serial.println("ERR: fatfs.begin() failed (partition scheme mismatch?)");
    return;
  }

  Serial.println("=== ION flash dumper ready (read-only) ===");
  listFiles();
  dumpLog();
  Serial.println("Send any character to dump again.");
}

void loop() {
  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    dumpLog();
  }
  delay(50);
}
