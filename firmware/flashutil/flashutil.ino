/*
 * Print size, modify date/time, and name for all files in root.
 */
#include <SPI.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"
#include "ff.h"
#include "diskio.h"

// up to 11 characters
#define DISK_LABEL "EXT FLASH"

// for flashTransport definition
#include "flash_config.h"

const byte numChars = 35;
char filename[numChars] = { 0 };  // an array to store the received data
boolean newData = false;
char endMarker = '\n';

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs;
File32 root;
File32 file;

void format_fat12(void) {
// Working buffer for f_mkfs.
#ifdef __AVR__
  uint8_t workbuf[512];
#else
  uint8_t workbuf[4096];
#endif

  // Elm Cham's fatfs objects
  FATFS elmchamFatfs;

  // Make filesystem.
  FRESULT r = f_mkfs("", FM_FAT, 0, workbuf, sizeof(workbuf));
  if (r != FR_OK) {
    Serial.print(F("Error, f_mkfs failed with error code: "));
    Serial.println(r, DEC);
    while (1) yield();
  }

  // mount to set disk label
  r = f_mount(&elmchamFatfs, "0:", 1);
  if (r != FR_OK) {
    Serial.print(F("Error, f_mount failed with error code: "));
    Serial.println(r, DEC);
    while (1) yield();
  }

  // Setting label
  Serial.println(F("Setting disk label to: " DISK_LABEL));
  r = f_setlabel(DISK_LABEL);
  if (r != FR_OK) {
    Serial.print(F("Error, f_setlabel failed with error code: "));
    Serial.println(r, DEC);
    while (1) yield();
  }

  // unmount
  f_unmount("0:");

  // sync to make sure all data is written to flash
  flash.syncBlocks();

  Serial.println(F("Formatted flash!"));
}

void check_fat12(void) {
  // Check new filesystem
  FatVolume fatfs;
  if (!fatfs.begin(&flash)) {
    Serial.println(F("Error, failed to mount newly formatted filesystem!"));
    while (1) delay(1);
  }
}

void deleteFile() {
  static byte ndx = 0;
  char rc;
  Serial.println(F("Type filename to delete: (remember to set Serial Monitor to New Line)"));
  //while (Serial.available() > 0) {
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
      Serial.println(filename);
      if (fatfs.remove(filename))
        Serial.printf("File %s deleted\n", filename);
      else
        Serial.println("File not found");
      listdir();
    }
  }
}

void readFile() {
  static byte ndx = 0;
  char rc;
  Serial.println(F("Type filename to read: (remember to set Serial Monitor to New Line)"));

  //while (Serial.available() > 0) {
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
          // Use the read function to read the next character.
          // You can alternatively use other functions like readUntil, readString,
          // etc. See the fatfs_full_usage example for more details.
          char c = dataFile.read();
          Serial.print(c);
        }
      } else
        Serial.printf("Failed to open file %s\n", filename);
      listdir();
    }
  }
}

void listdir() {
  long dataUsed = 0;
  root.rewind();
  Serial.println("Listing files");
  root.open("/");
  Serial.println();
  while (file.openNext(&root, O_RDONLY)) {
    dataUsed += file.size();
    file.printFileSize(&Serial);
    Serial.write(' ');
    file.printModifyDateTime(&Serial);
    Serial.write(' ');
    file.printName(&Serial);
    if (file.isDir()) {
      Serial.write('/');
    }
    Serial.println();
    file.close();
  }

  if (root.getError()) {
    Serial.println("directory listing failed");
  } else {
    Serial.print(F("Used = "));
    Serial.println(dataUsed);
    Serial.print("Free = ");
    Serial.println(flash.size() - dataUsed);
    Serial.println("Done!");
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  // set CS for LoRa High
  digitalWrite(8, HIGH);
  pinMode(8, OUTPUT);

  flash.begin();
  fatfs.begin(&flash);

  Serial.print("JEDEC ID: 0x");
  Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: ");
  Serial.print(flash.size());
  Serial.println(" Bytes");
  Serial.println("Mounted filesystem");

  flash.setIndicator(LED_BUILTIN, true);
  listdir();
}

void loop() {
  char cmd;
  while (Serial.available()) {
    Serial.read();
  }
  Serial.println(F("FLASH UTIL - Press 'D filename' to delete, 'R filename' to Read, or F to Format (not functional yet)"));

  while (!Serial.available())
    ;
  cmd = toupper(Serial.read());

  if (cmd == 'D') {
    deleteFile();
  }
  if (cmd == 'R') {
    readFile();
  }

  if (cmd == 'F') {
    do {
      Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
      Serial.println(F("This sketch will ERASE ALL DATA on the flash chip and format it with a new filesystem!"));
      Serial.println(F("Type OK (all caps) and press enter to continue."));
      Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
    } while (!Serial.find((char *)"OK"));

    Serial.println(F("Creating and formatting FAT filesystem (this takes ~60 seconds)..."));
    format_fat12();
    check_fat12();
    Serial.println(F("Flash chip successfully formatted with new empty filesystem!"));
  }
}

//--------------------------------------------------------------------+
// fatfs diskio
//--------------------------------------------------------------------+
extern "C" {

  DSTATUS disk_status(BYTE pdrv) {
    (void)pdrv;
    return 0;
  }

  DSTATUS disk_initialize(BYTE pdrv) {
    (void)pdrv;
    return 0;
  }

  DRESULT disk_read(
    BYTE pdrv,    /* Physical drive nmuber to identify the drive */
    BYTE *buff,   /* Data buffer to store read data */
    DWORD sector, /* Start sector in LBA */
    UINT count    /* Number of sectors to read */
  ) {
    (void)pdrv;
    return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
  }

  DRESULT disk_write(
    BYTE pdrv,        /* Physical drive nmuber to identify the drive */
    const BYTE *buff, /* Data to be written */
    DWORD sector,     /* Start sector in LBA */
    UINT count        /* Number of sectors to write */
  ) {
    (void)pdrv;
    return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
  }

  DRESULT disk_ioctl(
    BYTE pdrv, /* Physical drive nmuber (0..) */
    BYTE cmd,  /* Control code */
    void *buff /* Buffer to send/receive control data */
  ) {
    (void)pdrv;

    switch (cmd) {
      case CTRL_SYNC:
        flash.syncBlocks();
        return RES_OK;

      case GET_SECTOR_COUNT:
        *((DWORD *)buff) = flash.size() / 512;
        return RES_OK;

      case GET_SECTOR_SIZE:
        *((WORD *)buff) = 512;
        return RES_OK;

      case GET_BLOCK_SIZE:
        *((DWORD *)buff) = 8;  // erase block size in units of sector size
        return RES_OK;

      default:
        return RES_PARERR;
    }
  }
}
