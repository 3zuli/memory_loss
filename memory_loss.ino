// Arduino Nano, may need to select processor "ATmega328P (Old bootloader)"

// A custom version of E24 by @blemasle https://github.com/blemasle/arduino-e24
// Transmits only single byte addresses, this makes it work with 24LC16 
#include "eeprom.h"

#define FORCE_EEPROM_INIT 0
// #define SIMULATE_FAIL_AFTER 10

#define WRITE_CYCLE_DELAY_FAST 1 // [ms] // EEPROM read + write cycle takes about 7ms
#define WRITE_CYCLE_DELAY_SLOW 5000 // [ms]

#define SERIAL_PRINT_INTERVAL 500
#define SAVE_CHECKPOINT_INTERVAL 2000

#define SECRET 0xDEADBEEF
#define INFO_DATA_ADDR 0
#define TEST_DATA_BASE_ADDR 16
#define TEST_DATA_LENGTH 8

#define BUFFER_LENGTH 10
#define DATA_ADDR 0x10

#define LED_PIN 13

typedef struct {
  uint32_t secret = SECRET;
  uint32_t countCheckpoint = 0;
  uint32_t firstError = 0;
  uint32_t currentDataOffset = 0;
} EEInfo;

typedef struct {
  uint8_t data[TEST_DATA_LENGTH];
} EEData;


const EEData defaultTestData = {.data = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff}};
const EEData testData[2] = {
  {.data = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}},
  {.data = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}}
};

EEInfo currentEEInfo;
EEData currentTestData;
EEData prevTestData;
uint8_t currentTestDataIndex = 0;

uint32_t writeCycleCounter = 0;
uint32_t writeCycleDelay = WRITE_CYCLE_DELAY_FAST;
bool firstLoop = true;

E24 e24 = E24(E24Size_t::E24_16K, E24_DEFAULT_ADDR);


void printEEInfo(const EEInfo &eeinfo);
void printTestData(const EEData &eedata, uint8_t format = HEX);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pinMode(LED_PIN, OUTPUT);
  delay(100);
  // Serial.println(sizeof(EEInfo));
  Serial.println("Start");


  EEInfo info = readEEInfo();
  Serial.print("Got secret ");
  Serial.println(info.secret, HEX);
  if (info.secret != SECRET || FORCE_EEPROM_INIT) {
    Serial.println("Config not found, writing it");
    initEEInfo();
    info = readEEInfo();
  }
  currentEEInfo = info;
  printEEInfo(currentEEInfo);

  if (currentEEInfo.countCheckpoint == 0) {
    Serial.println("Test data not found, writing it");
    initTestData();
  }
  else {
    Serial.print("Resuming write cycle count from checkpoint ");
    Serial.println(currentEEInfo.countCheckpoint);
    writeCycleCounter = currentEEInfo.countCheckpoint;
  }

  // Write test data for the first time
  writeTestData(testData[currentTestDataIndex]);
  
  currentTestData = readTestData();
  Serial.println("Starting with test data:");
  printTestData(currentTestData);
  
}

void loop() {
  // uint32_t t1 = millis();
  prevTestData = currentTestData;
  writeCycleCounter++;

  writeTestData(testData[currentTestDataIndex]); // Write test data into EEPROM
  currentTestData = readTestData(); // Read it back

  if (writeCycleCounter % SERIAL_PRINT_INTERVAL == 0) {
    Serial.println(writeCycleCounter);
  }
  if (writeCycleCounter % SAVE_CHECKPOINT_INTERVAL ==0) {
    currentEEInfo.countCheckpoint = writeCycleCounter;
    writeEEInfo(currentEEInfo);
    Serial.println("Saved checkpoint");
    Serial.print("Written data: "); printTestData(testData[currentTestDataIndex]);
    Serial.print("Read data:    "); printTestData(currentTestData);
  }

  #ifdef SIMULATE_FAIL_AFTER
    // Simulate eeprom failure by changing the read value
    if (writeCycleCounter >= SIMULATE_FAIL_AFTER) {
      currentTestData.data[0] = random(256);
    }
  #endif

  // Check if the read data has changed
  if (!checkTestData(testData[currentTestDataIndex], currentTestData)) {
    Serial.println("READ DATA MISMATCH!!!");
    Serial.print("Cycle:        "); Serial.println(writeCycleCounter);
    Serial.print("Written data: "); printTestData(testData[currentTestDataIndex]);
    Serial.print("Read data:    "); printTestData(currentTestData);
    if (currentEEInfo.firstError == 0) {
      // If this is the first error, save it and save a count checkpoint as well
      currentEEInfo.firstError = writeCycleCounter;
      currentEEInfo.countCheckpoint = writeCycleCounter;
      writeEEInfo(currentEEInfo);
    }
    Serial.print("First error:  "); Serial.println(currentEEInfo.firstError);
    writeCycleDelay = WRITE_CYCLE_DELAY_SLOW;
  }

  // Status blinkenlights
  if (currentEEInfo.firstError == 0) {
    digitalWrite(LED_PIN, (writeCycleCounter >> 4) & 0x01);
    // digitalWrite(LED_PIN, (writeCycleCounter % 50) > 25);
  }
  else {
    digitalWrite(LED_PIN, writeCycleCounter & 0x01);
  }
  // uint32_t t2 = millis();
  // Serial.println(t2-t1);

  currentTestDataIndex = (currentTestDataIndex + 1) % 2;
  delay(writeCycleDelay);
}

void initEEInfo() {
  // Store an EEInfo struct with default values into EEPROM
  EEInfo defaultInfo;
  e24.write(INFO_DATA_ADDR, (uint8_t*)&defaultInfo, sizeof(EEInfo));
}

EEInfo readEEInfo() {
  // Read stored EEinfo struct from EEPROM
  EEInfo result;
  e24.read(INFO_DATA_ADDR, (uint8_t*)&result, sizeof(EEInfo));
  return result;
}

void writeEEInfo(const EEInfo &eeinfo) {
  // Write the specified EEinfo into EEPROM
  e24.write(INFO_DATA_ADDR, (uint8_t*)&eeinfo, sizeof(EEInfo));
}

void initTestData() {
  // Store an EEData struct with default values into EEPROM at the address TEST_DATA_BASE_ADDR + currentEEInfo.currentDataOffset,
  // increment the checkpoint counter and store EEInfo
  // e24.write(TEST_DATA_BASE_ADDR + currentEEInfo.currentDataOffset, (uint8_t*)&defaultTestData, sizeof(EEData));
  writeTestData(defaultTestData);
  currentEEInfo.countCheckpoint = 1;
  writeEEInfo(currentEEInfo);
}

EEData readTestData() {
  // Read EEData from EEPROM at the address TEST_DATA_BASE_ADDR + currentEEInfo.currentDataOffset
  EEData result;
  e24.read(TEST_DATA_BASE_ADDR + currentEEInfo.currentDataOffset, (uint8_t*)&result, sizeof(EEData));
  return result;
}

void writeTestData(const EEData &eedata) {
  // Write EEData specified by `eedata` into EEPROM at the address TEST_DATA_BASE_ADDR + currentEEInfo.currentDataOffset
  e24.write(TEST_DATA_BASE_ADDR + currentEEInfo.currentDataOffset, (uint8_t*)&eedata, sizeof(EEData));
}

bool checkTestData(const EEData &oldData, const EEData &newData) {
  for (int i=0; i<TEST_DATA_LENGTH; i++) {
    if (oldData.data[i] != newData.data[i])
      return false;
  }
  return true;
}

void printEEInfo(const EEInfo &eeinfo) {
  Serial.println("EEInfo:");
  Serial.print("  secret:            "); Serial.println(eeinfo.secret, HEX);
  Serial.print("  countCheckpoint:   "); Serial.println(eeinfo.countCheckpoint);
  Serial.print("  firstError:        "); Serial.println(eeinfo.firstError);
  Serial.print("  currentDataOffset: "); Serial.println(eeinfo.currentDataOffset);
}

void printTestData(const EEData &eedata, uint8_t format) {
  for (int i=0; i<TEST_DATA_LENGTH; i++) {
    Serial.print(eedata.data[i], format);
    Serial.print(" ");
  }
  Serial.println();
}

void eepromTest() {
  Serial.println("write single");
  //write two arbitrary bytes
  e24.write(DATA_ADDR, 101);
  e24.write(DATA_ADDR + 1, 102);

  Serial.println("read single");
  //read the first byte
  uint8_t first = e24.read(DATA_ADDR);
  //read the next byte
  uint8_t second = e24.read();
  Serial.println(first);
  Serial.println(second);

  uint8_t data[BUFFER_LENGTH];
  for(uint8_t i = 0; i < BUFFER_LENGTH; i++) {
      data[i] = i;
  }
  
  Serial.println("write buff");
  //write the entire array
  e24.write(DATA_ADDR, data, BUFFER_LENGTH);

  Serial.println("read buff");
  //read back the array
  e24.read(DATA_ADDR, data, BUFFER_LENGTH);

  Serial.println("result");
  for(uint8_t i = 0; i < BUFFER_LENGTH; i++) {
      Serial.print(data[i]);
      Serial.print(" ");
  }
  Serial.println();
}
