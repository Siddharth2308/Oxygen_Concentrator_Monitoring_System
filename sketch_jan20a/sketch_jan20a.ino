



/*  Pinout
  sd card :

  CS   - D5
  MOSI - D23
  SCK  - D18
  MISO - D19

  i2c modules ( 3231 RTC + 2 ADS1115 + flow sensor )

  SDA - D21
  SCL - 22

  accelerometer  (software serial)

  RX - TX2
  TX - RX2

*/
#include "WiFi.h"
#include "ESPAsyncWebServer.h"
#include "AsyncTCP.h"
#include "Wire.h"
#include "SPI.h"
#include "SD.h"
#include "FS.h"
#include "SFM3X00.h"
#include "Adafruit_ADS1X15.h"
#include "DFRobot_WT61PC.h"

//----------------------------------------RTC---------------------------------------
#define DS3231_READ  0xD1
#define DS3231_WRITE 0xD0
#define DS3231_ADDR  0x68

#define DS3231_SECONDS  0x00
#define DS3231_MINUTES  0x01
#define DS3231_HOURS  0x02
#define DS3231_DAY 0x03
#define DS3231_DATE 0x04
#define DS3231_CEN_MONTH 0x05
#define DS3231_DEC_YEAR 0x06
#define DS3231_TEMP_MSB 0x11
#define DS3231_TEMP_LSB 0x12

//----------------------------------------ACCELEROMETER---------------------------------------
#define RXD2 16
#define TXD2 17

//----------------------------------------SD CARD---------------------------------------
#define SD_CS 5

//----------------------------------------VARIABLES---------------------------------------

int16_t adc0, adc1, adc2, adc3 , adc20, adc21, adc22, adc23;
float temp , volts0, volts1, volts2, volts3 , volts20, volts21, volts22, volts23 , accX, accY, accZ, gyroX, gyroY, gyroZ, angX, angY, angZ , flow ;
char const * wdays[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
uint8_t hours, minutes, seconds, datee, month, yearr, day ;
String dataMessage;

SFM3X00 flowSensor( 0x40 );
Adafruit_ADS1115 ads1 ;
Adafruit_ADS1115 ads2 ;
DFRobot_WT61PC sensor(&Serial2);

void setup(void)
{
  Serial.begin(115200);
  Wire.begin();

  //----------------------------------------SD INIT---------------------------------------

  Serial.print("Initializing SD card...");
  SD.begin(SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println("Failed to initialize SD Card.");
    while (1);
  }
  Serial.println("initialization of SD card done.");

  File file = SD.open("/data.txt");
  if (!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.txt", "ESP32 and SD Card \r\n");
  }
  else {
    Serial.println("File already exists");
  }
  file.close();

  //----------------------------------------ADS1115 INIT---------------------------------------

  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS 1.");
    while (1);
  }
  if (!ads2.begin(0x49)) {
    Serial.println("Failed to initialize ADS 2.");
    while (1);
  }

  // ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V     0.1875mV (default)
  // ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V     0.125mV
  // ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V     0.0625mV
  // ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V     0.03125mV
  // ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V     0.015625mV
  //  ads1.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V     0.0078125mV
  //  ads2.setGain(GAIN_SIXTEEN);

  //----------------------------------------ACCELERO INIT---------------------------------------

  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  sensor.modifyFrequency(FREQUENCY_200HZ);

  /*  Revise the data output data frequncy of sensor FREQUENCY_0_1HZ for 0.1Hz, FREQUENCY_0_5HZ for 0.5Hz,
      FREQUENCY_1HZ for 1Hz, FREQUENCY_2HZ for 2Hz,FREQUENCY_5HZ for 5Hz, FREQUENCY_10HZ for 10Hz, FREQUENCY_20HZ for 20Hz,
      FREQUENCY_50HZ for 50Hz,FREQUENCY_100HZ for 100Hz, FREQUENCY_125HZ for 125Hz, FREQUENCY_200HZ for 200Hz. */

  //----------------------------------------RTC INIT---------------------------------------

  //      writeRegister(DS3231_HOURS, 12);
  //      writeRegister(DS3231_MINUTES, 37);
  //      writeRegister(DS3231_SECONDS, 0);
  //
  //      writeRegister(DS3231_DAY, 7);
  //      writeRegister(DS3231_DATE, 23);
  //      writeRegister(DS3231_CEN_MONTH, 01);
  //      writeRegister(DS3231_DEC_YEAR, 22);

  //----------------------------------------FLOW INIT---------------------------------------

  flowSensor.begin();
}

void loop(void)
{
  day     = readRegister(DS3231_DAY)     ;
  hours   = readRegister(DS3231_HOURS)   ;
  minutes = readRegister(DS3231_MINUTES) ;
  seconds = readRegister(DS3231_SECONDS) ;
  datee   = readRegister(DS3231_DATE)    ;
  month   = readRegister(DS3231_CEN_MONTH);
  yearr   = readRegister(DS3231_DEC_YEAR) ;
  temp    = ( ( readRegister(DS3231_TEMP_MSB) << 8 | readRegister(DS3231_TEMP_LSB) ) >> 6 ) / 4.0f ;

  adc0 = ads1.readADC_SingleEnded(0);
  adc1 = ads1.readADC_SingleEnded(1);
  adc2 = ads1.readADC_SingleEnded(2);
  adc3 = ads1.readADC_SingleEnded(3);

  adc20 = ads2.readADC_SingleEnded(0);
  adc21 = ads2.readADC_SingleEnded(1);
  adc22 = ads2.readADC_SingleEnded(2);
  adc23 = ads2.readADC_SingleEnded(3);

  volts0 = ads1.computeVolts(adc0);
  volts1 = ads1.computeVolts(adc1);
  volts2 = ads1.computeVolts(adc2);
  volts3 = ads1.computeVolts(adc3);

  volts20 = ads2.computeVolts(adc20);
  volts21 = ads2.computeVolts(adc21);
  volts22 = ads2.computeVolts(adc22);
  volts23 = ads2.computeVolts(adc23);

  float flow = flowSensor.readFlow();

  if (flowSensor.checkRange(flow))
  {
    Serial.print("flow exceeded sensor limits:  ");
    Serial.print(flow);
    Serial.println(" slm");
  }
  else
  {
    Serial.print("flow : ");
    Serial.print(flow);
    Serial.println(" slm");
  }

  if (sensor.available())
  {
    accX = sensor.Acc.X ; accY = sensor.Acc.Y ; accZ = sensor.Acc.Z ;
    gyroX = sensor.Gyro.X ; gyroY = sensor.Gyro.Y, gyroZ = sensor.Gyro.Z ;
    angX = sensor.Angle.X , angY = sensor.Angle.Y, angZ = sensor.Angle.Z ;
  }

  Serial.println("-----------------------------------------------------------");
  Serial.print("AIN0: "); Serial.print(adc0); Serial.print("  "); Serial.print(volts0); Serial.println("V");
  Serial.print("AIN1: "); Serial.print(adc1); Serial.print("  "); Serial.print(volts1); Serial.println("V");
  Serial.print("AIN2: "); Serial.print(adc2); Serial.print("  "); Serial.print(volts2); Serial.println("V");
  Serial.print("AIN3: "); Serial.print(adc3); Serial.print("  "); Serial.print(volts3); Serial.println("V");

  Serial.println("-----------------------------------------------------------");
  Serial.print("AIN20: "); Serial.print(adc20); Serial.print("  "); Serial.print(volts20); Serial.println("V");
  Serial.print("AIN21: "); Serial.print(adc21); Serial.print("  "); Serial.print(volts21); Serial.println("V");
  Serial.print("AIN22: "); Serial.print(adc22); Serial.print("  "); Serial.print(volts22); Serial.println("V");
  Serial.print("AIN23: "); Serial.print(adc23); Serial.print("  "); Serial.print(volts23); Serial.println("V");

  //    Serial.print("Acc\t"); Serial.print(sensor.Acc.X); Serial.print("\t"); Serial.print(sensor.Acc.Y); Serial.print("\t"); Serial.println(sensor.Acc.Z); //acceleration information of X,Y,Z
  //    Serial.print("Gyro\t"); Serial.print(sensor.Gyro.X); Serial.print("\t"); Serial.print(sensor.Gyro.Y); Serial.print("\t"); Serial.println(sensor.Gyro.Z); //angular velocity information of X,Y,Z
  Serial.print("Angle\t"); Serial.print( angX); Serial.print("\t"); Serial.print( angY); Serial.print("\t"); Serial.println( angZ); //angle information of X, Y, Z
  Serial.println(" ");

  Serial.println("-----------------------------------------------------------");
  Serial.print("Time is ");
  Serial.print(readRegister(DS3231_HOURS));
  Serial.print(":");
  Serial.print(readRegister(DS3231_MINUTES));
  Serial.print(":");
  Serial.print(readRegister(DS3231_SECONDS));
  Serial.print("\t");
  Serial.print(" Date is ");
  Serial.print(readRegister(DS3231_DATE));
  Serial.print("-");
  Serial.print(readRegister(DS3231_CEN_MONTH));
  Serial.print("-");
  Serial.print(readRegister(DS3231_DEC_YEAR));
  Serial.print("\t");
  Serial.print("Temp = ");
  Serial.println( ( ( readRegister(DS3231_TEMP_MSB) << 8 | readRegister(DS3231_TEMP_LSB) ) >> 6 ) / 4.0f );
  delay(1000);
}


//----------------------------------------RTC FUNCTIONS---------------------------------------

uint8_t toBcd(uint8_t num)
{
  uint8_t bcd = ((num / 10) << 4) + (num % 10);
  return bcd;
}

uint8_t fromBcd(uint8_t bcd)
{
  uint8_t num = (10 * ((bcd & 0xf0) >> 4)) + (bcd & 0x0f);
  return num;
}

uint8_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDR, 1);
  return fromBcd(Wire.read());
}

void writeRegister(uint8_t reg, uint8_t data)
{
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(reg);
  Wire.write(toBcd(data));
  Wire.endTransmission();
}


//----------------------------------------SD FUNCTIONS---------------------------------------

// Write the sensor readings on the SD card
void logSDCard() {
  dataMessage =  "Time = " + String(hours)    + ":"  + String(minutes) + ":" + String(seconds) + "\t"
                 "Date = " + String(datee)    + ":"  + String(month)   + ":" + String(yearr)  + "\t"
                 "Day = " + wdays[ day - 1 ]  + "\t" + "Temp = "       + String(temp)  +  "\n" ;


  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/data.txt", dataMessage.c_str());
}

void listDir(fs::FS &fs, const char * dirname, uint8_t levels) {
  Serial.printf("Listing directory: %s\n", dirname);

  File root = fs.open(dirname);
  if (!root) {
    Serial.println("Failed to open directory");
    return;
  }
  if (!root.isDirectory()) {
    Serial.println("Not a directory");
    return;
  }

  File file = root.openNextFile();
  while (file) {
    if (file.isDirectory()) {
      Serial.print("  DIR : ");
      Serial.println(file.name());
      if (levels) {
        listDir(fs, file.name(), levels - 1);
      }
    } else {
      Serial.print("  FILE: ");
      Serial.print(file.name());
      Serial.print("  SIZE: ");
      Serial.println(file.size());
    }
    file = root.openNextFile();
  }
}

void createDir(fs::FS &fs, const char * path) {
  Serial.printf("Creating Dir: %s\n", path);
  if (fs.mkdir(path)) {
    Serial.println("Dir created");
  } else {
    Serial.println("mkdir failed");
  }
}

void removeDir(fs::FS &fs, const char * path) {
  Serial.printf("Removing Dir: %s\n", path);
  if (fs.rmdir(path)) {
    Serial.println("Dir removed");
  } else {
    Serial.println("rmdir failed");
  }
}

void readFile(fs::FS &fs, const char * path) {
  Serial.printf("Reading file: %s\n", path);

  File file = fs.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }

  Serial.print("Read from file: ");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if (file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2) {
  Serial.printf("Renaming file %s to %s\n", path1, path2);
  if (fs.rename(path1, path2)) {
    Serial.println("File renamed");
  } else {
    Serial.println("Rename failed");
  }
}

void deleteFile(fs::FS &fs, const char * path) {
  Serial.printf("Deleting file: %s\n", path);
  if (fs.remove(path)) {
    Serial.println("File deleted");
  } else {
    Serial.println("Delete failed");
  }
}

void testFileIO(fs::FS &fs, const char * path) {
  File file = fs.open(path);
  static uint8_t buf[512];
  size_t len = 0;
  uint32_t start = millis();
  uint32_t end = start;
  if (file) {
    len = file.size();
    size_t flen = len;
    start = millis();
    while (len) {
      size_t toRead = len;
      if (toRead > 512) {
        toRead = 512;
      }
      file.read(buf, toRead);
      len -= toRead;
    }
    end = millis() - start;
    Serial.printf("%u bytes read for %u ms\n", flen, end);
    file.close();
  } else {
    Serial.println("Failed to open file for reading");
  }


  file = fs.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }

  size_t i;
  start = millis();
  for (i = 0; i < 2048; i++) {
    file.write(buf, 512);
  }
  end = millis() - start;
  Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
  file.close();
}
