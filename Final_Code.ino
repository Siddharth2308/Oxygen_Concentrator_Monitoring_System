#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "Wire.h"
#include "SFM3X00.h"
#include "Adafruit_ADS1X15.h"
#include "DFRobot_WT61PC.h"
#include "LiquidCrystal_I2C.h"

#define INIT_RTC
// ads.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V     0.1875mV (default)
// ads.setGain(GAIN_ONE);        // 1x gain   +/- 4.096V     0.125mV
// ads.setGain(GAIN_TWO);        // 2x gain   +/- 2.048V     0.0625mV
// ads.setGain(GAIN_FOUR);       // 4x gain   +/- 1.024V     0.03125mV
// ads.setGain(GAIN_EIGHT);      // 8x gain   +/- 0.512V     0.015625mV
//  ads1.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V     0.0078125mV
//  ads2.setGain(GAIN_SIXTEEN);
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
#define buzzer 33

const char* ssid = "RTR_JOY202";
const char* password = "RTR@2021";

int16_t adc0, adc1, adc2, adc3 , adc20, adc21, adc22, adc23;
float temp , volts0, volts1, volts2, volts3 , volts20, volts21, volts22, volts23 , accX, accY, accZ, gyroX, gyroY, gyroZ, angX, angY, angZ , flow ;
char const * wdays[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
uint8_t hours, minutes, seconds, datee, month, yearr, day ;
String dataMessage;

float flowrate = 0.0f;
float oxy_raw = 0.0f, oxy_volt = 0.0f, oxy_data = 0.0f;
float batt_raw = 0.0f, batt_volt = 0.0f, batt_volt_scaled = 0.0f ;
float invertor_raw = 0.0f, invertor_volt = 0.0f, invertor_volt_scaled = 0.0f ;
float batt_curr_raw = 0.0f, batt_curr = 0.0f, batt_curr_scaled = 0.0f ;

unsigned long time_varA = 0;
unsigned int getTime_count = 0;
bool hour_count = false;

AsyncWebServer server(80);
SFM3X00 flowSensor( 0x40 );
Adafruit_ADS1115 ads1 ;
Adafruit_ADS1115 ads2 ;
DFRobot_WT61PC sensor(&Serial2);
LiquidCrystal_I2C lcd(0x27, 16, 4);

void setup() {
  // Serial port for debugging purposes
  Serial.begin(115200);
  Serial.print("Setting AP (Access Point)…");
  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  initSDCard();
  Wire.begin();
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  sensor.modifyFrequency(FREQUENCY_200HZ);
  sfm34_init();
  lcd.begin();
  lcd.backlight();
  lcd.print("Hope");

#ifdef INIT_RTC
  writeRegister(DS3231_HOURS, 12);
  writeRegister(DS3231_MINUTES, 0);
  writeRegister(DS3231_SECONDS, 0);

  writeRegister(DS3231_DAY, 5);
  writeRegister(DS3231_DATE, 16);
  writeRegister(DS3231_CEN_MONTH, 02);
  writeRegister(DS3231_DEC_YEAR, 22);
#endif

  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS 1.");
    while (1);
  }
  if (!ads2.begin(0x49)) {
    Serial.println("Failed to initialize ADS 2.");
    while (1);
  }
  ads1.setGain(GAIN_ONE);
  getDate();

  server.on("/oxy", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", sendOxyHttp().c_str());
  });

  server.on("/flow", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", sendFlowHttp().c_str());
  });

  server.on("/vDC", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", sendVDCHttp().c_str());
  });

  server.on("/ampDC", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", sendAmpDCHttp().c_str());
  });

  server.on("/vAC", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", sendVACHttp().c_str());
  });

  server.on("/ampAC", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send_P(200, "text/plain", sendAmpACHttp().c_str());
  });

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SD, "/index.html", "text/html");
  });

  server.on("/csv", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(SD, "/Data_Log.csv", "text/csv");
  });

  // Start server
  server.begin();
}

void loop() {
  if (millis() - time_varB > 10000){
    getTime();
    logSDCard();
  }
  
  if (millis() - last_refresh > 500) {
    flowrate = flowSensor.readFlow();
    
    if (sensor.available()){
      accX = sensor.Acc.X ; accY = sensor.Acc.Y ; accZ = sensor.Acc.Z ;
      gyroX = sensor.Gyro.X ; gyroY = sensor.Gyro.Y, gyroZ = sensor.Gyro.Z ;
      angX = sensor.Angle.X , angY = sensor.Angle.Y, angZ = sensor.Angle.Z ;
    }

    oxydata = getOxyData();
    batt_volt_scaled = getBatteryVoltage();
    batt_curr_scaled = getBatteryCurrent();
    invertor_volt_scaled = getInvertorVoltage();
    invertor_curr_scaled = getInvertorCurrent();
    
    lcd.print("Oxygen:"); lcd.print((int)oxy_data); lcd.print("%");
    lcd.setCursor(0, 1);
    lcd.print("Flowrate:"); lcd.print(flowrate);
    lcd.setCursor(-4, 2);
    lcd.print("Bat:"); lcd.print(batt_volt_scaled); lcd.print("V "); lcd.print(batt_curr_scaled); lcd.print("A");
    lcd.setCursor(-4, 3);
    lcd.print("Int:"); lcd.print(invertor_volt_scaled); lcd.print("V "); lcd.print(invertor_curr_scaled); lcd.print("A");
  }

  if (hour_count) getDate();
  yield();
}

float getOxyData() {
  ads2.setGain(GAIN_SIXTEEN);
  oxy_raw = ads2.readADC_SingleEnded(0);
  oxy_volt = ( oxy_raw * 0.256 ) / 32767;
  oxy_data = ( oxy_volt * 21 * 100 ) / 9.0;
  return oxy_data;
}

float getBatteryVoltage() {
  batt_raw = ads1.readADC_SingleEnded(2);
  batt_volt = ads1.computeVolts(batt_raw);
  batt_volt_scaled = 12.0f; // formula tak
  return batt_volt_scaled;
}

float getBatteryCurrent() {
  ads2.setGain(GAIN_ONE);
  batt_curr_raw = ads2.readADC_SingleEnded(1);
  batt_curr = ads1.computeVolts(batt_curr_raw);
  batt_curr_scaled = ( batt_curr * 21 * 100 ) / 9.0;
  return batt_curr_scaled;
}

float getInvertorVoltage() {
  invertor_raw = ads1.readADC_SingleEnded(1);
  invertor_volt = ads1.computeVolts(invertor_raw);
  invertor_volt_scaled = 12.0f;
  return invertor_volt_scaled;
}

float getInvertorCurrent() {
  invertor_curr_raw = ads1.readADC_SingleEnded(0);
  invertor_curr = ads1.computeVolts(invertor_curr_raw);
  invertor_curr_scaled = 12.0f;
  return invertor_curr_scaled;
}

String sendOxyHttp() {
  return (String)oxy_data;
}

String sendFlowHttp() {
  return (String)flowrate;
}

String sendVDCHttp() {
  return (String)batt_volt_scaled;
}

String sendAmpDCHttp() {
  return (String)batt_curr_scaled;
}

String sendVACHttp() {
  return (String)invertor_volt_scaled;
}

String sendAmpACHttp() {
  return (String)invertor_curr_scaled;
}

void getDate() {
  day     = readRegister(DS3231_DAY)     ;
  hours   = readRegister(DS3231_HOURS)   ;
  minutes = readRegister(DS3231_MINUTES) ;
  seconds = readRegister(DS3231_SECONDS) ;
  datee   = readRegister(DS3231_DATE)    ;
  month   = readRegister(DS3231_CEN_MONTH);
  yearr   = readRegister(DS3231_DEC_YEAR) ;
  temp    = ( ( readRegister(DS3231_TEMP_MSB) << 8 | readRegister(DS3231_TEMP_LSB) ) >> 6 ) / 4.0f ;
  hour_count = false;
}

void getTime() {
  hours   = readRegister(DS3231_HOURS)   ;
  minutes = readRegister(DS3231_MINUTES) ;
  seconds = readRegister(DS3231_SECONDS) ;
  yearr   = readRegister(DS3231_DEC_YEAR) ;
  temp    = ( ( readRegister(DS3231_TEMP_MSB) << 8 | readRegister(DS3231_TEMP_LSB) ) >> 6 ) / 4.0f ;
  getTime_count++;
  if (getTime_count > 24) {
    hour_count = true;
    getTime_count = 0;
  }
}

//----------------------------------------RTC FUNCTIONS---------------------------------------

uint8_t toBcd(uint8_t num)
{
  uint8_t bcd = ((num / 10) << 4) + (num % 10);
  return bcd;
}

uint8_t fromBcd(uint8_t bcd) {
  uint8_t num = (10 * ((bcd & 0xf0) >> 4)) + (bcd & 0x0f);
  return num;
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(DS3231_ADDR, 1);
  return fromBcd(Wire.read());
}

void writeRegister(uint8_t reg, uint8_t data) {
  Wire.beginTransmission(DS3231_ADDR);
  Wire.write(reg);
  Wire.write(toBcd(data));
  Wire.endTransmission();
}


//----------------------------------------SD FUNCTIONS---------------------------------------
void logSDCard() {

  dataMessage =  (String)datee + ":"  + (String)month + ":" + (String)yearr + ","
                 (String)hours + ":"  + (String)minutes + ":" + (String)seconds + ","
                 (String)flowrate  + "," + (String)oxy_data + ","  + (String)batt_volt_scaled
                 ","  + (String)batt_curr_scaled + ","  + (String)invertor_volt_scaled + ","  + (String)invertor_curr_scaled +
                 ","  + (String)accX + ","  + (String)accY ","  + (String)accZ + "\n" ;

  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/Data_Log.csv", dataMessage.c_str());
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

void initSDCard() {
  if (!SD.begin()) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

void sfm34_init() {
  flowSensor.begin();
  Serial.print("sensor serial number: ");
  Serial.println(flowSensor.serialNumber, HEX);
  Serial.print("sensor article number: ");
  Serial.println(flowSensor.articleNumber, HEX);
  Serial.println();
  Serial.print("read scale factor: ");
  Serial.println(flowSensor.flowScale);
  Serial.print("read flow offset: ");
  Serial.println(flowSensor.flowOffset);
}

void lcd_init() {
  lcd.init();
  lcd.backlight();
  lcd.print("Initialization");
  lcd.setCursor(-4, 1);
  lcd.print("Completed");
  delay(1000);
  lcd.clear();
}

float map_1(float x, float in_min, float in_max, float out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
