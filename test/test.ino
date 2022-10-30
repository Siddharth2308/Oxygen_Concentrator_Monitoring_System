#include <Wire.h>
#include "notLCD.h"
#include "DFRobot_WT61PC.h"

#define RXD2 16
#define TXD2 17

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 4);
DFRobot_WT61PC sensor(&Serial2);

int16_t adc0, adc1, adc2, adc3 , adc20, adc21, adc22, adc23;
float temp , volts0, volts1, volts2, volts3 , volts20, volts21, volts22, volts23 , accX, accY, accZ, gyroX, gyroY, gyroZ, angX, angY, angZ , flow ;
char const * wdays[] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
uint8_t hours, minutes, seconds, datee, month, yearr, day ;
String dataMessage;


void setup()
{
  // initialize the LCD
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  sensor.modifyFrequency(FREQUENCY_200HZ);
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Oxygen:"); lcd.print("90%");
  lcd.setCursor(0, 1);
  lcd.print("Flowrate:"); lcd.print(0.00); lcd.print("slm");
  lcd.setCursor(-4, 2);
  lcd.print("Bat:"); lcd.print("18.11V 2.02A");
  lcd.setCursor(-4, 3);
  lcd.print("Int:"); lcd.print("18.11V 2.02A");
}

void loop()
{
  if (sensor.available()) {
    accX = sensor.Acc.X ; accY = sensor.Acc.Y ; accZ = sensor.Acc.Z ;
    gyroX = sensor.Gyro.X ; gyroY = sensor.Gyro.Y, gyroZ = sensor.Gyro.Z ;
    angX = sensor.Angle.X , angY = sensor.Angle.Y, angZ = sensor.Angle.Z ;
  }
  Serial.print("Angle\t"); Serial.print( angX); Serial.print("\t"); Serial.print( angY); Serial.print("\t"); Serial.println( angZ); //angle information of X, Y, Z
  delay(500);
}
