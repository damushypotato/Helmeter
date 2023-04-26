#include <Arduino.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

int RXPin = 2;
int TXPin = 3;

int GPSBaud = 9600;

TinyGPSPlus gps;

SoftwareSerial gpsSerial(RXPin, TXPin);

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
    gpsSerial.begin(GPSBaud);

    lcd.init();
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print("GPS Speedometer");
    lcd.setCursor(0, 1);
    lcd.print("by Arjun K");

    delay(3000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Speed: ");
    lcd.setCursor(12, 0);
    lcd.print("km/h");
}

int getSpeed()
{
    if (gps.speed.isValid())
    {
        return round(gps.speed.kmph());
    }
    else
    {
        return 0;
    }
}

void loop()
{
    while (gpsSerial.available() > 0)
    {
        if (gps.encode(gpsSerial.read()))
        {
            lcd.setCursor(7, 0);
            lcd.print("     ");
            lcd.setCursor(7, 0);
            lcd.print(getSpeed());

            delay(500);
        }
    }

    if (millis() > 10000 && gps.charsProcessed() < 10)
    {
        Serial.println("No GPS detected");
        while (true)
            ;
    }
}
