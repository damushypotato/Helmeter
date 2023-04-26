#include <Arduino.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>

int GPSBaud = 9600;

TinyGPSPlus gps;

SoftwareSerial gpsSerial(2, 3);

LiquidCrystal_I2C lcd(0x27, 16, 2);

int speedInterval = 200;
unsigned long speedDelta = 0;

int peaksInterval = 3000;
unsigned long peaksDelta = 0;

int minSpeed = 9999;
int maxSpeed = -1;

unsigned long currentMillis = 0;

const unsigned char UBLOX_INIT[] PROGMEM = {
    //  0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12, //(10Hz)
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A, //(5Hz)
    //   0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39 //(1Hz)
};

void setup()
{
    gpsSerial.begin(GPSBaud);

    for (unsigned int i = 0; i < sizeof(UBLOX_INIT); i++)
    {
        gpsSerial.write(pgm_read_byte(UBLOX_INIT + i));
    }

    lcd.init();
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print("GPS Speedometer");
    lcd.setCursor(0, 1);
    lcd.print("by Arjun K");

    delay(1000);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Loading");
    lcd.setCursor(0, 1);
    lcd.print("WWOT");
    lcd.setCursor(12, 1);
    lcd.print("V1.2");

    delay(500);

    lcd.setCursor(7, 0);
    lcd.print(".");

    delay(500);

    lcd.setCursor(7, 0);
    lcd.print("..");

    delay(500);

    lcd.setCursor(7, 0);
    lcd.print("...");

    delay(500);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Spd");
    lcd.setCursor(13, 0);
    lcd.print("kph");
}

int getSpeed()
{
    if (gps.speed.isValid())
    {
        return round(gps.speed.kmph());
    }
    else
    {
        return 9999;
    }
}

void loop()
{
    if (gpsSerial.available() > 0)
    {
        if (gps.encode(gpsSerial.read()))
        {
            if (currentMillis - speedDelta >= speedInterval)
            {
                int speed = getSpeed();

                if (speed > maxSpeed)
                {
                    maxSpeed = speed;
                }

                if (speed < minSpeed)
                {
                    minSpeed = speed;
                }

                lcd.setCursor(7, 0);
                lcd.print("    ");
                lcd.setCursor(7, 0);
                lcd.print(speed);

                lcd.setCursor(7, 1);
                lcd.print((peaksInterval - (currentMillis - peaksDelta)) / 1000 + 1);

                speedDelta = currentMillis;
            }
        }
    }

    if (currentMillis - peaksDelta >= peaksInterval)
    {
        lcd.setCursor(0, 1);
        lcd.print("                ");
        if (minSpeed != 9999 || maxSpeed != 9999)
        {
            lcd.setCursor(0, 1);
            lcd.print(minSpeed);
            int length = ceil(maxSpeed / 10);
            lcd.setCursor(15 - length, 1);
            lcd.print(maxSpeed);
        }
        peaksDelta = currentMillis;

        minSpeed = 9999;
        maxSpeed = -1;
    }

    currentMillis = millis();
}
