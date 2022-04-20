#include <math.h>
#include <SoftwareSerial.h>

#define RESISTOR_VALUE 68000
#define THERMISTOR_PIN A0
#define THERMISTOR_ENABLE 0
#define THERMISTOR_CALIBRATION -3
#define NUM_SAMPLES 10
#define TRANSMISSION_MASK 1 << 8

SoftwareSerial bleDevice(10,11);

char appData;
String inData = "";
long prevTime;
uint8_t transmission[2];

void setup()
{
    pinMode(THERMISTOR_ENABLE, OUTPUT);
    pinMode(THERMISTOR_PIN, INPUT);
    
    Serial.begin(9600);
    bleDevice.begin(9600);

    prevTime = millis();
}

void loop()
{
    bleDevice.listen();
    
    while (bleDevice.available() > 0)
    {
        appData = bleDevice.read();
        inData = String(appData);
        Serial.write(appData);
    }

    if (Serial.available())
    {
        delay(10);
        bleDevice.write(Serial.read());
    }

    if (millis() - prevTime > 2000)
    {
        prevTime = millis();
        Serial.println("TEMP_CALC");
        double temp = getTemp();
        Serial.println(temp);
        floatToArray(temp);
        bleDevice.write(transmission, 2);
        delay(500);
    }
}

double getTemp()
{
    float temp { 0 }, resistance { 0 };

    for (int i = 0; i < NUM_SAMPLES; i++)
    {
        digitalWrite(THERMISTOR_ENABLE, HIGH);
        resistance += analogRead(THERMISTOR_PIN);
        digitalWrite(THERMISTOR_ENABLE, LOW);
        delay(100);
    }

    resistance /= NUM_SAMPLES;
    
    resistance = (RESISTOR_VALUE * resistance)/((1023 - resistance) * 1000);

    if (resistance > 32.747)
        temp = 59.1 - 17.1 * log(resistance);
    else if (resistance > 5.324)
        temp = 76.1 - 22 * log(resistance);
    else
        temp = 99.8 - 36 * log(resistance);

    temp += THERMISTOR_CALIBRATION;

    return temp;
}

void floatToArray(double &temperature)
{
    int16_t value = temperature * 100;
    
    transmission[0] = value >> 8;
    transmission[1] = (uint8_t)value;
}