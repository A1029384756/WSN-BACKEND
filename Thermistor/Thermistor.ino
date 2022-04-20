// ECE 2804 - Wireless Sensor Node
// Group 6 - Dillon Domnick & Hayden Gray
// Version 4/13/2022

#include <math.h>
#include <SoftwareSerial.h>
#include <math.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

// Minimum duty% = 44.4% = .444 * 255 ~ 113
// Maximum duty% = 63.6% = .636 * 255 ~ 162
// Write range = 162 - 113 = 49
const int WRITE_MIN = 113;
const int WRITE_MAX = 162;
const int WRITE_RANGE = WRITE_MAX - WRITE_MIN;

// Temperature collection globals
#define RESISTOR_VALUE 68000
#define THERMISTOR_PIN A0
#define THERMISTOR_ENABLE 0
#define THERMISTOR_CALIBRATION -3
#define NUM_SAMPLES 10
#define TRANSMISSION_MASK 1 << 8

SoftwareSerial bleDevice(10, 11);
char appData;
String inData = "";
long prevTime;
uint8_t transmission[2];
int f_wdt;

void setup() {
  // Clear Prescale value before loop
  pinMode(6, 128);
  TCCR0B = TCCR0B & B11111000 | B1;

  pinMode(THERMISTOR_ENABLE, OUTPUT);
  pinMode(THERMISTOR_PIN, INPUT);

  Serial.begin(9600);
  bleDevice.begin(9600);

  prevTime = millis();
}


// Watchdog Interrupt handler
ISR(WDT_vect) {
  if (f_wdt == 0) {
    f_wdt = 1;
  }
}

void sleep() {
  // Power down logic
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_mode();

  // Power on logic
  sleep_disable();
  power_all_enable();
}

void sleepTimer(const byte time_b) {
  MCUSR &= ~(1 << WDRF); // Clear reset flag
  WDTCSR |= (1 << WDCE) | (1 << WDE);

  wdt_reset();
  WDTCSR = 0b1000 | time_b;
  WDTCSR |= _BV(WDIE);
  sleep();
}

void loop() {
  //if (f_wdt == 0) return;
  /*******  Bluetooth communication *******/

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

  prevTime = millis();
  Serial.println("TEMP_CALC");
  double temp = getTemp();
  Serial.println(temp);
  floatToArray(temp);
  bleDevice.write(transmission, 2);
  delay(500);

  /*******  PWM and voltage control *******/

  // Read input voltage
  float Vin = analogRead(A0); // between 0 - 1023 (0 - 5V)

  // This project only support 4-5V or 819-1023 (204) aRead values
  Vin = (Vin < 819) ? 1 : Vin - 819;
  int op = (Vin / 204) * WRITE_RANGE;
  int wVal = WRITE_MAX - op;

  // Write PWM signal
  analogWrite(6, wVal);
  TCCR0B = TCCR0B & B11111000 | B1; // Reset prescale value

  // Reset WDT flag
  f_wdt = 0;

  // Sleep for 30 seconds
  sleepTimer(0b100001);
  sleepTimer(0b100001);
  sleepTimer(0b100001);
  sleepTimer(0b100000);
  sleepTimer(0b000111);
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

  resistance = (RESISTOR_VALUE * resistance) / ((1023 - resistance) * 1000);

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
  uint16_t value = temperature * 100;

  transmission[0] = value >> 8;
  transmission[1] = (uint8_t)value;
}