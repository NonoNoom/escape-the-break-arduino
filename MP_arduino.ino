#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_TSL2591.h"
#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BTSerial(2, 3); // RX, TX

#define lasersensor 4
#define bigButton 5
#define resetButton 7
Servo servo;
int servoPin = 6;

Adafruit_7segment timer = Adafruit_7segment();
Adafruit_TSL2591 tsl = Adafruit_TSL2591(2591); // pass in a number for the sensor identifier (for your use later)

unsigned long previousSecondMillis = 0UL;
long oneSecond = 1000UL; // milliseconds per second

#define startMinute 20  //  Modify these defines to
#define startSecond 0 // change the timer interval
int minutes = startMinute;
int seconds = startSecond;

bool start = false;
bool game_over = false;
int resetButton_state;

void displaySensorDetails(void)
{
  sensor_t sensor;
  tsl.getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(sensor.max_value); Serial.println(F(" lux"));
  Serial.print  (F("Min Value:    ")); Serial.print(sensor.min_value); Serial.println(F(" lux"));
  Serial.print  (F("Resolution:   ")); Serial.print(sensor.resolution, 4); Serial.println(F(" lux"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  delay(500);
}

void configureSensor(void)
{
  tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);

  /* Display the gain and integration time for reference sake */  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Gain:         ")); Serial.println(F("428x (High)"));
  Serial.print  (F("Timing:       ")); Serial.print((tsl.getTiming() + 1) * 100, DEC); 
  Serial.println(F(" ms"));
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
}


void setup() {
  BTSerial.begin(9600); // HC-05 default baud rate
  pinMode(lasersensor, INPUT);
  pinMode(bigButton, INPUT);
  pinMode(resetButton, INPUT_PULLUP);
  servo.attach(servoPin);
  servo.write(0);

#ifndef __AVR_ATtiny85__
  Serial.println("7 Segment Backpack Test");
#endif
  timer.begin(0x70);
  
  Serial.begin(9600); // Initialize serial communication for debugging
  displaySensorDetails();
  configureSensor();
}

void advancedRead(void)
{
  // More advanced data read example. Read 32 bits with top 16 bits IR, bottom 16 bits full spectrum
  // That way you can do whatever math and comparisons you want!
  uint32_t lum = tsl.getFullLuminosity();
  uint16_t ir, full;
  ir = lum >> 16;
  full = lum & 0xFFFF;
  Serial.print(F("[ ")); Serial.print(millis()); Serial.print(F(" ms ] "));
  Serial.print(F("Lux: ")); Serial.println(tsl.calculateLux(full, ir), 6);
  if (tsl.calculateLux(full, ir) > 5)
  {
    Serial.print("GAME ENDS");
    game_over = true;
    start = false;
  }
  else {
    Serial.print("Game busy...");
  }
}

void reset(void)
{
  resetButton_state = digitalRead(resetButton);

  if (resetButton_state == LOW)
  {
    Serial.print("Reset game");
    servo.write(0);
    timer.print("");
    timer.writeDisplay();
    start = false;
    game_over = true;
    BTSerial.write('2');
  }
}

void laserSensor(void)
{
  bool lasersensorValue = digitalRead(lasersensor);
  
  if (lasersensorValue == LOW) {
    Serial.println("\nSensor touched - ");
    servo.write(90);
    BTSerial.write('0');
  }
}

void timerLoop(void)
{
  if ((millis() - previousSecondMillis >= oneSecond) && (!game_over)) {
    timer.writeDigitNum(0, (minutes / 10));
    timer.writeDigitNum(1, (minutes % 10));
    timer.writeDigitNum(3, (seconds / 10));
    timer.writeDigitNum(4, (seconds % 10));
    timer.writeDigitRaw(2, 0x02);
    timer.writeDisplay();

    if (seconds-- == 0) {
      if (minutes == 0) {
        Serial.println("GAME OVER");
        timer.blinkRate(1);
        servo.write(0);
        start = false;
      } else {
        minutes -= 1;
        seconds = 59;
      }
    }
    previousSecondMillis += oneSecond;
  }
}

void arcadeButton(void)
{
  int buttonState = digitalRead(bigButton);
  if (buttonState == HIGH) {
    Serial.print("Button pressed! GAME START! - ");
    start = true;
    game_over = false;
    minutes = startMinute;
    seconds = startSecond;
    previousSecondMillis = millis();
    timer.blinkRate(0);
    BTSerial.write('1'); // Send signal to slave module
  } else {
    Serial.println("Waiting for button press...");
  }
}

void loop(void) 
{
  reset();
  
  if (start == false) 
  {
    arcadeButton();
  } 
  else 
  {
    laserSensor();
    advancedRead();
    timerLoop();
  }
}
