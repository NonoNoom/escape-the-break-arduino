#include <SoftwareSerial.h>
SoftwareSerial BTSerial(2, 3); // RX, TX
#define laser 4

int state = 20;

void setup() {
  // put your setup code here, to run once:
  BTSerial.begin(9600); // HC-05 default baud rate
  pinMode(laser, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (BTSerial.available() > 0) {
    Serial.println("Bluetooth connected");
    state = BTSerial.read(); // Read data from master module
  } else {
    Serial.println("Waiting for bluetooth connection...");
  }
  
  if (state == '1') {
    digitalWrite(laser, HIGH); // Turn on laser
    Serial.println("Laser on");
  } else if (state == '0') {
    delay(10000);
    digitalWrite(laser, LOW); // Turn off laser
    Serial.println("Laser off");
  } else {
    digitalWrite(laser, LOW); // Turn off laser
    Serial.println("Laser off");
  }
}
