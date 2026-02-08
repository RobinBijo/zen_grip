#include <Arduino.h>
//Simple Blink Example for ESP32 using PlatformIO
// put function declarations here:
#define LED_PIN 2

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                  // wait for a second
  digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                  // wait for a second

}

// put function definitions here:
