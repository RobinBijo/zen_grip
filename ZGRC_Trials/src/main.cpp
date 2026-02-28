#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include "esp_wifi.h"

// -------- TB6612 --------
#define PWMA 25
#define PWMB 14
#define AIN1 26
#define AIN2 27
#define BIN1 12
#define BIN2 13
#define STBY 33

// -------- Stepper A4988 --------
#define DIR_PIN 19
#define STEP_PIN 18
#define EN_PIN 5

// -------- Servos MG996R --------
#define SERVO1_PIN 21
#define SERVO2_PIN 22
#define SERVO3_PIN 23

Servo servo1;
Servo servo2;
Servo servo3;

// Data format from master
// [0] robot cmd
// [1] stepper direction (1 or -1 or 0)
// [2] servo1 angle 0–50
// [3] servo2 angle 0 or 90
// [4] gripper 0 close 1 open

int received[5] = {0,0,0,0,0};
int lastProcessed[5] = {999,999,999,999,999};

unsigned long lastPacketTime = 0;
const unsigned long timeoutMs = 500;

const int stepsPer50Deg = 200;   // Tune after testing

// ---------------- Robot Drive ----------------
void moveRobot(int cmd)
{
  digitalWrite(STBY, HIGH);

  // Full PWM speed
  ledcWrite(0, 255);
  ledcWrite(1, 255);

  if (cmd == 1) // Forward
  {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  }
  else if (cmd == 2) // Backward
  {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  }
  else if (cmd == 3) // Left
  {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  }
  else if (cmd == 4) // Right
  {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH);
  }
  else // Stop
  {
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
    ledcWrite(0, 0);
    ledcWrite(1, 0);
  }
}

// ---------------- Stepper Increment ----------------
void stepperMove(int direction)
{
  if (direction == 0) return;

  digitalWrite(EN_PIN, LOW);  // Enable driver
  digitalWrite(DIR_PIN, direction > 0 ? HIGH : LOW);

  for (int i = 0; i < stepsPer50Deg; i++)
  {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(700);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(700);
  }

  digitalWrite(EN_PIN, HIGH); // Disable after move
}

// ---------------- ESPNOW Callback ----------------
void OnDataRecv(const uint8_t *mac, const uint8_t *data, int len)
{
  memcpy(received, data, sizeof(received));
  lastPacketTime = millis();
}

// ---------------- Setup ----------------
void setup()
{
  Serial.begin(115200);

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Stepper pins
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH); // disabled initially

  // PWM setup for TB6612
  ledcSetup(0, 1000, 8);
  ledcAttachPin(PWMA, 0);

  ledcSetup(1, 1000, 8);
  ledcAttachPin(PWMB, 1);

  // MG996R attach with proper pulse range
  servo1.attach(SERVO1_PIN, 500, 2500);
  servo2.attach(SERVO2_PIN, 500, 2500);
  servo3.attach(SERVO3_PIN, 500, 2500);

  // Default safe positions
  servo1.write(5);
  servo2.write(5);
  servo3.write(10);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(10, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK)
  {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
}

// ---------------- Loop ----------------
void loop()
{
  // Fail safe stop if signal lost
  if (millis() - lastPacketTime > timeoutMs)
  {
    moveRobot(0);
  }

  // Robot movement
  if (received[0] != lastProcessed[0])
  {
    moveRobot(received[0]);
    lastProcessed[0] = received[0];
  }

  // Stepper incremental
  if (received[1] != 0 && received[1] != lastProcessed[1])
  {
    stepperMove(received[1]);
    lastProcessed[1] = received[1];
  }
  if (received[1] == 0)
    lastProcessed[1] = 0;

  // Servo1 absolute 0–50
  if (received[2] != lastProcessed[2])
  {
    servo1.write(constrain(received[2], 5, 50));
    lastProcessed[2] = received[2];
  }

  // Servo2 0–90
  if (received[3] != lastProcessed[3])
  {
    servo2.write(constrain(received[3], 5, 85));
    lastProcessed[3] = received[3];
  }

  // Gripper
  if (received[4] != lastProcessed[4])
  {
    if (received[4] == 1)
      servo3.write(80);
    else
      servo3.write(10);

    lastProcessed[4] = received[4];
  }
}